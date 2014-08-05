#include "ArmPlanningNode.h"
#include <cstdio>
#include <leatherman/utils.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "HDTRobotModel.h"

namespace hdt
{

JointInterpolationPathGenerator::JointInterpolationPathGenerator() :
    collision_checker_(),
    min_limits_(),
    max_limits_(),
    continuous_()
{
}

bool JointInterpolationPathGenerator::initialize(
    const std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace>& collision_checker,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits,
    const std::vector<bool>& continuous)
{
    if (!collision_checker || min_limits.size() != max_limits.size() || min_limits.size() != continuous.size()) {
        return false;
    }

    collision_checker_ = collision_checker;
    min_limits_ = min_limits;
    max_limits_ = max_limits;
    continuous_ = continuous;
    return true;
}

bool JointInterpolationPathGenerator::generate_path(
    const trajectory_msgs::JointTrajectoryPoint& start,
    const trajectory_msgs::JointTrajectoryPoint& end,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& path_out,
    int& cost_out) const
{
    std::vector<double> inc(min_limits_.size(), sbpl::utils::ToRadians(1.0));

    std::vector<std::vector<double>> path;
    if (!sbpl::interp::InterpolatePath(start.positions, end.positions, min_limits_, max_limits_, inc, continuous_, path)) {
        ROS_ERROR("Failed to shortcut between %s and %s with joint interpolation", to_string(start.positions).c_str(), to_string(end.positions).c_str());
        return false;
    }
    else {
        for (std::vector<double>& point : path) {
            double dist;
            if (collision_checker_->isStateValid(point, false, false, dist)) {
                trajectory_msgs::JointTrajectoryPoint jt_point;
                jt_point.positions = std::move(point);
                path_out.push_back(std::move(jt_point));
            }
            else {
                return false;
            }
        }
        cost_out = 0; ///< hacking my way to victory
        return true;
    }
}

ArmPlanningNode::ArmPlanningNode() :
    nh_(),
    ph_("~"),
    marker_array_pub_(),
    joint_trajectory_pub_(),
    joint_states_sub_(),
    move_command_server_(),
    action_client_("arm_controller/joint_trajectory_action"),
    action_set_filename_(),
    group_name_(),
    kinematics_frame_(),
    planning_frame_(),
    planning_link_(),
    chain_tip_link_(),
    object_filename_(),
    urdf_(),
    planning_joints_(),
    robot_model_(),
    distance_field_(),
    grid_(),
    collision_checker_(),
    sbpl_action_set_(),
    planner_(),
    last_joint_state_(),
    use_action_server_(false),
    urdf_model_(),
    statistic_names_({
        "initial solution planning time",
        "initial epsilon",
        "initial solution expansions",
        "final epsilon planning time",
        "final epsilon",
        "solution epsilon",
        "expansions",
        "solution cost" })
{
}

bool ArmPlanningNode::init()
{
    if (!init_robot()) {
        ROS_ERROR("Failed to initialize robot model");
        return false;
    }

    if (!init_collision_model()) {
        ROS_ERROR("Failed to initialize collision model");
        return false;
    }

    if (!init_sbpl()) {
        ROS_ERROR("Failed to initialize SBPL");
        return false;
    }

    ph_.param("use_action_server", use_action_server_, true);
    if (use_action_server_) {
        ROS_INFO("Using action server 'arm_controller/joint_trajectory_action'");
    }
    else {
        ROS_INFO("Publishing joint trajectories to command");
    }

    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &ArmPlanningNode::joint_states_callback, this);

    move_command_server_.reset(new MoveArmActionServer(
            "move_arm_command", boost::bind(&ArmPlanningNode::move_arm, this, _1), true));
    if (!move_command_server_) {
        ROS_ERROR("Failed to instantiate Move Arm Action Server");
        return false;
    }

    move_command_server_->start();

    ROS_INFO("Waiting for action client \"arm_controller/joint_trajectory_action\"");
    action_client_.waitForServer();

    return true;
}

int ArmPlanningNode::run()
{
    ROS_INFO("Spinning...");
    ros::spin();
    ROS_INFO("Done spinning");
    return 0;
}

bool ArmPlanningNode::init_robot()
{
    group_name_ = "hdt_arm"; // TODO: figure out what this is for; it only gets passed to the collision checker

    if (!nh_.hasParam("robot_description")) {
        ROS_ERROR("Missing parameter \"robot_description\"");
        return false;
    }

    nh_.getParam("robot_description", urdf_);

    urdf_model_ = urdf::parseURDF(urdf_);
    if (!urdf_model_) {
        ROS_ERROR("Failed to parse URDF");
        return false;
    }

    boost::shared_ptr<const urdf::Joint> first_joint = urdf_model_->getJoint("arm_1_shoulder_twist");
    if (!first_joint) {
        ROS_ERROR("Failed to find joint 'arm_1_shoulder_twist'");
        return false;
    }

    kinematics_frame_ = first_joint->parent_link_name; // TODO: figure out what this is for; it doesn't appear to be used anywhere
    planning_frame_ = first_joint->parent_link_name;

    planning_link_ = "arm_7_gripper_lift_link";
    chain_tip_link_ = "arm_7_gripper_lift_link"; // TODO: figure out what this is for; it also doesn't appear to be used anywhere

    HDTRobotModel* hdt_robot_model = new HDTRobotModel;
    if (!hdt_robot_model || !hdt_robot_model->init(urdf_)) {
        ROS_ERROR("Failed to initialize HDT Robot Model");
        return false;
    }

    robot_model_.reset(hdt_robot_model);
    if (!robot_model_) {
        ROS_ERROR("Failed to instantiate KDL Robot Model");
    }

    planning_joints_ = { "arm_1_shoulder_twist",
                         "arm_2_shoulder_lift",
                         "arm_3_elbow_twist",
                         "arm_4_elbow_lift",
                         "arm_5_wrist_twist",
                         "arm_6_wrist_lift",
                         "arm_7_gripper_lift" };

    manipulator_joint_names_ = planning_joints_;

    min_limits_.clear();
    max_limits_.clear();
    continuous_.clear();
    min_limits_.reserve(planning_joints_.size());
    max_limits_.reserve(planning_joints_.size());
    continuous_.reserve(planning_joints_.size());
    for (const std::string& planning_joint : planning_joints_) {
        boost::shared_ptr<const urdf::Joint> joint = urdf_model_->getJoint(planning_joint);
        if (!joint->limits) {
            ROS_WARN("No limits found for joint '%s'. Using defaults of [-pi, pi]", planning_joint.c_str());
            min_limits_.push_back(-M_PI);
            max_limits_.push_back(M_PI);
            continuous_.push_back(true);
        }
        else {
            min_limits_.push_back(joint->limits->lower);
            max_limits_.push_back(joint->limits->upper);
            continuous_.push_back(joint->type == urdf::Joint::CONTINUOUS ? true : false);
        }
    }

    if (!robot_model_->init(urdf_, planning_joints_)) {
        ROS_ERROR("Failed to initialize KDL Robot Model");
        return false;
    }

    robot_model_->setPlanningLink(planning_link_);

    return true;
}

bool ArmPlanningNode::init_collision_model()
{
    const double size_x = 3.0;
    const double size_y = 3.0;
    const double size_z = 3.0;
    const double cell_res_m = 0.02;
    const double origin_x = -0.75;
    const double origin_y = -1.25;
    const double origin_z = -1.0;
    const double max_dist_m = 0.2;

    distance_field_.reset(new distance_field::PropagationDistanceField(
            size_x, size_y, size_z,
            cell_res_m,
            origin_x, origin_y, origin_z,
            max_dist_m));

    if (!distance_field_) {
        ROS_ERROR("Failed to instantiate Propagation Distance Field");
        return false;
    }
    distance_field_->reset();

    grid_.reset(new sbpl_arm_planner::OccupancyGrid(distance_field_.get()));
    if (!grid_) {
        ROS_ERROR("Failed to instantiate Occupancy Grid");
        return false;
    }
    grid_->setReferenceFrame(planning_frame_);

    collision_checker_.reset(new sbpl_arm_planner::SBPLCollisionSpace(grid_.get()));
    if (!collision_checker_) {
        ROS_ERROR("Failed to instantiate SBPL Collision Space");
        return false;
    }
    if (!collision_checker_->init(group_name_) || !collision_checker_->setPlanningJoints(planning_joints_)) {
        ROS_ERROR("Failed to initialize SBPL Collision Checker");
        return false;
    }

    return true;
}

bool ArmPlanningNode::init_sbpl()
{
    if (!ph_.hasParam("action_set_filename")) {
        ROS_ERROR("Missing parameter \"action_set_filename\"");
        return false;
    }

    ph_.getParam("action_set_filename", action_set_filename_);

    sbpl_action_set_.reset(new sbpl_arm_planner::ActionSet(action_set_filename_));
    if (!sbpl_action_set_) {
        ROS_ERROR("Failed to instantiate Action Set");
        return false;
    }

    planner_.reset(new sbpl_arm_planner::SBPLArmPlannerInterface(
            robot_model_.get(), collision_checker_.get(), sbpl_action_set_.get(), distance_field_.get()));

    if (!planner_) {
        ROS_ERROR("Failed to instantiate SBPL Arm Planner Interface");
        return false;
    }

    if (!planner_->init()) {
        ROS_ERROR("Failed to initialize SBPL Arm Planner Interface");
        return false;
    }

    return true;
}

void ArmPlanningNode::move_arm(const hdt::MoveArmCommandGoal::ConstPtr& request)
{
    move_command_server_->acceptNewGoal();

    tf::Quaternion goal_quat(request->goal_pose.orientation.x, request->goal_pose.orientation.y, request->goal_pose.orientation.z, request->goal_pose.orientation.w);
    tf::Matrix3x3 goal_rotation_matrix(goal_quat);

    double goal_roll, goal_pitch, goal_yaw;
    goal_rotation_matrix.getEulerYPR(goal_yaw, goal_pitch, goal_roll);
    const std::vector<double> goal = {
            request->goal_pose.position.x, request->goal_pose.position.y, request->goal_pose.position.z,
            goal_roll, goal_pitch, goal_yaw };

    // collision objects
    moveit_msgs::PlanningScenePtr scene(new moveit_msgs::PlanningScene);

    ph_.param<std::string>("object_filename", object_filename_, "");
    if (!object_filename_.empty()) {
        scene->world.collision_objects = get_collision_objects(object_filename_, planning_frame_);
    }

    // create goal
    moveit_msgs::GetMotionPlan::Request req;
    moveit_msgs::GetMotionPlan::Response res;
    scene->world.collision_map.header.frame_id = planning_frame_;

    // fill goal state
    req.motion_plan_request.goal_constraints.resize(1);
    fill_constraint(goal, planning_frame_, req.motion_plan_request.goal_constraints[0]);
    ROS_WARN("Created a goal in the '%s' frame", req.motion_plan_request.goal_constraints.front().position_constraints[0].header.frame_id.c_str());
    req.motion_plan_request.allowed_planning_time = 2.0;

    // fill start state
    if (!get_initial_configuration(ph_, scene->robot_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        move_command_server_->setAborted();
        return;
    }

    scene->robot_state.joint_state.header.frame_id = planning_frame_;
    req.motion_plan_request.start_state = scene->robot_state;

    // set planning scene
    //cc->setPlanningScene(*scene);

    // plan
    ROS_INFO("Calling solve...");
    bool plan_result = planner_->solve(scene, req, res);
    if (!plan_result) {
        ROS_ERROR("Failed to plan.");
    }
    else {
        ROS_INFO("Planning succeeded");
    }

    // print statistics
    std::map<std::string, double> planning_stats = planner_->getPlannerStats();

    ROS_INFO("Planning statistics");
    for (const auto& statistic : statistic_names_) {
        auto it = planning_stats.find(statistic);
        if (it != planning_stats.end()) {
            ROS_INFO("    %s: %0.3f", statistic.c_str(), it->second);
        }
        else {
            ROS_WARN("Did not find planning statistic \"%s\"", statistic.c_str());
        }
    }

    // visualizations
    marker_array_pub_.publish(collision_checker_->getVisualization("bounds"));
    marker_array_pub_.publish(collision_checker_->getVisualization("distance_field"));
    marker_array_pub_.publish(collision_checker_->getVisualization("collision_objects"));
    marker_array_pub_.publish(planner_->getVisualization("goal"));

    // visualize, filter, and publish plan
    if (plan_result) {
        marker_array_pub_.publish(planner_->getCollisionModelTrajectoryMarker());

        trajectory_msgs::JointTrajectory& joint_trajectory = res.motion_plan_response.trajectory.joint_trajectory;
        ROS_INFO("Original joint path (%zd points):", joint_trajectory.points.size());
        for (int i = 0; i < (int)joint_trajectory.points.size(); ++i) {
            const trajectory_msgs::JointTrajectoryPoint& joint_state = joint_trajectory.points[i];
            ROS_INFO("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
        }

        apply_shortcutting(res);

        ROS_INFO("Shortcut trajectory (%zd points):", joint_trajectory.points.size());
        for (int i = 0; i < (int)joint_trajectory.points.size(); ++i) {
            const trajectory_msgs::JointTrajectoryPoint& joint_state = joint_trajectory.points[i];
            ROS_INFO("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
        }

        bool interp_res = add_interpolation_to_plan(res);
        if (!interp_res) {
            ROS_ERROR("Failed to interpolate joint trajectory");
            move_command_server_->setAborted();
            return;
        }

        publish_trajectory(res.motion_plan_response.trajectory);
    }

    move_command_server_->setSucceeded();
}

void ArmPlanningNode::fill_constraint(const std::vector<double>& pose,
                                      const std::string& frame_id,
                                      moveit_msgs::Constraints& goals)
{
    if(pose.size() < 6)
        return;

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

    //  goals.position_constraints[0].position.x = pose[0];
    //  goals.position_constraints[0].position.y = pose[1];
    //  goals.position_constraints[0].position.z = pose[2];

    leatherman::rpyToQuatMsg(pose[3], pose[4], pose[5], goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

moveit_msgs::CollisionObject ArmPlanningNode::get_collision_cube(const geometry_msgs::Pose& pose,
                                                                 const std::vector<double>& dims,
                                                                 const std::string& frame_id,
                                                                 const std::string& id)
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

std::vector<moveit_msgs::CollisionObject>
ArmPlanningNode::get_collision_cubes(const std::vector<std::vector<double>>& objects,
                                     const std::vector<std::string>& object_ids,
                                     const std::string& frame_id)
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(get_collision_cube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

bool ArmPlanningNode::get_initial_configuration(ros::NodeHandle& nh, moveit_msgs::RobotState& state)
{
    if (!last_joint_state_.header.stamp.isValid()) {
        ROS_ERROR("Haven't received a valid joint state yet");
        return false;
    }

    state.joint_state = last_joint_state_;

    geometry_msgs::Pose pose;
    state.multi_dof_joint_state.header.frame_id = "map";
    state.multi_dof_joint_state.joint_names.resize(1);
    state.multi_dof_joint_state.joint_transforms.resize(1);
    state.multi_dof_joint_state.joint_names[0] = "world_pose";

    // identity transform from map to root
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    leatherman::rpyToQuatMsg(0.0, 0.0, 0.0, pose.orientation);
    state.multi_dof_joint_state.joint_transforms[0].translation.x = pose.position.x;
    state.multi_dof_joint_state.joint_transforms[0].translation.y = pose.position.y;
    state.multi_dof_joint_state.joint_transforms[0].translation.z = pose.position.z;
    state.multi_dof_joint_state.joint_transforms[0].rotation.w = pose.orientation.w;
    state.multi_dof_joint_state.joint_transforms[0].rotation.x = pose.orientation.x;
    state.multi_dof_joint_state.joint_transforms[0].rotation.y = pose.orientation.y;
    state.multi_dof_joint_state.joint_transforms[0].rotation.z = pose.orientation.z;

    return true;
}

std::vector<moveit_msgs::CollisionObject>
ArmPlanningNode::get_collision_objects(const std::string& filename, const std::string& frame_id)
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg, "%s", sTemp) < 1)
        printf("Parsed string has length < 1.\n");

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file", num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i = 0; i < num_obs; ++i) {
        if (fscanf(fCfg, "%s", sTemp) < 1)
            printf("Parsed string has length < 1.\n");
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j = 0; j < 6; ++j) {
            if (fscanf(fCfg, "%s", sTemp) < 1)
                printf("Parsed string has length < 1.\n");
            if (!feof(fCfg) && strlen(sTemp) != 0)
                objects[i][j] = atof(sTemp);
        }
    }

    return get_collision_cubes(objects, object_ids, frame_id);
}

void ArmPlanningNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    last_joint_state_.name = manipulator_joint_names_;
    last_joint_state_.position.resize(manipulator_joint_names_.size());
    for (int i = 0; i < (int)manipulator_joint_names_.size(); ++i) {
        const auto& joint_names = msg->name; // the joint names in this joint message
        for (int j = 0; j < (int)msg->name.size(); ++j) {
            if (msg->name[j] == manipulator_joint_names_[i]) {
                last_joint_state_.position[i] = msg->position[j];
            }
        }
    }
}

bool ArmPlanningNode::add_interpolation_to_plan(moveit_msgs::GetMotionPlan::Response& res) const
{
    if (min_limits_.size() != planning_joints_.size() ||
        max_limits_.size() != planning_joints_.size() ||
        continuous_.size() != planning_joints_.size())
    {
        ROS_ERROR("Hardcoded joint variable descriptions do not match the number of planning joints");
        return false;
    }

    trajectory_msgs::JointTrajectory& joint_trajectory = res.motion_plan_response.trajectory.joint_trajectory;

    ROS_INFO("Interpolating trajectory of size %zd", joint_trajectory.points.size());

    if (joint_trajectory.joint_names.empty()) {
        joint_trajectory.joint_names = planning_joints_;
    }

    // find the index of each planning joint in the returned message
    std::map<std::string, int> planning_joint_indices;
    std::vector<std::string> index_to_planning_joint;
    for (int i = 0; i < (int)joint_trajectory.joint_names.size(); ++i) {
        const std::string& joint_name = joint_trajectory.joint_names[i];
        planning_joint_indices[joint_name] = i;
    }

    // interpolate between each consecutive pair of waypoints
    trajectory_msgs::JointTrajectory interp_traj;
    interp_traj.joint_names = joint_trajectory.joint_names;
    for (int i = 0; i < (int)joint_trajectory.points.size() - 1; ++i) {
        const trajectory_msgs::JointTrajectoryPoint& curr_point = joint_trajectory.points[i];
        const trajectory_msgs::JointTrajectoryPoint& next_point = joint_trajectory.points[i + 1];

        if (curr_point.positions.size() != planning_joints_.size() || next_point.positions.size() != planning_joints_.size()) {
            ROS_WARN("Intermediate joint trajectory point does not have as many joints as the number of planning joints (%zd)", curr_point.positions.size());
            return false;
        }

        // grab the start and end angles in the order of the planning joints
        std::vector<double> start(planning_joints_.size());
        std::vector<double> end(planning_joints_.size());
        for (int j = 0; j < (int)planning_joints_.size(); ++j) {
            const std::string& planning_joint = planning_joints_[j];
            start[j]  = curr_point.positions[planning_joint_indices[planning_joint]];
            end[j]    = next_point.positions[planning_joint_indices[planning_joint]];
        }

        // add the first point
        if (i == 0) {
            trajectory_msgs::JointTrajectoryPoint first_point;
            first_point.positions.resize(planning_joints_.size());
            for (int j = 0; j < (int)planning_joints_.size(); ++j) {
                const std::string& planning_joint = planning_joints_[j];
                first_point.positions[planning_joint_indices[planning_joint]] = start[j];
            }
            interp_traj.points.push_back(first_point);
        }

        // find the largest angle difference from start to end to determine the number of increments
        std::vector<double> angle_distances(planning_joints_.size());
        for (int j = 0; j < (int)planning_joints_.size(); ++j) {
            angle_distances[j] = sbpl::utils::ShortestAngleDistWithLimits(start[j], end[j], min_limits_[j], max_limits_[j]);
        }
        double largest_angle_dist = *std::max_element(angle_distances.begin(), angle_distances.end());
        // interpolate all angles at the resolution necessary to make sure no
        // angle moves more than 1 degree between points and all angles start
        // and stop moving at the same time
        double num_increments = largest_angle_dist / sbpl::utils::ToRadians(1.0);

        std::vector<double> inc(planning_joints_.size(), sbpl::utils::ToRadians(1.0)); // this is variable depending on transition
//        std::vector<double> inc(planning_joints_.size()); // this is variable depending on transition
//        for (int j = 0; j < (int)planning_joints_.size(); ++j) {
//            inc[j] = angle_distances[j] / num_increments;
//        }

        std::vector<std::vector<double>> path;
        if (!sbpl::interp::InterpolatePath(start, end, min_limits_, max_limits_, inc, continuous_, path)) {
            ROS_ERROR("Failed to interpolate planned path");
            return false;
        }

        // for each point in the interpolated path
        for (int j = 0; j < (int)path.size(); ++j) {
            // except the first point (already covered by the end point of the previous interpolated waypoint trajectory
            if (j != 0) {
                const std::vector<double>& interm_point = path[j];
                trajectory_msgs::JointTrajectoryPoint interm_jtpt;
                interm_jtpt.positions.resize(planning_joints_.size());
                for (int k = 0; k < planning_joints_.size(); ++k) {
                    const std::string& planning_joint = planning_joints_[k];
                    interm_jtpt.positions[planning_joint_indices[planning_joint]] = interm_point[k];
                }
                interp_traj.points.push_back(interm_jtpt);
            }
        }
    }

    ROS_INFO("Interpolated joint trajectory contains %zd points", interp_traj.points.size());

    joint_trajectory = interp_traj;

    return true;
}

void ArmPlanningNode::publish_trajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
    const std::vector<std::string> joint_names = { "arm_1_shoulder_twist",
                                                   "arm_2_shoulder_lift",
                                                   "arm_3_elbow_twist",
                                                   "arm_4_elbow_lift",
                                                   "arm_5_wrist_twist",
                                                   "arm_6_wrist_lift",
                                                   "arm_7_gripper_lift",
    };

    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = "";
    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();

    // map from planning joints to index in resulting message
    std::map<std::string, int> planning_joint_indices;
    for (int i = 0; i < (int)trajectory.joint_trajectory.joint_names.size(); ++i) {
        const std::string& joint_name = trajectory.joint_trajectory.joint_names[i];
        planning_joint_indices.insert({joint_name, i});
    }

    // for each point in the resulting trajectory
    if (!trajectory.joint_trajectory.points.empty()) {
        traj.joint_names = joint_names;
        traj.points.resize(trajectory.joint_trajectory.points.size());
        int pidx = 0;
        double t = 0.0;
        for (const auto& point : trajectory.joint_trajectory.points) {
            trajectory_msgs::JointTrajectoryPoint& traj_point = traj.points[pidx];
            traj_point.positions.resize(traj.joint_names.size());
            traj_point.velocities.resize(traj.joint_names.size());
            int jidx = 0;
            for (const auto& joint_name : traj.joint_names) {
                if (std::find(planning_joints_.begin(), planning_joints_.end(), joint_name) != planning_joints_.end()) {
                    // if this joint is a planning joint grab it from the planner trajectory
                    traj_point.positions[jidx] = point.positions[planning_joint_indices[joint_name]];
                }
                else {
                    // just set to 0 for now; search should be seeded with the initial non-variable state
                    traj_point.positions[jidx] = 0.0;
                }
                traj_point.time_from_start = ros::Duration(t);
                traj_point.velocities[jidx] = 20.0;
                ++jidx;
            }

            t += 0.10;
            ++pidx;
        }

        if (!use_action_server_) {
            joint_trajectory_pub_.publish(traj);
        }
        else {
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = std::move(traj);
            action_client_.sendGoal(goal);
            action_client_.waitForResult();
            if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Yay! The dishes are now clean");
            }
            ROS_INFO("Current State: %s", action_client_.getState().toString().c_str());
        }
    }
}

void ArmPlanningNode::apply_shortcutting(moveit_msgs::GetMotionPlan::Response& res) const
{
    trajectory_msgs::JointTrajectory& joint_trajectory = res.motion_plan_response.trajectory.joint_trajectory;

    std::vector<int> costs(joint_trajectory.points.size() - 1, 1);

    JointInterpolationPathGenerator generator;
    if (!generator.initialize(collision_checker_, min_limits_, max_limits_, continuous_)) {
        ROS_ERROR("Failed to initialize Joint Interpolation Path Generator");
        return;
    }

    std::vector<JointInterpolationPathGenerator> generators;
    generators.push_back(generator);

    std::vector<trajectory_msgs::JointTrajectoryPoint> new_points;
    bool shortcut_res = sbpl::shortcut::ShortcutPath(joint_trajectory.points, costs, generators, new_points);

    if (!shortcut_res) {
        ROS_ERROR("Failed to shortcut trajectory");
    }
    else {
        joint_trajectory.points = std::move(new_points);
    }
}

} //namespace hdt

