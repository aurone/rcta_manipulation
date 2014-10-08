#include "ArmPlanningNode.h"
#include <cstdio>
#include <leatherman/utils.h>
#include <octomap_msgs/conversions.h>
#include <sbpl_geometry_utils/utils.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include "HDTRobotModel.h"

namespace hdt
{

ArmPlanningNode::ArmPlanningNode() :
    nh_(),
    ph_("~"),
    marker_array_pub_(),
    joint_trajectory_pub_(),
    joint_states_sub_(),
    move_command_server_(),
    action_client_("arm_controller/joint_trajectory_action"),
    action_set_filename_(),
    kinematics_frame_(),
    planning_frame_(),
    object_filename_(),
    urdf_string_(),
    robot_model_(),
    planner_robot_model_(),
    distance_field_(),
    grid_(),
    collision_checker_(),
    sbpl_action_set_(),
    planner_(),
    last_joint_state_(),
    joint_staleness_threshold_(10.0),
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

    auto move_command_callback = boost::bind(&ArmPlanningNode::move_arm, this, _1);
    move_command_server_.reset(new MoveArmActionServer("move_arm_command", move_command_callback, false));
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

bool ArmPlanningNode::reinit(const hdt::MoveArmCommandGoal& goal)
{
    return reinit_robot(goal) && reinit_collision_model(goal) && reinit_sbpl(goal);
}

bool ArmPlanningNode::reinit_robot(const hdt::MoveArmCommandGoal& goal)
{
    planner_robot_model_->setKinematicsToPlanningTransform(KDL::Frame(), std::string());
    return true;
}

bool ArmPlanningNode::reinit_collision_model(const hdt::MoveArmCommandGoal& goal)
{
    ROS_INFO("Reinitializing collision model");
    // update the world
    std::shared_ptr<octomap::AbstractOcTree> octomap(octomap_msgs::fullMsgToMap(goal.octomap));
    octomap::OcTree* octree(dynamic_cast<octomap::OcTree*>(octomap.get()));
    bool valid_octomap = (bool)octree;

    planning_frame_ = valid_octomap ? goal.octomap.header.frame_id : robot_frame_;

    const double max_dist_m = 0.2;
    const double cell_res_m = 0.02;

    // initialize the distance field
    std::unique_ptr<distance_field::PropagationDistanceField> distance_field;
    if (!valid_octomap) {
        // still need to instantiate a distance field to check for collisions against other collision groups
        ROS_WARN("  Octomap from message does not support OcTree type requirements");
        ROS_WARN("  Planning with empty collision map in the robot frame");

        const double size_x = 3.0, size_y = 3.0, size_z = 3.0;
        const double origin_x = -0.75, origin_y = -1.25, origin_z = -1.0;
        distance_field.reset(new distance_field::PropagationDistanceField(
                size_x, size_y, size_z, cell_res_m, origin_x, origin_y, origin_z, max_dist_m));
        if (!distance_field) {
            ROS_ERROR("Failed to instantiate Propagation Distance Field");
            return false;
        }

        distance_field->reset();
    }
    else {
        ROS_INFO("  Octree: %p", octree);
        size_t num_nodes = octree->calcNumNodes();
        ROS_INFO("  Num Nodes: %zd", num_nodes);
        ROS_INFO("  Memory Usage: %zd bytes", octree->memoryUsage());
        ROS_INFO("  Num Leaf Nodes: %zd", octree->getNumLeafNodes());

        unsigned num_thresholded, num_other;
        octree->calcNumThresholdedNodes(num_thresholded, num_other);
        ROS_INFO("  Num Thresholded Nodes: %u", num_thresholded);
        ROS_INFO("  Num Other Nodes: %u", num_other);

        const octomap::point3d octomap_min = octree->getBBXMin();
        const octomap::point3d octomap_max = octree->getBBXMax();
        const octomap::point3d octomap_center = octree->getBBXCenter();
        double clamping_thresh_min = octree->getClampingThresMin();
        double clamping_thresh_max = octree->getClampingThresMax();

        ROS_INFO("  Bounding Box Set: %s", octree->bbxSet() ? "TRUE" : "FALSE");
        ROS_INFO("  Bounding Box Min: (%0.3f, %0.3f, %0.3f)", octomap_min.x(), octomap_min.y(), octomap_min.z());
        ROS_INFO("  Bounding Box Max: (%0.3f, %0.3f, %0.3f)", octomap_max.x(), octomap_max.y(), octomap_max.z());
        ROS_INFO("  Bounding Box Center: (%0.3f, %0.3f, %0.3f)", octomap_center.x(), octomap_center.y(), octomap_center.z());
        ROS_INFO("  Clamping Threshold Min: %0.3f", clamping_thresh_min);
        ROS_INFO("  Clamping Threshold Max: %0.3f", clamping_thresh_max);

        double metric_min_x, metric_min_y, metric_min_z;
        double metric_max_x, metric_max_y, metric_max_z;
        double metric_size_x, metric_size_y, metric_size_z;
        octree->getMetricMin(metric_min_x, metric_min_y, metric_min_z);
        octree->getMetricMax(metric_max_x, metric_max_y, metric_max_z);

        ROS_INFO("  Metric Min: (%0.3f, %0.3f, %0.3f)", metric_min_x, metric_min_y, metric_min_z);
        ROS_INFO("  Metric Max: (%0.3f, %0.3f, %0.3f)", metric_max_x, metric_max_y, metric_max_z);

        octree->getMetricSize(metric_size_x, metric_size_y, metric_size_z);
        ROS_INFO("  Metric Size: (%0.3f, %0.3f, %0.3f)", metric_size_x, metric_size_y, metric_size_z);

        ROS_INFO("  Node Size (max depth): %0.6f", octree->getNodeSize(octree->getTreeDepth()));
        ROS_INFO("  Occupancy Threshold: %0.3f", octree->getOccupancyThres());
        ROS_INFO("  Probability Hit: %0.3f", octree->getProbHit());
        ROS_INFO("  Probability Miss: %0.3f", octree->getProbMiss());
        ROS_INFO("  Resolution: %0.3f", octree->getResolution());
        ROS_INFO("  Depth: %u", octree->getTreeDepth());
        ROS_INFO("  Tree Type: %s", octree->getTreeType().c_str());

        distance_field.reset(new distance_field::PropagationDistanceField(*octree, octomap_min, octomap_max, max_dist_m));
        if (!distance_field) {
            ROS_ERROR("Failed to instantiate Propagation Distance Field");
            return false;
        }
    }

    distance_field_ = std::move(distance_field);

    ROS_INFO("Initializing Occupancy Grid");
    grid_.reset(new sbpl_arm_planner::OccupancyGrid(distance_field_.get()));
    if (!grid_) {
        ROS_ERROR("Failed to instantiate Occupancy Grid");
        return false;
    }

    std::string collision_model_frame = valid_octomap ? goal.octomap.header.frame_id : robot_root_;
    grid_->setReferenceFrame(collision_model_frame);

    ROS_INFO("  OccupancyGrid:");
    int num_cells_x, num_cells_y, num_cells_z;
    grid_->getGridSize(num_cells_x, num_cells_y, num_cells_z);
    ROS_INFO("    Dimensions (cells): %d x %d x %d", num_cells_x, num_cells_y, num_cells_z);
    double grid_size_x, grid_size_y, grid_size_z;
    grid_->getWorldSize(grid_size_x, grid_size_y, grid_size_z);
    ROS_INFO("    Dimensions (world): %0.3f x %0.3f x %0.3f", grid_size_x, grid_size_y, grid_size_z);
    double grid_origin_x, grid_origin_y, grid_origin_z;
    grid_->getOrigin(grid_origin_x, grid_origin_y, grid_origin_z);
    ROS_INFO("    Origin (world): < %0.3f, %0.3f, %0.3f >", grid_origin_x, grid_origin_y, grid_origin_z);
    ROS_INFO("    Resolution: %0.3f", grid_->getResolution());
    ROS_INFO("    Reference Frame: %s", grid_->getReferenceFrame().c_str());

    collision_checker_.reset(new sbpl_arm_planner::SBPLCollisionSpace(grid_.get()));
    if (!collision_checker_) {
        ROS_ERROR("  Failed to instantiate SBPL Collision Space");
        return false;
    }

    const std::string group_name = "manipulator"; // This has something to do with the collision checking config file?
    if (!collision_checker_->init(group_name) ||
        !collision_checker_->setPlanningJoints(robot_model_->joint_names()))
    {
        ROS_ERROR("  Failed to initialize SBPL Collision Checker");
        return false;
    }

    ROS_INFO("Successfully reinitialized collision model");
    return true;
}

bool ArmPlanningNode::reinit_sbpl(const hdt::MoveArmCommandGoal& goal)
{
    planner_.reset(new sbpl_arm_planner::SBPLArmPlannerInterface(
            planner_robot_model_.get(), collision_checker_.get(), sbpl_action_set_.get(), distance_field_.get()));

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

bool ArmPlanningNode::init_robot()
{
    if (!nh_.hasParam("robot_description")) {
        ROS_ERROR("Missing parameter \"robot_description\"");
        return false;
    }

    nh_.getParam("robot_description", urdf_string_);

    auto urdf_model = urdf::parseURDF(urdf_string_);
    if (!urdf_model) {
        ROS_ERROR("Failed to parse URDF");
        return false;
    }

    robot_name_ = urdf_model->getName();
    robot_root_ = urdf_model->getRoot()->name;

    size_t num_robot_joints = urdf_model->joints_.size();
    last_joint_state_.header.frame_id = "";
    last_joint_state_.header.seq = 0;
    last_joint_state_.header.stamp = ros::Time(0);
    last_joint_state_.name = urdf_model->joints_;
    last_joint_state_.position = std::vector<double>(num_robot_joints, 0.0);
    last_joint_state_.velocity = std::vector<double>(num_robot_joints, 0.0);
    last_joint_state_.effort = std::vector<double>(num_robot_joints, 0.0);
    received_joint_state_ = std::vector<ros::Time>(num_robot_joints, ros::Time(0));

    // Initialize Robot Model
    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string_);
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model from URDF");
        return false;
    }

    // Initialize Planning Robot Model
    HDTRobotModel* hdt_robot_model = new HDTRobotModel;
    if (!hdt_robot_model || !hdt_robot_model->init(urdf_string_)) {
        ROS_ERROR("Failed to initialize HDT Robot Model");
        return false;
    }
    planner_robot_model_.reset(hdt_robot_model);
    if (!planner_robot_model_) {
        ROS_ERROR("Failed to instantiate KDL Robot Model");
    }

    std::vector<std::string> planning_joints = robot_model_->joint_names();
    if (!planner_robot_model_->init(urdf_string_, planning_joints)) {
        ROS_ERROR("Failed to initialize KDL Robot Model");
        return false;
    }
    planner_robot_model_->setPlanningLink(robot_model_->joint_names().back());

    kinematics_frame_ = "arm_mount_panel_dummy";

    return true;
}

bool ArmPlanningNode::init_collision_model()
{
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

    return true;
}

void ArmPlanningNode::move_arm(const hdt::MoveArmCommandGoal::ConstPtr& request)
{
    ros::Time now = ros::Time::now();
    ROS_INFO("Received Move Arm Command Goal");

    ////////////////////////////////////////////////////////////////////////////////
    // Check for up-to-date state
    ////////////////////////////////////////////////////////////////////////////////

    // check for recent joint information
    if (std::any_of(received_joint_state_.begin(), received_joint_state_.end(),
        [&](const ros::Time& time) { return time < now - joint_staleness_threshold_; }))
    {
        hdt::MoveArmCommandResult result;
        result.success = false;
        std::stringstream ss;
        ss << "Joint information is more than " << joint_staleness_threshold_.toSec() << " seconds old";
        move_command_server_->setAborted(result, ss.str());
        return;
    }

    geometry_msgs::Pose robot_pose_planning_frame = geometry_msgs::IdentityPose();
    if (request->octomap.id.empty()) {
        ROS_WARN("Unidentfied octomap received. Planning in robot's local frame without obstacles");
    }
    else {
        const std::string& robot_local_frame = robot_root_;
        tf::StampedTransform map_to_robot;
        try {
            listener_.lookupTransform(request->octomap.header.frame_id, robot_local_frame, ros::Time(0), map_to_robot);
            Eigen::Affine3d map_to_robot_eigen;
            msg_utils::convert(map_to_robot, map_to_robot_eigen);
            tf::poseEigenToMsg(map_to_robot_eigen, robot_pose_planning_frame);
        }
        catch (const tf::TransformException& ex) {
            hdt::MoveArmCommandResult result;
            result.success = false;
            move_command_server_->setAborted(result, ex.what());
            return;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Set up the planning scene
    ////////////////////////////////////////////////////////////////////////////////

    moveit_msgs::PlanningScenePtr scene(new moveit_msgs::PlanningScene);
    scene->name = "manipulation_planning_scene";

    moveit_msgs::RobotState robot_state;
    robot_state.joint_state = last_joint_state_;
    robot_state.multi_dof_joint_state.joint_names = { "world_pose" };
    robot_state.multi_dof_joint_state.joint_transforms.resize(1);
    robot_state.multi_dof_joint_state.joint_transforms[0].translation.x = robot_pose_planning_frame.position.x;
    robot_state.multi_dof_joint_state.joint_transforms[0].translation.y = robot_pose_planning_frame.position.y;
    robot_state.multi_dof_joint_state.joint_transforms[0].translation.z = robot_pose_planning_frame.position.z;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.w = robot_pose_planning_frame.orientation.w;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.x = robot_pose_planning_frame.orientation.x;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.y = robot_pose_planning_frame.orientation.y;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.z = robot_pose_planning_frame.orientation.z;
    scene->robot_state = robot_state;

    scene->robot_model_name = robot_name_;
    scene->robot_model_root = robot_root_;

    scene->fixed_frame_transforms.clear(); // todo: are these necessary?
    scene->allowed_collision_matrix; // todo: covered by sbpl_collision_model?
    scene->link_padding.clear(); // todo: covered by sbpl_collision_model?
    scene->link_scale.clear(); // todo: covered by sbpl_collision_model?
    scene->object_colors.clear(); // todo: also probably not used

    scene->world; // todo: covered by sbpl_collision_model/occupancy grid?

    scene->is_diff = false;

    ////////////////////////////////////////////////////////////////////////////////
    // Reinitialize for updated collision model
    ////////////////////////////////////////////////////////////////////////////////

    if (!reinit(*request)) {
        hdt::MoveArmCommandResult result;
        result.success = false;
        move_command_server_->setAborted(result, "Failed to initialize Arm Planner");
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Plan to the received goal state
    ////////////////////////////////////////////////////////////////////////////////

    bool success = false;
    trajectory_msgs::JointTrajectory result_traj;

    collision_checker_->setPlanningScene(*scene);

    if (request->type == hdt::MoveArmCommandGoal::JointGoal) {
        ROS_INFO("Received a joint goal");
        success = plan_to_joint_goal(scene, scene->robot_state, *request, result_traj);
    }
    else if (request->type == hdt::MoveArmCommandGoal::EndEffectorGoal) {
        ROS_INFO("Received an end effector goal");
        success = plan_to_eef_goal(scene, scene->robot_state, *request, result_traj);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Post-process the plan and send to the joint trajectory follower
    ////////////////////////////////////////////////////////////////////////////////

    if (success) {

        ROS_INFO("Original joint path (%zd points):", result_traj.points.size());
        for (int i = 0; i < (int)result_traj.points.size(); ++i) {
            const trajectory_msgs::JointTrajectoryPoint& joint_state = result_traj.points[i];
            ROS_INFO("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
        }

        apply_shortcutting(result_traj);

        ROS_INFO("Shortcut trajectory (%zd points):", result_traj.points.size());
        for (int i = 0; i < (int)result_traj.points.size(); ++i) {
            const trajectory_msgs::JointTrajectoryPoint& joint_state = result_traj.points[i];
            ROS_INFO("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
        }

        bool interp_res = add_interpolation_to_plan(result_traj);
        if (!interp_res) {
            ROS_ERROR("Failed to interpolate joint trajectory");
            hdt::MoveArmCommandResult result;
            result.success = false;
            move_command_server_->setAborted(result);
            return;
        }

        publish_trajectory(result_traj);

        hdt::MoveArmCommandResult result;
        result.success = true;
        result.trajectory = result_traj;
        move_command_server_->setSucceeded(result);
    }
    else {
        hdt::MoveArmCommandResult result;
        result.success = false;
        result.trajectory;
        move_command_server_->setAborted(result);
    }
}

void ArmPlanningNode::fill_constraint(
    const std::vector<double>& pose,
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

moveit_msgs::CollisionObject ArmPlanningNode::get_collision_cube(
    const geometry_msgs::Pose& pose,
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
    ros::Time now = ros::Time::now();
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        double joint_position = msg->position[i];
        double joint_velocity = msg->velocity[i];
        double joint_effort = msg->effort[i];

        int jidx = msg_utils::get_joint_index(last_joint_state_, joint_name);
        if (jidx == -1) {
            ROS_WARN("Ignoring unrecognized joint '%s' from joint state message", joint_name.c_str());
            continue;
        }

        received_joint_state_[jidx] = now;

        last_joint_state_.header.stamp = msg->header.stamp;
        last_joint_state_.position[jidx] = joint_position;
        last_joint_state_.velocity[jidx] = joint_velocity;
        last_joint_state_.effort[jidx] = joint_effort;
    }
}

bool ArmPlanningNode::add_interpolation_to_plan(trajectory_msgs::JointTrajectory& res_traj) const
{
    if (robot_model_->min_limits().size() != robot_model_->num_joints() ||
        robot_model_->max_limits().size() != robot_model_->num_joints() ||
        robot_model_->continuous().size() != robot_model_->num_joints())
    {
        ROS_ERROR("Joint variable descriptions do not match the number of planning joints");
        ROS_ERROR("  Num Joints: %zd", robot_model_->num_joints());
        ROS_ERROR("  Num Min Limits: %zd", robot_model_->min_limits().size());
        ROS_ERROR("  Num Max Limits: %zd", robot_model_->max_limits().size());
        ROS_ERROR("  Num Continuous: %zd", robot_model_->continuous().size());
        return false;
    }

    ROS_INFO("Interpolating trajectory of size %zd", res_traj.points.size());

    if (res_traj.joint_names.empty()) {
        res_traj.joint_names = robot_model_->joint_names();
    }

    // find the index of each planning joint in the returned message
    std::map<std::string, int> planning_joint_indices;
    std::vector<std::string> index_to_planning_joint;
    for (int i = 0; i < (int)res_traj.joint_names.size(); ++i) {
        const std::string& joint_name = res_traj.joint_names[i];
        planning_joint_indices[joint_name] = i;
    }

    // interpolate between each consecutive pair of waypoints
    trajectory_msgs::JointTrajectory interp_traj;
    interp_traj.joint_names = res_traj.joint_names;
    for (int i = 0; i < (int)res_traj.points.size() - 1; ++i) {
        const trajectory_msgs::JointTrajectoryPoint& curr_point = res_traj.points[i];
        const trajectory_msgs::JointTrajectoryPoint& next_point = res_traj.points[i + 1];

        if (curr_point.positions.size() != robot_model_->joint_names().size() || next_point.positions.size() != robot_model_->joint_names().size()) {
            ROS_WARN("Intermediate joint trajectory point does not have as many joints as the number of planning joints (%zd)", curr_point.positions.size());
            return false;
        }

        // grab the start and end angles in the order of the planning joints
        std::vector<double> start(robot_model_->joint_names().size());
        std::vector<double> end(robot_model_->joint_names().size());
        for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
            const std::string& planning_joint = robot_model_->joint_names()[j];
            start[j]  = curr_point.positions[planning_joint_indices[planning_joint]];
            end[j]    = next_point.positions[planning_joint_indices[planning_joint]];
        }

        // add the first point
        if (i == 0) {
            trajectory_msgs::JointTrajectoryPoint first_point;
            first_point.positions.resize(robot_model_->joint_names().size());
            for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
                const std::string& planning_joint = robot_model_->joint_names()[j];
                first_point.positions[planning_joint_indices[planning_joint]] = start[j];
            }
            interp_traj.points.push_back(first_point);
        }

        // find the largest angle difference from start to end to determine the number of increments
        std::vector<double> angle_distances(robot_model_->joint_names().size());
        for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
            angle_distances[j] = sbpl::utils::ShortestAngleDistWithLimits(start[j], end[j], robot_model_->min_limits()[j], robot_model_->max_limits()[j]);
        }
        double largest_angle_dist = *std::max_element(angle_distances.begin(), angle_distances.end());
        // interpolate all angles at the resolution necessary to make sure no
        // angle moves more than 1 degree between points and all angles start
        // and stop moving at the same time
        double num_increments = largest_angle_dist / sbpl::utils::ToRadians(1.0);

        std::vector<double> inc(robot_model_->joint_names().size(), sbpl::utils::ToRadians(1.0)); // this is variable depending on transition
//        std::vector<double> inc(robot_model_->joint_names().size()); // this is variable depending on transition
//        for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
//            inc[j] = angle_distances[j] / num_increments;
//        }

        std::vector<std::vector<double>> path;
        if (!sbpl::interp::InterpolatePath(start, end, robot_model_->min_limits(), robot_model_->max_limits(), inc, robot_model_->continuous(), path)) {
            ROS_ERROR("Failed to interpolate planned path");
            return false;
        }

        // for each point in the interpolated path
        for (int j = 0; j < (int)path.size(); ++j) {
            // except the first point (already covered by the end point of the previous interpolated waypoint trajectory
            if (j != 0) {
                const std::vector<double>& interm_point = path[j];
                trajectory_msgs::JointTrajectoryPoint interm_jtpt;
                interm_jtpt.positions.resize(robot_model_->joint_names().size());
                for (int k = 0; k < robot_model_->joint_names().size(); ++k) {
                    const std::string& planning_joint = robot_model_->joint_names()[k];
                    interm_jtpt.positions[planning_joint_indices[planning_joint]] = interm_point[k];
                }
                interp_traj.points.push_back(interm_jtpt);
            }
        }
    }

    ROS_INFO("Interpolated joint trajectory contains %zd points", interp_traj.points.size());

    res_traj = interp_traj;

    return true;
}

void ArmPlanningNode::publish_trajectory(const trajectory_msgs::JointTrajectory& joint_trajectory)
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
    for (int i = 0; i < (int)joint_trajectory.joint_names.size(); ++i) {
        const std::string& joint_name = joint_trajectory.joint_names[i];
        planning_joint_indices.insert({joint_name, i});
    }

    // for each point in the resulting trajectory
    if (!joint_trajectory.points.empty()) {
        traj.joint_names = joint_names;
        traj.points.resize(joint_trajectory.points.size());
        int pidx = 0;
        double t = 0.0;
        for (const auto& point : joint_trajectory.points) {
            trajectory_msgs::JointTrajectoryPoint& traj_point = traj.points[pidx];
            traj_point.positions.resize(traj.joint_names.size());
            traj_point.velocities.resize(traj.joint_names.size());
            int jidx = 0;
            for (const auto& joint_name : traj.joint_names) {
                if (std::find(robot_model_->joint_names().begin(), robot_model_->joint_names().end(), joint_name) != robot_model_->joint_names().end()) {
                    // if this joint is a planning joint grab it from the planner trajectory
                    traj_point.positions[jidx] = point.positions[planning_joint_indices[joint_name]];
                }
                else {
                    // just set to 0 for now; search should be seeded with the initial non-variable state
                    traj_point.positions[jidx] = 0.0;
                }
                traj_point.time_from_start = ros::Duration(t);
                traj_point.velocities[jidx] = sbpl::utils::ToRadians(20.0);
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

void ArmPlanningNode::apply_shortcutting(trajectory_msgs::JointTrajectory& joint_trajectory) const
{
    std::vector<int> costs(joint_trajectory.points.size() - 1, 1);

    JointInterpolationPathGenerator generator;
    if (!generator.initialize(collision_checker_, robot_model_->min_limits(), robot_model_->max_limits(), robot_model_->continuous())) {
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

bool ArmPlanningNode::plan_to_eef_goal(
    const moveit_msgs::PlanningScenePtr& scene,
    const moveit_msgs::RobotState& start,
    const hdt::MoveArmCommandGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{
    // fill goal state
    moveit_msgs::GetMotionPlan::Request req;
    req.motion_plan_request.goal_constraints.resize(1);
    std::vector<double> goal_vector = convert_to_sbpl_goal(goal.goal_pose);
    fill_constraint(goal_vector, planning_frame_, req.motion_plan_request.goal_constraints[0]);
    ROS_WARN("Created a goal in the '%s' frame", req.motion_plan_request.goal_constraints.front().position_constraints[0].header.frame_id.c_str());
    req.motion_plan_request.allowed_planning_time = 10.0; //2.0;
    req.motion_plan_request.start_state = scene->robot_state;

    // plan
    ROS_INFO("Calling solve...");
    moveit_msgs::GetMotionPlan::Response res;
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
        traj = res.motion_plan_response.trajectory.joint_trajectory;
    }

    return plan_result;
}

bool ArmPlanningNode::plan_to_joint_goal(
    const moveit_msgs::PlanningScenePtr& scene,
    const moveit_msgs::RobotState& start,
    const hdt::MoveArmCommandGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{

    sensor_msgs::JointState start_joint_state = start.joint_state;
    if (!msg_utils::reorder_joints(start_joint_state, robot_model_->joint_names())) {
        ROS_WARN("Start robot state contains joints other than manipulator joints");
        return false;
    }

    sensor_msgs::JointState goal_joint_state = goal.goal_joint_state;
    if (!msg_utils::reorder_joints(goal_joint_state, robot_model_->joint_names())) {
        ROS_WARN("Goal state contains joints other than manipulator joints");
        return false;
    }

    // fill goal state
    moveit_msgs::GetMotionPlan::Request req;
    req.motion_plan_request.goal_constraints.resize(1);

    std::vector<double> goal_vector = goal_joint_state.position;
    std::vector<double> goal_tolerances (7, 0.05); 

    req.motion_plan_request.goal_constraints.resize(1);
    req.motion_plan_request.goal_constraints.front().joint_constraints.resize(goal_vector.size());
    for(int i = 0; i < goal_vector.size(); i++){
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].position = goal_joint_state.position[i];
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].joint_name = goal_joint_state.name[i];
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_above = 3.0 * M_PI / 180.0;
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_below = -3.0 * M_PI / 180.0;
      ROS_INFO("%s: %.3f tol(%.3f, %.3f)", goal_joint_state.name[i].c_str(), goal_joint_state.position[i], req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_below, req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_above);
    }

    ROS_WARN("Created a 7DoF goal!");

    req.motion_plan_request.allowed_planning_time = 10.0; //2.0;
    req.motion_plan_request.start_state = scene->robot_state;

    // plan
    ROS_INFO("Calling solve...");
    moveit_msgs::GetMotionPlan::Response res;
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
        traj = res.motion_plan_response.trajectory.joint_trajectory;
    }

    return plan_result;

    /*trajectory_msgs::JointTrajectory interp_traj;
    interp_traj.header.frame_id = "";
    interp_traj.joint_names = robot_model_->joint_names();
    interp_traj.points.resize(2);

    sensor_msgs::JointState start_joint_state = start.joint_state;
    if (!msg_utils::reorder_joints(start_joint_state, robot_model_->joint_names())) {
        ROS_WARN("Start robot state contains joints other than manipulator joints");
        return false;
    }

    sensor_msgs::JointState goal_joint_state = goal.goal_joint_state;
    if (!msg_utils::reorder_joints(goal_joint_state, robot_model_->joint_names())) {
        ROS_WARN("Goal state contains joints other than manipulator joints");
        return false;
    }

    interp_traj.points[0].positions = start_joint_state.position;
    interp_traj.points[1].positions = goal_joint_state.position;

    clamp_to_joint_limits(interp_traj.points[0].positions);
    clamp_to_joint_limits(interp_traj.points[1].positions);

    add_interpolation_to_plan(interp_traj);

    for (const trajectory_msgs::JointTrajectoryPoint& point : interp_traj.points) {
        double dist = 0.0;
        if (!collision_checker_->isStateValid(point.positions, false, false, dist)) {
            return false;
        }
    }

    traj = std::move(interp_traj);
    return true;*/
}

std::vector<double> ArmPlanningNode::convert_to_sbpl_goal(const geometry_msgs::Pose& pose)
{
    tf::Quaternion goal_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 goal_rotation_matrix(goal_quat);
    double goal_roll, goal_pitch, goal_yaw;
    goal_rotation_matrix.getEulerYPR(goal_yaw, goal_pitch, goal_roll);
    return { pose.position.x, pose.position.y, pose.position.z, goal_roll, goal_pitch, goal_yaw };
}

void ArmPlanningNode::clamp_to_joint_limits(std::vector<double>& joint_vector)
{
    auto clamp = [](double d, double min, double max) { if (d < min) return min; else if (d > max) return max; else return d; };
    for (std::size_t i = 0; i < joint_vector.size(); ++i) {
        joint_vector[i] = clamp(joint_vector[i], robot_model_->min_limits()[i], robot_model_->max_limits()[i]);
    }
}

bool ArmPlanningNode::valid_octomap(const octomap_msgs::Octomap& msg)
{
    return true;
}

} //namespace hdt

