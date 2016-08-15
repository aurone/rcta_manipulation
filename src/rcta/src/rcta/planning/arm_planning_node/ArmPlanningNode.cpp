#include "ArmPlanningNode.h"

// standard includes
#include <cassert>
#include <cstdio>
#include <queue>

// system includes
#include <boost/date_time/posix_time/posix_time.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <octomap_msgs/conversions.h>
#include <sbpl_geometry_utils/utils.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/utils.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <rcta/common/hdt_description/RobotModel.h>

namespace hdt {

ArmPlanningNode::ArmPlanningNode() :
    m_nh(),
    m_ph("~"),
    m_marker_array_pub(),
    m_joint_trajectory_pub(),
    m_joint_states_sub(),
    m_move_arm_command_server(),
    m_follow_trajectory_client("arm_controller/joint_trajectory_action"),
    m_use_action_server(false),
    m_action_set_filename(),
    m_kinematics_frame(),
    m_planning_frame(),
    m_object_filename(),
    m_urdf_string(),
    robot_name_(),
    robot_local_frame_(),
    robot_model_(),
    m_planner_robot_model(),
    grid_(),
    m_collision_checker(),
    m_action_set(),
    m_planner(),
    last_joint_state_(),
    received_joint_state_(),
    listener_(),
    joint_staleness_threshold_(10.0),
    urdf_model_(),
    statistic_names_({
        "initial solution planning time",
        "initial epsilon",
        "initial solution expansions",
        "final epsilon planning time",
        "final epsilon",
        "solution epsilon",
        "expansions",
        "solution cost" }),
    planning_scene_()
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

    m_ph.param("use_action_server", m_use_action_server, true);
    if (m_use_action_server) {
        ROS_INFO("Using action server 'arm_controller/joint_trajectory_action'");
    }
    else {
        ROS_INFO("Publishing joint trajectories to command");
    }

    m_ph.param("apply_shortcutting", apply_shortcutting_, true);
    ROS_INFO("Apply Shortcutting: %s", boolstr(apply_shortcutting_));

    m_marker_array_pub = m_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    m_joint_trajectory_pub = m_nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    m_joint_states_sub = m_nh.subscribe("joint_states", 1, &ArmPlanningNode::joint_states_callback, this);

    auto move_command_callback = boost::bind(&ArmPlanningNode::move_arm, this, _1);
    m_move_arm_command_server.reset(new MoveArmActionServer("move_arm_command", move_command_callback, false));
    if (!m_move_arm_command_server) {
        ROS_ERROR("Failed to instantiate Move Arm Action Server");
        return false;
    }

    m_move_arm_command_server->start();

    ROS_INFO("Waiting for action client \"arm_controller/joint_trajectory_action\"");
    m_follow_trajectory_client.waitForServer();

    return true;
}

int ArmPlanningNode::run()
{
    ROS_INFO("Spinning...");
    ros::spin();
    ROS_INFO("Done spinning");
    return 0;
}

bool ArmPlanningNode::reinit(
    const Eigen::Affine3d& T_kinematics_planning,
    const std::string& planning_frame,
    const octomap_msgs::Octomap& octomap)
{
    // Reset here out of paranoia to make sure there's no way the planner can
    // attempt to reference the occupancy grid or the collision checker while
    // we're constructing fresh ones
    m_planner.reset();

    ROS_INFO("Reinitializing Planner");
    return reinit_robot(T_kinematics_planning) &&
            reinit_collision_model(planning_frame, octomap) &&
            reinit_sbpl();
}

bool ArmPlanningNode::reinit_robot(const Eigen::Affine3d& T_kinematics_planning)
{
    ROS_INFO("Reinitializing Robot");
    m_planner_robot_model.setPlanningToKinematicsTransform(T_kinematics_planning);
    return true;
}

bool ArmPlanningNode::reinit_collision_model(
    const std::string& planning_frame,
    const octomap_msgs::Octomap& octomap)
{
    // Reset here for the same reasons as resetting the planner. see reinit()
    grid_.reset();

    ROS_INFO("Reinitializing collision model for planning frame %s", planning_frame.c_str());
    // if we didn't receive an octomap, we better be planning in the robot's base frame
    assert(!octomap.header.frame_id.empty() ? (true) : (planning_frame == robot_local_frame_));
    if (planning_frame != robot_local_frame_) {
        ROS_WARN("Planning frame '%s' does not match robot local frame '%s'!", planning_frame.c_str(), robot_local_frame_.c_str());
    }

    ROS_INFO("Initializing Distance Field");

    // initialize the distance field
    const double size_x = 3.0, size_y = 3.0, size_z = 2.0;
    const double cell_res_m = 0.02;
    const double origin_x = -0.75, origin_y = -1.25, origin_z = 0.0;
    const double max_dist_m = 0.2;

    auto df = std::make_shared<distance_field::PropagationDistanceField>(
            size_x, size_y, size_z,
            cell_res_m,
            origin_x, origin_y, origin_z,
            max_dist_m);

    ROS_INFO("Initialized Distance Field! origin: (%0.3f, %0.3f, %0.3f), size: (%0.3f, %0.3f, %0.3f)", origin_x, origin_y, origin_z, size_x, size_y, size_z);

    ROS_INFO("Initializing Occupancy Grid");
    grid_ = std::make_shared<sbpl::OccupancyGrid>(df);

    const std::string df_frame = robot_local_frame_;
    grid_->setReferenceFrame(df_frame);

    ROS_INFO("  OccupancyGrid:");
    {
        int num_cells_x, num_cells_y, num_cells_z;
        grid_->getGridSize(num_cells_x, num_cells_y, num_cells_z);
        ROS_INFO("    Dimensions (cells): (%d, %d, %d)", num_cells_x, num_cells_y, num_cells_z);
        double grid_size_x, grid_size_y, grid_size_z;
        grid_->getWorldSize(grid_size_x, grid_size_y, grid_size_z);
        ROS_INFO("    Dimensions (m): (%0.3f, %0.3f, %0.3f)", grid_size_x, grid_size_y, grid_size_z);
        double grid_origin_x, grid_origin_y, grid_origin_z;
        grid_->getOrigin(grid_origin_x, grid_origin_y, grid_origin_z);
        ROS_INFO("    Origin (m): (%0.3f, %0.3f, %0.3f)", grid_origin_x, grid_origin_y, grid_origin_z);
        ROS_INFO("    Resolution: %0.3f", grid_->getResolution());
        ROS_INFO("    Reference Frame: %s", grid_->getReferenceFrame().c_str());
    }

    ROS_INFO("Initialized Occupancy Grid");

    // load the collision checking config
    // TODO: only need to do this once
    sbpl::collision::CollisionModelConfig cm_cfg;
    if (!sbpl::collision::CollisionModelConfig::Load(m_nh, cm_cfg)) {
        ROS_ERROR("Failed to initialize collision model config");
        return false;
    }

    sbpl::collision::CollisionSpaceBuilder builder;

    const std::string group_name = "manipulator";
    const std::vector<std::string>& planning_joints =
            robot_model_->joint_names();
    m_collision_checker = builder.build(
            grid_.get(), m_urdf_string, cm_cfg, group_name, planning_joints);
    if (!m_collision_checker) {
        ROS_ERROR("Failed to build Collision Space for group '%s'", group_name.c_str());
        return false;
    }

    octomap_msgs::OctomapWithPose octomap_msg;
    octomap_msg.header = octomap.header;
    octomap_msg.origin.orientation.w = 1.0;
    octomap_msg.octomap = octomap;
    if (!m_collision_checker->processOctomapMsg(octomap_msg)) {
        ROS_ERROR("Failed to add octomap to the collision checker");
        return false;
    }

    ROS_INFO("Successfully reinitialized collision model");

    return true;
}

bool ArmPlanningNode::reinit_sbpl()
{
    ROS_INFO("Reinitializing SBPL");
    m_planner.reset(new sbpl::manip::ArmPlannerInterface(
            &m_planner_robot_model,
            m_collision_checker.get(),
            &m_action_set,
            grid_.get()));

    if (!m_planner) {
        ROS_ERROR("Failed to instantiate SBPL Arm Planner Interface");
        return false;
    }

    sbpl::manip::PlanningParams params;
    params.planning_frame_ = "base_link";
    params.num_joints_;
    params.planning_joints_;
    params.coord_vals_;
    params.coord_delta_;
    params.use_multiple_ik_solutions_;
//    params.cost_multiplier_;
//    params.cost_per_cell_;
//    params.cost_per_meter_;
//    params.cost_per_second_;
//    params.time_per_cell_;
//    params.max_mprim_offset_;
    params.use_bfs_heuristic_;
    params.planning_link_sphere_radius_;
    params.planner_name_;
    params.epsilon_;
    params.allowed_time_;
    params.search_mode_;
    params.shortcut_path_ = apply_shortcutting_;
    params.interpolate_path_ = sbpl::manip::ShortcutType::JOINT_SPACE;
    params.waypoint_time_;
    params.shortcut_type;
    params.print_path_;
    params.verbose_;
    params.verbose_heuristics_;
    params.verbose_collisions_;
//    params.graph_log_;
//    params.heuristic_log_;
//    params.expands_log_;
//    params.rmodel_log_;
//    params.post_processing_log_;
//    params.solution_log_;
    if (!m_planner->init()) {
        ROS_ERROR("Failed to initialize SBPL Arm Planner Interface");
        return false;
    }

    return true;
}

bool ArmPlanningNode::init_robot()
{
    ROS_INFO("Initializing Robot Models");

    if (!m_nh.hasParam("robot_description")) {
        ROS_ERROR("Missing parameter \"robot_description\"");
        return false;
    }

    m_nh.getParam("robot_description", m_urdf_string);

    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(m_urdf_string);
    if (!urdf_model) {
        ROS_ERROR("Failed to parse URDF");
        return false;
    }

    std::vector<std::string> all_joints;
    all_joints.reserve(urdf_model->joints_.size());

    std::queue<boost::shared_ptr<const urdf::Link>> links;
    links.push(urdf_model->getRoot());
    // traverse the tree in breadth-first fashion to gather all the joints for this model
    while (!links.empty()) {
        // grab the next link
        boost::shared_ptr<const urdf::Link> link = links.front();
        links.pop();

        // gather the child joints of this link
        for (const boost::shared_ptr<urdf::Joint>& child_joint : link->child_joints) {
            if (child_joint->type != urdf::Joint::FIXED) {
                all_joints.push_back(child_joint->name);
            }
        }

        // add all children to the queue
        for (const boost::shared_ptr<urdf::Link>& child_link : link->child_links) {
            links.push(child_link);
        }
    }

    ROS_INFO("  All Joints (%zd):", all_joints.size());
    for (const std::string& joint : all_joints) {
        ROS_INFO("    %s", joint.c_str());
    }

    robot_name_ = urdf_model->getName();
    robot_local_frame_ = urdf_model->getRoot()->name;
    ROS_INFO("  Robot Name: %s", robot_name_.c_str());
    ROS_INFO("  Robot Local Frame: %s", robot_local_frame_.c_str());

    size_t num_nonfixed_joints = all_joints.size();
    last_joint_state_.header.frame_id = "";
    last_joint_state_.header.seq = 0;
    last_joint_state_.header.stamp = ros::Time(0);
    last_joint_state_.name = std::move(all_joints);
    last_joint_state_.position = std::vector<double>(num_nonfixed_joints, 0.0);
    last_joint_state_.velocity = std::vector<double>(num_nonfixed_joints, 0.0);
    last_joint_state_.effort = std::vector<double>(num_nonfixed_joints, 0.0);
    received_joint_state_ = std::vector<ros::Time>(num_nonfixed_joints, ros::Time(0));

    // Initialize Robot Model
    robot_model_ = hdt::RobotModel::LoadFromURDF(m_urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model from URDF");
        return false;
    }

    // Initialize Planning Robot Model
    if (!m_planner_robot_model.init(m_urdf_string)) {
        ROS_ERROR("Failed to initialize HDT Robot Model");
        return false;
    }

//    std::vector<std::string> planning_joints = robot_model_->joint_names();
//    if (!planner_robot_model_->init(m_urdf_string, planning_joints)) {
//        ROS_ERROR("Failed to initialize KDL Robot Model");
//        return false;
//    }
//    planner_robot_model_->setPlanningLink(robot_model_->joint_names().back());

    m_kinematics_frame = "arm_mount_panel_dummy";

    return true;
}

bool ArmPlanningNode::init_collision_model()
{
    return true;
}

bool ArmPlanningNode::init_sbpl()
{
    if (!m_ph.hasParam("action_set_filename")) {
        ROS_ERROR("Missing parameter \"action_set_filename\"");
        return false;
    }

    m_ph.getParam("action_set_filename", m_action_set_filename);

    if (!sbpl::manip::ActionSet::Load(m_action_set_filename, m_action_set)) {
        ROS_ERROR("Failed to instantiate Action Set");
        return false;
    }

    planning_scene_.reset(new moveit_msgs::PlanningScene);
    if (!planning_scene_) {
        ROS_ERROR("Failed to instantiate Planning Scene");
        return false;
    }

    return true;
}

void ArmPlanningNode::move_arm(const rcta::MoveArmCommandGoal::ConstPtr& request)
{
    ros::Time now = ros::Time::now();
    ROS_INFO("Received Move Arm Command Goal at %s", boost::posix_time::to_simple_string(now.toBoost()).c_str());

    std::string planning_frame;
    if (!request->octomap.header.frame_id.empty()) {
        planning_frame = request->octomap.header.frame_id;
    }
    else {
        planning_frame = robot_local_frame_;
    }

    ////////////////////////////////
    // Check for up-to-date state //
    ////////////////////////////////

    // check for recent joint information
    for (size_t i = 0; i < received_joint_state_.size(); ++i) {
        const ros::Time& time = received_joint_state_[i];
        if (time < now - joint_staleness_threshold_) {
            rcta::MoveArmCommandResult result;
            result.success = false;
            std::stringstream ss;
            ss << "Joint information for joint '" << last_joint_state_.name[i]
               << "' is more than " << joint_staleness_threshold_.toSec() << " seconds old";
            ROS_WARN("%s", ss.str().c_str());
            m_move_arm_command_server->setAborted(result, ss.str());
            return;
        }
    }

    ROS_INFO("  Joint States are up-to-date (within %0.3f seconds)", joint_staleness_threshold_.toSec());

    // check for recent transform from the robot to the planning frame
    geometry_msgs::Pose robot_pose_planning_frame = geometry_msgs::IdentityPose();
    if (planning_frame == robot_local_frame_) {
        ROS_WARN("Unidentfied octomap received. Planning in robot's local frame without obstacles");
    }
    else {
        tf::StampedTransform T_map_robot_tf;
        try {
            listener_.lookupTransform(planning_frame, robot_local_frame_, ros::Time(0), T_map_robot_tf);
            Eigen::Affine3d T_map_to_robot;
            msg_utils::convert(T_map_robot_tf, T_map_to_robot);
            tf::poseEigenToMsg(T_map_to_robot, robot_pose_planning_frame);
        }
        catch (const tf::TransformException& ex) {
            rcta::MoveArmCommandResult result;
            result.success = false;
            m_move_arm_command_server->setAborted(result, ex.what());
            return;
        }
    }

    ROS_INFO("  Robot Pose [%s]: %s", planning_frame.c_str(), to_string(robot_pose_planning_frame).c_str());

    // look up the transform from the kinematics frame (the frame that goals are specified in) to the planning frame
    tf::StampedTransform kinematics_to_planning_tf;
    Eigen::Affine3d T_kinematics_planning;
    ROS_INFO("Looking up trasform from planning_frame (%s) to kinematics_frame (%s)", planning_frame.c_str(), m_kinematics_frame.c_str());
    try {
        listener_.lookupTransform(m_kinematics_frame, planning_frame, ros::Time(0), kinematics_to_planning_tf);
        msg_utils::convert(kinematics_to_planning_tf, T_kinematics_planning);
    }
    catch (const tf::TransformException& ex) {
        rcta::MoveArmCommandResult result;
        result.success = false;
        m_move_arm_command_server->setAborted(result, ex.what());
        return;
    }

    ROS_INFO("  Kinematics -> Planning: %s", to_string(T_kinematics_planning).c_str());

    ///////////////////////////////
    // Set up the planning scene //
    ///////////////////////////////

    planning_scene_->name = "manipulation_planning_scene";

    moveit_msgs::RobotState robot_state;
    robot_state.joint_state = last_joint_state_;
    robot_state.multi_dof_joint_state.joint_names = { "robot_pose" };
    robot_state.multi_dof_joint_state.transforms.resize(1);
    robot_state.multi_dof_joint_state.transforms[0].translation.x = robot_pose_planning_frame.position.x;
    robot_state.multi_dof_joint_state.transforms[0].translation.y = robot_pose_planning_frame.position.y;
    robot_state.multi_dof_joint_state.transforms[0].translation.z = robot_pose_planning_frame.position.z;
    robot_state.multi_dof_joint_state.transforms[0].rotation.w = robot_pose_planning_frame.orientation.w;
    robot_state.multi_dof_joint_state.transforms[0].rotation.x = robot_pose_planning_frame.orientation.x;
    robot_state.multi_dof_joint_state.transforms[0].rotation.y = robot_pose_planning_frame.orientation.y;
    robot_state.multi_dof_joint_state.transforms[0].rotation.z = robot_pose_planning_frame.orientation.z;
    planning_scene_->robot_state = robot_state;

    planning_scene_->robot_model_name = robot_name_;
//    planning_scene_->robot_model_root = robot_local_frame_;

    planning_scene_->fixed_frame_transforms.clear();  // todo: are these necessary?
    planning_scene_->allowed_collision_matrix;        // todo: covered by sbpl_collision_model?
    planning_scene_->link_padding.clear();            // todo: covered by sbpl_collision_model?
    planning_scene_->link_scale.clear();              // todo: covered by sbpl_collision_model?
    planning_scene_->object_colors.clear();           // todo: also probably not used

    // Fill out header information for the collision map so that sbpl::manipulation doesn't get pissed off
    planning_scene_->world.collision_objects.clear();
    // planning_scene_->world.octomap = request->octomap; // todo: covered by sbpl_collision_model/occupancy grid?
    planning_scene_->world.octomap.header = std_msgs::CreateHeader(0, ros::Time(0), planning_frame);
//    planning_scene_->world.collision_map.header = std_msgs::CreateHeader(0, ros::Time(0), planning_frame);

    planning_scene_->is_diff = false;

    //////////////////////////////////////////////
    // Reinitialize for updated collision model //
    //////////////////////////////////////////////

    if (!reinit(T_kinematics_planning, planning_frame, request->octomap)) {
        rcta::MoveArmCommandResult result;
        result.success = false;
        m_move_arm_command_server->setAborted(result, "Failed to initialize Arm Planner");
        return;
    }

    /////////////////////////////////////
    // Plan to the received goal state //
    /////////////////////////////////////

    m_collision_checker->setPlanningScene(*planning_scene_);
      //////////////////////////////////////
      // Deal with attached object if any //
      //////////////////////////////////////
      if (request->has_attached_object){
        m_collision_checker->processAttachedCollisionObject(request->attached_object);
      }

    // transform the wrist goal from the kinematics frame to the planning frame
    Eigen::Affine3d T_kinematics_wrist_goal;
    tf::poseMsgToEigen(request->goal_pose, T_kinematics_wrist_goal);

    Eigen::Affine3d T_planning_kinematics(T_kinematics_planning.inverse());
    Eigen::Affine3d T_planning_wrist_goal(T_planning_kinematics * T_kinematics_wrist_goal);

    geometry_msgs::PoseStamped wrist_goal_planning_frame;
    wrist_goal_planning_frame.header.seq = 0;
    wrist_goal_planning_frame.header.stamp = ros::Time(0);
    wrist_goal_planning_frame.header.frame_id = planning_frame;
    tf::poseEigenToMsg(T_planning_wrist_goal, wrist_goal_planning_frame.pose);

    bool success = false;
    trajectory_msgs::JointTrajectory result_traj;
    if (request->type == rcta::MoveArmCommandGoal::JointGoal) {
        ROS_INFO("Received a joint goal");
        success = plan_to_joint_goal(planning_scene_, planning_scene_->robot_state, *request, result_traj);
    }
    else if (request->type == rcta::MoveArmCommandGoal::EndEffectorGoal) {
        ROS_INFO("Received an end effector goal");
        success = plan_to_eef_goal(planning_scene_, wrist_goal_planning_frame, result_traj);
    }

    /////////////////////////////////////////////////////////////////////
    // Post-process the plan and send to the joint trajectory follower //
    /////////////////////////////////////////////////////////////////////

    if (!success) {
        rcta::MoveArmCommandResult result;
        result.success = false;
        result.trajectory;
        m_move_arm_command_server->setAborted(result, "Failed to plan path");
        return;
    }

    ROS_INFO("Original joint path (%zd points):", result_traj.points.size());
    for (int i = 0; i < (int)result_traj.points.size(); ++i) {
        const trajectory_msgs::JointTrajectoryPoint& joint_state = result_traj.points[i];
        ROS_INFO("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
    }

    ROS_INFO("Shortcut trajectory (%zd points):", result_traj.points.size());
    for (int i = 0; i < (int)result_traj.points.size(); ++i) {
        const trajectory_msgs::JointTrajectoryPoint& joint_state = result_traj.points[i];
        ROS_INFO("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
    }

    bool interp_res = add_interpolation_to_plan(result_traj);
    if (!interp_res) {
        ROS_ERROR("Failed to interpolate joint trajectory");
        rcta::MoveArmCommandResult result;
        result.success = false;
        m_move_arm_command_server->setAborted(result, "Failed to interpolate joint trajectory");
        return;
    }

    if (request->execute_path) {
        publish_trajectory(result_traj);
    }

    rcta::MoveArmCommandResult result;
    result.success = true;
    result.trajectory = result_traj;
    m_move_arm_command_server->setSucceeded(result);
}

void ArmPlanningNode::fill_constraint(
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    moveit_msgs::Constraints& goals)
{
//    if(pose.size() < 6)
//        return;

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose.position.x;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose.position.y;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose.position.z;

    //  goals.position_constraints[0].position.x = pose[0];
    //  goals.position_constraints[0].position.y = pose[1];
    //  goals.position_constraints[0].position.z = pose[2];

//    leatherman::rpyToQuatMsg(pose[3], pose[4], pose[5], goals.orientation_constraints[0].orientation);

    goals.orientation_constraints[0].orientation.w = pose.orientation.w;
    goals.orientation_constraints[0].orientation.x = pose.orientation.x;
    goals.orientation_constraints[0].orientation.y = pose.orientation.y;
    goals.orientation_constraints[0].orientation.z = pose.orientation.z;

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
ArmPlanningNode::get_collision_cubes(
    const std::vector<std::vector<double>>& objects,
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

std::vector<moveit_msgs::CollisionObject>
ArmPlanningNode::get_collision_objects(
    const std::string& filename,
    const std::string& frame_id)
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

void ArmPlanningNode::joint_states_callback(
    const sensor_msgs::JointState::ConstPtr& msg)
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

bool ArmPlanningNode::add_interpolation_to_plan(
    trajectory_msgs::JointTrajectory& res_traj) const
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

void ArmPlanningNode::publish_trajectory(
    const trajectory_msgs::JointTrajectory& joint_trajectory)
{
    const std::vector<std::string> joint_names = {
            "arm_1_shoulder_twist",
            "arm_2_shoulder_lift",
            "arm_3_elbow_twist",
            "arm_4_elbow_lift",
            "arm_5_wrist_twist",
            "arm_6_wrist_lift",
            "arm_7_gripper_lift",
    };

    // map from planning joints to index in resulting message
    std::map<std::string, int> planning_joint_indices;
    for (int i = 0; i < (int)joint_trajectory.joint_names.size(); ++i) {
        const std::string& joint_name = joint_trajectory.joint_names[i];
        planning_joint_indices.insert({joint_name, i});
    }

    if (joint_trajectory.points.empty()) {
        ROS_INFO("Not publishing empty trajectory");
        return;
    }

    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = "";
    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();

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

    if (!m_use_action_server) {
        ROS_INFO("Publishing trajectory to the arm controller");
        m_joint_trajectory_pub.publish(traj);
    }
    else {
        ROS_INFO("Sending trajectory goal via actionlib");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = std::move(traj);
        m_follow_trajectory_client.sendGoal(goal);
        m_follow_trajectory_client.waitForResult();
        if (m_follow_trajectory_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Yay! The dishes are now clean");
        }
        ROS_INFO("Current State: %s", m_follow_trajectory_client.getState().toString().c_str());
    }
}

bool ArmPlanningNode::plan_to_eef_goal(
    const moveit_msgs::PlanningScenePtr& scene,
    const geometry_msgs::PoseStamped& goal_pose,
    trajectory_msgs::JointTrajectory& traj)
{
    // clamp planning joints in robot start state
    for (size_t i = 0; i < robot_model_->joint_names().size(); ++i) {
        const std::string& planning_joint = robot_model_->joint_names()[i];
        size_t joint_idx = i;
        for (size_t j = 0; j < scene->robot_state.joint_state.name.size(); ++j) {
           const std::string& joint_name = scene->robot_state.joint_state.name[j];
           if (joint_name == planning_joint) {
              ROS_WARN("Clamping joint '%s' to [%0.3f, %0.3f]", joint_name.c_str(), robot_model_->min_safety_limits()[joint_idx], robot_model_->max_safety_limits()[joint_idx]);
              scene->robot_state.joint_state.position[j] = clamp(scene->robot_state.joint_state.position[j], robot_model_->min_safety_limits()[joint_idx], robot_model_->max_safety_limits()[joint_idx]);
              ROS_WARN("  %0.3f", scene->robot_state.joint_state.position[j]);
           }
        }
    }

    // fill goal state
    moveit_msgs::MotionPlanRequest req;
    req.goal_constraints.resize(1);
    ROS_WARN("Converting goal pose '%s' to 6dof sbpl goal", to_string(goal_pose.pose).c_str());
    std::vector<double> goal_vector = convert_to_sbpl_goal(goal_pose.pose);
    fill_constraint(goal_pose.pose, goal_pose.header.frame_id, req.goal_constraints[0]);
    ROS_WARN("Created a goal in the '%s' frame", req.goal_constraints.front().position_constraints[0].header.frame_id.c_str());
    req.allowed_planning_time = 10.0; //2.0;
    req.start_state = scene->robot_state;

    // plan
    ROS_INFO("Calling solve...");
    moveit_msgs::MotionPlanResponse res;
    bool plan_result = m_planner->solve(scene, req, res);
    if (!plan_result) {
        ROS_ERROR("Failed to plan.");
    }
    else {
        ROS_INFO("Planning succeeded");
    }

    // print statistics
    std::map<std::string, double> planning_stats = m_planner->getPlannerStats();

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
    m_marker_array_pub.publish(m_collision_checker->getVisualization("bounds"));
    auto distance_field_markers = m_collision_checker->getVisualization("distance_field");
    ROS_INFO("Distance Field Visualization contains %zd markers", distance_field_markers.markers.front().points.size());
    m_marker_array_pub.publish(distance_field_markers);
    m_marker_array_pub.publish(m_collision_checker->getVisualization("collision_objects"));
    m_marker_array_pub.publish(m_planner->getVisualization("goal"));

    // visualize, filter, and publish plan
    if (plan_result) {
        m_marker_array_pub.publish(m_planner->getCollisionModelTrajectoryMarker());
        traj = res.trajectory.joint_trajectory;
    }

    return plan_result;
}

bool ArmPlanningNode::plan_to_joint_goal(
    const moveit_msgs::PlanningScenePtr& scene,
    const moveit_msgs::RobotState& start,
    const rcta::MoveArmCommandGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{
    // clamp planning joints in robot start state
    for (size_t i = 0; i < robot_model_->joint_names().size(); ++i) {
        const std::string& planning_joint = robot_model_->joint_names()[i];
        size_t joint_idx = i;
        for (size_t j = 0; j < scene->robot_state.joint_state.name.size(); ++j) {
           const std::string& joint_name = scene->robot_state.joint_state.name[j];
           if (joint_name == planning_joint) {
              ROS_WARN("Clamping joint '%s' to [%0.3f, %0.3f]", joint_name.c_str(), robot_model_->min_safety_limits()[joint_idx], robot_model_->max_safety_limits()[joint_idx]);
              scene->robot_state.joint_state.position[j] = clamp(scene->robot_state.joint_state.position[j], robot_model_->min_safety_limits()[joint_idx], robot_model_->max_safety_limits()[joint_idx]);
              ROS_WARN("  %0.3f", scene->robot_state.joint_state.position[j]);
           }
        }
    }

    sensor_msgs::JointState goal_joint_state = goal.goal_joint_state;
    if (!msg_utils::reorder_joints(goal_joint_state, robot_model_->joint_names())) {
        ROS_WARN("Goal state contains joints other than manipulator joints");
        return false;
    }

    // fill goal state
    moveit_msgs::MotionPlanRequest req;
    req.goal_constraints.resize(1);

    std::vector<double> goal_vector = goal_joint_state.position;
    std::vector<double> goal_tolerances (7, 0.05);

    req.goal_constraints.resize(1);
    req.goal_constraints.front().joint_constraints.resize(goal_vector.size());
    for(int i = 0; i < goal_vector.size(); i++){
      req.goal_constraints.front().joint_constraints[i].position = goal_joint_state.position[i];
      req.goal_constraints.front().joint_constraints[i].joint_name = goal_joint_state.name[i];
      req.goal_constraints.front().joint_constraints[i].tolerance_above = 3.0 * M_PI / 180.0;
      req.goal_constraints.front().joint_constraints[i].tolerance_below = -3.0 * M_PI / 180.0;
      ROS_INFO("%s: %.3f tol(%.3f, %.3f)", goal_joint_state.name[i].c_str(), goal_joint_state.position[i], req.goal_constraints.front().joint_constraints[i].tolerance_below, req.goal_constraints.front().joint_constraints[i].tolerance_above);
    }

    ROS_WARN("Created a 7DoF goal!");

    req.allowed_planning_time = 10.0; //2.0;
    req.start_state = scene->robot_state;

    // plan
    ROS_INFO("Calling solve...");
    moveit_msgs::MotionPlanResponse res;
    bool plan_result = m_planner->solve(scene, req, res);
    if (!plan_result) {
        ROS_ERROR("Failed to plan.");
    }
    else {
        ROS_INFO("Planning succeeded");
    }

    // print statistics
    std::map<std::string, double> planning_stats = m_planner->getPlannerStats();

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
    m_marker_array_pub.publish(m_collision_checker->getVisualization("bounds"));
    auto distance_field_markers = m_collision_checker->getVisualization("distance_field");
    ROS_INFO("Distance Field Visualization contains %zd markers", distance_field_markers.markers.front().points.size());
    m_marker_array_pub.publish(distance_field_markers);    m_marker_array_pub.publish(m_collision_checker->getVisualization("collision_objects"));
    m_marker_array_pub.publish(m_planner->getVisualization("goal"));

    // visualize, filter, and publish plan
    if (plan_result) {
        m_marker_array_pub.publish(m_planner->getCollisionModelTrajectoryMarker());
        traj = res.trajectory.joint_trajectory;
    }

    return plan_result;
}

std::vector<double> ArmPlanningNode::convert_to_sbpl_goal(
    const geometry_msgs::Pose& pose)
{
    tf::Quaternion goal_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 goal_rotation_matrix(goal_quat);
    double goal_roll, goal_pitch, goal_yaw;
    goal_rotation_matrix.getEulerYPR(goal_yaw, goal_pitch, goal_roll);
    return { pose.position.x, pose.position.y, pose.position.z, goal_roll, goal_pitch, goal_yaw };
}

void ArmPlanningNode::clamp_to_joint_limits(
    std::vector<double>& joint_vector,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits)
{
    auto clamp = [](double d, double min, double max) { if (d < min) return min; else if (d > max) return max; else return d; };
    for (std::size_t i = 0; i < joint_vector.size(); ++i) {
        joint_vector[i] = clamp(joint_vector[i], robot_model_->min_limits()[i], robot_model_->max_limits()[i]);
    }
}

} //namespace hdt

