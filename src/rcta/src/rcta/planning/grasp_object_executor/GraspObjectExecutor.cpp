#include "GraspObjectExecutor.h"

// standard includes
#include <cmath>

// system includes
#include <Eigen/Dense>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <sbpl_geometry_utils/utils.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/random/gaussian.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/RunUponDestruction.h>
#include <spellbook/utils/utils.h>

// project includes
#include <rcta/control/robotiq_controllers/gripper_model.h>
#include <rcta/common/comms/actionlib.h>

/// Create a collision object representing the gascan.
///
/// The gascan is represented by a simple box, bounding the body and handle of
/// the gascan, and a cylinder, bounding the nozzle. This function only fills
/// out the primitives and poses in the frame of the gascan. It does not fill
/// out the message header, id, or collision object operation.
moveit_msgs::CollisionObject CreateGascanCollisionObject()
{
    moveit_msgs::CollisionObject co;

    double body_part_x = 0.0;           // 0.0
    double body_part_y = 0.01303;       // 0.0
    double body_part_z = -0.01241;      // -0.05
    double body_part_qw = 1.0;          // 1.0
    double body_part_qx = 0.0001895;    // 0.0
    double body_part_qy = -0.009;       // 0.0
    double body_part_qz = 0.021;        // 0.0
    double body_part_dim_x = 0.212;     // 0.20
    double body_part_dim_y = 0.2; //0.296;     // 0.25
    double body_part_dim_z = 0.296;     // 0.25

    double noz_part_x = 0.00816;            // 0.0
    double noz_part_y = -0.15700;           // 0.0
    double noz_part_z = 0.15571;            // 0.15
    double noz_part_qw = 0.932;             // (45 degs about x)
    double noz_part_qx = 0.354;             // ^
    double noz_part_qy = 0.028;             // ^
    double noz_part_qz = -0.072;            // ^
    double noz_part_radius = 0.5 * 0.052;   // 0.035
    double noz_part_height = 0.183;         // 0.35

    shape_msgs::SolidPrimitive body_part;
    body_part.type = shape_msgs::SolidPrimitive::BOX;
    body_part.dimensions.resize(3);
    body_part.dimensions[shape_msgs::SolidPrimitive::BOX_X] = body_part_dim_x;
    body_part.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = body_part_dim_y;
    body_part.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = body_part_dim_z;

    Eigen::Affine3d body_part_offset =
            Eigen::Translation3d(body_part_x, body_part_y, body_part_z) *
            Eigen::Quaterniond(body_part_qw, body_part_qx, body_part_qy, body_part_qz).normalized();
    Eigen::Affine3d body_pose_in_grasp_frame = body_part_offset;
    geometry_msgs::Pose body_part_pose;
    tf::poseEigenToMsg(body_pose_in_grasp_frame, body_part_pose);

    shape_msgs::SolidPrimitive nozzle_part;
    nozzle_part.type = shape_msgs::SolidPrimitive::CYLINDER;
    nozzle_part.dimensions.resize(2);
    nozzle_part.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = noz_part_height;
    nozzle_part.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = noz_part_radius;

    Eigen::Affine3d nozzle_pose_offset =
            Eigen::Translation3d(noz_part_x, noz_part_y, noz_part_z) *
            Eigen::Quaterniond(noz_part_qw, noz_part_qx, noz_part_qy, noz_part_qz).normalized();
    Eigen::Affine3d nozzle_pose_in_grasp_frame = nozzle_pose_offset;
    geometry_msgs::Pose nozzle_part_pose;
    tf::poseEigenToMsg(nozzle_pose_in_grasp_frame, nozzle_part_pose);

    co.primitives.reserve(2);
    co.primitives.push_back(body_part);
    co.primitive_poses.push_back(body_part_pose);
    co.primitives.push_back(nozzle_part);
    co.primitive_poses.push_back(nozzle_part_pose);

    return co;
}

namespace GraspObjectExecutionStatus
{

std::string to_string(Status status)
{
    switch (status) {
    case IDLE:
        return "Idle";
    case FAULT:
        return "Fault";
    case GENERATING_GRASPS:
        return "GeneratingGrasps";
    case MOVING_ARM_TO_PREGRASP:
        return "MovingArmToPregrasp";
    case MOVING_ARM_TO_GRASP:
        return "MovingArmToGrasp";
    case OPENING_GRIPPER:
        return "OpeningGripper";
    case EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return "ExecutingVisualServoMotionToPregrasp";
    case EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return "ExecutingVisualServoMotionToGrasp";
    case GRASPING_OBJECT:
        return "GraspingObject";
    case RETRACTING_GRIPPER:
        return "RetractingGripper";
    case MOVING_ARM_TO_STOW:
        return "MovingArmToStow";
    case COMPLETING_GOAL:
        return "CompletingGoal";
    default:
        return "Invalid";
    }
}

} // namespace GraspObjectExecutionStatus

bool extract_xml_value(XmlRpc::XmlRpcValue& value, StowPosition& stow_position)
{
    ROS_INFO("Extract stow position");

    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Stow position config must be a struct");
        return false;
    }

    if (!value.hasMember("name") || !value.hasMember("joint_vector_degs")) {
        ROS_ERROR("Stow position config must have members 'name' and 'joint_vector_degs'");
        return false;
    }

    std::string name;
    if (!msg_utils::extract_xml_value(value["name"], name)) {
        ROS_ERROR("Failed to extract 'name' field");
        return false;
    }

    std::map<std::string, double> joint_positions;
    if (!msg_utils::extract_xml_value(value["joint_vector_degs"], joint_positions)) {
        ROS_ERROR("Failed to extract 'joint_vector_degs' field");
        return false;
    }

    stow_position.name = std::move(name);
    stow_position.joint_positions = std::move(joint_positions);
    return true;
}

GraspObjectExecutor::GraspObjectExecutor() :
    nh_(),
    ph_("~"),
    filtered_costmap_pub_(),
    extrusion_octomap_pub_(),
    attach_obj_pub_(),
    costmap_sub_(),
    listener_(),
    as_(),
    move_arm_command_client_(),
    viservo_command_client_(),
    gripper_command_client_(),
    action_name_("grasp_object_command"),
    move_arm_command_action_name_("move_arm"),
    viservo_command_action_name_("viservo_command"),
    gripper_command_action_name_("right_gripper/gripper_action"),
    m_viz(),
    m_rml(),
    m_robot_model(),
    extruder_(100, false),
    m_grasp_planner(),
    m_scene_monitor(),
    m_gripper_links(),
    m_camera_view_frame("camera_rgb_optical_frame"),
    m_manip_name(),
    m_manip_group(nullptr),
    object_filter_radius_m_(),
    use_extrusion_octomap_(false),
    skip_viservo_(false),
    attached_markers_(),
    max_grasp_candidates_(0),
    stow_positions_(),
    attach_obj_req_wait_(),
    gas_can_detection_threshold_(0.0),
    current_goal_(),
    current_occupancy_grid_(),
    current_octomap_(),
    gas_can_in_grid_frame_(),
    sent_move_arm_goal_(false),
    pending_move_arm_command_(false),
    move_arm_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    move_arm_command_result_(),
    sent_viservo_command_(false),
    pending_viservo_command_(false),
    viservo_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    viservo_command_result_(),
    sent_gripper_command_(false),
    pending_gripper_command_(false),
    gripper_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    gripper_command_result_(),
    last_occupancy_grid_(),
    reachable_grasp_candidates_(),
    last_move_arm_pregrasp_goal_(),
    last_successful_grasp_(),
    last_viservo_pregrasp_goal_(),
    next_stow_position_to_attempt_(-1),
    attach_obj_req_time_(),
    wait_for_grid_start_time_()
{
    sbpl::viz::set_visualizer(&m_viz);
}

GraspObjectExecutor::~GraspObjectExecutor()
{
    if (sbpl::viz::visualizer() == &m_viz) {
        sbpl::viz::unset_visualizer();
    }
}

bool GraspObjectExecutor::initialize()
{
    ///////////////////////////////////////////////////////////////////////
    // Copied from RepositionBaseExecutor to set up state monitoring and //
    // some other interesting stuff                                      //
    ///////////////////////////////////////////////////////////////////////

    m_rml.reset(new robot_model_loader::RobotModelLoader);
    m_robot_model = m_rml->getModel();
    if (!m_robot_model) {
        ROS_ERROR("Failed to load Robot Model");
        return false;
    }

    if (!m_robot_model->hasLinkModel(m_camera_view_frame)) {
        ROS_ERROR("No link '%s' found in the robot model", m_camera_view_frame.c_str());
        return false;
    }

    auto transformer = boost::shared_ptr<tf::Transformer>(new tf::TransformListener);
    m_scene_monitor = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(m_rml, transformer);
    auto update_fn = boost::bind(&GraspObjectExecutor::processSceneUpdate, this, _1);
    m_scene_monitor->addUpdateCallback(update_fn);
    m_scene_monitor->startStateMonitor();

    if (!msg_utils::download_param(ph_, "manipulator_group_name", m_manip_name)) {
        return false;
    }

    if (!m_robot_model->hasJointModelGroup(m_manip_name)) {
        ROS_ERROR("Robot '%s' has no group named '%s'", m_robot_model->getName().c_str(), m_manip_name.c_str());
        return false;
    }
    m_manip_group = m_robot_model->getJointModelGroup(m_manip_name);

    const auto& tip_frames = m_manip_group->getSolverInstance()->getTipFrames();
    ROS_INFO("'%s' group tip frames:", m_manip_name.c_str());
    for (const auto& tip_frame : tip_frames) {
        ROS_INFO("  %s", tip_frame.c_str());
    }

    ros::NodeHandle grasp_nh(ph_, "grasping");
    if (!m_grasp_planner.init(grasp_nh)) {
        ROS_ERROR("Failed to initialize Grasp Planner");
        return false;
    }

    // read in max grasps
    if (!msg_utils::download_param(ph_, "max_grasp_candidates", max_grasp_candidates_) ||
        max_grasp_candidates_ < 0)
    {
        ROS_ERROR("Failed to retrieve 'max_grasp_candidates' from the param server or 'max_grasp_candidates' is negative");
        return false;
    }

    ////////////////////////////////////////
    // GraspObjectExecutor-specific stuff //
    ////////////////////////////////////////

    // read in stow positions
    if (!msg_utils::download_param(ph_, "stow_positions", stow_positions_)) {
        ROS_ERROR("Failed to retrieve 'stow_positions' from the param server");
        return false;
    }

    ROS_INFO("Stow Positions:");
    for (StowPosition& position : stow_positions_) {
        ROS_INFO("  %s:", position.name.c_str());
        for (auto& entry : position.joint_positions) {
            ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
            entry.second = entry.second * M_PI / 180.0;
        }
    }

    if (!msg_utils::download_param(ph_, "use_extrusion_octomap", use_extrusion_octomap_)) {
        ROS_ERROR("Failed to retrieve 'use_extrusion_octomap' from the param server");
        return false;
    }

    if (!msg_utils::download_param(ph_, "skip_viservo", skip_viservo_)) {
        ROS_ERROR("Failed to retrieve 'skip_viservo' from the param server");
        return false;
    }

    if (!msg_utils::download_param(ph_, "object_filter_radius_m", object_filter_radius_m_)) {
        ROS_ERROR("Failed to retrieve 'object_filter_radius_m' from the param server");
        return false;
    }

    if (!msg_utils::download_param(ph_, "gas_can_detection_threshold", gas_can_detection_threshold_)) {
        return false;
    }

    if (!downloadMarkerParams()) {
        return false; // errors printed within
    }

    // subscribers
    costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
            "map", 1, &GraspObjectExecutor::occupancyGridCallback, this);

    // publishers
    filtered_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap_filtered", 1);
    extrusion_octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("extrusion_octomap", 1);
    attach_obj_pub_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(
            "attached_collision_object", 1);

    move_arm_command_client_.reset(new MoveArmActionClient(move_arm_command_action_name_, false));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Client");
        return false;
    }

    viservo_command_client_.reset(new ViservoCommandActionClient(viservo_command_action_name_, false));
    if (!viservo_command_client_) {
        ROS_ERROR("Failed to instantiate Viservo Command Client");
        return false;
    }

    gripper_command_client_.reset(new GripperCommandActionClient(gripper_command_action_name_, false));
    if (!gripper_command_client_) {
        ROS_ERROR("Failed to instantiate Gripper Command Client");
        return false;
    }

    as_.reset(new GraspObjectCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Grasp Object Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&GraspObjectExecutor::goalCallback, this));
    as_->registerPreemptCallback(boost::bind(&GraspObjectExecutor::preemptCallback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    return true;
}

int GraspObjectExecutor::run()
{
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

    GraspObjectExecutionStatus::Status prev_status = GraspObjectExecutionStatus::IDLE;
    GraspObjectExecutionStatus::Status status = GraspObjectExecutionStatus::IDLE;
    GraspObjectExecutionStatus::Status next_status = GraspObjectExecutionStatus::IDLE;

    ros::Rate loop_rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();

        if (status != prev_status) {
            ROS_INFO("Grasp Job Executor Transitioning: %s -> %s", to_string(prev_status).c_str(), to_string(status).c_str());
            switch (status) {
            case GraspObjectExecutionStatus::IDLE:
                onIdleExit(prev_status);
                break;
            case GraspObjectExecutionStatus::FAULT:
                onFaultEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::GENERATING_GRASPS:
                onGeneratingGraspsEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP:
                onMovingArmToPregraspEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP:
                onMovingArmToGraspEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::OPENING_GRIPPER:
                onOpeningGripperEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
                onExecutingVisualServoMotionToPregraspEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
                onExecutingVisualServoMotionToGraspEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::GRASPING_OBJECT:
                onGraspingObjectEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::RETRACTING_GRIPPER:
                onRetractingGripperEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::MOVING_ARM_TO_STOW:
                onMovingArmToStowEnter(prev_status);
                break;
            case GraspObjectExecutionStatus::COMPLETING_GOAL:
                onCompletingGoalEnter(prev_status);
                break;
            default:
                break;
            }

            prev_status = status;
        }

        // publish feedback status for active goals
        if (as_->isActive()) {
            rcta_msgs::GraspObjectCommandFeedback feedback;
            feedback.status = executionStatusToFeedbackStatus(status);
            if (feedback.status != 255) {
                as_->publishFeedback(feedback);
            }
        }

        switch (status) {
        case GraspObjectExecutionStatus::IDLE:
            next_status = onIdle();
            break;
        case GraspObjectExecutionStatus::FAULT:
            next_status = onFault();
            break;
        case GraspObjectExecutionStatus::GENERATING_GRASPS:
            next_status = onGeneratingGrasps();
            break;
        case GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP:
            next_status = onMovingArmToPregrasp();
            break;
        case GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP:
            next_status = onMovingArmToGrasp();
            break;
        case GraspObjectExecutionStatus::OPENING_GRIPPER:
            next_status = onOpeningGripper();
            break;
        case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
            next_status = onExecutingVisualServoMotionToPregrasp();
            break;
        case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
            next_status = onExecutingVisualServoMotionToGrasp();
            break;
        case GraspObjectExecutionStatus::GRASPING_OBJECT:
            next_status = onGraspingObject();
            break;
        case GraspObjectExecutionStatus::RETRACTING_GRIPPER:
            next_status = onRetractingGripper();
            break;
        case GraspObjectExecutionStatus::MOVING_ARM_TO_STOW:
            next_status = onMovingArmToStow();
            break;
        case GraspObjectExecutionStatus::COMPLETING_GOAL:
            next_status = onCompletingGoal();
            break;
        default:
            break;
        }

        if (next_status != status) {
            switch (status) {
            case GraspObjectExecutionStatus::IDLE:
                onIdleExit(next_status);
                break;
            case GraspObjectExecutionStatus::FAULT:
                onFaultExit(next_status);
                break;
            case GraspObjectExecutionStatus::GENERATING_GRASPS:
                onGeneratingGraspsExit(next_status);
                break;
            case GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP:
                onMovingArmToPregraspExit(next_status);
                break;
            case GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP:
                onMovingArmToGraspExit(next_status);
                break;
            case GraspObjectExecutionStatus::OPENING_GRIPPER:
                onOpeningGripperExit(next_status);
                break;
            case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
                onExecutingVisualServoMotionToPregraspExit(next_status);
                break;
            case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
                onExecutingVisualServoMotionToGraspExit(next_status);
                break;
            case GraspObjectExecutionStatus::GRASPING_OBJECT:
                onGraspingObjectExit(next_status);
                break;
            case GraspObjectExecutionStatus::RETRACTING_GRIPPER:
                onRetractingGripperExit(next_status);
                break;
            case GraspObjectExecutionStatus::MOVING_ARM_TO_STOW:
                onMovingArmToStowExit(next_status);
                break;
            case GraspObjectExecutionStatus::COMPLETING_GOAL:
                onCompletingGoalExit(next_status);
                break;
            default:
                break;
            }

            status = next_status;
        }

        loop_rate.sleep();
    }

    return SUCCESS;
}

void GraspObjectExecutor::goalCallback()
{
    ROS_INFO("Received a new goal");
    current_goal_ = as_->acceptNewGoal();
    ROS_INFO("  Goal ID: %u", current_goal_->id);
    ROS_INFO("  Retry Count: %d", current_goal_->retry_count);
    ROS_INFO("  Gas Can Pose [world frame: %s]: %s", current_goal_->gas_can_in_map.header.frame_id.c_str(), to_string(current_goal_->gas_can_in_map.pose).c_str());
    ROS_INFO("  Gas Can Pose [robot frame: %s]: %s", current_goal_->gas_can_in_base_link.header.frame_id.c_str(), to_string(current_goal_->gas_can_in_base_link.pose).c_str());
    ROS_INFO("  Octomap ID: %s", current_goal_->octomap.id.c_str());

    next_stow_position_to_attempt_ = 0;

    // make a hard copy of the grid
    if (last_occupancy_grid_) {
        current_occupancy_grid_.reset(new nav_msgs::OccupancyGrid);
        *current_occupancy_grid_ = *last_occupancy_grid_;
    }

    // get the gas can pose in the frame of the occupancy grid
    if (current_occupancy_grid_) {
        const std::string& grid_frame = current_occupancy_grid_->header.frame_id;
        const std::string& world_frame = current_goal_->gas_can_in_map.header.frame_id;

        // clear gas_can_in_grid_frame_
        gas_can_in_grid_frame_.header.frame_id = "";
        gas_can_in_grid_frame_.header.stamp = ros::Time(0);
        gas_can_in_grid_frame_.header.seq = 0;
        gas_can_in_grid_frame_.pose = geometry_msgs::IdentityPose();

        if (grid_frame != world_frame) {
            ROS_INFO("Transforming gas can into frame '%s' to clear from the occupancy grid", grid_frame.c_str());
            try {
                geometry_msgs::PoseStamped gas_can_in_world_frame = current_goal_->gas_can_in_map;
                gas_can_in_world_frame.header.stamp = ros::Time(0);

                gas_can_in_grid_frame_.header.frame_id = current_occupancy_grid_->header.frame_id;
                gas_can_in_grid_frame_.header.stamp = ros::Time(0);

                listener_.transformPose(grid_frame, gas_can_in_world_frame, gas_can_in_grid_frame_);
            }
            catch (const tf::TransformException& ex) {
                ROS_ERROR("Failed to lookup transform gas can into the grid frame. Assuming Identity");
                ROS_ERROR("%s", ex.what());
                gas_can_in_grid_frame_ = current_goal_->gas_can_in_map;
            }
        } else {
            ROS_INFO("Gas can already exists in grid frame '%s'", grid_frame.c_str());
            gas_can_in_grid_frame_ = current_goal_->gas_can_in_map;
            gas_can_in_grid_frame_.header.stamp = ros::Time(0);
        }

        ROS_INFO("Gas Can Pose [grid frame: %s]: %s", grid_frame.c_str(), to_string(gas_can_in_grid_frame_.pose).c_str());

        // todo: save all the cells that didn't need to be cleared, so that we can check for their existence later

        clearCircleFromGrid(
                *current_occupancy_grid_,
                gas_can_in_grid_frame_.pose.position.x,
                gas_can_in_grid_frame_.pose.position.y,
                object_filter_radius_m_);

        filtered_costmap_pub_.publish(current_occupancy_grid_);
    }

    if (use_extrusion_octomap_ && current_occupancy_grid_) {
        const double HARDCODED_EXTRUSION = 2.0; // Extrude the occupancy grid from 0m to 2m
        extruder_.extrude(*current_occupancy_grid_, HARDCODED_EXTRUSION, *current_octomap_);
        if (current_octomap_) {
            extrusion_octomap_pub_.publish(current_octomap_);
        }
    }
}

void GraspObjectExecutor::preemptCallback()
{

}

void GraspObjectExecutor::onIdleEnter(GraspObjectExecutionStatus::Status from)
{

}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onIdle()
{
    if (as_->isActive()) {
        if (use_extrusion_octomap_ &&
            (!current_occupancy_grid_ || !current_octomap_))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            std::string msg = (bool)current_occupancy_grid_ ?
                    "Failed to extrude Occupancy Grid" : "Have yet to receive Occupancy Grid";
            ROS_WARN("%s", msg.c_str());
            as_->setAborted(result, msg.c_str());
            return GraspObjectExecutionStatus::FAULT;
        } else {
            return GraspObjectExecutionStatus::GENERATING_GRASPS;
        }
    }

    return GraspObjectExecutionStatus::IDLE;
}

void GraspObjectExecutor::onIdleExit(GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onFaultEnter(GraspObjectExecutionStatus::Status from)
{

}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onFault()
{
    if (as_->isActive()) {
        if (use_extrusion_octomap_ &&
            (!current_occupancy_grid_ || !current_octomap_))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            std::string msg = (bool)current_occupancy_grid_ ?
                    "Failed to extrude Occupancy Grid" : "Have yet to receive Occupancy Grid";
            ROS_WARN("%s", msg.c_str());
            as_->setAborted(result, msg.c_str());
            return GraspObjectExecutionStatus::FAULT;
        } else {
            return GraspObjectExecutionStatus::GENERATING_GRASPS;
        }
    }

    return GraspObjectExecutionStatus::FAULT;
}

void GraspObjectExecutor::onFaultExit(GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onGeneratingGraspsEnter(
    GraspObjectExecutionStatus::Status from)
{

}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onGeneratingGrasps()
{
    Eigen::Affine3d object_pose;
    tf::poseMsgToEigen(current_goal_->gas_can_in_map.pose, object_pose);
    std::vector<rcta::GraspCandidate> candidates;

    // 1. generate grasp candidates (poses of the wrist in the robot frame) from the object pose
    int max_samples = 100;
    if (!m_grasp_planner.sampleGrasps(object_pose, max_samples, candidates)) {
        ROS_ERROR("Failed to sample grasps");
        return GraspObjectExecutionStatus::FAULT;
    }

    ROS_INFO("Sampled %zd grasp poses", candidates.size());
    SV_SHOW_INFO(getGraspCandidatesVisualization(candidates, "candidate_grasp"));

    moveit::core::RobotState robot_state = currentRobotState();
    const Eigen::Affine3d& camera_pose =
            robot_state.getGlobalLinkTransform(m_camera_view_frame);

    ROS_INFO("world -> camera: %s", to_string(camera_pose).c_str());

    const double vis_angle_thresh = sbpl::utils::ToRadians(45.0);
    pruneGraspCandidates(candidates, robot_state.getGlobalLinkTransform(m_robot_model->getRootLink()), camera_pose, vis_angle_thresh);

    ROS_INFO("Produced %zd reachable grasp poses", candidates.size());

    rcta::RankGrasps(candidates);

    std::swap(candidates, reachable_grasp_candidates_);

    if (reachable_grasp_candidates_.empty()) {
        ROS_WARN("No reachable grasp candidates available");
        rcta_msgs::GraspObjectCommandResult result;
        result.result = rcta_msgs::GraspObjectCommandResult::OBJECT_OUT_OF_REACH;
        as_->setAborted(result, "No reachable grasp candidates available");
        return GraspObjectExecutionStatus::FAULT;
    }


    ROS_INFO("Attempting %zd grasps", reachable_grasp_candidates_.size());
    SV_SHOW_INFO(getGraspCandidatesVisualization(reachable_grasp_candidates_, "reachable_candidates"));

    return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP;
}

void GraspObjectExecutor::onGeneratingGraspsExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onMovingArmToPregraspEnter(
    GraspObjectExecutionStatus::Status from)
{
    // limit the number of grasp attempts by the configured amount
    limitGraspCandidates(reachable_grasp_candidates_, max_grasp_candidates_);
    sent_move_arm_goal_ = false;
    pending_move_arm_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onMovingArmToPregrasp()
{
    if (!sent_move_arm_goal_) {
        if (reachable_grasp_candidates_.empty()) {
            ROS_WARN("Failed to plan to all reachable grasps");
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            as_->setAborted(result, "Failed on all reachable grasps");
            return GraspObjectExecutionStatus::FAULT;
        }

        ROS_WARN("Sending Move Arm Goal to pregrasp pose");
        if (!rcta::ReconnectActionClient(
                move_arm_command_client_,
                move_arm_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            std::stringstream ss; ss << "Failed to connect to '" << move_arm_command_action_name_ << "' action server";
            ROS_ERROR("%s", ss.str().c_str());
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            as_->setAborted(result, ss.str());
            return GraspObjectExecutionStatus::FAULT;
        }

        const rcta::GraspCandidate& next_best_grasp = reachable_grasp_candidates_.back();

        // 4. send a move arm goal for the best grasp
        last_move_arm_pregrasp_goal_.type = rcta::MoveArmGoal::EndEffectorGoal;
        tf::poseEigenToMsg(next_best_grasp.pose, last_move_arm_pregrasp_goal_.goal_pose);

        last_move_arm_pregrasp_goal_.octomap = use_extrusion_octomap_ ?
                *current_octomap_ : current_goal_->octomap;

        last_move_arm_pregrasp_goal_.execute_path = true;

        auto result_cb = boost::bind(&GraspObjectExecutor::moveArmResultCallback, this, _1, _2);
        move_arm_command_client_->sendGoal(last_move_arm_pregrasp_goal_, result_cb);

        sent_move_arm_goal_ = true;
        pending_move_arm_command_ = true;

        reachable_grasp_candidates_.pop_back();
        last_successful_grasp_ = next_best_grasp;
    } else if (!pending_move_arm_command_) {
        ROS_INFO("Move Arm Goal is no longer pending");
        if (move_arm_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            move_arm_command_result_ && move_arm_command_result_->success)
        {
            ROS_INFO("Move Arm Command succeeded");
            return GraspObjectExecutionStatus::OPENING_GRIPPER;
        } else {
            ROS_INFO("Move Arm Command failed");
            ROS_INFO("    Simple Client Goal State: %s", move_arm_command_goal_state_.toString().c_str());
            ROS_INFO("    Error Text: %s", move_arm_command_goal_state_.getText().c_str());
            ROS_INFO("    result.success = %s", move_arm_command_result_ ? (move_arm_command_result_->success ? "TRUE" : "FALSE") : "null");

            sent_move_arm_goal_ = false;
            pending_move_arm_command_ = false;
            // stay in MOVING_ARM_TO_PREGRASP until there are no more grasps
            // TODO: consider moving back to the stow position
        }
    }

    return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP;
}

void GraspObjectExecutor::onMovingArmToPregraspExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onMovingArmToGraspEnter(GraspObjectExecutionStatus::Status from)
{
    sent_move_arm_goal_ = false;
    pending_move_arm_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onMovingArmToGrasp()
{
    if (!sent_move_arm_goal_) {
        ROS_WARN("Sending Move Arm Goal to pregrasp pose");
        if (!rcta::ReconnectActionClient(
                move_arm_command_client_,
                move_arm_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            std::stringstream ss; ss << "Failed to connect to '" << move_arm_command_action_name_ << "' action server";
            ROS_ERROR("%s", ss.str().c_str());
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            as_->setAborted(result, ss.str());
            return GraspObjectExecutionStatus::FAULT;
        }

        rcta::MoveArmGoal grasp_goal;
        grasp_goal.type = rcta::MoveArmGoal::EndEffectorGoal;

        // compute goal pose for grasping from pregrasp pose
        Eigen::Affine3d pregrasp_pose; // model -> wrist (pregrasp)
        tf::poseMsgToEigen(last_move_arm_pregrasp_goal_.goal_pose, pregrasp_pose);
        Eigen::Affine3d grasp_pose; // model -> wrist (grasp)
        grasp_pose = pregrasp_pose * m_grasp_planner.graspToPregrasp().inverse();
        tf::poseEigenToMsg(grasp_pose, grasp_goal.goal_pose);

        if (use_extrusion_octomap_) {
            grasp_goal.octomap = *current_octomap_;
        } else {
            grasp_goal.octomap = current_goal_->octomap;
        }

        grasp_goal.execute_path = true;

        auto result_cb = boost::bind(&GraspObjectExecutor::moveArmResultCallback, this, _1, _2);
        move_arm_command_client_->sendGoal(grasp_goal, result_cb);

        sent_move_arm_goal_ = true;
        pending_move_arm_command_ = true;
    } else if (!pending_move_arm_command_) {
        ROS_INFO("Move Arm Goal is no longer pending");
        if (move_arm_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            move_arm_command_result_ && move_arm_command_result_->success)
        {
            ROS_INFO("Move Arm Command succeeded");
            return GraspObjectExecutionStatus::GRASPING_OBJECT;
        } else {
            ROS_INFO("Move Arm Command failed");
            ROS_INFO("    Simple Client Goal State: %s", move_arm_command_goal_state_.toString().c_str());
            ROS_INFO("    Error Text: %s", move_arm_command_goal_state_.getText().c_str());
            ROS_INFO("    result.success = %s", move_arm_command_result_ ? (move_arm_command_result_->success ? "TRUE" : "FALSE") : "null");
            return GraspObjectExecutionStatus::FAULT;
        }
    }

    return GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP;
}

void GraspObjectExecutor::onMovingArmToGraspExit(GraspObjectExecutionStatus::Status to)
{
}

void GraspObjectExecutor::onOpeningGripperEnter(
    GraspObjectExecutionStatus::Status from)
{
    sent_gripper_command_ = false;
    pending_gripper_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onOpeningGripper()
{
    // send gripper command upon entering
    if (!sent_gripper_command_) {
        ROS_WARN("Sending Gripper Goal to open gripper");
        if (!rcta::ReconnectActionClient(
                gripper_command_client_,
                gripper_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::stringstream ss; ss << "Failed to connect to '" << gripper_command_action_name_ << "' action server";
            as_->setAborted(result, ss.str());
            ROS_ERROR("%s", ss.str().c_str());
            return GraspObjectExecutionStatus::FAULT;
        }

        control_msgs::GripperCommandGoal gripper_goal;
        gripper_goal.command.position = GripperModel().maximum_width();
        gripper_goal.command.max_effort = GripperModel().maximum_force();

        auto result_cb = boost::bind(&GraspObjectExecutor::gripperCommandResultCallback, this, _1, _2);
        gripper_command_client_->sendGoal(gripper_goal, result_cb);

        pending_gripper_command_ = true;
        sent_gripper_command_ = true;
    }

    // wait for gripper command to finish
    if (!pending_gripper_command_) {
        ROS_INFO("Gripper Goal to open gripper is no longer pending");

        if (gripper_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            (gripper_command_result_->reached_goal || gripper_command_result_->stalled))
        {
            ROS_INFO("Gripper Command Succeeded");
            if (gripper_command_result_->stalled) {
                ROS_WARN("    Open Gripper Command Succeeded but Stalled During Execution");
            }
            if (skip_viservo_) {
                return GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP;
            } else {
                return GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
            }
        } else {
            ROS_WARN("Open Gripper Command failed");
            ROS_WARN("    Simple Client Goal State: %s", gripper_command_goal_state_.toString().c_str());
            ROS_WARN("    Error Text: %s", gripper_command_goal_state_.getText().c_str());
            ROS_WARN("    result.reached_goal = %s", gripper_command_result_ ? (gripper_command_result_->reached_goal ? "TRUE" : "FALSE") : "null");
            ROS_WARN("    result.stalled = %s", gripper_command_result_ ? (gripper_command_result_->stalled ? "TRUE" : "FALSE") : "null");
            return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP;
        }
    }

    return GraspObjectExecutionStatus::OPENING_GRIPPER;
}

void GraspObjectExecutor::onOpeningGripperExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onExecutingVisualServoMotionToPregraspEnter(
    GraspObjectExecutionStatus::Status from)
{
    sent_viservo_command_ = false;
    pending_viservo_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onExecutingVisualServoMotionToPregrasp()
{
    if (!sent_viservo_command_) {
        ROS_WARN("Sending Viservo Goal to pregrasp pose");

        if (!rcta::ReconnectActionClient(
                viservo_command_client_,
                viservo_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::stringstream ss; ss << "Failed to connect to '" << viservo_command_action_name_ << "' action server";
            as_->setAborted(result, ss.str());
            ROS_ERROR("%s", ss.str().c_str());
            return GraspObjectExecutionStatus::FAULT;
        }

        const std::string kinematics_frame = "arm_mount_panel_dummy";
        const std::string camera_frame = "camera_rgb_frame";

        tf::StampedTransform tf_transform;
        try {
            listener_.lookupTransform(camera_frame, kinematics_frame, ros::Time(0), tf_transform);
        }
        catch (const tf::TransformException& ex) {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::stringstream ss;
            ss << "Failed to lookup transform " << kinematics_frame << " -> " << camera_frame << "; Unable to determine viservo goal";
            as_->setAborted(result, ss.str());
            return GraspObjectExecutionStatus::FAULT;
        }

        Eigen::Affine3d camera_to_kinematics;
        msg_utils::convert(tf_transform, camera_to_kinematics);

        // camera -> kinematics * kinematics -> wrist
        Eigen::Affine3d kinematics_to_wrist;
        tf::poseMsgToEigen(last_move_arm_pregrasp_goal_.goal_pose, kinematics_to_wrist);

        Eigen::Affine3d viservo_pregrasp_goal_transform =
                camera_to_kinematics * kinematics_to_wrist;

        last_viservo_pregrasp_goal_.goal_id = current_goal_->id;
        tf::poseEigenToMsg(viservo_pregrasp_goal_transform, last_viservo_pregrasp_goal_.goal_pose);

        auto result_cb = boost::bind(&GraspObjectExecutor::viservoCommandResultCallback, this, _1, _2);
        viservo_command_client_->sendGoal(last_viservo_pregrasp_goal_, result_cb);

        pending_viservo_command_ = true;
        sent_viservo_command_ = true;
    } else if (!pending_viservo_command_) {
        ROS_INFO("Viservo Goal to pregrasp is no longer pending");

        if (viservo_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            viservo_command_result_->result == rcta::ViservoCommandResult::SUCCESS)
        {
            ROS_INFO("Viservo Command Succeeded");
            return GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
        } else {
            ROS_INFO("Viservo Command failed");
            ROS_INFO("    Simple Client Goal State: %s", viservo_command_goal_state_.toString().c_str());
            ROS_INFO("    Error Text: %s", viservo_command_goal_state_.getText().c_str());
            ROS_INFO("    result.result = %s", viservo_command_result_ ? (viservo_command_result_->result == rcta::ViservoCommandResult::SUCCESS? "SUCCESS" : "NOT SUCCESS") : "null");
            ROS_WARN("Failed to complete visual servo motion to pregrasp");
            return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP;
        }
    }

    return GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
}

void GraspObjectExecutor::onExecutingVisualServoMotionToPregraspExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onExecutingVisualServoMotionToGraspEnter(
    GraspObjectExecutionStatus::Status from)
{
    sent_viservo_command_ = false;
    pending_viservo_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onExecutingVisualServoMotionToGrasp()
{
    if (!sent_viservo_command_) {
        ROS_WARN("Sending Viservo Goal to grasp pose");

        if (!rcta::ReconnectActionClient(
                viservo_command_client_,
                viservo_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::stringstream ss; ss << "Failed to connect to '" << viservo_command_action_name_ << "' action server";
            as_->setAborted(result, ss.str());
            ROS_ERROR("%s", ss.str().c_str());
            return GraspObjectExecutionStatus::FAULT;
        }

        Eigen::Affine3d viservo_grasp_goal_transform;
        tf::poseMsgToEigen(last_viservo_pregrasp_goal_.goal_pose, viservo_grasp_goal_transform);
        // camera -> pregrasp * pregrasp -> grasp = camera -> grasp
        viservo_grasp_goal_transform = viservo_grasp_goal_transform * m_grasp_planner.graspToPregrasp().inverse();

        rcta::ViservoCommandGoal last_viservo_grasp_goal;

        last_viservo_grasp_goal.goal_id = current_goal_->id;
        tf::poseEigenToMsg(viservo_grasp_goal_transform, last_viservo_grasp_goal.goal_pose);

        auto result_cb = boost::bind(&GraspObjectExecutor::viservoCommandResultCallback, this, _1, _2);
        viservo_command_client_->sendGoal(last_viservo_grasp_goal, result_cb);

        pending_viservo_command_ = true;
        sent_viservo_command_ = true;
    } else if (!pending_viservo_command_) {
        ROS_INFO("Viservo Goal to grasp is no longer pending");

        if (viservo_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            viservo_command_result_->result == rcta::ViservoCommandResult::SUCCESS)
        {
            ROS_INFO("Viservo Command Succeeded");
            return GraspObjectExecutionStatus::GRASPING_OBJECT;
        } else {
            ROS_INFO("Viservo Command failed");
            ROS_INFO("    Simple Client Goal State: %s", viservo_command_goal_state_.toString().c_str());
            ROS_INFO("    Error Text: %s", viservo_command_goal_state_.getText().c_str());
            ROS_INFO("    result.result = %s", viservo_command_result_ ? (viservo_command_result_->result == rcta::ViservoCommandResult::SUCCESS? "SUCCESS" : "NOT SUCCESS (lol)") : "null");
            ROS_WARN("Failed to complete visual servo motion to grasp");
            return GraspObjectExecutionStatus::FAULT;
        }
    }

    return GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
}

void GraspObjectExecutor::onExecutingVisualServoMotionToGraspExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onGraspingObjectEnter(
    GraspObjectExecutionStatus::Status from)
{
    sent_gripper_command_ = false;
    pending_gripper_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onGraspingObject()
{
    // send the gripper command to close the gripper
    if (!sent_gripper_command_) {
        ROS_WARN("Sending Gripper Goal to close gripper");
        if (!rcta::ReconnectActionClient(
                gripper_command_client_,
                gripper_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::stringstream ss; ss << "Failed to connect to '" << gripper_command_action_name_ << "' action server";
            as_->setAborted(result, ss.str());
            ROS_ERROR("%s", ss.str().c_str());
            return GraspObjectExecutionStatus::FAULT;
        }

        control_msgs::GripperCommandGoal gripper_goal;
        gripper_goal.command.max_effort = GripperModel().maximum_force();
        gripper_goal.command.position = GripperModel().minimum_width();

        auto result_cb = boost::bind(&GraspObjectExecutor::gripperCommandResultCallback, this, _1, _2);
        gripper_command_client_->sendGoal(gripper_goal, result_cb);

        pending_gripper_command_ = true;
        sent_gripper_command_ = true;
    }

    // check whether we've grabbed the object
    if (!pending_gripper_command_) {
        ROS_INFO("Gripper Goal to close gripper is no longer pending");

        const bool grabbed_object =
                gripper_command_result_->reached_goal ||
                gripper_command_result_->stalled;

        // note: reverting to the old method since the gripper does not
        // think that it has made contact with an object when a wraparound grasp is performed
//                const bool grabbed_object = !gripper_command_result_->reached_goal || gripper_command_result_->stalled;

        if (gripper_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            grabbed_object)
        {
            ROS_INFO("Gripper Command Succeeded");
            return GraspObjectExecutionStatus::MOVING_ARM_TO_STOW;
        } else {
            ROS_WARN("Close Gripper Command failed");
            ROS_WARN("    Simple Client Goal State: %s", gripper_command_goal_state_.toString().c_str());
            ROS_WARN("    Error Text: %s", gripper_command_goal_state_.getText().c_str());
            ROS_WARN("    result.reached_goal = %s", gripper_command_result_ ? (gripper_command_result_->reached_goal ? "TRUE" : "FALSE") : "null");
            ROS_WARN("    result.stalled = %s", gripper_command_result_ ? (gripper_command_result_->stalled ? "TRUE" : "FALSE") : "null");
            return GraspObjectExecutionStatus::RETRACTING_GRIPPER;
        }
    }

    return GraspObjectExecutionStatus::GRASPING_OBJECT;
}

void GraspObjectExecutor::onGraspingObjectExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onRetractingGripperEnter(
    GraspObjectExecutionStatus::Status from)
{
    sent_gripper_command_ = false;
    pending_gripper_command_ = false;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onRetractingGripper()
{
    // TODO: could refactor this into an explicit state machine for opening the
    // gripper with defined completion and fault transition states to share code
    // with the OpeningGripper state
    if (!sent_gripper_command_) {
        ROS_WARN("Sending Gripper Goal to retract gripper");
        if (!rcta::ReconnectActionClient(
                gripper_command_client_,
                gripper_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::stringstream ss; ss << "Failed to connect to '" << gripper_command_action_name_ << "' action server";
            as_->setAborted(result, ss.str());
            ROS_ERROR("%s", ss.str().c_str());
            return GraspObjectExecutionStatus::FAULT;
        }

        control_msgs::GripperCommandGoal gripper_goal;
        gripper_goal.command.max_effort = GripperModel().maximum_force();
        gripper_goal.command.position = GripperModel().maximum_width();

        auto result_cb = boost::bind(&GraspObjectExecutor::gripperCommandResultCallback, this, _1, _2);
        gripper_command_client_->sendGoal(gripper_goal, result_cb);

        pending_gripper_command_ = true;
        sent_gripper_command_ = true;
    }

    if (!pending_gripper_command_) {
        ROS_INFO("Gripper Goal to retract gripper is no longer pending");

        if (gripper_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Gripper Command Succeeded");
        } else {
            ROS_INFO("Close Gripper Command failed");
            ROS_INFO("    Simple Client Goal State: %s", gripper_command_goal_state_.toString().c_str());
            ROS_INFO("    Error Text: %s", gripper_command_goal_state_.getText().c_str());
            ROS_INFO("    result.reached_goal = %s", gripper_command_result_ ? (gripper_command_result_->reached_goal ? "TRUE" : "FALSE") : "null");
            ROS_INFO("    result.stalled = %s", gripper_command_result_ ? (gripper_command_result_->stalled ? "TRUE" : "FALSE") : "null");
        }

        return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP; // Transfer back to planning regardless
    }

    return GraspObjectExecutionStatus::RETRACTING_GRIPPER;
}

void GraspObjectExecutor::onRetractingGripperExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onMovingArmToStowEnter(
    GraspObjectExecutionStatus::Status from)
{
    auto solver = m_manip_group->getSolverInstance();
    const std::string& tip_link = solver->getTipFrames().front();

    moveit_msgs::AttachedCollisionObject aco;

    // aco.link_name
    aco.link_name = tip_link;

    // aco.object
    aco.object = CreateGascanCollisionObject();

    ROS_INFO("Create attached object with %zu shapes and %zu poses", aco.object.primitives.size(), aco.object.primitive_poses.size());

    // transform the object to the grasp frame
    const Eigen::Affine3d T_grasp_object =
            last_successful_grasp_.pose_in_object.inverse();

    for (geometry_msgs::Pose& pose : aco.object.primitive_poses) {
        Eigen::Affine3d T_object_part;
        tf::poseMsgToEigen(pose, T_object_part);
        Eigen::Affine3d T_grasp_part = T_grasp_object * T_object_part;
        tf::poseEigenToMsg(T_grasp_part, pose);
    }

    for (geometry_msgs::Pose& pose : aco.object.mesh_poses) {
        Eigen::Affine3d T_object_part;
        tf::poseMsgToEigen(pose, T_object_part);
        Eigen::Affine3d T_grasp_part = T_grasp_object * T_object_part;
        tf::poseEigenToMsg(T_grasp_part, pose);
    }

    aco.object.header.frame_id = tip_link;
    aco.object.id = "gascan";
    aco.object.operation = moveit_msgs::CollisionObject::ADD;

//    aco.object.primitive_poses[0].position.x -= 0.15;
//    aco.object.primitive_poses[1].position.x -= 0.15;

    // aco.touch_links
    aco.touch_links = {
        "limb_right_link7",
        "limb_right_palm",

        "limb_right_finger_1_link_0",
        "limb_right_finger_1_link_1",
        "limb_right_finger_1_link_2",
        "limb_right_finger_1_link_3",

        "limb_right_finger_2_link_0",
        "limb_right_finger_2_link_1",
        "limb_right_finger_2_link_2",
        "limb_right_finger_2_link_3",

        "limb_right_finger_middle_link_0",
        "limb_right_finger_middle_link_1",
        "limb_right_finger_middle_link_2",
        "limb_right_finger_middle_link_3",

        "limb_right_tool",
    };

//    aco.detach_posture;

    aco.weight = 1.0; //arbitrary (not used anyways)

    attach_obj_pub_.publish(aco);

    attach_obj_req_time_ = ros::Time::now();

    sent_move_arm_goal_ = false;
    pending_move_arm_command_ = false;
    next_stow_position_to_attempt_ = 0;
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onMovingArmToStow()
{
    if (!sent_move_arm_goal_) {
        ROS_INFO("Sending Move Arm Goal to stow position");
        if (!rcta::ReconnectActionClient(
                move_arm_command_client_,
                move_arm_command_action_name_,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            std::stringstream ss; ss << "Failed to connect to '" << move_arm_command_action_name_ << "' action server";
            ROS_ERROR("%s", ss.str().c_str());
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            as_->setAborted(result, ss.str());
            next_stow_position_to_attempt_ = 0;
            return GraspObjectExecutionStatus::FAULT;
        }

        if (next_stow_position_to_attempt_ >= stow_positions_.size()) {
            rcta_msgs::GraspObjectCommandResult result;
            std::string error = "Ran out of stow positions to attempt";
            ROS_ERROR("%s", error.c_str());
            result.result = rcta_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            as_->setAborted(result, error);
            next_stow_position_to_attempt_ = 0;
            return GraspObjectExecutionStatus::FAULT;
        }

        // 4. send a move arm goal for the best grasp

        const StowPosition& next_stow_position = stow_positions_[next_stow_position_to_attempt_++];

        rcta::MoveArmGoal move_arm_stow_goal;

        move_arm_stow_goal.type = rcta::MoveArmGoal::JointGoal;

        move_arm_stow_goal.goal_joint_state.name.reserve(next_stow_position.joint_positions.size());
        move_arm_stow_goal.goal_joint_state.position.reserve(next_stow_position.joint_positions.size());
        for (const auto& entry : next_stow_position.joint_positions) {
            move_arm_stow_goal.goal_joint_state.name.push_back(entry.first);
            move_arm_stow_goal.goal_joint_state.position.push_back(entry.second);
        }

        move_arm_stow_goal.octomap = use_extrusion_octomap_ ?
                *current_octomap_ : current_goal_->octomap;

        //include the attached object in the goal
        // TODO: include the attached object here

        move_arm_stow_goal.execute_path = true;

        auto result_cb = boost::bind(&GraspObjectExecutor::moveArmResultCallback, this, _1, _2);
        move_arm_command_client_->sendGoal(move_arm_stow_goal, result_cb);

        pending_move_arm_command_ = true;
        sent_move_arm_goal_ = true;
    } else if (!pending_move_arm_command_) {
        ROS_INFO("Move Arm Goal is no longer pending");
        if (move_arm_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
            move_arm_command_result_ && move_arm_command_result_->success)
        {
            ROS_INFO("Move Arm Command succeeded");
            next_stow_position_to_attempt_ = 0;
            return GraspObjectExecutionStatus::COMPLETING_GOAL;
        } else {
            ROS_INFO("Move Arm Command failed");
            ROS_INFO("    Simple Client Goal State: %s", move_arm_command_goal_state_.toString().c_str());
            ROS_INFO("    Error Text: %s", move_arm_command_goal_state_.getText().c_str());
            ROS_INFO("    result.success = %s", move_arm_command_result_ ? (move_arm_command_result_->success ? "TRUE" : "FALSE") : "null");
        }

        // allow to try next stow
        sent_move_arm_goal_ = false;
        pending_move_arm_command_ = false;
    }

    return GraspObjectExecutionStatus::MOVING_ARM_TO_STOW;
}

void GraspObjectExecutor::onMovingArmToStowExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::onCompletingGoalEnter(
    GraspObjectExecutionStatus::Status from)
{
    ros::Time now = ros::Time::now();
    wait_for_grid_start_time_ = now;
    ROS_INFO("Waiting for a costmap more recent than %s", boost::posix_time::to_simple_string(now.toBoost()).c_str());
}

GraspObjectExecutionStatus::Status GraspObjectExecutor::onCompletingGoal()
{
    if (last_occupancy_grid_ &&
        last_occupancy_grid_->header.stamp > wait_for_grid_start_time_)
    {
        ROS_INFO("Received a fresh costmap");
        // get here once we've received a newer costmap to evaluate for the object's position
        OccupancyGridConstPtr occupancy_grid_after_grasp = last_occupancy_grid_;

        double success_pct = calcProbSuccessfulGrasp(
            *occupancy_grid_after_grasp,
            gas_can_in_grid_frame_.pose.position.x,
            gas_can_in_grid_frame_.pose.position.y,
            object_filter_radius_m_);

        occupancy_grid_after_grasp.reset();

        success_pct = clamp(success_pct, 0.0, 1.0);
        if (success_pct == 1.0) {
            ROS_WARN("Mighty confident now, aren't we?");
        } else {
            ROS_WARN("Grasped object with %0.3f %% confidence", 100.0 * success_pct);
        }

        if (success_pct > gas_can_detection_threshold_) {
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::SUCCESS;
            as_->setSucceeded(result);
            return GraspObjectExecutionStatus::IDLE;
        } else {
            std::string message = "It appears that we have likely not grasped the object";
            ROS_WARN("%s", message.c_str());
            rcta_msgs::GraspObjectCommandResult result;
            result.result = rcta_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            as_->setAborted(result, message);
            return GraspObjectExecutionStatus::FAULT;
        }
    }

    return GraspObjectExecutionStatus::COMPLETING_GOAL;
}

void GraspObjectExecutor::onCompletingGoalExit(
    GraspObjectExecutionStatus::Status to)
{

}

void GraspObjectExecutor::moveArmResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const rcta::MoveArmResult::ConstPtr& result)
{
    move_arm_command_goal_state_ = state;
    move_arm_command_result_ = result;
    pending_move_arm_command_ = false;
}

void GraspObjectExecutor::viservoCommandResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const rcta::ViservoCommandResult::ConstPtr& result)
{
    viservo_command_goal_state_ = state;
    viservo_command_result_ = result;
    pending_viservo_command_ = false;
}

void GraspObjectExecutor::gripperCommandResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::GripperCommandResult::ConstPtr& result)
{
    gripper_command_goal_state_ = state;
    gripper_command_result_ = result;
    pending_gripper_command_ = false;
}

void GraspObjectExecutor::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    last_occupancy_grid_ = msg;
}

uint8_t GraspObjectExecutor::executionStatusToFeedbackStatus(GraspObjectExecutionStatus::Status status)
{
    switch (status) {
    case GraspObjectExecutionStatus::IDLE:
    case GraspObjectExecutionStatus::FAULT:
        return -1;
    case GraspObjectExecutionStatus::GENERATING_GRASPS:
    case GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP:
        return rcta_msgs::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::OPENING_GRIPPER:
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return rcta_msgs::GraspObjectCommandFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return rcta_msgs::GraspObjectCommandFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
    case GraspObjectExecutionStatus::GRASPING_OBJECT:
    case GraspObjectExecutionStatus::RETRACTING_GRIPPER:
        return rcta_msgs::GraspObjectCommandFeedback::GRASPING_OBJECT;
    case GraspObjectExecutionStatus::MOVING_ARM_TO_STOW:
        // same fall-through reasonining from above
        return rcta_msgs::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_STOW;
    case GraspObjectExecutionStatus::COMPLETING_GOAL:
        return -1;
    default:
        return -1;
    }
}

void GraspObjectExecutor::pruneGraspCandidates(
    std::vector<rcta::GraspCandidate>& candidates,
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& camera_pose,
    double marker_incident_angle_threshold_rad) const
{
    ROS_INFO("Filter %zu grasp candidates", candidates.size());

    EigenSTL::vector_Affine3d marker_poses;
    marker_poses.reserve(attached_markers_.size());
    for (const auto& marker : attached_markers_) {
        marker_poses.push_back(marker.link_to_marker);
    }

    // run this first since this is significantly less expensive than IK
    rcta::PruneGraspsByVisibility(
            candidates,
            marker_poses,
            camera_pose,
            marker_incident_angle_threshold_rad);

    pruneGraspCandidatesIK(candidates, robot_pose);
}

void GraspObjectExecutor::pruneGraspCandidatesIK(
    std::vector<rcta::GraspCandidate>& candidates,
    const Eigen::Affine3d& T_grasp_robot) const
{
    // TODO: shamefully copied from RepositionBaseExecutor...it's probably about
    // time to make a minimal API for grasp filtering and create a shareable
    // module for this

    ROS_INFO("Filter %zu grasp candidate via IK", candidates.size());
    std::vector<rcta::GraspCandidate> filtered_candidates;
    filtered_candidates.reserve(candidates.size());

    for (const rcta::GraspCandidate& grasp_candidate : candidates) {
        moveit::core::RobotState robot_state(m_robot_model);
        robot_state.setToDefaultValues();
        // place the robot in the grasp frame
        const moveit::core::JointModel* root_joint = m_robot_model->getRootJoint();
        robot_state.setJointPositions(root_joint, T_grasp_robot);
        robot_state.update();

        ROS_DEBUG("test grasp candidate %s for ik solution", to_string(grasp_candidate.pose).c_str());

        // check for an ik solution to this grasp pose
        std::vector<double> sol;
        if (robot_state.setFromIK(m_manip_group, grasp_candidate.pose)) {
            robot_state.copyJointGroupPositions(m_manip_group, sol);
            rcta::GraspCandidate reachable_grasp_candidate(
                    grasp_candidate.pose,
                    grasp_candidate.pose_in_object,
                    grasp_candidate.u);
            filtered_candidates.push_back(reachable_grasp_candidate);

            ROS_INFO("Grasp pose: %s", to_string(grasp_candidate.pose).c_str());
            ROS_INFO("IK sol: %s", to_string(sol).c_str());
        }
    }

    ROS_INFO("%zu/%zu reachable candidates", filtered_candidates.size(), candidates.size());
    candidates = std::move(filtered_candidates);
}

void GraspObjectExecutor::limitGraspCandidates(
    std::vector<rcta::GraspCandidate>& candidates,
    int max_candidates) const
{
    candidates.resize(max_candidates);
	std::reverse(candidates.begin(), candidates.end());
}

visualization_msgs::MarkerArray
GraspObjectExecutor::getGraspCandidatesVisualization(
    const std::vector<rcta::GraspCandidate>& grasps,
    const std::string& ns) const
{
    const std::string& frame_id = current_goal_->gas_can_in_base_link.header.frame_id;
    return rcta::GetGraspCandidatesVisualization(grasps, frame_id, ns);
}

void GraspObjectExecutor::clearCircleFromGrid(
    nav_msgs::OccupancyGrid& grid,
    double circle_x, double circle_y,
    double circle_radius) const
{
    ROS_INFO("Clearing circle at (%0.3f, %0.3f) with radius %0.3f from costmap", circle_x, circle_y, circle_radius);
    Eigen::Vector2d circle_center(circle_x, circle_y);

    // get the min and max grid coordinates to scan
    Eigen::Vector2i min_grid, max_grid;
    world_to_grid(grid, circle_x - circle_radius, circle_y - circle_radius, min_grid(0), min_grid(1));
    max_grid(0) = (int)std::ceil((circle_x + circle_radius - grid.info.origin.position.x) / grid.info.resolution);
    max_grid(1) = (int)std::ceil((circle_y + circle_radius - grid.info.origin.position.y) / grid.info.resolution);

    ROS_INFO("Clearing circle points within [%d, %d] x [%d, %d]", min_grid(0), min_grid(1), max_grid(0), max_grid(1));

    int num_cleared = 0;
    for (int grid_x = min_grid(0); grid_x < max_grid(0); ++grid_x) {
        for (int grid_y = min_grid(1); grid_y <  max_grid(1); ++grid_y) {
            Eigen::Vector2i gp(grid_x, grid_y);
            if (!within_bounds(grid, gp(0), gp(1))) {
                ROS_WARN("Grid point (%d, %d) is outside of costmap bounds", gp(0), gp(1));
                continue;
            }

            Eigen::Vector2d wp;
            grid_to_world(grid, grid_x, grid_y, wp(0), wp(1));

            Eigen::Vector2d bl = wp - Eigen::Vector2d(-0.5 * grid.info.resolution, -0.5 * grid.info.resolution);
            Eigen::Vector2d br = wp - Eigen::Vector2d(0.5 * grid.info.resolution, -0.5 * grid.info.resolution);
            Eigen::Vector2d tr = wp - Eigen::Vector2d(0.5 * grid.info.resolution, 0.5 * grid.info.resolution);
            Eigen::Vector2d tl = wp - Eigen::Vector2d(-0.5 * grid.info.resolution, 0.5 * grid.info.resolution);
            if ((bl - circle_center).norm() <= circle_radius ||
                (br - circle_center).norm() <= circle_radius ||
                (tr - circle_center).norm() <= circle_radius ||
                (tl - circle_center).norm() <= circle_radius)
            {
                if (grid_at(grid, gp(0), gp(1)) != 0) {
                    grid_at(grid, gp(0), gp(1)) = 0;
                    ++num_cleared;
                    ROS_INFO("Clearing cell (%d, %d)", gp(0), gp(1));
                }
            }
        }
    }

    ROS_INFO("Cleared %d cells", num_cleared);
}

bool GraspObjectExecutor::within_bounds(const nav_msgs::OccupancyGrid& grid, int x, int y) const
{
    return x >= 0 && x < grid.info.width && y >= 0 && y < grid.info.height;
}

void GraspObjectExecutor::grid_to_world(
    const nav_msgs::OccupancyGrid& grid,
    int grid_x,
    int grid_y,
    double& world_x,
    double& world_y) const
{
    world_x = grid_x * grid.info.resolution + 0.5 * grid.info.resolution + grid.info.origin.position.x;
    world_y = grid_y * grid.info.resolution + 0.5 * grid.info.resolution + grid.info.origin.position.y;
}

void GraspObjectExecutor::world_to_grid(
    const nav_msgs::OccupancyGrid& grid,
    double world_x,
    double world_y,
    int& grid_x,
    int& grid_y) const
{
    grid_x = (int)((world_x - grid.info.origin.position.x) / grid.info.resolution);
    grid_y = (int)((world_y - grid.info.origin.position.y) / grid.info.resolution);
}

std::int8_t& GraspObjectExecutor::grid_at(nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const
{
    return grid.data[grid_y * grid.info.width + grid_x];
}

const std::int8_t& GraspObjectExecutor::grid_at(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const
{
    return grid.data[grid_y * grid.info.width + grid_x];
}

double GraspObjectExecutor::calcProbSuccessfulGrasp(
    const nav_msgs::OccupancyGrid& grid,
    double circle_x,
    double circle_y,
    double circle_radius) const
{
    au::vector2d mean = { circle_x, circle_y };
    au::matrix2d covariance(au::matrix2d::zeros());
    covariance(0, 0) = 0.1;
    covariance(1, 1) = 0.1;
    au::gaussian_distribution<2> gauss(mean, covariance);

    ROS_INFO("Setting up gaussian with mean (%0.3f, %0.3f) and covariance (%0.3f, %0.3f)", mean[0], mean[1], covariance(0, 0), covariance(1, 1));

    ROS_INFO("Evaluating circle at (%0.3f, %0.3f) with radius %0.3f from costmap", circle_x, circle_y, circle_radius);
    Eigen::Vector2d circle_center(circle_x, circle_y);

    // get the min and max grid coordinates to scan
    Eigen::Vector2i min_grid, max_grid;
    world_to_grid(grid, circle_x - circle_radius, circle_y - circle_radius, min_grid(0), min_grid(1));
    max_grid(0) = (int)std::ceil((circle_x + circle_radius - grid.info.origin.position.x) / grid.info.resolution);
    max_grid(1) = (int)std::ceil((circle_y + circle_radius - grid.info.origin.position.y) / grid.info.resolution);

    int num_x_cells = max_grid(0) - min_grid(0);
    int num_y_cells = max_grid(1) - min_grid(1);
    ROS_INFO("Instantiating %d x %d probability mask", num_x_cells, num_y_cells);
    Eigen::MatrixXd mask(num_x_cells, num_y_cells);
    for (int i = 0; i < num_x_cells; ++i) {
        for (int j = 0; j < num_y_cells; ++j) {
            mask(i, j) = 0.0;
        }
    }

    ROS_INFO("Initializing probability mask over [%d, %d] x [%d, %d]", min_grid(0), min_grid(1), max_grid(0), max_grid(1));
    // initialize the mask to the gaussian values in each cell
    double sum = 0.0;
    for (int grid_x = min_grid(0); grid_x < max_grid(0); ++grid_x) {
        for (int grid_y = min_grid(1); grid_y <  max_grid(1); ++grid_y) {

            Eigen::Vector2i gp(grid_x, grid_y);
            if (!within_bounds(grid, gp(0), gp(1))) {
                ROS_WARN("Grid point (%d, %d) is outside of costmap bounds", gp(0), gp(1));
                continue;
            }

            Eigen::Vector2d wp;
            grid_to_world(grid, grid_x, grid_y, wp(0), wp(1));

            // calculate the probability modulus for this cell
            int xrow = grid_x - min_grid(0);
            int ycol = grid_y - min_grid(1);
            mask(xrow, ycol) = gauss({ wp.x(), wp.y() });
            sum += mask(xrow, ycol);

        }
    }

    sum = gauss(mean);
    ROS_INFO("Normalizing gaussian mask with normalizer %0.6f", sum);

    // normalize the mask
    for (int x = 0; x < num_x_cells; ++x) {
        for (int y = 0; y < num_y_cells; ++y) {
            mask(x, y) *= (1.0 / sum);
            printf("%0.3f ", mask(x, y));
        }
        printf("\n");
    }

    const std::int8_t obsthresh = 100;

    double probability = 1.0;
    int free_cell_count = 0;
    for (int grid_x = min_grid(0); grid_x < max_grid(0); ++grid_x) {
        for (int grid_y = min_grid(1); grid_y <  max_grid(1); ++grid_y) {
            int xrow = grid_x - min_grid(0);
            int ycol = grid_y - min_grid(1);
            double probmask = mask(xrow, ycol);

            if (grid_at(grid, grid_x, grid_y) >= obsthresh) {
                ROS_INFO("Obstacle cell detected. probmask = %0.6f", probmask);
                probability *= (1.0 - probmask);
            } else {
                ++free_cell_count;
            }
        }
    }

    ROS_INFO("Checking for gascan");
    ROS_INFO(" -> %d cells free", free_cell_count);
    ROS_INFO(" -> p(grasped) = %0.3f", probability);

    return probability;
}

const moveit::core::RobotState& GraspObjectExecutor::currentRobotState() const
{
    assert(m_scene_monitor && m_scene_monitor->getPlanningScene());
    return m_scene_monitor->getPlanningScene()->getCurrentState();
}

void GraspObjectExecutor::processSceneUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    switch (type) {
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE: {

    }   break;
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS: {

    }   break;
    }
}

bool GraspObjectExecutor::downloadMarkerParams()
{
    double marker_to_link_x;
    double marker_to_link_y;
    double marker_to_link_z;
    double marker_to_link_roll_degs;
    double marker_to_link_pitch_degs;
    double marker_to_link_yaw_degs;

    AttachedMarker attached_marker;

    bool success =
            msg_utils::download_param(ph_, "tracked_marker_id", attached_marker.marker_id) &&
            msg_utils::download_param(ph_, "tracked_marker_attached_link", attached_marker.attached_link) &&
            msg_utils::download_param(ph_, "marker_to_link_x", marker_to_link_x) &&
            msg_utils::download_param(ph_, "marker_to_link_y", marker_to_link_y) &&
            msg_utils::download_param(ph_, "marker_to_link_z", marker_to_link_z) &&
            msg_utils::download_param(ph_, "marker_to_link_roll_deg", marker_to_link_roll_degs) &&
            msg_utils::download_param(ph_, "marker_to_link_pitch_deg", marker_to_link_pitch_degs) &&
            msg_utils::download_param(ph_, "marker_to_link_yaw_deg", marker_to_link_yaw_degs);

    attached_marker.link_to_marker = Eigen::Affine3d(
        Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
        Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

    attached_markers_.push_back(std::move(attached_marker));

    if (!success) {
        ROS_WARN("Failed to download marker params");
    }

    return success;
}
