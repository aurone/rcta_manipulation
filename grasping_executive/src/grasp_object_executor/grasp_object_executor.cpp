// standard includes
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

// system includes
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cmu_manipulation_msgs/GraspObjectCommandAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <grasp_planner_interface/grasp_planner_plugin.h>
#include <grasp_planner_interface/grasp_utils.h>
#include <leatherman/print.h>
#include <message_filters/subscriber.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_loader.h>
#include <rcta_manipulation_common/comms/actionlib.h>
#include <robotiq_controllers/gripper_model.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/random/gaussian.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/utils.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>

// project includes
#include <grasping_executive/MoveArmAction.h>

namespace rcta {

///////////////////////
// Utility Functions //
///////////////////////

moveit_msgs::CollisionObject CreateGroundPlaneObject()
{
    moveit_msgs::CollisionObject gpo;
    gpo.header.frame_id = "map"; // TODO: model frame

    shape_msgs::Plane ground_plane;
    ground_plane.coef[0] = 0.0;
    ground_plane.coef[1] = 0.0;
    ground_plane.coef[2] = 1.0;

    // TODO: derive this from the resolution set in the world collision model
    // to be -0.5 * res, which should make one layer of voxels immediately
    // beneath z = 0
    ground_plane.coef[3] = 0.075;

    gpo.planes.push_back(ground_plane);
    gpo.plane_poses.push_back(geometry_msgs::IdentityPose());

    gpo.operation = moveit_msgs::CollisionObject::ADD;
    return gpo;
}

auto BuildMoveGroupGoal(const grasping_executive::MoveArmGoal& goal)
    -> moveit_msgs::MoveGroupGoal
{
    double allowed_planning_time = 10.0;
    const char* group_name = "right_arm_and_torso";
    const char* workspace_frame = "base_link";
    geometry_msgs::Vector3 workspace_min;
    workspace_min.x = -0.5;
    workspace_min.y = -1.5;
    workspace_min.z =  -0.2;
    geometry_msgs::Vector3 workspace_max;
    workspace_max.x =  1.5;
    workspace_max.y =  0.5;
    workspace_max.z =  1.9;

    const char* pose_goal_planner_id = "right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]";
    const char* joint_goal_planner_id = "right_arm_and_torso[right_arm_and_torso_ARA_JD_ML]";

    double joint_tolerance = sbpl::angles::to_radians(5.0);
    double pos_tolerance = 0.01;
    double rot_tolerance = sbpl::angles::to_radians(5.0);
    std::string tip_link = "limb_right_link7";

    moveit_msgs::MoveGroupGoal g;
    g.request.allowed_planning_time = allowed_planning_time;
    g.request.group_name = group_name;
    g.request.max_acceleration_scaling_factor = 1.0;
    g.request.max_velocity_scaling_factor = 1.0;
    g.request.num_planning_attempts = 1;
    g.request.path_constraints.joint_constraints.clear();
    g.request.path_constraints.name = "";
    g.request.path_constraints.orientation_constraints.clear();
    g.request.path_constraints.position_constraints.clear();
    g.request.path_constraints.visibility_constraints.clear();
    g.request.trajectory_constraints.constraints.clear();
    if (!goal.execute_path) {
        g.request.start_state.is_diff = true;
    }
    g.request.workspace_parameters.header.frame_id = workspace_frame;
    g.request.workspace_parameters.min_corner = workspace_min;
    g.request.workspace_parameters.max_corner = workspace_max;

    auto& request = g.request;
    auto& ops = g.planning_options;

    ops.planning_scene_diff.robot_state.is_diff = true;
    ops.planning_scene_diff.world.octomap.origin.orientation.w = 1.0;
    ops.planning_scene_diff.world.octomap.octomap = goal.octomap;
    ops.planning_scene_diff.world.collision_objects = goal.planning_options.planning_scene_diff.world.collision_objects;
    ops.planning_scene_diff.world.collision_objects.push_back(CreateGroundPlaneObject());
    ops.planning_scene_diff.is_diff = true;

    ops.plan_only = !goal.execute_path;

    ops.look_around = false;

    ops.look_around_attempts = 0;

    ops.max_safe_execution_cost = 1.0;

    ops.replan = false;

    ops.replan_attempts = 0;

    ops.replan_delay = 0.0;

    request.start_state = goal.start_state;

    request.goal_constraints.clear();
    switch (goal.type) {
    case grasping_executive::MoveArmGoal::JointGoal:
    {
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.name = "goal_constraints";
        for (size_t jidx = 0; jidx < goal.goal_joint_state.name.size(); ++jidx) {
            auto& joint_name = goal.goal_joint_state.name[jidx];
            double joint_pos = goal.goal_joint_state.position[jidx];

            moveit_msgs::JointConstraint joint_constraint;
            joint_constraint.joint_name = joint_name;
            joint_constraint.position = joint_pos;
            joint_constraint.tolerance_above = joint_tolerance;
            joint_constraint.tolerance_below = joint_tolerance;
            joint_constraint.weight = 1.0;
            goal_constraints.joint_constraints.push_back(joint_constraint);
        }
        request.goal_constraints.push_back(goal_constraints);

        request.planner_id = joint_goal_planner_id;
        break;
    }
    case grasping_executive::MoveArmGoal::CartesianGoal:
    case grasping_executive::MoveArmGoal::EndEffectorGoal:
    {
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.name = "goal_constraints";

        geometry_msgs::PoseStamped tip_goal;

        // TODO: get this from the configured planning frame in moveit
        tip_goal.header.frame_id = "map";
        tip_goal.pose = goal.goal_pose;

        // one position constraint
        moveit_msgs::PositionConstraint goal_pos_constraint;
        goal_pos_constraint.header.frame_id = tip_goal.header.frame_id;
        goal_pos_constraint.link_name = tip_link;
        goal_pos_constraint.target_point_offset = geometry_msgs::CreateVector3(0.0, 0.0, 0.0);
        shape_msgs::SolidPrimitive tolerance_volume;
        tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
        tolerance_volume.dimensions = { pos_tolerance };
        goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
        goal_pos_constraint.constraint_region.primitive_poses.push_back(tip_goal.pose);
        goal_pos_constraint.weight = 1.0;

        // one orientation constraint
        moveit_msgs::OrientationConstraint goal_rot_constraint;
        goal_rot_constraint.header.frame_id = tip_goal.header.frame_id;
        goal_rot_constraint.orientation = tip_goal.pose.orientation;
        goal_rot_constraint.link_name = tip_link;
        goal_rot_constraint.absolute_x_axis_tolerance = rot_tolerance;
        goal_rot_constraint.absolute_y_axis_tolerance = rot_tolerance;
        goal_rot_constraint.absolute_z_axis_tolerance = rot_tolerance;
        goal_rot_constraint.weight = 1.0;

        goal_constraints.position_constraints.push_back(goal_pos_constraint);
        goal_constraints.orientation_constraints.push_back(goal_rot_constraint);
        request.goal_constraints.push_back(goal_constraints);

        request.planner_id = pose_goal_planner_id;
        break;
    }
    default:
        ROS_ERROR("Unrecognized MoveArmGoal type");
        return g;
    }

    return g;
}

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
    double body_part_z = -0.03741; //-0.01241;      // -0.05
    double body_part_qw = 1.0;          // 1.0
    double body_part_qx = 0.0001895;    // 0.0
    double body_part_qy = -0.009;       // 0.0
    double body_part_qz = 0.021;        // 0.0
    double body_part_dim_x = 0.20; //0.212;     // 0.20
    double body_part_dim_y = 0.20; //0.212;     // 0.25
    double body_part_dim_z = 0.15; //0.296;     // 0.25

    double noz_part_x = 0.00816;            // 0.0
    double noz_part_y = -0.15700;           // 0.0
    double noz_part_z = 0.15571;            // 0.15
    double noz_part_qw = 0.932;             // (45 degs about x)
    double noz_part_qx = 0.354;             // ^
    double noz_part_qy = 0.028;             // ^
    double noz_part_qz = -0.072;            // ^
    double noz_part_radius = 0.5 * 0.5 * 0.052;   // 0.035
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

void transformCollisionObject(
    moveit_msgs::CollisionObject& o,
    const Eigen::Affine3d& t)
{
    for (auto& pose : o.primitive_poses) {
        Eigen::Affine3d T_object_part;
        tf::poseMsgToEigen(pose, T_object_part);
        Eigen::Affine3d T_grasp_part = t * T_object_part;
        tf::poseEigenToMsg(T_grasp_part, pose);
    }

    for (auto& pose : o.mesh_poses) {
        Eigen::Affine3d T_object_part;
        tf::poseMsgToEigen(pose, T_object_part);
        Eigen::Affine3d T_grasp_part = t * T_object_part;
        tf::poseEigenToMsg(T_grasp_part, pose);
    }
}

////////////////////
// Grid Functions //
////////////////////

bool within_bounds(const nav_msgs::OccupancyGrid& grid, int x, int y)
{
    return x >= 0 && x < grid.info.width && y >= 0 && y < grid.info.height;
}

void grid_to_world(
    const nav_msgs::OccupancyGrid& grid,
    int grid_x,
    int grid_y,
    double& world_x,
    double& world_y)
{
    world_x = grid_x * grid.info.resolution + 0.5 * grid.info.resolution + grid.info.origin.position.x;
    world_y = grid_y * grid.info.resolution + 0.5 * grid.info.resolution + grid.info.origin.position.y;
}

void world_to_grid(
    const nav_msgs::OccupancyGrid& grid,
    double world_x,
    double world_y,
    int& grid_x,
    int& grid_y)
{
    grid_x = (int)((world_x - grid.info.origin.position.x) / grid.info.resolution);
    grid_y = (int)((world_y - grid.info.origin.position.y) / grid.info.resolution);
}

std::int8_t& grid_at(nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y)
{
    return grid.data[grid_y * grid.info.width + grid_x];
}

const std::int8_t& grid_at(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y)
{
    return grid.data[grid_y * grid.info.width + grid_x];
}

void clearCircleFromGrid(
    nav_msgs::OccupancyGrid& grid,
    double circle_x, double circle_y,
    double circle_radius)
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

double calcProbSuccessfulGrasp(
    const nav_msgs::OccupancyGrid& grid,
    double circle_x,
    double circle_y,
    double circle_radius)
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

/////////////////////////
// GraspObjectExecutor //
/////////////////////////

enum GraspObjectExecutionStatus
{
    IDLE = 0,
    FAULT,
    GENERATING_GRASPS,
    PRESHAPE_HAND,
    MOVING_ARM_TO_PREGRASP,
    MOVING_ARM_TO_GRASP,
    OPENING_GRIPPER,
    EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP,
    EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP,
    GRASPING_OBJECT,
    RETRACTING_GRIPPER,
    MOVING_ARM_TO_STOW,
    COMPLETING_GOAL,
    GRASP_OBJECT_EXECUTION_STATUS_COUNT
};

std::string to_string(GraspObjectExecutionStatus status);

uint8_t executionStatusToFeedbackStatus(GraspObjectExecutionStatus status);

struct StowPosition
{
    std::string name;
    std::string type;
    std::map<std::string, double> joint_positions;
};

bool extract_xml_value(XmlRpc::XmlRpcValue& value, StowPosition& stow_position);

struct AttachedMarker
{
    int marker_id;
    std::string attached_link;
    Eigen::Affine3d link_to_marker;
};

struct GraspObjectExecutor
{
    using GraspObjectCommandActionServer
            = actionlib::SimpleActionServer<cmu_manipulation_msgs::GraspObjectCommandAction>;

    using MoveArmActionClient
            = actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>;
    using GripperCommandActionClient
            = actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

    using GraspPlannerPluginPtr = class_loader::ClassLoader::UniquePtr<GraspPlannerPlugin>;

    /// \name ROS stuff
    ///@{
    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph = ros::NodeHandle("~");

    ros::Publisher  m_filtered_costmap_pub;
    ros::Publisher  m_attach_obj_pub;
    ros::Subscriber m_costmap_sub;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> point_cloud_sub;
    std::unique_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> point_cloud_filter;

    ros::Subscriber m_point_cloud_sub;

    tf::TransformListener m_listener;

    std::unique_ptr<GraspObjectCommandActionServer> m_server;
    std::unique_ptr<MoveArmActionClient>            m_move_arm_command_client;
    std::unique_ptr<GripperCommandActionClient>     m_gripper_command_client;

    std::string m_action_name                   = "grasp_object_command";
    std::string m_move_arm_command_action_name  = "move_group";
    std::string m_gripper_command_action_name;

    std::unique_ptr<ros::ServiceClient> m_check_state_validity_client;

    sbpl::VisualizerROS m_viz;
    ///@}

    robot_model_loader::RobotModelLoaderPtr         m_rml;
    moveit::core::RobotModelPtr                     m_robot_model;
    pluginlib::ClassLoader<GraspPlannerPlugin>      m_grasp_planner_loader;
    GraspPlannerPluginPtr                           m_grasp_planner;
    planning_scene_monitor::CurrentStateMonitorPtr  m_state_monitor;
    std::vector<std::string>                        m_gripper_links;

    /// \name Global Parameters
    ///@{

    std::string m_camera_view_frame = "asus_rgb_optical_frame";
    std::string m_manip_name;
    moveit::core::JointModelGroup* m_manip_group = NULL;

    bool m_attach_object = false;
    bool m_filter_visibility = false;

    ///@}

    /// \name GenerateGrasps Parameters
    ///@{
    std::vector<AttachedMarker> m_attached_markers;
    ///@}

    /// \name MoveArmToPregrasp Parameters
    ///@{
    int m_max_grasp_candidates = 0;
    ///@}

    /// \name MoveArmToStow Parameters
    ///@{
    std::vector<std::vector<StowPosition>> m_stow_sequences;
    ///@}

    /// \name CompleteGoal Parameters
    ///@{
    bool m_check_for_gascan = false;
    double m_object_filter_radius;
    double m_gas_can_detection_threshold = 0.0;
    ///@}

    /// \name Goal Context
    ///@{

    cmu_manipulation_msgs::GraspObjectCommandGoal::ConstPtr m_current_goal;

    Eigen::Affine3d m_obj_pose;

    ///@}

    std::mutex m_last_occupancy_grid_mutex;
    nav_msgs::OccupancyGridConstPtr m_last_occupancy_grid; ///< most recent OccupancyGrid message

    std::mutex m_last_point_cloud_mutex;
    sensor_msgs::PointCloud2::ConstPtr m_last_point_cloud;

    /// \name Shared State
    ///@{

    // shared(GenerateGrasps, MoveArmToPregrasp)
    // -> to plan to a number of different grasps, ranked by graspability
    std::vector<Grasp> m_grasp_candidates;

    // shared(MoveArmToPregrasp, ExecuteVisualServoMotionToPregrasp)
    // shared(MoveArmToPregrasp, MoveArmToGrasp)
    // -> to enforce visual servo to the same pose
    grasping_executive::MoveArmGoal m_last_move_arm_pregrasp_goal;

    // shared(MoveArmToPregrasp, MoveArmToStow)
    // -> to know how to attach the object to the arm
    Grasp m_last_successful_grasp;

    ///@}

    /// \name Completing state
    ///@{
    ros::Time m_wait_for_grid_start_time;
    ///@}

    GraspObjectExecutor();
    ~GraspObjectExecutor();

    bool initialize();

    bool downloadMarkerParams();

    auto currentRobotState() const -> moveit::core::RobotState;

    void goalCallback();
    void preemptCallback();

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void pruneGrasps(
        std::vector<Grasp>& candidates,
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& camera_pose,
        double marker_incident_angle_threshold_rad) const;

    void pruneGraspsIK(
        std::vector<Grasp>& candidates,
        const Eigen::Affine3d& T_grasp_robot) const;

    auto getGraspsVisualization(
        const std::vector<Grasp>& grasps,
        const std::string& ns) const
        -> visualization_msgs::MarkerArray;
};

std::string to_string(GraspObjectExecutionStatus status)
{
    switch (status) {
    case GraspObjectExecutionStatus::IDLE:                                      return "Idle";
    case GraspObjectExecutionStatus::FAULT:                                     return "Fault";
    case GraspObjectExecutionStatus::GENERATING_GRASPS:                         return "GeneratingGrasps";
    case GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP:                    return "MovingArmToPregrasp";
    case GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP:                       return "MovingArmToGrasp";
    case GraspObjectExecutionStatus::OPENING_GRIPPER:                           return "OpeningGripper";
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP: return "ExecutingVisualServoMotionToPregrasp";
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:    return "ExecutingVisualServoMotionToGrasp";
    case GraspObjectExecutionStatus::GRASPING_OBJECT:                           return "GraspingObject";
    case GraspObjectExecutionStatus::RETRACTING_GRIPPER:                        return "RetractingGripper";
    case GraspObjectExecutionStatus::MOVING_ARM_TO_STOW:                        return "MovingArmToStow";
    case GraspObjectExecutionStatus::COMPLETING_GOAL:                           return "CompletingGoal";
    default:                                                                    return "<Unknown>";
    }
}

uint8_t executionStatusToFeedbackStatus(GraspObjectExecutionStatus status)
{
    switch (status) {
    case GraspObjectExecutionStatus::IDLE:
    case GraspObjectExecutionStatus::FAULT:
        return -1;
    case GraspObjectExecutionStatus::GENERATING_GRASPS:
    case GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP:
        return cmu_manipulation_msgs::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::OPENING_GRIPPER:
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return cmu_manipulation_msgs::GraspObjectCommandFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return cmu_manipulation_msgs::GraspObjectCommandFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
    case GraspObjectExecutionStatus::GRASPING_OBJECT:
    case GraspObjectExecutionStatus::RETRACTING_GRIPPER:
        return cmu_manipulation_msgs::GraspObjectCommandFeedback::GRASPING_OBJECT;
    case GraspObjectExecutionStatus::MOVING_ARM_TO_STOW:
        // same fall-through reasonining from above
        return cmu_manipulation_msgs::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_STOW;
    case GraspObjectExecutionStatus::COMPLETING_GOAL:
        return -1;
    default:
        return -1;
    }
}

bool extract_xml_value(XmlRpc::XmlRpcValue& value, StowPosition& stow_position)
{
    ROS_INFO("Extract stow position");

    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Stow position config must be a struct");
        return false;
    }

    if (!value.hasMember("name") ||
        !value.hasMember("type") ||
        !value.hasMember("joint_vector_degs"))
    {
        ROS_ERROR("Stow position config must have members 'name' and 'joint_vector_degs'");
        return false;
    }

    std::string name;
    if (!msg_utils::extract_xml_value(value["name"], name)) {
        ROS_ERROR("Failed to extract 'name' field");
        return false;
    }

    std::string type;
    if (!msg_utils::extract_xml_value(value["type"], type)) {
        ROS_ERROR("Failed to extract 'type' field");
        return false;
    }

    std::map<std::string, double> joint_positions;
    if (!msg_utils::extract_xml_value(value["joint_vector_degs"], joint_positions)) {
        ROS_ERROR("Failed to extract 'joint_vector_degs' field");
        return false;
    }

    stow_position.name = std::move(name);
    stow_position.type = std::move(type);
    stow_position.joint_positions = std::move(joint_positions);
    return true;
}

void TransformWristPosesToPregraspPoses(
    GraspObjectExecutor* ex,
    std::vector<Grasp>& grasps)
{
    for (auto& grasp : grasps) {
        // model -> pregrasp[wrist] = model -> grasp[wrist] * grasp -> pregrasp
        grasp.pose = grasp.pose * ex->m_grasp_planner->graspToPregrasp();
    }
}

GraspObjectExecutor::GraspObjectExecutor() :
    m_grasp_planner_loader("grasp_planner_interface", "rcta::GraspPlannerPlugin")
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

    ROS_INFO("Initialize Grasp Object Executor");

    m_rml.reset(new robot_model_loader::RobotModelLoader);
    m_robot_model = m_rml->getModel();
    if (!m_robot_model) {
        ROS_ERROR("Failed to load Robot Model");
        return false;
    }

    if (m_filter_visibility) {
        if (!m_robot_model->hasLinkModel(m_camera_view_frame)) {
            ROS_ERROR("No link '%s' found in the robot model", m_camera_view_frame.c_str());
            return false;
        }
    }

    auto transformer = boost::shared_ptr<tf::Transformer>(new tf::TransformListener);
    m_state_monitor = boost::make_shared<planning_scene_monitor::CurrentStateMonitor>(m_robot_model, transformer);
    m_state_monitor->startStateMonitor("joint_states");

    if (!msg_utils::download_param(m_ph, "manipulator_group_name", m_manip_name)) {
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

    std::string grasp_planner_plugin;
    if (!m_ph.getParam("grasp_planner_plugin", grasp_planner_plugin)) {
        ROS_ERROR("Failed to retrieve 'grasp_planner_plugin' from the param server");
        return false;
    }

    ROS_INFO("Load grasp planner plugin '%s'", grasp_planner_plugin.c_str());
    m_grasp_planner =
            m_grasp_planner_loader.createUniqueInstance(grasp_planner_plugin);

    ROS_INFO("Initialize grasp planner plugin");
    ros::NodeHandle grasp_nh(m_ph, "grasping");
    if (!m_grasp_planner->init(m_nh, grasp_nh)) {
        ROS_ERROR("Failed to initialize Grasp Planner");
        return false;
    }

    // read in max grasps
    if (!msg_utils::download_param(m_ph, "max_grasp_candidates", m_max_grasp_candidates) ||
        m_max_grasp_candidates < 0)
    {
        ROS_ERROR("Failed to retrieve 'max_grasp_candidates' from the param server or 'max_grasp_candidates' is negative");
        return false;
    }

    m_check_state_validity_client.reset(new ros::ServiceClient);
    *m_check_state_validity_client =
            m_nh.serviceClient<moveit_msgs::GetStateValidity>(
                    "check_state_validity");

    ////////////////////////////////////////
    // GraspObjectExecutor-specific stuff //
    ////////////////////////////////////////

    // TODO: get these from end-effector group in SRDF
    m_gripper_links = {
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

    // read in stow positions
    if (!msg_utils::download_param(m_ph, "stow_sequences", m_stow_sequences)) {
        ROS_ERROR("Failed to retrieve 'stow_positions' from the param server");
        return false;
    }

    // log stow sequences and convert to radians from degrees
    ROS_INFO("Stow Sequences:");
    int seqno = 0;
    for (auto& sequence : m_stow_sequences) {
        ROS_INFO("  Sequence %d", seqno++);
        for (StowPosition& position : sequence) {
            ROS_INFO("    Name: %s", position.name.c_str());
            ROS_INFO("    Type: %s", position.type.c_str());
            for (auto& entry : position.joint_positions) {
                ROS_INFO("      %s: %0.3f", entry.first.c_str(), entry.second);
                entry.second = entry.second * M_PI / 180.0;
            }
        }
    }

    if (!msg_utils::download_param(m_ph, "check_for_gascan", m_check_for_gascan) ||
        !msg_utils::download_param(m_ph, "object_filter_radius_m", m_object_filter_radius) ||
        !msg_utils::download_param(m_ph, "gas_can_detection_threshold", m_gas_can_detection_threshold))
    {
        return false;
    }

    if (!downloadMarkerParams()) {
        return false; // errors printed within
    }

    // subscribers
    m_costmap_sub = m_nh.subscribe<nav_msgs::OccupancyGrid>(
            "map", 1, &GraspObjectExecutor::occupancyGridCallback, this);
//    m_point_cloud_sub = m_nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, &GraspObjectExecutor::pointCloudCallback, this);

    this->point_cloud_sub.reset(
            new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                    m_nh, "cloud_in", 1));
    assert(this->m_robot_model);
    this->point_cloud_filter.reset(
            new tf::MessageFilter<sensor_msgs::PointCloud2>(
                *this->point_cloud_sub,
                this->m_listener,
                this->m_robot_model->getModelFrame(),
                1));
    this->point_cloud_filter->registerCallback(boost::bind(&GraspObjectExecutor::pointCloudCallback, this, _1));

    // publishers
    m_filtered_costmap_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("costmap_filtered", 1);
    m_attach_obj_pub = m_nh.advertise<moveit_msgs::AttachedCollisionObject>(
            "attached_collision_object", 1);

    m_move_arm_command_client.reset(new MoveArmActionClient(m_move_arm_command_action_name, false));
    if (!m_move_arm_command_client) {
        ROS_ERROR("Failed to instantiate Move Arm Command Client");
        return false;
    }

    if (!msg_utils::download_param(
            m_ph,
            "gripper_command_action_name",
            m_gripper_command_action_name))
    {
        return false;
    }

    m_gripper_command_client.reset(new GripperCommandActionClient(m_gripper_command_action_name, false));
    if (!m_gripper_command_client) {
        ROS_ERROR("Failed to instantiate Gripper Command Client");
        return false;
    }

    m_server.reset(new GraspObjectCommandActionServer(m_action_name, false));
    if (!m_server) {
        ROS_ERROR("Failed to instantiate Grasp Object Action Server");
        return false;
    }

    m_server->registerGoalCallback(boost::bind(&GraspObjectExecutor::goalCallback, this));
    m_server->registerPreemptCallback(boost::bind(&GraspObjectExecutor::preemptCallback, this));

    ROS_INFO("Starting action server '%s'...", m_action_name.c_str());
    m_server->start();
    ROS_INFO("Action server started");

    return true;
}

void GraspObjectExecutor::goalCallback()
{
    if (m_server->isActive()) {
        ROS_WARN("I'm busy!");
        return;
    }

    ROS_INFO("Received a new goal");
    m_current_goal = m_server->acceptNewGoal();
    ROS_INFO("  Goal ID: %u", m_current_goal->id);
    ROS_INFO("  Retry Count: %d", m_current_goal->retry_count);
    ROS_INFO("  Gas Can Pose [world frame: %s]: %s", m_current_goal->gas_can_in_map.header.frame_id.c_str(), to_string(m_current_goal->gas_can_in_map.pose).c_str());
    ROS_INFO("  Gas Can Pose [robot frame: %s]: %s", m_current_goal->gas_can_in_base_link.header.frame_id.c_str(), to_string(m_current_goal->gas_can_in_base_link.pose).c_str());
    ROS_INFO("  Octomap ID: %s", m_current_goal->octomap.id.c_str());

    auto& obj_pose_in = m_current_goal->gas_can_in_map;
    if (obj_pose_in.header.frame_id != m_robot_model->getModelFrame()) {
        ROS_INFO("Transform object pose into model frame");
        geometry_msgs::PoseStamped obj_pose_in_model;
        try {
            m_listener.transformPose(
                    m_robot_model->getModelFrame(),
                    obj_pose_in,
                    obj_pose_in_model);
            tf::poseMsgToEigen(obj_pose_in_model.pose, m_obj_pose);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("Failed to transform from '%s' to '%s' (%s)", obj_pose_in.header.frame_id.c_str(), m_robot_model->getModelFrame().c_str(), ex.what());
            m_server->setAborted();
            return;
        }
    } else {
        tf::poseMsgToEigen(obj_pose_in.pose, m_obj_pose);
    }
}

void GraspObjectExecutor::preemptCallback()
{

}

void GraspObjectExecutor::occupancyGridCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(m_last_occupancy_grid_mutex);
    m_last_occupancy_grid = msg;
}

void GraspObjectExecutor::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(m_last_point_cloud_mutex);

    ROS_DEBUG("Received point cloud message");
    m_last_point_cloud = msg;
}

void GraspObjectExecutor::pruneGrasps(
    std::vector<Grasp>& candidates,
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& camera_pose,
    double marker_incident_angle_threshold_rad) const
{
    ROS_INFO("Filter %zu grasp candidates", candidates.size());

    if (m_filter_visibility) {
        EigenSTL::vector_Affine3d marker_poses;
        marker_poses.reserve(m_attached_markers.size());
        for (auto& marker : m_attached_markers) {
            marker_poses.push_back(marker.link_to_marker);
        }

        // run this first since this is significantly less expensive than IK
        PruneGraspsByVisibility(
                candidates,
                marker_poses,
                camera_pose,
                marker_incident_angle_threshold_rad);
    }

    pruneGraspsIK(candidates, robot_pose);
}

void GraspObjectExecutor::pruneGraspsIK(
    std::vector<Grasp>& grasps,
    const Eigen::Affine3d& robot_pose) const
{
    // TODO: shamefully copied from RepositionBaseExecutor...it's probably about
    // time to make a minimal API for grasp filtering and create a shareable
    // module for this

    ROS_INFO("Filter %zu grasp candidate via IK", grasps.size());
    std::vector<Grasp> filtered_candidates;
    filtered_candidates.reserve(grasps.size());

    int pregrasp_ik_filter_count = 0;
    int grasp_ik_filter_count = 0;
    int collision_filter_count = 0;

    for (auto& grasp : grasps) {
        // TODO: use current state here instead?
        moveit::core::RobotState robot_state(m_robot_model);
        robot_state.setToDefaultValues();
        auto* root_joint = m_robot_model->getRootJoint();
        robot_state.setJointPositions(root_joint, robot_pose);

        ROS_DEBUG("test grasp candidate %s for ik solution", to_string(grasp.pose).c_str());

        // check for an ik solution to the pre-grasp pose
        if (!robot_state.setFromIK(m_manip_group, grasp.pose)) {
            ++pregrasp_ik_filter_count;
            continue;
        }

        // check for an ik solution to the grasp pose
        if (!robot_state.setFromIK(
                m_manip_group, grasp.pose * m_grasp_planner->pregraspToGrasp()))
        {
            ++grasp_ik_filter_count;
            continue;
        }

        // check for arm collisions at the IK solution
        moveit_msgs::GetStateValidityRequest req;
        req.group_name = m_manip_group->getName();
        moveit::core::robotStateToRobotStateMsg(robot_state, req.robot_state);
        moveit_msgs::GetStateValidityResponse res;
        if (!m_check_state_validity_client->call(req, res) || !res.valid) {
            ++collision_filter_count;
            continue;
        }

        ROS_INFO("Pregrasp pose: %s", to_string(grasp.pose).c_str());
        filtered_candidates.push_back(grasp);

        // log the ik solution to the grasp pose
        std::vector<double> sol;
        robot_state.copyJointGroupPositions(m_manip_group, sol);
        ROS_INFO("IK sol: %s", to_string(sol).c_str());
    }

    ROS_INFO("%zu/%zu reachable grasps", filtered_candidates.size(), grasps.size());
    ROS_INFO("  %d pregrasp ik failures", pregrasp_ik_filter_count);
    ROS_INFO("  %d grasp ik failures", grasp_ik_filter_count);
    ROS_INFO("  %d grasp collision failures", collision_filter_count);
    grasps = std::move(filtered_candidates);
}

auto GraspObjectExecutor::getGraspsVisualization(
    const std::vector<Grasp>& grasps,
    const std::string& ns) const
    -> visualization_msgs::MarkerArray
{
    auto& frame_id = m_robot_model->getModelFrame();
    return GetGraspCandidatesVisualization(grasps, frame_id, ns);
}

auto GraspObjectExecutor::currentRobotState() const -> moveit::core::RobotState
{
    assert(m_state_monitor && m_state_monitor->getPlanningScene());
    auto current_state = m_state_monitor->getCurrentState();
    if (!current_state) {
        ROS_WARN("FAILED TO GET CURRENT STATE");
        moveit::core::RobotState zero_state(m_robot_model);
        zero_state.setToDefaultValues();
        return zero_state;
    } else {
        return *current_state;
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
            msg_utils::download_param(m_ph, "tracked_marker_id", attached_marker.marker_id) &&
            msg_utils::download_param(m_ph, "tracked_marker_attached_link", attached_marker.attached_link) &&
            msg_utils::download_param(m_ph, "marker_to_link_x", marker_to_link_x) &&
            msg_utils::download_param(m_ph, "marker_to_link_y", marker_to_link_y) &&
            msg_utils::download_param(m_ph, "marker_to_link_z", marker_to_link_z) &&
            msg_utils::download_param(m_ph, "marker_to_link_roll_deg", marker_to_link_roll_degs) &&
            msg_utils::download_param(m_ph, "marker_to_link_pitch_deg", marker_to_link_pitch_degs) &&
            msg_utils::download_param(m_ph, "marker_to_link_yaw_deg", marker_to_link_yaw_degs);

    attached_marker.link_to_marker = Eigen::Affine3d(
        Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
        Eigen::AngleAxisd(sbpl::angles::to_radians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(sbpl::angles::to_radians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(sbpl::angles::to_radians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

    m_attached_markers.push_back(std::move(attached_marker));

    if (!success) {
        ROS_WARN("Failed to download marker params");
    }

    return success;
}

////////////////////////////////////////////
// Grasp Object Executive State Functions //
////////////////////////////////////////////

auto OnIdle(GraspObjectExecutor* ex) -> GraspObjectExecutionStatus
{
    ros::Rate poll_rate(10.0);
    while (!ex->m_server->isActive() && ros::ok()) {
        poll_rate.sleep();
    }

    if (!ros::ok()) {
        return GraspObjectExecutionStatus::IDLE;
    }

    return GraspObjectExecutionStatus::GENERATING_GRASPS;
}

auto OnFault(GraspObjectExecutor* ex) -> GraspObjectExecutionStatus
{
    ros::Rate poll_rate(10.0);
    while (!ex->m_server->isActive() && ros::ok()) {
        poll_rate.sleep();
    }
    if (!ros::ok()) {
        return GraspObjectExecutionStatus::FAULT;
    }

    return GraspObjectExecutionStatus::GENERATING_GRASPS;
}

auto DoGenerateGrasps(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    std::vector<Grasp> candidates;

    sensor_msgs::PointCloud2::ConstPtr point_cloud;
    {
        std::unique_lock<std::mutex> lock(ex->m_last_point_cloud_mutex);
        point_cloud = ex->m_last_point_cloud;
    }

    std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> grasp_cloud;
    if (point_cloud) {
        ROS_INFO("Have point cloud for grasp planning!");
        // sensor_msgs/PointCloud2 -> pcl::PointXYZ
        ROS_INFO("Convert point cloud from ROS message");
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*point_cloud, cloud);

        // transform cloud to model frame
        ROS_INFO("Wait for transform from '%s' to '%s'", point_cloud->header.frame_id.c_str(), ex->m_robot_model->getModelFrame().c_str());
        std::string error;
        if (!ex->m_listener.waitForTransform(
                ex->m_robot_model->getModelFrame(),
                point_cloud->header.frame_id,
                point_cloud->header.stamp,
                ros::Duration(10.0),
                ros::Duration(0.01),
                &error))
        {
            ROS_ERROR("Failed to wait for transform (%s)", error.c_str());
            cmu_manipulation_msgs::GraspObjectCommandResult result;
            result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            ex->m_server->setAborted(result, "Failed to transform point cloud into model frame for grasp planning");
            return GraspObjectExecutionStatus::FAULT;
        }

        ROS_INFO("Lookup transform");
        tf::StampedTransform transform;
        try {
            ex->m_listener.lookupTransform(
                    ex->m_robot_model->getModelFrame(),
                    point_cloud->header.frame_id,
                    point_cloud->header.stamp,
                    transform);
        } catch (const tf::TransformException& e) {
            ROS_ERROR("Failed to lookup transform (%s)", e.what());
            cmu_manipulation_msgs::GraspObjectCommandResult result;
            result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            ex->m_server->setAborted(result, "Failed to transform point cloud into model frame for grasp planning");
            return GraspObjectExecutionStatus::FAULT;
        }

        ROS_INFO("Transform point cloud");
        // convert to pcl point cloud for transformation
        pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
        pcl_ros::transformPointCloud(cloud, cloud_transformed, transform);

        pcl::PointCloud<pcl::PointXYZ>* cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
        cloud_filtered->points.reserve(cloud_transformed.size());
        cloud_filtered->header.frame_id = ex->m_robot_model->getModelFrame();

        double dist_thresh = 0.5; // TODO: configurate me
        for (int i = 0; i < cloud_transformed.points.size(); ++i) {
            auto& point = cloud_transformed.points[i];
            Eigen::Vector3d p(point.x, point.y, point.z);
            auto dist_sq = (ex->m_obj_pose.translation() - p).squaredNorm();
            if (dist_sq <= dist_thresh * dist_thresh) {
                pcl::PointXYZ pp; pp.x = p.x(); pp.y = p.y(); pp.z = p.z();
                cloud_filtered->points.push_back(pp);
            }
        }

        grasp_cloud.reset(cloud_filtered);
    } else {
        ROS_WARN("No point cloud available for grasp planning");
    }

    // 1. Generate grasp candidates (poses of the tool in the robot frame) from
    // the object pose. We ask the grasp planner to produce more grasps than the
    // amount we will actually use so that we have a larger set to apply our own
    // reachability/graspability analysis to.
    int max_samples = 100;
    if (!ex->m_grasp_planner->planGrasps(
            "gascan", ex->m_obj_pose, grasp_cloud.get(), max_samples, candidates))
    {
        ROS_ERROR("Failed to sample grasps");
        return GraspObjectExecutionStatus::FAULT;
    }

    ROS_INFO("Sampled %zd grasp poses", candidates.size());

    // 2. Get poses for the wrist from poses for the tool
    TransformWristPosesToPregraspPoses(ex, candidates);

    SV_SHOW_INFO(ex->getGraspsVisualization(candidates, "candidate_grasp"));

    // 3. Prune grasps by reachability/IK/etc
    auto robot_state = ex->currentRobotState();

    auto& T_world_robot = robot_state.getGlobalLinkTransform(ex->m_robot_model->getRootLink());
    ROS_INFO("world -> robot: %s", to_string(T_world_robot).c_str());

    Eigen::Affine3d T_world_camera = Eigen::Affine3d::Identity();
    if (ex->m_filter_visibility) {
        T_world_camera = robot_state.getGlobalLinkTransform(ex->m_camera_view_frame);
        ROS_INFO("world -> camera: %s", to_string(T_world_camera).c_str());
    }

    double vis_angle_thresh = sbpl::angles::to_radians(45.0);
    ex->pruneGrasps(candidates, T_world_robot, T_world_camera, vis_angle_thresh);

    ROS_INFO("Produced %zd reachable grasp poses", candidates.size());

    // 4. Order grasp candidates by their graspability/reachability.
    std::sort(begin(candidates), end(candidates),
            [&](const rcta::Grasp& a, const rcta::Grasp& b)
            {
                return a.u > b.u;
            });

    if (candidates.empty()) {
        ROS_WARN("No reachable grasp candidates available");
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::OBJECT_OUT_OF_REACH;
        ex->m_server->setAborted(result, "No reachable grasp candidates available");
        return GraspObjectExecutionStatus::FAULT;
    }

    if (candidates.size() > ex->m_max_grasp_candidates) {
        candidates.resize(ex->m_max_grasp_candidates);
    }

    ROS_INFO("Attempting %zd grasps", candidates.size());
    SV_SHOW_INFO(ex->getGraspsVisualization(candidates, "reachable_candidates"));

    ex->m_grasp_candidates = std::move(candidates);

    return GraspObjectExecutionStatus::PRESHAPE_HAND;
}

auto DoMoveArmToPreGrasp(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    for (auto& pregrasp : ex->m_grasp_candidates) {
        ROS_WARN("Sending Move Arm Goal to pregrasp pose");
        if (!ReconnectActionClient(
                ex->m_move_arm_command_client,
                ex->m_move_arm_command_action_name,
                ros::Rate(10.0),
                ros::Duration(5.0)))
        {
            std::stringstream ss; ss << "Failed to connect to '" << ex->m_move_arm_command_action_name << "' action server";
            ROS_ERROR("%s", ss.str().c_str());
            cmu_manipulation_msgs::GraspObjectCommandResult result;
            result.result = cmu_manipulation_msgs::GraspObjectCommandResult::PLANNING_FAILED;
            ex->m_server->setAborted(result, ss.str());
            return GraspObjectExecutionStatus::FAULT;
        }

        // 4. send a move arm goal for the best grasp
        grasping_executive::MoveArmGoal goal;
        goal.type = grasping_executive::MoveArmGoal::EndEffectorGoal;

        tf::poseEigenToMsg(pregrasp.pose, goal.goal_pose);

        if (!ex->m_current_goal->octomap.data.empty()) {
            goal.octomap = ex->m_current_goal->octomap;
        } else {
            ROS_WARN("No octomap received!");
        }

        goal.execute_path = true;

        auto gascan = CreateGascanCollisionObject();
        gascan.header.frame_id = ex->m_robot_model->getModelFrame();
        gascan.id = "gascan";
        gascan.operation = moveit_msgs::CollisionObject::ADD;
        transformCollisionObject(gascan, ex->m_obj_pose);

        goal.planning_options.planning_scene_diff.is_diff = true;
        goal.planning_options.planning_scene_diff.world.collision_objects.push_back(gascan);

        auto state = ex->m_move_arm_command_client->sendGoalAndWait(BuildMoveGroupGoal(goal));
        auto result = ex->m_move_arm_command_client->getResult();

        ROS_INFO("Move Arm Goal is no longer pending");
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED &&
            result &&
            (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS))
        {
            ex->m_last_move_arm_pregrasp_goal = std::move(goal);
            ex->m_last_successful_grasp = pregrasp;

            // TODO: inserting arbitrary sleeps to ensure that plans are not
            // requested too quickly...
            ros::Duration(1.0).sleep();

            ROS_INFO("Move Arm Command succeeded");
            return GraspObjectExecutionStatus::OPENING_GRIPPER;
        }

        ROS_INFO("Move Arm Command failed");
        ROS_INFO("    Simple Client Goal State: %s", state.toString().c_str());
        ROS_INFO("    Error Text: %s", state.getText().c_str());
        ROS_INFO("    result.success = %s", result ? ((result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) ? "TRUE" : "FALSE") : "null");

        // stay in MOVING_ARM_TO_PREGRASP until there are no more grasps
        // TODO: consider moving back to the stow position
    }

    ROS_WARN("Failed to plan to all reachable grasps");
    cmu_manipulation_msgs::GraspObjectCommandResult result;
    result.result = cmu_manipulation_msgs::GraspObjectCommandResult::PLANNING_FAILED;
    ex->m_server->setAborted(result, "Failed on all reachable grasps");
    return GraspObjectExecutionStatus::FAULT;
}

auto DoPreshapeHand(GraspObjectExecutor* ex) -> GraspObjectExecutionStatus
{
    // send gripper command upon entering
    ROS_WARN("Sending Gripper Goal to open gripper");
    if (!ReconnectActionClient(
            ex->m_gripper_command_client,
            ex->m_gripper_command_action_name,
            ros::Rate(10.0),
            ros::Duration(5.0)))
    {
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
        std::stringstream ss; ss << "Failed to connect to '" << ex->m_gripper_command_action_name << "' action server";
        ex->m_server->setAborted(result, ss.str());
        ROS_ERROR("%s", ss.str().c_str());
        return GraspObjectExecutionStatus::FAULT;
    }

    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = GripperModel().maximum_width();
    gripper_goal.command.max_effort = GripperModel().maximum_force();

    auto state = ex->m_gripper_command_client->sendGoalAndWait(gripper_goal);
    auto result = ex->m_gripper_command_client->getResult();

    ROS_INFO("Gripper Goal to open gripper is no longer pending");

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED &&
        result && (result->reached_goal || result->stalled))
    {
        ROS_INFO("Gripper Command Succeeded");
        if (result->stalled) {
            ROS_WARN("    Open Gripper Command Succeeded but Stalled During Execution");
        }
        return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP;
    } else {
        ROS_WARN("Open Gripper Command failed");
        ROS_WARN("    Simple Client Goal State: %s", state.toString().c_str());
        ROS_WARN("    Error Text: %s", state.getText().c_str());
        ROS_WARN("    result.reached_goal = %s", result ? (result->reached_goal ? "TRUE" : "FALSE") : "null");
        ROS_WARN("    result.stalled = %s", result ? (result->stalled ? "TRUE" : "FALSE") : "null");
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
        ex->m_server->setAborted(result, "Failed to preshape hand");
        return GraspObjectExecutionStatus::FAULT;
    }
}

auto DoOpenGripper(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    // send gripper command upon entering
    ROS_WARN("Sending Gripper Goal to open gripper");
    if (!ReconnectActionClient(
            ex->m_gripper_command_client,
            ex->m_gripper_command_action_name,
            ros::Rate(10.0),
            ros::Duration(5.0)))
    {
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
        std::stringstream ss; ss << "Failed to connect to '" << ex->m_gripper_command_action_name << "' action server";
        ex->m_server->setAborted(result, ss.str());
        ROS_ERROR("%s", ss.str().c_str());
        return GraspObjectExecutionStatus::FAULT;
    }

    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.position = GripperModel().maximum_width();
    gripper_goal.command.max_effort = GripperModel().maximum_force();

    auto state = ex->m_gripper_command_client->sendGoalAndWait(gripper_goal);
    auto result = ex->m_gripper_command_client->getResult();

    ROS_INFO("Gripper Goal to open gripper is no longer pending");

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED &&
        result && (result->reached_goal || result->stalled))
    {
        ROS_INFO("Gripper Command Succeeded");
        if (result->stalled) {
            ROS_WARN("    Open Gripper Command Succeeded but Stalled During Execution");
        }
        return GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP;
    } else {
        ROS_WARN("Open Gripper Command failed");
        ROS_WARN("    Simple Client Goal State: %s", state.toString().c_str());
        ROS_WARN("    Error Text: %s", state.getText().c_str());
        ROS_WARN("    result.reached_goal = %s", result ? (result->reached_goal ? "TRUE" : "FALSE") : "null");
        ROS_WARN("    result.stalled = %s", result ? (result->stalled ? "TRUE" : "FALSE") : "null");
        return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP;
    }
}

auto DoMoveArmToGrasp(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    ROS_WARN("Sending Move Arm Goal to grasp pose");
    if (!ReconnectActionClient(
            ex->m_move_arm_command_client,
            ex->m_move_arm_command_action_name,
            ros::Rate(10.0),
            ros::Duration(5.0)))
    {
        std::stringstream ss; ss << "Failed to connect to '" <<
                ex->m_move_arm_command_action_name << "' action server";
        ROS_ERROR("%s", ss.str().c_str());
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::PLANNING_FAILED;
        ex->m_server->setAborted(result, ss.str());
        return GraspObjectExecutionStatus::FAULT;
    }

    grasping_executive::MoveArmGoal grasp_goal;
//      grasp_goal.type = grasping_executive::MoveArmGoal::EndEffectorGoal;
    grasp_goal.type = grasping_executive::MoveArmGoal::CartesianGoal;

    // compute goal pose for grasping from pregrasp pose
    Eigen::Affine3d pregrasp_pose; // model -> wrist (pregrasp)
    tf::poseMsgToEigen(ex->m_last_move_arm_pregrasp_goal.goal_pose, pregrasp_pose);
    Eigen::Affine3d grasp_pose; // model -> wrist (grasp)
    grasp_pose = pregrasp_pose * ex->m_grasp_planner->pregraspToGrasp();;
    tf::poseEigenToMsg(grasp_pose, grasp_goal.goal_pose);

    // TODO: use monitored octomap
    if (!ex->m_current_goal->octomap.data.empty()) {
        grasp_goal.octomap = ex->m_current_goal->octomap;
    } else {
        ROS_WARN("No octomap received!");
    }

    grasp_goal.execute_path = true;

    auto state = ex->m_move_arm_command_client->sendGoalAndWait(BuildMoveGroupGoal(grasp_goal));
    auto result = ex->m_move_arm_command_client->getResult();

    ROS_INFO("Move Arm Goal is no longer pending");
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED &&
        result && (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS))
    {
        ROS_INFO("Move Arm Command succeeded");

        ros::Duration(1.0).sleep();

        return GraspObjectExecutionStatus::GRASPING_OBJECT;
    }

    ROS_INFO("Move Arm Command failed");
    ROS_INFO("    Simple Client Goal State: %s", state.toString().c_str());
    ROS_INFO("    Error Text: %s", state.getText().c_str());
    ROS_INFO("    result.success = %s", result ? ((result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) ? "TRUE" : "FALSE") : "null");
    return GraspObjectExecutionStatus::FAULT;
    // TODO: move back to "moving to pregrasp" until there
    // are no more (pregrasp, grasp) transitions to try
}

auto DoGraspObject(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    // send the gripper command to close the gripper
    ROS_WARN("Sending Gripper Goal to close gripper");
    if (!ReconnectActionClient(
            ex->m_gripper_command_client,
            ex->m_gripper_command_action_name,
            ros::Rate(10.0),
            ros::Duration(5.0)))
    {
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
        std::stringstream ss; ss << "Failed to connect to '" << ex->m_gripper_command_action_name << "' action server";
        ex->m_server->setAborted(result, ss.str());
        ROS_ERROR("%s", ss.str().c_str());
        return GraspObjectExecutionStatus::FAULT;
    }

    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.max_effort = GripperModel().maximum_force();
    gripper_goal.command.position = GripperModel().minimum_width();

    auto state = ex->m_gripper_command_client->sendGoalAndWait(gripper_goal);
    auto result = ex->m_gripper_command_client->getResult();

    // check whether we've grabbed the object
    ROS_INFO("Gripper Goal to close gripper is no longer pending");

    const bool grabbed_object = result->reached_goal || result->stalled;

    // note: reverting to the old method since the gripper does not
    // think that it has made contact with an object when a wraparound grasp is performed
//                const bool grabbed_object = !result->reached_goal || result->stalled;

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && grabbed_object) {
        ROS_INFO("Gripper Command Succeeded");
        return GraspObjectExecutionStatus::MOVING_ARM_TO_STOW;
    }

    ROS_WARN("Close Gripper Command failed");
    ROS_WARN("    Simple Client Goal State: %s", state.toString().c_str());
    ROS_WARN("    Error Text: %s", state.getText().c_str());
    ROS_WARN("    result.reached_goal = %s", result ? (result->reached_goal ? "TRUE" : "FALSE") : "null");
    ROS_WARN("    result.stalled = %s", result ? (result->stalled ? "TRUE" : "FALSE") : "null");
    return GraspObjectExecutionStatus::RETRACTING_GRIPPER;
}

auto DoRetractGripper(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    // TODO: could refactor this into an explicit state machine for opening the
    // gripper with defined completion and fault transition states to share code
    // with the OpeningGripper state
    ROS_WARN("Sending Gripper Goal to retract gripper");
    if (!ReconnectActionClient(
            ex->m_gripper_command_client,
            ex->m_gripper_command_action_name,
            ros::Rate(10.0),
            ros::Duration(5.0)))
    {
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
        std::stringstream ss; ss << "Failed to connect to '" << ex->m_gripper_command_action_name << "' action server";
        ex->m_server->setAborted(result, ss.str());
        ROS_ERROR("%s", ss.str().c_str());
        return GraspObjectExecutionStatus::FAULT;
    }

    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.max_effort = GripperModel().maximum_force();
    gripper_goal.command.position = GripperModel().maximum_width();

    auto state = ex->m_gripper_command_client->sendGoalAndWait(gripper_goal);
    auto result = ex->m_gripper_command_client->getResult();

    ROS_INFO("Gripper Goal to retract gripper is no longer pending");

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper Command Succeeded");
    } else {
        ROS_INFO("Close Gripper Command failed");
        ROS_INFO("    Simple Client Goal State: %s", state.toString().c_str());
        ROS_INFO("    Error Text: %s", state.getText().c_str());
        ROS_INFO("    result.reached_goal = %s", result ? (result->reached_goal ? "TRUE" : "FALSE") : "null");
        ROS_INFO("    result.stalled = %s", result ? (result->stalled ? "TRUE" : "FALSE") : "null");
    }

    return GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP; // Transfer back to planning regardless
}

void DoMoveArmToStowEnter(
    GraspObjectExecutor* ex,
    GraspObjectExecutionStatus from)
{
    if (ex->m_attach_object) {
        moveit_msgs::AttachedCollisionObject aco;

        auto solver = ex->m_manip_group->getSolverInstance();
        auto& tip_link = solver->getTipFrames().front();
        aco.link_name = tip_link;

        aco.object = CreateGascanCollisionObject();

        ROS_INFO("Created attached object with %zu shapes and %zu poses", aco.object.primitives.size(), aco.object.primitive_poses.size());

        // transform the object to the grasp frame

        // T_grasp_object = T_grasp_pregrasp * T_pregrasp_object;
        // grasp -> object = (world -> object * world -> grasp[wrist])^1
        auto& T_world_grasp = ex->m_last_successful_grasp.pose;
        Eigen::Affine3d T_grasp_object = (ex->m_obj_pose * T_world_grasp).inverse();

        ROS_INFO("Attaching gascan at offset %s from the wrist", to_string(T_grasp_object).c_str());

        transformCollisionObject(aco.object, T_grasp_object);

        aco.object.header.frame_id = tip_link;
        aco.object.id = "gascan";
        aco.object.operation = moveit_msgs::CollisionObject::ADD;

        aco.touch_links = ex->m_gripper_links;

//        aco.detach_posture;

        aco.weight = 1.0; //arbitrary (not used anyways)

        ex->m_attach_obj_pub.publish(aco);
    }
}

auto DoMoveArmToStow(GraspObjectExecutor* ex) -> GraspObjectExecutionStatus
{
    for (int ss = 0; ss < ex->m_stow_sequences.size(); ++ss) {
        auto& stow_sequence = ex->m_stow_sequences[ss];
        for (int sp = 0; sp < stow_sequence.size(); ++sp) {
            auto& stow_position = stow_sequence[sp];

            ROS_INFO("Sending Move Arm Goal to stow position");
            if (!ReconnectActionClient(
                    ex->m_move_arm_command_client,
                    ex->m_move_arm_command_action_name,
                    ros::Rate(10.0),
                    ros::Duration(5.0)))
            {
                std::stringstream ss; ss << "Failed to connect to '" << ex->m_move_arm_command_action_name << "' action server";
                ROS_ERROR("%s", ss.str().c_str());
                cmu_manipulation_msgs::GraspObjectCommandResult result;
                result.result = cmu_manipulation_msgs::GraspObjectCommandResult::PLANNING_FAILED;
                ex->m_server->setAborted(result, ss.str());
                return GraspObjectExecutionStatus::FAULT;
            }

            grasping_executive::MoveArmGoal move_arm_stow_goal;

            // fill the appropriate goal type
            if (stow_position.type == "pose") {
                move_arm_stow_goal.type = grasping_executive::MoveArmGoal::EndEffectorGoal;
                auto rs = ex->currentRobotState();
                for (auto& entry : stow_position.joint_positions) {
                    rs.setVariablePosition(entry.first, entry.second);
                }
                rs.updateLinkTransforms();
                Eigen::Affine3d tip_pose =
                        rs.getGlobalLinkTransform(ex->m_manip_group->getOnlyOneEndEffectorTip());
                tf::poseEigenToMsg(tip_pose, move_arm_stow_goal.goal_pose);
            } else if (stow_position.type == "cart") {
                move_arm_stow_goal.type = grasping_executive::MoveArmGoal::CartesianGoal;
                move_arm_stow_goal.type = grasping_executive::MoveArmGoal::EndEffectorGoal;
                auto rs = ex->currentRobotState();
                for (auto& entry : stow_position.joint_positions) {
                    rs.setVariablePosition(entry.first, entry.second);
                }
                rs.updateLinkTransforms();
                Eigen::Affine3d tip_pose =
                        rs.getGlobalLinkTransform(ex->m_manip_group->getOnlyOneEndEffectorTip());
                tf::poseEigenToMsg(tip_pose, move_arm_stow_goal.goal_pose);
            } else {
                auto rs = ex->currentRobotState();

                // NOTE: this bit assumes all single-dof joint
                std::vector<double> vars(ex->m_manip_group->getVariableCount());
                rs.copyJointGroupPositions(ex->m_manip_group, vars);

                move_arm_stow_goal.type = grasping_executive::MoveArmGoal::JointGoal;
                move_arm_stow_goal.goal_joint_state.name = ex->m_manip_group->getVariableNames();
                move_arm_stow_goal.goal_joint_state.position = vars;
                for (auto& entry : stow_position.joint_positions) {
                    auto jit = std::find(
                            ex->m_manip_group->getVariableNames().begin(),
                            ex->m_manip_group->getVariableNames().end(),
                            entry.first);
                    if (jit == ex->m_manip_group->getVariableNames().end()) {
                        ROS_WARN("Joint '%s' not found in the group", entry.first.c_str());
                        continue;
                    }

                    int jidx =  std::distance(ex->m_manip_group->getVariableNames().begin(), jit);
                    move_arm_stow_goal.goal_joint_state.position[jidx] = entry.second;
                }
            }

            if (!ex->m_current_goal->octomap.data.empty()) {
                move_arm_stow_goal.octomap = ex->m_current_goal->octomap;
            } else {
                ROS_WARN("No octomap received!");
            }

            // TODO: include the attached object here just for this request

            move_arm_stow_goal.execute_path = true;

            auto state = ex->m_move_arm_command_client->sendGoalAndWait(BuildMoveGroupGoal(move_arm_stow_goal));
            auto result = ex->m_move_arm_command_client->getResult();

            ROS_INFO("Move Arm Goal is no longer pending");

            // transition to completing goal
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED &&
                result && (result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS))
            {
                ROS_INFO("Move Arm Command succeeded");
                // TODO: move to the next stow position
            } else {
                ROS_INFO("Move Arm Command failed");
                ROS_INFO("    Simple Client Goal State: %s", state.toString().c_str());
                ROS_INFO("    Error Text: %s", state.getText().c_str());
                ROS_INFO("    result.success = %s", result ? ((result->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) ? "TRUE" : "FALSE") : "null");
                // TODO: begin with the next stow sequence
            }
        }

        // transition to completing goal if we finished the stow sequence
        ROS_INFO("Finished stow sequence");
        return GraspObjectExecutionStatus::COMPLETING_GOAL;
    }

    // transition to fault if we exhausted stow sequences
    cmu_manipulation_msgs::GraspObjectCommandResult result;
    std::string error = "Ran out of stow positions to attempt";
    ROS_ERROR("%s", error.c_str());
    result.result = cmu_manipulation_msgs::GraspObjectCommandResult::PLANNING_FAILED;
    ex->m_server->setAborted(result, error);
    return GraspObjectExecutionStatus::FAULT;
}

void DoMoveArmToStowExit(
    GraspObjectExecutor* ex,
    GraspObjectExecutionStatus to)
{
    if (ex->m_attach_object) {
        moveit_msgs::AttachedCollisionObject aco;
        aco.object.id = "gascan";
        aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
        ex->m_attach_obj_pub.publish(aco);
    }
    ros::Duration(1.0).sleep();
}

void DoCompleteGoalEnter(
    GraspObjectExecutor* ex,
    GraspObjectExecutionStatus from)
{
    if (ex->m_check_for_gascan) {
        ros::Time now = ros::Time::now();
        ex->m_wait_for_grid_start_time = now;
        ROS_INFO("Waiting for a costmap more recent than %0.3f", now.toSec());
    }
}

auto DoCompleteGoal(GraspObjectExecutor* ex)
    -> GraspObjectExecutionStatus
{
    if (!ex->m_check_for_gascan) {
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::SUCCESS;
        ex->m_server->setSucceeded(result);
        return GraspObjectExecutionStatus::IDLE;
    }

    ////////////////////////////////////
    // wait for a fresh occupancy map //
    ////////////////////////////////////

    ros::Rate poll_rate(1.0);
    ex->m_last_occupancy_grid_mutex.lock();
    while (!(ex->m_last_occupancy_grid && ex->m_last_occupancy_grid->header.stamp > ex->m_wait_for_grid_start_time)) {
        ex->m_last_occupancy_grid_mutex.unlock();
        ROS_INFO_THROTTLE(5.0, "Waiting for fresh occupancy grid");
        poll_rate.sleep();
        ex->m_last_occupancy_grid_mutex.lock();
    }

    nav_msgs::OccupancyGridConstPtr current_occupancy_grid = ex->m_last_occupancy_grid;
    ex->m_last_occupancy_grid_mutex.unlock();

    ROS_INFO("Received a fresh costmap");

    //////////////////////////////////////////////////
    // get the pose of the object in the grid frame //
    //////////////////////////////////////////////////

    auto& grid_frame = current_occupancy_grid->header.frame_id;
    auto& model_frame = ex->m_robot_model->getModelFrame();

    Eigen::Affine3d T_grid_model;
    Eigen::Affine3d T_model_grid;

    if (grid_frame != ex->m_robot_model->getModelFrame()) {
        ROS_INFO("Lookup transform from model frame to grid frame");
        tf::StampedTransform t;

        try {
            ex->m_listener.lookupTransform(ex->m_robot_model->getModelFrame(), grid_frame, ros::Time(0), t);
            tf::transformTFToEigen(t, T_model_grid);
            T_grid_model = T_model_grid.inverse();
        } catch (const tf::TransformException& e) {
            ROS_ERROR("Failed to lookup transform from '%s' to '%s' (%s)", ex->m_robot_model->getModelFrame().c_str(), grid_frame.c_str(), e.what());
            cmu_manipulation_msgs::GraspObjectCommandResult result;
            result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
            std::string message = "Failed to check object pickup";
            ex->m_server->setAborted(result, message);
            return GraspObjectExecutionStatus::FAULT;
        }
    } else {
        T_model_grid = Eigen::Affine3d::Identity();
        T_grid_model = Eigen::Affine3d::Identity();
    }

    Eigen::Affine3d T_grid_obj = T_grid_model * ex->m_obj_pose;
    ROS_INFO("Object Pose [grid frame]: %s", to_string(T_grid_obj).c_str());

    //////////////////////////////////////
    // Check the costmap for the object //
    //////////////////////////////////////

    double success_pct = calcProbSuccessfulGrasp(
            *current_occupancy_grid,
            T_grid_obj.translation()[0],
            T_grid_obj.translation()[1],
            ex->m_object_filter_radius);

    success_pct = clamp(success_pct, 0.0, 1.0);
    if (success_pct == 1.0) {
        ROS_WARN("Mighty confident now, aren't we?");
    } else {
        ROS_WARN("Grasped object with %0.3f %% confidence", 100.0 * success_pct);
    }

    if (success_pct > ex->m_gas_can_detection_threshold) {
        cmu_manipulation_msgs::GraspObjectCommandResult result;
        result.result = cmu_manipulation_msgs::GraspObjectCommandResult::SUCCESS;
        ex->m_server->setSucceeded(result);
        return GraspObjectExecutionStatus::IDLE;
    }

    std::string message = "It appears that we have likely not grasped the object";
    ROS_WARN("%s", message.c_str());
    cmu_manipulation_msgs::GraspObjectCommandResult result;
    result.result = cmu_manipulation_msgs::GraspObjectCommandResult::EXECUTION_FAILED;
    ex->m_server->setAborted(result, message);
    return GraspObjectExecutionStatus::FAULT;
}

using StateMachineDoFn = GraspObjectExecutionStatus (*)(GraspObjectExecutor* user);
using StateMachineEnterFn = void (*)(GraspObjectExecutor* user, GraspObjectExecutionStatus next);
using StateMachineExitFn = void (*)(GraspObjectExecutor* user, GraspObjectExecutionStatus prev);

struct StateMachineState
{
    StateMachineDoFn    do_fn       = NULL;
    StateMachineEnterFn enter_fn    = NULL;
    StateMachineExitFn  exit_fn     = NULL;
};

} // namespace rcta

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "object_pickup_executor");

    using namespace rcta;

    StateMachineState smach_states[GRASP_OBJECT_EXECUTION_STATUS_COUNT];

    smach_states[GraspObjectExecutionStatus::IDLE].do_fn = OnIdle;

    smach_states[GraspObjectExecutionStatus::FAULT].do_fn = OnFault;

    smach_states[GraspObjectExecutionStatus::GENERATING_GRASPS].do_fn = DoGenerateGrasps;

    smach_states[GraspObjectExecutionStatus::PRESHAPE_HAND].do_fn = DoPreshapeHand;

    smach_states[GraspObjectExecutionStatus::MOVING_ARM_TO_PREGRASP].do_fn = DoMoveArmToPreGrasp;

    smach_states[GraspObjectExecutionStatus::MOVING_ARM_TO_GRASP].do_fn = DoMoveArmToGrasp;

    smach_states[GraspObjectExecutionStatus::OPENING_GRIPPER].do_fn = DoOpenGripper;

    smach_states[GraspObjectExecutionStatus::GRASPING_OBJECT].do_fn = DoGraspObject;

    smach_states[GraspObjectExecutionStatus::RETRACTING_GRIPPER].do_fn = DoRetractGripper;

    smach_states[GraspObjectExecutionStatus::MOVING_ARM_TO_STOW].enter_fn = DoMoveArmToStowEnter;
    smach_states[GraspObjectExecutionStatus::MOVING_ARM_TO_STOW].do_fn = DoMoveArmToStow;
    smach_states[GraspObjectExecutionStatus::MOVING_ARM_TO_STOW].exit_fn = DoMoveArmToStowExit;

    smach_states[GraspObjectExecutionStatus::COMPLETING_GOAL].enter_fn = DoCompleteGoalEnter;
    smach_states[GraspObjectExecutionStatus::COMPLETING_GOAL].do_fn = DoCompleteGoal;

    GraspObjectExecutor executor;
    if (!executor.initialize()) {
        ROS_ERROR("Failed to initialize Grasp Object Executor");
        return 1;
    }

    ros::AsyncSpinner spinner(4);
    spinner.start();

    auto prev_status = GraspObjectExecutionStatus::IDLE;
    auto status = GraspObjectExecutionStatus::IDLE;
    auto next_status = GraspObjectExecutionStatus::IDLE;

    ros::Rate loop_rate(10.0);
    while (ros::ok()) {
        if (status != prev_status) {
            ROS_INFO("Grasp Job Executor Transitioning: %s -> %s", to_string(prev_status).c_str(), to_string(status).c_str());
            if (smach_states[status].enter_fn) {
                smach_states[status].enter_fn(&executor, prev_status);
            }

            prev_status = status;
        }

        // publish feedback status for active goals
        if (executor.m_server->isActive()) {
            cmu_manipulation_msgs::GraspObjectCommandFeedback feedback;
            feedback.status = executionStatusToFeedbackStatus(status);
            if (feedback.status != 255) {
                executor.m_server->publishFeedback(feedback);
            }
        }

        next_status = smach_states[status].do_fn(&executor);

        if (next_status != status) {
            if (smach_states[status].exit_fn) {
                smach_states[status].exit_fn(&executor, next_status);
            }

            status = next_status;
        }

        loop_rate.sleep();
    }

    spinner.stop();
    return 0;
}
