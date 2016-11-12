#ifndef ObjectPickupExecutor_h
#define ObjectPickupExecutor_h

// standard includes
#include <cstdint>
#include <memory>
#include <string>

// system includes
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <smpl/debug/visualizer_ros.h>
#include <spellbook/costmap_extruder/CostmapExtruder.h>
#include <spellbook/geometry/nurb/NURB.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rcta_msgs/GraspObjectCommandAction.h>

// project includes
#include <rcta/MoveArmAction.h>
#include <rcta/ViservoCommandAction.h>
#include <rcta/common/hdt_description/RobotModel.h>
#include <rcta/planning/grasping/gascan_grasp_planner.h>

namespace GraspObjectExecutionStatus {

enum Status
{
    INVALID = -1,
    IDLE = 0,
    FAULT,
    GENERATING_GRASPS,
    MOVING_ARM_TO_PREGRASP,
    MOVING_ARM_TO_GRASP,
    OPENING_GRIPPER,
    EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP,
    EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP,
    GRASPING_OBJECT,
    RETRACTING_GRIPPER,
    MOVING_ARM_TO_STOW,
    COMPLETING_GOAL
};

std::string to_string(Status status);

} // namespace GraspObjectExecutionStatus

struct StowPosition
{
    std::string name;
    std::map<std::string, double> joint_positions;
};

bool extract_xml_value(XmlRpc::XmlRpcValue& value, StowPosition& stow_position);

class GraspObjectExecutor
{
public:

    GraspObjectExecutor();
    ~GraspObjectExecutor();

    bool initialize();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    struct AttachedMarker
    {
        int marker_id;
        std::string attached_link;
        Eigen::Affine3d link_to_marker;
    };

    typedef nav_msgs::OccupancyGrid::ConstPtr OccupancyGridConstPtr;
    typedef nav_msgs::OccupancyGrid::Ptr OccupancyGridPtr;
    typedef octomap_msgs::Octomap::Ptr OctomapPtr;
    typedef octomap_msgs::Octomap::ConstPtr OctomapConstPtr;

    typedef actionlib::SimpleActionServer<rcta_msgs::GraspObjectCommandAction> GraspObjectCommandActionServer;
    typedef actionlib::SimpleActionClient<rcta::MoveArmAction> MoveArmActionClient;
    typedef actionlib::SimpleActionClient<rcta::ViservoCommandAction> ViservoCommandActionClient;
    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;

    /// \name ROS stuff
    ///@{
    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;

    ros::Publisher m_filtered_costmap_pub;
    ros::Publisher m_extrusion_octomap_pub;
    ros::Publisher m_attach_obj_pub;
    ros::Subscriber m_costmap_sub;

    tf::TransformListener m_listener;

    std::unique_ptr<GraspObjectCommandActionServer> m_as;
    std::unique_ptr<MoveArmActionClient> m_move_arm_command_client;
    std::unique_ptr<ViservoCommandActionClient> m_viservo_command_client;
    std::unique_ptr<GripperCommandActionClient> m_gripper_command_client;
    std::string m_action_name;
    std::string m_move_arm_command_action_name;
    std::string m_viservo_command_action_name;
    std::string m_gripper_command_action_name;

    std::unique_ptr<ros::ServiceClient> m_check_state_validity_client;

    sbpl::VisualizerROS m_viz;
    ///@}

    robot_model_loader::RobotModelLoaderPtr m_rml;
    moveit::core::RobotModelPtr m_robot_model;
    CostmapExtruder m_extruder;
    rcta::GascanGraspPlanner m_grasp_planner;
    planning_scene_monitor::PlanningSceneMonitorPtr m_scene_monitor;
    std::vector<std::string> m_gripper_links;

    /// \name Global Parameters
    ///@{

    std::string m_camera_view_frame;
    std::string m_manip_name;
    moveit::core::JointModelGroup* m_manip_group;

    /// Circumscribed radius of the object, used to remove object cells from
    /// the occupancy grid and extruded octomap
    double m_object_filter_radius;

    /// Whether to override incoming octomaps with an extruded costmap variant
    bool m_use_extrusion_octomap;

    bool m_skip_viservo;
    bool m_attach_object;

    ///@}

    /// \name GenerateGrasps Parameters
    ///@{
    std::vector<AttachedMarker> m_attached_markers;
    ///@}

    /// \name MoveArmToPregrasp Parameters
    ///@{
    int m_max_grasp_candidates;
    ///@}

    /// \name MoveArmToStow Parameters
    ///@{
    std::vector<std::vector<StowPosition>> m_stow_sequences;
    ros::Duration m_attach_obj_req_wait;
    ///@}

    /// \name CompleteGoal Parameters
    ///@{
    double m_gas_can_detection_threshold;
    ///@}

    /// \name Goal Context
    ///@{

    rcta_msgs::GraspObjectCommandGoal::ConstPtr m_current_goal;

    /// copy of most recent OccupancyGrid message when the goal was received
    OccupancyGridPtr m_current_occupancy_grid;

    /// extruded current occupancy grid
    OctomapPtr m_current_octomap;

    Eigen::Affine3d m_obj_pose;
    Eigen::Affine3d m_T_grid_model;
    Eigen::Affine3d m_T_model_grid;

    ///@}

    /// \name Internal state machine for move arm goal stages
    ///@{
    bool m_sent_move_arm_goal;
    bool m_pending_move_arm_command;
    actionlib::SimpleClientGoalState m_move_arm_command_goal_state;
    rcta::MoveArmResult::ConstPtr m_move_arm_command_result;
    ///@}

    /// \name Internal state machine for viservo goal stages
    ///@{
    bool m_sent_viservo_command;
    bool m_pending_viservo_command;
    actionlib::SimpleClientGoalState m_viservo_command_goal_state;
    rcta::ViservoCommandResult::ConstPtr m_viservo_command_result;
    ///@}

    /// \name Internal state machine for gripper goal stages
    ///@{
    bool m_sent_gripper_command;
    bool m_pending_gripper_command;
    actionlib::SimpleClientGoalState m_gripper_command_goal_state;
    control_msgs::GripperCommandResult::ConstPtr m_gripper_command_result;
    ///@}

    OccupancyGridConstPtr m_last_occupancy_grid; ///< most recent OccupancyGrid message

    /// \name Shared State
    ///@{

    // shared(GenerateGrasps, MoveArmToPregrasp)
    // -> to plan to a number of different grasps, ranked by graspability
    std::vector<rcta::GraspCandidate> m_reachable_grasp_candidates;

    // shared(MoveArmToPregrasp, ExecuteVisualServoMotionToPregrasp)
    // shared(MoveArmToPregrasp, MoveArmToGrasp)
    // -> to enforce visual servo to the same pose
    rcta::MoveArmGoal m_last_move_arm_pregrasp_goal;

    // shared(MoveArmToPregrasp, MoveArmToStow)
    // -> to know how to attach the object to the arm
    rcta::GraspCandidate m_last_successful_grasp;

    // shared(ExecuteVisualServoMotionToPregrasp, ExecuteVisualServoMotionToGrasp)
    // -> propagate wrist goal originating from MoveArmToPregrasp
    rcta::ViservoCommandGoal m_last_viservo_pregrasp_goal;

    ///@}

    /// \name MoveArmToStowPosition State
    ///@{
    int m_next_stow_sequence;
    int m_next_stow_position;
    ros::Time m_attach_obj_req_time;
    ///@}

    /// \name Completing state
    ///@{
    ros::Time m_wait_for_grid_start_time;
    ///@}

    bool downloadMarkerParams();

    const moveit::core::RobotState& currentRobotState() const;
    void processSceneUpdate(
        planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);

    void goalCallback();
    void preemptCallback();

    void onIdleEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onIdle();
    void onIdleExit(GraspObjectExecutionStatus::Status to);

    void onFaultEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onFault();
    void onFaultExit(GraspObjectExecutionStatus::Status to);

    void onGeneratingGraspsEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onGeneratingGrasps();
    void onGeneratingGraspsExit(GraspObjectExecutionStatus::Status to);

    void onMovingArmToPregraspEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onMovingArmToPregrasp();
    void onMovingArmToPregraspExit(GraspObjectExecutionStatus::Status to);

    void onMovingArmToGraspEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onMovingArmToGrasp();
    void onMovingArmToGraspExit(GraspObjectExecutionStatus::Status to);

    void onOpeningGripperEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onOpeningGripper();
    void onOpeningGripperExit(GraspObjectExecutionStatus::Status to);

    void onExecutingVisualServoMotionToPregraspEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onExecutingVisualServoMotionToPregrasp();
    void onExecutingVisualServoMotionToPregraspExit(GraspObjectExecutionStatus::Status to);

    void onExecutingVisualServoMotionToGraspEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onExecutingVisualServoMotionToGrasp();
    void onExecutingVisualServoMotionToGraspExit(GraspObjectExecutionStatus::Status to);

    void onGraspingObjectEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onGraspingObject();
    void onGraspingObjectExit(GraspObjectExecutionStatus::Status to);

    void onRetractingGripperEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onRetractingGripper();
    void onRetractingGripperExit(GraspObjectExecutionStatus::Status to);

    void onMovingArmToStowEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onMovingArmToStow();
    void onMovingArmToStowExit(GraspObjectExecutionStatus::Status to);

    void onCompletingGoalEnter(GraspObjectExecutionStatus::Status from);
    GraspObjectExecutionStatus::Status onCompletingGoal();
    void onCompletingGoalExit(GraspObjectExecutionStatus::Status to);

    void moveArmResultCallback(
            const actionlib::SimpleClientGoalState& state,
            const rcta::MoveArmResult::ConstPtr& result);

    void viservoCommandResultCallback(
            const actionlib::SimpleClientGoalState& state,
            const rcta::ViservoCommandResult::ConstPtr& result);

    void gripperCommandResultCallback(
            const actionlib::SimpleClientGoalState& state,
            const control_msgs::GripperCommandResult::ConstPtr& result);

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    uint8_t executionStatusToFeedbackStatus(GraspObjectExecutionStatus::Status status);

    void pruneGraspCandidates(
        std::vector<rcta::GraspCandidate>& candidates,
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& camera_pose,
        double marker_incident_angle_threshold_rad) const;

    void pruneGraspCandidatesIK(
        std::vector<rcta::GraspCandidate>& candidates,
        const Eigen::Affine3d& T_grasp_robot) const;

    void limitGraspCandidates(
        std::vector<rcta::GraspCandidate>& candidates,
        int max_candidates) const;

    visualization_msgs::MarkerArray
    getGraspCandidatesVisualization(
        const std::vector<rcta::GraspCandidate>& grasps,
        const std::string& ns) const;

    void clearCircleFromGrid(nav_msgs::OccupancyGrid& grid, double x, double y, double radius) const;
    bool within_bounds(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const;
    void grid_to_world(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y, double& world_x, double& world_y) const;
    void world_to_grid(const nav_msgs::OccupancyGrid& grid, double world_x, double world_y, int& grid_x, int& grid_y) const;
    std::int8_t& grid_at(nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const;
    const std::int8_t& grid_at(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const;

    double calcProbSuccessfulGrasp(
            const nav_msgs::OccupancyGrid& grid,
            double circle_x,
            double circle_y,
            double radius) const;

    void transformCollisionObject(
        moveit_msgs::CollisionObject& o,
        const Eigen::Affine3d& t) const;
};

#endif
