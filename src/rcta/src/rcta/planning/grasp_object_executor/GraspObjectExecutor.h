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
#include <leatherman/print.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <sbpl_arm_planner/visualizer_ros.h>
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
    OPENING_GRIPPER,
    EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP,
    EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP,
    MOVING_ARM_TO_GRASP,
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
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher filtered_costmap_pub_;
    ros::Publisher extrusion_octomap_pub_;
    ros::Subscriber costmap_sub_;

    tf::TransformListener listener_;

    std::unique_ptr<GraspObjectCommandActionServer> as_;
    std::unique_ptr<MoveArmActionClient> move_arm_command_client_;
    std::unique_ptr<ViservoCommandActionClient> viservo_command_client_;
    std::unique_ptr<GripperCommandActionClient> gripper_command_client_;
    std::string action_name_;
    std::string move_arm_command_action_name_;
    std::string viservo_command_action_name_;
    std::string gripper_command_action_name_;

    sbpl::VisualizerROS m_viz;
    ///@}

    robot_model_loader::RobotModelLoaderPtr m_rml;
    moveit::core::RobotModelPtr m_robot_model;
    hdt::RobotModelConstPtr robot_model_;
    CostmapExtruder extruder_;
    rcta::GascanGraspPlanner m_grasp_planner;
    planning_scene_monitor::PlanningSceneMonitorPtr m_scene_monitor;

    /// \name Global Parameters
    ///@{

    std::string m_camera_view_frame;
    std::string m_manip_name;
    moveit::core::JointModelGroup* m_manip_group;

    /// Circumscribed radius of the object, used to remove object cells from
    /// the occupancy grid and extruded octomap
    double object_filter_radius_m_;

    /// Whether to override incoming octomaps with an extruded costmap variant
    bool use_extrusion_octomap_;

    bool skip_viservo_;

    ///@}

    /// \name GenerateGrasps Parameters
    ///@{
    std::vector<AttachedMarker> attached_markers_;
    ///@}

    /// \name PlanArmMotionToPregrasp Parameters
    ///@{
    int max_grasp_candidates_;
    ///@}

    /// \name PlanArmMotionToStow Parameters
    ///@{
    std::vector<StowPosition> stow_positions_;
    ///@}

    /// \name CompleteGoal Parameters
    ///@{
    double gas_can_detection_threshold_;
    ///@}

    /// \name Goal Context
    ///@{

    rcta_msgs::GraspObjectCommandGoal::ConstPtr current_goal_;

    /// copy of most recent OccupancyGrid message when the goal was received
    OccupancyGridPtr current_occupancy_grid_;

    /// extruded current occupancy grid
    OctomapPtr current_octomap_;

    geometry_msgs::PoseStamped gas_can_in_grid_frame_;

    ///@}

    /// \name Internal state machine for move arm goal stages
    ///@{
    bool sent_move_arm_goal_;
    bool pending_move_arm_command_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    rcta::MoveArmResult::ConstPtr move_arm_command_result_;
    ///@}

    /// \name Internal state machine for viservo goal stages
    ///@{
    bool sent_viservo_command_;
    bool pending_viservo_command_;
    actionlib::SimpleClientGoalState viservo_command_goal_state_;
    rcta::ViservoCommandResult::ConstPtr viservo_command_result_;
    ///@}

    /// \name Internal state machine for gripper goal stages
    ///@{
    bool sent_gripper_command_;
    bool pending_gripper_command_;
    actionlib::SimpleClientGoalState gripper_command_goal_state_;
    control_msgs::GripperCommandResult::ConstPtr gripper_command_result_;
    ///@}

    OccupancyGridConstPtr last_occupancy_grid_; ///< most recent OccupancyGrid message

    /// \name Shared State
    ///@{

    // shared(GenerateGrasps, PlanArmMotionToPregrasp)
    // -> to plan to a number of different grasps, ranked by graspability
    std::vector<rcta::GraspCandidate> reachable_grasp_candidates_;

    // shared(PlanArmMotionToPregrasp, ExecuteVisualServoMotionToPregrasp)
    // -> to enforce visual servo to the same pose
    rcta::MoveArmGoal last_move_arm_pregrasp_goal_;

    // shared(PlanArmMotionToPregrasp, PlanArmMotionToStow)
    // -> to know how to attach the object to the arm
    rcta::GraspCandidate last_successful_grasp_;

    // shared(ExecuteVisualServoMotionToPregrasp, ExecuteVisualServoMotionToGrasp)
    // -> propagate wrist goal originating from PlanArmMotionToPregrasp
    rcta::ViservoCommandGoal last_viservo_pregrasp_goal_;

    ///@}

    /// \name PlanArmMotionToStowPosition State
    ///@{
    int next_stow_position_to_attempt_;
    ///@}

    /// \name Completing state
    ///@{
    ros::Time wait_for_grid_start_time_;
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

    void cull_grasp_candidates(
        std::vector<rcta::GraspCandidate>& candidates,
        int max_candidates) const;

    visualization_msgs::MarkerArray
    getGraspCandidatesVisualization(
        const std::vector<rcta::GraspCandidate>& grasps,
        const std::string& ns) const;

    void clear_circle_from_grid(nav_msgs::OccupancyGrid& grid, double x, double y, double radius) const;
    bool within_bounds(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const;
    void grid_to_world(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y, double& world_x, double& world_y) const;
    void world_to_grid(const nav_msgs::OccupancyGrid& grid, double world_x, double world_y, int& grid_x, int& grid_y) const;
    std::int8_t& grid_at(nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const;
    const std::int8_t& grid_at(const nav_msgs::OccupancyGrid& grid, int grid_x, int grid_y) const;

    double calc_prob_successful_grasp(
            const nav_msgs::OccupancyGrid& grid,
            double circle_x,
            double circle_y,
            double radius) const;
};

#endif
