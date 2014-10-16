#ifndef ObjectPickupExecutor_h
#define ObjectPickupExecutor_h

#include <cstdint>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <leatherman/print.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <hdt_msgs/GraspObjectCommandAction.h>
#include <hdt/MoveArmCommandAction.h>
#include <hdt/ViservoCommandAction.h>
#include <hdt/common/geometry/nurb/NURB.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <hdt/simulation/costmap_extruder/CostmapExtruder.h>

namespace GraspObjectExecutionStatus
{

enum Status
{
    INVALID = -1,
    IDLE = 0,
    FAULT,
    GENERATING_GRASPS,
    PLANNING_ARM_MOTION_TO_PREGRASP,
    EXECUTING_ARM_MOTION_TO_PREGRASP,
    OPENING_GRIPPER,
    EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP,
    EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP,
    GRASPING_OBJECT,
    RETRACTING_GRIPPER,
    PLANNING_ARM_MOTION_TO_STOW_POSITION,
    EXECUTING_ARM_MOTION_TO_STOW_POSITION,
    COMPLETING_GOAL
};

std::string to_string(Status status);

}

class GraspObjectExecutor
{
    struct StowPosition
    {
        std::string name;
        std::vector<double> joint_positions;
    };

public:

    friend bool extract_xml_value(XmlRpc::XmlRpcValue& value, StowPosition&);

    GraspObjectExecutor();

    bool initialize();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    struct GraspCandidate
    {
        Eigen::Affine3d grasp_candidate_transform;
        double u;

        GraspCandidate(const Eigen::Affine3d& grasp_candidate_transform = Eigen::Affine3d::Identity(), double u = -1.0) :
            grasp_candidate_transform(grasp_candidate_transform),
            u(u) { }
    };

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Subscriber costmap_sub_;

    typedef nav_msgs::OccupancyGrid::ConstPtr OccupancyGridConstPtr;
    typedef nav_msgs::OccupancyGrid::Ptr OccupancyGridPtr;
    typedef octomap_msgs::Octomap::ConstPtr OctomapConstPtr;

    double object_filter_radius_m_;
    ros::Publisher filtered_costmap_pub_;

    OccupancyGridConstPtr last_occupancy_grid_; ///< most recent OccupancyGrid message
    OccupancyGridPtr current_occupancy_grid_; ///< copy of most recent OccupancyGrid message when the goal was received

    geometry_msgs::PoseStamped gas_can_in_grid_frame_;
    bool wait_for_after_grasp_grid_;
    ros::Time wait_for_grid_start_time_;
    OccupancyGridConstPtr occupancy_grid_after_grasp_;

    bool use_extrusion_octomap_; ///< Whether to override incoming octomaps with an extruded costmap variant
    OctomapConstPtr current_octomap_; ///< extruded current occupancy grid
    CostmapExtruder extruder_;
    ros::Publisher extrusion_octomap_pub_;

    hdt::RobotModelConstPtr robot_model_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<hdt_msgs::GraspObjectCommandAction> GraspObjectCommandActionServer;
    std::unique_ptr<GraspObjectCommandActionServer> as_;

    typedef actionlib::SimpleActionClient<hdt::MoveArmCommandAction> MoveArmCommandActionClient;
    std::string move_arm_command_action_name_;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    bool sent_move_arm_goal_;
    bool pending_move_arm_command_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    hdt::MoveArmCommandResult::ConstPtr move_arm_command_result_;

    hdt::MoveArmCommandGoal last_move_arm_pregrasp_goal_;
    hdt::MoveArmCommandGoal last_move_arm_stow_goal_;

    typedef actionlib::SimpleActionClient<hdt::ViservoCommandAction> ViservoCommandActionClient;
    std::string viservo_command_action_name_;
    std::unique_ptr<ViservoCommandActionClient> viservo_command_client_;
    bool sent_viservo_command_;
    bool pending_viservo_command_;
    actionlib::SimpleClientGoalState viservo_command_goal_state_;
    hdt::ViservoCommandResult::ConstPtr viservo_command_result_;

    hdt::ViservoCommandGoal last_viservo_pregrasp_goal_;

    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;
    std::string gripper_command_action_name_;
    std::unique_ptr<GripperCommandActionClient> gripper_command_client_;
    bool sent_gripper_command_;
    bool pending_gripper_command_;
    actionlib::SimpleClientGoalState gripper_command_goal_state_;
    control_msgs::GripperCommandResult::ConstPtr gripper_command_result_;

    hdt::ViservoCommandGoal last_viservo_grasp_goal_;

    int next_stow_position_to_attempt_;

    hdt_msgs::GraspObjectCommandGoal::ConstPtr current_goal_;

    GraspObjectExecutionStatus::Status status_;
    GraspObjectExecutionStatus::Status last_status_;

    std::unique_ptr<Nurb<Eigen::Vector3d>> grasp_spline_;

    std::string gas_can_mesh_path_;
    double gas_can_scale_;

    Eigen::Affine3d wrist_to_tool_;
    double pregrasp_to_grasp_offset_m_;
    Eigen::Affine3d grasp_to_pregrasp_;

    tf::TransformListener listener_;
    std::vector<GraspCandidate> reachable_grasp_candidates_;

    ros::Publisher marker_arr_pub_;

    std::vector<StowPosition> stow_positions_;

    double gas_can_detection_threshold_;

    int max_grasp_candidates_;

    void goal_callback();
    void preempt_callback();

    void move_arm_command_active_cb();
    void move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback);
    void move_arm_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::MoveArmCommandResult::ConstPtr& result);

    void viservo_command_active_cb();
    void viservo_command_feedback_cb(const hdt::ViservoCommandFeedback::ConstPtr& feedback);
    void viservo_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::ViservoCommandResult::ConstPtr& result);

    void gripper_command_active_cb();
    void gripper_command_feedback_cb(const control_msgs::GripperCommandFeedback::ConstPtr& feedback);
    void gripper_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const control_msgs::GripperCommandResult::ConstPtr& result);

    void occupancy_grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    template <typename ActionType>
    bool wait_for_action_server(
        std::unique_ptr<actionlib::SimpleActionClient<ActionType>>& action_client,
        const std::string& action_name,
        const ros::Duration& poll_duration,
        const ros::Duration& timeout)
    {
        ROS_INFO_PRETTY("Waiting for action server '%s'", action_name.c_str());

        if (!action_client) {
            ROS_WARN_PRETTY("Action client is null");
            return false;
        }

        ros::Time start = ros::Time::now();
        while (timeout == ros::Duration(0) || ros::Time::now() < start + timeout) {
            ros::spinOnce();
            if (!action_client->isServerConnected()) {
                action_client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
                if (!action_client) {
                    ROS_WARN_PRETTY("Failed to reinstantiate action client '%s'", action_name.c_str());
                    return false;
                }
            }

            if (action_client->isServerConnected()) {
                return true;
            }

            poll_duration.sleep();

            ROS_INFO_PRETTY("Waited %0.3f seconds for action server '%s'...", (ros::Time::now() - start).toSec(), action_name.c_str());
        }

        return false;
    }

    uint8_t execution_status_to_feedback_status(GraspObjectExecutionStatus::Status status);

    /// @brief Sample uniformly along the grasp spline to produce pregrasp poses for the wrist
    std::vector<GraspCandidate> sample_grasp_candidates(const Eigen::Affine3d& robot_to_object, int num_candidates) const;
    void visualize_grasp_candidates(const std::vector<GraspCandidate>& grasps) const;

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
