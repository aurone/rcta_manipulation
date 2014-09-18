#ifndef RetrieveObjectSimulator_h
#define RetrieveObjectSimulator_h

#include <memory>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <hdt_msgs/GraspObjectCommandAction.h>
#include <hdt_msgs/RepositionBaseCommandAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <hdt/TeleportAndaliteCommandAction.h>
#include <hdt/TeleportHDTCommandAction.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include "CollisionModel2.h"

namespace RetrieveObjectExecutionStatus
{

enum Status
{
    INVALID = -1,
    INITIALIZING,
    PLANNING_REPOSITION_BASE,
    EXECUTING_REPOSITION_BASE,
    GRASPING_OBJECT,
    COMPLETE
};

inline std::string to_string(Status status)
{
    switch (status) {
    case INVALID:
        return "Invalid";
    case INITIALIZING:
        return "Idle";
    case PLANNING_REPOSITION_BASE:
        return "PlanningRepositionBase";
    case EXECUTING_REPOSITION_BASE:
        return "ExecutingRepositionBase";
    case GRASPING_OBJECT:
        return "GraspingObject";
    case COMPLETE:
        return "Complete";
    default:
        return "InvalidStatus";
    }
}

} // namespace RetrieveObjectExecutionStatus

class RetrieveObjectSimulator
{
public:

    RetrieveObjectSimulator();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };

    bool initialize();

    /// Test suite to conduct a series of queries to the retrieve-object pipeline for robustness
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    RetrieveObjectExecutionStatus::Status last_status_;
    RetrieveObjectExecutionStatus::Status status_;

    hdt::RobotModelPtr robot_model_;

    /// @{ Configured Environment Setup
    // the initial pose of the robot in the map frame to be used for every reposition base query
    geometry_msgs::PoseStamped initial_robot_pose_; // in the world frame
    std::vector<double> initial_robot_joint_values_;
    std::string world_frame_;
    Eigen::Affine3d world_to_room_;
    Eigen::Affine3d object_to_footprint_;
    /// @}

    /// @{ object pose sampling parameters
    double room_length_m_;
    double room_width_m_;
    int num_disc_x_;
    int num_disc_y_;
    int num_disc_yaw_;
    /// @}

    ros::Subscriber occupancy_grid_sub_;
    nav_msgs::OccupancyGrid::ConstPtr last_occupancy_grid_;

    /// @{ Action Clients and State
    typedef actionlib::SimpleActionClient<hdt_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionClient;
    std::unique_ptr<RepositionBaseCommandActionClient> reposition_base_command_client_;
    bool sent_reposition_base_command_;
    bool pending_reposition_base_command_;

    actionlib::SimpleClientGoalState last_reposition_base_goal_state_;
    hdt_msgs::RepositionBaseCommandResult::ConstPtr last_reposition_base_result_;

    typedef actionlib::SimpleActionClient<hdt::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionClient;
    std::unique_ptr<TeleportAndaliteCommandActionClient> teleport_andalite_command_client_;
    bool sent_teleport_andalite_command_;
    bool pending_teleport_andalite_command_;

    actionlib::SimpleClientGoalState last_teleport_andalite_goal_state_;
    hdt::TeleportAndaliteCommandResult::ConstPtr last_teleport_andalite_result_;

    typedef actionlib::SimpleActionClient<hdt::TeleportHDTCommandAction> TeleportHDTCommandActionClient;
    std::unique_ptr<TeleportHDTCommandActionClient> teleport_hdt_command_client_;
    bool sent_teleport_hdt_command_;
    bool pending_teleport_hdt_command_;

    actionlib::SimpleClientGoalState last_teleport_hdt_goal_state_;
    hdt::TeleportHDTCommandResult::ConstPtr last_teleport_hdt_result_;

    typedef actionlib::SimpleActionClient<hdt_msgs::GraspObjectCommandAction> GraspObjectCommandActionClient;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool sent_grasp_object_command_;
    bool pending_grasp_object_command_;

    actionlib::SimpleClientGoalState last_grasp_object_goal_state_;
    hdt_msgs::GraspObjectCommandResult::ConstPtr last_grasp_object_result_;
    /// @}

    /// @{ Currently-being-processed samples/candidates
    std::vector<geometry_msgs::PoseStamped> sample_object_poses_;   ///< stack of object poses in the 'room' frame
    std::vector<geometry_msgs::PoseStamped> candidate_base_poses_;  ///< stack of base poses in the 'world' frame

    geometry_msgs::PoseStamped current_sample_object_pose_;
    geometry_msgs::PoseStamped current_candidate_base_pose_;
    /// @}

    void reposition_base_active_cb();
    void reposition_base_feedback_cb(const hdt_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback);
    void reposition_base_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::RepositionBaseCommandResult::ConstPtr& result);

    void teleport_andalite_active_cb();
    void teleport_andalite_feedback_cb(const hdt::TeleportAndaliteCommandFeedback::ConstPtr& feedback);
    void teleport_andalite_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::TeleportAndaliteCommandResult::ConstPtr& result);

    void teleport_hdt_active_cb();
    void teleport_hdt_feedback_cb(const hdt::TeleportHDTCommandFeedback::ConstPtr& feedback);
    void teleport_hdt_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::TeleportHDTCommandResult::ConstPtr& result);

    void grasp_object_active_cb();
    void grasp_object_feedback_cb(const hdt_msgs::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::GraspObjectCommandResult::ConstPtr& result);

    std::vector<geometry_msgs::PoseStamped> create_sample_object_poses() const;

    // TODO: Duplicate from GraspObjectExecutor.cpp and ManipulatorCommandPanel.cpp
    template <typename ActionType>
    bool wait_for_action_server(
        std::unique_ptr<actionlib::SimpleActionClient<ActionType>>& client,
        const std::string& action_name,
        const ros::Duration& poll_duration,
        const ros::Duration& timeout)
    {
        if (!client) {
            client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
        }

        ROS_DEBUG("Waiting for action server '%s'", action_name.c_str());

        if (!client) {
            ROS_WARN("Action client is null");
            return false;
        }

        ros::Time start = ros::Time::now();
        while (ros::ok() && (timeout == ros::Duration(0) || ros::Time::now() < start + timeout)) {
            ros::spinOnce();
            if (!client->isServerConnected()) {
                client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
                if (!client) {
                    ROS_WARN("Failed to reinstantiate action client '%s'", action_name.c_str());
                    return false;
                }
            }

            if (client->isServerConnected()) {
                return true;
            }

            poll_duration.sleep();

            ROS_DEBUG("Waited %0.3f seconds for action server '%s'...", (ros::Time::now() - start).toSec(), action_name.c_str());
        }

        return false;
    }

    void occupancy_grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    std::vector<geometry_msgs::PoseStamped>
    collision_check_object_poses(const std::vector<geometry_msgs::PoseStamped>& sample_poses);
};

#endif
