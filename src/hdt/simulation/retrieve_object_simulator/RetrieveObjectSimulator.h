#ifndef RetrieveObjectSimulator_h
#define RetrieveObjectSimulator_h

#include <memory>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <hdt_msgs/GraspObjectCommandAction.h>
#include <hdt_msgs/RepositionBaseCommandAction.h>
#include <hdt/TeleportAndaliteCommandAction.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/utils/RunUponDestruction.h>

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
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    RetrieveObjectExecutionStatus::Status last_status_;
    RetrieveObjectExecutionStatus::Status status_;

    // the initial pose of the robot in the map frame to be used for every reposition base query
    geometry_msgs::PoseStamped initial_robot_pose_;
    std::string world_frame_;

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

    typedef actionlib::SimpleActionClient<hdt_msgs::GraspObjectCommandAction> GraspObjectCommandActionClient;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool sent_grasp_object_command_;
    bool pending_grasp_object_command_;

    actionlib::SimpleClientGoalState last_grasp_object_goal_state_;
    hdt_msgs::GraspObjectCommandResult::ConstPtr last_grasp_object_result_;

    std::vector<geometry_msgs::PoseStamped> sample_object_poses_;
    std::vector<geometry_msgs::PoseStamped> candidate_base_poses_;

    geometry_msgs::PoseStamped current_sample_object_pose_;
    geometry_msgs::PoseStamped current_candidate_base_pose_;

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

    void grasp_object_active_cb();
    void grasp_object_feedback_cb(const hdt_msgs::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::GraspObjectCommandResult::ConstPtr& result);

    std::vector<geometry_msgs::PoseStamped> create_sample_object_poses();
};

#endif
