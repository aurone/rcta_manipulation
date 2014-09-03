#ifndef ObjectPickupExecutor_h
#define ObjectPickupExecutor_h

#include <cstdint>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/ros.h>
#include <hdt/GraspObjectAction.h>
#include <hdt/MoveArmCommandAction.h>
#include <hdt/ViservoCommandAction.h>
#include <hdt/common/geometry/nurb/NURB.h>

#define TEST_STATE_MACHINE 0

namespace GraspObjectExecutionStatus
{

enum Status
{
    INVALID = -1,
    IDLE = 0,
    PLANNING_ARM_MOTION_TO_PREGRASP,
    EXECUTING_ARM_MOTION_TO_PREGRASP,
    EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP,
    EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP,
    GRASPING_OBJECT,
    PLANNING_ARM_MOTION_TO_STOW_POSITION,
    EXECUTING_ARM_MOTION_TO_STOW_POSITION,
    COMPLETING_GOAL
};

std::string to_string(Status status);

}


class GraspObjectExecutor
{
public:

    GraspObjectExecutor();

    bool initialize();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<hdt::GraspObjectAction> GraspObjectActionServer;
    std::unique_ptr<GraspObjectActionServer> as_;

    typedef actionlib::SimpleActionClient<hdt::MoveArmCommandAction> MoveArmCommandActionClient;
    std::string move_arm_command_action_name_;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    bool sent_move_arm_goal_;
    bool pending_move_arm_command_;

    typedef actionlib::SimpleActionClient<hdt::ViservoCommandAction> ViservoCommandActionClient;
    std::string viservo_command_action_name_;
    std::unique_ptr<ViservoCommandActionClient> viservo_command_client_;
    bool sent_viservo_command_;
    bool pending_viservo_command_;

    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;
    std::string gripper_command_action_name_;
    std::unique_ptr<GripperCommandActionClient> gripper_command_client_;
    bool sent_gripper_command_;
    bool pending_gripper_command_;

    hdt::GraspObjectGoal::ConstPtr current_goal_;

    GraspObjectExecutionStatus::Status status_;
    GraspObjectExecutionStatus::Status last_status_;

    std::unique_ptr<Nurb<Eigen::Vector3d>> grasp_spline_;

    const double gas_can_scale_;

    Eigen::Affine3d wrist_to_tool_;
    double pregrasp_to_grasp_offset_m_;

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
            const control_msgs::GripperCommandResult& result);

    template <typename ActionType>
    bool wait_for_action_server(
        std::unique_ptr<actionlib::SimpleActionClient<ActionType>>& action_client,
        const std::string& action_name,
        const ros::Duration& poll_duration,
        const ros::Duration& timeout)
    {
        if (!action_client) {
            ROS_WARN("Action client is null");
            return false;
        }

        ros::Time start = ros::Time::now();
        while (timeout == ros::Duration(0) || ros::Time::now() < start + timeout) {
            if (!action_client->isServerConnected()) {
                action_client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
                if (!action_client) {
                    return false;
                }
            }

            if (action_client->isServerConnected()) {
                return true;
            }

            poll_duration.sleep();
        }

        return false;
    }

    uint8_t execution_status_to_feedback_status(GraspObjectExecutionStatus::Status status);
};

#endif
