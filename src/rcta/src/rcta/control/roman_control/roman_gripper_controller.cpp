#include "roman_gripper_controller.h"

#include <rcta/control/robotiq_controllers/gripper_model.h>

#include <roman_client_ros_utils/RobotiqActivate.h>
#include <roman_client_ros_utils/RobotiqGenericCmd.h>
#include <roman_client_ros_utils/RobotiqSimpleCmd.h>

RomanGripperController::RomanGripperController() :
    m_nh(),
    m_server(ros::NodeHandle("right_gripper"), "gripper_action", false),
    m_hand_activate_pub(),
    m_hand_goto_pub(),
    m_hand_open_pub(),
    m_hand_close_pub(),
    m_roman_state_sub(),
    m_goal(),
    m_feedback(),
    m_result(),
    m_state()
{
    m_hand_activate_pub = m_nh.advertise<roman_client_ros_utils::RobotiqActivate>(
            "roman_hand_activate", 1);
    m_hand_goto_pub = m_nh.advertise<roman_client_ros_utils::RobotiqGenericCmd>(
            "roman_hand_goto", 1);
    m_hand_open_pub = m_nh.advertise<roman_client_ros_utils::RobotiqSimpleCmd>(
            "roman_hand_open", 1);
    m_hand_close_pub = m_nh.advertise<roman_client_ros_utils::RobotiqSimpleCmd>(
            "roman_hand_close", 1);

    m_roman_state_sub = m_nh.subscribe("roman_state", 1, &RomanGripperController::romanStateCallback, this);

    m_server.registerGoalCallback(boost::bind(&RomanGripperController::goalCallback, this));
    m_server.registerPreemptCallback(boost::bind(&RomanGripperController::preemptCallback, this));
}

RomanGripperController::~RomanGripperController()
{
    if (m_server.isActive()) {
        m_server.setAborted();
    }
    m_server.shutdown();
}

int RomanGripperController::run()
{
    m_server.start();
    ros::spin();
    return 0;
}

void RomanGripperController::goalCallback()
{
    ROS_INFO("Received gripper command goal");
    m_goal = m_server.acceptNewGoal();
    if (m_server.isPreemptRequested()) {

    }

    roman_client_ros_utils::RobotiqSimpleCmd cmd;
    cmd.utime = ros::Time::now().toNSec() / 1e3;
    cmd.mode = roman_client_ros_utils::RobotiqSimpleCmd::MODE_BASIC;
    cmd.mech = roman_client_ros_utils::RobotiqSimpleCmd::MECH_RIGHT;
    if (m_goal->command.position == GripperModel().maximum_width()) {
        cmd.close = false;
        m_hand_open_pub.publish(cmd);

        m_result.position = GripperModel().maximum_width();
//        m_result.effort;
        m_result.stalled = false;
        m_result.reached_goal = true;
        m_server.setSucceeded(m_result);
    } else if (m_goal->command.position == GripperModel().minimum_width()) {
        cmd.close = true;
        m_hand_close_pub.publish(cmd);

        m_result.position = GripperModel().minimum_width();
//        m_result.effort;
        m_result.stalled = false;
        m_result.reached_goal = true;
        m_server.setSucceeded(m_result);
    } else {
        ROS_WARN("Roman Gripper Controller currently only accepts binary open/close input represented as { %0.3f, %0.3f }", GripperModel().minimum_width(), GripperModel().maximum_width());
//        m_result.position;
//        m_result.effort;
        m_result.stalled = false;
        m_result.reached_goal = false;
        m_server.setAborted(m_result);
    }
}

void RomanGripperController::preemptCallback()
{
}

void RomanGripperController::romanStateCallback(
    const roman_client_ros_utils::RomanState::ConstPtr& msg)
{
    m_state = msg;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roman_gripper_controller");
    return RomanGripperController().run();
}
