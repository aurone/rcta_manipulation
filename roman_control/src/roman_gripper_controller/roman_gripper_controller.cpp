#include "roman_gripper_controller.h"

#include <robotiq_controllers/gripper_model.h>

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
    m_expected_state_rate(10.0),
    m_state_history_length(1.0)
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

    m_hand = Hand::Right;

    int histlen = (int)std::ceil(m_state_history_length * m_expected_state_rate);
    if (histlen < 0) {
        std::stringstream sserr;
        sserr << "Invalid history size (" << histlen << ")";
        throw std::runtime_error(sserr.str());
    }

    m_state_hist.set_capacity(histlen);
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
        ROS_INFO("Preempt current goal");
        m_server.setPreempted();
        return;
    }

    m_state_hist.clear();

    roman_client_ros_utils::RobotiqSimpleCmd cmd;

    // prepare command
    cmd.utime = ros::Time::now().toNSec() / 1e3;
    cmd.mode = roman_client_ros_utils::RobotiqSimpleCmd::MODE_BASIC; ///MODE_PINCH;
    if (m_hand == Hand::Right) {
        cmd.mech = roman_client_ros_utils::RobotiqSimpleCmd::MECH_RIGHT;
    } else {
        cmd.mech = roman_client_ros_utils::RobotiqSimpleCmd::MECH_LEFT;
    }

    // send command
    if (m_goal->command.position == GripperModel().maximum_width()) {
        cmd.close = false;
        m_hand_open_pub.publish(cmd);
    } else if (m_goal->command.position == GripperModel().minimum_width()) {
        cmd.close = true;
        m_hand_close_pub.publish(cmd);
    } else {
        ROS_WARN("Roman Gripper Controller currently only accepts binary open/close input represented as { %0.3f, %0.3f }", GripperModel().minimum_width(), GripperModel().maximum_width());
        m_result.stalled = false;
        m_result.reached_goal = false;
        m_server.setAborted(m_result);
    }
}

void RomanGripperController::preemptCallback()
{
    ROS_INFO("Preempt current goal");
    // TODO: send a command to stop the gripper here
    m_server.setPreempted();
}

void RomanGripperController::romanStateCallback(
    const roman_client_ros_utils::RomanState::ConstPtr& msg)
{
    if (!m_server.isActive()) {
        return;
    }

    m_state_hist.push_back(msg);

    if (m_state_hist.size() == 0) {
        // no checking for done
        m_server.setSucceeded(m_result);
    }

    if (m_state_hist.full()) {
        ROS_DEBUG("History collected. Checking for done");

        // check if we've moved significantly
        RomanState::ConstPtr oldest = m_state_hist.back();
        RomanState::ConstPtr newest = m_state_hist.front();

        double dp0, dp1, dp2, dp3;
        if (m_hand == Hand::Right) {
            dp0 = fabs(newest->right_hand_state.actuator_positions[0] - oldest->right_hand_state.actuator_positions[0]);
            dp1 = fabs(newest->right_hand_state.actuator_positions[1] - oldest->right_hand_state.actuator_positions[1]);
            dp2 = fabs(newest->right_hand_state.actuator_positions[2] - oldest->right_hand_state.actuator_positions[2]);
            dp3 = fabs(newest->right_hand_state.actuator_positions[3] - oldest->right_hand_state.actuator_positions[3]);
        } else {
            dp0 = fabs(newest->left_hand_state.actuator_positions[0] - oldest->left_hand_state.actuator_positions[0]);
            dp1 = fabs(newest->left_hand_state.actuator_positions[1] - oldest->left_hand_state.actuator_positions[1]);
            dp2 = fabs(newest->left_hand_state.actuator_positions[2] - oldest->left_hand_state.actuator_positions[2]);
            dp3 = fabs(newest->left_hand_state.actuator_positions[3] - oldest->left_hand_state.actuator_positions[3]);
        }

        const double threshold = 1.0;
        if (dp0 < threshold && dp1 < threshold && dp2 < threshold && dp3 < threshold) {
            m_result.stalled = false;
            m_result.reached_goal = true;
            m_result.position = m_goal->command.position;
            ROS_INFO("Gripper reached final position");
            m_server.setSucceeded(m_result);
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roman_gripper_controller");
    return RomanGripperController().run();
}
