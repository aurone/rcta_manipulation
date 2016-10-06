// standard includes
#include <algorithm>

// system includes
#include <roman_client_ros_utils/RobotiqSimpleCmd.h>
#include <roman_client_ros_utils/RobotiqGenericCmd.h>
#include <roman_client_ros_utils/RomanState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot/robot_pub_types.h>

roman_client_ros_utils::RobotiqGenericCmd g_right_hand_cmd;
roman_client_ros_utils::RobotiqGenericCmd g_left_hand_cmd;

void RomanHandOpenCallback(
    const roman_client_ros_utils::RobotiqSimpleCmd::ConstPtr& msg)
{
    ROS_INFO("Open Hand");
    g_right_hand_cmd.cmd_positions[0] = 240.0;
    g_right_hand_cmd.cmd_positions[1] = 240.0;
    g_right_hand_cmd.cmd_positions[2] = 240.0;
    g_right_hand_cmd.cmd_positions[3] = 137.0;
    g_right_hand_cmd.cmd_speed[0] = (240.0 - 6.0) / 3.0;
    g_right_hand_cmd.cmd_speed[1] = (240.0 - 6.0) / 3.0;
    g_right_hand_cmd.cmd_speed[2] = (240.0 - 6.0) / 3.0;
    g_right_hand_cmd.cmd_speed[3] = (240.0 - 6.0) / 3.0;
}

void RomanHandCloseCallback(
    const roman_client_ros_utils::RobotiqSimpleCmd::ConstPtr& msg)
{
    ROS_INFO("Close Hand");
    g_right_hand_cmd.cmd_positions[0] = 6.0;
    g_right_hand_cmd.cmd_positions[1] = 6.0;
    g_right_hand_cmd.cmd_positions[2] = 6.0;
    g_right_hand_cmd.cmd_positions[3] = 137.0;
    g_right_hand_cmd.cmd_speed[0] = (240.0 - 6.0) / 3.0;
    g_right_hand_cmd.cmd_speed[1] = (240.0 - 6.0) / 3.0;
    g_right_hand_cmd.cmd_speed[2] = (240.0 - 6.0) / 3.0;
    g_right_hand_cmd.cmd_speed[3] = (240.0 - 6.0) / 3.0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roman_simulator");
    ros::NodeHandle nh;

    roman_client_ros_utils::RomanState state;
    sensor_msgs::JointState joint_state;

    ros::Subscriber roman_hand_open_sub =
            nh.subscribe("roman_hand_open", 1, RomanHandOpenCallback);
    ros::Subscriber roman_hand_close_sub =
            nh.subscribe("roman_hand_close", 1, RomanHandCloseCallback);

    ros::Publisher state_pub =
            nh.advertise<roman_client_ros_utils::RomanState>("roman_state", 1);
    ros::Publisher joint_state_pub =
            nh.advertise<sensor_msgs::JointState>("roman_joint_states", 1);

    std::vector<std::string> joint_names =
    {
        "limb_right_joint1",
        "limb_right_joint2",
        "limb_right_joint3",
        "limb_right_joint4",
        "limb_right_joint5",
        "limb_right_joint6",
        "limb_right_joint7",
        "limb_left_joint1",
        "limb_left_joint2",
        "limb_left_joint3",
        "limb_left_joint4",
        "limb_left_joint5",
        "limb_left_joint6",
        "limb_left_joint7",
        "torso_joint1",
        "track_left_joint",
        "track_right_joint",
    };

    // initialize state
    state.in_fault = 0;

    // initialize fault state
    state.fault_status.ctrl_fault_status;
    state.fault_status.mech_id;
    state.fault_status.joint_id;
    state.fault_status.num_mechanisms;
    state.fault_status.mech_fault_status;

    // initialize waypoint state
    state.state.num_joints = ROBOT_NUM_JOINTS;
    state.state.positions.resize(ROBOT_NUM_JOINTS, 0.0);
    state.state.world2robot.orientation.w = 1.0;

    // initialize right hand state
    auto& right_hand_state = state.right_hand_state;
    right_hand_state.active = 1;
    right_hand_state.in_fault = 0;
    std::fill(
            right_hand_state.actuator_positions.begin(),
            right_hand_state.actuator_positions.end(),
            0.0);
    std::fill(
            right_hand_state.actuator_currents.begin(),
            right_hand_state.actuator_currents.end(),
            40.0);
    std::fill(
            right_hand_state.passive_positions.begin(),
            right_hand_state.passive_positions.end(),
            0.0);

    // initialize left hand state
    auto& left_hand_state = state.left_hand_state;
    left_hand_state.active = 1;
    left_hand_state.in_fault = 0;
    std::fill(
            left_hand_state.actuator_positions.begin(),
            state.left_hand_state.actuator_positions.end(),
            0.0);
    std::fill(
            left_hand_state.actuator_currents.begin(),
            left_hand_state.actuator_currents.end(),
            40.0);
    std::fill(
            left_hand_state.passive_positions.begin(),
            left_hand_state.passive_positions.end(),
            0.0);

    // initialize joint_state
    joint_state.name = joint_names;
    joint_state.position = state.state.positions;

    ros::Time last_state_publish_time(0);
    ros::Rate state_publish_rate(10.0);

    ros::Rate rate(100.0);
    while (ros::ok()) {
        ros::spinOnce();

        double dt = rate.expectedCycleTime().toSec();

        ros::Time now = ros::Time::now();

        state.utime = now.toNSec() / 1e3;
        state.fault_status.utime = state.utime;
        state.state.utime = state.utime;
        state.right_hand_state.utime = state.utime;
        state.left_hand_state.utime = state.utime;

        // update hand position
        double dp0 = g_right_hand_cmd.cmd_positions[0] - state.right_hand_state.actuator_positions[0];
        double dp1 = g_right_hand_cmd.cmd_positions[1] - state.right_hand_state.actuator_positions[1];
        double dp2 = g_right_hand_cmd.cmd_positions[2] - state.right_hand_state.actuator_positions[2];
        double dp3 = g_right_hand_cmd.cmd_positions[3] - state.right_hand_state.actuator_positions[3];
        state.right_hand_state.actuator_positions[0] +=
                std::copysign(1.0, dp0) * g_right_hand_cmd.cmd_speed[0] * dt;
        state.right_hand_state.actuator_positions[1] +=
                std::copysign(1.0, dp1) * g_right_hand_cmd.cmd_speed[1] * dt;
        state.right_hand_state.actuator_positions[2] +=
                std::copysign(1.0, dp2) * g_right_hand_cmd.cmd_speed[2] * dt;
        state.right_hand_state.actuator_positions[3] +=
                std::copysign(1.0, dp3) * g_right_hand_cmd.cmd_speed[3] * dt;

        // update joint positions
        joint_state.header.stamp = now;

        if (now > last_state_publish_time + state_publish_rate.expectedCycleTime()) {
            state_pub.publish(state);
            joint_state_pub.publish(joint_state);
        }

        rate.sleep();
    }
    return 0;
}
