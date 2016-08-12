#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <rcta/common/stringifier/stringifier.h>
#include "HDTManipulator.h"

enum MainResult
{
    SUCCESS,
    FAILED_TO_INIT_HDT
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_handler");
    ros::NodeHandle nh;

    std::vector<std::string> joint_names = {
        "arm_1_shoulder_twist",
        "arm_2_shoulder_lift",
        "arm_3_elbow_twist",
        "arm_4_elbow_lift",
        "arm_5_wrist_twist",
        "arm_6_wrist_lift",
        "arm_7_gripper_lift"
    };

    std::vector<double> joint_offsets = {
            -1.40 * M_PI / 180.0,
             3.00 * M_PI / 180.0,
             2.28 * M_PI / 180.0,
             0.48 * M_PI / 180.0,
             2.02 * M_PI / 180.0,
             5.36 * M_PI / 180.0,
            -1.27 * M_PI / 180.0 };

    bool arm_running = true;

    HDTManipulatorTestApp manip_interface;

    if (!manip_interface.Initialize()) {
        ROS_ERROR("Failed to initialize HDT Interface");
        return FAILED_TO_INIT_HDT;
    }

    printf("Setting position offsets: %s\n", to_string(joint_offsets).c_str());
    manip_interface.SetPositionOffsets(joint_offsets);

    manip_interface.Run();
    arm_running = manip_interface.Running();

    ros::Publisher pub_joint_sensor_state = nh.advertise<sensor_msgs::JointState>("joint_states",  10);
    ros::Publisher pub_joint_control_state = nh.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 10);

    ros::Rate r(10); // 10 hz
    while (arm_running) {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_torques;
        std::vector<double> joint_currents;

        if (arm_running) {
            manip_interface.GetJointState(joint_positions, joint_velocities, joint_torques, joint_currents);
        }

        // assign values to messages
        control_msgs::FollowJointTrajectoryFeedback control_state; // always start with a "clean" message
        control_state.header.stamp = ros::Time::now();
        control_state.joint_names = joint_names;
        control_state.actual.positions = joint_positions;

        sensor_msgs::JointState sensor_state;
        sensor_state.header.stamp = ros::Time::now();
        sensor_state.name = joint_names;
        sensor_state.position = joint_positions;

        pub_joint_control_state.publish(control_state);
        pub_joint_sensor_state.publish(sensor_state);

        ros::spinOnce();
        r.sleep();
        arm_running = manip_interface.Running();
    }

    manip_interface.Shutdown();
    return 0;
}
