/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Yaskawa America, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*       * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Yaskawa America, Inc., nor the names
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>
#include <simple_message/socket/simple_socket.h>
#include <simple_message/socket/tcp_client.h>

#include "HDTManipulator.h"
#include "joint_trajectory_downloader.h"

using namespace industrial::simple_socket;

template <typename T>
std::string vector_to_string(const std::vector<T>& vec)
{
    std::stringstream ss;
    ss << "[ ";
    for (int i = 0; i < (int)vec.size(); ++i)
    {
        ss << vec[i] << " ";
    }
    ss << "]";
    return ss.str();
}

enum MainResult
{
    SUCCESS,
    FAILED_TO_INIT_HDT
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_handler");

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

    ros::NodeHandle node;
    ros::Publisher pub_joint_control_state = node.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
    ros::Publisher pub_joint_sensor_state = node.advertise<sensor_msgs::JointState>("joint_states", 1);

    industrial::tcp_client::TcpClient robot;

    const unsigned int IP_ARG_IDX = 1;
    const int JOINT_ANGLE_ARGS_BASE_IDX = 2;

    // accept one argument to connect via IP address
    if (argc > 1) {
        ROS_INFO("Motion download interface connecting to IP address: %s",  argv[IP_ARG_IDX]);
        robot.init(argv[IP_ARG_IDX], StandardSocketPorts::MOTION);
    }
    else {
        ROS_INFO("Skipping IP connection. We will use CAN bus");
    }

    motoman::joint_trajectory_downloader::JointTrajectoryDownloader jtHandler(node, &robot);

    bool arm_running = true;

    if (!jtHandler.HDTApp.Initialize()) {
        ROS_ERROR("Failed to initialize HDT Interface");
        return FAILED_TO_INIT_HDT;
    }

    printf("Setting position offsets: %s\n", vector_to_string(joint_offsets).c_str());

    jtHandler.HDTApp.SetPositionOffsets(joint_offsets);

    if (argc > 1) {
        // the trailing arguments specify the target joint position (in degrees)
        std::vector<double> jointAngles;
        jointAngles.resize(argc - JOINT_ANGLE_ARGS_BASE_IDX);
        std::cout << "JointAngles: ";
        for (int i = JOINT_ANGLE_ARGS_BASE_IDX; i < argc; i++) {
            jointAngles[i - JOINT_ANGLE_ARGS_BASE_IDX] = atof(argv[i]);
            std::cout << jointAngles[i - JOINT_ANGLE_ARGS_BASE_IDX] << " ";
        }

        std::cout << "Setting target position: " << vector_to_string(jointAngles) << std::endl;
        jtHandler.HDTApp.SetTargetPositionDegrees(jointAngles);
    }

    jtHandler.HDTApp.Run();

    arm_running = jtHandler.HDTApp.Running();

    ros::Rate r(10); // 10 hz
    while (arm_running) {
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_torques;
        std::vector<double> joint_currents;

        if (arm_running) {
            jtHandler.HDTApp.GetJointState(joint_positions, joint_velocities, joint_torques, joint_currents);
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
        arm_running = jtHandler.HDTApp.Running();
    }

    jtHandler.HDTApp.Shutdown();

    return 0;
}
