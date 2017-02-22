// standard includes
#include <cmath>
#include <string>

// system includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tilt_mount_simulator");

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    double fixed_tilt_angle_deg;
    if (!ph.getParam("fixed_tilt_angle_deg", fixed_tilt_angle_deg)) {
        ROS_ERROR("Failed to retrieve 'fixed_tilt_angle' from the param server");
        return 1;
    }

    std::string tilt_mount_name = "tilt_mount";
    std::string tilt_joint_name = tilt_mount_name + "_tilt_joint";

    sensor_msgs::JointState joint_state;

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate loop_rate(10);
    int seqno = 0;
    while (ros::ok()) {
        joint_state.header.frame_id = "";
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.seq = seqno++;

        joint_state.name = { tilt_joint_name };
        joint_state.position = { M_PI * fixed_tilt_angle_deg / 180.0 };
        joint_state.velocity = { 0.0 };
        joint_state.effort = { 0.0 };

        joint_state_pub.publish(joint_state);
        loop_rate.sleep();
    }

    return 0;
}

