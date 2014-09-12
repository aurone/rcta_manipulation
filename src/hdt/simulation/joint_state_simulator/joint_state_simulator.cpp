#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

///< Basically duplicate functionality of industrial_robot_simulator to avoid having another source dependency...
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joint_state_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    std::vector<std::string> joint_names;
    if (!ph.getParam("joint_names", joint_names)) {
        ROS_ERROR("Failed to retrieve 'joint_names' from the param server");
        return 1;
    }

    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "";
    joint_state.name = joint_names;
    joint_state.position = std::vector<double>(joint_names.size(), 0.0);
    joint_state.velocity = std::vector<double>(joint_names.size(), 0.0);
    joint_state.effort = std::vector<double>(joint_names.size(), 0.0);

    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop(10.0);
    int seqno = 0;
    while (ros::ok()) {
        joint_state.header.seq = seqno++;
        joint_state.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_state);
        loop.sleep();
    }

    return 0;
}
