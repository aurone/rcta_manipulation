#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/stringifier/stringifier.h>

sensor_msgs::JointState::ConstPtr last_msg;
hdt::RobotModelPtr robot_model;
ros::Publisher joint_command_pub;
trajectory_msgs::JointTrajectory::Ptr last_command;
std::vector<double> differences_sum(7, 0.0);

void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{

    if (msg_utils::contains_only_joints(*msg, robot_model->joint_names())) {
        last_msg = msg;

        if (last_command) {
            std::vector<double> joint_diff;
            msg_utils::vector_diff(last_msg->position, last_command->points.front().positions, joint_diff);
            ROS_INFO("Joint Differences: %s", to_string(msg_utils::to_degrees(joint_diff)).c_str());

            static int num_commands = 0;
            msg_utils::vector_sum(differences_sum, joint_diff, differences_sum);
            ++num_commands;

            std::vector<double> averages;
            msg_utils::vector_mul(differences_sum, std::vector<double>(7, 1.0 / num_commands), averages);
            ROS_INFO("Average Joint Difference: %s", to_string(msg_utils::to_degrees(averages)).c_str());
        }

        trajectory_msgs::JointTrajectory traj_cmd;

        traj_cmd.header.frame_id = "";
        traj_cmd.header.stamp = ros::Time::now();
        static int seqno = 0;
        traj_cmd.header.seq = seqno++;

        traj_cmd.points.resize(1);
        traj_cmd.joint_names = robot_model->joint_names();
        traj_cmd.points[0].positions = last_msg->position;

        joint_command_pub.publish(traj_cmd);

        if (!last_command) {
            last_command.reset(new trajectory_msgs::JointTrajectory);
        }
        *last_command = traj_cmd;
    }
}

int main(int argc, char* argv[])
{
    // This node listens to incoming joint states for the arm, publishes joint
    // commands to achieve the current set of joint states, and measures the
    // subsequent drift of the joints

    ros::init(argc, argv, "joint_position_offset_test");
    ros::NodeHandle nh;

    std::string urdf_string;
    if (!nh.getParam("robot_description", urdf_string)) {
        ROS_WARN("Failed to retrieve 'robot_description' from the param server");
        return 1;
    }

    robot_model = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model) {
        ROS_WARN("Failed to load Robot Model from the URDF");
        return 1;
    }

    ros::Subscriber joint_states_sub = nh.subscribe("joint_states", 5, joint_states_cb);
    joint_command_pub = nh.advertise<trajectory_msgs::JointTrajectory>("command", 1);


    ros::spin();
    return 0;
}
