#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using GripperCommandActionServer =
        actionlib::SimpleActionServer<control_msgs::GripperCommandAction>;

void Execute(
    GripperCommandActionServer* server,
    ros::Publisher* command_pub,
    const control_msgs::GripperCommandGoal::ConstPtr& goal)
{
    ROS_DEBUG("Execute gripper command { position = %f, max_effort = %f }", goal->command.position, goal->command.max_effort);

    std_msgs::Float64MultiArray command;

    command.layout.dim.resize(1);
    command.layout.dim[0].label = "joint_positions";
    command.layout.dim[0].size = 3;
    command.layout.dim[0].stride = 0;

    // hardcoded maximum width for the robotiq c-model gripper
    if (goal->command.position == 0.0841) {
        command.data = { 0.0495, 0.0495, 0.0495 };
    } 
    
    else {
        command.data = { 1.2218, 1.2218, 1.2218 };
    }

    command_pub->publish(command);

    control_msgs::GripperCommandResult result;
    result.reached_goal = true;
    result.position = goal->command.position;
    server->setSucceeded(result);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gripper_command_action_server");
    ros::NodeHandle nh;

//    auto action_name = "gripper_action";

    auto action_name = "rcta_right_robotiq_controller/gripper_action";

    auto autostart = false;

//    using ExecuteCallback = GripperCommandActionServer::ExecuteCallback;
//    using ExecuteCallback = boost::function<void(const control_msgs::GripperCommandGoal)

    auto command_pub = nh.advertise<std_msgs::Float64MultiArray>(
            "rcta_right_robotiq_controller/command", 1);

    GripperCommandActionServer server(
            action_name,
            [&](const control_msgs::GripperCommandGoal::ConstPtr& goal) {
                return Execute(&server, &command_pub, goal);
            },
            autostart);

    server.start();

    ros::spin();
    return 0;
}

