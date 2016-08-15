// system includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

class GripperCommandClient
{
public:

    GripperCommandClient() :
        nh_(),
        client_("gripper_controller/gripper_command_action")
    {

    }

    ~GripperCommandClient()
    {

    }

    int run()
    {
        while (!client_.waitForActionServerToStart(ros::Duration(1.0))) {
            ROS_INFO("Waiting for action server to start");
            ros::spinOnce();
        }
        ROS_INFO("Done waiting for action server");

        ros::Rate rate(10);
        while (ros::ok()) {
            GripperCommandGoal goal_msg;
            goal_msg.command.position = 0.0854;
            current_goal_ = client_.sendGoal(goal_msg,
                    boost::bind(&GripperCommandClient::gripper_command_action_transition, this, _1),
                    boost::bind(&GripperCommandClient::gripper_command_action_feedback, this, _1, _2));
            ros::spinOnce();
            rate.sleep();
        }
        return 0;
    }

private:

    typedef control_msgs::GripperCommandGoal GripperCommandGoal;
    typedef control_msgs::GripperCommandFeedback GripperCommandFeedback;
    typedef control_msgs::GripperCommandResult GripperCommandResult;
    typedef actionlib::ActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;
    typedef GripperCommandActionClient::GoalHandle GoalHandle;

    ros::NodeHandle nh_;

    GripperCommandActionClient client_;

    GoalHandle current_goal_;

    void gripper_command_action_transition(GoalHandle handle)
    {
        ROS_INFO("received transition");
    }

    void gripper_command_action_feedback(GoalHandle goalHandle, const GripperCommandFeedback::ConstPtr& msg)
    {
        ROS_INFO("received feedback");
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gripper_command_action_client");
    GripperCommandClient client;
    return client.run();
}
