#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <cmu_manipulation_msgs/ManipulateAction.h>
#include <cmu_manipulation_msgs/GraspObjectCommandAction.h>
#include <ros/ros.h>

using ManipulateActionServer = actionlib::SimpleActionServer<cmu_manipulation_msgs::ManipulateAction>;
using GraspObjectActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::GraspObjectCommandAction>;

struct ManipulationExecutive
{
    std::unique_ptr<ManipulateActionServer> manipulate_server;
    std::unique_ptr<GraspObjectActionClient> grasp_object_client;

    int grasp_goal_id = 0;
};

void Manipulate(
    const cmu_manipulation_msgs::ManipulateGoal::ConstPtr& goal,
    ManipulationExecutive* executive)
{
    ROS_INFO("Receive a Manipulate request with task '%s'", goal->task.c_str());
    if (goal->task == "pickup") {
        if (goal->goal_poses.empty()) {
            ROS_ERROR("'pickup' task expects one goal pose (pose of the object)");
            executive->manipulate_server->setAborted();
            return;
        }

        cmu_manipulation_msgs::GraspObjectCommandGoal grasp_goal;
        grasp_goal.retry_count = 0;
        grasp_goal.id = executive->grasp_goal_id++;
        grasp_goal.gas_can_in_map = goal->goal_poses[0];

        auto state = executive->grasp_object_client->sendGoalAndWait(grasp_goal);
        if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
            cmu_manipulation_msgs::ManipulateResult result;
            ROS_ERROR("Grasp Object action failed");
            executive->manipulate_server->setAborted(result);
            return;
        }
        cmu_manipulation_msgs::ManipulateResult result;
        ROS_INFO("'pickup' task succeeded");
        executive->manipulate_server->setSucceeded(result);
    } else if (goal->task == "goto") {
        ROS_ERROR("Unimplemented task 'goto'");
        executive->manipulate_server->setAborted();
    } else if (goal->task == "move") {
        ROS_ERROR("Unimplemented task 'move'");
        executive->manipulate_server->setAborted();
    } else if (goal->task == "place") {
        ROS_ERROR("Unimplemented task 'place'");
        executive->manipulate_server->setAborted();
    } else if (goal->task == "open_door") {
        ROS_ERROR("Unimplemented task 'open_door'");
        executive->manipulate_server->setAborted();
    } else {
        ROS_ERROR("Unrecognized task command '%s'", goal->task.c_str());
        executive->manipulate_server->setAborted();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manipulation_executive");
    ros::NodeHandle nh;

    auto manipulate_action_name = "manipulate";
    auto grasp_object_action_name = "grasp_object_command";

    ManipulationExecutive executive;
    executive.manipulate_server.reset(new ManipulateActionServer(
            manipulate_action_name, boost::bind(Manipulate, _1, &executive), false));
    executive.grasp_object_client.reset(new GraspObjectActionClient(
                grasp_object_action_name, false));

    executive.manipulate_server->start();

    ros::spin();
    return 0;
}

