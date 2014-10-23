#ifndef stow_arm_h
#define stow_arm_h

#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <hdt/MoveArmCommandAction.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <ros/ros.h>

void result_callback(
	const actionlib::SimpleClientGoalState& state,
	const hdt::MoveArmCommandResult::ConstPtr& result)
{
	ROS_INFO("");
}

enum MainResult
{
	SUCCESS = 0,
	FAILED_TO_RETRIEVE_ROBOT_DESCRIPTION,
	TIMED_OUT_WAITING_FOR_SERVER,
	TIMED_OUT_WAITING_FOR_RESULT
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "stow_arm");
	ros::NodeHandle nh;

	std::string urdf_string;
	if (!nh.getParam("robot_description", urdf_string)) {
		ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
		exit(FAILED_TO_RETRIEVE_ROBOT_DESCRIPTION);
	}

	hdt::RobotModelPtr robot_model = hdt::RobotModel::LoadFromURDF(urdf_string);

	const std::string move_arm_command_action_name = "move_arm_command";
	actionlib::SimpleActionClient<hdt::MoveArmCommandAction> client(move_arm_command_action_name, true);

	if (!client.waitForServer(ros::Duration(10.0))) {
		exit(TIMED_OUT_WAITING_FOR_SERVER);
	}

	hdt::MoveArmCommandGoal move_arm_stow_goal;
    move_arm_stow_goal.type = hdt::MoveArmCommandGoal::JointGoal;
    move_arm_stow_goal.goal_joint_state.name = robot_model->joint_names();
    move_arm_stow_goal.goal_joint_state.position =
    		std::vector<double>(robot_model->joint_names().size(), 0);

    move_arm_stow_goal.octomap; // meh?

    //include the attached object in the goal
    move_arm_stow_goal.has_attached_object = false;

    move_arm_stow_goal.execute_path = true;

	auto result_cb = boost::bind(result_callback, _1, _2);
	client.sendGoal(move_arm_stow_goal, result_cb);
	if (!client.waitForResult()) {
		exit(TIMED_OUT_WAITING_FOR_RESULT);
	}

	return SUCCESS;
}

#endif
