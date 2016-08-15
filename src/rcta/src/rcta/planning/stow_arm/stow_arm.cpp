#ifndef stow_arm_h
#define stow_arm_h

// standard includes
#include <cstdlib>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <rcta/MoveArmCommandAction.h>
#include <rcta/common/hdt_description/RobotModel.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/utils.h>

struct StowPosition
{
	std::string name;
	std::vector<double> joint_positions;
};

bool cb_success = false;
std::vector<StowPosition> stow_positions_;
actionlib::SimpleActionClient<rcta::MoveArmCommandAction> *client_;
hdt::RobotModelPtr robot_model;

ros::Publisher pub;
ros::Subscriber sub;

bool extract_xml_value(
    XmlRpc::XmlRpcValue& value,
    StowPosition& stow_position)
{
    if (!value.hasMember("name") || !value.hasMember("joint_vector_degs")) {
        return false;
    }

    StowPosition tmp;
    bool success =
            msg_utils::extract_xml_value(value["name"], tmp.name) &&
            msg_utils::extract_xml_value(value["joint_vector_degs"], tmp.joint_positions);

    if (success) {
        stow_position = tmp;
    }
    return success;
}


void result_callback(
	const actionlib::SimpleClientGoalState& state,
	const rcta::MoveArmCommandResult::ConstPtr& result)
{
	if(result->success){
		ROS_INFO("Arm stowed!");
		cb_success = true;
	} else {
		ROS_WARN("...");
	}
}


bool try_stow_arm(){

	if (!client_->waitForServer(ros::Duration(10.0))) {
		return false;
	}

	rcta::MoveArmCommandGoal move_arm_stow_goal;
	move_arm_stow_goal.type = rcta::MoveArmCommandGoal::JointGoal;
	move_arm_stow_goal.goal_joint_state.name = robot_model->joint_names();
	move_arm_stow_goal.octomap; // meh?
    	//include the attached object in the goal
    	move_arm_stow_goal.has_attached_object = false;
    	move_arm_stow_goal.execute_path = true;

	for(size_t i = 0; i < stow_positions_.size(); i++){
		ROS_INFO("Trying stow position %s: %s", stow_positions_[i].name.c_str(), to_string(stow_positions_[i].joint_positions).c_str());

		move_arm_stow_goal.goal_joint_state.position = stow_positions_[i].joint_positions;
    			//std::vector<double>(robot_model->joint_names().size(), 0);

		auto result_cb = boost::bind(result_callback, _1, _2);
		client_->sendGoal(move_arm_stow_goal, result_cb);
		if (!client_->waitForResult()) {
			return false;
		}
		if(cb_success) return true;
	}
	//if we got here all stow positions failed!
	ROS_WARN("All stow positions failed!");
	return false;
}

void stow_request_callback(const std_msgs::String::ConstPtr& message){
	if(message != NULL){
		std::string msg = message->data;
		ROS_INFO("Got message '%s'", msg.c_str());
		if(msg.compare("stow")==0){
			std_msgs::String response;
			cb_success = false;
			if(!try_stow_arm()){
				ROS_WARN("Arm failed to stow!");
				response.data = std::string("error");
			} else {
				ROS_INFO("Arm stowed!");
				response.data = std::string("done");
			}
			sleep(4);
			pub.publish(response);
		}
	}
}

enum MainResult
{
	SUCCESS = 0,
	FAILED_TO_STOW_ARM,
	FAILED_TO_RETRIEVE_ROBOT_DESCRIPTION,
	TIMED_OUT_WAITING_FOR_SERVER,
	TIMED_OUT_WAITING_FOR_RESULT,
	FAILED_TO_RETRIEVE_STOW_POSITIONS
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "stow_arm");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	std::string urdf_string;
	if (!nh.getParam("robot_description", urdf_string)) {
		ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
		exit(FAILED_TO_RETRIEVE_ROBOT_DESCRIPTION);
	}

	robot_model = hdt::RobotModel::LoadFromURDF(urdf_string);

	// read in stow positions
    	if (!msg_utils::download_param(ph, "stow_positions", stow_positions_)) {
        	ROS_ERROR("Failed to retrieve 'stow_positions' from the param server");
	        exit(FAILED_TO_RETRIEVE_STOW_POSITIONS);
    	}

	ROS_INFO("Stow Positions:");
	for (StowPosition& position : stow_positions_) {
		ROS_INFO("    %s: %s", position.name.c_str(), to_string(position.joint_positions).c_str());
		position.joint_positions = msg_utils::to_radians(position.joint_positions);
	}

	const std::string move_arm_command_action_name = "move_arm_command";
	actionlib::SimpleActionClient<rcta::MoveArmCommandAction> client(move_arm_command_action_name, true);
	client_ = &client;

	pub = nh.advertise<std_msgs::String>("/hdt/stow_arm_response", 5);
        sub = nh.subscribe("/hdt/stow_arm_request", 1, stow_request_callback);

	ROS_INFO("Initialized! Waiting on messages!");
	ros::spin();
	//return SUCCESS;
}

#endif
