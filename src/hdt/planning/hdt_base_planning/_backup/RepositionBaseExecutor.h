#ifndef RepositionBaseExecutor_h
#define RepositionBaseExecutor_h


#include <ros/ros.h>
#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <sstream>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <hdt/planning/arm_planning_node/HDTRobotModel.h>

#include <hdt/RepositionBaseCommandAction.h>
#include <actionlib/server/simple_action_server.h>


namespace RepositionBaseExecutionStatus
{

enum Status
{
	INVALID = -1,
	IDLE = 0,
	FAULT,
	COMPUTING_REPOSITION_BASE,
// 	GENERATING_SEARCH_SPACE,
// 	CHECKING_COLLISION,
// 	CHECKING_KINEMATICS,
// 	SELECTING_CANDIDATES,
	COMPLETING_GOAL
};

std::string to_string(Status status);

}


class RepositionBaseExecutor
{
public:
	
	RepositionBaseExecutor();
	bool initialize();
	int run();
	
	enum MainResult
	{
		SUCCESS = 0,
		FAILED_TO_INITIALIZE		
	};


private:

	hdt::HDTRobotModel* hdt_robot_model_;

	double wrapAngle(double ang);
	void computeRobPose(double objx, double objy, double objY,  double robx0, double roby0, double robY0,  double& robxf, double& robyf, double& robYf, hdt::HDTRobotModel* hdt_robot_model);

	
	ros::NodeHandle nh_;
	
	std::string action_name_;
	typedef actionlib::SimpleActionServer<hdt::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
	std::unique_ptr<RepositionBaseCommandActionServer> as_;

	bool bComputedRobPose_;


	hdt::RepositionBaseCommandGoal::ConstPtr current_goal_;

	RepositionBaseExecutionStatus::Status status_;
	RepositionBaseExecutionStatus::Status last_status_;

	void goal_callback();
	void preempt_callback();

    uint8_t execution_status_to_feedback_status(RepositionBaseExecutionStatus::Status status);
};

#endif
