#include "RepositionBaseExecutor.h"
// #include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/common/stringifier/stringifier.h>


// TODO: remove the followings when actionlib works with /map
///////////////////////////////////////////////////////////
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::OccupancyGrid::ConstPtr map_;
geometry_msgs::PoseStamped::ConstPtr rob0_;
int bMapReceived_ = 0;			// 0: ready to receive, 1: received, -1: blocked (in processing)
int bRobPoseReceived_ = 0;

void subMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if (bMapReceived_!=-1)
	{
		map_ = msg;
		bMapReceived_ = 1;
// 		printf("map received!\n");
// 		std::cerr << msg->header << std::endl;
// 		std::cerr << msg->info << std::endl;
	}
}

void subRobPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (bRobPoseReceived_!=-1)
	{
		rob0_ = msg;
		bRobPoseReceived_ = 1;
// 		printf("robot pose received!\n");
// 		std::cerr << msg->header << std::endl;
	}
}
///////////////////////////////////////////////////////////




namespace RepositionBaseExecutionStatus
{
	std::string to_string(Status status)
	{
		switch (status) {
			case IDLE:
				return "Idle";
			case FAULT:
				return "Fault";
			case COMPUTING_REPOSITION_BASE:
				return "ComputingRepositionBase";
			default:
				return "Invalid";
		}   
	}
}

RepositionBaseExecutor::RepositionBaseExecutor() :
nh_(),
action_name_("reposition_base_command"),
as_(),
bComputedRobPose_(false),
current_goal_(),
status_(RepositionBaseExecutionStatus::INVALID),
last_status_(RepositionBaseExecutionStatus::INVALID)
{
}

bool RepositionBaseExecutor::initialize()
{
	// action server initialization
	std::string action_name_ = "reposition_base_command";
	as_.reset(new RepositionBaseCommandActionServer(action_name_, false));
	if (!as_) {
		ROS_ERROR("Failed to instantiate Reposition Base Command Action Server");
		return false;
	}   

	as_->registerGoalCallback(boost::bind(&RepositionBaseExecutor::goal_callback,this));
	as_->registerPreemptCallback(boost::bind(&RepositionBaseExecutor::preempt_callback,this));

	ROS_INFO("Starting action server '%s'...", action_name_.c_str());
	as_->start();
	ROS_INFO("Action server started");


	// hdt robot initialization for inverse kinematics check
	std::string group_name_;
	std::string kinematics_frame_;
	std::string planning_frame_;
	std::string planning_link_;
	std::string chain_tip_link_;
	std::string urdf_;
	boost::shared_ptr<urdf::ModelInterface> urdf_model_;

	if (!nh_.hasParam("robot_description")) {
		ROS_ERROR("Missing parameter \"robot_description\"");
		return false;
	}   

	nh_.getParam("robot_description", urdf_);

	urdf_model_ = urdf::parseURDF(urdf_);
	if (!urdf_model_) {
		ROS_ERROR("Failed to parse URDF");
		return false;
	}   

	boost::shared_ptr<const urdf::Joint> first_joint = urdf_model_->getJoint("arm_1_shoulder_twist");
	if (!first_joint) {
		ROS_ERROR("Failed to find joint 'arm_1_shoulder_twist'");
		return false;
	}   

	group_name_ = "hdt_arm";

	kinematics_frame_ = first_joint->parent_link_name;
	planning_frame_ = first_joint->parent_link_name;

	planning_link_ = "arm_7_gripper_lift_link";
	chain_tip_link_ = "arm_7_gripper_lift_link";

// 	hdt::HDTRobotModel* hdt_robot_model = new hdt::HDTRobotModel;
	hdt_robot_model_ = new hdt::HDTRobotModel;
	if (!hdt_robot_model_ || !hdt_robot_model_->init(urdf_)) {
		ROS_ERROR("Failed to initialize HDT Robot Model");
		return false;
	}   


// TODO: remove the followings when actionlib works with /map
	subMap_ = nh_.subscribe("/local_costmap/costmap/costmap",1,subMapCallback);   // TODO: frame_id: /abs_nwu (but identical to /abs_ned)
	subRobPose_ = nh_.subscribe("/rrnav/absPose",1,subRobPoseCallback);           // TODO: frame_id: /abs_ned

}


int RepositionBaseExecutor::run()
{
	if (!initialize()) {
		return FAILED_TO_INITIALIZE;
	}

	status_ = RepositionBaseExecutionStatus::IDLE;

	ros::Rate loop_rate(1);
	while (ros::ok()) {
// 		RunUponDestruction rod([&](){ loop_rate.sleep(); });
// 		ROS_INFO("Spinning (%s)...", to_string(status_).c_str());

		ros::spinOnce();

		if (status_ != last_status_) {
			ROS_INFO("Reposition Base Executor Transitioning: %s -> %s", to_string(last_status_).c_str(), to_string(status_).c_str());
			last_status_ = status_;
		}

		assert((bool)as_);

		switch (status_) {
			case RepositionBaseExecutionStatus::IDLE:
				{
					if (as_->isActive()) {
						status_ = RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE;
					}
				}   break;
			case RepositionBaseExecutionStatus::FAULT:
				{
					if (as_->isActive()) {
						status_ = RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE;
					}
				}   break;
			case RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE:
				{
					if (!bComputedRobPose_) {
						
						// given object pose (x-axis is aligned to canister hose)
						double objx = current_goal_->gas_can_in_map.pose.position.x;	
						double objy = current_goal_->gas_can_in_map.pose.position.y;	
						double objz = current_goal_->gas_can_in_map.pose.position.z;
						double objY = 2.0*acos(current_goal_->gas_can_in_map.pose.orientation.w)*sign(current_goal_->gas_can_in_map.pose.orientation.z);		// assuming that rotation axis is parallel to z-axis
						double objP0 = 0.0;
						double objR0 = 0.0;
						
						// initial robot pose
						double robx0 = current_goal_->base_link_in_map.pose.position.x;	
						double roby0 = current_goal_->base_link_in_map.pose.position.y;	
						double robz0 = current_goal_->base_link_in_map.pose.position.z;
						double robY0 = 2.0*acos(current_goal_->base_link_in_map.pose.orientation.w)*sign(current_goal_->base_link_in_map.pose.orientation.z);	// assuming that rotation axis is parallel to z-axis
						double robP0 = 0.0;
						double robR0 = 0.0;
						
						// desired robot pose
						double robxf=robx0, robyf=roby0, robzf=robz0, robYf=robY0, robPf=robP0, robRf=robR0;	// final (computed) robot pose
						
						bool ret = computeRobPose(objx,objy,objY, robx0,roby0,robY0, robxf,robyf,robYf, hdt_robot_model_); 
						if (ret)
						{
							bComputedRobPose_ = true;	

// //						hdt::RepositionBaseCommandFeedback feedback;
// // 						feedback.status = execution_status_to_feedback_status(status_?);
// // 						as_->publishFeedback(feedback);
// 							status_ = RepositionBaseExecutionStatus::COMPLETING_GOAL;

							std::vector<geometry_msgs::PoseStamped> candidate_base_poses;
							geometry_msgs::PoseStamped candidate_base_pose;
							candidate_base_pose.header.frame_id = "/abs_nwu";
							candidate_base_pose.header.seq = 0;
							candidate_base_pose.header.stamp = ros::Time::now();

							candidate_base_pose.pose.position.x = robxf;
							candidate_base_pose.pose.position.y = robyf;
							candidate_base_pose.pose.position.z = robzf;

							tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf,robPf,robYf);
							candidate_base_pose.pose.orientation.x = robqf[0]; 
							candidate_base_pose.pose.orientation.y = robqf[1]; 
							candidate_base_pose.pose.orientation.z = robqf[2]; 
							candidate_base_pose.pose.orientation.w = robqf[3];
							candidate_base_poses.push_back(candidate_base_pose);

							hdt::RepositionBaseCommandResult result;
							result.result = hdt::RepositionBaseCommandResult::SUCCESS;
							result.candidate_base_poses = candidate_base_poses;
							as_->setSucceeded(result);
							status_ = RepositionBaseExecutionStatus::IDLE;
						}
					}
				}   break;
			case RepositionBaseExecutionStatus::COMPLETING_GOAL:
				{
// 					hdt::RepositionBaseCommandResult result;
// 					result.result = hdt::RepositionBaseCommandResult::SUCCESS;
// 					result.candidate_base_poses = 
// 					as_->setSucceeded(result);
// 					status_ = RepositionBaseExecutionStatus::IDLE;
				}   break;
			default:
				break;
		}
	}

	nh_.deleteParam("hdt_base_planning_node/objx");
	nh_.deleteParam("hdt_base_planning_node/objy");
	nh_.deleteParam("hdt_base_planning_node/objY");
	
	return SUCCESS;
}

void RepositionBaseExecutor::goal_callback()
{
	ROS_INFO("Received a new goal");
	current_goal_ = as_->acceptNewGoal();
	ROS_INFO("    Goal ID: %u", current_goal_->id);
	ROS_INFO("    Retry Count: %d", current_goal_->retry_count);
	ROS_INFO("    Gas Can Pose [map]: %s", to_string(current_goal_->gas_can_in_map.pose).c_str());
	ROS_INFO("    Base Link Pose [map]: %s", to_string(current_goal_->base_link_in_map.pose).c_str());
	ROS_INFO("    Occupancy Grid Frame ID: %s", current_goal_->map.header.frame_id.c_str());

	bComputedRobPose_ = false;	
}

void RepositionBaseExecutor::preempt_callback()
{
}

uint8_t RepositionBaseExecutor::execution_status_to_feedback_status(RepositionBaseExecutionStatus::Status status)
{
	switch (status) {
		case RepositionBaseExecutionStatus::IDLE:
			return -1;
		case RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE:
			return (float) 0.0;	// TODO: feedback of planning time in [s]
		default:
			return -1;
	}
}


double RepositionBaseExecutor::sign(double val)
{
	return (val >=0) ? 1.0 : -1.0;
}

double RepositionBaseExecutor::wrapAngle(double ang)
{
	while(ang >= M_PI)
		ang -= 2*M_PI;
	while(ang < -M_PI)
		ang += 2*M_PI;
	return ang;
}

bool RepositionBaseExecutor::computeRobPose(double objx, double objy, double objY,  double robx0, double roby0, double robY0,  double& robxf, double& robyf, double& robYf, hdt::HDTRobotModel* hdt_robot_model) 
{
	// COMPARISON OPTIONS
	// A) flag for pGrasp operation
	bool bCheckGrasp = true;
// 	bool bCheckGrasp = false;

	// B) flag for pObs operation
	bool bCheckObs = true;
// 	bool bCheckObs = false;

	// C) flag for pWork operation
// 	bool bCheckWork = true;
	bool bCheckWork = false;

	// D) arm position offset for computing pObs
// 	double armOffsety = 0.0;
// 	double armOffsety = 0.5;
	double armOffsety = 1.0;

	// E) /base_link offset from /top_shelf for final robot pose return
	double baseOffsetx = -0.3;


	// policy for robot pose selection
	// 0) set search space
	// 1) probability of successul grasping
	// 2) distance to obstacles
	// 3) (workspace of hdt arm)
	// 4) multiplication of 1~3)
	// 5) if tied, one with minimum distance from robot
	// 6) conversion for /camera_link frame


	// 0) set search space (r,th,Y)
	// (r,th): polar coordinate with respect to the object with the center fixed at the origin and the nozzle aligned to 0 rad
	// (Y): orientation about z-axis
	int nDist = 6;		// r in [0.5:0.1:1.0] + camera offset 			// TODO: define adequate sample range (frame_id: /top_shelf)
	double distMin = 0.5, distStep = 0.1;
	int nAng = 12;		// th in [0:30:330]
	double angMin = 0.0, angStep = 30.0/180.0*M_PI;
	int nYaw = 9;		// Y in [-20:5:20]
	double yawMin = -20.0/180.0*M_PI, yawStep = 5.0/180.0*M_PI;

	double pTot[nDist][nAng][nYaw], pGrasp[nDist][nAng][nYaw], pObs[nDist][nAng][nYaw], pWork[nDist][nAng][nYaw];
	bool bTotMax[nDist][nAng][nYaw];
	for (int i=0; i<nDist; i++)
		for (int j=0; j<nAng; j++)
			for (int k=0; k<nYaw; k++)
			{
				pTot[i][j][k] = 0.0;	
				pGrasp[i][j][k] = 1.0;	
				pObs[i][j][k] = 1.0;	
				pWork[i][j][k] = 1.0;
				bTotMax[i][j][k] = false;	// used when 1) check candidate validity before IK (process 1~4)
											//			 2) select the one with maximum pTot (process 5~6)
			}

	double robx[nDist][nAng][nYaw], roby[nDist][nAng][nYaw], robY[nDist][nAng][nYaw];	// candidate robot poses
	for (int i=0; i<nDist; i++)
		for (int j=0; j<nAng; j++)
			for (int k=0; k<nYaw; k++)
			{
				robx[i][j][k] = objx + cos(objY + angMin + angStep*j)*(distMin + distStep*i);
				roby[i][j][k] = objy + sin(objY + angMin + angStep*j)*(distMin + distStep*i);
				robY[i][j][k] = objY + (angMin + angStep*j) - M_PI - (yawMin + yawStep*k);		// angular coordinate of x-axis (frontal direction) of the robot with respect to global frame
			}


	// 1) probability of successful grasping 
	if (bCheckGrasp)
	{
		// heuristic probability of successful grasping
		// a) position: the object should be within left-hand side of the robot and 45 deg angle of view
			// exception: exclude poses at close distance, include ones in right-hand side at far distance
		// b) orientation: the object handle should be directed to the right of the robot
			// c) exception: allow more deviation at close distance, reject most of deviations at far distance
	
		int secDist[] = { (int)(nDist/3), (int)(nDist*2/3) };	// divide zone by distance (for acceptable object orientation setting)
		double secAngYaw[2];					// accept object orientations between these two values 
        double secSide[2] = {0.0, M_PI/4.0};    // divide left and right-hand side  // [rad]
		for (int i=0; i<nDist; i++)
		{
			// c) set acceptable object orientation range
			if (i < secDist[0])
			{
				secAngYaw[0] = 0.0/180.0*M_PI;
				secAngYaw[1] = 120.0/180.0*M_PI;
			}
			else if (i < secDist[1])
			{
				secAngYaw[0] = 0.0/180.0*M_PI;
				secAngYaw[1] = 90.0/180.0*M_PI;
			}
			else // if (i >= secDist[1])
			{
				secAngYaw[0] = 0.0/180.0*M_PI;
				secAngYaw[1] = 30.0/180.0*M_PI;
			}

			for (int j=0; j<nAng; j++)
			{
				for (int k=0; k<nYaw; k++)
				{
					// a) object position at left-hand side to the robot
					double rob2obj = atan2(objy-roby[i][j][k], objx-robx[i][j][k]);	// angular coordinate of a vector from robot position to object position (not orientation)
					double diffAng = wrapAngle( rob2obj-robY[i][j][k] );
					if ( diffAng >= secSide[0] && diffAng < secSide[1] )    // left-hand side and angle of view
					{
						// b) object handle orientation to the right of the robot
						double diffY = wrapAngle( objY-robY[i][j][k] );
						if ( diffY >= secAngYaw[0] && diffY <= secAngYaw[1] )
						{
							bTotMax[i][j][k] = true;	// using bTotMax instead of pGrasp to explicitly represent candidate validity (before expensive IK test)
						}
					}
				}
			}
		}
	}


	// 2) probability not to collide with obstacles
	// depends on distance to obstacles considering the position offset wrt orientation
	if (bCheckObs == true)
	{
		// TODO: not working with actionlib yet...
// 		double resolution = current_goal_->map.info.resolution;
// 		int width = current_goal_->map.info.width;
// 		int height = current_goal_->map.info.height;
// 		printf("width: %d   height: %d",width,height);
// 		geometry_msgs::Pose origin = current_goal_->map.info.origin;
// 		int i=0, j=0;
// 		printf("Printing map data...\n");
// 		printf("map data: %d\n",current_goal_->map.data[width*(i-1)+j]);

		if (bMapReceived_==1 && bRobPoseReceived_==1)
		{
			bMapReceived_ = -1;
			bRobPoseReceived_ = -1;

			double resolution = map_->info.resolution;
			int width = map_->info.width;
			int height = map_->info.height;
			geometry_msgs::Pose origin = map_->info.origin;
			printf("width: %d   height: %d   resolution: %f\n", width,height,resolution);

			printf("Printing map data...\n");
			double datax, datay;
			int i=100;
				for (int j=0; j<height; j+=10)
				{
					datax = origin.position.x + resolution*i;
					datay = origin.position.y + resolution*j;
					printf("map data (%d,%d): %f %f %d\n", i,j,datax,datay,map_->data[width*(j-1)+i]);
				}
			int j=100;
			for (int i=0; i<width; i+=10)
				{
					datax = origin.position.x + resolution*i;
					datay = origin.position.y + resolution*j;
					printf("map data (%d,%d): %f %f %d\n", i,j,datax,datay,map_->data[width*(j-1)+i]);
				}

// 			double armx, army, armY;	// arm center position biased from /top_shelf by armOffsety
// 			for (int i=0; i<nDist; i++)
// 				for (int j=0; j<nAng; j++)
// 					for (int k=0; k<nYaw; k++)
// 						if (bTotMax[i][j][k]==true)
// 						{
// 							armx = robx[i][j][k] - sin(robY[i][j][k])*armOffsety;
// 							army = roby[i][j][k] + cos(robY[i][j][k])*armOffsety;
// 
// 							map_->data[]
// 
// 
// 
// 
// 
// 							if (collision)
// 								bTotMax[i][j][k] = false;
// 						}


// 		double obsxMin = -5, obsxMax = 5;
// 		double obsyMin = -5, obsyMax = 5;
// 		double distR2W;				// distance from robot to wall (obstacle)
// 		double distR2Wthr = 1.0;	// distance threshold for downweight of obstacle probability
// 		double orderDistR2W = 2.0;	// order of distR2W for obstacle probabilty relation
// 		double robxoff, robyoff;	// (approximate) center position of arm with offset from robot position
// 		for (int i=0; i<nDist; i++)
// 			for (int j=0; j<nAng; j++)
// 			{
// 				robxoff = objx + cos(objY + angMin + angStep*j)*(distMin + distStep*i) - sin(objY + angMin + angStep*j)*(armOffsety);
// 				robyoff = objy + sin(objY + angMin + angStep*j)*(distMin + distStep*i) + cos(objY + angMin + angStep*j)*(armOffsety);
// 
// 				// for squared boundary & robot at center case only
// 				distR2W = std::min( std::min( std::max(0.0,obsxMax-robxoff), std::max(0.0,-(obsxMin-robxoff)) ), std::min( std::max(0.0,obsyMax-robyoff), std::max(0.0,-(obsyMin-robyoff)) ) );
// 
// 				pObs[i][j][k] = std::min(1.0, std::pow(distR2W/distR2Wthr,orderDistR2W));
// 			}






			bMapReceived_ = 0;
			bRobPoseReceived_ = 0;
		}

	}


	// 3) robot arm workspace limit should be included here!

	if (bCheckWork)
	{
		std::vector<double> pose = { 0,0,0, 0,0,0 };		// { x,y,z, r,p,y }		// TODO: check if this is wrt robot base frame
		std::vector<double> start = { 0,0,0,0,0,0,0 };		// TODO: check home pose of the arm
		std::vector<double> solution = { 0,0,0,0,0,0,0 };
		int option = 0;
		bool ret = false;

		for (int i=0; i<nDist; i++)
			for (int j=0; j<nAng; j++)
				for (int k=0; k<nYaw; k++)
					if (bTotMax[i][j][k]==true)
					{
						// TODO: check for arm home position and z-offset(-0.1?) from robot base to gastank handle
// 						pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,objY-robY[i][j][k] };
						pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,0.0 };

						ret = hdt_robot_model->computeIK(pose,start,solution,option);
						if (!ret)
							pWork[i][j][k] = 0.0; 
					}
	}


	// 4) total probability
	for (int i=0; i<nDist; i++)
		for (int j=0; j<nAng; j++)
			for (int k=0; k<nYaw; k++)
				if (bTotMax[i][j][k]==true)
					pTot[i][j][k] = pGrasp[i][j][k] * pObs[i][j][k] * pWork[i][j][k];	
				else 
					pTot[i][j][k] = 0.0;


	// 5) select maximum probability with min distance from robot
	// HEURISTICALLY, minimum difference in angular coordinate wrt object, then farthest from the origin of object
	double pTotMax = 1E-10;		// to opt out the poses with pTot[i][j][k]==0	
	for (int i=0; i<nDist; i++)
		for (int j=0; j<nAng; j++)
			for (int k=0; k<nYaw; k++)
				if (pTot[i][j][k] > pTotMax)
					pTotMax = pTot[i][j][k];
	int iMax = 0, jMax = 0, kMax = 0;		// indices for the final selection of robot pose
	int cntTotMax = 0;
	for (int i=0; i<nDist; i++)
		for (int j=0; j<nAng; j++)
			for (int k=0; k<nYaw; k++)
				if (pTot[i][j][k] == pTotMax)
				{
					bTotMax[i][j][k] = true;
					iMax = i;
					jMax = j;
					kMax = k;
					cntTotMax++;
				}
				else
					bTotMax[i][j][k] = false;

	ROS_INFO("\tcntTotMax: %d",cntTotMax);

	
	if (cntTotMax==0)
	{
		ROS_ERROR("No candidate pose was found!");
	}
	else if (cntTotMax > 1)	// if more than one candidate poses are selected
					 	// (we can select poses with pTot higher than a THRESHOLD)
	{
		// a) sorting by difference of angular coordinates for current and desired poses
		// ASSUME? backward driving is allowed for husky
		// HEURISTIC: minimize angle difference between robot orientation (robY) and robot motion direction (angO2Rcur)
		double angO2Rcur = atan2(roby0-objy,robx0-objx);	// current angular coordinate from object to robot
		double angO2Rdes, angO2Rerr, angO2RerrMin = M_PI;
		cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
		for (int i=0; i<nDist; i++)
			for (int j=0; j<nAng; j++)
				for (int k=0; k<nYaw; k++)
				{
					if (bTotMax[i][j][k] == true)
					{
						angO2Rdes = objY + angMin + angStep*j;		// desired angular coordinate from object to robot
						angO2Rerr = wrapAngle( angO2Rdes-angO2Rcur );

						if (fabs(angO2Rerr) <= angO2RerrMin)
						{
							angO2RerrMin = fabs(angO2Rerr);
							iMax = i;
							jMax = j;
							kMax = k;
							cntTotMax++;
						}
						else
							bTotMax[i][j][k] = false;
					}
				}
		// update bTotMax[i][j][k]
		for (int i=0; i<nDist; i++)
			for (int j=0; j<nAng; j++)
				for (int k=0; k<nYaw; k++)
					if (bTotMax[i][j][k] == true)
					{
						angO2Rdes = objY + angMin + angStep*j;
						angO2Rerr = wrapAngle( angO2Rdes-angO2Rcur );
						if (fabs(angO2Rerr) != angO2RerrMin)
						{
							bTotMax[i][j][k] = false;
							cntTotMax--;
						}
					}
		ROS_INFO("\tcntTotMax: %d",cntTotMax);
			
		
		if (cntTotMax > 1)	// if more than one candidate poses are still selected
		{
			// b) sorting by distance from current and desired positions
			double xysqerr, xysqerrMin = 1E10;		// distance from current to desired robot position
			cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
			for (int i=0; i<nDist; i++)
				for (int j=0; j<nAng; j++)
					for (int k=0; k<nYaw; k++)
					{
						if (bTotMax[i][j][k] == true)
						{
							xysqerr = std::pow(robx[i][j][k]-robx0,2.0) + std::pow(roby[i][j][k]-roby0,2.0);

							if (xysqerr <= xysqerrMin)
							{
								xysqerrMin = xysqerr;
								iMax = i;
								jMax = j;
								kMax = k;
								cntTotMax++;
							}
							else
								bTotMax[i][j][k] = false;
						}
					}

			// OPTIONAL: update bTotMax[i][j][k]
			for (int i=0; i<nDist; i++)
				for (int j=0; j<nAng; j++)
					for (int k=0; k<nYaw; k++)
						if (bTotMax[i][j][k] == true)
						{
							xysqerr = std::pow(robx[i][j][k]-robx0,2.0) + std::pow(roby[i][j][k]-roby0,2.0);
							if (xysqerr != xysqerrMin)
							{
								bTotMax[i][j][k] = false;
								cntTotMax--;
							}
						}
			ROS_INFO("\tcntTotMax: %d",cntTotMax);
			if (cntTotMax > 1)
				ROS_WARN("\tMultiple candidate poses exist!");
		
		}
	}

	// computing final desired robot pose with maximum pTot
	// /top_shelf pose
	robxf = robx[iMax][jMax][kMax];
	robyf = roby[iMax][jMax][kMax];
	robYf = robY[iMax][jMax][kMax];
	// /base_link pose
	robxf += cos(robYf)*baseOffsetx;
	robyf += sin(robYf)*baseOffsetx;
	ROS_INFO("Reposition Base Command Result:");
	ROS_INFO("\tObject Pose (initial): %f %f %f",objx,objy,wrapAngle(objY)*180/M_PI);
	ROS_INFO("\tRobot Pose (intial):   %f %f %f", robx0,roby0,wrapAngle(robY0)*180/M_PI);
	ROS_INFO("\tRobot Pose (desired):  %f %f %f", robxf,robyf,wrapAngle(robYf)*180/M_PI);


	return true;
}

