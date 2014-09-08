#include "RepositionBaseExecutor.h"


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

RepositionBaseExecutor::RepositionBaseExecutor()
{

}


int RepositionBaseExecutor::run()
{
	return 0;
}

/*
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


typedef actionlib::SimpleActionServer<hdt::RepositionBaseCommandAction> RepositionBaseCommandActionServer;


// # goal
// uint32 id
// int32 retry_count
// geometry_msgs/PoseStamped gas_can_in_map
// geometry_msgs/PoseStamped base_link_in_map
// nav_msgs/OccupancyGrid map 
// ---
// # result
// uint8 SUCCESS=0
// uint8 PLANNING_FAILURE_OBJECT_UNREACHABLE=1
// uint8 result
// geometry_msgs/PoseStamped[] candidate_base_poses
// ---
// # feedback
// float64 planning_time_s


double wrapAngle(double ang)
{
	while(ang >= M_PI)
		ang -= 2*M_PI;
	while(ang < -M_PI)
		ang += 2*M_PI;
	return ang;
}

void computeRobPose(double objx, double objy, double objY,  double robx0, double roby0, double robY0,  double& robxf, double& robyf, double& robYf, hdt::HDTRobotModel* hdt_robot_model) 
{
	// COMPARISON OPTIONS
	// a) flag for pGrasp operation
	bool bCheckGrasp = true;
// 	bool bCheckGrasp = false;

	// b) flag for pObs operation
// 	bool bCheckObs = true;
 	bool bCheckObs = false;

	// c) arm position offset for computing pObs
// 	double armOffsety = 0.0;
// 	double armOffsety = 0.5;
	double armOffsety = 1.0;


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
// 	int nDist = 5;		// r in [0.762:0.0762:1.0668] + camera offset 	// Hao's sample
// 	double distMin = 0.762, distStep = 0.0762;
	int nDist = 6;		// r in [0.5:0.1:1.0] + camera offset 			// TODO: define adequate sample range
	double distMin = 0.5, distStep = 0.1;
	int nAng = 12;		// th in [0:30:330]
	double angMin = 0.0, angStep = 30.0/180.0*M_PI;
	int nYaw = 9;		// Y in [-20:5:20]
	double yawMin = -20.0/180.0*M_PI, yawStep = 5.0/180.0*M_PI;
// 	double wTotxy = 0.7, wTotY = 0.3;		// weights for heuristically selecting the best among tiers

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
				robY[i][j][k] = objY + (angMin + angStep*j) - M_PI + (yawMin + yawStep*k);		// angular coordinate of x-axis (frontal direction) of the robot with respect to global frame
			}


	// 1) probability of successful grasping 
	if (bCheckGrasp)
	{
		// probability of successful object pose recognition (by Hao)
// 		pGrasp[0][0]  = 1.0;	pGrasp[1][0]  = .95;	pGrasp[2][0]  = .95;	pGrasp[3][0]  = 1.0;	pGrasp[4][0]  = .90;	
// 		pGrasp[0][1]  = .60;	pGrasp[1][1]  = .70;	pGrasp[2][1]  = .70;	pGrasp[3][1]  = .75;	pGrasp[4][1]  = .50;	
// 		pGrasp[0][2]  = .80;	pGrasp[1][2]  = .00;	pGrasp[2][2]  = .00;	pGrasp[3][2]  = .00;	pGrasp[4][2]  = .70;	
// 		pGrasp[0][3]  = .00;	pGrasp[1][3]  = 1.0;	pGrasp[2][3]  = 1.0;	pGrasp[3][3]  = 1.0;	pGrasp[4][3]  = .00;	
// 		pGrasp[0][4]  = .00;	pGrasp[1][4]  = .95;	pGrasp[2][4]  = .95;	pGrasp[3][4]  = 1.0;	pGrasp[4][4]  = 1.0;	
// 		pGrasp[0][5]  = 1.0;	pGrasp[1][5]  = 1.0;	pGrasp[2][5]  = 1.0;	pGrasp[3][5]  = .85;	pGrasp[4][5]  = 1.0;	
// 		pGrasp[0][6]  = 1.0;	pGrasp[1][6]  = 1.0;	pGrasp[2][6]  = 1.0;	pGrasp[3][6]  = .95;	pGrasp[4][6]  = 1.0;	
// 		pGrasp[0][7]  = 1.0;	pGrasp[1][7]  = .65;	pGrasp[2][7]  = .65;	pGrasp[3][7]  = .60;	pGrasp[4][7]  = .75;	
// 		pGrasp[0][8]  = 1.0;	pGrasp[1][8]  = .90;	pGrasp[2][8]  = .90;	pGrasp[3][8]  = .95;	pGrasp[4][8]  = .75;	
// 		pGrasp[0][9]  = .00;	pGrasp[1][9]  = .00;	pGrasp[2][9]  = .00;	pGrasp[3][9]  = .00;	pGrasp[4][9]  = .00;	
// 		pGrasp[0][10] = .00;	pGrasp[1][10] = .00;	pGrasp[2][10] = .00;	pGrasp[3][10] = .00;	pGrasp[4][10] = .85;	
// 		pGrasp[0][11] = 1.0;	pGrasp[1][11] = 1.0;	pGrasp[2][11] = .80;	pGrasp[3][11] = .80;	pGrasp[4][11] = .80;	


		// heuristic probability of successful grasping
		// a) position: the object should be within left-hand side of the robot and 45 deg angle of view
			// exception: exclude poses at close distance, include ones in right-hand side at far distance
		// b) orientation: the object handle should be directed to the right of the robot
			// c) exception: allow more deviation at close distance, reject most of deviations at far distance
	
// 		int secDist1 = (int)(nDist/3), secDist2 = (int)(nDist*2/3);
// 		int secAngYaw1 = (int)(nDist/3), secAngYaw2 = (int)(nDist*2/3);
// 		double secDist1 = (int)(nDist/3), secDist2 = (int)(nDist*2/3);
// 		double secAngYaw = { (yawMin + (yawStep*(nYaw-1))/3.0 ), };
// 		double secAngYaw = { 0, 30, 60 };
		int secDist[] = { (int)(nDist/3), (int)(nDist*2/3) };	// divide zone by distance (for acceptable object orientation setting)
		double secAngYaw[2];	// accept object orientations between these two values 
		double secSide = 0.0;	// divide left and right-hand side	// [rad]
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
					if ( diffAng >= secSide )	// left-hand side
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
	// obstacle map; should be implemented here!; temporarily, obstacles of walls in a square room
// // 	double obsxMin = -5, obsxMax = 5;
// // 	double obsyMin = -5, obsyMax = 5;
// 	double obsxMin = -5, obsxMax = 5;
// 	double obsyMin = -5, obsyMax = 5;
// 	double distR2W;				// distance from robot to wall (obstacle)
// 	double distR2Wthr = 1.0;	// distance threshold for downweight of obstacle probability
// 	double orderDistR2W = 2.0;	// order of distR2W for obstacle probabilty relation
// 	double robxoff, robyoff;	// (approximate) center position of arm with offset from robot position
// 	for (int i=0; i<nDist; i++)
// 		for (int j=0; j<nAng; j++)
// 		{
// 			robxoff = objx + cos(objY + angMin + angStep*j)*(distMin + distStep*i) - sin(objY + angMin + angStep*j)*(armOffsety);
// 			robyoff = objy + sin(objY + angMin + angStep*j)*(distMin + distStep*i) + cos(objY + angMin + angStep*j)*(armOffsety);
// 
// 			// for squared boundary & robot at center case only
// 			distR2W = std::min( std::min( std::max(0.0,obsxMax-robxoff), std::max(0.0,-(obsxMin-robxoff)) ), std::min( std::max(0.0,obsyMax-robyoff), std::max(0.0,-(obsyMin-robyoff)) ) );
// // 			ROS_INFO("distR2W: %f",distR2W);
// 
// 			pObs[i][j][k] = std::min(1.0, std::pow(distR2W/distR2Wthr,orderDistR2W));
// 			if (bCheckObs == false)
// 				pObs[i][j][k] = 1.0;	// for comparison only
// 		}


	// 3) robot arm workspace limit should be included here!

	std::vector<double> pose = { 0,0,0, 0,0,0 };		// { x,y,z, r,p,y }		// TODO: check if this is wrt robot base frame
	std::vector<double> start = { 0,0,0,0,0,0,0 };		// TODO: check home pose of the arm
	std::vector<double> solution = { 0,0,0,0,0,0,0 };
	int option = 0;
	bool ret = false;

	// test for forward kinematics to get home position
// 	start = { M_PI/2.0,0,0,0,0,0,0 };
// 	ret = hdt_robot_model->computePlanningLinkFK(start,pose);
// 	ROS_INFO("Home pose: %f %f %f  %f %f %f",pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);

	for (int i=0; i<nDist; i++)
		for (int j=0; j<nAng; j++)
			for (int k=0; k<nYaw; k++)
				if (bTotMax[i][j][k]==true)
				{
					// TODO: check for arm home position and z-offset(-0.1?) from robot base to gastank handle
// 					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,objY-robY[i][j][k] };
					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,0.0 };
// 					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,M_PI };
// 					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, M_PI,0.0,0.0 };
// 					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,M_PI,0.0 };
// 					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, -M_PI/2.0,M_PI,0.0 };
// 					pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, -M_PI/2.0,0.0,M_PI };

// 					ret = hdt_robot_model->computeIK(pose,start,solution,option);
// 					if (!ret)
// 						pWork[i][j][k] = 0.0; 
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
// 	double pTotMax = 0.0;
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

	ROS_INFO("pTotMax: %f",pTotMax);
	ROS_INFO("cntTotMax: %d",cntTotMax);

	
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
// 						ROS_INFO("angO2RerrMin: %f    (i: %d, j: %d)",angO2RerrMin,i,j);
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
// 		ROS_INFO("angO2RerrMin: %f",angO2RerrMin);
		ROS_INFO("cntTotMax: %d",cntTotMax);
			
		
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
								// 							ROS_INFO("xysqerrMin: %f    (i: %d, j: %d)",xysqerrMin,i,j);
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
// 			ROS_INFO("xysqerrMin: %f",xysqerrMin);
			ROS_INFO("cntTotMax: %d",cntTotMax);
			if (cntTotMax > 1)
				ROS_WARN("Multiple candidate poses exist!");
		
		}
	}
	ROS_INFO("iMax: %d,    jMax: %d    kMax: %d",iMax,jMax,kMax);

	// computing final desired robot pose with maximum pTot
	robxf = objx + cos(objY + angMin + angStep*jMax)*(distMin + distStep*iMax);		// kMax comes here!
	robyf = objy + sin(objY + angMin + angStep*jMax)*(distMin + distStep*iMax);
	robYf = objY + angMin + angStep*jMax - M_PI; 
	ROS_INFO("Object Pose: %f %f %f",objx,objy,objY*180/M_PI);
	ROS_INFO("Robot Pose: %f %f %f", robxf,robyf,robYf*180/M_PI);


	// 6) conversion for /camera_link frame
	double robxfswp=robxf, robyfswp=robyf;
	robYf = -robYf;
	robxf = -cos(robYf)*robxfswp + sin(robYf)*robyfswp;
	robyf = -sin(robYf)*robxfswp - cos(robYf)*robyfswp;
	ROS_INFO("Robot Pose (cvt): %f %f %f", robxf,robyf,robYf*180/M_PI);

}

void goal_callback()
{
}

void preempt_callback()
{
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "hdt_base_planning_node");
	ros::NodeHandle nh;


	// object pose parameters for simulation
// 	double objx = 1.0, objy = -0.5, objz = -0.05;	// for /base_link frame
	double objx = 1.0, objy = -0.5, objz = 0.1;		// for /camera_link frame
	double objR = M_PI/2, objP = 0.0, objY = 0.0;
	if (nh.hasParam("hdt_base_planning_node/objx"))
		nh.getParam("hdt_base_planning_node/objx",objx);
	if (nh.hasParam("hdt_base_planning_node/objy"))
		nh.getParam("hdt_base_planning_node/objy",objy);
	if (nh.hasParam("hdt_base_planning_node/objY"))
	{
		nh.getParam("hdt_base_planning_node/objY",objY);
		objY *= M_PI/180.0;			// when "objY" is in [deg]
	}
// 	ROS_INFO("Object Pose: %f %f %f",objx,objy,objY*180/M_PI);


	// action server initialization
	std::string action_name_ = "reposition base command";
	std::unique_ptr<RepositionBaseCommandActionServer> as_;
	as_.reset(new RepositionBaseCommandActionServer(action_name_, false));
	if (!as_) {
		ROS_ERROR("Failed to instantiate Reposition Base Command Action Server");
		return false;
	}   

	as_->registerGoalCallback(boost::bind(&goal_callback));
	as_->registerPreemptCallback(boost::bind(&preempt_callback));

	ROS_INFO("Starting action server '%s'...", action_name_.c_str());
	as_->start();
	ROS_INFO("Action server started");



	
	// visualization marker for gastank
	ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "hdt_base_planning_node/visualization_marker", 1 );

	visualization_msgs::Marker marker;
// 	marker.header.frame_id = "/base_link";
	marker.header.frame_id = "/camera_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "hdt_base_planning_node";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = objx;
	marker.pose.position.y = objy;
	marker.pose.position.z = objz;

	tf::Quaternion q = tf::createQuaternionFromRPY(objR,objP,objY);
	marker.pose.orientation.x = q[0];
	marker.pose.orientation.y = q[1];
	marker.pose.orientation.z = q[2];
	marker.pose.orientation.w = q[3];

	// marker.scale.x = 10;	// gastank_c.dae
	// marker.scale.y = 10;
	// marker.scale.z = 10;
	marker.scale.x = 0.1;	// gastank_c_sparse.dae
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.lifetime = ros::Duration();

	//only if using a MESH_RESOURCE marker type:
// 	marker.mesh_resource = "package://hdt_base_planning/meshes/gastank_c_sparse.dae";
	marker.mesh_resource = "package://hdt/resource/meshes/gastank/gastank_c.dae";


	// to give some time to publish markers
	int cntPub = 0;
	ros::Rate r(10);
// 	while(ros::ok())
	while(cntPub < 5)
	{
		vis_pub.publish( marker );

		ros::spinOnce();
		r.sleep();
		cntPub++;
	}


	// hdt robot initialization for inverse kinematics check
	std::string group_name_;
	std::string kinematics_frame_;
	std::string planning_frame_;
	std::string planning_link_;
	std::string chain_tip_link_;
	std::string urdf_;
    boost::shared_ptr<urdf::ModelInterface> urdf_model_;

	if (!nh.hasParam("robot_description")) {
		ROS_ERROR("Missing parameter \"robot_description\"");
		return false;
	}   

	nh.getParam("robot_description", urdf_);

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

	hdt::HDTRobotModel* hdt_robot_model = new hdt::HDTRobotModel;
	if (!hdt_robot_model || !hdt_robot_model->init(urdf_)) {
		ROS_ERROR("Failed to initialize HDT Robot Model");
		return false;
	}   

// 	std::vector<double> pose = { 1.5,0,0, 0,0,0 };	// { x,y,z, r,p,y }
// 	std::vector<double> start = { 0,0,0,0,0,0,0 };
// 	std::vector<double> solution = { 0,0,0,0,0,0,0 };
// 	int option = 0;
// 	bool ret = hdt_robot_model->computeIK(pose,start,solution,option);
	


	// COMPUTE ROBOT POSE HERE
  	double robx0=0, roby0=0, robz0=-0.65, robY0=0, robP0=0, robR0=0;						// initial robot pose	// z-offset for /camera_link frame
  	double robxf=robx0, robyf=roby0, robzf=robz0, robYf=robY0, robPf=robP0, robRf=robR0;	// final (computed) robot pose
	computeRobPose(objx,objy,objY, robx0,roby0,robY0, robxf,robyf,robYf, hdt_robot_model); 


	// VISUALIZE THE ROBOT 
	std::stringstream str;
	str << "rosrun hdt hdt_base_planning_shell.sh " <<  robxf << " " << robyf << " " << robzf << " " << robYf << " " << robPf << " " << robRf;
	if (system(str.str().c_str())!=-1)
	{
		ROS_INFO("Robot Pose Established!\n");
	}


// 	int cntPub = 0;
// 	ros::Rate r(10);
// // 	while(ros::ok())
// 	while(cntPub < 5)
// 	{
// 		vis_pub.publish( marker );
// 
// 		ros::spinOnce();
// 		r.sleep();
// 		cntPub++;
// 	}


	nh.deleteParam("hdt_base_planning_node/objx");
	nh.deleteParam("hdt_base_planning_node/objy");
	nh.deleteParam("hdt_base_planning_node/objY");

	return 0;
}
*/
