#include "RepositionBaseExecutor.h"

#include <eigen_conversions/eigen_msg.h>
#include <sbpl_geometry_utils/utils.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <tf_conversions/tf_eigen.h>

#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/common/stringifier/stringifier.h>
#include <algorithm>

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
bComputedRobPose_(true),
current_goal_(),
status_(RepositionBaseExecutionStatus::INVALID),
last_status_(RepositionBaseExecutionStatus::INVALID),
    ph_("~"),
    grasp_spline_(),
    gas_can_scale_(),
    wrist_to_tool_(),
    pregrasp_to_grasp_offset_m_(0.0),
    generated_grasps_(false),
    last_move_arm_pregrasp_goal_(),
    pending_move_arm_command_(false),
    sent_move_arm_goal_(false),
    move_arm_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    move_arm_command_result_(),
    move_arm_command_action_name_("move_arm_command"),
    move_arm_command_client_(),
    listener_()
{
}

bool RepositionBaseExecutor::initialize()
{
	// hdt robot initialization for inverse kinematics check
	std::string group_name_;
	std::string kinematics_frame_;
	std::string planning_frame_;
	std::string planning_link_;
	std::string chain_tip_link_;
	std::string urdf_;
	boost::shared_ptr<urdf::ModelInterface> urdf_model_;

    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model from the URDF");
        return false;
    }

     if (!msg_utils::download_param(ph_, "gas_canister_mesh", gas_can_mesh_path_) ||
        !msg_utils::download_param(ph_, "gas_canister_mesh_scale", gas_can_scale_))
    {
        ROS_ERROR("Failed to download gas canister parameters");
        return false;
    }

   // read in max grasps
    if (!msg_utils::download_param(ph_, "max_grasp_candidates", max_grasp_candidates_) ||
        max_grasp_candidates_ < 0)
    {
        ROS_ERROR("Failed to retrieve 'max_grasp_candidates' from the param server or 'max_grasp_candidates' is negative");
        return false;
    }

    std::vector<geometry_msgs::Point> control_points;
    int degree;
    if (!msg_utils::download_param(ph_, "degree", degree) ||
        !msg_utils::download_param(ph_, "control_points", control_points))
    {
        ROS_ERROR("Failed to retrieve grasp spline parameters");
        return false;
    }

   std::vector<Eigen::Vector3d> grasp_spline_control_points(control_points.size());
    for (std::size_t i = 0; i < control_points.size(); ++i) {
        const geometry_msgs::Point& p = control_points[i];
        grasp_spline_control_points[i] = Eigen::Vector3d(p.x, p.y, p.z);
    }

    grasp_spline_.reset(new Nurb<Eigen::Vector3d>(grasp_spline_control_points, degree));
    if (!grasp_spline_) {
        ROS_ERROR("Failed to instantiate Nurb");
        return false;
    }

    ROS_INFO("Control Points:");
    for (const Eigen::Vector3d& control_vertex : grasp_spline_->control_points()) {
        ROS_INFO("    %s", to_string(control_vertex).c_str());
    }

    ROS_INFO("Knot Vector: %s", to_string(grasp_spline_->knots()).c_str());
    ROS_INFO("Degree: %s", std::to_string(grasp_spline_->degree()).c_str());

    ROS_INFO("Control Points:");
    for (const Eigen::Vector3d& control_vertex : grasp_spline_->control_points()) {
        ROS_INFO("    %s", to_string(control_vertex).c_str());
    }

    ROS_INFO("Knot Vector: %s", to_string(grasp_spline_->knots()).c_str());
    ROS_INFO("Degree: %s", std::to_string(grasp_spline_->degree()).c_str());

    geometry_msgs::Pose tool_pose_wrist_frame;
    if (!msg_utils::download_param(ph_, "wrist_to_tool_transform", tool_pose_wrist_frame)) {
        ROS_ERROR("Failed to retrieve 'wrist_to_tool_transform' from the param server");
        return false;
    }

    tf::poseMsgToEigen(tool_pose_wrist_frame, wrist_to_tool_);
    ROS_INFO("Wrist-to-Tool Transform: %s", to_string(wrist_to_tool_).c_str());

    if (!msg_utils::download_param(ph_, "pregrasp_to_grasp_offset_m", pregrasp_to_grasp_offset_m_)) {
        ROS_ERROR("Failed to retrieve 'pregrasp_to_grasp_offset_m' from the param server");
        return false;
    }

    grasp_to_pregrasp_ = Eigen::Translation3d(-pregrasp_to_grasp_offset_m_, 0, 0);

    marker_arr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);

    move_arm_command_client_.reset(new MoveArmCommandActionClient(move_arm_command_action_name_, false));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Client");
        return false;
    }

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
}


int RepositionBaseExecutor::run()
{
	if (!initialize()) {
		return FAILED_TO_INITIALIZE;
	}

	status_ = RepositionBaseExecutionStatus::IDLE;

	ros::Rate loop_rate(1);
	while (ros::ok()) {
		RunUponDestruction rod([&](){ loop_rate.sleep(); });
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
						objY = wrapAngle(objY-M_PI/2.0);	// M_PI/2 offset due to definition of object frame in new mesh file 
						double objP = 0.0;
						double objR = 0.0;

						// initial robot pose
						double robx0 = current_goal_->base_link_in_map.pose.position.x;
						double roby0 = current_goal_->base_link_in_map.pose.position.y;
						double robz0 = current_goal_->base_link_in_map.pose.position.z;
						double robY0 = 2.0*acos(current_goal_->base_link_in_map.pose.orientation.w)*sign(current_goal_->base_link_in_map.pose.orientation.z);	// assuming that rotation axis is parallel to z-axis
						double robP0 = 0.0;
						double robR0 = 0.0;
						// transform from /base_link to /top_shelf (in 2D)
// 						double baseOffsetx = -0.49+0.148975;	// /base_link to /base_link_front_bumper_part to /top_shelf 			// same as in computeRobPose()
						double baseOffsetx = -0.49;				// /base_link to /base_link_front_bumper_part (in new Hokuyo setting) 	// same as in computeRobPose()
						robx0 -= cos(robY0)*baseOffsetx;
						roby0 -= sin(robY0)*baseOffsetx;

						// desired robot pose
// 						double robxf=robx0, robyf=roby0, robzf=robz0, robYf=robY0, robPf=robP0, robRf=robR0;	// final (computed) robot pose


						std::vector<geometry_msgs::PoseStamped> candidate_base_poses;
					
						
						// check for inverse kinematics and arm planning (if object is in reachable range and within angle of view)
						double secDist[2] = {0.5, 1.5};		// TODO: a more general than (distMin + (nDist-1)*distStep) determined in computeRobPose()
						double distRob2Obj = std::sqrt( std::pow(objx-robx0,2) + std::pow(objy-roby0,2) );	
						if (distRob2Obj >= secDist[0] && distRob2Obj <= secDist[1]) {
							double secSide[2] = {-15.0/180.0*M_PI, 45.0/180.0*M_PI};	// TODO: a more general than secSide[2] determined in computeRobPose()
							double rob2obj = atan2(objy-roby0, objx-robx0);	// angular coordinate of a vector from robot position to object position
							double diffAng = wrapAngle( rob2obj-robY0 );
							if ( diffAng >= secSide[0] && diffAng <= secSide[1] ) {		// left-hand side and angle of view
								int retIKPLAN = checkIKPLAN();	// -4,-3,-2: inverse kinematics failed, -1,0: arm planning failed, +1: possible to grasp
								//std::cerr << "retIKPLAN: " << retIKPLAN << std::endl;
								if (retIKPLAN==1) {		// you can grasp it now!
									hdt_msgs::RepositionBaseCommandResult result;
									result.result = hdt_msgs::RepositionBaseCommandResult::SUCCESS;

									candidate_base_poses.push_back(current_goal_->base_link_in_map);
									result.candidate_base_poses = candidate_base_poses;
									as_->setSucceeded(result);
									status_ = RepositionBaseExecutionStatus::IDLE;

									bComputedRobPose_ = true;
									break;
								}
							}
						}


						// compute robot pose
						bool ret = computeRobPose(objx,objy,objY, robx0,roby0,robY0, candidate_base_poses);
// //					hdt_msgs::RepositionBaseCommandFeedback feedback;
// // 					feedback.status = execution_status_to_feedback_status(status_?);
// // 					as_->publishFeedback(feedback);
// 						status_ = RepositionBaseExecutionStatus::COMPLETING_GOAL;

						if (ret)
						{
							hdt_msgs::RepositionBaseCommandResult result;
							result.result = hdt_msgs::RepositionBaseCommandResult::SUCCESS;
							result.candidate_base_poses = candidate_base_poses;
							as_->setSucceeded(result);
							status_ = RepositionBaseExecutionStatus::IDLE;
						}
						else
						{
							hdt_msgs::RepositionBaseCommandResult result;
							result.result = hdt_msgs::RepositionBaseCommandResult::PLANNING_FAILURE_OBJECT_UNREACHABLE;
							result.candidate_base_poses = candidate_base_poses;		// this pose is same with the initial base pose
							as_->setSucceeded(result);
							status_ = RepositionBaseExecutionStatus::IDLE;
						}
						bComputedRobPose_ = true;
					}
				}   break;
			case RepositionBaseExecutionStatus::COMPLETING_GOAL:
				{
// 					hdt_msgs::RepositionBaseCommandResult result;
// 					result.result = hdt_msgs::RepositionBaseCommandResult::SUCCESS;
// 					result.candidate_base_poses =
// 					as_->setSucceeded(result);
// 					status_ = RepositionBaseExecutionStatus::IDLE;
				}   break;
			default:
				break;
		}
	}

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
	ROS_INFO("    Occupancy Grid Seq: %d", current_goal_->map.header.seq);
	ROS_INFO("    Occupancy Grid Stamp: %d", current_goal_->map.header.stamp.sec);
	ROS_INFO("    Occupancy Grid Frame ID: %s", current_goal_->map.header.frame_id.c_str());

	bComputedRobPose_ = false;
	
	generated_grasps_ = false;
    sent_move_arm_goal_ = false;
    pending_move_arm_command_ = false;
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


////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////1qa


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

bool RepositionBaseExecutor::computeRobPose(double objx, double objy, double objY,  double robx0, double roby0, double robY0,  std::vector<geometry_msgs::PoseStamped>& candidate_base_poses)
{

	// policy for robot pose selection
	// 0) set search space
	// 1) candidate pose validity for grasing
	// 2) distance to obstacles
	// 3) kinematics of hdt arm
	// 4) multiplication of 1~3)
	// 5) candidate sorting by some metrics
	// 6) generate final candidate base poses


	// PARAMETER SETTING

	// 1) flag for pGrasp operation
	bool bCheckGrasp = true;
// 	bool bCheckGrasp = false;

	// 2) flag for pObs operation
	bool bCheckObs = true;		// TODO: sometimes map data is corrupted and NO candidate_base_pose can be found!
// 	bool bCheckObs = false;

	// 3) flag for pWork operation
// 	bool bCheckWork = true;
	bool bCheckWork = false;

	// 5) flag for candidate sorting
	int bSortMetric = 3;		// 1: angle, 2: angle, position, 3: pTot threshold (default)


	// 0) search space
	int nDist = 6;		// r in [0.5:0.1:1.0] + camera offset 		// TODO: define adequate sample range (frame_id: /top_shelf)
	double distMin = 0.5, distStep = 0.1;
// 	int nDist = 6;		// r in [0.6:0.1:1.1] + camera offset 		// TODO: define adequate sample range (frame_id: /top_shelf)
// 	double distMin = 0.6, distStep = 0.1;
	int nAng = 12;		// th in [0:30:330]
	double angMin = 0.0, angStep = 30.0/180.0*M_PI;
	int nYaw = 9;		// Y in [-20:5:20]
	double yawMin = -20.0/180.0*M_PI, yawStep = 5.0/180.0*M_PI;
// 	int nYaw = 5;		// Y in [-20:5:20]
// 	double yawMin = -10.0/180.0*M_PI, yawStep = 5.0/180.0*M_PI;

	// 1) grasp achievable zone 	// TODO: tune these parameters
	int secDist[] = { (int)(nDist/3), (int)(nDist*2/3) };	// divide zone by distance (for acceptable object orientation setting)
	double secAngYaw[2];							// accept object orientations between these two values 	// seperately defined for different regions by secDist (values within the detail code of 1))
	double bestAngYaw;								// best object orientation to get highest pGrasp probability (set as the mean value of secAngYaw[0~1] currently)
	double bestDist = 0.70;							// best object distance to get highest pGrasp probability (set as the nominal value from experiment)
//	double secSide[2] = {0.0, M_PI/4.0};    		// divide left and right-hand side  // [rad]
	double secSide[2] = {0.0, 40.0/180.0*M_PI};		// divide left and right-hand side  // [rad]
// 	double secSide[2] = {0.0, 20.0/180.0*M_PI};		// divide left and right-hand side  // [rad]
// 	double scalepGraspAngYaw = 0.1;	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
// 	double scalepGraspDist = 0.1;	// pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)
	double scalepGraspAngYaw = 0.05;	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
	double scalepGraspDist = 0.05;	// pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)

	// 2) arm position offset for computing pObs
// 	double armOffsety = -0.0;
// 	double armOffsety = -0.5;
// 	double armOffsety = -1.1;	// center of arm (when straightend)
// 	double armLength = 1.5;		// workspace radius about the center of arm
// 	double armLengthCore = 0.6;	// core workspace radius about the center of arm (pObs = 0.0)
	double armOffsetx = 0.3;	// center of arm (for normal grasping motion)
	double armOffsety = -0.6;	// center of arm (for normal grasping motion)
	double armLength = 1.0;		// workspace radius about the center of arm
	double armLengthCore = 0.6;	// core workspace radius about the center of arm (pObs = 0.0)
	double bodyOffsetx = -0.49+0.148975;	// center of body (except arm)
	double bodyLength = 1.0;	// support polygon radius about the center of body
	double bodyLengthCore = 0.65;	// core support polygon radius about the center of body (pObs = 0.0)
	int mapObsThr = 1;			// threshold for classifying clear and occupied regions

	// 5) candidate selection criterion
	double scaleDiffYglob = 0.05;	// multiply a quadratic function to pTot (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI) for bSortMetric==3
	double pTotThr = 0.8;		// pTot threshold for bSortMetric==3

	// 6) /base_link offset from /top_shelf for final robot pose return
// 	double baseOffsetx = -0.0;
// 	double baseOffsetx = -0.3;
// 	double baseOffsetx = -0.5;
// 	double baseOffsetx = -0.49+0.148975;	// /base_link to /base_link_front_bumper_part to /top_shelf
	double baseOffsetx = -0.49;				// /base_link to /base_link_front_bumper_part (in new Hokuyo setting) 	// same as in computeRobPose()



////////////////////////////////////////////////////////////////////////////////////



	// 0) set search space (r,th,Y)
	// (r,th): polar coordinate with respect to the object with the center fixed at the origin and the nozzle aligned to 0 rad
	// (Y): orientation about z-axis

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

	double robz0 = 0.0, robP0 = 0.0, robR0 = 0.0;
	double robxf=robx0, robyf=roby0, robzf=robz0, robYf=robY0, robPf=robP0, robRf=robR0;	// final (computed) robot pose

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

		for (int i=0; i<nDist; i++)
		{
			// c) set acceptable object orientation range
			if (i < secDist[0])
			{
				secAngYaw[0] = 0.0/180.0*M_PI;
				secAngYaw[1] = 120.0/180.0*M_PI;
// 				bestAngYaw   = 60.0/180.0*M_PI;
				bestAngYaw   = 65.0/180.0*M_PI;
			}
			else if (i < secDist[1])
			{
				secAngYaw[0] = 0.0/180.0*M_PI;
				secAngYaw[1] = 90.0/180.0*M_PI;
// 				bestAngYaw   = 45.0/180.0*M_PI;
				bestAngYaw   = 60.0/180.0*M_PI;
			}
			else // if (i >= secDist[1])
			{
				secAngYaw[0] = 0.0/180.0*M_PI;
// 				secAngYaw[1] = 30.0/180.0*M_PI;
				secAngYaw[1] = 60.0/180.0*M_PI;
// 				bestAngYaw   = 15.0/180.0*M_PI;
				bestAngYaw   = 50.0/180.0*M_PI;
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
						double diffYMax = std::max( fabs(secAngYaw[0]-bestAngYaw), fabs(secAngYaw[1]-bestAngYaw) );
						double diffDistMax = std::max( fabs(distMin-bestDist), fabs(distMin+distStep*(nDist-1)-bestDist) );
						if ( diffY >= secAngYaw[0] && diffY <= secAngYaw[1] )
						{
							bTotMax[i][j][k] = true;	// using bTotMax instead of pGrasp to explicitly represent candidate validity (before expensive IK test)

							// higher probability around diffY==bestAngYaw
// 							pGrasp[i][j][k] = std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 );	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
// 							pGrasp[i][j][k] = std::max( std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 ), pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
							
							// higher probability around diffY==bestAngYaw and diffDist==bestDist
							pGrasp[i][j][k] = std::max( std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0) * std::pow( (diffDistMax-fabs(distMin+distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0), pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
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
		map_ = current_goal_->map;

// 		if (bMapReceived_==1 && bRobPoseReceived_==1)
		{
// 			bMapReceived_ = -1;
// 			bRobPoseReceived_ = -1;

			double resolution = map_.info.resolution;
			int width = map_.info.width;
			int height = map_.info.height;
            geometry_msgs::Pose origin = map_.info.origin;
			// OccupancyGrid index usage
			//	mapx = origin.position.x + resolution*ii;	// x-position in world_frame
			//	mapy = origin.position.y + resolution*jj;	// y-position in world_frame
			//	mapObs = map_.data[width*jj+ii];			// probability of occupancy in this (x,y) position 	// -1: unknown, 0: clear, 1-100: higher probability

			int patchSize2 = (int)(armLength/resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
			double armx, army, armY;	// arm center position biased from /top_shelf by armOffsety
			int armi, armj;				// index for patch center
			for (int i=0; i<nDist; i++)
				for (int j=0; j<nAng; j++)
					for (int k=0; k<nYaw; k++)
						if (bTotMax[i][j][k]==true)
						{
// 							armx = robx[i][j][k] - sin(robY[i][j][k])*armOffsety;
// 							army = roby[i][j][k] + cos(robY[i][j][k])*armOffsety;
							armx = robx[i][j][k] + cos(robY[i][j][k])*armOffsetx - sin(robY[i][j][k])*armOffsety;
							army = roby[i][j][k] + sin(robY[i][j][k])*armOffsetx + cos(robY[i][j][k])*armOffsety;

							armi = (int)((armx-origin.position.x)/resolution);
							armj = (int)((army-origin.position.y)/resolution);

							if (armi>=patchSize2 && armi<width-patchSize2 && armj>=patchSize2 && armj<height-patchSize2)
							{
								bool bCollided = false;
								for (int ii=-patchSize2; ii<=patchSize2 && !bCollided; ii++)
									for (int jj=-patchSize2; jj<=patchSize2 && !bCollided; jj++)
										if (map_.data[width*(armj+jj)+armi+ii] >= mapObsThr)		// including unknown region
//										if (map_.data[width*(armj+jj)+armi+ii] != 0)				// only in clear region
										{
											double distObs = std::sqrt((double)(ii*ii+jj*jj))*resolution;
											if (distObs < armLengthCore)
											{
												bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
												bCollided = true;
											}
											else if (distObs < armLength)
												pObs[i][j][k] = std::min( pObs[i][j][k], std::pow( (distObs-armLengthCore)/(armLength-armLengthCore), 2.0) );	// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
										}
							}
							else
							{
// 								bTotMax[i][j][k] = false;
// 								ROS_ERROR("    Patch for obstacle check is out of the map");
							}
						}

			patchSize2 = (int)(bodyLength/resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
			double bodyx, bodyy, bodyY;		// body center position biased from /top_shelf by bodyOffsetx
			int bodyi, bodyj;				// index for patch center
			for (int i=0; i<nDist; i++)
				for (int j=0; j<nAng; j++)
					for (int k=0; k<nYaw; k++)
						if (bTotMax[i][j][k]==true)
						{
							bodyx = robx[i][j][k] + cos(robY[i][j][k])*bodyOffsetx;
							bodyy = roby[i][j][k] + sin(robY[i][j][k])*bodyOffsetx;

							bodyi = (int)((bodyx-origin.position.x)/resolution);
							bodyj = (int)((bodyy-origin.position.y)/resolution);

							if (bodyi>=patchSize2 && bodyi<width-patchSize2 && bodyj>=patchSize2 && bodyj<height-patchSize2)
							{
								bool bCollided = false;
								for (int ii=-patchSize2; ii<=patchSize2 && !bCollided; ii++)
									for (int jj=-patchSize2; jj<=patchSize2 && !bCollided; jj++)
										if (map_.data[width*(bodyj+jj)+bodyi+ii] >= mapObsThr)		// including unknown region
//										if (map_.data[width*(bodyj+jj)+bodyi+ii] != 0)				// only in clear region
										{
											double distObs = std::sqrt((double)(ii*ii+jj*jj))*resolution;
											if (distObs < bodyLengthCore)
											{
												bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
												bCollided = true;
											}
											else if (distObs < bodyLength)
												pObs[i][j][k] *= std::min( pObs[i][j][k], std::pow( (distObs-bodyLengthCore)/(bodyLength-bodyLengthCore), 2.0) );	// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
										}
							}
							else
							{
// 								bTotMax[i][j][k] = false;
// 								ROS_ERROR("    Patch for obstacle check is out of the map");
							}
						}


// 			bMapReceived_ = 0;
// 			bRobPoseReceived_ = 0;
		}
	}


	// 3) robot arm workspace limit should be included here!

	if (bCheckWork)
	{
// 		std::vector<double> pose = { 0,0,0, 0,0,0 };		// { x,y,z, r,p,y }		// TODO: check if this is wrt robot base frame
// 		std::vector<double> start = { 0,0,0,0,0,0,0 };		// TODO: check home pose of the arm
// 		std::vector<double> solution = { 0,0,0,0,0,0,0 };
// 		int option = 0;
// 		bool ret = false;
// 
// 		for (int i=0; i<nDist; i++)
// 			for (int j=0; j<nAng; j++)
// 				for (int k=0; k<nYaw; k++)
// 					if (bTotMax[i][j][k]==true)
// 					{
// 						// TODO: check for arm home position and z-offset(-0.1?) from robot base to gastank handle
// // 						pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,objY-robY[i][j][k] };
// 						pose = { objx-robx[i][j][k],objy-roby[i][j][k],-0.1, 0.0,0.0,0.0 };
// 
// 						ret = hdt_robot_model_->computeIK(pose,start,solution,option);
// 						if (!ret)
// 							pWork[i][j][k] = 0.0;
// 					}
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
	if (bSortMetric==1 || bSortMetric==2)
	{
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

		ROS_INFO("    cntTotMax: %d",cntTotMax);


		if (cntTotMax==0)
		{
			// TODO: check why the previous candidate_base_poses remain in RViz panel
			// just set to initial robot poase
			robxf = robx0;
			robyf = roby0;
			robYf = robY0;

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

			ROS_WARN("    No candidate pose was found!");
			return false;
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
			ROS_INFO("    cntTotMax: %d",cntTotMax);

			// TODO: comment these part for more candidates
			if (bSortMetric==2)
			{
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
					ROS_INFO("    cntTotMax: %d",cntTotMax);
					if (cntTotMax > 1)
						ROS_WARN("    Multiple candidate poses exist!");

				}
			}	// if (bSortMetric==2)
		}

		// 6-1) generate final desired robot poses with maximum pTot
		ROS_INFO("Reposition Base Command Result:");
		ROS_INFO("    Object Pose (initial): %f %f %f",objx,objy,wrapAngle(objY)*180/M_PI);
		ROS_INFO("    Robot Pose (intial):   %f %f %f", robx0,roby0,wrapAngle(robY0)*180/M_PI);
		// index order regarding to sorting priority
	// 	for (int i=0; i<nDist; i++)
		for (int i=nDist-1; i>=0; i--)
			for (int j=0; j<nAng; j++)
	// 			for (int k=0; k<nYaw; k++)
				for (int k=nYaw-1; k>=0; k--)
					if (bTotMax[i][j][k] == true)
					{
						// /top_shelf pose
						robxf = robx[i][j][k];
						robyf = roby[i][j][k];
						robYf = robY[i][j][k];
						// /base_link pose
						robxf += cos(robYf)*baseOffsetx;
						robyf += sin(robYf)*baseOffsetx;
						ROS_INFO("    Robot Pose (desired):  %f %f %f", robxf,robyf,wrapAngle(robYf)*180/M_PI);


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
					}
	}	// if (bSortMetric==1 || bSortMetric==2)

	else if (bSortMetric==3)
	{
		// candidate prefernece by difference of angular coordinates for current and desired poses
		// HEURISTIC: minimize angle difference between robot orientation (robY) and robot motion direction (angO2Rcur)
		double rob2objY = atan2(objy-roby0,objx-robx0);	// angular coordinate of displacement from robot to object
		double diffYglob;
		int cntTotMax = 0;
		for (int i=0; i<nDist; i++)
			for (int j=0; j<nAng; j++)
				for (int k=0; k<nYaw; k++)
					if (bTotMax[i][j][k] == true)
					{
						diffYglob = wrapAngle( robY[i][j][k]-rob2objY );
// 						pTot[i][j][k] *= std::pow(1 - (scaleDiffYglob * fabs(diffYglob)/M_PI), 2.0);	// quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
						pTot[i][j][k] *= std::max( std::pow(1 - (scaleDiffYglob * fabs(diffYglob)/M_PI), 2.0), pTotThr);	// quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
						cntTotMax++;
					}
		ROS_INFO("Reposition Base Command Result:");
// 		ROS_INFO("    cntTotMax: %d",cntTotMax);
// 		ROS_INFO("    Number of valid candidates: %d",cntTotMax);

		if (cntTotMax==0)
		{
			// TODO: check why the previous candidate_base_poses remain in RViz panel
			// just set to initial robot poase
			robxf = robx0;
			robyf = roby0;
			robYf = robY0;

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

			ROS_WARN("    No valid candidate poses for grasping!");
			return false;
		}

		// sort candidates with respect to pTot
		RepositionBaseCandidate::candidate cand;
		std::vector<RepositionBaseCandidate::candidate> cands;
// 		for (int i=0; i<nDist; i++)
// 			for (int j=0; j<nAng; j++)
// 				for (int k=0; k<nYaw; k++)
		for (int j=0; j<nAng; j++)
			for (int i=nDist-1; i>=0; i--)
				for (int k=nYaw-1; k>=0; k--)
					if (bTotMax[i][j][k]==true)
					{
						cand.i = i;
						cand.j = j;
						cand.k = k;
						cand.pTot = pTot[i][j][k];
						cands.push_back(cand);
					}
		std::sort(cands.begin(), cands.end());


		// 6-2) generate final desired robot poses with maximum pTot
// 		ROS_INFO("Reposition Base Command Result:");
		ROS_INFO("    Object Pose (initial): %f %f %f",objx,objy,wrapAngle(objY)*180/M_PI);
		ROS_INFO("    Robot Pose (intial):   %f %f %f", robx0,roby0,wrapAngle(robY0)*180/M_PI);

		int cntTotThr = 0;	// number of candidate poses with pTot higher than threshold
		for (std::vector<RepositionBaseCandidate::candidate>::iterator m=cands.begin(); m!=cands.end(); ++m)
			if (m->pTot >= pTotThr)
			{
				int i = m->i;
				int j = m->j;
				int k = m->k;

				// /top_shelf pose
				robxf = robx[i][j][k];
				robyf = roby[i][j][k];
				robYf = robY[i][j][k];
				// /base_link pose
				robxf += cos(robYf)*baseOffsetx;
				robyf += sin(robYf)*baseOffsetx;
// 				ROS_INFO("    Robot Pose (desired):  %f %f %f", robxf,robyf,wrapAngle(robYf)*180/M_PI);
				ROS_INFO("    Robot Pose (desired):  %f %f %f  (i:%d j:%d k:%d)    pTot: %f", robxf,robyf,wrapAngle(robYf)*180/M_PI, i,j,k, m->pTot);

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

				//int retIKPLAN = checkIKPLAN(candidate_base_pose);
				//printf("checkIKPLAN (%d,%d,%d): %d\n", i,j,k, retIKPLAN);

				cntTotThr++;
			}
		if (cntTotThr==0)
		{
			robxf = robx0;
			robyf = roby0;
			robYf = robY0;

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

			ROS_WARN("    No probable candidate poses higher than a threshold!");
			return false;
		}
		else
		{
			ROS_INFO("    Number of valid candidates: %d",cntTotMax);
			ROS_INFO("    Number of probable candidates: %d",cntTotThr);
		}

	}

	return true;
}



////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////2ws



int RepositionBaseExecutor::checkIKPLAN()
{
	checkIKPLAN(current_goal_->base_link_in_map);
}

int RepositionBaseExecutor::checkIKPLAN(const geometry_msgs::PoseStamped& candidate_base_pose)
{
            ////////////////////////////////////////////////////////////////////////////////
            // Executed upon entering PLANNING_ARM_MOTION_TO_PREGRASP
            ////////////////////////////////////////////////////////////////////////////////

            if (!generated_grasps_) {
//                 Eigen::Affine3d base_link_to_gas_canister;
//                 tf::poseMsgToEigen(current_goal_->gas_can_in_base_link.pose, base_link_to_gas_canister);
                Eigen::Affine3d base_link_in_map;
                Eigen::Affine3d gas_can_in_map;
                tf::poseMsgToEigen(current_goal_->gas_can_in_map.pose, gas_can_in_map);

//                 tf::poseMsgToEigen(current_goal_->base_link_in_map.pose, base_link_in_map);
                tf::poseMsgToEigen(candidate_base_pose.pose, base_link_in_map);
				Eigen::Affine3d base_link_to_gas_canister = base_link_in_map.inverse() * gas_can_in_map;
                
				// 1. generate grasp candidates (poses of the wrist in the robot frame) from the object pose
                int max_num_candidates = 100;
                std::vector<GraspCandidate> grasp_candidates = sample_grasp_candidates(base_link_to_gas_canister, max_num_candidates);
                ROS_INFO("Sampled %zd grasp poses", grasp_candidates.size());

                visualize_grasp_candidates(grasp_candidates);

                // mount -> wrist = mount -> robot * robot -> wrist

//                 const std::string robot_frame = current_goal_->gas_can_in_base_link.header.frame_id;
                const std::string robot_frame = "/base_footprint";
                const std::string kinematics_frame = "arm_mount_panel_dummy";
                tf::StampedTransform tf_transform;
                try {
                    listener_.lookupTransform(robot_frame, kinematics_frame, ros::Time(0), tf_transform);
                }
                catch (const tf::TransformException& ex) {
//                     hdt_msgs::GraspObjectCommandResult result;
//                     result.result = hdt_msgs::GraspObjectCommandResult::PLANNING_FAILED;
                    std::stringstream ss;
                    ss << "Failed to lookup transform " << robot_frame << " -> " << kinematics_frame << "; Unable to determine grasp reachability";
                    ROS_WARN("%s", ss.str().c_str());
//                     as_->setAborted(result, ss.str());
//                     status_ = GraspObjectExecutionStatus::FAULT;
//                     break;
					return -4;
                }

                Eigen::Affine3d robot_to_kinematics;
                msg_utils::convert(tf_transform, robot_to_kinematics);

                // 2. filter unreachable grasp candidates
                reachable_grasp_candidates_.clear();
                reachable_grasp_candidates_.reserve(grasp_candidates.size());
                for (const GraspCandidate& grasp_candidate : grasp_candidates) {
                    Eigen::Affine3d kinematics_to_grasp_candidate =
                            robot_to_kinematics.inverse() * grasp_candidate.grasp_candidate_transform;

                    std::vector<double> fake_seed(robot_model_->joint_names().size(), 0.0);
                    std::vector<double> sol;
                    if (robot_model_->search_nearest_ik(
                            kinematics_to_grasp_candidate, fake_seed, sol, sbpl::utils::ToRadians(1.0)))
                    {
                        GraspCandidate reachable_grasp_candidate(kinematics_to_grasp_candidate, grasp_candidate.u);
                        reachable_grasp_candidates_.push_back(reachable_grasp_candidate);
                    }
                }

                ROS_INFO("Produced %zd reachable grasp poses", reachable_grasp_candidates_.size());

                if (reachable_grasp_candidates_.empty()) {
                    ROS_WARN("No reachable grasp candidates available");
//                     hdt_msgs::GraspObjectCommandResult result;
//                     result.result = hdt_msgs::GraspObjectCommandResult::OBJECT_OUT_OF_REACH;
//                     as_->setAborted(result, "No reachable grasp candidates available");
//                     status_ = GraspObjectExecutionStatus::FAULT;
//                     break;
					return -3;
                }

                const double min_u = 0.0;
                const double max_u = 1.0;

                // 3. sort grasp candidates by desirability (note: more desirable grasps are at the end of the vector)
                std::sort(reachable_grasp_candidates_.begin(), reachable_grasp_candidates_.end(),
                          [&](const GraspCandidate& a, const GraspCandidate& b) -> bool
                          {
                              double mid_u = 0.5 * (min_u + max_u);
                              return fabs(a.u - mid_u) > fabs(b.u - mid_u);
                          });

                // limit the number of grasp attempts by the configured amount
                std::reverse(reachable_grasp_candidates_.begin(), reachable_grasp_candidates_.end()); // lol
                while (reachable_grasp_candidates_.size() > max_grasp_candidates_) {
                    reachable_grasp_candidates_.pop_back();
                }
                std::reverse(reachable_grasp_candidates_.begin(), reachable_grasp_candidates_.end()); // lol

                ROS_INFO("Attempting %zd grasps", reachable_grasp_candidates_.size());

                generated_grasps_ = true;
            }

            ////////////////////////////////////////////////////////////////////////////////
            // Main loop of PLANNING_ARM_MOTION_TO_PREGRASP
            ////////////////////////////////////////////////////////////////////////////////

            if (!sent_move_arm_goal_) {
//                 hdt_msgs::GraspObjectCommandFeedback feedback;
//                 feedback.status = execution_status_to_feedback_status(status_);
//                 as_->publishFeedback(feedback);

                if (reachable_grasp_candidates_.empty()) {
                    ROS_WARN("Failed to plan to all reachable grasps");
//                     hdt_msgs::GraspObjectCommandResult result;
//                     result.result = hdt_msgs::GraspObjectCommandResult::PLANNING_FAILED;
//                     as_->setAborted(result, "Failed to plan to all reachable grasps");
//                     status_ = GraspObjectExecutionStatus::FAULT;
//                     break;
					return -2;
                }

                ROS_WARN("Sending Move Arm Goal to pregrasp pose");
                if (!wait_for_action_server(
                        move_arm_command_client_,
                        move_arm_command_action_name_,
                        ros::Duration(0.1),
                        ros::Duration(5.0)))
                {
                    std::stringstream ss; ss << "Failed to connect to '" << move_arm_command_action_name_ << "' action server";
                    ROS_WARN("%s", ss.str().c_str());
//                     hdt_msgs::GraspObjectCommandResult result;
//                     result.result = hdt_msgs::GraspObjectCommandResult::PLANNING_FAILED;
//                     as_->setAborted(result, ss.str());
//                     status_ = GraspObjectExecutionStatus::FAULT;
//                     break;
					return -1;
                }

                const GraspCandidate& next_best_grasp = reachable_grasp_candidates_.back();

                // 4. send a move arm goal for the best grasp
                last_move_arm_pregrasp_goal_.type = hdt::MoveArmCommandGoal::EndEffectorGoal;
                tf::poseEigenToMsg(next_best_grasp.grasp_candidate_transform, last_move_arm_pregrasp_goal_.goal_pose);

				// TODO
//                 last_move_arm_pregrasp_goal_.octomap = current_goal_->octomap;

                auto result_cb = boost::bind(&RepositionBaseExecutor::move_arm_command_result_cb, this, _1, _2);
                move_arm_command_client_->sendGoal(last_move_arm_pregrasp_goal_, result_cb);

                pending_move_arm_command_ = true;
                sent_move_arm_goal_ = true;

                reachable_grasp_candidates_.pop_back();
            }
            else if (!pending_move_arm_command_) {
                // NOTE: short-circuiting "EXECUTING_ARM_MOTION_TO_PREGRASP" for
                // now since the move_arm action handles execution and there is
                // presently no feedback to distinguish planning vs. execution

                ROS_INFO("Move Arm Goal is no longer pending");
                if (move_arm_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    move_arm_command_result_ && move_arm_command_result_->success)
                {
                    ROS_INFO("Move Arm Command succeeded");
//                     status_ = GraspObjectExecutionStatus::OPENING_GRIPPER;
                }
                else {
                    ROS_INFO("Move Arm Command failed");
                    ROS_INFO("    Simple Client Goal State: %s", move_arm_command_goal_state_.toString().c_str());
                    ROS_INFO("    Error Text: %s", move_arm_command_goal_state_.getText().c_str());
                    ROS_INFO("    result.success = %s", move_arm_command_result_ ? (move_arm_command_result_->success ? "TRUE" : "FALSE") : "null");
                    // stay in PLANNING_ARM_MOTION_TO_PREGRASP until there are no more grasps
                    // TODO: consider moving back to the stow position
					return -0;
                }

                sent_move_arm_goal_ = false; // reset for future move arm goals
            }

	return 1;	// success
}

std::vector<RepositionBaseExecutor::GraspCandidate>
RepositionBaseExecutor::sample_grasp_candidates(const Eigen::Affine3d& robot_to_object, int num_candidates) const
{
	const double min_u = 0.0;
	const double max_u = 1.0;

    std::vector<GraspCandidate> grasp_candidates;
    grasp_candidates.reserve(num_candidates);
    for (int i = 0; i < num_candidates; ++i) {
//         ROS_INFO("Candidate Pregrasp %3d", i);
        // sample uniformly the position and derivative of the gas canister grasp spline
        double u = (max_u - min_u) * i / (num_candidates - 1);

        int knot_num = -1;
        for (int j = 0; j < grasp_spline_->knots().size() - 1; ++j) {
            double curr_knot = grasp_spline_->knot(j);
            double next_knot = grasp_spline_->knot(j + 1);
            if (u >= curr_knot && u < next_knot) {
                knot_num = j;
                break;
            }
        }

        if (knot_num < grasp_spline_->degree() ||
            knot_num >= grasp_spline_->knots().size() - grasp_spline_->degree())
        {
            ROS_INFO("Skipping grasp_spline(%0.3f) [point governed by same knot more than once]", u);
            continue;
        }

        Eigen::Vector3d object_pos_robot_frame(robot_to_object.translation());

        Eigen::Vector3d sample_spline_point = (*grasp_spline_)(u);
        ROS_INFO("    Sample Spline Point [canister frame]: %s", to_string(sample_spline_point).c_str());
        Eigen::Vector3d sample_spline_deriv = grasp_spline_->deriv(u);
        ROS_INFO("    Sample Spline Deriv [canister frame]: %s", to_string(sample_spline_deriv).c_str());

        Eigen::Affine3d mark_to_menglong(Eigen::Affine3d::Identity());
        Eigen::Vector3d sample_spline_point_robot_frame =
                robot_to_object * mark_to_menglong * Eigen::Scaling(gas_can_scale_) * sample_spline_point;
        ROS_INFO("    Sample Spline Point [robot frame]: %s", to_string(sample_spline_point_robot_frame).c_str());

        Eigen::Vector3d sample_spline_deriv_robot_frame =
                robot_to_object.rotation() * sample_spline_deriv.normalized();
        ROS_INFO("    Sample Spline Deriv [robot frame]: %s", to_string(sample_spline_deriv_robot_frame).c_str());

        // compute the normal to the grasp spline that most points "up" in the robot frame
        Eigen::Vector3d up_bias(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d up_grasp_dir =
            up_bias - up_bias.dot(sample_spline_deriv_robot_frame) * sample_spline_deriv_robot_frame;
        up_grasp_dir.normalize();
        up_grasp_dir *= -1.0;

        Eigen::Vector3d down_bias(-Eigen::Vector3d::UnitZ());
        Eigen::Vector3d down_grasp_dir =
            down_bias - down_bias.dot(sample_spline_deriv_robot_frame) * sample_spline_deriv_robot_frame;
        down_grasp_dir.normalize();
        down_grasp_dir *= -1.0;

        Eigen::Vector3d grasp_dir;
        if (up_grasp_dir.dot(sample_spline_point_robot_frame - object_pos_robot_frame) < 0) {
            grasp_dir = up_grasp_dir;
        }
        else {
            ROS_INFO("Skipping grasp_spline(%0.3f) [derivative goes backwards along the spline]", u);
            continue;
//            grasp_dir = down_grasp_dir;
        }

        ROS_INFO("    Grasp Direction [robot frame]: %s", to_string(grasp_dir).c_str());

        Eigen::Vector3d grasp_candidate_dir_x = grasp_dir;
        Eigen::Vector3d grasp_candidate_dir_y = sample_spline_deriv_robot_frame;
        Eigen::Vector3d grasp_candidate_dir_z = grasp_dir.cross(sample_spline_deriv_robot_frame);

        Eigen::Matrix3d grasp_rotation_matrix;
        grasp_rotation_matrix(0, 0) = grasp_candidate_dir_x.x();
        grasp_rotation_matrix(1, 0) = grasp_candidate_dir_x.y();
        grasp_rotation_matrix(2, 0) = grasp_candidate_dir_x.z();
        grasp_rotation_matrix(0, 1) = grasp_candidate_dir_y.x();
        grasp_rotation_matrix(1, 1) = grasp_candidate_dir_y.y();
        grasp_rotation_matrix(2, 1) = grasp_candidate_dir_y.z();
        grasp_rotation_matrix(0, 2) = grasp_candidate_dir_z.x();
        grasp_rotation_matrix(1, 2) = grasp_candidate_dir_z.y();
        grasp_rotation_matrix(2, 2) = grasp_candidate_dir_z.z();

        // robot_frame -> candidate tool frame pose
        Eigen::Affine3d grasp_candidate_rotation =
                Eigen::Translation3d(sample_spline_point_robot_frame) * grasp_rotation_matrix;

        // robot -> grasp candidate (desired tool) * tool -> wrist * wrist (grasp) -> pregrasp = robot -> wrist
        Eigen::Affine3d candidate_wrist_transform = grasp_candidate_rotation * wrist_to_tool_.inverse() * grasp_to_pregrasp_;

        ROS_INFO("    Pregrasp Pose [robot frame]: %s", to_string(candidate_wrist_transform).c_str());

        grasp_candidates.push_back(GraspCandidate(candidate_wrist_transform, u));
    }

    std::size_t original_size = grasp_candidates.size();
    for (int i = 0; i < original_size; ++i) {
        const GraspCandidate& grasp = grasp_candidates[i];
        Eigen::Affine3d flipped_candidate_transform =
                grasp.grasp_candidate_transform * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        grasp_candidates.push_back(GraspCandidate(flipped_candidate_transform, grasp.u));
    }
    return grasp_candidates;
}

void RepositionBaseExecutor::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::MoveArmCommandResult::ConstPtr& result)
{
    move_arm_command_goal_state_ = state;
    move_arm_command_result_ = result;
    pending_move_arm_command_ = false;
}

void RepositionBaseExecutor::visualize_grasp_candidates(const std::vector<GraspCandidate>& grasps) const
{
    ROS_INFO("Visualizing %zd grasps", grasps.size());
    geometry_msgs::Vector3 triad_scale = geometry_msgs::CreateVector3(0.1, 0.01, 0.01);

    visualization_msgs::MarkerArray all_triad_markers;

    // create triad markers to be reused for each grasp candidate
    visualization_msgs::MarkerArray triad_markers = msg_utils::create_triad_marker_arr(triad_scale);
    for (visualization_msgs::Marker& marker : triad_markers.markers) {
//         marker.header.frame_id = current_goal_->gas_can_in_base_link.header.frame_id;
        marker.header.frame_id = "/base_footprint";
        marker.ns = "candidate_grasp";
    }

    visualization_msgs::MarkerArray single_triad_markers;

    // save the original marker transforms
    std::vector<Eigen::Affine3d> marker_transforms;
    marker_transforms.reserve(triad_markers.markers.size());
    for (const auto& marker : triad_markers.markers) {
        Eigen::Affine3d marker_transform;
        tf::poseMsgToEigen(marker.pose, marker_transform);
        marker_transforms.push_back(marker_transform);
    }

    int id = 0;
    for (const GraspCandidate& candidate : grasps) {
        // transform triad markers to the candidate grasp transform
        for (visualization_msgs::Marker& marker : triad_markers.markers) {
            marker.id = id++;
            Eigen::Affine3d marker_transform;
            tf::poseMsgToEigen(marker.pose, marker_transform);
            Eigen::Affine3d new_marker_transform = candidate.grasp_candidate_transform * marker_transform;
            tf::poseEigenToMsg(new_marker_transform, marker.pose);
        }

        // add triad markers for this grasp to the total marker set
        all_triad_markers.markers.insert(all_triad_markers.markers.end(), triad_markers.markers.begin(), triad_markers.markers.end());

        // publish the triad for this grasp by itself to a separate namespace, but keep the same id scheme so that older grasps are not overwritten
        single_triad_markers.markers = triad_markers.markers;
        assert(single_triad_markers.markers.size() == 4);
        ROS_INFO("Publishing marker for grasp %d", single_triad_markers.markers.front().id >> 2); // each marker set should have 3 markers for the axes and 1 marker for the origin
        for (visualization_msgs::Marker& marker : single_triad_markers.markers) {
            marker.ns = "solo_candidate_grasp";
        }
        marker_arr_pub_.publish(single_triad_markers);
        single_triad_markers.markers.clear();

        // restore marker transforms to their original frame
        for (std::size_t i = 0; i < triad_markers.markers.size(); ++i) {
            tf::poseEigenToMsg(marker_transforms[i], triad_markers.markers[i].pose);
        }
    }

    ROS_INFO("Visualizing %zd triad markers", all_triad_markers.markers.size());
    marker_arr_pub_.publish(all_triad_markers);
}

