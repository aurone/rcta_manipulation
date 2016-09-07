#include "RepositionBaseExecutor.h"

// standard includes
#include <algorithm>

// systemm includes
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl_geometry_utils/utils.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/RunUponDestruction.h>
#include <tf_conversions/tf_eigen.h>

namespace RepositionBaseExecutionStatus {
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
} // namespace RepositionBaseExecutionStatus

double sign(double val)
{
    return (val >= 0) ? 1.0 : -1.0;
}

RepositionBaseExecutor::RepositionBaseExecutor() :
    nh_(),
    ph_("~"),
    action_name_("reposition_base_command"),
    viz_pub_(),
    listener_(),
    as_(),
    robot_model_(),
    manip_group_(nullptr),
    manip_name_(),
    camera_view_frame_(),
    cc_(),
    attached_markers_(),
    gas_can_mesh_path_(),
    gas_can_scale_(),
    max_grasp_candidates_(0),
    pregrasp_to_grasp_offset_m_(0.0),
    wrist_to_tool_(Eigen::Affine3d::Identity()),
    grasp_to_pregrasp_(Eigen::Affine3d::Identity()),
    grasp_spline_(),
    move_arm_command_client_(),
    move_arm_command_action_name_(),
    move_arm_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    move_arm_command_result_(),
    map_(),
    current_goal_(),
    status_(RepositionBaseExecutionStatus::INVALID),
    last_status_(RepositionBaseExecutionStatus::INVALID)
{
}

bool RepositionBaseExecutor::initialize()
{
    robot_frame_ = "base_footprint";
    camera_view_frame_ = "camera_rgb_optical_frame";

    rml_.reset(new robot_model_loader::RobotModelLoader);
    robot_model_ = rml_->getModel();
    if (!robot_model_) {
        ROS_ERROR("Failed to load robot model");
        return false;
    }

    robot_state_.reset(new moveit::core::RobotState(robot_model_));

    if (!msg_utils::download_param(ph_, "manipulator_group_name", manip_name_)) {
        return false;
    }

    if (!robot_model_->hasJointModelGroup(manip_name_)) {
        ROS_ERROR("robot '%s' has no group named '%s'", robot_model_->getName().c_str(), manip_name_.c_str());
        return false;
    }

    if (!msg_utils::download_param(ph_, "gas_canister_mesh", gas_can_mesh_path_) ||
        !msg_utils::download_param(ph_, "gas_canister_mesh_scale", gas_can_scale_))
    {
        ROS_ERROR("Failed to download gas canister parameters");
        return false;
    }

    if (!downloadGraspingParameters(ph_)) {
        ROS_ERROR("Failed to retrieve parameters for grasp database");
        return false;
    }

    viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 5);

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

    as_->registerGoalCallback(boost::bind(&RepositionBaseExecutor::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&RepositionBaseExecutor::preempt_callback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    if (!download_marker_params()) {
        ROS_WARN("Failed to download marker params");
        return false;
    }

    std::vector<geometry_msgs::Point> footprint;
    if (!msg_utils::download_param(ph_, "footprint", footprint)) {
        return false;
    }

    // hard-coded robot perimeter here
    int obs_thresh = 50;
    int num_heading_disc = (int)(360.0 / 5.0); //5 degree discretization

    std::vector<sbpl_2Dpt_t> footprint_polygon;
    footprint_polygon.resize(footprint.size());
    ROS_INFO("Footprint:");
    for (size_t i = 0; i < footprint.size(); ++i) {
        ROS_INFO("  (%0.3f, %0.3f)", footprint[i].x, footprint[i].y);
        footprint_polygon[i].x = footprint[i].x;
        footprint_polygon[i].y = footprint[i].y;
    }

    cc_.reset(new XYThetaCollisionChecker(footprint_polygon, obs_thresh, num_heading_disc));

    return true;
}

bool RepositionBaseExecutor::downloadGraspingParameters(ros::NodeHandle& nh)
{
    // read in max grasps
    if (!msg_utils::download_param(ph_, "max_grasp_candidates", max_grasp_candidates_) || max_grasp_candidates_ < 0) {
        ROS_ERROR("Failed to retrieve 'max_grasp_candidates' from the param server or 'max_grasp_candidates' is negative");
        return false;
    }

    std::vector<geometry_msgs::Point> control_points;
    int degree;
    if (!msg_utils::download_param(ph_, "degree", degree) || !msg_utils::download_param(ph_, "control_points", control_points)) {
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

    return true;
}

int RepositionBaseExecutor::run()
{
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

    status_ = RepositionBaseExecutionStatus::IDLE;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });
//        ROS_INFO("Spinning (%s)...", to_string(status_).c_str());

        ros::spinOnce();

        if (status_ != last_status_) {
            ROS_INFO("Reposition Base Executor Transitioning: %s -> %s", to_string(last_status_).c_str(), to_string(status_).c_str());
            last_status_ = status_;
        }

        assert((bool)as_);

        switch (status_) {
        case RepositionBaseExecutionStatus::IDLE: {
            if (as_->isActive()) {
                status_ = RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE;
            }
        }   break;
        case RepositionBaseExecutionStatus::FAULT: {
            if (as_->isActive()) {
                status_ = RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE;
            }
        }   break;
        case RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE: {
            Eigen::Affine3d robot_pose;
            tf::poseMsgToEigen(robot_pose_world_frame_.pose, robot_pose);

            Eigen::Affine3d object_pose;
            tf::poseMsgToEigen(current_goal_->gas_can_in_map.pose, object_pose);

            std::vector<geometry_msgs::PoseStamped> candidate_base_poses;

            if (tryFeasibleArmCheck(robot_pose, object_pose)) {
                int err = checkFeasibleMoveToPregraspTrajectory(robot_pose, object_pose);
                ROS_DEBUG("arm plan result: %d", err);
                if (err) {
                    int v_id = 0;
                    visualizeRobot(robot_pose, 0, "base_checkIKPLAN_fail", v_id);
                } else {
                    // you can grasp it now!
                    rcta_msgs::RepositionBaseCommandResult result;
                    result.result = rcta_msgs::RepositionBaseCommandResult::SUCCESS;

                    geometry_msgs::PoseStamped p;
                    tf::poseEigenToMsg(robot_pose, p.pose);
                    candidate_base_poses.push_back(p);
                    result.candidate_base_poses = candidate_base_poses;
                    as_->setSucceeded(result);
                    status_ = RepositionBaseExecutionStatus::IDLE;
                }
            } else if (computeRobPose(robot_pose, object_pose, candidate_base_poses)) {
                rcta_msgs::RepositionBaseCommandResult result;
                result.result = rcta_msgs::RepositionBaseCommandResult::SUCCESS;
                result.candidate_base_poses = candidate_base_poses;
                as_->setSucceeded(result);
                status_ = RepositionBaseExecutionStatus::IDLE;
            } else if (computeRobPoseExhaustive(robot_pose, object_pose, candidate_base_poses)) {
                rcta_msgs::RepositionBaseCommandResult result;
                result.result = rcta_msgs::RepositionBaseCommandResult::SUCCESS;
                result.candidate_base_poses = candidate_base_poses;
                as_->setSucceeded(result);
                status_ = RepositionBaseExecutionStatus::IDLE;
            } else {
                rcta_msgs::RepositionBaseCommandResult result;
                result.result = rcta_msgs::RepositionBaseCommandResult::PLANNING_FAILURE_OBJECT_UNREACHABLE;
                result.candidate_base_poses = candidate_base_poses;	// this pose is same with the initial base pose
                as_->setSucceeded(result);
                status_ = RepositionBaseExecutionStatus::IDLE;
            }
        }   break;
        case RepositionBaseExecutionStatus::COMPLETING_GOAL: {
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
    ROS_INFO("    Gas Can Pose: %s", to_string(current_goal_->gas_can_in_map).c_str());
    ROS_INFO("    Base Link Pose: %s", to_string(current_goal_->base_link_in_map).c_str());
    ROS_INFO("    Map Header: %s", to_string(current_goal_->map.header).c_str());

    // NOTE: There was some fishy business going on here transforming the
    // incoming pose to a pose specified in another frame, presumably for
    // compatibility. Removing for now, but keep this in mind...it shouldn't
    // be necessary, strictly speaking
//    const std::string world_frame = "abs_nwu";

    const std::string& map_frame = current_goal_->map.header.frame_id;
    if (current_goal_->base_link_in_map.header.frame_id != map_frame) {
        try {
            listener_.transformPose(
                    map_frame,
                    current_goal_->base_link_in_map,
                    robot_pose_world_frame_);
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("Failed to transform from '%s' to '%s' (%s)", current_goal_->base_link_in_map.header.frame_id.c_str(), map_frame.c_str(), ex.what());
            as_->setAborted();
            return;
        }
    }
    else {
        robot_pose_world_frame_ = current_goal_->base_link_in_map;
    }

    // update collision checker if valid map
    if (current_goal_->map.data.size() > 0) {
        // seems like valid map
        cc_->UpdateOccupancyGrid(current_goal_->map);
    }
    else {
        ROS_WARN("No valid map received with goal!");
    }
}

void RepositionBaseExecutor::preempt_callback()
{
}

uint8_t RepositionBaseExecutor::execution_status_to_feedback_status(
    RepositionBaseExecutionStatus::Status status)
{
    switch (status) {
    case RepositionBaseExecutionStatus::IDLE:
        return -1;
    case RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE:
        return (float)0.0; // TODO: feedback of planning time in [s]
    default:
        return -1;
    }
}

bool RepositionBaseExecutor::computeRobPose(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose,
    std::vector<geometry_msgs::PoseStamped>& candidate_base_poses)
{
    double objx, objy, objY, robx0, roby0, robY0;
    Eigen::Affine2d robot_pose_planar;
    computeRobotPlanarPose(robot_pose, robot_pose_planar);
    Eigen::Affine2d object_pose_planar;
    computeObjectPlanarPose(object_pose, object_pose_planar);
    robx0 = robot_pose_planar.translation()[0];
    roby0 = robot_pose_planar.translation()[1];
    robY0 = Eigen::Rotation2Dd(0.0).fromRotationMatrix(robot_pose_planar.rotation()).angle();
    objx = object_pose_planar.translation()[0];
    objy = object_pose_planar.translation()[1];
    objY = Eigen::Rotation2Dd(0.0).fromRotationMatrix(object_pose_planar.rotation()).angle();

    //visualizations
    int base_candidates_viz_id = 0;
    int base_probcandidates_viz_id = 0;
    int base_collision_viz_id = 0;
    int base_probreject_viz_id = 0;
    int base_validityreject_viz_id = 0;
    int base_failedik_viz_id = 0;
//    viz.setReferenceFrame("/abs_nwu");

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
    int nDist = 6;// r in [0.5:0.1:1.0] + camera offset 		// TODO: define adequate sample range (frame_id: /top_shelf)
    double distMin = 0.5, distStep = 0.1;
// 	int nDist = 6;		// r in [0.6:0.1:1.1] + camera offset 		// TODO: define adequate sample range (frame_id: /top_shelf)
// 	double distMin = 0.6, distStep = 0.1;
// 	int nAng = 12;		// th in [0:30:330]
// 	double angMin = 0.0, angStep = 30.0/180.0*M_PI;
    int nAng = 24;		// th in [0:15:330]
    double angMin = 0.0, angStep = 15.0 / 180.0 * M_PI;
    int nYaw = 9;		// Y in [-20:5:20]
    double yawMin = -20.0 / 180.0 * M_PI, yawStep = 5.0 / 180.0 * M_PI;
// 	int nYaw = 5;		// Y in [-20:5:20]
// 	double yawMin = -10.0/180.0*M_PI, yawStep = 5.0/180.0*M_PI;

    // 1) grasp achievable zone 	// TODO: tune these parameters
    int secDist[] = { (int)(nDist / 3), (int)(nDist * 2 / 3) };	// divide zone by distance (for acceptable object orientation setting)
    double secAngYaw[2];// accept object orientations between these two values 	// seperately defined for different regions by secDist (values within the detail code of 1))
    double bestAngYaw;// best object orientation to get highest pGrasp probability (set as the mean value of secAngYaw[0~1] currently)
    double bestDist = 0.70;	// best object distance to get highest pGrasp probability (set as the nominal value from experiment)
//	double secSide[2] = {0.0, M_PI/4.0};    		// divide left and right-hand side  // [rad]
    double secSide[2] = { -05.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI };		// divide left and right-hand side  // [rad]
// 	double secSide[2] = {0.0, 20.0/180.0*M_PI};		// divide left and right-hand side  // [rad]
// 	double scalepGraspAngYaw = 0.1;	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
// 	double scalepGraspDist = 0.1;	// pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)
    double scalepGraspAngYaw = 0.05;// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
    double scalepGraspDist = 0.05;	// pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)

    // 2) arm position offset for computing pObs
// 	double armOffsety = -0.0;
// 	double armOffsety = -0.5;
// 	double armOffsety = -1.1;	// center of arm (when straightend)
// 	double armLength = 1.5;		// workspace radius about the center of arm
// 	double armLengthCore = 0.6;	// core workspace radius about the center of arm (pObs = 0.0)
    double armOffsetx = 0.3;	// center of arm (for normal grasping motion)
    double armOffsety = -0.5;	// center of arm (for normal grasping motion)
    double armLength = 0.5;		// workspace radius about the center of arm
    double armLengthCore = 0.3;	// core workspace radius about the center of arm (pObs = 0.0)
// 	double bodyOffsetx = -0.49+0.148975;	// center of body (except arm)
// 	double bodyLength = 0.8;	// support polygon radius about the center of body
// 	double bodyLengthCore = 0.65;	// core support polygon radius about the center of body (pObs = 0.0)
    double bodyOffsetx1 = -0.49 + 0.148975 + 0.27;	// center of front body (except arm)
    double bodyOffsetx2 = -0.49 + 0.148975 - 0.27;	// center of rear body (except arm)
    double bodyLength = 0.5;	// support polygon radius about the center of body
    double bodyLengthCore = 0.335;	// core support polygon radius about the center of body (pObs = 0.0)
    int mapObsThr = 1;			// threshold for classifying clear and occupied regions

    // 5) candidate selection criterion
    double scaleDiffYglob = 0.05;// multiply a quadratic function to pTot (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI) for bSortMetric==3
// 	double pTotThr = 0.5;		// pTot threshold for bSortMetric==3
    double pTotThr = 0.0;		// pTot threshold for bSortMetric==3

    // 6) /base_link offset from /top_shelf for final robot pose return
// 	double baseOffsetx = -0.0;
// 	double baseOffsetx = -0.3;
// 	double baseOffsetx = -0.5;
    double baseOffsetx = -0.49 + 0.148975;	// /base_link to /base_link_front_bumper_part to /top_shelf
//	double baseOffsetx = -0.49;				// /base_link to /base_link_front_bumper_part (in new Hokuyo setting) 	// same as in computeRobPose()

////////////////////////////////////////////////////////////////////////////////////

    // 0) set search space (r,th,Y)
    // (r,th): polar coordinate with respect to the object with the center fixed at the origin and the nozzle aligned to 0 rad
    // (Y): orientation about z-axis

    double pTot[nDist][nAng][nYaw], pGrasp[nDist][nAng][nYaw], pObs[nDist][nAng][nYaw], pWork[nDist][nAng][nYaw];
    bool bTotMax[nDist][nAng][nYaw];
    for (int i = 0; i < nDist; i++)
        for (int j = 0; j < nAng; j++)
            for (int k = 0; k < nYaw; k++) {
                pTot[i][j][k] = 0.0;
                pGrasp[i][j][k] = 1.0;
                pObs[i][j][k] = 1.0;
                pWork[i][j][k] = 1.0;
                bTotMax[i][j][k] = false;	// used when 1) check candidate validity before IK (process 1~4)
                                            //			 2) select the one with maximum pTot (process 5~6)
            }

    double robz0 = 0.0, robP0 = 0.0, robR0 = 0.0;
    double robxf = robx0, robyf = roby0, robzf = robz0, robYf = robY0, robPf = robP0, robRf = robR0; // final (computed) robot pose

    double robx[nDist][nAng][nYaw], roby[nDist][nAng][nYaw], robY[nDist][nAng][nYaw];	// candidate robot poses
    for (int i = 0; i < nDist; i++)
        for (int j = 0; j < nAng; j++)
            for (int k = 0; k < nYaw; k++) {
                robx[i][j][k] = objx + cos(objY + angMin + angStep * j) * (distMin + distStep * i);
                roby[i][j][k] = objy + sin(objY + angMin + angStep * j) * (distMin + distStep * i);
                robY[i][j][k] = objY + (angMin + angStep * j) - M_PI - (yawMin + yawStep * k);// angular coordinate of x-axis (frontal direction) of the robot with respect to global frame
            }

    // 1) probability of successful grasping

    if (bCheckGrasp) {
        // heuristic probability of successful grasping
        // a) position: the object should be within left-hand side of the robot and 45 deg angle of view
        // exception: exclude poses at close distance, include ones in right-hand side at far distance
        // b) orientation: the object handle should be directed to the right of the robot
        // c) exception: allow more deviation at close distance, reject most of deviations at far distance

        for (int i = 0; i < nDist; i++) {
            // c) set acceptable object orientation range
            if (i < secDist[0]) {
// 				secAngYaw[0] = 15.0/180.0*M_PI;
// 				secAngYaw[1] = 100.0/180.0*M_PI;
// 				bestAngYaw   = 65.0/180.0*M_PI;
                secAngYaw[0] = 45.0 / 180.0 * M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[1] = 130.0 / 180.0 * M_PI;
                bestAngYaw = 60.0 / 180.0 * M_PI;
            }
            else if (i < secDist[1]) {
// 				secAngYaw[0] = 15.0/180.0*M_PI;
// 				secAngYaw[0] = 45.0/180.0*M_PI;
// 				bestAngYaw   = 45.0/180.0*M_PI;
                secAngYaw[0] = 45.0 / 180.0 * M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[1] = 130.0 / 180.0 * M_PI;
                bestAngYaw = 60.0 / 180.0 * M_PI;
            }
            else // if (i >= secDist[1])
            {
// 				secAngYaw[0] = 15.0/180.0*M_PI;
// 				secAngYaw[1] = 30.0/180.0*M_PI;
// 				bestAngYaw   = 15.0/180.0*M_PI;
// 				bestAngYaw   = 50.0/180.0*M_PI;
                secAngYaw[0] = 45.0 / 180.0 * M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[1] = 130.0 / 180.0 * M_PI;
                bestAngYaw = 60.0 / 180.0 * M_PI;
            }

            for (int j = 0; j < nAng; j++) {
                for (int k = 0; k < nYaw; k++) {
                    // a) object position at left-hand side to the robot
                    double rob2obj = atan2(objy - roby[i][j][k], objx - robx[i][j][k]);	// angular coordinate of a vector from robot position to object position (not orientation)
                    double diffAng = angles::normalize_angle(rob2obj - robY[i][j][k]);
                    if (diffAng >= secSide[0] && diffAng < secSide[1])    // left-hand side and angle of view
                            {
                        // b) object handle orientation to the right of the robot
                        double diffY = angles::normalize_angle(objY - robY[i][j][k]);
                        double diffYMax = std::max(fabs(secAngYaw[0] - bestAngYaw), fabs(secAngYaw[1] - bestAngYaw));
                        double diffDistMax = std::max(fabs(distMin - bestDist),
                                fabs(distMin + distStep * (nDist - 1) - bestDist));
                        if (diffY >= secAngYaw[0] && diffY <= secAngYaw[1]) {
                            bTotMax[i][j][k] = true; // using bTotMax instead of pGrasp to explicitly represent candidate validity (before expensive IK test)

                            // higher probability around diffY==bestAngYaw
// 							pGrasp[i][j][k] = std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 );	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
// 							pGrasp[i][j][k] = std::max( std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 ), pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)

                            // higher probability around diffY==bestAngYaw and diffDist==bestDist
                            pGrasp[i][j][k] = std::max(
                                    std::pow((diffYMax - fabs(diffY - bestAngYaw) * scalepGraspAngYaw) / (diffYMax),
                                            2.0)
                                            * std::pow(
                                                    (diffDistMax
                                                            - fabs(distMin + distStep * i - bestDist) * scalepGraspDist)
                                                            / (diffDistMax), 2.0), pTotThr); // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                        }
                    }
                }
            }
        }
    }

    // 2) probability not to collide with obstacles
    // depends on distance to obstacles considering the position offset wrt orientation

    if (bCheckObs == true) {
        // TODO TODO
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

            int patchSize2 = (int)(armLength / resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
            double armx, army, armY;	// arm center position biased from /top_shelf by armOffsety
            int armi, armj;				// index for patch center
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
// 							armx = robx[i][j][k] - sin(robY[i][j][k])*armOffsety;
// 							army = roby[i][j][k] + cos(robY[i][j][k])*armOffsety;
                            armx = robx[i][j][k] + cos(robY[i][j][k]) * armOffsetx - sin(robY[i][j][k]) * armOffsety;
                            army = roby[i][j][k] + sin(robY[i][j][k]) * armOffsetx + cos(robY[i][j][k]) * armOffsety;

                            armi = (int)((armx - origin.position.x) / resolution);
                            armj = (int)((army - origin.position.y) / resolution);

                            if (armi >= patchSize2 && armi < width - patchSize2 && armj >= patchSize2
                                    && armj < height - patchSize2) {
                                bool bCollided = false;
                                for (int ii = -patchSize2; ii <= patchSize2 && !bCollided; ii++)
                                    for (int jj = -patchSize2; jj <= patchSize2 && !bCollided; jj++)
                                        if (map_.data[width * (armj + jj) + armi + ii] >= mapObsThr)// including unknown region
//										if (map_.data[width*(armj+jj)+armi+ii] != 0)				// only in clear region
                                                {
                                            double distObs = std::sqrt((double)(ii * ii + jj * jj)) * resolution;
                                            if (distObs < armLengthCore) {
                                                bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                                bCollided = true;
                                                // /top_shelf pose
                                                double robxf = robx[i][j][k];
                                                double robyf = roby[i][j][k];
                                                double robYf = robY[i][j][k];
                                                // /base_link pose
                                                robxf += cos(robYf) * baseOffsetx;
                                                robyf += sin(robYf) * baseOffsetx;
                                                visualizeRobot(
                                                        poseFrom2D(robxf, robyf, robYf),
                                                        0,
                                                        "base_candidates_collision",
                                                        base_collision_viz_id);
                                            }
                                            else if (distObs < armLength)
                                                pObs[i][j][k] = std::min(pObs[i][j][k],
                                                        std::pow(
                                                                (distObs - armLengthCore) / (armLength - armLengthCore),
                                                                2.0));// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
                                        }
                            }
                            else {
// 								bTotMax[i][j][k] = false;
// 								ROS_ERROR("    Patch for obstacle check is out of the map");
                            }
                        }

            patchSize2 = (int)(bodyLength / resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
            double bodyx, bodyy, bodyY;		// body center position biased from /top_shelf by bodyOffsetx
            int bodyi, bodyj;				// index for patch center
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            bodyx = robx[i][j][k] + cos(robY[i][j][k]) * bodyOffsetx1;
                            bodyy = roby[i][j][k] + sin(robY[i][j][k]) * bodyOffsetx1;

                            //body coords in map cells
                            bodyi = (int)((bodyx - origin.position.x) / resolution);
                            bodyj = (int)((bodyy - origin.position.y) / resolution);

                            if (!cc_->isValidState((double)(bodyi * resolution), (double)(bodyj * resolution), (double)robY[i][j][k])) {
                                //ROS_WARN("Footprint collision!");
                                bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                //bCollided = true;
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(
                                        poseFrom2D(robxf, robyf, robYf),
                                        0,
                                        "base_candidates_fp_collision",
                                        base_collision_viz_id);
                            }
                            else {
                                if (bodyi >= patchSize2 &&
                                    bodyi < width - patchSize2 &&
                                    bodyj >= patchSize2 &&
                                    bodyj < height - patchSize2)
                                {
                                    bool bCollided = false;
                                    for (int ii = -patchSize2; ii <= patchSize2 && !bCollided; ii++)
                                        for (int jj = -patchSize2; jj <= patchSize2 && !bCollided; jj++)
                                            if (map_.data[width * (bodyj + jj) + bodyi + ii] >= mapObsThr) // including unknown region
                                                    //										if (map_.data[width*(bodyj+jj)+bodyi+ii] != 0)				// only in clear region
                                                    {
                                                double distObs = std::sqrt((double)(ii * ii + jj * jj)) * resolution;
                                                if (distObs < bodyLengthCore) {
                                                    bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                                    bCollided = true;
                                                    double robxf = robx[i][j][k];
                                                    double robyf = roby[i][j][k];
                                                    double robYf = robY[i][j][k];
                                                    // /base_link pose
                                                    robxf += cos(robYf) * baseOffsetx;
                                                    robyf += sin(robYf) * baseOffsetx;
                                                    visualizeRobot(
                                                            poseFrom2D(robxf, robyf, robYf),
                                                            0,
                                                            "base_candidates_collision",
                                                            base_collision_viz_id);
                                                }
                                                else if (distObs < bodyLength)
                                                    pObs[i][j][k] *= std::min(pObs[i][j][k],
                                                            std::pow(
                                                                    (distObs - bodyLengthCore)
                                                                            / (bodyLength - bodyLengthCore), 2.0));	// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
                                            }
                                }
                                else {
                                    // 								bTotMax[i][j][k] = false;
                                    // 								ROS_ERROR("    Patch for obstacle check is out of the map");
                                }
                            }
                        }
// 			patchSize2 = (int)(bodyLength/resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
// 			double bodyx, bodyy, bodyY;		// body center position biased from /top_shelf by bodyOffsetx
// 			int bodyi, bodyj;				// index for patch center
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            bodyx = robx[i][j][k] + cos(robY[i][j][k]) * bodyOffsetx2;
                            bodyy = roby[i][j][k] + sin(robY[i][j][k]) * bodyOffsetx2;

                            bodyi = (int)((bodyx - origin.position.x) / resolution);
                            bodyj = (int)((bodyy - origin.position.y) / resolution);

                            if (!cc_->isValidState((double)(bodyi * resolution), (double)(bodyj * resolution), (double)robY[i][j][k])) {
                                //ROS_WARN("Footprint collision!");
                                bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                //bCollided = true;
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                std::vector<double> pos(3, 0);
                                pos[0] = robxf;
                                pos[1] = robyf;
                                pos[2] = robYf;
                                visualizeRobot(
                                        poseFrom2D(robxf, robyf, robYf),
                                        0,
                                        "base_candidates_fp_collision",
                                        base_collision_viz_id);
                            }
                            else {

                                if (bodyi >= patchSize2 && bodyi < width - patchSize2 && bodyj >= patchSize2
                                        && bodyj < height - patchSize2) {
                                    bool bCollided = false;
                                    for (int ii = -patchSize2; ii <= patchSize2 && !bCollided; ii++)
                                        for (int jj = -patchSize2; jj <= patchSize2 && !bCollided; jj++)
                                            if (map_.data[width * (bodyj + jj) + bodyi + ii] >= mapObsThr) // including unknown region
                                                    //										if (map_.data[width*(bodyj+jj)+bodyi+ii] != 0)				// only in clear region
                                                    {
                                                double distObs = std::sqrt((double)(ii * ii + jj * jj)) * resolution;
                                                if (distObs < bodyLengthCore) {
                                                    bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                                    bCollided = true;
                                                    // /top_shelf pose
                                                    double robxf = robx[i][j][k];
                                                    double robyf = roby[i][j][k];
                                                    double robYf = robY[i][j][k];
                                                    // /base_link pose
                                                    robxf += cos(robYf) * baseOffsetx;
                                                    robyf += sin(robYf) * baseOffsetx;
                                                    visualizeRobot(
                                                            poseFrom2D(robxf, robyf, robYf),
                                                            0,
                                                            "base_candidates_collision",
                                                            base_collision_viz_id);
                                                }
                                                else if (distObs < bodyLength)
                                                    pObs[i][j][k] *= std::min(pObs[i][j][k],
                                                            std::pow(
                                                                    (distObs - bodyLengthCore)
                                                                            / (bodyLength - bodyLengthCore), 2.0));	// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
                                            }
                                }
                                else {
                                    // 								bTotMax[i][j][k] = false;
                                    // 								ROS_ERROR("    Patch for obstacle check is out of the map");
                                }
                            }
                        }

// 			bMapReceived_ = 0;
// 			bRobPoseReceived_ = 0;
        }
    }

    // 3) robot arm workspace limit should be included here!

    if (bCheckWork) {
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

    for (int i = 0; i < nDist; i++)
        for (int j = 0; j < nAng; j++)
            for (int k = 0; k < nYaw; k++)
                if (bTotMax[i][j][k] == true)
                    pTot[i][j][k] = pGrasp[i][j][k] * pObs[i][j][k] * pWork[i][j][k];
                else
                    pTot[i][j][k] = 0.0;

    // 5) select maximum probability with min distance from robot
    // HEURISTICALLY, minimum difference in angular coordinate wrt object, then farthest from the origin of object
    if (bSortMetric == 1 || bSortMetric == 2) {
        double pTotMax = 1E-10;		// to opt out the poses with pTot[i][j][k]==0

        for (int i = 0; i < nDist; i++)
            for (int j = 0; j < nAng; j++)
                for (int k = 0; k < nYaw; k++)
                    if (pTot[i][j][k] > pTotMax)
                        pTotMax = pTot[i][j][k];
        int iMax = 0, jMax = 0, kMax = 0;		// indices for the final selection of robot pose
        int cntTotMax = 0;
        for (int i = 0; i < nDist; i++)
            for (int j = 0; j < nAng; j++)
                for (int k = 0; k < nYaw; k++)
                    if (pTot[i][j][k] == pTotMax) {
                        bTotMax[i][j][k] = true;
                        iMax = i;
                        jMax = j;
                        kMax = k;
                        cntTotMax++;
                    }
                    else {
                        bTotMax[i][j][k] = false;
                        // /top_shelf pose
                        double robxf = robx[i][j][k];
                        double robyf = roby[i][j][k];
                        double robYf = robY[i][j][k];
                        // /base_link pose
                        robxf += cos(robYf) * baseOffsetx;
                        robyf += sin(robYf) * baseOffsetx;
                        visualizeRobot(
                                poseFrom2D(robxf, robyf, robYf),
                                0,
                                "base_candidates_validity_reject",
                                base_validityreject_viz_id);
                    }

        ROS_INFO("    cntTotMax: %d", cntTotMax);

        if (cntTotMax == 0) {
            // TODO: check why the previous candidate_base_poses remain in RViz panel
            // just set to initial robot poase
            robxf = robx0;
            robyf = roby0;
            robYf = robY0;
            robxf += cos(robYf) * baseOffsetx;
            robyf += sin(robYf) * baseOffsetx;

            geometry_msgs::PoseStamped candidate_base_pose;
            candidate_base_pose.header.frame_id = "/abs_nwu";
            candidate_base_pose.header.seq = 0;
            candidate_base_pose.header.stamp = ros::Time::now();

            candidate_base_pose.pose.position.x = robxf;
            candidate_base_pose.pose.position.y = robyf;
            candidate_base_pose.pose.position.z = robzf;

            tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
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
            double angO2Rcur = atan2(roby0 - objy, robx0 - objx);	// current angular coordinate from object to robot
            double angO2Rdes, angO2Rerr, angO2RerrMin = M_PI;
            cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++) {
                        if (bTotMax[i][j][k] == true) {
                            angO2Rdes = objY + angMin + angStep * j;// desired angular coordinate from object to robot
                            angO2Rerr = angles::normalize_angle(angO2Rdes - angO2Rcur);

                            if (fabs(angO2Rerr) <= angO2RerrMin) {
                                angO2RerrMin = fabs(angO2Rerr);
                                iMax = i;
                                jMax = j;
                                kMax = k;
                                cntTotMax++;
                            }
                            else {
                                bTotMax[i][j][k] = false;
                                // /top_shelf pose
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(
                                        poseFrom2D(robxf, robyf, robYf),
                                        0,
                                        "base_candidates_prob_reject",
                                        base_probreject_viz_id);
                            }
                        }
                    }
            // update bTotMax[i][j][k]
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            angO2Rdes = objY + angMin + angStep * j;
                            angO2Rerr = angles::normalize_angle(angO2Rdes - angO2Rcur);
                            if (fabs(angO2Rerr) != angO2RerrMin) {
                                bTotMax[i][j][k] = false;
                                // /top_shelf pose
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(
                                        poseFrom2D(robxf, robyf, robYf),
                                        0,
                                        "base_candidates_prob_reject",
                                        base_probreject_viz_id);
                                cntTotMax--;
                            }
                        }
            ROS_INFO("    cntTotMax: %d", cntTotMax);

            // TODO: comment these part for more candidates
            if (bSortMetric == 2) {
                if (cntTotMax > 1)	// if more than one candidate poses are still selected
                        {

                    // b) sorting by distance from current and desired positions
                    double xysqerr, xysqerrMin = 1E10;		// distance from current to desired robot position
                    cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
                    for (int i = 0; i < nDist; i++)
                        for (int j = 0; j < nAng; j++)
                            for (int k = 0; k < nYaw; k++) {
                                if (bTotMax[i][j][k] == true) {
                                    xysqerr = std::pow(robx[i][j][k] - robx0, 2.0)
                                            + std::pow(roby[i][j][k] - roby0, 2.0);

                                    if (xysqerr <= xysqerrMin) {
                                        xysqerrMin = xysqerr;
                                        iMax = i;
                                        jMax = j;
                                        kMax = k;
                                        cntTotMax++;
                                    }
                                    else
                                        bTotMax[i][j][k] = false;
                                    // /top_shelf pose
                                    double robxf = robx[i][j][k];
                                    double robyf = roby[i][j][k];
                                    double robYf = robY[i][j][k];
                                    // /base_link pose
                                    robxf += cos(robYf) * baseOffsetx;
                                    robyf += sin(robYf) * baseOffsetx;
                                    std::vector<double> pos(3, 0);
                                    pos[0] = robxf;
                                    pos[1] = robyf;
                                    pos[2] = robYf;
                                    visualizeRobot(
                                            poseFrom2D(robxf, robyf, robYf),
                                            0,
                                            "base_candidates_prob_reject",
                                            base_probreject_viz_id);
                                }
                            }

                    // OPTIONAL: update bTotMax[i][j][k]
                    for (int i = 0; i < nDist; i++)
                        for (int j = 0; j < nAng; j++)
                            for (int k = 0; k < nYaw; k++)
                                if (bTotMax[i][j][k] == true) {
                                    xysqerr = std::pow(robx[i][j][k] - robx0, 2.0)
                                            + std::pow(roby[i][j][k] - roby0, 2.0);
                                    if (xysqerr != xysqerrMin) {
                                        bTotMax[i][j][k] = false;
                                        // /top_shelf pose
                                        double robxf = robx[i][j][k];
                                        double robyf = roby[i][j][k];
                                        double robYf = robY[i][j][k];
                                        // /base_link pose
                                        robxf += cos(robYf) * baseOffsetx;
                                        robyf += sin(robYf) * baseOffsetx;
                                        visualizeRobot(
                                                poseFrom2D(robxf, robyf, robYf),
                                                0,
                                                "base_candidates_prob_reject",
                                                base_probreject_viz_id);
                                        cntTotMax--;
                                    }
                                }
                    ROS_INFO("    cntTotMax: %d", cntTotMax);
                    if (cntTotMax > 1)
                        ROS_WARN("    Multiple candidate poses exist!");

                }
            }	// if (bSortMetric==2)
        }

        // 6-1) generate final desired robot poses with maximum pTot
        ROS_INFO("Reposition Base Command Result:");
        ROS_INFO("    Object Pose (initial): %f %f %f", objx, objy, angles::normalize_angle(objY)*180/M_PI);
        ROS_INFO("    Robot Pose (intial):   %f %f %f", robx0, roby0, angles::normalize_angle(robY0)*180/M_PI);
        // index order regarding to sorting priority
        // 	for (int i=0; i<nDist; i++)
        for (int i = nDist - 1; i >= 0; i--)
            for (int j = 0; j < nAng; j++)
                // 			for (int k=0; k<nYaw; k++)
                for (int k = nYaw - 1; k >= 0; k--)
                    if (bTotMax[i][j][k] == true) {
                        // /top_shelf pose
                        robxf = robx[i][j][k];
                        robyf = roby[i][j][k];
                        robYf = robY[i][j][k];
                        // /base_link pose
                        robxf += cos(robYf) * baseOffsetx;
                        robyf += sin(robYf) * baseOffsetx;
                        ROS_INFO("    Robot Pose (desired):  %f %f %f", robxf, robyf, angles::normalize_angle(robYf)*180/M_PI);

                        geometry_msgs::PoseStamped candidate_base_pose;
                        candidate_base_pose.header.frame_id = "/abs_nwu";
                        candidate_base_pose.header.seq = 0;
                        candidate_base_pose.header.stamp = ros::Time::now();

                        candidate_base_pose.pose.position.x = robxf;
                        candidate_base_pose.pose.position.y = robyf;
                        candidate_base_pose.pose.position.z = robzf;

                        tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
                        candidate_base_pose.pose.orientation.x = robqf[0];
                        candidate_base_pose.pose.orientation.y = robqf[1];
                        candidate_base_pose.pose.orientation.z = robqf[2];
                        candidate_base_pose.pose.orientation.w = robqf[3];
                        candidate_base_poses.push_back(candidate_base_pose);

                        visualizeRobot(
                                poseFrom2D(robxf, robyf, robYf),
                                120,
                                "base_valid_candidates",
                                base_candidates_viz_id);
                    }
    }	// if (bSortMetric==1 || bSortMetric==2)

    else if (bSortMetric == 3) {
        // candidate prefernece by difference of angular coordinates for current and desired poses
        // HEURISTIC: minimize angle difference between robot orientation (robY) and robot motion direction (angO2Rcur)
        double rob2objY = atan2(objy - roby0, objx - robx0);  // angular coordinate of displacement from robot to object
        double diffYglob;
        int cntTotMax = 0;
        for (int i = 0; i < nDist; i++)
            for (int j = 0; j < nAng; j++)
                for (int k = 0; k < nYaw; k++)
                    if (bTotMax[i][j][k] == true) {
                        diffYglob = angles::normalize_angle(robY[i][j][k] - rob2objY);
// 						pTot[i][j][k] *= std::pow(1 - (scaleDiffYglob * fabs(diffYglob)/M_PI), 2.0);	// quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
                        pTot[i][j][k] *= std::max(std::pow(1 - (scaleDiffYglob * fabs(diffYglob) / M_PI), 2.0),
                                pTotThr); // quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
                        cntTotMax++;
                    }
        ROS_INFO("Reposition Base Command Result:");
// 		ROS_INFO("    cntTotMax: %d",cntTotMax);
// 		ROS_INFO("    Number of valid candidates: %d",cntTotMax);

        if (cntTotMax == 0) {
            // TODO: check why the previous candidate_base_poses remain in RViz panel
            // just set to initial robot poase
            robxf = robx0;
            robyf = roby0;
            robYf = robY0;
            robxf += cos(robYf) * baseOffsetx;
            robyf += sin(robYf) * baseOffsetx;

            geometry_msgs::PoseStamped candidate_base_pose;
            candidate_base_pose.header.frame_id = "/abs_nwu";
            candidate_base_pose.header.seq = 0;
            candidate_base_pose.header.stamp = ros::Time::now();

            candidate_base_pose.pose.position.x = robxf;
            candidate_base_pose.pose.position.y = robyf;
            candidate_base_pose.pose.position.z = robzf;

            tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
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
        for (int j = 0; j < nAng; j++)
            for (int i = nDist - 1; i >= 0; i--)
                for (int k = nYaw - 1; k >= 0; k--)
                    if (bTotMax[i][j][k] == true) {
                        cand.i = i;
                        cand.j = j;
                        cand.k = k;
                        cand.pTot = pTot[i][j][k];
                        cands.push_back(cand);
                    }
        std::sort(cands.begin(), cands.end());

        // 6-2) generate final desired robot poses with maximum pTot
// 		ROS_INFO("Reposition Base Command Result:");
        ROS_INFO("    Object Pose (initial): %f %f %f", objx, objy, angles::normalize_angle(objY)*180/M_PI);
        ROS_INFO("    Robot Pose (intial):   %f %f %f", robx0, roby0, angles::normalize_angle(robY0)*180/M_PI);

        int cntTotThr = 0;	// number of candidate poses with pTot higher than threshold
        for (std::vector<RepositionBaseCandidate::candidate>::iterator m = cands.begin(); m != cands.end(); ++m) {
            if (m->pTot >= pTotThr) {
                int i = m->i;
                int j = m->j;
                int k = m->k;

                // /top_shelf pose
                robxf = robx[i][j][k];
                robyf = roby[i][j][k];
                robYf = robY[i][j][k];
                // /base_link pose
                robxf += cos(robYf) * baseOffsetx;
                robyf += sin(robYf) * baseOffsetx;
                ROS_INFO("    Robot Pose (desired):  %f %f %f  (i:%d j:%d k:%d)    pTot: %f", robxf, robyf, angles::normalize_angle(robYf)*180/M_PI, i, j, k, m->pTot);

                geometry_msgs::PoseStamped candidate_base_pose;
                candidate_base_pose.header.frame_id = "/abs_nwu";
                candidate_base_pose.header.seq = 0;
                candidate_base_pose.header.stamp = ros::Time::now();

                candidate_base_pose.pose.position.x = robxf;
                candidate_base_pose.pose.position.y = robyf;
                candidate_base_pose.pose.position.z = robzf;

                tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
                candidate_base_pose.pose.orientation.x = robqf[0];
                candidate_base_pose.pose.orientation.y = robqf[1];
                candidate_base_pose.pose.orientation.z = robqf[2];
                candidate_base_pose.pose.orientation.w = robqf[3];
                candidate_base_poses.push_back(candidate_base_pose);

                visualizeRobot(
                        poseFrom2D(robxf, robyf, robYf),
                        (m->pTot * 240) - 120,
                        "base_probable_candidates",
                        base_probcandidates_viz_id);

                cntTotThr++;
            }
        }
        if (cntTotThr == 0) {
            robxf = robx0;
            robyf = roby0;
            robYf = robY0;
            robxf += cos(robYf) * baseOffsetx;
            robyf += sin(robYf) * baseOffsetx;

            geometry_msgs::PoseStamped candidate_base_pose;
            candidate_base_pose.header.frame_id = "/abs_nwu";
            candidate_base_pose.header.seq = 0;
            candidate_base_pose.header.stamp = ros::Time::now();

            candidate_base_pose.pose.position.x = robxf;
            candidate_base_pose.pose.position.y = robyf;
            candidate_base_pose.pose.position.z = robzf;

            tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
            candidate_base_pose.pose.orientation.x = robqf[0];
            candidate_base_pose.pose.orientation.y = robqf[1];
            candidate_base_pose.pose.orientation.z = robqf[2];
            candidate_base_pose.pose.orientation.w = robqf[3];
            candidate_base_poses.push_back(candidate_base_pose);

            ROS_WARN("    No probable candidate poses higher than a threshold!");
            return false;
        }
        else {
            ROS_INFO("    Number of valid candidates: %d", cntTotMax);
            ROS_INFO("    Number of probable candidates: %d", cntTotThr);
        }
    }

    return true;
}

bool RepositionBaseExecutor::computeRobPoseExhaustive(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose,
    std::vector<geometry_msgs::PoseStamped>& candidate_base_poses)
{
    double objx, objy, objY, robx0, roby0, robY0;
    Eigen::Affine2d robot_pose_planar;
    computeRobotPlanarPose(robot_pose, robot_pose_planar);
    Eigen::Affine2d object_pose_planar;
    computeObjectPlanarPose(object_pose, object_pose_planar);
    robx0 = robot_pose_planar.translation()[0];
    roby0 = robot_pose_planar.translation()[1];
    robY0 = Eigen::Rotation2Dd(0.0).fromRotationMatrix(robot_pose_planar.rotation()).angle();
    objx = object_pose_planar.translation()[0];
    objy = object_pose_planar.translation()[1];
    objY = Eigen::Rotation2Dd(0.0).fromRotationMatrix(object_pose_planar.rotation()).angle();

    //visualizations
    int base_candidates_viz_id = 0;
    int base_probcandidates_viz_id = 0;
    int base_collision_viz_id = 0;
    int base_probreject_viz_id = 0;
    int base_validityreject_viz_id = 0;
    int base_failedik_viz_id = 0;
    int base_seen_viz_id = 0;
//    viz.setReferenceFrame("/abs_nwu");

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
    int nDist = 6;// r in [0.5:0.1:1.0] + camera offset 		// TODO: define adequate sample range (frame_id: /top_shelf)
    double distMin = 0.5, distStep = 0.1;
// 	int nDist = 6;		// r in [0.6:0.1:1.1] + camera offset 		// TODO: define adequate sample range (frame_id: /top_shelf)
// 	double distMin = 0.6, distStep = 0.1;
    int nAng = 12;		// th in [0:30:330]
    double angMin = 0.0, angStep = 30.0 / 180.0 * M_PI;
    int nYaw = 9;		// Y in [-20:5:20]
    double yawMin = -20.0 / 180.0 * M_PI, yawStep = 5.0 / 180.0 * M_PI;
// 	int nYaw = 5;		// Y in [-20:5:20]
// 	double yawMin = -10.0/180.0*M_PI, yawStep = 5.0/180.0*M_PI;

    // 1) grasp achievable zone 	// TODO: tune these parameters
    int secDist[] = { (int)(nDist / 3), (int)(nDist * 2 / 3) };	// divide zone by distance (for acceptable object orientation setting)
    double secAngYaw[2];// accept object orientations between these two values 	// seperately defined for different regions by secDist (values within the detail code of 1))
    double bestAngYaw;// best object orientation to get highest pGrasp probability (set as the mean value of secAngYaw[0~1] currently)
    double bestDist = 0.60;	// best object distance to get highest pGrasp probability (set as the nominal value from experiment)
//	double secSide[2] = {0.0, M_PI/4.0};    		// divide left and right-hand side  // [rad]
    double secSide[2] = { -05.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI };		// divide left and right-hand side  // [rad]
// 	double secSide[2] = {0.0, 20.0/180.0*M_PI};		// divide left and right-hand side  // [rad]
// 	double scalepGraspAngYaw = 0.1;	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
// 	double scalepGraspDist = 0.1;	// pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)
// 	double scalepGraspAngYaw = 0.05;	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
    double scalepGraspAngYaw = 0.3;	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
    double scalepGraspDist = 0.05;	// pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)

    // 2) arm position offset for computing pObs
// 	double armOffsety = -0.0;
// 	double armOffsety = -0.5;
// 	double armOffsety = -1.1;	// center of arm (when straightend)
// 	double armLength = 1.5;		// workspace radius about the center of arm
// 	double armLengthCore = 0.6;	// core workspace radius about the center of arm (pObs = 0.0)
    double armOffsetx = 0.3;	// center of arm (for normal grasping motion)
    double armOffsety = -0.5;	// center of arm (for normal grasping motion)
    double armLength = 0.5;		// workspace radius about the center of arm
    double armLengthCore = 0.3;	// core workspace radius about the center of arm (pObs = 0.0)
// 	double bodyOffsetx = -0.49+0.148975;	// center of body (except arm)
// 	double bodyLength = 0.8;	// support polygon radius about the center of body
// 	double bodyLengthCore = 0.65;	// core support polygon radius about the center of body (pObs = 0.0)
    double bodyOffsetx1 = -0.49 + 0.148975 + 0.27;	// center of front body (except arm)
    double bodyOffsetx2 = -0.49 + 0.148975 - 0.27;	// center of rear body (except arm)
    double bodyLength = 0.5;	// support polygon radius about the center of body
    double bodyLengthCore = 0.335;	// core support polygon radius about the center of body (pObs = 0.0)
    int mapObsThr = 1;			// threshold for classifying clear and occupied regions

    // 5) candidate selection criterion
    double scaleDiffYglob = 0.05;// multiply a quadratic function to pTot (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI) for bSortMetric==3
// 	double pTotThr = 0.5;		// pTot threshold for bSortMetric==3
    double pTotThr = 0.0;		// pTot threshold for bSortMetric==3

    // 6) /base_link offset from /top_shelf for final robot pose return
// 	double baseOffsetx = -0.0;
// 	double baseOffsetx = -0.3;
// 	double baseOffsetx = -0.5;
    double baseOffsetx = -0.49 + 0.148975;	// /base_link to /base_link_front_bumper_part to /top_shelf
//	double baseOffsetx = -0.49;				// /base_link to /base_link_front_bumper_part (in new Hokuyo setting) 	// same as in computeRobPose()

////////////////////////////////////////////////////////////////////////////////////

    // 0) set search space (r,th,Y)
    // (r,th): polar coordinate with respect to the object with the center fixed at the origin and the nozzle aligned to 0 rad
    // (Y): orientation about z-axis

    double pTot[nDist][nAng][nYaw], pGrasp[nDist][nAng][nYaw], pObs[nDist][nAng][nYaw], pWork[nDist][nAng][nYaw];
    bool bTotMax[nDist][nAng][nYaw];
    for (int i = 0; i < nDist; i++)
        for (int j = 0; j < nAng; j++)
            for (int k = 0; k < nYaw; k++) {
                pTot[i][j][k] = 0.0;
                pGrasp[i][j][k] = 1.0;
                pObs[i][j][k] = 1.0;
                pWork[i][j][k] = 1.0;
                bTotMax[i][j][k] = true;	// used when 1) check candidate validity before IK (process 1~4)
                                            //			 2) select the one with maximum pTot (process 5~6)
            }

    double robz0 = 0.0, robP0 = 0.0, robR0 = 0.0;
    double robxf = robx0, robyf = roby0, robzf = robz0, robYf = robY0, robPf = robP0, robRf = robR0; // final (computed) robot pose

    double robx[nDist][nAng][nYaw], roby[nDist][nAng][nYaw], robY[nDist][nAng][nYaw];	// candidate robot poses
    for (int i = 0; i < nDist; i++)
        for (int j = 0; j < nAng; j++)
            for (int k = 0; k < nYaw; k++) {
                robx[i][j][k] = objx + cos(objY + angMin + angStep * j) * (distMin + distStep * i);
                roby[i][j][k] = objy + sin(objY + angMin + angStep * j) * (distMin + distStep * i);
                robY[i][j][k] = objY + (angMin + angStep * j) - M_PI - (yawMin + yawStep * k);// angular coordinate of x-axis (frontal direction) of the robot with respect to global frame
            }

    // 1) probability of successful grasping

    if (bCheckGrasp) {
        // heuristic probability of successful grasping
        // a) position: the object should be within left-hand side of the robot and 45 deg angle of view
        // exception: exclude poses at close distance, include ones in right-hand side at far distance
        // b) orientation: the object handle should be directed to the right of the robot
        // c) exception: allow more deviation at close distance, reject most of deviations at far distance

        for (int i = 0; i < nDist; i++) {
            // c) set acceptable object orientation range
            if (i < secDist[0]) {
// 				secAngYaw[0] = 15.0/180.0*M_PI;
// 				secAngYaw[1] = 100.0/180.0*M_PI;
                secAngYaw[0] = 45.0 / 180.0 * M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[1] = 130.0 / 180.0 * M_PI;
// 				bestAngYaw   = 60.0/180.0*M_PI;
// 				bestAngYaw   = 65.0/180.0*M_PI;
// 				bestAngYaw   = -60.0/180.0*M_PI;
                bestAngYaw = -90.0 / 180.0 * M_PI;
            }
            else if (i < secDist[1]) {
// 				secAngYaw[0] = 15.0/180.0*M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[0] = 45.0 / 180.0 * M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[1] = 130.0 / 180.0 * M_PI;
// 				bestAngYaw   = 45.0/180.0*M_PI;
// 				bestAngYaw   = 60.0/180.0*M_PI;
                bestAngYaw = -90.0 / 180.0 * M_PI;
// 				bestAngYaw   = -60.0/180.0*M_PI;
            }
            else // if (i >= secDist[1])
            {
// 				secAngYaw[0] = 15.0/180.0*M_PI;
// // 				secAngYaw[1] = 30.0/180.0*M_PI;
// 				secAngYaw[1] = 60.0/180.0*M_PI;
                secAngYaw[0] = 45.0 / 180.0 * M_PI;
// 				secAngYaw[1] = 90.0/180.0*M_PI;
                secAngYaw[1] = 130.0 / 180.0 * M_PI;
// 				bestAngYaw   = 15.0/180.0*M_PI;
// 				bestAngYaw   = 50.0/180.0*M_PI;
// 				bestAngYaw   = -120.0/180.0*M_PI;
                bestAngYaw = -90.0 / 180.0 * M_PI;
// 				bestAngYaw   = -60.0/180.0*M_PI;
            }

            for (int j = 0; j < nAng; j++) {
                for (int k = 0; k < nYaw; k++) {
                    // a) object position at left-hand side to the robot
                    double rob2obj = atan2(objy - roby[i][j][k], objx - robx[i][j][k]);	// angular coordinate of a vector from robot position to object position (not orientation)
                    double diffAng = angles::normalize_angle(rob2obj - robY[i][j][k]);
                    double diffAngMax = fabs(yawMin - secSide[0]);
                    // b) object handle orientation to the right of the robot
// 						double diffY = angles::normalize_angle( objY-robY[i][j][k] );
// 						double diffYMax = std::max( fabs(secAngYaw[0]-bestAngYaw), fabs(secAngYaw[1]-bestAngYaw) );
// 						double diffYMax = M_PI/1.0;
                    double diffY = angles::normalize_angle(objY - robY[i][j][k]);
                    double diffYMax = M_PI / 2.0;
                    double diffDistMax = std::max(fabs(distMin - bestDist),
                            fabs(distMin + distStep * (nDist - 1) - bestDist));
                    if (diffAng >= secSide[0] && diffAng < secSide[1])    // left-hand side and angle of view
                            {
                        if (diffY >= secAngYaw[0] && diffY <= secAngYaw[1]) {
                            // EXCLUDE CANDIDATES WE HAVE ALREADY SEEN
                            bTotMax[i][j][k] = false; // using bTotMax instead of pGrasp to explicitly represent candidate validity (before expensive IK test)
                            // /top_shelf pose
                            double robxf = robx[i][j][k];
                            double robyf = roby[i][j][k];
                            double robYf = robY[i][j][k];
                            // /base_link pose
                            robxf += cos(robYf) * baseOffsetx;
                            robyf += sin(robYf) * baseOffsetx;
                            visualizeRobot(
                                    poseFrom2D(robxf, robyf, robYf),
                                    0,
                                    "base_candidates_seen",
                                    base_seen_viz_id);
                            // higher probability around diffY==bestAngYaw
// 							pGrasp[i][j][k] = std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 );	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
// 							pGrasp[i][j][k] = std::max( std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 ), pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)

                            // higher probability around diffY==bestAngYaw and diffDist==bestDist
// 							pGrasp[i][j][k] = std::max( std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0) * std::pow( (diffDistMax-fabs(distMin+distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0), pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                        }
                    }
// 					pGrasp[i][j][k] = std::max( std::pow( std::max( std::min(diffAng-secSide[0], 0.0)/diffAngMax + 1.0, 0.0), 1.0)
// 												* std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0)
// 												* std::pow( (diffDistMax-fabs(distMin+distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0)
// 										, pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
// 					pGrasp[i][j][k] = std::max( std::pow( std::max( std::min(diffAng-secSide[0], 0.0)/diffAngMax + 1.0, 0.0), 1.0)
                    pGrasp[i][j][k] = std::max(
                            std::pow(std::max(std::min(diffAng - 5.0 / 180.0 * M_PI, 0.0) / diffAngMax + 1.0, 0.0), 1.0)
                                    * std::pow(
                                            (diffYMax
                                                    - std::min(fabs(angles::normalize_angle(diffY - bestAngYaw)),
                                                            fabs(angles::normalize_angle(diffY - M_PI - bestAngYaw)))
                                                            * scalepGraspAngYaw) / (diffYMax), 2.0)
                                    * std::pow(
                                            (diffDistMax - fabs(distMin + distStep * i - bestDist) * scalepGraspDist)
                                                    / (diffDistMax), 2.0), pTotThr); // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                }
            }
        }
    }

    // 2) probability not to collide with obstacles
    // depends on distance to obstacles considering the position offset wrt orientation

    if (bCheckObs == true) {
        // TODO TODO
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

            int patchSize2 = (int)(armLength / resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
            double armx, army, armY;	// arm center position biased from /top_shelf by armOffsety
            int armi, armj;				// index for patch center
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
// 							armx = robx[i][j][k] - sin(robY[i][j][k])*armOffsety;
// 							army = roby[i][j][k] + cos(robY[i][j][k])*armOffsety;
                            armx = robx[i][j][k] + cos(robY[i][j][k]) * armOffsetx - sin(robY[i][j][k]) * armOffsety;
                            army = roby[i][j][k] + sin(robY[i][j][k]) * armOffsetx + cos(robY[i][j][k]) * armOffsety;

                            armi = (int)((armx - origin.position.x) / resolution);
                            armj = (int)((army - origin.position.y) / resolution);

                            if (armi >= patchSize2 && armi < width - patchSize2 && armj >= patchSize2
                                    && armj < height - patchSize2) {
                                bool bCollided = false;
                                for (int ii = -patchSize2; ii <= patchSize2 && !bCollided; ii++)
                                    for (int jj = -patchSize2; jj <= patchSize2 && !bCollided; jj++)
                                        if (map_.data[width * (armj + jj) + armi + ii] >= mapObsThr)// including unknown region
//										if (map_.data[width*(armj+jj)+armi+ii] != 0)				// only in clear region
                                                {
                                            double distObs = std::sqrt((double)(ii * ii + jj * jj)) * resolution;
                                            if (distObs < armLengthCore) {
                                                bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                                bCollided = true;
                                                // /top_shelf pose
                                                double robxf = robx[i][j][k];
                                                double robyf = roby[i][j][k];
                                                double robYf = robY[i][j][k];
                                                // /base_link pose
                                                robxf += cos(robYf) * baseOffsetx;
                                                robyf += sin(robYf) * baseOffsetx;
                                                visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_collision", base_collision_viz_id);
                                            }
                                            else if (distObs < armLength)
                                                pObs[i][j][k] = std::min(pObs[i][j][k],
                                                        std::pow(
                                                                (distObs - armLengthCore) / (armLength - armLengthCore),
                                                                2.0));// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
                                        }
                            }
                            else {
// 								bTotMax[i][j][k] = false;
// 								ROS_ERROR("    Patch for obstacle check is out of the map");
                            }
                        }

            patchSize2 = (int)(bodyLength / resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
            double bodyx, bodyy, bodyY;		// body center position biased from /top_shelf by bodyOffsetx
            int bodyi, bodyj;				// index for patch center
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            bodyx = robx[i][j][k] + cos(robY[i][j][k]) * bodyOffsetx1;
                            bodyy = roby[i][j][k] + sin(robY[i][j][k]) * bodyOffsetx1;

                            bodyi = (int)((bodyx - origin.position.x) / resolution);
                            bodyj = (int)((bodyy - origin.position.y) / resolution);

                            if (!cc_->isValidState((double)(bodyi * resolution), (double)(bodyj * resolution), (double)robY[i][j][k])) {
                                //ROS_WARN("Footprint collision!");
                                bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                //bCollided = true;
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_fp_collision", base_collision_viz_id);
                            }
                            else {

                                if (bodyi >= patchSize2 && bodyi < width - patchSize2 && bodyj >= patchSize2
                                        && bodyj < height - patchSize2) {
                                    bool bCollided = false;
                                    for (int ii = -patchSize2; ii <= patchSize2 && !bCollided; ii++)
                                        for (int jj = -patchSize2; jj <= patchSize2 && !bCollided; jj++)
                                            if (map_.data[width * (bodyj + jj) + bodyi + ii] >= mapObsThr) // including unknown region
                                                    //										if (map_.data[width*(bodyj+jj)+bodyi+ii] != 0)				// only in clear region
                                                    {
                                                double distObs = std::sqrt((double)(ii * ii + jj * jj)) * resolution;
                                                if (distObs < bodyLengthCore) {
                                                    bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                                    bCollided = true;
                                                    // /top_shelf pose
                                                    double robxf = robx[i][j][k];
                                                    double robyf = roby[i][j][k];
                                                    double robYf = robY[i][j][k];
                                                    // /base_link pose
                                                    robxf += cos(robYf) * baseOffsetx;
                                                    robyf += sin(robYf) * baseOffsetx;
                                                    visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_collision", base_collision_viz_id);
                                                }
                                                else if (distObs < bodyLength)
                                                    pObs[i][j][k] *= std::min(pObs[i][j][k],
                                                            std::pow(
                                                                    (distObs - bodyLengthCore)
                                                                            / (bodyLength - bodyLengthCore), 2.0));	// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
                                            }
                                }
                                else {
                                    // 								bTotMax[i][j][k] = false;
                                    // 								ROS_ERROR("    Patch for obstacle check is out of the map");
                                }
                            }
                        }
// 			patchSize2 = (int)(bodyLength/resolution);	// occupancy check for (patchSize2+1)x(patchSize2+1) cells
// 			double bodyx, bodyy, bodyY;		// body center position biased from /top_shelf by bodyOffsetx
// 			int bodyi, bodyj;				// index for patch center
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            bodyx = robx[i][j][k] + cos(robY[i][j][k]) * bodyOffsetx2;
                            bodyy = roby[i][j][k] + sin(robY[i][j][k]) * bodyOffsetx2;

                            bodyi = (int)((bodyx - origin.position.x) / resolution);
                            bodyj = (int)((bodyy - origin.position.y) / resolution);

                            if (!cc_->isValidState((double)(bodyi * resolution), (double)(bodyj * resolution), (double)robY[i][j][k])) {
                                //ROS_WARN("Footprint collision!");
                                bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                //bCollided = true;
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_fp_collision", base_collision_viz_id);
                            }
                            else {

                                if (bodyi >= patchSize2 && bodyi < width - patchSize2 && bodyj >= patchSize2
                                        && bodyj < height - patchSize2) {
                                    bool bCollided = false;
                                    for (int ii = -patchSize2; ii <= patchSize2 && !bCollided; ii++)
                                        for (int jj = -patchSize2; jj <= patchSize2 && !bCollided; jj++)
                                            if (map_.data[width * (bodyj + jj) + bodyi + ii] >= mapObsThr) // including unknown region
                                                    //										if (map_.data[width*(bodyj+jj)+bodyi+ii] != 0)				// only in clear region
                                                    {
                                                double distObs = std::sqrt((double)(ii * ii + jj * jj)) * resolution;
                                                if (distObs < bodyLengthCore) {
                                                    bTotMax[i][j][k] = false;	// equivalent to pObs[i][j][k] = 0;
                                                    bCollided = true;
                                                    // /top_shelf pose
                                                    double robxf = robx[i][j][k];
                                                    double robyf = roby[i][j][k];
                                                    double robYf = robY[i][j][k];
                                                    // /base_link pose
                                                    robxf += cos(robYf) * baseOffsetx;
                                                    robyf += sin(robYf) * baseOffsetx;
                                                    visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_collision", base_collision_viz_id);
                                                }
                                                else if (distObs < bodyLength)
                                                    pObs[i][j][k] *= std::min(pObs[i][j][k],
                                                            std::pow(
                                                                    (distObs - bodyLengthCore)
                                                                            / (bodyLength - bodyLengthCore), 2.0));	// pObs: quadratic function (1 at outer borders, 0 at inner borders)	// lowest value among the patch cells for current i,j,k
                                            }
                                }
                                else {
                                    // 								bTotMax[i][j][k] = false;
                                    // 								ROS_ERROR("    Patch for obstacle check is out of the map");
                                }
                            }
                        }

// 			bMapReceived_ = 0;
// 			bRobPoseReceived_ = 0;
        }
    }

    // 3) robot arm workspace limit should be included here!

    if (bCheckWork) {
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

        // check for inverse kinematics and arm planning (if object is in reachable range and within angle of view)
        for (int i = 0; i < nDist; i++) {
            for (int j = 0; j < nAng; j++) {
                for (int k = 0; k < nYaw; k++) {
                    if (bTotMax[i][j][k] == true) {
                        geometry_msgs::PoseStamped candidate_base_pose;
                        candidate_base_pose.header.frame_id = "/abs_nwu";
                        candidate_base_pose.header.seq = 0;
                        candidate_base_pose.header.stamp = ros::Time::now();

                        double robY_base = robY[i][j][k];
                        double robx_base = robx[i][j][k] + cos(robY_base) * baseOffsetx;
                        double roby_base = roby[i][j][k] + sin(robY_base) * baseOffsetx;
                        candidate_base_pose.pose.position.x = robx_base;
                        candidate_base_pose.pose.position.y = roby_base;
                        candidate_base_pose.pose.position.z = 0.0;

                        tf::Quaternion robqf = tf::createQuaternionFromRPY(0.0, 0.0, robY_base);
                        candidate_base_pose.pose.orientation.x = robqf[0];
                        candidate_base_pose.pose.orientation.y = robqf[1];
                        candidate_base_pose.pose.orientation.z = robqf[2];
                        candidate_base_pose.pose.orientation.w = robqf[3];

                        int retIKPLAN = checkIK(robot_pose, object_pose);
                        if (retIKPLAN != 1) {
                            //checkIK failed!
                            // /top_shelf pose
                            double robxf = robx[i][j][k];
                            double robyf = roby[i][j][k];
                            double robYf = robY[i][j][k];
                            // /base_link pose
                            robxf += cos(robYf) * baseOffsetx;
                            robyf += sin(robYf) * baseOffsetx;
                            visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_failed_ik", base_failedik_viz_id);
                            bTotMax[i][j][k] = false;
                        }
                        else {
                            //checkIK success!
                        }
                    }
                }
            }
        }
    }

    // 4) total probability

    for (int i = 0; i < nDist; i++)
        for (int j = 0; j < nAng; j++)
            for (int k = 0; k < nYaw; k++)
                if (bTotMax[i][j][k] == true)
                    pTot[i][j][k] = pGrasp[i][j][k] * pObs[i][j][k] * pWork[i][j][k];
                else
                    pTot[i][j][k] = 0.0;

    // 5) select maximum probability with min distance from robot
    // HEURISTICALLY, minimum difference in angular coordinate wrt object, then farthest from the origin of object
    if (bSortMetric == 1 || bSortMetric == 2) {
        double pTotMax = 1E-10;		// to opt out the poses with pTot[i][j][k]==0

        for (int i = 0; i < nDist; i++)
            for (int j = 0; j < nAng; j++)
                for (int k = 0; k < nYaw; k++)
                    if (pTot[i][j][k] > pTotMax)
                        pTotMax = pTot[i][j][k];
        int iMax = 0, jMax = 0, kMax = 0;		// indices for the final selection of robot pose
        int cntTotMax = 0;
        for (int i = 0; i < nDist; i++)
            for (int j = 0; j < nAng; j++)
                for (int k = 0; k < nYaw; k++)
                    if (pTot[i][j][k] == pTotMax) {
                        bTotMax[i][j][k] = true;
                        iMax = i;
                        jMax = j;
                        kMax = k;
                        cntTotMax++;
                    }
                    else {
                        bTotMax[i][j][k] = false;
                        // /top_shelf pose
                        double robxf = robx[i][j][k];
                        double robyf = roby[i][j][k];
                        double robYf = robY[i][j][k];
                        // /base_link pose
                        robxf += cos(robYf) * baseOffsetx;
                        robyf += sin(robYf) * baseOffsetx;
                        visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_validity_reject", base_validityreject_viz_id);
                    }

        ROS_INFO("    cntTotMax: %d", cntTotMax);

        if (cntTotMax == 0) {
            // TODO: check why the previous candidate_base_poses remain in RViz panel
            // just set to initial robot poase
            robxf = robx0;
            robyf = roby0;
            robYf = robY0;
            robxf += cos(robYf) * baseOffsetx;
            robyf += sin(robYf) * baseOffsetx;

            geometry_msgs::PoseStamped candidate_base_pose;
            candidate_base_pose.header.frame_id = "/abs_nwu";
            candidate_base_pose.header.seq = 0;
            candidate_base_pose.header.stamp = ros::Time::now();

            candidate_base_pose.pose.position.x = robxf;
            candidate_base_pose.pose.position.y = robyf;
            candidate_base_pose.pose.position.z = robzf;

            tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
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
            double angO2Rcur = atan2(roby0 - objy, robx0 - objx);	// current angular coordinate from object to robot
            double angO2Rdes, angO2Rerr, angO2RerrMin = M_PI;
            cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++) {
                        if (bTotMax[i][j][k] == true) {
                            angO2Rdes = objY + angMin + angStep * j;// desired angular coordinate from object to robot
                            angO2Rerr = angles::normalize_angle(angO2Rdes - angO2Rcur);

                            if (fabs(angO2Rerr) <= angO2RerrMin) {
                                angO2RerrMin = fabs(angO2Rerr);
                                iMax = i;
                                jMax = j;
                                kMax = k;
                                cntTotMax++;
                            }
                            else {
                                bTotMax[i][j][k] = false;
                                // /top_shelf pose
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_prob_reject", base_probreject_viz_id);
                            }
                        }
                    }
            // update bTotMax[i][j][k]
            for (int i = 0; i < nDist; i++)
                for (int j = 0; j < nAng; j++)
                    for (int k = 0; k < nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            angO2Rdes = objY + angMin + angStep * j;
                            angO2Rerr = angles::normalize_angle(angO2Rdes - angO2Rcur);
                            if (fabs(angO2Rerr) != angO2RerrMin) {
                                bTotMax[i][j][k] = false;
                                // /top_shelf pose
                                double robxf = robx[i][j][k];
                                double robyf = roby[i][j][k];
                                double robYf = robY[i][j][k];
                                // /base_link pose
                                robxf += cos(robYf) * baseOffsetx;
                                robyf += sin(robYf) * baseOffsetx;
                                visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_prob_reject", base_probreject_viz_id);
                                cntTotMax--;
                            }
                        }
            ROS_INFO("    cntTotMax: %d", cntTotMax);

            // TODO: comment these part for more candidates
            if (bSortMetric == 2) {
                if (cntTotMax > 1)	// if more than one candidate poses are still selected
                        {

                    // b) sorting by distance from current and desired positions
                    double xysqerr, xysqerrMin = 1E10;		// distance from current to desired robot position
                    cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
                    for (int i = 0; i < nDist; i++)
                        for (int j = 0; j < nAng; j++)
                            for (int k = 0; k < nYaw; k++) {
                                if (bTotMax[i][j][k] == true) {
                                    xysqerr = std::pow(robx[i][j][k] - robx0, 2.0)
                                            + std::pow(roby[i][j][k] - roby0, 2.0);

                                    if (xysqerr <= xysqerrMin) {
                                        xysqerrMin = xysqerr;
                                        iMax = i;
                                        jMax = j;
                                        kMax = k;
                                        cntTotMax++;
                                    }
                                    else {
                                        bTotMax[i][j][k] = false;
                                        // /top_shelf pose
                                        double robxf = robx[i][j][k];
                                        double robyf = roby[i][j][k];
                                        double robYf = robY[i][j][k];
                                        // /base_link pose
                                        robxf += cos(robYf) * baseOffsetx;
                                        robyf += sin(robYf) * baseOffsetx;
                                        visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_prob_reject", base_probreject_viz_id);
                                    }
                                }
                            }

                    // OPTIONAL: update bTotMax[i][j][k]
                    for (int i = 0; i < nDist; i++)
                        for (int j = 0; j < nAng; j++)
                            for (int k = 0; k < nYaw; k++)
                                if (bTotMax[i][j][k] == true) {
                                    xysqerr = std::pow(robx[i][j][k] - robx0, 2.0)
                                            + std::pow(roby[i][j][k] - roby0, 2.0);
                                    if (xysqerr != xysqerrMin) {
                                        bTotMax[i][j][k] = false;
                                        // /top_shelf pose
                                        double robxf = robx[i][j][k];
                                        double robyf = roby[i][j][k];
                                        double robYf = robY[i][j][k];
                                        // /base_link pose
                                        robxf += cos(robYf) * baseOffsetx;
                                        robyf += sin(robYf) * baseOffsetx;
                                        visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_prob_reject", base_probreject_viz_id);
                                        cntTotMax--;
                                    }
                                }
                    ROS_INFO("    cntTotMax: %d", cntTotMax);
                    if (cntTotMax > 1)
                        ROS_WARN("    Multiple candidate poses exist!");

                }
            }	// if (bSortMetric==2)
        }

        // 6-1) generate final desired robot poses with maximum pTot
        ROS_INFO("Reposition Base Command Result:");
        ROS_INFO("    Object Pose (initial): %f %f %f", objx, objy, angles::normalize_angle(objY)*180/M_PI);
        ROS_INFO("    Robot Pose (intial):   %f %f %f", robx0, roby0, angles::normalize_angle(robY0)*180/M_PI);
        // index order regarding to sorting priority
        // 	for (int i=0; i<nDist; i++)
        for (int i = nDist - 1; i >= 0; i--)
            for (int j = 0; j < nAng; j++)
                // 			for (int k=0; k<nYaw; k++)
                for (int k = nYaw - 1; k >= 0; k--)
                    if (bTotMax[i][j][k] == true) {
                        // /top_shelf pose
                        robxf = robx[i][j][k];
                        robyf = roby[i][j][k];
                        robYf = robY[i][j][k];
                        // /base_link pose
                        robxf += cos(robYf) * baseOffsetx;
                        robyf += sin(robYf) * baseOffsetx;
                        ROS_INFO("    Robot Pose (desired):  %f %f %f", robxf, robyf, angles::normalize_angle(robYf)*180/M_PI);

                        geometry_msgs::PoseStamped candidate_base_pose;
                        candidate_base_pose.header.frame_id = "/abs_nwu";
                        candidate_base_pose.header.seq = 0;
                        candidate_base_pose.header.stamp = ros::Time::now();

                        candidate_base_pose.pose.position.x = robxf;
                        candidate_base_pose.pose.position.y = robyf;
                        candidate_base_pose.pose.position.z = robzf;

                        tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
                        candidate_base_pose.pose.orientation.x = robqf[0];
                        candidate_base_pose.pose.orientation.y = robqf[1];
                        candidate_base_pose.pose.orientation.z = robqf[2];
                        candidate_base_pose.pose.orientation.w = robqf[3];
                        candidate_base_poses.push_back(candidate_base_pose);
                        visualizeRobot(poseFrom2D(robxf, robyf, robYf), 120, "base_candidates", base_candidates_viz_id);
                    }
    }	// if (bSortMetric==1 || bSortMetric==2)

    else if (bSortMetric == 3) {
        // candidate prefernece by difference of angular coordinates for current and desired poses
        // HEURISTIC: minimize angle difference between robot orientation (robY) and robot motion direction (angO2Rcur)
        double rob2objY = atan2(objy - roby0, objx - robx0);  // angular coordinate of displacement from robot to object
        double diffYglob;
        int cntTotMax = 0;
        for (int i = 0; i < nDist; i++)
            for (int j = 0; j < nAng; j++)
                for (int k = 0; k < nYaw; k++)
                    if (bTotMax[i][j][k] == true) {
                        diffYglob = angles::normalize_angle(robY[i][j][k] - rob2objY);
// 						pTot[i][j][k] *= std::pow(1 - (scaleDiffYglob * fabs(diffYglob)/M_PI), 2.0);	// quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
                        pTot[i][j][k] *= std::max(std::pow(1 - (scaleDiffYglob * fabs(diffYglob) / M_PI), 2.0),
                                pTotThr); // quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
                        cntTotMax++;
                    }
                    else {
                        // /top_shelf pose
                        double robxf = robx[i][j][k];
                        double robyf = roby[i][j][k];
                        double robYf = robY[i][j][k];
                        // /base_link pose
                        robxf += cos(robYf) * baseOffsetx;
                        robyf += sin(robYf) * baseOffsetx;
                        visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_validity_reject", base_validityreject_viz_id);
                    }

        ROS_INFO("Reposition Base Command Result:");
// 		ROS_INFO("    cntTotMax: %d",cntTotMax);
// 		ROS_INFO("    Number of valid candidates: %d",cntTotMax);

        if (cntTotMax == 0) {
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

            tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
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
        for (int j = 0; j < nAng; j++)
            for (int i = nDist - 1; i >= 0; i--)
                for (int k = nYaw - 1; k >= 0; k--)
                    if (bTotMax[i][j][k] == true) {
                        cand.i = i;
                        cand.j = j;
                        cand.k = k;
                        cand.pTot = pTot[i][j][k];
                        cands.push_back(cand);
                    }
        std::sort(cands.begin(), cands.end());

        // check for arm planning (at least 10 candidates)
        int cntCheckPLAN = 0;
        int cntCheckPLANreject = 0;
        int cntCheckPLANMax = 2; 		// TODO: decide the number of arm planning test
        for (std::vector<RepositionBaseCandidate::candidate>::iterator m = cands.begin(); m != cands.end(); ++m) {
            if (cntCheckPLAN == cntCheckPLANMax)
                break;

            int i = m->i;
            int j = m->j;
            int k = m->k;
            geometry_msgs::PoseStamped candidate_base_pose;
            candidate_base_pose.header.frame_id = "/abs_nwu";
            candidate_base_pose.header.seq = 0;
            candidate_base_pose.header.stamp = ros::Time::now();

            double robY_base = robY[i][j][k];
            double robx_base = robx[i][j][k] + cos(robY_base) * baseOffsetx;
            double roby_base = roby[i][j][k] + sin(robY_base) * baseOffsetx;
            candidate_base_pose.pose.position.x = robx_base;
            candidate_base_pose.pose.position.y = roby_base;
            candidate_base_pose.pose.position.z = 0.0;

            tf::Quaternion robqf = tf::createQuaternionFromRPY(0.0, 0.0, robY_base);
            candidate_base_pose.pose.orientation.x = robqf[0];
            candidate_base_pose.pose.orientation.y = robqf[1];
            candidate_base_pose.pose.orientation.z = robqf[2];
            candidate_base_pose.pose.orientation.w = robqf[3];

            int err = checkFeasibleMoveToPregraspTrajectory(candidate_base_pose, current_goal_->gas_can_in_map);
            printf("retIKPLAN: %d\n", err);
            if (err) {
                cntCheckPLANreject++;
                cands.erase(m);
                m--;
            }
            else {
                cntCheckPLAN++;
            }
        }
        printf("Number of rejection until finding %d feasible candidates: %d\n", cntCheckPLANMax, cntCheckPLANreject);

        // 6-2) generate final desired robot poses with maximum pTot
// 		ROS_INFO("Reposition Base Command Result:");
        ROS_INFO("    Object Pose (initial): %f %f %f", objx, objy, angles::normalize_angle(objY)*180/M_PI);
        ROS_INFO("    Robot Pose (intial):   %f %f %f", robx0, roby0, angles::normalize_angle(robY0)*180/M_PI);

        int cntTotThr = 0;	// number of candidate poses with pTot higher than threshold
        for (std::vector<RepositionBaseCandidate::candidate>::iterator m = cands.begin(); m != cands.end(); ++m)
            if (m->pTot >= pTotThr) {
                int i = m->i;
                int j = m->j;
                int k = m->k;

                // /top_shelf pose
                robxf = robx[i][j][k];
                robyf = roby[i][j][k];
                robYf = robY[i][j][k];
                // /base_link pose
                robxf += cos(robYf) * baseOffsetx;
                robyf += sin(robYf) * baseOffsetx;
// 				ROS_INFO("    Robot Pose (desired):  %f %f %f", robxf,robyf,angles::normalize_angle(robYf)*180/M_PI);
                ROS_INFO("    Robot Pose (desired):  %f %f %f  (i:%d j:%d k:%d)    pTot: %f", robxf, robyf,
                        angles::normalize_angle(robYf)*180/M_PI, i, j, k, m->pTot);

                geometry_msgs::PoseStamped candidate_base_pose;
                candidate_base_pose.header.frame_id = "/abs_nwu";
                candidate_base_pose.header.seq = 0;
                candidate_base_pose.header.stamp = ros::Time::now();

                candidate_base_pose.pose.position.x = robxf;
                candidate_base_pose.pose.position.y = robyf;
                candidate_base_pose.pose.position.z = robzf;

                tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
                candidate_base_pose.pose.orientation.x = robqf[0];
                candidate_base_pose.pose.orientation.y = robqf[1];
                candidate_base_pose.pose.orientation.z = robqf[2];
                candidate_base_pose.pose.orientation.w = robqf[3];
                candidate_base_poses.push_back(candidate_base_pose);

                visualizeRobot(poseFrom2D(robxf, robyf, robYf), (m->pTot * 240) - 120, "base_probable_candidates", base_candidates_viz_id);

                cntTotThr++;
            }
        if (cntTotThr == 0) {
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

            tf::Quaternion robqf = tf::createQuaternionFromRPY(robRf, robPf, robYf);
            candidate_base_pose.pose.orientation.x = robqf[0];
            candidate_base_pose.pose.orientation.y = robqf[1];
            candidate_base_pose.pose.orientation.z = robqf[2];
            candidate_base_pose.pose.orientation.w = robqf[3];
            candidate_base_poses.push_back(candidate_base_pose);

            ROS_WARN("    No probable candidate poses higher than a threshold!");
            return false;
        }
        else {
            ROS_INFO("    Number of IK feasible candidates: %d", cntTotMax);
            ROS_INFO("    Number of PLAN feasible candidates: %d", cntTotThr);
        }

    }

    return true;
}

/// \brief Filter grasp candidates by kinematics and visibility
void RepositionBaseExecutor::filterGraspCandidates(
    std::vector<GraspCandidate>& candidates,
    const Eigen::Affine3d& T_camera_robot,
    double marker_incident_angle_threshold_rad) const
{
    filterGraspCandidatesIK(candidates);
    filterGraspCandidatesVisibility(candidates, T_camera_robot, marker_incident_angle_threshold_rad);
}

/// \brief Filter grasp candidates by kinematic feasibility (inverse kinematics)
void RepositionBaseExecutor::filterGraspCandidatesIK(
    std::vector<GraspCandidate>& candidates) const
{
    std::vector<GraspCandidate> filtered_candidates;
    filtered_candidates.reserve(candidates.size());

    int num_not_reachable = 0;
    for (const GraspCandidate& grasp_candidate : candidates) {
        // check for an ik solution to this grasp pose
        moveit::core::RobotState robot_state(robot_model_);
        robot_state.setToDefaultValues();
        bool check_ik = robot_state.setFromIK(manip_group_, grasp_candidate.grasp_candidate_transform);
        std::vector<double> sol;
        if (check_ik) {
            robot_state.copyJointGroupPositions(manip_group_, sol);
        }

        if (!check_ik) {
            ++num_not_reachable;
        }

        if (check_ik) {
            GraspCandidate reachable_grasp_candidate(
                    grasp_candidate.grasp_candidate_transform,
                    grasp_candidate.T_object_grasp,
                    grasp_candidate.u);
            filtered_candidates.push_back(reachable_grasp_candidate);

            ROS_INFO("Grasp pose: %s", to_string(grasp_candidate.grasp_candidate_transform).c_str());
            ROS_INFO("IK sol: %s", to_string(sol).c_str());
        }
    }

    ROS_INFO("%d of %zd grasps not reachable", num_not_reachable, candidates.size());
    std::swap(filtered_candidates, candidates);
}

/// \brief Filter grasp candidates by fiducial visibility.
///
/// Fiducial visibility defined by the angle of incidence between the viewer
/// and the fiducial frame (assumed z normal).
///
/// \param[in,out] candidates Grasp candidates in the robot frame
void RepositionBaseExecutor::filterGraspCandidatesVisibility(
    std::vector<GraspCandidate>& candidates,
    const Eigen::Affine3d& T_camera_robot,
    double marker_incident_angle_threshold_rad) const
{
    std::vector<GraspCandidate> filtered_candidates;
    filtered_candidates.reserve(candidates.size());

    int num_not_visible = 0;
    for (const GraspCandidate& grasp_candidate : candidates) {
        // check for visibility of the ar marker to this grasp pose
        Eigen::Affine3d T_camera_marker = // assume link == wrist
                T_camera_robot *
                grasp_candidate.grasp_candidate_transform *
                attached_markers_[0].link_to_marker;

        ROS_DEBUG("  Camera -> Marker: %s", to_string(T_camera_marker).c_str());
        Eigen::Vector3d camera_view_axis = -Eigen::Vector3d::UnitZ();
        Eigen::Vector3d marker_plane_normal;
        marker_plane_normal.x() = T_camera_marker(0, 2);
        marker_plane_normal.y() = T_camera_marker(1, 2);
        marker_plane_normal.z() = T_camera_marker(2, 2);

        ROS_DEBUG("  Marker Plane Normal [camera view frame]: %s", to_string(marker_plane_normal).c_str());

        double dp = marker_plane_normal.dot(camera_view_axis);
        ROS_DEBUG("  Marker Normal * Camera View Axis: %0.3f", dp);

        // the optimal situation is when the camera is facing the marker
        // directly and this angle is 0
        double angle = acos(dp);
        ROS_DEBUG("  Angle: %0.3f", angle);

        bool check_visibility = angle < marker_incident_angle_threshold_rad;

        if (!check_visibility) {
            ++num_not_visible;
        }

        if (check_visibility) {
            GraspCandidate reachable_grasp_candidate(
                    grasp_candidate.grasp_candidate_transform,
                    grasp_candidate.T_object_grasp,
                    grasp_candidate.u);
            filtered_candidates.push_back(reachable_grasp_candidate);

            ROS_INFO("Grasp pose: %s", to_string(grasp_candidate.grasp_candidate_transform).c_str());
        }
    }

    ROS_INFO("%d of %zd grasps not visible", num_not_visible, candidates.size());
    std::swap(filtered_candidates, candidates);
}

/// \brief Return a sequence of grasp candidates to try
///
/// The grasp candidates are filtered by inverse kinematics and tag detection
/// feasibility checks. The resulting sequence is sorted by the probability of
/// success of the grasp.
std::vector<RepositionBaseExecutor::GraspCandidate>
RepositionBaseExecutor::generateFilteredGraspCandidates(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose)
{
    Eigen::Affine3d T_robot_object = robot_pose.inverse() * object_pose;

    int max_num_candidates = 100;
    std::vector<GraspCandidate> grasp_candidates =
            sampleGraspCandidates(T_robot_object, max_num_candidates);
    ROS_INFO("Sampled %zd grasp poses", grasp_candidates.size());

    visualizeGraspCandidates(grasp_candidates, robot_pose, "grasp_candidates_checkIKPLAN");

    Eigen::Affine3d T_robot_camera = robot_state_->getGlobalLinkTransform(camera_view_frame_);

    filterGraspCandidates(grasp_candidates, T_robot_camera.inverse(), sbpl::utils::ToRadians(45.0));

    ROS_INFO("Produced %zd reachable grasp poses", grasp_candidates.size());

    if (grasp_candidates.empty()) {
        ROS_WARN("No reachable grasp candidates available");
        return grasp_candidates;
    }

    visualizeGraspCandidates(grasp_candidates, robot_pose, "grasp_candidates_checkIKPLAN_filtered");

    // grasp poses in the middle of the handle are more desirable; sort them
    // that way
    const double min_u = 0.0;
    const double max_u = 1.0;

    std::sort(grasp_candidates.begin(), grasp_candidates.end(),
            [&](const GraspCandidate& a, const GraspCandidate& b) -> bool
            {
                double mid_u = 0.5 * (min_u + max_u);
                return fabs(a.u - mid_u) < fabs(b.u - mid_u);
            });

    // limit the number of grasp attempts by the configured amount
    grasp_candidates.resize(max_grasp_candidates_);

    ROS_INFO("Attempting %zd grasps", grasp_candidates.size());
    return grasp_candidates;
}

Eigen::Affine3d RepositionBaseExecutor::poseFrom2D(
    double x, double y, double yaw) const
{
    return Eigen::Translation3d(x, y, 0.0) *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void RepositionBaseExecutor::computeRobotPlanarPose(
    const Eigen::Affine3d& robot_pose,
    Eigen::Affine2d& planar_pose) const
{
    // initial robot pose
    double robx0 = robot_pose.translation()[0];
    double roby0 = robot_pose.translation()[1];
    double robz0 = robot_pose.translation()[2];
    // assuming that rotation axis is parallel to z-axis
    const Eigen::Quaterniond robot_quat(robot_pose.rotation());
    double robY0 = 2.0 * acos(robot_quat.w()) * sign(robot_quat.z());
    double robP0 = 0.0;
    double robR0 = 0.0;

    // transform from base_link to top_shelf (in 2D)
//                double baseOffsetx = -0.49+0.148975; // base_link to base_link_front_bumper_part to /top_shelf
    double baseOffsetx = -0.49; // /base_link to /base_link_front_bumper_part (in new Hokuyo setting) // same as in computeRobPose()
    robx0 -= cos(robY0) * baseOffsetx;
    roby0 -= sin(robY0) * baseOffsetx;

    planar_pose = Eigen::Translation2d(robx0, roby0) * Eigen::Rotation2Dd(robY0);
}

void RepositionBaseExecutor::computeObjectPlanarPose(
    const Eigen::Affine3d& object_pose,
    Eigen::Affine2d& planar_pose) const
{
    // given object pose (x-axis is aligned to canister hose)
    double objx = object_pose.translation()[0];
    double objy = object_pose.translation()[1];
    double objz = object_pose.translation()[2];

    Eigen::AngleAxisd objAA(object_pose.rotation());
    Eigen::Vector3d objAxis = objAA.axis();
    Eigen::Vector3d zAxis(0, 0, 1);
    // assuming that rotation axis is parallel to z-axis
    // M_PI / 2 offset due to definition of object frame in new mesh file
    double objY = objAA.angle() * objAxis.dot(zAxis);
    objY = angles::normalize_angle(objY - M_PI / 2.0);
    double objP = 0.0;
    double objR = 0.0;

    planar_pose = Eigen::Translation2d(objx, objy) * Eigen::Rotation2Dd(objY);
}

bool RepositionBaseExecutor::tryFeasibleArmCheck(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose) const
{
    Eigen::Affine2d rp;
    computeRobotPlanarPose(robot_pose, rp);

    Eigen::Affine2d op;
    computeObjectPlanarPose(object_pose, op);

    // check for inverse kinematics and arm planning (if object is in reachable range and within angle of view)
    // TODO: a more general than (distMin + (nDist-1)*distStep) determined in computeRobPose()
//    double secDist[2] = { 0.3, 1.5 };
    double secDist[2] = { 0.5, 1.0 };
    Eigen::Vector2d dp = op.translation() - rp.translation();

    Eigen::Rotation2Dd robot_yaw(0.0); robot_yaw.fromRotationMatrix(rp.rotation());
    Eigen::Rotation2Dd object_yaw(0.0); object_yaw.fromRotationMatrix(op.rotation());
    double distRob2Obj = dp.norm();
    if (distRob2Obj >= secDist[0] && distRob2Obj <= secDist[1]) {
        // TODO: a more general than secSide[2] determined in computeRobPose()
//        double secSide[2] = { -20.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI };
        double secSide[2] = { -05.0 / 180.0 * M_PI, 40.0 / 180.0 * M_PI };
        // angular coordinate of a vector from robot position to object position
        double rob2obj = std::atan2(dp.y(), dp.x());
        double diffAng = angles::normalize_angle(rob2obj - robot_yaw.angle());
        if (diffAng >= secSide[0] && diffAng <= secSide[1]) {
            // left-hand side and angle of view
            double secAngYaw[2];
            secAngYaw[0] = 45.0 / 180.0 * M_PI;
            secAngYaw[1] = 130.0 / 180.0 * M_PI;

            double diffY = angles::normalize_angle(object_yaw.angle() - robot_yaw.angle());
            if (diffY >= secAngYaw[0] && diffY <= secAngYaw[1]) {
                return true;
            }
        }
    }

    return false;
}

/// \brief Check for a feasible trajectory exists to grasp the object
int RepositionBaseExecutor::checkFeasibleMoveToPregraspTrajectory(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose)
{
    ROS_INFO("check for feasible arm trajectory!");

    std::vector<GraspCandidate> grasp_candidates;
    try {
        grasp_candidates = generateFilteredGraspCandidates(robot_pose, object_pose);
    }
    catch (const tf::TransformException& ex) {
        ROS_WARN("Failed to generate grasp candidates %s", ex.what());
        return 1;
    }

    if (grasp_candidates.empty()) {
        ROS_WARN("Failed to plan to all reachable grasps");
        return 2;
    }

    // wait for move arm action to come up
    ROS_WARN("wait for action '%s' to come up", move_arm_command_action_name_.c_str());
    if (!waitForActionServer(move_arm_command_client_, move_arm_command_action_name_, ros::Duration(0.1), ros::Duration(5.0))) {
        ROS_WARN("failed to connect to '%s' action server", move_arm_command_action_name_.c_str());
        return 3;
    }

    // try all grasps
    // TODO: all grasping code should be taken out of this service and moved to
    // a new service that can plan simultaneously for all grasp poses
    for (size_t gidx = 0; gidx < grasp_candidates.size(); ++gidx) {
        ROS_INFO("attempt grasp %zu/%zu", gidx, grasp_candidates.size());
        const GraspCandidate& grasp = grasp_candidates[gidx];

        rcta::MoveArmCommandGoal pregrasp_goal;
        pregrasp_goal.type = rcta::MoveArmCommandGoal::EndEffectorGoal;
        tf::poseEigenToMsg(grasp.grasp_candidate_transform, pregrasp_goal.goal_pose);
        auto result_cb = boost::bind(&RepositionBaseExecutor::move_arm_command_result_cb, this, _1, _2);
        move_arm_command_client_->sendGoal(pregrasp_goal, result_cb);

        const ros::Duration move_arm_command_timeout(30.0);
        move_arm_command_client_->waitForResult(move_arm_command_timeout);

        if (move_arm_command_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return 0;
        }
    }

    return 1;
}

int RepositionBaseExecutor::checkIK(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose)
{
    ROS_INFO("checkIK!");
    auto candidates = generateFilteredGraspCandidates(robot_pose, object_pose);
    return !candidates.empty();
}

int RepositionBaseExecutor::checkFeasibleMoveToPregraspTrajectory(
    const geometry_msgs::PoseStamped& robot_pose,
    const geometry_msgs::PoseStamped& object_pose)
{
    Eigen::Affine3d T_world_robot, T_world_object;
    tf::poseMsgToEigen(robot_pose.pose, T_world_robot);
    tf::poseMsgToEigen(object_pose.pose, T_world_object);
    return checkFeasibleMoveToPregraspTrajectory(T_world_robot, T_world_object);
}

/// \brief Sample a set of candidate grasp poses
///
/// Grasp candidates are defined as pregrasp poses for the "wrist" of the robot.
/// The pregrasp is offset from the grasp pose by the configured fixed offset.
/// The resulting pregrasp samples are specified in the robot frame.
std::vector<RepositionBaseExecutor::GraspCandidate>
RepositionBaseExecutor::sampleGraspCandidates(
    const Eigen::Affine3d& robot_to_object,
    int num_candidates) const
{
    const double min_u = 0.0;
    const double max_u = 1.0;

    std::vector<GraspCandidate> grasp_candidates;
    grasp_candidates.reserve(num_candidates);
    for (int i = 0; i < num_candidates; ++i) {
        ROS_DEBUG("Candidate Pregrasp %3d", i);
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

        if (knot_num < grasp_spline_->degree() || knot_num >= grasp_spline_->knots().size() - grasp_spline_->degree()) {
            ROS_DEBUG("Skipping grasp_spline(%0.3f) [point governed by same knot more than once]", u);
            continue;
        }

        Eigen::Vector3d object_pos_robot_frame(robot_to_object.translation());

        Eigen::Vector3d sample_spline_point = (*grasp_spline_)(u);
        Eigen::Vector3d sample_spline_deriv = grasp_spline_->deriv(u);

        ROS_DEBUG("    Sample Spline Point [canister frame]: %s", to_string(sample_spline_point).c_str());
        ROS_DEBUG("    Sample Spline Deriv [canister frame]: %s", to_string(sample_spline_deriv).c_str());

        Eigen::Vector3d sample_spline_point_robot_frame =
                robot_to_object *
                Eigen::Affine3d(Eigen::Scaling(gas_can_scale_)) *
                sample_spline_point;

        Eigen::Vector3d sample_spline_deriv_robot_frame =
                robot_to_object.rotation() *
                sample_spline_deriv.normalized();

        ROS_DEBUG("    Sample Spline Point [robot frame]: %s", to_string(sample_spline_point_robot_frame).c_str());
        ROS_DEBUG("    Sample Spline Deriv [robot frame]: %s", to_string(sample_spline_deriv_robot_frame).c_str());

        // compute the normal to the grasp spline that most points "up" in the
        // robot frame
        Eigen::Vector3d up_bias(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d up_grasp_dir =
                up_bias -
                        up_bias.dot(sample_spline_deriv_robot_frame) *
                        sample_spline_deriv_robot_frame;
        up_grasp_dir.normalize();
        up_grasp_dir *= -1.0;

        // compute the normal to the grasp spline that most points "down" in the
        // robot frame
        Eigen::Vector3d down_bias(-Eigen::Vector3d::UnitZ());
        Eigen::Vector3d down_grasp_dir =
                down_bias -
                        down_bias.dot(sample_spline_deriv_robot_frame) *
                        sample_spline_deriv_robot_frame;
        down_grasp_dir.normalize();
        down_grasp_dir *= -1.0;

        Eigen::Vector3d grasp_dir;
        if (up_grasp_dir.dot(sample_spline_point_robot_frame - object_pos_robot_frame) < 0) {
            grasp_dir = up_grasp_dir;
        }
        else {
            ROS_DEBUG("Skipping grasp_spline(%0.3f) [derivative goes backwards along the spline]", u);
            continue;
//            grasp_dir = down_grasp_dir;
        }

        ROS_DEBUG("    Grasp Direction [robot frame]: %s", to_string(grasp_dir).c_str());

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
                Eigen::Translation3d(sample_spline_point_robot_frame) *
                grasp_rotation_matrix;

        // robot -> grasp candidate (desired tool) *
        // tool -> wrist *
        // wrist (grasp) -> pregrasp =
        // robot -> wrist
        Eigen::Affine3d candidate_wrist_transform =
                grasp_candidate_rotation *
                wrist_to_tool_.inverse() *
                grasp_to_pregrasp_;

        ROS_DEBUG("    Pregrasp Pose [robot frame]: %s", to_string(candidate_wrist_transform).c_str());

        grasp_candidates.push_back(
                GraspCandidate(candidate_wrist_transform, robot_to_object.inverse() * candidate_wrist_transform, u));
    }

    // add each grasps flipped variant
    std::size_t original_size = grasp_candidates.size();
    for (int i = 0; i < original_size; ++i) {
        const GraspCandidate& grasp = grasp_candidates[i];
        Eigen::Affine3d flipped_candidate_transform =
                grasp.grasp_candidate_transform *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        grasp_candidates.push_back(
                GraspCandidate(
                        flipped_candidate_transform,
                        robot_to_object.inverse() * flipped_candidate_transform,
                        grasp.u));
    }
    return grasp_candidates;
}

void RepositionBaseExecutor::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::MoveArmCommandResult::ConstPtr& result)
{
    move_arm_command_goal_state_ = state;
    move_arm_command_result_ = result;
}

void RepositionBaseExecutor::visualizeGraspCandidates(
    const std::vector<GraspCandidate>& grasps,
    Eigen::Affine3d T_robot_to_map,
    std::string ns) const
{
    ROS_INFO("Visualizing %zd grasps", grasps.size());
    geometry_msgs::Vector3 triad_scale = geometry_msgs::CreateVector3(0.1, 0.01, 0.01);

    visualization_msgs::MarkerArray all_triad_markers;

    // create triad markers to be reused for each grasp candidate
    visualization_msgs::MarkerArray triad_markers = msg_utils::create_triad_marker_arr(triad_scale);
    for (visualization_msgs::Marker& marker : triad_markers.markers) {
        marker.header.frame_id = "/abs_nwu";
        marker.ns = ns;
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
            Eigen::Affine3d new_marker_transform =
                    T_robot_to_map *
                    candidate.grasp_candidate_transform *
                    marker_transform;
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
        viz_pub_.publish(single_triad_markers);
        single_triad_markers.markers.clear();

        // restore marker transforms to their original frame
        for (std::size_t i = 0; i < triad_markers.markers.size(); ++i) {
            tf::poseEigenToMsg(marker_transforms[i], triad_markers.markers[i].pose);
        }
    }

    ROS_INFO("Visualizing %zd triad markers", all_triad_markers.markers.size());
    viz_pub_.publish(all_triad_markers);
}

template <typename ActionType>
bool RepositionBaseExecutor::waitForActionServer(
    std::unique_ptr<actionlib::SimpleActionClient<ActionType>>& action_client,
    const std::string& action_name,
    const ros::Duration& poll_duration,
    const ros::Duration& timeout)
{
    ROS_INFO("Waiting for action server '%s'", action_name.c_str());
    if (!action_client) {
        ROS_WARN("Action client is null");
        return false;
    }
    ros::Time start = ros::Time::now();
    while (timeout == ros::Duration(0) || ros::Time::now() < start + timeout) {
        ros::spinOnce();
        if (!action_client->isServerConnected()) {
            action_client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
            if (!action_client) {
                ROS_WARN("Failed to reinstantiate action client '%s'", action_name.c_str());
                return false;
            }
        }
        if (action_client->isServerConnected()) {
            return true;
        }
        poll_duration.sleep();
        ROS_INFO("Waited %0.3f seconds for action server '%s'...", (ros::Time::now() - start).toSec(),
                action_name.c_str());
    }
    return false;
}

bool RepositionBaseExecutor::download_marker_params()
{
    double marker_to_link_x;
    double marker_to_link_y;
    double marker_to_link_z;
    double marker_to_link_roll_degs;
    double marker_to_link_pitch_degs;
    double marker_to_link_yaw_degs;

    AttachedMarker attached_marker;

    bool success =
            msg_utils::download_param(ph_, "tracked_marker_id", attached_marker.marker_id) &&
            msg_utils::download_param(ph_, "tracked_marker_attached_link", attached_marker.attached_link) &&
            msg_utils::download_param(ph_, "marker_to_link_x", marker_to_link_x) &&
            msg_utils::download_param(ph_, "marker_to_link_y", marker_to_link_y) &&
            msg_utils::download_param(ph_, "marker_to_link_z", marker_to_link_z) &&
            msg_utils::download_param(ph_, "marker_to_link_roll_deg", marker_to_link_roll_degs) &&
            msg_utils::download_param(ph_, "marker_to_link_pitch_deg", marker_to_link_pitch_degs) &&
            msg_utils::download_param(ph_, "marker_to_link_yaw_deg", marker_to_link_yaw_degs);
    if (!success) {
        ROS_WARN("Failed to download marker params");
        return false;
    }

    attached_marker.link_to_marker =
            Eigen::Affine3d(
                    Eigen::Translation3d(marker_to_link_x, marker_to_link_y, marker_to_link_z) *
                    Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(sbpl::utils::ToRadians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

    attached_markers_.push_back(std::move(attached_marker));
    return true;
}

void RepositionBaseExecutor::visualizeRobot(
    const Eigen::Affine3d& pose,
    int hue,
    const std::string& ns,
    int& id)
{
    const moveit::core::JointModel* root_joint = robot_model_->getRootJoint();
    robot_state_->setJointPositions(root_joint, pose);
    visualization_msgs::MarkerArray ma;
    const auto& link_names = robot_model_->getLinkModelNames();
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    robot_state_->getRobotMarkers(ma, link_names, color, ns, ros::Duration(0), false);
    for (auto& marker : ma.markers) {
        marker.id = id++;
    }
    viz_pub_.publish(ma);
}
