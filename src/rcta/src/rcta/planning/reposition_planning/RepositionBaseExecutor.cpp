#include "RepositionBaseExecutor.h"

// standard includes
#include <algorithm>

// systemm includes
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl/headers.h>
#include <sbpl_geometry_utils/utils.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/RunUponDestruction.h>
#include <spellbook/utils/utils.h>
#include <tf_conversions/tf_eigen.h>

#include <rcta/common/comms/actionlib.h>

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
    manip_group_ = robot_model_->getJointModelGroup(manip_name_);
    const auto& tip_frames = manip_group_->getSolverInstance()->getTipFrames();
    ROS_INFO("'%s' group tip frames:", manip_name_.c_str());
    for (const auto& tip_frame : tip_frames) {
        ROS_INFO("  %s", tip_frame.c_str());
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

    viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 100);
    pgrasp_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("pgrasp_map", 1);
    pobs_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("pobs_map", 1);

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

    double baseOffsetx = -0.49 + 0.148975;  // -0.49, -0.5, -0.3, 0.0
    T_mount_robot_ = Eigen::Translation2d(baseOffsetx, 0.0);

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

            // NOTE: special for the gascan
            Pose2D gascan_pose = poseEigen3ToSimpleGascan(object_pose);

            const auto& map = current_goal_->map;

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
            } else if (computeRobPose(
                    map,
                    poseEigen3ToSimple(robot_pose),
                    gascan_pose,
                    candidate_base_poses))
            {
                rcta_msgs::RepositionBaseCommandResult result;
                result.result = rcta_msgs::RepositionBaseCommandResult::SUCCESS;
                result.candidate_base_poses = candidate_base_poses;
                as_->setSucceeded(result);
                status_ = RepositionBaseExecutionStatus::IDLE;
            } else if (computeRobPoseExhaustive(
                    map,
                    robot_pose,
                    object_pose,
                    candidate_base_poses))
            {
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
    const auto& map = current_goal_->map;
    ROS_INFO("  Goal ID: %u", current_goal_->id);
    ROS_INFO("  Retry Count: %d", current_goal_->retry_count);
    ROS_INFO("  Gas Can Pose: %s", to_string(current_goal_->gas_can_in_map).c_str());
    ROS_INFO("  Base Link Pose: %s", to_string(current_goal_->base_link_in_map).c_str());
    ROS_INFO("  Map:");
    ROS_INFO("    header: %s", to_string(map.header).c_str());
    ROS_INFO("    origin: (%0.3f, %0.3f)", map.info.origin.position.x, map.info.origin.position.y);
    ROS_INFO("    size: (%u, %u)", map.info.width, map.info.height);
    ROS_INFO("    res: %0.3f", map.info.resolution);
    ROS_INFO("    data: <%zu elements>", map.data.size());

    // NOTE: There was some fishy business going on here transforming the
    // incoming pose to a pose specified in another frame, presumably for
    // compatibility. Removing for now, but keep this in mind...it shouldn't
    // be necessary, strictly speaking
//    const std::string world_frame = "abs_nwu";

    const geometry_msgs::PoseStamped& robot_pose = current_goal_->base_link_in_map;
    const std::string& map_frame = current_goal_->map.header.frame_id;
    if (robot_pose.header.frame_id != map_frame) {
        try {
            ROS_INFO("transform robot pose from frame '%s' into frame '%s'", robot_pose.header.frame_id.c_str(), map_frame.c_str());
            listener_.transformPose(
                    map_frame,
                    robot_pose,
                    robot_pose_world_frame_);
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("Failed to transform from '%s' to '%s' (%s)", current_goal_->base_link_in_map.header.frame_id.c_str(), map_frame.c_str(), ex.what());
            as_->setAborted();
            return;
        }
    }
    else {
        ROS_INFO("robot pose already in map frame :)");
        robot_pose_world_frame_ = current_goal_->base_link_in_map;
    }

    // update collision checker if valid map
    if (!cc_->updateOccupancyGrid(current_goal_->map)) {
        ROS_ERROR("Failed to update collision checker with latest map");
        as_->setAborted();
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

/// \brief Compute a sequence of candidate poses for grasping
///
/// The sequence is sorted by priority determined by a number of factors. The
/// current policy for robot pose selection is:
///
/// 0) set search space
/// 1) candidate pose validity for grasing
/// 2) distance to obstacles
/// 3) kinematics of hdt arm
/// 4) multiplication of 1-3
/// 5) candidate sorting by some metrics
/// 6) generate final candidate base poses
bool RepositionBaseExecutor::computeRobPose(
    const nav_msgs::OccupancyGrid& map,
    const Pose2D& robot_pose,
    const Pose2D& object_pose,
    std::vector<geometry_msgs::PoseStamped>& candidate_base_poses)
{
    ROS_INFO("computeRobPose");
    ROS_INFO("robot pose: (%0.3f, %0.3f, %0.3f)", robot_pose.x, robot_pose.y, angles::to_degrees(robot_pose.yaw));
    ROS_INFO("object pose: (%0.3f, %0.3f, %0.3f)", object_pose.x, object_pose.y, angles::to_degrees(object_pose.yaw));

    // visualizations
    int base_candidates_viz_id = 0;
    int base_probcandidates_viz_id = 0;
    int base_collision_viz_id = 0;
    int base_footprint_viz_id = 0;
    int base_probreject_viz_id = 0;
    int base_validityreject_viz_id = 0;
    int base_failedik_viz_id = 0;

    ///////////////////////
    // PARAMETER SETTING //
    ///////////////////////

    SearchSpaceParams ss;

    // r in [min:step:min+step*n] + camera offset
    ss.nDist = 6;       // 6
    ss.distMin = 0.5;   // 0.6
    ss.distStep = 0.1;  // 0.1

    ss.nAng = 24;                               // 12
    ss.angMin = angles::from_degrees(0.0);      // 0.0
    ss.angStep = angles::from_degrees(15.0);    // 30.0

    ss.nYaw = 9;                                // 5
    ss.yawMin = angles::from_degrees(-20.0);    // -10.0
    ss.yawStep = angles::from_degrees(5.0);     // 5.0

    bool bCheckGrasp = true;

    bool bCheckObs = true;

    // 2) arm position offset for computing pObs

    // center of arm (for normal grasping motion)
    double armOffsetx = 0.3;
    double armOffsety = -0.5;   // -1.1,  0.0

    // workspace radius about the center of arm
    double armLength = 0.5; // 1.5

    // core workspace radius about the center of arm (pObs = 0.0)
    double armLengthCore = 0.3; // 0.6

    // center of front body (except arm)
    double bodyOffsetx1 = -0.49 + 0.148975 + 0.27;

    // center of rear body (except arm)
    double bodyOffsetx2 = -0.49 + 0.148975 - 0.27;

    // support polygon radius about the center of body
    double bodyLength = 0.5; // 0.8

    // core support polygon radius about the center of body (pObs = 0.0)
    double bodyLengthCore = 0.335; // 0.65

    // threshold for classifying clear and occupied regions
    int mapObsThr = 1;

    // 5) candidate selection criterion

    // multiply a quadratic function to pTot (1 at diffYglob==0,
    // (1-wDiffYglob)^2 at diffYglob==M_PI) for bSortMetric==3
    double scaleDiffYglob = 0.05;
    // pTot threshold for bSortMetric==3
    double pTotThr = 0.0; // 0.5

    // 6) /base_link offset from /top_shelf for final robot pose return

    // /base_link to /base_link_front_bumper_part to /top_shelf
    // /base_link to /base_link_front_bumper_part (in new Hokuyo setting)
    double baseOffsetx = -0.49 + 0.148975;  // -0.49, -0.5, -0.3, 0.0

    // 5) flag for candidate sorting
    // 1: angle
    // 2: angle position
    // 3: pTot threshold (default)
    int bSortMetric = 3;

    ///////////////////////////
    // END PARAMETER SETTING //
    ///////////////////////////

    // 0) set search space (r, th, Y)
    // (r, th): polar coordinate with respect to the object with the center fixed at the origin and the nozzle aligned to 0 rad
    // (Y): orientation about z-axis

    // binary flag to indicate whether the probability is zero and the state
    // can be pruned; true -> non-zero
    au::grid<3, bool> bTotMax(ss.nDist, ss.nAng, ss.nYaw);

    au::grid<3, double> pGrasp(ss.nDist, ss.nAng, ss.nYaw);
    au::grid<3, double> pObs(ss.nDist, ss.nAng, ss.nYaw);

    pObs.assign(1.0);
    pGrasp.assign(1.0);
    bTotMax.assign(true);

    // generate candidate robot poses (x, y, theta) from (radius, theta, yaw)
    // around the object
    au::grid<3, Pose2D> rob;
    generateCandidatePoseSamples(object_pose, ss, rob);
    const bool publish_all_candidate_viz = false;
    if (publish_all_candidate_viz) {
        int throttle_r = 3;
        int throttle_a = 4;
        int throttle_y = 3;
        int raw_candidate_viz_id = 0;
        for (int i = 0; i < ss.nDist; ++i) {
            if (i % throttle_r != 0) continue;
            for (int j = 0; j < ss.nAng; ++j) {
                if (j % throttle_a != 0) continue;
                for (int k = 0; k < ss.nYaw; ++k) {
                    if (k % throttle_y != 0) continue;
                    Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
                    Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                    Pose2D robot_pose = poseEigen2ToSimple(T_world_robot);
                    visualizeRobot(T_world_robot, 180, "raw_candidates", raw_candidate_viz_id);
                }
            }
        }
    }

    // 1) probability of successful grasping

    if (bCheckGrasp) {
        computeGraspProbabilities(ss, rob, object_pose, pTotThr, pGrasp, bTotMax);

        const bool publish_pgrasp_map = true;
        if (publish_pgrasp_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pGrasp, bTotMax, prob_grid);
            pgrasp_map_pub_.publish(prob_grid);
        }
    }

    // 2) probability not to collide with obstacles

    if (bCheckObs) {
        // filter out poses where the robot is definitely in collision
        for (int i = 0; i < ss.nDist; ++i) {
        for (int j = 0; j < ss.nAng; ++j) {
        for (int k = 0; k < ss.nYaw; ++k) {
            if (!bTotMax(i, j, k)) {
                continue;
            }
            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
            Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;

            Pose2D rp = poseEigen2ToSimple(T_world_robot);

            if (!cc_->isValidState(rp.x, rp.y, rp.yaw)) {
                // equivalent to pObs[i][j][k] = 0;
                bTotMax(i, j, k) = false;
                auto fp_markers = cc_->getFootprintVisualization(rp.x, rp.y, rp.yaw);
                for (auto& m : fp_markers.markers) {
                    m.ns = "footprint_polygon_collisions";
                    m.id = base_footprint_viz_id++;
                }
                viz_pub_.publish(fp_markers);
            }
        }
        }
        }

        const bool publish_pobs_map = true;
        if (publish_pobs_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pObs, bTotMax, prob_grid);
            pobs_map_pub_.publish(prob_grid);
        }

        // filter out poses that are definitely in collision with the arm
        // (minimum workspace radius constraint) and update collision
        // probabilities of collision if within the maximum workspace radius
        for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
        for (int k = 0; k < ss.nYaw; k++) {
            if (!bTotMax(i, j, k)) {
                continue;
            }
            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
            Eigen::Affine2d T_mount_arm(
                    Eigen::Translation2d(armOffsetx, armOffsety));
            Eigen::Affine2d T_world_arm = T_world_mount * T_mount_arm;
            const double armx = T_world_arm.translation()[0];
            const double army = T_world_arm.translation()[1];

            double distObs = cc_->getCellObstacleDistance(armx, army);
            if (distObs < armLengthCore) {
                bTotMax(i, j, k) = false;
                Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j ,k));
                Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                visualizeRobot(T_world_robot, 0, "candidate_arm_collisions", base_collision_viz_id);
            } else if (distObs < armLength) {
                double p = sqrd((distObs - armLengthCore) / (armLength - armLengthCore));
                pObs(i, j, k) = std::min(pObs(i, j, k), p);
            }
        }
        }
        }

        // filter out poses that are definitely in collision with the base and
        // update probabilities of collision if within some threshold
        for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
        for (int k = 0; k < ss.nYaw; k++) {
            if (!bTotMax(i, j, k)) {
                continue;
            }
            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
            Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;

            double bodyx = T_world_robot.translation()[0];
            double bodyy = T_world_robot.translation()[1];

            double distObs = cc_->getCellObstacleDistance(bodyx, bodyy);
            if (distObs < bodyLengthCore) {
                bTotMax(i, j, k) = false;
                visualizeRobot(T_world_robot, 0, "candidate_body_collisions", base_collision_viz_id);
            } else if (distObs < bodyLength) {
                // pObs: quadratic function (1 at outer borders, 0 at
                // inner borders) lowest value among the patch
                // cells for current i,j,k
                double p = sqrd((distObs - bodyLengthCore) / (bodyLength - bodyLengthCore));
                pObs(i, j, k) *= std::min(pObs(i, j, k), p);
            }
        }
        }
        }
    }

    // 3) TODO: robot arm workspace limit should be included here!

    // 4) total probability

    au::grid<3, double> pTot(ss.nDist, ss.nAng, ss.nYaw);
    pTot.assign(0.0);
    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                if (bTotMax(i, j, k)) {
                    pTot(i, j, k) = pGrasp(i, j, k) * pObs(i, j, k);
                }
                else {
                    pTot(i, j, k) = 0.0;
                }
            }
        }
    }

    // 5) select maximum probability with min distance from robot
    // HEURISTICALLY, minimum difference in angular coordinate wrt object, then farthest from the origin of object
    if (bSortMetric == 1 || bSortMetric == 2) {
        double pTotMax = 1E-10; // to opt out the poses with pTot[i][j][k]==0

        for (int i = 0; i < ss.nDist; i++) {
            for (int j = 0; j < ss.nAng; j++) {
                for (int k = 0; k < ss.nYaw; k++) {
                    if (pTot(i, j, k) > pTotMax) {
                        pTotMax = pTot(i, j, k);
                    }
                }
            }
        }
        // indices for the final selection of robot pose
        int iMax = 0;
        int jMax = 0;
        int kMax = 0;
        int cntTotMax = 0;
        for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
        for (int k = 0; k < ss.nYaw; k++) {
            if (pTot(i, j, k) == pTotMax) {
                bTotMax(i, j, k) = true;
                iMax = i;
                jMax = j;
                kMax = k;
                cntTotMax++;
            } else {
                bTotMax(i, j, k) = false;
                Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
                Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                visualizeRobot(T_world_robot, 0, "base_candidates_validity_reject", base_validityreject_viz_id);
            }
        }
        }
        }

        assert(cntTotMax >= 0);

        ROS_INFO("    cntTotMax: %d", cntTotMax);

        if (cntTotMax == 0) {
            ROS_WARN("    No candidate pose was found!");
            return false;
        } else {
            // if more than one candidate poses are selected
            // (we can select poses with pTot higher than a THRESHOLD)
            // a) sorting by difference of angular coordinates for current and desired poses
            // ASSUME? backward driving is allowed for husky
            // HEURISTIC: minimize angle difference between robot orientation (robY) and robot motion direction (angO2Rcur)
            double angO2Rcur = atan2(robot_pose.y - object_pose.y, robot_pose.x - object_pose.x);	// current angular coordinate from object to robot
            double angO2Rdes, angO2Rerr, angO2RerrMin = M_PI;
            cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
            for (int i = 0; i < ss.nDist; i++)
                for (int j = 0; j < ss.nAng; j++)
                    for (int k = 0; k < ss.nYaw; k++) {
                        if (bTotMax(i, j, k) == true) {
                            angO2Rdes = object_pose.yaw + ss.angMin + ss.angStep * j;// desired angular coordinate from object to robot
                            angO2Rerr = angles::normalize_angle(angO2Rdes - angO2Rcur);

                            if (fabs(angO2Rerr) <= angO2RerrMin) {
                                angO2RerrMin = fabs(angO2Rerr);
                                iMax = i;
                                jMax = j;
                                kMax = k;
                                cntTotMax++;
                            }
                            else {
                                bTotMax(i, j, k) = false;
                                // /top_shelf pose
                                double robxf = rob(i, j, k).x;
                                double robyf = rob(i, j, k).y;
                                double robYf = rob(i, j, k).yaw;
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
            // update bTotMax(i, j, k)
            for (int i = 0; i < ss.nDist; i++)
                for (int j = 0; j < ss.nAng; j++)
                    for (int k = 0; k < ss.nYaw; k++)
                        if (bTotMax(i, j, k) == true) {
                            angO2Rdes = object_pose.yaw + ss.angMin + ss.angStep * j;
                            angO2Rerr = angles::normalize_angle(angO2Rdes - angO2Rcur);
                            if (fabs(angO2Rerr) != angO2RerrMin) {
                                bTotMax(i, j, k) = false;
                                // /top_shelf pose
                                double robxf = rob(i, j, k).x;
                                double robyf = rob(i, j, k).y;
                                double robYf = rob(i, j, k).yaw;
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
                    for (int i = 0; i < ss.nDist; i++)
                        for (int j = 0; j < ss.nAng; j++)
                            for (int k = 0; k < ss.nYaw; k++) {
                                if (bTotMax(i, j, k) == true) {
                                    xysqerr = sqrd(rob(i, j, k).x - robot_pose.x)
                                            + sqrd(rob(i, j, k).y - robot_pose.y);

                                    if (xysqerr <= xysqerrMin) {
                                        xysqerrMin = xysqerr;
                                        iMax = i;
                                        jMax = j;
                                        kMax = k;
                                        cntTotMax++;
                                    }
                                    else {
                                        bTotMax(i, j, k) = false;
                                    }
                                    // /top_shelf pose
                                    double robxf = rob(i, j, k).x;
                                    double robyf = rob(i, j, k).y;
                                    double robYf = rob(i, j, k).yaw;
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

                    // OPTIONAL: update bTotMax(i, j, k)
                    for (int i = 0; i < ss.nDist; i++)
                        for (int j = 0; j < ss.nAng; j++)
                            for (int k = 0; k < ss.nYaw; k++)
                                if (bTotMax(i, j, k) == true) {
                                    xysqerr = sqrd(rob(i, j, k).x - robot_pose.x)
                                            + sqrd(rob(i, j, k).y - robot_pose.y);
                                    if (xysqerr != xysqerrMin) {
                                        bTotMax(i, j, k) = false;
                                        // /top_shelf pose
                                        double robxf = rob(i, j, k).x;
                                        double robyf = rob(i, j, k).y;
                                        double robYf = rob(i, j, k).yaw;
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
        ROS_INFO("    Object Pose (initial): %f %f %f", object_pose.x, object_pose.y, angles::normalize_angle(object_pose.yaw)*180/M_PI);
        ROS_INFO("    Robot Pose (intial):   %f %f %f", robot_pose.x, robot_pose.y, angles::normalize_angle(robot_pose.yaw)*180/M_PI);
        // index order regarding to sorting priority
        // 	for (int i=0; i<ss.nDist; i++)
        for (int i = ss.nDist - 1; i >= 0; i--)
            for (int j = 0; j < ss.nAng; j++)
                // 			for (int k=0; k<ss.nYaw; k++)
                for (int k = ss.nYaw - 1; k >= 0; k--)
                    if (bTotMax(i, j, k)) {
                        // /top_shelf pose
                        double robxf = rob(i, j, k).x;
                        double robyf = rob(i, j, k).y;
                        double robYf = rob(i, j, k).yaw;
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
                        candidate_base_pose.pose.position.z = robot_pose.x;

                        tf::Quaternion robqf = tf::createQuaternionFromRPY(0.0, 0.0, robYf);
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
    }
    else if (bSortMetric == 3) {
        // candidate preference by difference of angular coordinates for current
        // and desired poses
        // HEURISTIC: minimize angle difference between robot orientation (robY)
        // and robot motion direction (angO2Rcur)
        double rob2objY = atan2(object_pose.y - robot_pose.y, object_pose.x - robot_pose.x);  // angular coordinate of displacement from robot to object
        double diffYglob;
        for (int i = 0; i < ss.nDist; i++) {
            for (int j = 0; j < ss.nAng; j++) {
                for (int k = 0; k < ss.nYaw; k++) {
                    if (bTotMax(i, j, k)) {
                        diffYglob = angles::normalize_angle(rob(i, j, k).yaw - rob2objY);
                        // quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
//                        pTot(i, j, k) *= std::pow(1 - (scaleDiffYglob * fabs(diffYglob)/M_PI), 2.0);
                        // quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
                        pTot(i, j, k) *= sqrd(1 - (scaleDiffYglob * fabs(diffYglob) / M_PI));
                        pTot(i, j, k) = std::max(pTot(i, j, k), pTotThr);
                    }
                }
            }
        }

        // sort candidates with respect to pTot
        std::vector<RepositionBaseCandidate::candidate> cands;
        for (int j = 0; j < ss.nAng; j++) {
        for (int i = ss.nDist - 1; i >= 0; i--) {
        for (int k = ss.nYaw - 1; k >= 0; k--) {
            if (bTotMax(i, j, k)) {
                RepositionBaseCandidate::candidate cand;
                cand.i = i;
                cand.j = j;
                cand.k = k;
                cand.pTot = pTot(i, j, k);
                cands.push_back(cand);
            }
        }
        }
        }
        std::sort(cands.begin(), cands.end());

        // 6-2) generate final desired robot poses with maximum pTot
        ROS_INFO("Reposition Base Command Result:");
        ROS_INFO("    Object Pose (initial): %f %f %f", object_pose.x, object_pose.y, angles::normalize_angle(object_pose.yaw)*180/M_PI);
        ROS_INFO("    Robot Pose (intial):   %f %f %f", robot_pose.x, robot_pose.y, angles::normalize_angle(robot_pose.yaw)*180/M_PI);

        int cntTotThr = 0; // number of candidate poses with pTot higher than threshold
        for (size_t cidx = 0; cidx < cands.size(); ++cidx) {
            const auto& cand = cands[cidx];
            if (cand.pTot >= pTotThr) {
                int i = cand.i;
                int j = cand.j;
                int k = cand.k;

                Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
                Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                ROS_DEBUG("  Pose Candidate %3zu: %0.3f %0.3f %0.3f (i:%d j:%d k:%d), pTot: %f",
                        cidx,
                        T_world_robot.translation()[0],
                        T_world_robot.translation()[1],
                        angles::to_degrees(angles::normalize_angle(rob(i, j, k).yaw)),
                        i, j, k, cand.pTot);

                geometry_msgs::PoseStamped candidate_pose;

                candidate_pose.header.frame_id = map.header.frame_id;
                candidate_pose.header.stamp = ros::Time::now();

                Eigen::Affine3d T_world_robot_3d = poseEigen2ToEigen3(T_world_robot);
                tf::poseEigenToMsg(T_world_robot_3d, candidate_pose.pose);

                candidate_base_poses.push_back(candidate_pose);

                visualizeRobot(T_world_robot, (cand.pTot * 240) - 120, "base_probable_candidates", base_probcandidates_viz_id);
                cntTotThr++;
            }
        }

        if (candidate_base_poses.empty()) {
            ROS_WARN("No probable candidate poses higher than a threshold!");
            ROS_WARN("Number of valid candidates: %zu", cands.size());
            return false;
        } else {
            ROS_INFO("Number of valid candidates: %zu", cands.size());
            ROS_INFO("Number of probable candidates: %zu", candidate_base_poses.size());
        }
    }

    return true;
}

/// \brief Evaluate the probabilities of a successful grasp for all candidates
///
/// This function computes probabilities based solely off the relative
/// configuration of the robot and the object. The embedded policies for
/// grasping with a right- side mounted arm are as follows:
///
/// * position: The object should be just within left-hand side of the robot.
///   At close distances, exclude more poses; at far distances, include more
///   poses.
/// * orientation: The object handle should be directed to the right of the
///   robot. At close distances, allow more deviation from the nominal
///   heading offset; at far distances, be more aggressive about excluding
///   poses.
void RepositionBaseExecutor::computeGraspProbabilities(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    const Pose2D& obj,
    double pTotThr,
    au::grid<3, double>& pGrasp,
    au::grid<3, bool>& bTotMax)
{
    int nDist = rob.size(0);
    int nAng = rob.size(1);
    int nYaw = rob.size(2);

    ///////////////////
    // CONFIGURATION //
    ///////////////////

    // Define cone in front in front of the robot within which the object is
    // graspable
//  double secSide[2] = { 0.0, angles::from_degrees(20.0) };
//  double secSide[2] = { 0.0, angles::from_degrees(45.0) };
    double secSide[2] = { angles::from_degrees(-5.0), angles::from_degrees(40.0) };

    // partition radial samples into zones for distance-dependent allowable
    // heading differences between the robot and the object
    int secDist[2] = { 1 * nDist / 3, 2 * nDist / 3 };

    // distance-dependent allowable heading offset range
    double secAngYaw[2];

    // distance-dependent "most desirable" heading offset
    double bestAngYaw;

    // most desirable robot-object distance
    double bestDist = 0.70;

    // pGrasp: quadratic function (1 at bestDist, (1 - scalepGraspDist)^2 at borders)
//    double scalepGraspDist = 0.1;
    double scalepGraspDist = 0.05;

    // pGrasp: quadratic function (1 at bestAngYaw, (1 - scalepGraspAngYaw)^2 at borders)
//    double scalepGraspAngYaw = 0.1;
    double scalepGraspAngYaw = 0.05;

    ///////////////////////
    // END CONFIGURATION //
    ///////////////////////

    for (int i = 0; i < nDist; i++) {
        // c) set acceptable object orientation range
        if (i < secDist[0]) {
            secAngYaw[0] = angles::from_degrees(45.0);  // 15,  15
            secAngYaw[1] = angles::from_degrees(130.0); // 90, 100
            bestAngYaw   = angles::from_degrees(60.0);  // 65,  65
        } else if (i < secDist[1]) {
            secAngYaw[0] = angles::from_degrees(45.0);  // 45, 15
            secAngYaw[1] = angles::from_degrees(130.0); // 90, 90
            bestAngYaw   = angles::from_degrees(60.0);  // 45, 45
        } else { // if (i >= secDist[1])
            secAngYaw[0] = angles::from_degrees(45.0);  // 15, 15
            secAngYaw[1] = angles::from_degrees(130.0); // 90, 30
            bestAngYaw   = angles::from_degrees(60.0);  // 50, 15
        }

        for (int j = 0; j < nAng; ++j) {
            for (int k = 0; k < nYaw; ++k) {
                // angular coordinate of a vector from robot position to object
                // position (not orientation)
                double rob2obj = atan2(obj.y - rob(i, j, k).y, obj.x - rob(i, j, k).x);
                double diffAng = angles::normalize_angle(rob2obj - rob(i, j, k).yaw);

                // filter out all candidate poses that are not facing the object
                // within the above-defined thresholds
                if (diffAng >= secSide[0] && diffAng < secSide[1]) {
                    // difference in heading between robot and object
                    const double diffY =
                            angles::normalize_angle(obj.yaw - rob(i, j, k).yaw);

                    // maximum allowable angular distance away from most
                    // desirable angle configuration
                    const double diffYMax = std::max(
                            fabs(secAngYaw[0] - bestAngYaw),
                            fabs(secAngYaw[1] - bestAngYaw));

                    // maximum allowable linear distance away from most
                    // desirable distance configuration
                    const double diffDistMax = std::max(
                            fabs(ss.distMin - bestDist),
                            fabs(ss.distMin + ss.distStep * (nDist - 1) - bestDist));

                    // filter out all candidate poses where the difference in
                    // heading between the robot and the object lies outside the
                    // acceptable range
                    if (diffY >= secAngYaw[0] && diffY <= secAngYaw[1]) {
                        // higher probability around diffY == bestAngYaw
                        // pGrasp: quadratic function
                        // (1 at bestAngYaw, (1 - scalepGrasp)^2 at borders)
//                        pGrasp(i, j, k) = sqrd((diffYMax - fabs(diffY - bestAngYaw) * scalepGrasp) / diffYMax);
//                        pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);

                        // higher probability around diffY == bestAngYaw and
                        // diffDist == bestDist
                        // pGrasp: quadratic function
                        // (1 at bestAngYaw, (1 - scalepGrasp)^2 at borders)
                        pGrasp(i, j, k) =
                                sqrd((diffYMax - fabs(diffY - bestAngYaw) * scalepGraspAngYaw) / diffYMax) *
                                sqrd((diffDistMax - fabs(ss.distMin + ss.distStep * i - bestDist) * scalepGraspDist) / diffDistMax),
                        pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);
                    } else {
                        bTotMax(i, j, k) = false;
                    }
                } else {
                    bTotMax(i, j, k) = false;
                }
            }
        }
    }
}

/// \brief Construct an occupancy grid for visualizing probabilities.
///
/// \param map existing occupancy map for determining bounds, position, and res
/// \param ss The search space parameterization; the following probability maps
///     must have dimensions corresponding to the discretization set in this
///     parameterization
/// \param obj_pose Pose of the object in the world frame
/// \param prob probabilites over the search space
/// \param valid flags indicating whether probabilities may be pruned
/// \param grid Output probability grid
void RepositionBaseExecutor::projectProbabilityMap(
    const nav_msgs::OccupancyGrid& map,
    const SearchSpaceParams& ss,
    const Pose2D& obj_pose,
    const au::grid<3, double>& prob,
    const au::grid<3, bool>& valid,
    nav_msgs::OccupancyGrid& grid)
{
    grid.header.frame_id = map.header.frame_id;
    grid.header.stamp = ros::Time(0);
    grid.info = map.info;
    grid.data.resize(map.data.size(), 0);

    for (int x = 0; x < grid.info.width; ++x) {
        for (int y = 0; y < grid.info.height; ++y) {
            double wx = grid.info.origin.position.x + x * grid.info.resolution + 0.5 * grid.info.resolution;
            double wy = grid.info.origin.position.y + y * grid.info.resolution + 0.5 * grid.info.resolution;
            double rx = wx - obj_pose.x;
            double ry = wy - obj_pose.y;
            double r = std::sqrt(sqrd(rx) + sqrd(ry));
            double th = angles::normalize_angle_positive(std::atan2(ry, rx));
            int cr = (int)((r - ss.distMin) / ss.distStep);
            int cth = (int)(angles::normalize_angle_positive(th - obj_pose.yaw) / ss.angStep + 0.5);
            if (cth == ss.nAng) {
                cth = 0;
            }
            if (cr < 0 || cr >= ss.nDist) {
                // out of range of the sample space
                if (cth < 0 || cth >= ss.nAng) {
                    ROS_ERROR("Invalid discrete angle %d", cth);
                }
                grid.data[y * grid.info.width + x] = 0;
            }
            else {
                if (cth < 0 || cth >= ss.nAng) {
                    ROS_ERROR("Invalid discrete angle %d", cth);
                }
                int valid_count = 0;
                double d = 0.0;
                for (int k = 0; k < prob.size(2); ++k) {
                    if (valid(cr, cth, k)) {
                        ++valid_count;
                        d += prob(cr, cth, k);
                    }
                }
                d = valid_count ? d /= valid_count : 0.0;
                grid.data[y * grid.info.width + x] = (std::int8_t)(100.0 * d);
            }
        }
    }
}

void RepositionBaseExecutor::generateCandidatePoseSamples(
    const Pose2D& obj_pose,
    const SearchSpaceParams& params,
    au::grid<3, Pose2D>& poses) const
{
    Eigen::Affine2d T_world_object = poseSimpleToEigen2(obj_pose);
    poses.resize(params.nDist, params.nAng, params.nYaw);
    for (int i = 0; i < params.nDist; ++i) {
        for (int j = 0; j < params.nAng; ++j) {
            for (int k = 0; k < params.nYaw; ++k) {
                double r = params.distMin + i * params.distStep;
                double th = params.angMin + j * params.angStep;
                double yaw = th - M_PI - (params.yawMin + k * params.yawStep);

                Eigen::Affine2d T_object_robot =
                        Eigen::Translation2d(r * cos(th), r * sin(th)) *
                        Eigen::Rotation2Dd(yaw);
                Eigen::Affine2d T_world_robot = T_world_object * T_object_robot;
                poses(i, j, k) = poseEigen2ToSimple(T_world_robot);
                ROS_DEBUG("sample(%0.3f, %0.3f, %0.3f) -> (%0.3f, %0.3f, %0.3f)", r, th, yaw, poses(i, j, k).x, poses(i, j, k).y, poses(i, j, k).yaw);
            }
        }
    }
}

bool RepositionBaseExecutor::computeRobPoseExhaustive(
    const nav_msgs::OccupancyGrid& map,
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose,
    std::vector<geometry_msgs::PoseStamped>& candidate_base_poses)
{
    return false;

    double objx, objy, objY, robx0, roby0, robY0;
    Eigen::Affine2d robot_pose_planar;
    computeRobotPlanarPose(robot_pose, robot_pose_planar);
    Pose2D object_pose_planar = poseEigen3ToSimpleGascan(object_pose);
    robx0 = robot_pose_planar.translation()[0];
    roby0 = robot_pose_planar.translation()[1];
    robY0 = Eigen::Rotation2Dd(0.0).fromRotationMatrix(robot_pose_planar.rotation()).angle();
    objx = object_pose_planar.x;
    objy = object_pose_planar.y;
    objY = object_pose_planar.yaw;

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

    SearchSpaceParams ss;

    // 0) search space
    // r in [0.5:0.1:1.0] + camera offset
    ss.nDist = 6;       // 6
    ss.distMin = 0.5;   // 0.6
    ss.distStep = 0.1;  // 0.1

    ss.nAng = 12;                               // 12
    ss.angMin = 0.0;                            // 0
    ss.angStep = angles::from_degrees(30.0);    // 30

    ss.nYaw = 9;                                // 5
    ss.yawMin = angles::from_degrees(-20.0);    // -10
    ss.yawStep = angles::from_degrees(5.0);     // 5

    // 1) grasp achievable zone 	// TODO: tune these parameters
    int secDist[] = { (int)(ss.nDist / 3), (int)(ss.nDist * 2 / 3) };	// divide zone by distance (for acceptable object orientation setting)
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

    double pTot[ss.nDist][ss.nAng][ss.nYaw];
    double pGrasp[ss.nDist][ss.nAng][ss.nYaw];
    double pObs[ss.nDist][ss.nAng][ss.nYaw];
    bool bTotMax[ss.nDist][ss.nAng][ss.nYaw];
    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                pTot[i][j][k] = 0.0;
                pGrasp[i][j][k] = 1.0;
                pObs[i][j][k] = 1.0;
                bTotMax[i][j][k] = true;	// used when 1) check candidate validity before IK (process 1~4)
                                            //			 2) select the one with maximum pTot (process 5~6)
            }
        }
    }

    double robz0 = 0.0, robP0 = 0.0, robR0 = 0.0;
    double robxf = robx0, robyf = roby0, robzf = robz0, robYf = robY0, robPf = robP0, robRf = robR0; // final (computed) robot pose

    double robx[ss.nDist][ss.nAng][ss.nYaw], roby[ss.nDist][ss.nAng][ss.nYaw], robY[ss.nDist][ss.nAng][ss.nYaw];	// candidate robot poses
    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                robx[i][j][k] = objx + cos(objY + ss.angMin + ss.angStep * j) * (ss.distMin + ss.distStep * i);
                roby[i][j][k] = objy + sin(objY + ss.angMin + ss.angStep * j) * (ss.distMin + ss.distStep * i);
                // angular coordinate of x-axis (frontal direction) of the robot with respect to global frame
                robY[i][j][k] = objY + (ss.angMin + ss.angStep * j) - M_PI - (ss.yawMin + ss.yawStep * k);
            }
        }
    }

    // 1) probability of successful grasping

    if (bCheckGrasp) {
        // heuristic probability of successful grasping
        // a) position: the object should be within left-hand side of the robot and 45 deg angle of view
        // exception: exclude poses at close distance, include ones in right-hand side at far distance
        // b) orientation: the object handle should be directed to the right of the robot
        // c) exception: allow more deviation at close distance, reject most of deviations at far distance

        for (int i = 0; i < ss.nDist; i++) {
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

            for (int j = 0; j < ss.nAng; j++) {
                for (int k = 0; k < ss.nYaw; k++) {
                    // a) object position at left-hand side to the robot
                    double rob2obj = atan2(objy - roby[i][j][k], objx - robx[i][j][k]);	// angular coordinate of a vector from robot position to object position (not orientation)
                    double diffAng = angles::normalize_angle(rob2obj - robY[i][j][k]);
                    double diffAngMax = fabs(ss.yawMin - secSide[0]);
                    // b) object handle orientation to the right of the robot
// 						double diffY = angles::normalize_angle( objY-robY[i][j][k] );
// 						double diffYMax = std::max( fabs(secAngYaw[0]-bestAngYaw), fabs(secAngYaw[1]-bestAngYaw) );
// 						double diffYMax = M_PI/1.0;
                    double diffY = angles::normalize_angle(objY - robY[i][j][k]);
                    double diffYMax = M_PI / 2.0;
                    double diffDistMax = std::max(fabs(ss.distMin - bestDist),
                            fabs(ss.distMin + ss.distStep * (ss.nDist - 1) - bestDist));
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
// 							pGrasp[i][j][k] = std::max( std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0) * std::pow( (diffDistMax-fabs(ss.distMin+distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0), pTotThr);	// pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                        }
                    }
// 					pGrasp[i][j][k] = std::max( std::pow( std::max( std::min(diffAng-secSide[0], 0.0)/diffAngMax + 1.0, 0.0), 1.0)
// 												* std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0)
// 												* std::pow( (diffDistMax-fabs(ss.distMin+ss.distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0)
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
                                            (diffDistMax - fabs(ss.distMin + ss.distStep * i - bestDist) * scalepGraspDist)
                                                    / (diffDistMax), 2.0), pTotThr); // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                }
            }
        }
    }

    // 2) probability not to collide with obstacles
    // depends on distance to obstacles considering the position offset wrt orientation

    // 3) robot arm workspace limit should be included here!

    if (bCheckWork) {
        // check for inverse kinematics and arm planning (if object is in reachable range and within angle of view)
        for (int i = 0; i < ss.nDist; i++) {
            for (int j = 0; j < ss.nAng; j++) {
                for (int k = 0; k < ss.nYaw; k++) {
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
                        } else {
                            //checkIK success!
                        }
                    }
                }
            }
        }
    }

    // 4) total probability

    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                if (bTotMax[i][j][k] == true) {
                    pTot[i][j][k] = pGrasp[i][j][k] * pObs[i][j][k];
                }
                else {
                    pTot[i][j][k] = 0.0;
                }
            }
        }
    }

    // 5) select maximum probability with min distance from robot
    // HEURISTICALLY, minimum difference in angular coordinate wrt object, then farthest from the origin of object
    if (bSortMetric == 1 || bSortMetric == 2) {
        double pTotMax = 1E-10;		// to opt out the poses with pTot[i][j][k]==0

        for (int i = 0; i < ss.nDist; i++) {
            for (int j = 0; j < ss.nAng; j++) {
                for (int k = 0; k < ss.nYaw; k++) {
                    if (pTot[i][j][k] > pTotMax) {
                        pTotMax = pTot[i][j][k];
                    }
                }
            }
        }
        int iMax = 0, jMax = 0, kMax = 0;		// indices for the final selection of robot pose
        int cntTotMax = 0;
        for (int i = 0; i < ss.nDist; i++)
            for (int j = 0; j < ss.nAng; j++)
                for (int k = 0; k < ss.nYaw; k++)
                    if (pTot[i][j][k] == pTotMax) {
                        bTotMax[i][j][k] = true;
                        iMax = i;
                        jMax = j;
                        kMax = k;
                        cntTotMax++;
                    } else {
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
            for (int i = 0; i < ss.nDist; i++)
                for (int j = 0; j < ss.nAng; j++)
                    for (int k = 0; k < ss.nYaw; k++) {
                        if (bTotMax[i][j][k] == true) {
                            angO2Rdes = objY + ss.angMin + ss.angStep * j;// desired angular coordinate from object to robot
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
            for (int i = 0; i < ss.nDist; i++)
                for (int j = 0; j < ss.nAng; j++)
                    for (int k = 0; k < ss.nYaw; k++)
                        if (bTotMax[i][j][k] == true) {
                            angO2Rdes = objY + ss.angMin + ss.angStep * j;
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
                if (cntTotMax > 1) { // if more than one candidate poses are still selected
                    // b) sorting by distance from current and desired positions
                    double xysqerr, xysqerrMin = 1E10;		// distance from current to desired robot position
                    cntTotMax = 0;		// reinitialization for angular coordinate heuristic sorting
                    for (int i = 0; i < ss.nDist; i++)
                        for (int j = 0; j < ss.nAng; j++)
                            for (int k = 0; k < ss.nYaw; k++) {
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
                    for (int i = 0; i < ss.nDist; i++)
                        for (int j = 0; j < ss.nAng; j++)
                            for (int k = 0; k < ss.nYaw; k++)
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
        // 	for (int i=0; i<ss.nDist; i++)
        for (int i = ss.nDist - 1; i >= 0; i--)
            for (int j = 0; j < ss.nAng; j++)
                // 			for (int k=0; k<ss.nYaw; k++)
                for (int k = ss.nYaw - 1; k >= 0; k--)
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
        for (int i = 0; i < ss.nDist; i++)
            for (int j = 0; j < ss.nAng; j++)
                for (int k = 0; k < ss.nYaw; k++)
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
        for (int j = 0; j < ss.nAng; j++) {
            for (int i = ss.nDist - 1; i >= 0; i--) {
                for (int k = ss.nYaw - 1; k >= 0; k--) {
                    if (bTotMax[i][j][k] == true) {
                        cand.i = i;
                        cand.j = j;
                        cand.k = k;
                        cand.pTot = pTot[i][j][k];
                        cands.push_back(cand);
                    }
                }
            }
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
        } else {
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

Pose2D RepositionBaseExecutor::poseEigen3ToSimple(
    const Eigen::Affine3d& pose) const
{
    Pose2D out;
    tf::Transform t;
    tf::transformEigenToTF(pose, t);
    double y, p, r;
    t.getBasis().getEulerYPR(y, p, r);
    out.x = pose.translation()[0];
    out.y = pose.translation()[1];
    out.yaw = y;
    return out;
}

Pose2D RepositionBaseExecutor::poseEigen2ToSimple(
    const Eigen::Affine2d& pose) const
{
    Pose2D p;
    p.x = pose.translation()[0];
    p.y = pose.translation()[1];
    Eigen::Rotation2Dd r(0.0);
    r.fromRotationMatrix(pose.rotation());
    p.yaw = r.angle();
    return p;
}

Eigen::Affine2d RepositionBaseExecutor::poseSimpleToEigen2(
    const Pose2D& pose) const
{
    return Eigen::Translation2d(pose.x, pose.y) * Eigen::Rotation2Dd(pose.yaw);
}

Eigen::Affine2d RepositionBaseExecutor::poseEigen3ToEigen2(
    const Eigen::Affine3d& pose) const
{
    tf::Transform t;
    tf::transformEigenToTF(pose, t);
    double y, p, r;
    t.getBasis().getEulerYPR(y, p, r);
    return Eigen::Translation2d(pose.translation()[0], pose.translation()[1]) *
            Eigen::Rotation2Dd(y);
}

Eigen::Affine3d RepositionBaseExecutor::poseSimpleToEigen3(
    const Pose2D& pose,
    double z, double R, double P) const
{
    const double x = pose.x;
    const double y = pose.y;
    const double Y = pose.yaw;
    return Eigen::Translation3d(x, y, z) *
            Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX());
}

Eigen::Affine3d RepositionBaseExecutor::poseEigen2ToEigen3(
    const Eigen::Affine2d& pose,
    double z, double R, double P) const
{
    const double x = pose.translation()[0];
    const double y = pose.translation()[1];
    const double Y = Eigen::Rotation2Dd(0.0).fromRotationMatrix(pose.rotation()).angle();
    return Eigen::Translation3d(x, y, z) *
            Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX());
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

Pose2D RepositionBaseExecutor::poseEigen3ToSimpleGascan(
    const Eigen::Affine3d& object_pose) const
{
    Pose2D gascan_pose = poseEigen3ToSimple(object_pose);
    // M_PI / 2 offset due to definition of object frame in new mesh file
    gascan_pose.yaw = angles::normalize_angle(gascan_pose.yaw - 0.5 * M_PI);
    return gascan_pose;
}

bool RepositionBaseExecutor::tryFeasibleArmCheck(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose) const
{
    Eigen::Affine2d rp;
    computeRobotPlanarPose(robot_pose, rp);

    Pose2D op = poseEigen3ToSimpleGascan(object_pose);

    // check for inverse kinematics and arm planning (if object is in reachable range and within angle of view)
    // TODO: a more general than (distMin + (nDist-1)*ss.distStep) determined in computeRobPose()
//    double secDist[2] = { 0.3, 1.5 };
    double secDist[2] = { 0.5, 1.0 };
    Eigen::Vector2d dp = Eigen::Vector2d(op.x, op.y) - rp.translation();

    Eigen::Rotation2Dd robot_yaw(0.0); robot_yaw.fromRotationMatrix(rp.rotation());
    Eigen::Rotation2Dd object_yaw(op.yaw);
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
    if (!rcta::ReconnectActionClient(
            move_arm_command_client_,
            move_arm_command_action_name_,
            ros::Rate(10),
            ros::Duration(5.0)))
    {
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
    double x,
    double y,
    double yaw,
    int hue,
    const std::string& ns,
    int& id)
{
    return visualizeRobot(poseFrom2D(x, y, yaw), hue, ns, id);
}

void RepositionBaseExecutor::visualizeRobot(
    const Eigen::Affine2d& pose,
    int hue,
    const std::string& ns,
    int& id)
{
    return visualizeRobot(poseEigen2ToEigen3(pose), hue, ns, id);
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
    const std::vector<std::string>& link_names = { "talon", "torso_link0" }; //robot_model_->getLinkModelNames();
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    robot_state_->getRobotMarkers(ma, link_names, color, ns, ros::Duration(0), false);
    for (auto& marker : ma.markers) {
        marker.id = id++;
    }
    viz_pub_.publish(ma);
}
