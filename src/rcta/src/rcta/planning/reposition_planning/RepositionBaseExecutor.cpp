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
    pgrasp_map_pub_(),
    pobs_map_pub_(),
    pgrasp_exhaustive_map_pub_(),
    listener_(),
    viz_(),
    action_name_("reposition_base_command"),
    as_(),
    rml_(),
    robot_model_(),
    manip_group_(nullptr),
    manip_name_(),
    m_scene_monitor(),
    camera_view_frame_(),
    T_mount_robot_(),
    cc_(),
    m_arm_offset_x(0.0),
    m_arm_offset_y(0.0),
    m_arm_length(0.0),
    m_arm_length_core(0.0),
    m_body_length(0.0),
    m_body_length_core(0.0),
    attached_markers_(),
    m_max_grasp_samples(0),
    move_arm_command_client_(),
    move_arm_command_action_name_("move_arm"),
    move_arm_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    move_arm_command_result_(),
    robot_pose_world_frame_(),
    current_goal_(),
    status_(RepositionBaseExecutionStatus::INVALID),
    last_status_(RepositionBaseExecutionStatus::INVALID)
{
    sbpl::viz::set_visualizer(&viz_);
}

RepositionBaseExecutor::~RepositionBaseExecutor()
{
    if (sbpl::viz::visualizer() == &viz_) {
        sbpl::viz::unset_visualizer();
    }
}

bool RepositionBaseExecutor::initialize()
{
    //////////////////////////////////////////////////////////////////////////
    // This could maybe be refactoreds, since there is a lot of commonality //
    // with GraspObjectExecutor                                             //
    //////////////////////////////////////////////////////////////////////////

    camera_view_frame_ = "asus_rgb_optical_frame";

    rml_.reset(new robot_model_loader::RobotModelLoader);
    robot_model_ = rml_->getModel();
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model");
        return false;
    }

    if (!robot_model_->hasLinkModel(camera_view_frame_)) {
        ROS_ERROR("No link '%s' found in robot model", camera_view_frame_.c_str());
        return false;
    }

    auto transformer = boost::shared_ptr<tf::Transformer>(new tf::TransformListener);
    m_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(
            rml_, transformer));
    auto update_fn = boost::bind(&RepositionBaseExecutor::processSceneUpdate, this, _1);
    m_scene_monitor->addUpdateCallback(update_fn);
    m_scene_monitor->startStateMonitor();

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

    if (!initGraspPlanner(ph_)) {
        ROS_ERROR("Failed to initialize grasp planner");
    }

    ///////////////////////////////////////////
    // RepositionBaseExecutor-specific stuff //
    ///////////////////////////////////////////

    pgrasp_map_pub_ = ph_.advertise<nav_msgs::OccupancyGrid>("pgrasp_map", 1);
    pobs_map_pub_ = ph_.advertise<nav_msgs::OccupancyGrid>("pobs_map", 1);
    pgrasp_exhaustive_map_pub_ = ph_.advertise<nav_msgs::OccupancyGrid>("pgrasp_exhaustive_map", 1);

    move_arm_command_client_.reset(new MoveArmActionClient(move_arm_command_action_name_, false));
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

    as_->registerGoalCallback(boost::bind(&RepositionBaseExecutor::goalCallback, this));
    as_->registerPreemptCallback(boost::bind(&RepositionBaseExecutor::preemptCallback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    if (!downloadMarkerParameters()) {
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

    m_arm_offset_x = 0.3;
    m_arm_offset_y = -0.5;   // -1.1,  0.0

    // workspace radius about the center of arm
    m_arm_length = 0.5; // 1.5

    // core workspace radius about the center of arm (pObs = 0.0)
    m_arm_length_core = 0.3; // 0.6

    // support polygon radius about the center of body
    m_body_length = 0.5; // 0.8

    // core support polygon radius about the center of body (pObs = 0.0)
    m_body_length_core = 0.335; // 0.65

    double baseOffsetx = -0.49 + 0.148975;  // -0.49, -0.5, -0.3, 0.0
    T_mount_robot_ = Eigen::Translation2d(baseOffsetx, 0.0);

    return true;
}

bool RepositionBaseExecutor::initGraspPlanner(ros::NodeHandle& nh)
{
    ros::NodeHandle grasping_nh(nh, "grasping");
    if (!m_grasp_planner.init(grasping_nh)) {
        return false;
    }

    // read in max grasps
    if (!msg_utils::download_param(nh, "max_grasp_candidates", m_max_grasp_samples) || m_max_grasp_samples < 0) {
        ROS_ERROR("Failed to retrieve 'max_grasp_candidates' from the param server or 'max_grasp_candidates' is negative");
        return false;
    }

    return true;
}

bool RepositionBaseExecutor::downloadGraspingParameters(ros::NodeHandle& nh)
{
    return true;
}

const moveit::core::RobotState& RepositionBaseExecutor::currentRobotState() const
{
    assert(m_scene_monitor && m_scene_monitor->getPlanningScene());
    return m_scene_monitor->getPlanningScene()->getCurrentState();
}

void RepositionBaseExecutor::processSceneUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    switch (type) {
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE: {

    }   break;
    case planning_scene_monitor::PlanningSceneMonitor::UPDATE_TRANSFORMS: {

    }   break;
    }
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
            // NOTE: special for the gascan
            Pose2D gascan_pose = poseEigen3ToSimpleGascan(m_obj_pose);

            const auto& map = current_goal_->map;

            std::vector<geometry_msgs::PoseStamped> candidate_base_poses;

            if (tryFeasibleArmCheck(m_rob_pose, m_obj_pose)) {
                int err = checkFeasibleMoveToPregraspTrajectory(m_rob_pose, m_obj_pose);
                ROS_DEBUG("arm plan result: %d", err);
                if (err) {
                    int v_id = 0;
                    visualizeRobot(m_rob_pose, 0, "base_checkIKPLAN_fail", v_id);
                } else {
                    // you can grasp it now!
                    rcta_msgs::RepositionBaseCommandResult result;
                    result.result = rcta_msgs::RepositionBaseCommandResult::SUCCESS;

                    geometry_msgs::PoseStamped p;
                    tf::poseEigenToMsg(m_rob_pose, p.pose);
                    p.header.frame_id = robot_model_->getModelFrame();
                    candidate_base_poses.push_back(p);
                    result.candidate_base_poses = candidate_base_poses;
                    as_->setSucceeded(result);
                    status_ = RepositionBaseExecutionStatus::IDLE;
                }
            } else if (computeRobPose(
                    map,
                    poseEigen3ToSimple(m_rob_pose),
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
                    m_rob_pose,
                    m_obj_pose,
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
                // this pose is same with the initial base pose
                result.candidate_base_poses = candidate_base_poses;
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

void RepositionBaseExecutor::goalCallback()
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

    // transform the root pose into the model frame:
    //     model -> root = model -> input * input -> root
    //
    // transform the object pose into the model frame:
    //     model -> object = model -> input * input -> object
    //
    // example:
    //   model = "base_footprint", input_robot = "map", input_object = "map"
    //     "base_footprint" -> "base_footprint" = "base_footprint" -> "map" * "map -> "base_footprint"
    //     "base_footprint" -> object = "base_footprint" -> "map" * "map" -> object = "base_footprint" -> "object"
    //
    // example:
    //   model = "map", input_robot = "abs_nwu", input_object = "abs_nwu"
    //     "map" -> "base_footprint" = "map" -> "abs_nwu" * "abs_nwu" -> "base_footprint
    //     "map" -> "object" = "map" -> "abs_nwu" * "abs_nwu" -> "object"

    const geometry_msgs::PoseStamped& rob_pose_in = current_goal_->base_link_in_map;
    const geometry_msgs::PoseStamped& obj_pose_in = current_goal_->gas_can_in_map;
    const std::string& map_frame = map.header.frame_id;

    if (rob_pose_in.header.frame_id != robot_model_->getModelFrame()) {
        ROS_INFO("Transform robot pose into model frame");
        geometry_msgs::PoseStamped rob_pose_in_model;
        try {
            listener_.transformPose(
                    robot_model_->getModelFrame(),
                    rob_pose_in,
                    rob_pose_in_model);
            tf::poseMsgToEigen(rob_pose_in_model.pose, m_rob_pose);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("Failed to transform from '%s' to '%s' (%s)", rob_pose_in.header.frame_id.c_str(), robot_model_->getModelFrame().c_str(), ex.what());
            as_->setAborted();
            return;
        }
    } else {
        tf::poseMsgToEigen(rob_pose_in.pose, m_rob_pose);
    }

    if (obj_pose_in.header.frame_id != robot_model_->getModelFrame()) {
        ROS_INFO("Transform object pose into model frame");
        geometry_msgs::PoseStamped obj_pose_in_model;
        try {
            listener_.transformPose(
                    robot_model_->getModelFrame(),
                    obj_pose_in,
                    obj_pose_in_model);
            tf::poseMsgToEigen(obj_pose_in_model.pose, m_obj_pose);
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("Failed to transform from '%s' to '%s' (%s)", obj_pose_in.header.frame_id.c_str(), robot_model_->getModelFrame().c_str(), ex.what());
            as_->setAborted();
            return;
        }
    } else {
        tf::poseMsgToEigen(obj_pose_in.pose, m_obj_pose);
    }

    if (map_frame != robot_model_->getModelFrame()) {
        ROS_INFO("Lookup transform from model frame to grid frame");
        tf::StampedTransform t;
        try {
            listener_.lookupTransform(map_frame, robot_model_->getModelFrame(), ros::Time(0), t);
            tf::transformTFToEigen(t, m_T_model_grid);
            m_T_grid_model = m_T_model_grid.inverse();
        } catch (const tf::TransformException& ex) {
            ROS_ERROR("Failed to transform from '%s' to '%s' (%s)", map_frame.c_str(), robot_model_->getModelFrame().c_str(), ex.what());
            as_->setAborted();
            return;
        }
    } else {
        m_T_model_grid = Eigen::Affine3d::Identity();
        m_T_grid_model = Eigen::Affine3d::Identity();
    }

    ROS_INFO("  Robot Pose [model frame]: %s", to_string(m_rob_pose).c_str());
    ROS_INFO("  Gascan Pose [model frame]: %s", to_string(m_obj_pose).c_str());
    ROS_INFO("  T_grid_model: %s", to_string(m_T_grid_model).c_str());

    // update collision checker if valid map
    if (!cc_->updateOccupancyGrid(map)) {
        ROS_ERROR("Failed to update collision checker with latest map");
        as_->setAborted();
    }
}

void RepositionBaseExecutor::preemptCallback()
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
        visualizeBaseCandidates(rob, "raw_candidates", 3, 4, 3);
    }

    if (bCheckGrasp) {
        computeGraspProbabilities(ss, rob, object_pose, pTotThr, pGrasp, bTotMax);

        const bool publish_pgrasp_map = true;
        if (publish_pgrasp_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pGrasp, bTotMax, prob_grid);
            pgrasp_map_pub_.publish(prob_grid);
        }
    }

    if (bCheckObs) {
        // filter out poses where the robot is definitely in collision
        pruneCollisionStates(ss, rob, bTotMax);

        const bool publish_pobs_map = true;
        if (publish_pobs_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pObs, bTotMax, prob_grid);
            pobs_map_pub_.publish(prob_grid);
        }

        computeArmCollisionProbabilities(ss, rob, pObs, bTotMax);
        computeBaseCollisionProbabilities(ss, rob, pObs, bTotMax);
    }

    // TODO: robot arm workspace limit should be included here!

    au::grid<3, double> pTot;
    multiplyProbabilities(pGrasp, pObs, bTotMax, pTot);

    scaleByHeadingDifference(
            ss, rob, bTotMax, object_pose, robot_pose, scaleDiffYglob, pTotThr, pTot);

    // sort candidates with respect to pTot
    std::vector<RepositionBaseCandidate::candidate> cands;
    extractValidCandidatesSorted(ss, bTotMax, pTot, cands);

    // finally, gather all pose candidates with valid probabilities
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

            candidate_pose.header.frame_id = robot_model_->getModelFrame();
            candidate_pose.header.stamp = ros::Time::now();

            Eigen::Affine3d T_world_robot_3d = poseEigen2ToEigen3(T_world_robot);
            tf::poseEigenToMsg(T_world_robot_3d, candidate_pose.pose);

            candidate_base_poses.push_back(candidate_pose);

            visualizeRobot(T_world_robot, (cand.pTot * 240) - 120, "base_probable_candidates", base_probcandidates_viz_id);
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
    int secDist[2] = { 1 * ss.nDist / 3, 2 * ss.nDist / 3 };

    // distance-dependent allowable heading offset range
    double secAngYaw[2];

    // distance-dependent "most desirable" heading offset
    double bestAngYaw;

    // most desirable robot-object distance
    double bestDist = 0.70;

    // pGrasp: quadratic function (1 at bestDist, (1 - scalepGraspDist)^2 at borders)
    double scalepGraspDist = 0.05; // 0.1

    // pGrasp: quadratic function (1 at bestAngYaw, (1 - scalepGraspAngYaw)^2 at borders)
    double scalepGraspAngYaw = 0.05; // 0.1

    ///////////////////////
    // END CONFIGURATION //
    ///////////////////////

    for (int i = 0; i < ss.nDist; i++) {
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

        for (int j = 0; j < ss.nAng; ++j) {
            for (int k = 0; k < ss.nYaw; ++k) {
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
                            fabs(ss.distMin + ss.distStep * (ss.nDist - 1) - bestDist));

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

void RepositionBaseExecutor::computeExhaustiveGraspProbabilities(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    const Pose2D& obj,
    double pTotThr,
    au::grid<3, double>& pGrasp,
    au::grid<3, bool>& bTotMax)
{
    int base_seen_viz_id = 0;

    int secDist[2] = { ss.nDist / 3, 2 * ss.nDist / 3 };
    double secAngYaw[2];
    double bestAngYaw;
    double bestDist = 0.60;

//    double secSide[2] = { angles::from_degrees(0.0), angles::from_degrees(45.0) };
//    double secSide[2] = { angles::from_degrees(0.0), angles::from_degrees(20.0) };
    double secSide[2] = { angles::from_degrees(-5.0), angles::from_degrees(40.0) };

    // pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)
    double scalepGraspDist = 0.05; // 0.1

    // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
    double scalepGraspAngYaw = 0.3; // 0.05, 0.1

    // heuristic probability of successful grasping
    // a) position: the object should be within left-hand side of the robot and 45 deg angle of view
    // exception: exclude poses at close distance, include ones in right-hand side at far distance
    // b) orientation: the object handle should be directed to the right of the robot
    // c) exception: allow more deviation at close distance, reject most of deviations at far distance

    for (int i = 0; i < ss.nDist; i++) {
        // c) set acceptable object orientation range
        if (i < secDist[0]) {
            secAngYaw[0] = angles::from_degrees(45.0); // 15
            secAngYaw[1] = angles::from_degrees(130.0); // 90, 100
            bestAngYaw = angles::from_degrees(-90.0); // -60, 65, 60
        } else if (i < secDist[1]) {
            secAngYaw[0] = angles::from_degrees(45.0); // 15
            secAngYaw[1] = angles::from_degrees(130.0); // 90
            bestAngYaw = angles::from_degrees(-90.0); // -60, 60, 45
        } else { // if (i >= secDist[1])
            secAngYaw[0] = angles::from_degrees(45.0); // 15
            secAngYaw[1] = angles::from_degrees(130.0); // 90, 60, 30
            bestAngYaw = angles::from_degrees(-90.0); // -60, -120, 50, 15
        }

        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                // object position (not orientation)
                double rob2obj = atan2(obj.y - rob(i, j, k).y, obj.x - rob(i, j, k).x);
                double diffAng = angles::normalize_angle(rob2obj - rob(i, j, k).yaw);
                double diffAngMax = fabs(ss.yawMin - secSide[0]);
                // b) object handle orientation to the right of the robot
                double diffY = angles::normalize_angle(obj.yaw - rob(i, j, k).yaw);
                double diffYMax = M_PI / 2.0;
                double diffDistMax = std::max(fabs(ss.distMin - bestDist),
                        fabs(ss.distMin + ss.distStep * (ss.nDist - 1) - bestDist));
                // left-hand side and angle of view
                if (diffAng >= secSide[0] && diffAng < secSide[1]) {
                    if (diffY >= secAngYaw[0] && diffY <= secAngYaw[1]) {
                        // EXCLUDE CANDIDATES WE HAVE ALREADY SEEN
                        bTotMax(i, j, k) = false;
                        Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
                        Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                        visualizeRobot(T_world_robot, 0, "base_candidates_seen", base_seen_viz_id);
                        // higher probability around diffY==bestAngYaw
                        // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
//                        pGrasp(i, j, k) = std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGrasp)/(diffYMax), 2.0 );
//                        pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);

                        // higher probability around diffY==bestAngYaw and diffDist==bestDist
                        // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
//                        pGrasp(i, j, k) = std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0) * std::pow( (diffDistMax-fabs(ss.distMin+distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0));
//                        pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);
                    }
                }
                // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
//                pGrasp(i, j, k) = std::pow( std::max( std::min(diffAng-secSide[0], 0.0)/diffAngMax + 1.0, 0.0), 1.0) *
//                        std::pow( (diffYMax-fabs(diffY-bestAngYaw)*scalepGraspAngYaw)/(diffYMax), 2.0) *
//                        std::pow( (diffDistMax-fabs(ss.distMin+ss.distStep*i-bestDist)*scalepGraspDist)/(diffDistMax), 2.0);
//                pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);
//                pGrasp(i, j, k) = std::max( std::pow( std::max( std::min(diffAng-secSide[0], 0.0)/diffAngMax + 1.0, 0.0), 1.0)

                // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                pGrasp(i, j, k) =
                        std::max(std::min(diffAng - angles::from_degrees(5.0), 0.0) / diffAngMax + 1.0, 0.0) * // ^1
                        sqrd((diffYMax - std::min(fabs(angles::normalize_angle(diffY - bestAngYaw)), fabs(angles::normalize_angle(diffY - M_PI - bestAngYaw))) * scalepGraspAngYaw) / (diffYMax)) *
                        sqrd((diffDistMax - fabs(ss.distMin + ss.distStep * i - bestDist) * scalepGraspDist) / (diffDistMax));
                pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);
            }
        }
    }
}

/// \brief Update the probabilities of collision free, with respect to the base
///
/// The projected footprint of the robot is checked for collisions with the
/// occupancy grid. If a collision is found, the probability of being collision
/// free is (obviously) set to 0.
void RepositionBaseExecutor::pruneCollisionStates(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    au::grid<3, bool>& bTotMax)
{
    int base_footprint_viz_id = 0;
    for (int i = 0; i < ss.nDist; ++i) {
    for (int j = 0; j < ss.nAng; ++j) {
    for (int k = 0; k < ss.nYaw; ++k) {
        if (!bTotMax(i, j, k)) {
            continue;
        }
        Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
        Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;

        // Transform to simple pose in grid frame
        Eigen::Affine3d T_model_robot = poseEigen2ToEigen3(T_world_robot);
        Eigen::Affine3d T_grid_robot = m_T_model_grid.inverse() * T_model_robot;
        Pose2D rp = poseEigen2ToSimple(poseEigen3ToEigen2(T_grid_robot));

        if (!cc_->isValidState(rp.x, rp.y, rp.yaw)) {
            // equivalent to pObs[i][j][k] = 0;
            bTotMax(i, j, k) = false;
            auto fp_markers = cc_->getFootprintVisualization(rp.x, rp.y, rp.yaw);
            for (auto& m : fp_markers.markers) {
                m.ns = "footprint_polygon_collisions";
                m.id = base_footprint_viz_id++;
            }
            SV_SHOW_INFO(fp_markers);
        }
    }
    }
    }
}

/// \brief Update the probabilities of collision free, with respect to the arm
///
/// The approximate (circular) model of the arm is checked for distance with the
/// nearest obstacle in the occupancy grid. If the nearest obstacle is less than
/// a minimum radius for the arm, the pose is considered to be in collision (0%
/// chance of being collision free). If the nearest obstacle is further away
/// than the maximum radius of the arm, the pose is considered to be collision
/// free. Distances in between these two extents have their probabilities
/// represented as a quadratic function that is 0 when the nearest occupied cell
/// is at the minimum radius away and 1 when the nearest occupied cell is at
/// the maximum radius away.
void RepositionBaseExecutor::computeArmCollisionProbabilities(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    au::grid<3, double>& pObs,
    au::grid<3, bool>& bTotMax)
{
    int arm_collision_viz_id = 0;
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
                Eigen::Translation2d(m_arm_offset_x, m_arm_offset_y));
        Eigen::Affine2d T_world_arm = T_world_mount * T_mount_arm;

        Eigen::Affine3d T_model_arm = poseEigen2ToEigen3(T_world_arm);
        Eigen::Affine3d T_grid_arm = m_T_model_grid.inverse() * T_model_arm;

        const double armx = T_grid_arm.translation()[0];
        const double army = T_grid_arm.translation()[1];

        double distObs = cc_->getCellObstacleDistance(armx, army);
        if (distObs < m_arm_length_core) {
            bTotMax(i, j, k) = false;
            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j ,k));
            Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
            visualizeRobot(T_world_robot, 0, "candidate_arm_collisions", arm_collision_viz_id);
        } else if (distObs < m_arm_length) {
            double p = sqrd((distObs - m_arm_length_core) / (m_arm_length - m_arm_length_core));
            pObs(i, j, k) = std::min(pObs(i, j, k), p);
        }
    }
    }
    }
}

/// \brief Update the probabilities of collision free, with respect to the base
///
/// The approximate (circular) model of the base is checked for distance with
/// the nearest obstacle in the occupancy grid. If the nearest obstacle is less
/// than a minimum radius for the base, the pose is considered to be in
/// collision (0% chance of being collision free). If the nearest obstacle is
/// further away than the maximum radius of the base, the pose is considered to
/// be collision free. Distances in between these two extents have their
/// probabilities represented as a quadratic function that is 0 when the nearest
/// occupied cell is at the minimum radius away and 1 when the nearest occupied
/// cell is at the maximum radius away.
void RepositionBaseExecutor::computeBaseCollisionProbabilities(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    au::grid<3, double>& pObs,
    au::grid<3, bool>& bTotMax)
{
    int base_collision_viz_id = 0;
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

        Eigen::Affine3d T_model_body = poseEigen2ToEigen3(T_world_robot);
        Eigen::Affine3d T_grid_body = m_T_model_grid.inverse() * T_model_body;

        double bodyx = T_grid_body.translation()[0];
        double bodyy = T_grid_body.translation()[1];

        double distObs = cc_->getCellObstacleDistance(bodyx, bodyy);
        if (distObs < m_body_length_core) {
            bTotMax(i, j, k) = false;
            visualizeRobot(T_world_robot, 0, "candidate_body_collisions", base_collision_viz_id);
        } else if (distObs < m_body_length) {
            // pObs: quadratic function (1 at outer borders, 0 at
            // inner borders) lowest value among the patch
            // cells for current i,j,k
            double p = sqrd((distObs - m_body_length_core) / (m_body_length - m_body_length_core));
            pObs(i, j, k) *= std::min(pObs(i, j, k), p);
        }
    }
    }
    }
}

/// \brief Compute the pair-wise product of two probability grids.
///
/// The output grid is unmodified if the dimensions of the two input grids are
/// not identical.
///
/// \return true if the grids are of the same dimensions; false otherwise
bool RepositionBaseExecutor::multiplyProbabilities(
    const au::grid<3, double>& p1,
    const au::grid<3, double>& p2,
    const au::grid<3, bool>& b,
    au::grid<3, double>& p)
{
    if (p1.size(0) != p2.size(0) ||
        p1.size(1) != p2.size(1) ||
        p1.size(2) != p2.size(2))
    {
        return false;
    }

    p.resize(p1.size(0), p1.size(1), p1.size(2));
    p.assign(0.0);
    for (int i = 0; i < p1.size(0); i++) {
        for (int j = 0; j < p1.size(1); j++) {
            for (int k = 0; k < p1.size(2); k++) {
                if (b(i, j, k)) {
                    p(i, j, k) = p1(i, j, k) * p2(i, j, k);
                }
            }
        }
    }

    return true;
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
            } else {
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

void RepositionBaseExecutor::visualizeBaseCandidates(
    const au::grid<3, Pose2D>& cands,
    const std::string& ns,
    int radius_throttle,
    int angle_throttle,
    int yaw_throttle) const
{
    int raw_candidate_viz_id = 0;
    int throttle_r = radius_throttle;
    int throttle_a = angle_throttle;
    int throttle_y = yaw_throttle;
    for (int i = 0; i < cands.size(0); ++i) {
        if (i % throttle_r != 0) {
            continue;
        }
        for (int j = 0; j < cands.size(1); ++j) {
            if (j % throttle_a != 0) {
                continue;
            }
            for (int k = 0; k < cands.size(2); ++k) {
                if (k % throttle_y != 0) {
                    continue;
                }
                Eigen::Affine2d T_world_mount = poseSimpleToEigen2(cands(i, j, k));
                Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                visualizeRobot(T_world_robot, 180, ns, raw_candidate_viz_id);
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
    const Eigen::Affine3d& rp3,
    const Eigen::Affine3d& op3,
    std::vector<geometry_msgs::PoseStamped>& candidate_base_poses)
{
    ROS_INFO("computeRobPoseExhaustive");

    Pose2D robot_pose = poseEigen3ToSimple(rp3);
    Pose2D object_pose = poseEigen3ToSimpleGascan(op3);

    ROS_INFO("robot pose: (%0.3f, %0.3f, %0.3f)", robot_pose.x, robot_pose.y, angles::to_degrees(robot_pose.yaw));
    ROS_INFO("object pose: (%0.3f, %0.3f, %0.3f)", object_pose.x, object_pose.y, angles::to_degrees(object_pose.yaw));

    // visualizations
    int base_candidates_viz_id = 0;
    int base_probcandidates_viz_id = 0;
    int base_collision_viz_id = 0;
    int base_probreject_viz_id = 0;
    int base_validityreject_viz_id = 0;
    int base_failedik_viz_id = 0;

    ///////////////////////
    // PARAMETER SETTING //
    ///////////////////////

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

    bool bCheckGrasp = true;
    bool bCheckObs = true;
    bool bCheckWork = false;

    // threshold for classifying clear and occupied regions
    int mapObsThr = 1;

    // multiply a quadratic function to pTot (1 at diffYglob==0,
    // (1-wDiffYglob)^2 at diffYglob==M_PI) for bSortMetric==3
    double scaleDiffYglob = 0.05;

    // pTot threshold for bSortMetric==3
    double pTotThr = 0.0; // 0.5

    // 6) /base_link offset from /top_shelf for final robot pose return
    // /base_link to /base_link_front_bumper_part to /top_shelf
    // /base_link to /base_link_front_bumper_part (in new Hokuyo setting)
    double baseOffsetx = -0.49 + 0.148975; // -0.49, -0.5, -0.3, 0.0

    // 1: angle, 2: angle, position, 3: pTot threshold (default)
    int bSortMetric = 3;

    ///////////////////////////
    // END PARAMETER SETTING //
    ///////////////////////////

    au::grid<3, bool> bTotMax(ss.nDist, ss.nAng, ss.nYaw);

    au::grid<3, double> pGrasp(ss.nDist, ss.nAng, ss.nYaw);
    au::grid<3, double> pObs(ss.nDist, ss.nAng, ss.nYaw);

    bTotMax.assign(true);
    pGrasp.assign(1.0);
    pObs.assign(1.0);

    au::grid<3, Pose2D> rob;
    generateCandidatePoseSamples(object_pose, ss, rob);

    if (bCheckGrasp) {
        computeExhaustiveGraspProbabilities(ss, rob, object_pose, pTotThr, pGrasp, bTotMax);

        const bool publish_pgrasp_map = true;
        if (publish_pgrasp_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pGrasp, bTotMax, prob_grid);
            pgrasp_exhaustive_map_pub_.publish(prob_grid);
        }
    }

    if (bCheckObs) {
        pruneCollisionStates(ss, rob, bTotMax);
        computeArmCollisionProbabilities(ss, rob, pObs, bTotMax);
        computeBaseCollisionProbabilities(ss, rob, pObs, bTotMax);
    }

    // TODO: robot arm workspace limit should be included here!

    if (bCheckWork) {
        // check for inverse kinematics and arm planning (if object is in
        // reachable range and within angle of view)
        for (int i = 0; i < ss.nDist; i++) {
            for (int j = 0; j < ss.nAng; j++) {
                for (int k = 0; k < ss.nYaw; k++) {
                    if (bTotMax(i, j, k)) {
                        geometry_msgs::PoseStamped candidate_base_pose;
                        candidate_base_pose.header.frame_id = "/abs_nwu";
                        candidate_base_pose.header.seq = 0;
                        candidate_base_pose.header.stamp = ros::Time::now();

                        double robY_base = rob(i, j, k).yaw;
                        double robx_base = rob(i, j, k).x + cos(robY_base) * baseOffsetx;
                        double roby_base = rob(i, j, k).y + sin(robY_base) * baseOffsetx;
                        candidate_base_pose.pose.position.x = robx_base;
                        candidate_base_pose.pose.position.y = roby_base;
                        candidate_base_pose.pose.position.z = 0.0;

                        tf::Quaternion robqf = tf::createQuaternionFromRPY(0.0, 0.0, robY_base);
                        candidate_base_pose.pose.orientation.x = robqf[0];
                        candidate_base_pose.pose.orientation.y = robqf[1];
                        candidate_base_pose.pose.orientation.z = robqf[2];
                        candidate_base_pose.pose.orientation.w = robqf[3];

                        int retIKPLAN = checkIK(rp3, op3);
                        if (retIKPLAN != 1) {
                            bTotMax(i, j, k) = false;
                            // checkIK failed!
                            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
                            Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                            visualizeRobot(T_world_robot, 0, "base_candidates_failed_ik", base_failedik_viz_id);
                        }
                    }
                }
            }
        }
    }

    au::grid<3, double> pTot;
    multiplyProbabilities(pGrasp, pObs, bTotMax, pTot);

    int cntTotMax = std::count(bTotMax.begin(), bTotMax.end(), true);
    if (cntTotMax == 0) {
        ROS_ERROR("no pose candidates with valid probabilities");
        return false;
    }

    // TODO: visualize rejected candidates

    scaleByHeadingDifference(
            ss, rob, bTotMax, object_pose, robot_pose, scaleDiffYglob, pTotThr, pTot);

    // gather all valid candidates (p != 0)
    std::vector<RepositionBaseCandidate::candidate> cands;
    extractValidCandidatesSorted(ss, bTotMax, pTot, cands);

    // check for arm planning (at least 10 candidates)
    int cntCheckPLAN = 0;
    int cntCheckPLANreject = 0;

    // TODO: decide the number of arm planning test
    int cntCheckPLANMax = 2;
    for (auto m = cands.begin(); m != cands.end(); ++m) {
        if (cntCheckPLAN == cntCheckPLANMax) {
            break;
        }

        int i = m->i;
        int j = m->j;
        int k = m->k;

        Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
        Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
        Eigen::Affine3d T_world_robot_3d = poseEigen2ToEigen3(T_world_robot);
        int err = checkFeasibleMoveToPregraspTrajectory(T_world_robot_3d, op3);
        ROS_INFO("retIKPLAN: %d", err);
        if (err) {
            cntCheckPLANreject++;
            cands.erase(m);
            m--;
        } else {
            cntCheckPLAN++;
        }
    }
    ROS_INFO("Number of rejection until finding %d feasible candidates: %d\n", cntCheckPLANMax, cntCheckPLANreject);

    // finally, gather all pose candidates with valid probabilities
    for (auto m = cands.begin(); m != cands.end(); ++m) {
        if (m->pTot >= pTotThr) {
            int i = m->i;
            int j = m->j;
            int k = m->k;

            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
            Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;

            geometry_msgs::PoseStamped candidate_pose;
            candidate_pose.header.frame_id = robot_model_->getModelFrame();
            candidate_pose.header.stamp = ros::Time::now();
            Eigen::Affine3d T_world_robot_3d = poseEigen2ToEigen3(T_world_robot);
            tf::poseEigenToMsg(T_world_robot_3d, candidate_pose.pose);

            candidate_base_poses.push_back(candidate_pose);

            visualizeRobot(T_world_robot, (m->pTot * 240) - 120, "base_probable_candidates", base_candidates_viz_id);
        }
    }

    if (candidate_base_poses.empty()) {
        ROS_WARN("    No probable candidate poses higher than a threshold!");
        return false;
    }

    ROS_INFO("    Number of IK feasible candidates: %d", cntTotMax);
//    ROS_INFO("    Number of PLAN feasible candidates: %d", cntTotThr);

    return true;
}

/// \brief Filter grasp candidates by kinematics and visibility
void RepositionBaseExecutor::pruneGraspCandidates(
    std::vector<rcta::GraspCandidate>& candidates,
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& camera_pose,
    double marker_incident_angle_threshold_rad) const
{
    ROS_INFO("Filter %zu grasp candidates", candidates.size());

    EigenSTL::vector_Affine3d marker_poses;
    marker_poses.reserve(attached_markers_.size());
    for (const auto& marker : attached_markers_) {
        marker_poses.push_back(marker.link_to_marker);
    }

    // run this first since this is significantly less expensive than IK
    rcta::PruneGraspsByVisibility(
            candidates,
            marker_poses,
            camera_pose,
            marker_incident_angle_threshold_rad);

    pruneGraspCandidatesIK(candidates, robot_pose);
}

/// \brief Filter grasp candidates by kinematic feasibility (inverse kinematics)
void RepositionBaseExecutor::pruneGraspCandidatesIK(
    std::vector<rcta::GraspCandidate>& candidates,
    const Eigen::Affine3d& T_grasp_robot) const
{
    ROS_INFO("Filter %zu grasp candidate via IK", candidates.size());
    std::vector<rcta::GraspCandidate> filtered_candidates;
    filtered_candidates.reserve(candidates.size());

    for (const rcta::GraspCandidate& grasp_candidate : candidates) {
        moveit::core::RobotState robot_state(robot_model_);
        robot_state.setToDefaultValues();
        // place the robot in the grasp frame
        const moveit::core::JointModel* root_joint = robot_model_->getRootJoint();
        robot_state.setJointPositions(root_joint, T_grasp_robot);
        robot_state.update();

        ROS_DEBUG("test grasp candidate %s for ik solution", to_string(grasp_candidate.pose).c_str());

        // check for an ik solution to this grasp pose
        std::vector<double> sol;
        if (robot_state.setFromIK(manip_group_, grasp_candidate.pose)) {
            robot_state.copyJointGroupPositions(manip_group_, sol);
            rcta::GraspCandidate reachable_grasp_candidate(
                    grasp_candidate.pose,
                    grasp_candidate.pose_in_object,
                    grasp_candidate.u);
            filtered_candidates.push_back(reachable_grasp_candidate);

            ROS_INFO("Grasp pose: %s", to_string(grasp_candidate.pose).c_str());
            ROS_INFO("IK sol: %s", to_string(sol).c_str());
        }
    }

    ROS_INFO("%zu/%zu reachable candidates", filtered_candidates.size(), candidates.size());
    candidates = std::move(filtered_candidates);
}

/// \brief Return a sequence of grasp candidates to try
///
/// The grasp candidates are filtered by inverse kinematics and tag detection
/// feasibility checks. The resulting sequence is sorted by the probability of
/// success of the grasp.
bool RepositionBaseExecutor::generateFilteredGraspCandidates(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose,
    std::vector<rcta::GraspCandidate>& candidates)
{
    // generate grasps in the world frame
    int max_samples = m_max_grasp_samples; //100;
    if (!m_grasp_planner.sampleGrasps(object_pose, max_samples, candidates)) {
        ROS_ERROR("Failed to sample grasps");
        return false;
    }

    ROS_INFO("Sampled %zd grasp poses", candidates.size());
    SV_SHOW_INFO(getGraspCandidatesVisualization(candidates, "grasp_candidates_checkIKPLAN"));

    moveit::core::RobotState robot_state = currentRobotState();
    robot_state.setJointPositions(robot_model_->getRootJoint(), robot_pose);
    robot_state.update();
    const Eigen::Affine3d& camera_pose =
            robot_state.getGlobalLinkTransform(camera_view_frame_);

    ROS_INFO("world -> camera: %s", to_string(camera_pose).c_str());

    const double vis_angle_thresh = sbpl::utils::ToRadians(45.0);
    pruneGraspCandidates(candidates, robot_pose, camera_pose, vis_angle_thresh);

    ROS_INFO("Produced %zd feasible grasp poses", candidates.size());

    rcta::RankGrasps(candidates);

    SV_SHOW_INFO(getGraspCandidatesVisualization(candidates, "grasp_candidates_checkIKPLAN_filtered"));

    if (candidates.empty()) {
        ROS_WARN("No reachable grasp candidates available");
    }

    return true;
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

/// \brief Project the 6-dof pose of the gascan to a planar representation.
///
/// This method (sadly) also modifies the frame of the gascan to satisfy
/// assumptions made literally all over the place, namely that the +x axis of
/// the object frame is in line with handle and directed toward the spout, the
/// +z axis is "up", and the y-axis is chosen to produce a right-handed
/// coordinate system
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
    Eigen::Affine2d T_world_robot = poseEigen3ToEigen2(robot_pose);
    Eigen::Affine2d T_world_mount = T_world_robot * T_mount_robot_.inverse();

    Pose2D op = poseEigen3ToSimpleGascan(object_pose);

    // check for inverse kinematics and arm planning (if object is in reachable
    // range and within angle of view)
    // TODO: a more general than (distMin + (nDist-1)*ss.distStep) determined in computeRobPose()
//    double secDist[2] = { 0.3, 1.5 };
    double secDist[2] = { 0.5, 1.0 };
    Eigen::Vector2d dp = Eigen::Vector2d(op.x, op.y) - T_world_mount.translation();

    Eigen::Rotation2Dd robot_yaw(0.0);
    robot_yaw.fromRotationMatrix(T_world_mount.rotation());
    double distRob2Obj = dp.norm();
    if (distRob2Obj >= secDist[0] && distRob2Obj <= secDist[1]) {
        // TODO: a more general than secSide[2] determined in computeRobPose()
//        double secSide[2] = { -20.0 / 180.0 * M_PI, 45.0 / 180.0 * M_PI };
        double secSide[2] = { angles::from_degrees(-5.0), angles::from_degrees(40.0) };
        // angular coordinate of a vector from robot position to object position
        double rob2obj = std::atan2(dp.y(), dp.x());
        double diffAng = angles::normalize_angle(rob2obj - robot_yaw.angle());
        if (diffAng >= secSide[0] && diffAng <= secSide[1]) {
            // left-hand side and angle of view
            double secAngYaw[2];
            secAngYaw[0] = 45.0 / 180.0 * M_PI;
            secAngYaw[1] = 130.0 / 180.0 * M_PI;

            double diffY = angles::normalize_angle(op.yaw - robot_yaw.angle());
            if (diffY >= secAngYaw[0] && diffY <= secAngYaw[1]) {
                return true;
            }
        }
    }

    return false;
}

/// \brief Check for a feasible trajectory for the arm to grasp the object
int RepositionBaseExecutor::checkFeasibleMoveToPregraspTrajectory(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose)
{
    ROS_INFO("check for feasible arm trajectory!");

    std::vector<rcta::GraspCandidate> grasp_candidates;
    if (!generateFilteredGraspCandidates(robot_pose, object_pose, grasp_candidates)) {
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
        const rcta::GraspCandidate& grasp = grasp_candidates[gidx];

        rcta::MoveArmGoal pregrasp_goal;
        pregrasp_goal.type = rcta::MoveArmGoal::EndEffectorGoal;
        tf::poseEigenToMsg(grasp.pose, pregrasp_goal.goal_pose);

        // set the pose of the robot
        pregrasp_goal.start_state.is_diff = true;
        pregrasp_goal.start_state.multi_dof_joint_state.header.frame_id =
                robot_model_->getModelFrame();
        pregrasp_goal.start_state.multi_dof_joint_state.joint_names = {
            robot_model_->getRootJoint()->getName()
        };
        geometry_msgs::Transform robot_pose_msg;
        tf::transformEigenToMsg(robot_pose, robot_pose_msg);
        pregrasp_goal.start_state.multi_dof_joint_state.transforms = {
            robot_pose_msg
        };

        auto result_cb = boost::bind(&RepositionBaseExecutor::move_arm_command_result_cb, this, _1, _2);
        move_arm_command_client_->sendGoal(pregrasp_goal, result_cb);

        int count = 0;
        while (!move_arm_command_client_->waitForResult(ros::Duration(0.5)) &&
                count < 60)
        {
            ros::spinOnce();
            ++count;
        }
        bool finished = count < 60; //move_arm_command_client_->waitForResult();
        if (!finished) {
            ROS_WARN("timeout waiting for move arm action server to finish");
        }

        if (finished && move_arm_command_client_->getState() ==
                actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return 0;
        }
    }

    return 1;
}

/// \brief Scale pose probabilities by difference from start heading
///
/// Resulting probabilities should be such that poses whose heading is closer to
/// the start heading are preferred
///
/// HEURISTIC: minimize angle difference between robot
/// orientation (robY) and robot motion direction (angO2Rcur)
void RepositionBaseExecutor::scaleByHeadingDifference(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    const au::grid<3, bool>& bTotMax,
    const Pose2D& object_pose,
    const Pose2D& robot_pose,
    double scale,
    double pTotThr,
    au::grid<3, double>& pTot)
{
    // angular coordinate of displacement from robot to object
    double rob2objY = atan2(object_pose.y - robot_pose.y, object_pose.x - robot_pose.x);
    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                if (bTotMax(i, j, k)) {
                    double diffYglob = angles::normalize_angle(rob(i, j, k).yaw - rob2objY);
                    // quadratic function (1 at diffYglob==0, (1-wDiffYglob)^2 at diffYglob==M_PI)
                    pTot(i, j, k) *= sqrd(1 - (scale * fabs(diffYglob) / M_PI));
                    pTot(i, j, k) = std::max(pTot(i, j, k), pTotThr);
                }
            }
        }
    }
}

void RepositionBaseExecutor::extractValidCandidatesSorted(
    const SearchSpaceParams& ss,
    const au::grid<3, bool>& bTotMax,
    const au::grid<3, double>& pTot,
    std::vector<RepositionBaseCandidate::candidate>& cands)
{
    cands.clear();
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
}

int RepositionBaseExecutor::checkIK(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose)
{
    ROS_INFO("checkIK!");
    std::vector<rcta::GraspCandidate> candidates;
    if (!generateFilteredGraspCandidates(robot_pose, object_pose, candidates)) {
        return 0;
    }
    return !candidates.empty();
}

void RepositionBaseExecutor::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::MoveArmResult::ConstPtr& result)
{
    move_arm_command_goal_state_ = state;
    move_arm_command_result_ = result;
}

/// \brief Visualize world-frame grasp candidates
visualization_msgs::MarkerArray
RepositionBaseExecutor::getGraspCandidatesVisualization(
    const std::vector<rcta::GraspCandidate>& grasps,
    const std::string& ns) const
{
    return rcta::GetGraspCandidatesVisualization(grasps, robot_model_->getModelFrame(), ns);
}

bool RepositionBaseExecutor::downloadMarkerParameters()
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
    int& id) const
{
    return visualizeRobot(poseFrom2D(x, y, yaw), hue, ns, id);
}

void RepositionBaseExecutor::visualizeRobot(
    const Eigen::Affine2d& pose,
    int hue,
    const std::string& ns,
    int& id) const
{
    return visualizeRobot(poseEigen2ToEigen3(pose), hue, ns, id);
}

void RepositionBaseExecutor::visualizeRobot(
    const Eigen::Affine3d& pose,
    int hue,
    const std::string& ns,
    int& id) const
{
    moveit::core::RobotState robot_state = currentRobotState();
    const moveit::core::JointModel* root_joint = robot_model_->getRootJoint();
    robot_state.setJointPositions(root_joint, pose);
    robot_state.update();
    visualization_msgs::MarkerArray ma;
    const std::vector<std::string>& link_names = { "talon", "torso_link0" }; //robot_model_->getLinkModelNames();
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    robot_state.getRobotMarkers(ma, link_names, color, ns, ros::Duration(0), false);
    for (auto& marker : ma.markers) {
        marker.id = id++;
    }
    SV_SHOW_INFO(ma);
}

#define METRICS_DEPRECATED 0
// HEURISTICALLY, minimum difference in angular coordinate wrt object, then
// farthest from the origin of object
void RepositionBaseExecutor::aMetricIDontHaveTimeToMaintain()
{
#if METRICS_DEPRECATED
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
        // HEURISTIC: minimize angle difference between robot orientation (rob(i, j, k).yawot motion direction (angO2Rcur)
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
                        } else {
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
                                } else {
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
#endif
}

// HEURISTICALLY, minimum difference in angular coordinate wrt object, then
// farthest from the origin of object
void RepositionBaseExecutor::anotherMetricIDontHaveTimeToMaintain()
{
#if METRICS_DEPRECATED
    double pTotMax = 1E-10;		// to opt out the poses with pTot(i, j, k)==0

    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                if (pTot(i, j, k) > pTotMax) {
                    pTotMax = pTot(i, j, k);
                }
            }
        }
    }
    int iMax = 0, jMax = 0, kMax = 0;		// indices for the final selection of robot pose
    int cntTotMax = 0;
    for (int i = 0; i < ss.nDist; i++)
        for (int j = 0; j < ss.nAng; j++)
            for (int k = 0; k < ss.nYaw; k++)
                if (pTot(i, j, k) == pTotMax) {
                    bTotMax(i, j, k) = true;
                    iMax = i;
                    jMax = j;
                    kMax = k;
                    cntTotMax++;
                } else {
                    bTotMax(i, j, k) = false;
                    // /top_shelf pose
                    double robxf = rob(i, j, k).x;
                    double robyf = rob(i, j, k).y;
                    double robYf = rob(i, j, k).yaw;
                    // /base_link pose
                    robxf += cos(robYf) * baseOffsetx;
                    robyf += sin(robYf) * baseOffsetx;
                    visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_validity_reject", base_validityreject_viz_id);
                }

    ROS_INFO("    cntTotMax: %d", cntTotMax);

    if (cntTotMax == 0) {
        // TODO: check why the previous candidate_base_poses remain in RViz panel
        // just set to initial robot poase
        robxf = robot_pose.x;
        robyf = robot_pose.y;
        robYf = robot_pose.yaw;
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
    } else if (cntTotMax > 1)	// if more than one candidate poses are selected
            // (we can select poses with pTot higher than a THRESHOLD)
            {
        // a) sorting by difference of angular coordinates for current and desired poses
        // ASSUME? backward driving is allowed for husky
        // HEURISTIC: minimize angle difference between robot orientation (rob(i, j, k).yawot motion direction (angO2Rcur)
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
                        } else {
                            bTotMax(i, j, k) = false;
                            // /top_shelf pose
                            double robxf = rob(i, j, k).x;
                            double robyf = rob(i, j, k).y;
                            double robYf = rob(i, j, k).yaw;
                            // /base_link pose
                            robxf += cos(robYf) * baseOffsetx;
                            robyf += sin(robYf) * baseOffsetx;
                            visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_prob_reject", base_probreject_viz_id);
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
                            if (bTotMax(i, j, k) == true) {
                                xysqerr = std::pow(rob(i, j, k).x - robot_pose.x, 2.0)
                                        + std::pow(rob(i, j, k).y - robot_pose.y, 2.0);

                                if (xysqerr <= xysqerrMin) {
                                    xysqerrMin = xysqerr;
                                    iMax = i;
                                    jMax = j;
                                    kMax = k;
                                    cntTotMax++;
                                } else {
                                    bTotMax(i, j, k) = false;
                                    // /top_shelf pose
                                    double robxf = rob(i, j, k).x;
                                    double robyf = rob(i, j, k).y;
                                    double robYf = rob(i, j, k).yaw;
                                    // /base_link pose
                                    robxf += cos(robYf) * baseOffsetx;
                                    robyf += sin(robYf) * baseOffsetx;
                                    visualizeRobot(poseFrom2D(robxf, robyf, robYf), 0, "base_candidates_prob_reject", base_probreject_viz_id);
                                }
                            }
                        }

                // OPTIONAL: update bTotMax(i, j, k)
                for (int i = 0; i < ss.nDist; i++)
                    for (int j = 0; j < ss.nAng; j++)
                        for (int k = 0; k < ss.nYaw; k++)
                            if (bTotMax(i, j, k) == true) {
                                xysqerr = std::pow(rob(i, j, k).x - robot_pose.x, 2.0)
                                        + std::pow(rob(i, j, k).y - robot_pose.y, 2.0);
                                if (xysqerr != xysqerrMin) {
                                    bTotMax(i, j, k) = false;
                                    // /top_shelf pose
                                    double robxf = rob(i, j, k).x;
                                    double robyf = rob(i, j, k).y;
                                    double robYf = rob(i, j, k).yaw;
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
                    robxf = rob(i, j, k).x;
                    robyf = rob(i, j, k).y;
                    robYf = rob(i, j, k).yaw;
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
#endif
}

#undef METRICS_DEPRECATED
