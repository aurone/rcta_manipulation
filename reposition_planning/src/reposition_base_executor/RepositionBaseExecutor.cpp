// standard includes
#include <signal.h>
#include <stdlib.h>
#include <algorithm>
#include <memory>
#include <sstream>

// systemm includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <angles/angles.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cmu_manipulation_msgs/RepositionBaseCommandAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <grasp_planner_interface/grasp_planner_plugin.h>
#include <grasp_planner_interface/grasp_utils.h>
#include <grasping_executive/MoveArmAction.h>
#include <hdt_kinematics/RobotModel.h>
#include <leatherman/utils.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <pluginlib/class_loader.h>
#include <rcta_manipulation_common/comms/actionlib.h>
#include <rcta_manipulation_common/MoveGroupGoal.h>
#include <ros/ros.h>
#include <smpl/debug/visualizer_ros.h>
#include <spellbook/geometry/nurb/NURB.h>
#include <spellbook/grid/grid.h>
#include <spellbook/utils/utils.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/utils/RunUponDestruction.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/Marker.h>

//xytheta collision checking!
#include "xytheta_collision_checker.h"

namespace rcta {

enum struct RepositionBaseExecutionStatus
{
    INVALID = -1,
    IDLE = 0,
    FAULT,
    COMPUTING_REPOSITION_BASE,
//    GENERATING_SEARCH_SPACE,
//    CHECKING_COLLISION,
//    CHECKING_KINEMATICS,
//    SELECTING_CANDIDATES,
    COMPLETING_GOAL
};

std::string to_string(RepositionBaseExecutionStatus status);

struct Pose2D
{
    double x;
    double y;
    double yaw;

    Pose2D() = default;
    Pose2D(double x, double y, double yaw) : x(x), y(y), yaw(yaw) { }
};

/// Describes the space of discrete poses with respect to another pose
struct SearchSpaceParams
{
    int nDist;          ///< Number of radial samples
    double distMin;     ///< Minimum radius away from pivot position
    double distStep;    ///< Discretization of radial samples

    int nAng;           ///< Number of angular samples wrt the pivot position
    double angMin;      ///< Minimum angle offset from pivot pose
    double angStep;     ///< Discretization of angular samples

    int nYaw;           ///< Number of relative heading samples wrt the pivot heading
    double yawMin;      ///< Minimum relative heading offset from pivot heading
    double yawStep;     ///< Discretization of yaw samples
};

/// Defines how to split the search space between the exhaustive and non-
/// exhaustive passes
struct SimplePruningParams
{
    double min_heading;
    double max_heading;
    double min_angle;
    double max_angle;
};

/// Node handling requests to plan for the position of the base required to
/// pickup an object
///
/// Notes on frames (in preparation for the inevitable integration issues
/// related to tf):
///
/// * The reposition planner uses the same concept of a "planning frame" as in
///   MoveIt!. I believe the decision on the planning frame is slightly more
///   complicated in MoveIt!. Here it is just the model frame of the robot,
///   which should either be the world frame (the parent in the virtual joint
///   attached the robot to the world), or the root frame of the robot.
///
/// * Grasps are returned in the frame the object is specified in
///
/// * Grasps are checked against IK for feasibility--the frame of the IK checker
///   is the model frame
///
/// * Grasps are checked against the camera pose for visibility--links of the
///   robot are specified in the model frame
///
/// * For simplicity, most computations should assume entities are specified in
///   the model frame, this means the input robot and object poses should be
///   transformed into the model frame upon receipt (TODO). For example, if the
///   model frame is the "map" frame and the input robot and object poses are in
///   the "map" frame, then no transformation takes place; if the model frame is
///   the root link of the robot, the robot pose will be the identity and the
///   object pose will be the relative transform from the robot to the object.
///
/// * Visualizations are produced in the model frame
///
/// * Poses have their 2-d footprint projections checked against the collision
///   map for collisions. The input occupancy grid might be specified in a frame
///   other than the model frame (TODO: should the occupancy grid be transformed
///   into the model frame, or should a copy of the robot and object poses in
///   the occupancy grid's frame so that checks can be made directly).
class RepositionBaseExecutor
{
public:

    RepositionBaseExecutor();
    ~RepositionBaseExecutor();

    bool initialize();
    int run();

    enum MainResult
    {
        SUCCESS = 0, FAILED_TO_INITIALIZE
    };

private:

    struct AttachedMarker
    {
        int marker_id;
        std::string attached_link;
        Eigen::Affine3d link_to_marker;
    };

    struct candidate
    {
        int i;
        int j;
        int k;
        double pTot;

        bool operator<(const candidate& cand2) const
        {
            return pTot > cand2.pTot;
        }
    };

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher viz_pub_;
    ros::Publisher pgrasp_map_pub_;
    ros::Publisher pobs_map_pub_;
    ros::Publisher pgrasp_exhaustive_map_pub_;

    tf::TransformListener listener_;

    smpl::VisualizerROS viz_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<cmu_manipulation_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
    std::unique_ptr<RepositionBaseCommandActionServer> as_;

    robot_model_loader::RobotModelLoaderPtr rml_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::JointModelGroup* manip_group_;
    std::string manip_name_;

    planning_scene_monitor::CurrentStateMonitorPtr m_state_monitor;

    std::string camera_view_frame_;

    Eigen::Translation2d T_mount_robot_;

    double m_base_front_offset_x;

    SearchSpaceParams m_ss;
    SearchSpaceParams m_ss_exhaustive;

    SimplePruningParams m_prune_params;

    double m_best_dist;
    double m_best_dist_exhaustive;

    /// \name Planar Collision Constraints
    ///@{
    std::unique_ptr<XYThetaCollisionChecker> cc_;

    double m_arm_front_offset_x = 0.0;
    double m_arm_front_offset_y = 0.0;
    double m_arm_length = 0.0;
    double m_arm_length_core = 0.0;
    double m_body_length = 0.0;
    double m_body_length_core = 0.0;
    ///@}

    /// \name Visibility Constraints
    ///@{
    std::vector<AttachedMarker> attached_markers_;
    ///@}

    /// \name Grasping
    ///@{
    // the hand-generated grasping spline is done in model coordinates
    // and requires at least the scale to be brought into world coordinates
    pluginlib::ClassLoader<GraspPlannerPlugin> m_grasp_planner_loader;
    using GraspPlannerPluginPtr = class_loader::ClassLoader::UniquePtr<GraspPlannerPlugin>;
    GraspPlannerPluginPtr m_grasp_planner;
    int m_max_grasp_samples = 0;
    ///@}

    bool m_check_reach = true;
    au::grid<3, bool> m_reachable_table;

    /// \name Arm Planning Constraints
    /// @{
    typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupActionClient;
    std::unique_ptr<MoveGroupActionClient> move_arm_command_client_;
    std::string move_arm_command_action_name_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    moveit_msgs::MoveGroupResult::ConstPtr move_arm_command_result_;
    ///@}

    geometry_msgs::PoseStamped robot_pose_world_frame_;

    cmu_manipulation_msgs::RepositionBaseCommandGoal::ConstPtr current_goal_;
    RepositionBaseExecutionStatus status_;
    RepositionBaseExecutionStatus last_status_;
    Eigen::Affine3d m_rob_pose;
    Eigen::Affine3d m_obj_pose;
    Eigen::Affine3d m_T_model_grid;
    Eigen::Affine3d m_T_grid_model;

    // 0 -> model frame, 1 -> object frame, 2 -> grid frame
    int m_cand_frame_option = 0;

    /// \name Initialization
    ///@{
    bool initGraspPlanner(ros::NodeHandle& nh);
    bool downloadGraspingParameters(ros::NodeHandle& nh);
    bool downloadMarkerParameters();
    ///@}

    /// \name State Monitoring
    ///@{
    auto currentRobotState() const -> moveit::core::RobotState;
    ///@}

    /// \name 2D and 3D Pose Type Conversions (and some Frame Policies)
    ///@{
    Eigen::Affine3d poseFrom2D(double x, double y, double yaw) const;

    Pose2D poseEigen3ToSimple(const Eigen::Affine3d& pose) const;
    Pose2D poseEigen2ToSimple(const Eigen::Affine2d& in) const;

    Eigen::Affine2d poseSimpleToEigen2(const Pose2D& pose) const;
    Eigen::Affine2d poseEigen3ToEigen2(const Eigen::Affine3d& pose) const;

    Eigen::Affine3d poseSimpleToEigen3(
        const Pose2D& in,
        double z = 0.0, double R = 0.0, double P = 0.0) const;

    Eigen::Affine3d poseEigen2ToEigen3(
        const Eigen::Affine2d& in,
        double z = 0.0, double R = 0.0, double P = 0.0) const;

    Pose2D poseEigen3ToSimpleGascan(const Eigen::Affine3d& object_pose) const;
    Eigen::Affine3d poseSimpleGascanToEigen3(const Pose2D& object_pose) const;
    ///@}

    /// \name Grasp Candidate Selection
    ///@{
    bool generateFilteredGraspCandidates(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose,
        std::vector<Grasp>& candidates);

    void pruneGraspCandidates(
        std::vector<Grasp>& candidates,
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& camera_pose,
        double marker_incident_angle_threshold_rad) const;

    void pruneGraspCandidatesIK(
        std::vector<Grasp>& candidates,
        const Eigen::Affine3d& T_grasp_robot) const;
    ///@}

    /// \name Reposition Search Policies
    ///@{
    bool tryFeasibleArmCheck(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose) const;

    bool computeRobPose(
        const nav_msgs::OccupancyGrid& map,
        const Pose2D& robot_pose,
        const Pose2D& object_pose,
        std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);

    bool computeRobPoseExhaustive(
        const nav_msgs::OccupancyGrid& map,
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose,
        std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);

    void generateCandidatePoseSamples(
        const Pose2D& obj_pose,
        const SearchSpaceParams& params,
        au::grid<3, Pose2D>& g) const;

    void computeExhaustiveGraspProbabilities(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        const Pose2D& obj,
        double pTotThr,
        au::grid<3, double>& pGrasp,
        au::grid<3, bool>& bTotMax);

    void computeGraspProbabilities(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        const Pose2D& obj,
        double pTotThr,
        au::grid<3, double>& pGrasp,
        au::grid<3, bool>& bTotMax);

    void pruneCollisionStates(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        au::grid<3, bool>& bTotMax);

    void pruneUnreachingStates(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        const Pose2D& obj,
        au::grid<3, bool>& bTotMax);

    void computeArmCollisionProbabilities(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        au::grid<3, double>& pObs,
        au::grid<3, bool>& bTotMax);

    void computeBaseCollisionProbabilities(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        au::grid<3, double>& pObs,
        au::grid<3, bool>& bTotMax);

    bool multiplyProbabilities(
        const au::grid<3, double>& p1,
        const au::grid<3, double>& p2,
        const au::grid<3, bool>& b,
        au::grid<3, double>& p);

    bool checkIK(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose);

    int checkFeasibleMoveToPregraspTrajectory(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose);

    void scaleByHeadingDifference(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        const au::grid<3, bool>& bTotMax,
        const Pose2D& object_pose,
        const Pose2D& robot_pose,
        double scale,
        double pTotThr,
        au::grid<3, double>& pTot);

    void extractValidCandidatesSorted(
        const SearchSpaceParams& ss,
        const au::grid<3, bool>& bTotMax,
        const au::grid<3, double>& pTot,
        std::vector<candidate>& cands);
    ///@}

    ///\name Debug Visualization
    ///@{
    void projectProbabilityMap(
        const nav_msgs::OccupancyGrid& map,
        const SearchSpaceParams& ss,
        const Pose2D& obj_pose,
        const au::grid<3, double>& prob,
        const au::grid<3, bool>& valid,
        nav_msgs::OccupancyGrid& grid);

    void visualizeBaseCandidates(
        const au::grid<3, Pose2D>& cands,
        const std::string& ns,
        int radius_throttle = 1,
        int angle_throttle = 1,
        int yaw_throttle = 1) const;

    visualization_msgs::MarkerArray
    getGraspCandidatesVisualization(
        const std::vector<Grasp>& grasp,
        const std::string& ns) const;

    void visualizeRobot(
        double x,
        double y,
        double yaw,
        int hue,
        const std::string& ns,
        int& id) const;

    void visualizeRobot(
        const Eigen::Affine2d& pose,
        int hue,
        const std::string& ns,
        int& id) const;

    void visualizeRobot(
        const Eigen::Affine3d& pose,
        int hue,
        const std::string& ns,
        int& id) const;

    std_msgs::ColorRGBA rainbow(double d) const;
    ///@}

    void goalCallback();
    void preemptCallback();

    void transformToOutputFrame(
            const Eigen::Affine3d& in,
            geometry_msgs::PoseStamped& out) const;

    uint8_t execution_status_to_feedback_status(
        RepositionBaseExecutionStatus status);

    void move_arm_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::MoveGroupResult::ConstPtr& result);
};

moveit_msgs::CollisionObject CreateGroundPlaneObject()
{
    moveit_msgs::CollisionObject gpo;
    gpo.header.frame_id = "map"; // TODO: model frame

    shape_msgs::Plane ground_plane;
    ground_plane.coef[0] = 0.0;
    ground_plane.coef[1] = 0.0;
    ground_plane.coef[2] = 1.0;

    // TODO: derive this from the resolution set in the world collision model
    // to be -0.5 * res, which should make one layer of voxels immediately
    // beneath z = 0
    ground_plane.coef[3] = 0.075;

    gpo.planes.push_back(ground_plane);
    gpo.plane_poses.push_back(geometry_msgs::IdentityPose());

    gpo.operation = moveit_msgs::CollisionObject::ADD;
    return gpo;
}

// TODO: copied from grasping_executive, find a home for this and configure
// common options for the torso/arm planner somewhere
auto BuildMoveGroupGoal(const MoveArmGoal& goal)
    -> moveit_msgs::MoveGroupGoal
{
    double allowed_planning_time = 10.0;
    const char* group_name = "right_arm_and_torso";
    const char* workspace_frame = "base_footprint";
    geometry_msgs::Vector3 workspace_min;
    workspace_min.x = -0.5;
    workspace_min.y = -1.5;
    workspace_min.z =  0.0;
    geometry_msgs::Vector3 workspace_max;
    workspace_max.x =  1.5;
    workspace_max.y =  0.5;
    workspace_max.z =  1.9;

    const char* pose_goal_planner_id = "right_arm_and_torso[right_arm_and_torso_ARA_BFS_ML]";
    const char* joint_goal_planner_id = "right_arm_and_torso[right_arm_and_torso_ARA_JD_ML]";

    double joint_tolerance = smpl::angles::to_radians(5.0);
    double pos_tolerance = 0.01;
    double rot_tolerance = smpl::angles::to_radians(5.0);
    std::string tip_link = "limb_right_link7";

    moveit_msgs::MoveGroupGoal g;
    g.request.allowed_planning_time = allowed_planning_time;
    g.request.group_name = group_name;
    g.request.max_acceleration_scaling_factor = 1.0;
    g.request.max_velocity_scaling_factor = 1.0;
    g.request.num_planning_attempts = 1;
    g.request.path_constraints.joint_constraints.clear();
    g.request.path_constraints.name = "";
    g.request.path_constraints.orientation_constraints.clear();
    g.request.path_constraints.position_constraints.clear();
    g.request.path_constraints.visibility_constraints.clear();
    g.request.trajectory_constraints.constraints.clear();
    if (!goal.execute_path) {
        g.request.start_state.is_diff = true;
    }
    g.request.workspace_parameters.header.frame_id = workspace_frame;
    g.request.workspace_parameters.min_corner = workspace_min;
    g.request.workspace_parameters.max_corner = workspace_max;

    auto& request = g.request;
    auto& ops = g.planning_options;

    ops.planning_scene_diff.robot_state.is_diff = true;
    ops.planning_scene_diff.world.octomap.origin.orientation.w = 1.0;
    ops.planning_scene_diff.world.octomap.octomap = goal.octomap;
    ops.planning_scene_diff.world.collision_objects = goal.planning_options.planning_scene_diff.world.collision_objects;
    ops.planning_scene_diff.world.collision_objects.push_back(CreateGroundPlaneObject());
    ops.planning_scene_diff.is_diff = true;

    ops.plan_only = !goal.execute_path;

    ops.look_around = false;

    ops.look_around_attempts = 0;

    ops.max_safe_execution_cost = 1.0;

    ops.replan = false;

    ops.replan_attempts = 0;

    ops.replan_delay = 0.0;

    request.start_state = goal.start_state;

    request.goal_constraints.clear();
    switch (goal.type) {
    case MoveArmGoal::JointGoal:
    {
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.name = "goal_constraints";
        for (size_t jidx = 0; jidx < goal.goal_joint_state.name.size(); ++jidx) {
            auto& joint_name = goal.goal_joint_state.name[jidx];
            double joint_pos = goal.goal_joint_state.position[jidx];

            moveit_msgs::JointConstraint joint_constraint;
            joint_constraint.joint_name = joint_name;
            joint_constraint.position = joint_pos;
            joint_constraint.tolerance_above = joint_tolerance;
            joint_constraint.tolerance_below = joint_tolerance;
            joint_constraint.weight = 1.0;
            goal_constraints.joint_constraints.push_back(joint_constraint);
        }
        request.goal_constraints.push_back(goal_constraints);

        request.planner_id = joint_goal_planner_id;
        break;
    }
    case MoveArmGoal::CartesianGoal:
    case MoveArmGoal::EndEffectorGoal:
    {
        moveit_msgs::Constraints goal_constraints;
        goal_constraints.name = "goal_constraints";

        geometry_msgs::PoseStamped tip_goal;

        // TODO: get this from the configured planning frame in moveit
        tip_goal.header.frame_id = "map";
        tip_goal.pose = goal.goal_pose;

        // one position constraint
        moveit_msgs::PositionConstraint goal_pos_constraint;
        goal_pos_constraint.header.frame_id = tip_goal.header.frame_id;
        goal_pos_constraint.link_name = tip_link;
        goal_pos_constraint.target_point_offset = geometry_msgs::CreateVector3(0.0, 0.0, 0.0);
        shape_msgs::SolidPrimitive tolerance_volume;
        tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
        tolerance_volume.dimensions = { pos_tolerance };
        goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
        goal_pos_constraint.constraint_region.primitive_poses.push_back(tip_goal.pose);
        goal_pos_constraint.weight = 1.0;

        // one orientation constraint
        moveit_msgs::OrientationConstraint goal_rot_constraint;
        goal_rot_constraint.header.frame_id = tip_goal.header.frame_id;
        goal_rot_constraint.orientation = tip_goal.pose.orientation;
        goal_rot_constraint.link_name = tip_link;
        goal_rot_constraint.absolute_x_axis_tolerance = rot_tolerance;
        goal_rot_constraint.absolute_y_axis_tolerance = rot_tolerance;
        goal_rot_constraint.absolute_z_axis_tolerance = rot_tolerance;
        goal_rot_constraint.weight = 1.0;

        goal_constraints.position_constraints.push_back(goal_pos_constraint);
        goal_constraints.orientation_constraints.push_back(goal_rot_constraint);
        request.goal_constraints.push_back(goal_constraints);

        request.planner_id = pose_goal_planner_id;
        break;
    }
    default:
        ROS_ERROR("Unrecognized MoveArmGoal type");
        return g;
    }

    return g;
}

std::string to_string(RepositionBaseExecutionStatus status)
{
    switch (status) {
    case RepositionBaseExecutionStatus::IDLE:
        return "Idle";
    case RepositionBaseExecutionStatus::FAULT:
        return "Fault";
    case RepositionBaseExecutionStatus::COMPUTING_REPOSITION_BASE:
        return "ComputingRepositionBase";
    default:
        return "Invalid";
    }
}

double sign(double val)
{
    return (val >= 0) ? 1.0 : -1.0;
}

// quadratic function: returns 1 when value = best and (1 - scale)^2 at the
// borders. Borders are defined as the two points at max_best_dist away from
// best_dist
double quad(double best, double value, double max_best_dist, double scale)
{
    return sqrd((max_best_dist - fabs(value - best) * scale) / max_best_dist);
}

RepositionBaseExecutor::RepositionBaseExecutor() :
    ph_("~"),
    viz_(ph_),
    action_name_("reposition_base_command"),
    manip_group_(nullptr),
    move_arm_command_action_name_("move_group"),
    move_arm_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    status_(RepositionBaseExecutionStatus::INVALID),
    last_status_(RepositionBaseExecutionStatus::INVALID),
    m_grasp_planner_loader("grasp_planner_interface", "rcta::GraspPlannerPlugin")
{
    smpl::viz::set_visualizer(&viz_);
}

RepositionBaseExecutor::~RepositionBaseExecutor()
{
    if (smpl::viz::visualizer() == &viz_) {
        smpl::viz::unset_visualizer();
    }
}

bool RepositionBaseExecutor::initialize()
{
    /////////////////////////////////////////////////////////////////////////
    // This could maybe be refactored, since there is a lot of commonality //
    // with GraspObjectExecutor                                            //
    /////////////////////////////////////////////////////////////////////////

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
    m_state_monitor.reset(new planning_scene_monitor::CurrentStateMonitor(
            robot_model_, transformer));
    m_state_monitor->startStateMonitor("joint_states");

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

    move_arm_command_client_.reset(new MoveGroupActionClient(move_arm_command_action_name_, false));
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

    double base_front_offset_x;
    if (!msg_utils::download_param(ph_, "base_front_offset_x", base_front_offset_x) ||
        !msg_utils::download_param(ph_, "arm_front_offset_x", m_arm_front_offset_x) ||
        !msg_utils::download_param(ph_, "arm_front_offset_y", m_arm_front_offset_y) ||
        !msg_utils::download_param(ph_, "arm_length", m_arm_length) ||
        !msg_utils::download_param(ph_, "arm_length_core", m_arm_length_core) ||
        !msg_utils::download_param(ph_, "body_length", m_body_length) ||
        !msg_utils::download_param(ph_, "body_length_core", m_body_length_core))
    {
        return false;
    }
    T_mount_robot_ = Eigen::Translation2d(base_front_offset_x, 0.0);

    if (!msg_utils::download_param(ph_, "sampling/dist_min", m_ss.distMin) ||
        !msg_utils::download_param(ph_, "sampling/dist_step", m_ss.distStep) ||
        !msg_utils::download_param(ph_, "sampling/dist_count", m_ss.nDist) ||
        !msg_utils::download_param(ph_, "sampling/ang_min", m_ss.angMin) ||
        !msg_utils::download_param(ph_, "sampling/ang_step", m_ss.angStep) ||
        !msg_utils::download_param(ph_, "sampling/ang_count", m_ss.nAng) ||
        !msg_utils::download_param(ph_, "sampling/yaw_min", m_ss.yawMin) ||
        !msg_utils::download_param(ph_, "sampling/yaw_step", m_ss.yawStep) ||
        !msg_utils::download_param(ph_, "sampling/yaw_count", m_ss.nYaw))
    {
        return false;
    }

    m_ss.angMin = angles::from_degrees(m_ss.angMin);
    m_ss.angStep = angles::from_degrees(m_ss.angStep);
    m_ss.yawMin = angles::from_degrees(m_ss.yawMin);
    m_ss.yawStep = angles::from_degrees(m_ss.yawStep);

    if (!msg_utils::download_param(ph_, "exhaustive_sampling/dist_min", m_ss_exhaustive.distMin) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/dist_step", m_ss_exhaustive.distStep) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/dist_count", m_ss_exhaustive.nDist) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/ang_min", m_ss_exhaustive.angMin) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/ang_step", m_ss_exhaustive.angStep) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/ang_count", m_ss_exhaustive.nAng) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/yaw_min", m_ss_exhaustive.yawMin) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/yaw_step", m_ss_exhaustive.yawStep) ||
        !msg_utils::download_param(ph_, "exhaustive_sampling/yaw_count", m_ss_exhaustive.nYaw))
    {
        return false;
    }

    m_ss_exhaustive.angMin = angles::from_degrees(m_ss_exhaustive.angMin);
    m_ss_exhaustive.angStep = angles::from_degrees(m_ss_exhaustive.angStep);
    m_ss_exhaustive.yawMin = angles::from_degrees(m_ss_exhaustive.yawMin);
    m_ss_exhaustive.yawStep = angles::from_degrees(m_ss_exhaustive.yawStep);

    if (!msg_utils::download_param(ph_, "grasping/best_dist", m_best_dist) ||
        !msg_utils::download_param(ph_, "exhaustive_grasping/best_dist", m_best_dist_exhaustive))
    {
        return false;
    }

    m_prune_params.min_angle = angles::from_degrees(45.0);
    m_prune_params.max_angle = angles::from_degrees(80.0);
    m_prune_params.min_heading = angles::from_degrees(-5.0);
    m_prune_params.max_heading = angles::from_degrees(40.0);

    if (!msg_utils::download_param(ph_, "candidate_frame", m_cand_frame_option)) {
        return false;
    }

//    m_check_reach = true;
    if (m_check_reach) {
        m_reachable_table.resize(m_ss.nDist, m_ss.nAng, m_ss.nYaw);
        m_reachable_table.assign(true);

        auto then = std::chrono::high_resolution_clock::now();
        au::grid<3, Pose2D> rob;
        Pose2D zero(0.0, 0.0, 0.0);
        generateCandidatePoseSamples(zero, m_ss, rob);

        pruneUnreachingStates(m_ss, rob, zero, m_reachable_table);
        size_t reachable = std::count(m_reachable_table.begin(), m_reachable_table.end(), true);
        ROS_INFO("%zu reachable poses", reachable);

        auto now = std::chrono::high_resolution_clock::now();
        ROS_INFO("Precomputing reachability table took %0.3f seconds", std::chrono::duration<double>(now - then).count());
    }

    return true;
}

bool RepositionBaseExecutor::initGraspPlanner(ros::NodeHandle& nh)
{
    std::string grasp_planner_plugin;
    if (!nh.getParam("grasp_planner_plugin", grasp_planner_plugin)) {
        ROS_ERROR("Failed to retrieve 'grasp_planner_plugin' from the param server");
        return false;
    }

    ROS_INFO("Load grasp planner plugin '%s'", grasp_planner_plugin.c_str());

    m_grasp_planner =
            m_grasp_planner_loader.createUniqueInstance(grasp_planner_plugin);
    if (!m_grasp_planner) {
        ROS_ERROR("Failed to create grasp planner");
        return false;
    }

    ROS_INFO("Initialize grasp planner plugin");
    ros::NodeHandle grasp_nh(nh, "grasping");
    if (!m_grasp_planner->init(nh_, grasp_nh)) {
        ROS_ERROR("Failed to initialize grasp planner");
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

auto RepositionBaseExecutor::currentRobotState() const -> moveit::core::RobotState
{
    assert(m_state_monitor && m_state_monitor->getPlanningScene());
    auto current_state = m_state_monitor->getCurrentState();
    if (!current_state) {
        ROS_WARN("FAILED TO GET CURRENT STATE");
        moveit::core::RobotState zero_state(robot_model_);
        zero_state.setToDefaultValues();
        return zero_state;
    } else {
        return *current_state;
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
                    cmu_manipulation_msgs::RepositionBaseCommandResult result;
                    result.result = cmu_manipulation_msgs::RepositionBaseCommandResult::SUCCESS;

                    geometry_msgs::PoseStamped p;
                    transformToOutputFrame(m_rob_pose, p);
                    p.header.stamp = ros::Time::now();
                    candidate_base_poses.push_back(p);

                    result.candidate_base_poses = candidate_base_poses;

                    as_->setSucceeded(result);
                    status_ = RepositionBaseExecutionStatus::IDLE;
                    break;
                }
            }

            if (computeRobPose(
                    map,
                    poseEigen3ToSimple(m_rob_pose),
                    gascan_pose,
                    candidate_base_poses))
            {
                cmu_manipulation_msgs::RepositionBaseCommandResult result;
                result.result = cmu_manipulation_msgs::RepositionBaseCommandResult::SUCCESS;
                result.candidate_base_poses = candidate_base_poses;
                as_->setSucceeded(result);
                status_ = RepositionBaseExecutionStatus::IDLE;
            } else if (computeRobPoseExhaustive(
                    map,
                    m_rob_pose,
                    m_obj_pose,
                    candidate_base_poses))
            {
                cmu_manipulation_msgs::RepositionBaseCommandResult result;
                result.result = cmu_manipulation_msgs::RepositionBaseCommandResult::SUCCESS;
                result.candidate_base_poses = candidate_base_poses;
                as_->setSucceeded(result);
                status_ = RepositionBaseExecutionStatus::IDLE;
            } else {
                cmu_manipulation_msgs::RepositionBaseCommandResult result;
                result.result = cmu_manipulation_msgs::RepositionBaseCommandResult::PLANNING_FAILURE_OBJECT_UNREACHABLE;
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
    auto& map = current_goal_->map;
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

    auto& rob_pose_in = current_goal_->base_link_in_map;
    auto& obj_pose_in = current_goal_->gas_can_in_map;
    auto& map_frame = map.header.frame_id;

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
            listener_.lookupTransform(robot_model_->getModelFrame(), map_frame, ros::Time(0), t);
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

/// Transform a pose, assumed to be in the model frame, to a chosen (via config)
/// output frame. This function assumes the state of m_obj_pose, m_T_grid_model,
/// and m_T_model_grid are relevant with respect to the currently active goal.
void RepositionBaseExecutor::transformToOutputFrame(
    const Eigen::Affine3d& robot_pose,
    geometry_msgs::PoseStamped& out) const
{
    // TODO: add options for returning poses in the original frame of the
    // robot or the object
    if (m_cand_frame_option == 0) { // model frame
        tf::poseEigenToMsg(robot_pose, out.pose);
        out.header.frame_id = robot_model_->getModelFrame();
    } else if (m_cand_frame_option == 1) { // object
        Eigen::Affine3d T_object_robot = m_obj_pose.inverse() * robot_pose;
        tf::poseEigenToMsg(T_object_robot, out.pose);
        out.header.frame_id = "object"; // hmm not a real frame id
    } else if (m_cand_frame_option == 2) { // grid frame
        Eigen::Affine3d T_grid_robot = m_T_grid_model * robot_pose;
        tf::poseEigenToMsg(T_grid_robot, out.pose);
        out.header.frame_id = current_goal_->map.header.frame_id;
    } else {
        ROS_ERROR("Invalid candidate frame option. Defaulting to model frame");
        tf::poseEigenToMsg(robot_pose, out.pose);
    }
}

uint8_t RepositionBaseExecutor::execution_status_to_feedback_status(
    RepositionBaseExecutionStatus status)
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

    const SearchSpaceParams& ss = m_ss;

    const bool bCheckGrasp = true;
    const bool m_check_distances = true;

    double pTotThr = 0.0; // 0.5

    ///////////////////////////
    // END PARAMETER SETTING //
    ///////////////////////////

    // 0) set search space (r, th, Y)
    // (r, th): polar coordinate with respect to the object with the center fixed at the origin and the nozzle aligned to 0 rad
    // (Y): orientation about z-axis

    au::grid<3, Pose2D> rob;
    generateCandidatePoseSamples(object_pose, ss, rob);

    const bool publish_all_candidate_viz = false;
    if (publish_all_candidate_viz) {
        visualizeBaseCandidates(rob, "raw_candidates", 3, 4, 3);
    }

    // binary flag to indicate whether the probability is zero and the state
    // can be pruned; true -> non-zero
    au::grid<3, bool> bTotMax(ss.nDist, ss.nAng, ss.nYaw);
    bTotMax.assign(true);

    au::grid<3, double> pGrasp(ss.nDist, ss.nAng, ss.nYaw);
    pGrasp.assign(1.0);
    if (bCheckGrasp) {
        computeGraspProbabilities(ss, rob, object_pose, pTotThr, pGrasp, bTotMax);

        const bool publish_pgrasp_map = true;
        if (publish_pgrasp_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pGrasp, bTotMax, prob_grid);
            pgrasp_map_pub_.publish(prob_grid);
        }
    }

    au::grid<3, double> pObs(ss.nDist, ss.nAng, ss.nYaw);
    pObs.assign(1.0);
    if (m_check_distances) {
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

//    au::grid<3, double> pReach(ss.nDist, ss.nAng, ss.nYaw);
    if (m_check_reach) {
        for (int i = 0; i < ss.nDist; ++i) {
        for (int j = 0; j < ss.nAng; ++j) {
        for (int k = 0; k < ss.nYaw; ++k) {
            if (!bTotMax(i, j, k)) {
                continue;
            }

            if (!m_reachable_table(i, j, k)) {
                bTotMax(i, j, k) = false;
            }
        }
        }
        }
    }

    au::grid<3, double> pTot;
    multiplyProbabilities(pGrasp, pObs, bTotMax, pTot);

    const double scaleDiffYglob = 0.05;
    scaleByHeadingDifference(
            ss, rob, bTotMax, object_pose, robot_pose, scaleDiffYglob, pTotThr, pTot);

    std::vector<candidate> cands;
    extractValidCandidatesSorted(ss, bTotMax, pTot, cands);

    int cand_footprint_viz_id = 0;
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


            Eigen::Affine3d T_model_robot_3d = poseEigen2ToEigen3(T_world_robot);

            geometry_msgs::PoseStamped candidate_pose;
            transformToOutputFrame(T_model_robot_3d, candidate_pose);
            candidate_pose.header.stamp = ros::Time::now();

            candidate_base_poses.push_back(candidate_pose);

            visualizeRobot(T_world_robot, (cand.pTot * 240) - 120, "base_probable_candidates", base_probcandidates_viz_id);

            Pose2D rp = poseEigen2ToSimple(poseEigen3ToEigen2(T_model_robot_3d));
            auto fp_markers = cc_->getFootprintVisualization(rp.x, rp.y, rp.yaw);
            for (auto& m : fp_markers.markers) {
                m.header.frame_id = robot_model_->getModelFrame();
                m.ns = "candidate_footprints";
                m.id = cand_footprint_viz_id++;
                m.color = rainbow(cand.pTot);
            }
            SV_SHOW_INFO(fp_markers);
        }
    }

    if (candidate_base_poses.empty()) {
        ROS_WARN("No probable candidate poses higher than a threshold!");
        ROS_WARN("Number of valid candidates: %zu", cands.size());
        return false;
    }

    ROS_INFO("Number of valid candidates: %zu", cands.size());
    ROS_INFO("Number of probable candidates: %zu", candidate_base_poses.size());

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

    const SimplePruningParams& params = m_prune_params;

    // distance-dependent "most desirable" heading offset
    double bestAngle = angles::from_degrees(60.0);

    // maximum allowable angular distance away from most
    // desirable angle configuration
    const double diffYMax = std::max(
            fabs(params.min_angle - bestAngle),
            fabs(params.max_angle - bestAngle));

    // maximum allowable linear distance away from most
    // desirable distance configuration
    const double diffDistMax = std::max(
            fabs(ss.distMin - m_best_dist),
            fabs(ss.distMin + ss.distStep * (ss.nDist - 1) - m_best_dist));

    // pGrasp: quadratic function (1 at bestDist, (1 - scalepGraspDist)^2 at borders)
    double scalepGraspDist = 0.05; // 0.1

    // pGrasp: quadratic function (1 at bestAngle, (1 - scalepGraspAngYaw)^2 at borders)
    double scalepGraspAngYaw = 0.05; // 0.1

    ///////////////////////
    // END CONFIGURATION //
    ///////////////////////

    // Some history:
    // * The best angle around the object was distance-from-the-object dependent.

    for (int i = 0; i < ss.nDist; i++) {
        const double dist = ss.distMin + ss.distStep * i;
        for (int j = 0; j < ss.nAng; ++j) {
            for (int k = 0; k < ss.nYaw; ++k) {
                const Pose2D& pose = rob(i, j, k);

                // heading of a vector from robot position to object position
                double rob2obj = atan2(obj.y - pose.y, obj.x - pose.x);

                // deviation from directly facing the gascan
                double diffAng = angles::normalize_angle(rob2obj - pose.yaw);

                // filter out all candidate poses that are not facing the object
                // within the above-defined thresholds
                if (diffAng < params.min_heading ||
                    diffAng > params.max_heading)
                {
                    bTotMax(i, j, k) = false;
                    continue;
                }

                // heading of the robot in the object frame
                const double diffY = angles::normalize_angle(obj.yaw - pose.yaw);

                // filter out all candidate poses where the difference in
                // heading between the robot and the object lies outside the
                // acceptable range
                if (diffY < params.min_angle || diffY > params.max_angle) {
                    bTotMax(i, j, k) = false;
                    continue;
                }

                pGrasp(i, j, k) =
                        quad(bestAngle, diffY, diffYMax, scalepGraspAngYaw) *
                        quad(m_best_dist, dist, diffDistMax, scalepGraspDist);
                pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);
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
    const SimplePruningParams& params = m_prune_params;

    int base_seen_viz_id = 0;

    double bestAngYaw = angles::from_degrees(-90.0);

    // pGrasp: quadratic function (1 at bestDist, (1-scalepGraspDist)^2 at borders)
    double scalepGraspDist = 0.05; // 0.1

    // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGraspAngYaw)^2 at borders)
    double scalepGraspAngYaw = 0.3; // 0.05, 0.1

    for (int i = 0; i < ss.nDist; i++) {
        for (int j = 0; j < ss.nAng; j++) {
            for (int k = 0; k < ss.nYaw; k++) {
                // object position (not orientation)
                double rob2obj = atan2(obj.y - rob(i, j, k).y, obj.x - rob(i, j, k).x);
                double diffAng = angles::normalize_angle(rob2obj - rob(i, j, k).yaw);
                double diffAngMax = fabs(ss.yawMin - params.min_heading);
                // b) object handle orientation to the right of the robot
                double diffY = angles::normalize_angle(obj.yaw - rob(i, j, k).yaw);
                double diffYMax = M_PI / 2.0;
                double diffDistMax = std::max(fabs(ss.distMin - m_best_dist_exhaustive),
                        fabs(ss.distMin + ss.distStep * (ss.nDist - 1) - m_best_dist_exhaustive));

                // left-hand side and angle of view
                if (diffAng >= params.min_heading && diffAng < params.max_heading) {
                    if (diffY >= params.min_angle && diffY <= params.max_angle) {
                        // EXCLUDE CANDIDATES WE HAVE ALREADY SEEN
                        bTotMax(i, j, k) = false;
                        Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
                        Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
                        visualizeRobot(T_world_robot, 0, "base_candidates_seen", base_seen_viz_id);
                    }
                }

                // pGrasp: quadratic function (1 at bestAngYaw, (1-scalepGrasp)^2 at borders)
                pGrasp(i, j, k) =
                        std::max(std::min(diffAng - angles::from_degrees(5.0), 0.0) / diffAngMax + 1.0, 0.0) * // ^1
                        sqrd((diffYMax - std::min(fabs(angles::normalize_angle(diffY - bestAngYaw)), fabs(angles::normalize_angle(diffY - M_PI - bestAngYaw))) * scalepGraspAngYaw) / (diffYMax)) *
                        sqrd((diffDistMax - fabs(ss.distMin + ss.distStep * i - m_best_dist_exhaustive) * scalepGraspDist) / (diffDistMax));
                pGrasp(i, j, k) = std::max(pGrasp(i, j, k), pTotThr);
            }
        }
    }
}

/// Update the probabilities of collision free, with respect to the base.
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

void RepositionBaseExecutor::pruneUnreachingStates(
    const SearchSpaceParams& ss,
    const au::grid<3, Pose2D>& rob,
    const Pose2D& object_pose,
    au::grid<3, bool>& bTotMax)
{
    int viz_idx = 0;
    visualization_msgs::MarkerArray ma;
    for (int i = 0; i < ss.nDist; ++i) {
    for (int j = 0; j < ss.nAng; ++j) {
    for (int k = 0; k < ss.nYaw; ++k) {
        if (!bTotMax(i, j, k)) {
            continue;
        }

        Eigen::Affine2d T_object_mount = poseSimpleToEigen2(rob(i, j, k));
        Eigen::Affine2d T_object_robot = T_object_mount * T_mount_robot_;

        // Transform to simple pose in grid frame
        Eigen::Affine3d T_model_robot(Eigen::Affine3d::Identity()); // = poseEigen2ToEigen3(T_world_robot);
        Eigen::Affine3d T_model_object = poseEigen2ToEigen3(T_object_robot).inverse(); //poseSimpleGascanToEigen3(object_pose);

        ROS_INFO("Test IK from robot = %s, object = %s", to_string(T_model_robot).c_str(), to_string(T_model_object).c_str());

        if (!checkIK(T_model_robot, T_model_object)) {
            bTotMax(i, j, k) = false;
        } else {
            Pose2D rp = poseEigen2ToSimple(T_object_robot);
            auto fp_markers = cc_->getFootprintVisualization(rp.x, rp.y, rp.yaw);
            for (auto& m : fp_markers.markers) {
                m.header.frame_id = robot_model_->getModelFrame();
                m.ns = "reaching_candidates";
                m.id = viz_idx++;
                m.color.r = 0;
                m.color.g = 1.0;
                m.color.b = 1.0;
                m.color.a = 1.0;
            }
            ma.markers.insert(ma.markers.end(), fp_markers.markers.begin(), fp_markers.markers.end());
        }
    }
    }
    }
    SV_SHOW_INFO(ma);
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
                Eigen::Translation2d(m_arm_front_offset_x, m_arm_front_offset_y));
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

// generate candidate poses, in the model frame, for the "front" of the robot
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

    const SearchSpaceParams& ss = m_ss_exhaustive;

    bool bCheckGrasp = true;
    bool bCheckObs = true;

    double pTotThr = 0.0; // 0.5

    ///////////////////////////
    // END PARAMETER SETTING //
    ///////////////////////////

    au::grid<3, Pose2D> rob;
    generateCandidatePoseSamples(object_pose, ss, rob);

    au::grid<3, bool> bTotMax(ss.nDist, ss.nAng, ss.nYaw);
    bTotMax.assign(true);

    au::grid<3, double> pGrasp(ss.nDist, ss.nAng, ss.nYaw);
    pGrasp.assign(1.0);
    if (bCheckGrasp) {
        computeExhaustiveGraspProbabilities(ss, rob, object_pose, pTotThr, pGrasp, bTotMax);

        const bool publish_pgrasp_map = true;
        if (publish_pgrasp_map) {
            nav_msgs::OccupancyGrid prob_grid;
            projectProbabilityMap(map, ss, object_pose, pGrasp, bTotMax, prob_grid);
            pgrasp_exhaustive_map_pub_.publish(prob_grid);
        }
    }

    au::grid<3, double> pObs(ss.nDist, ss.nAng, ss.nYaw);
    pObs.assign(1.0);
    if (bCheckObs) {
        pruneCollisionStates(ss, rob, bTotMax);
        computeArmCollisionProbabilities(ss, rob, pObs, bTotMax);
        computeBaseCollisionProbabilities(ss, rob, pObs, bTotMax);
    }

    if (m_check_reach) {
        for (int i = 0; i < ss.nDist; ++i) {
        for (int j = 0; j < ss.nAng; ++j) {
        for (int k = 0; k < ss.nYaw; ++k) {
            if (!bTotMax(i, j, k)) {
                continue;
            }

            if (!m_reachable_table(i, j, k)) {
                bTotMax(i, j, k) = false;
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

    double scaleDiffYglob = 0.05;
    scaleByHeadingDifference(
            ss, rob, bTotMax, object_pose, robot_pose, scaleDiffYglob, pTotThr, pTot);

    std::vector<candidate> cands;
    extractValidCandidatesSorted(ss, bTotMax, pTot, cands);

    // check for arm planning (at least 10 candidates)
    const bool check_arm = false;
    if (check_arm) {
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
    }

    // finally, gather all pose candidates with valid probabilities
    for (auto m = cands.begin(); m != cands.end(); ++m) {
        if (m->pTot >= pTotThr) {
            int i = m->i;
            int j = m->j;
            int k = m->k;

            Eigen::Affine2d T_world_mount = poseSimpleToEigen2(rob(i, j, k));
            Eigen::Affine2d T_world_robot = T_world_mount * T_mount_robot_;
            Eigen::Affine3d T_world_robot_3d = poseEigen2ToEigen3(T_world_robot);

            geometry_msgs::PoseStamped candidate_pose;
            transformToOutputFrame(T_world_robot_3d, candidate_pose);
            candidate_pose.header.stamp = ros::Time::now();

            candidate_base_poses.push_back(candidate_pose);

            visualizeRobot(T_world_robot, (m->pTot * 240) - 120, "base_probable_candidates", base_candidates_viz_id);
        }
    }

    if (candidate_base_poses.empty()) {
        ROS_WARN("    No probable candidate poses higher than a threshold!");
        return false;
    }

    ROS_INFO("    Number of IK feasible candidates: %d", cntTotMax);

    return true;
}

/// \brief Filter grasp candidates by kinematics and visibility
void RepositionBaseExecutor::pruneGraspCandidates(
    std::vector<Grasp>& candidates,
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
    PruneGraspsByVisibility(
            candidates,
            marker_poses,
            camera_pose,
            marker_incident_angle_threshold_rad);

    pruneGraspCandidatesIK(candidates, robot_pose);
}

/// \brief Filter grasp candidates by kinematic feasibility (inverse kinematics)
void RepositionBaseExecutor::pruneGraspCandidatesIK(
    std::vector<Grasp>& candidates,
    const Eigen::Affine3d& T_grasp_robot) const
{
    ROS_INFO("Filter %zu grasp candidate via IK", candidates.size());
    std::vector<Grasp> filtered_candidates;
    filtered_candidates.reserve(candidates.size());

    int pregrasp_ik_filter_count = 0;
    int grasp_ik_filter_count = 0;

    for (const Grasp& grasp_candidate : candidates) {
        moveit::core::RobotState robot_state(robot_model_);
        robot_state.setToDefaultValues();
        // place the robot in the grasp frame
        const moveit::core::JointModel* root_joint = robot_model_->getRootJoint();
        robot_state.setJointPositions(root_joint, T_grasp_robot);
        robot_state.update();

        ROS_DEBUG("test grasp candidate %s for ik solution", to_string(grasp_candidate.pose).c_str());

        // check for an ik solution to the pre-grasp pose
        if (!robot_state.setFromIK(manip_group_, grasp_candidate.pose)) {
            ++pregrasp_ik_filter_count;
            continue;
        }

        // check for an ik solution to the grasp pose
        if (!robot_state.setFromIK(
            manip_group_,
            grasp_candidate.pose * m_grasp_planner->pregraspToGrasp()))
        {
            ++grasp_ik_filter_count;
            continue;
        }

        // push back this grasp pose
        filtered_candidates.push_back(grasp_candidate);
        ROS_DEBUG("Pregrasp pose: %s", to_string(grasp_candidate.pose).c_str());

        // log the ik solution to the grasp pose
        std::vector<double> sol;
        robot_state.copyJointGroupPositions(manip_group_, sol);
        ROS_DEBUG("IK sol: %s", to_string(sol).c_str());
    }

    ROS_INFO("%zu/%zu reachable candidates", filtered_candidates.size(), candidates.size());
    ROS_INFO("  %d pregrasp ik failures", pregrasp_ik_filter_count);
    ROS_INFO("  %d grasp ik failures", grasp_ik_filter_count);
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
    std::vector<Grasp>& candidates)
{
    // generate grasps in the world frame
    int max_samples = m_max_grasp_samples; //100;
    if (!m_grasp_planner->planGrasps(
            "gascan", object_pose, NULL, max_samples, candidates))
    {
        ROS_ERROR("Failed to sample grasps");
        return false;
    }

    // TODO: transform to pregrasp as in grasp object executor?

    ROS_INFO("Sampled %zd grasp poses", candidates.size());
    SV_SHOW_INFO(getGraspCandidatesVisualization(candidates, "grasp_candidates_checkIKPLAN"));

    auto robot_state = currentRobotState();
    robot_state.setJointPositions(robot_model_->getRootJoint(), robot_pose);
    robot_state.update();
    const Eigen::Affine3d& camera_pose =
            robot_state.getGlobalLinkTransform(camera_view_frame_);

    ROS_DEBUG("world -> camera: %s", to_string(camera_pose).c_str());

    const double vis_angle_thresh = smpl::angles::to_radians(45.0);
    pruneGraspCandidates(candidates, robot_pose, camera_pose, vis_angle_thresh);

    ROS_INFO("Produced %zd feasible grasp poses", candidates.size());

    sort(begin(candidates), end(candidates),
            [](const Grasp& a, const Grasp& b)
            {
                return a.u < b.u;
            });

//    RankGrasps(candidates);

    SV_SHOW_INFO(getGraspCandidatesVisualization(candidates, "grasp_candidates_checkIKPLAN_filtered"));

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

Eigen::Affine3d RepositionBaseExecutor::poseSimpleGascanToEigen3(
    const Pose2D& object_pose) const
{
    const double z_on_ground = 0.156666;
    return Eigen::Translation3d(object_pose.x, object_pose.y, z_on_ground) *
        Eigen::AngleAxisd(
                angles::normalize_angle(object_pose.yaw + 0.5 * M_PI),
                Eigen::Vector3d::UnitZ());
}

/// Return whether the robot is close enough to a candidate to bother validating
/// the current pose against the arm planner
bool RepositionBaseExecutor::tryFeasibleArmCheck(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose) const
{
    Eigen::Affine2d T_world_robot = poseEigen3ToEigen2(robot_pose);
    Eigen::Affine2d T_world_mount = T_world_robot * T_mount_robot_.inverse();

    Pose2D op = poseEigen3ToSimpleGascan(object_pose);

    Eigen::Vector2d dp = Eigen::Vector2d(op.x, op.y) - T_world_mount.translation();

    const double min_dist = 0.5;
    const double max_dist = 1.0;
    const double min_heading = angles::from_degrees(-5.0);
    const double max_heading = angles::from_degrees(40.0);
    const double min_angle = angles::from_degrees(45.0);
    const double max_angle = angles::from_degrees(130.0);

    const double dist = dp.norm();

    Eigen::Rotation2Dd robot_yaw(0.0);
    robot_yaw.fromRotationMatrix(T_world_mount.rotation());
    if (dist >= min_dist && dist <= max_dist) {
        const double rob2obj = std::atan2(dp.y(), dp.x());
        double diffAng = angles::normalize_angle(rob2obj - robot_yaw.angle());
        if (diffAng >= min_heading && diffAng <= max_heading) {
            double diffY = angles::normalize_angle(op.yaw - robot_yaw.angle());
            if (diffY >= min_angle && diffY <= max_angle) {
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

    std::vector<Grasp> grasp_candidates;
    if (!generateFilteredGraspCandidates(robot_pose, object_pose, grasp_candidates)) {
        return 1;
    }

    if (grasp_candidates.empty()) {
        ROS_WARN("Failed to plan to all reachable grasps");
        return 2;
    }

    // wait for move arm action to come up
    ROS_WARN("wait for action '%s' to come up", move_arm_command_action_name_.c_str());
    if (!ReconnectActionClient(
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
        auto& grasp = grasp_candidates[gidx];

        MoveArmGoal pregrasp_goal;
        pregrasp_goal.type = MoveArmGoal::EndEffectorGoal;
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

        auto move_group_goal = BuildMoveGroupGoal(pregrasp_goal);

        auto result_cb = boost::bind(&RepositionBaseExecutor::move_arm_command_result_cb, this, _1, _2);
        move_arm_command_client_->sendGoal(move_group_goal, result_cb);

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
    std::vector<candidate>& cands)
{
    cands.clear();
    for (int j = 0; j < ss.nAng; j++) {
    for (int i = ss.nDist - 1; i >= 0; i--) {
    for (int k = ss.nYaw - 1; k >= 0; k--) {
        if (bTotMax(i, j, k)) {
            candidate cand;
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

bool RepositionBaseExecutor::checkIK(
    const Eigen::Affine3d& robot_pose,
    const Eigen::Affine3d& object_pose)
{
    std::vector<Grasp> candidates;
    if (!generateFilteredGraspCandidates(robot_pose, object_pose, candidates)) {
        return 0;
    }
    return !candidates.empty();
}

void RepositionBaseExecutor::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const moveit_msgs::MoveGroupResult::ConstPtr& result)
{
    move_arm_command_goal_state_ = state;
    move_arm_command_result_ = result;
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
                    Eigen::AngleAxisd(smpl::angles::to_radians(marker_to_link_yaw_degs), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(smpl::angles::to_radians(marker_to_link_pitch_degs), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(smpl::angles::to_radians(marker_to_link_roll_degs), Eigen::Vector3d::UnitX())).inverse();

    attached_markers_.push_back(std::move(attached_marker));
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

/// \brief Visualize world-frame grasp candidates
visualization_msgs::MarkerArray
RepositionBaseExecutor::getGraspCandidatesVisualization(
    const std::vector<Grasp>& grasps,
    const std::string& ns) const
{
    return GetGraspCandidatesVisualization(grasps, robot_model_->getModelFrame(), ns);
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
    auto robot_state = currentRobotState();
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

std_msgs::ColorRGBA RepositionBaseExecutor::rainbow(double d) const
{
    std_msgs::ColorRGBA color;
    d = clamp(d, 0.0, 1.0);
    color.a = 1.0;
    if (d < 0.5) {
        // interp between 0 = red and 0.5 = green
        double alpha = 2.0 * d;
        color.r = 1.0 - alpha;
        color.g = alpha;
        color.b = 0.0;
    } else {
        // interp between 0.5 = green and 1 => blue
        double alpha = 2.0 * (d - 0.5);
        color.r = 0.0;
        color.g = 1.0 - alpha;
        color.b = alpha;
    }
    return color;
}

#define METRICS_DEPRECATED 0
// HEURISTICALLY, minimum difference in angular coordinate wrt object, then
// farthest from the origin of object
#undef METRICS_DEPRECATED

} // namespace rcta

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hdt_base_planning");
    return rcta::RepositionBaseExecutor().run();
}
