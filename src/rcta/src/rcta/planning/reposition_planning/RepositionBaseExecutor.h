#ifndef RepositionBaseExecutor_h
#define RepositionBaseExecutor_h

// standard includes
#include <signal.h>
#include <memory>
#include <sstream>
#include <stdlib.h>

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rcta_msgs/RepositionBaseCommandAction.h>
#include <ros/ros.h>
#include <smpl/debug/visualizer_ros.h>
#include <spellbook/geometry/nurb/NURB.h>
#include <spellbook/grid/grid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/Marker.h>

// project includes
#include <rcta/MoveArmAction.h>
#include <rcta/common/hdt_description/RobotModel.h>
#include <rcta/planning/grasping/gascan_grasp_planner.h>

//xytheta collision checking!
#include "xytheta_collision_checker.h"

namespace RepositionBaseCandidate {

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

} // namespace RepositionBaseCandidate

namespace RepositionBaseExecutionStatus {

enum Status
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

std::string to_string(Status status);

} // namespace RepositionBaseExecutionStatus

struct Pose2D
{
    double x;
    double y;
    double yaw;
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

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher viz_pub_;
    ros::Publisher pgrasp_map_pub_;
    ros::Publisher pobs_map_pub_;
    ros::Publisher pgrasp_exhaustive_map_pub_;

    tf::TransformListener listener_;

    sbpl::VisualizerROS viz_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<rcta_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
    std::unique_ptr<RepositionBaseCommandActionServer> as_;

    robot_model_loader::RobotModelLoaderPtr rml_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::JointModelGroup* manip_group_;
    std::string manip_name_;

    // TODO: monitor state to get the transform between the camera and the wrist
    planning_scene_monitor::PlanningSceneMonitorPtr m_scene_monitor;

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

    double m_arm_front_offset_x;
    double m_arm_front_offset_y;
    double m_arm_length;
    double m_arm_length_core;
    double m_body_length;
    double m_body_length_core;
    ///@}

    /// \name Visibility Constraints
    ///@{
    std::vector<AttachedMarker> attached_markers_;
    ///@}

    /// \name Grasping
    ///@{
    // the hand-generated grasping spline is done in model coordinates
    // and requires at least the scale to be brought into world coordinates
    rcta::GascanGraspPlanner m_grasp_planner;
    int m_max_grasp_samples;
    ///@}

    /// \name Arm Planning Constraints
    /// @{
    typedef actionlib::SimpleActionClient<rcta::MoveArmAction> MoveArmActionClient;
    std::unique_ptr<MoveArmActionClient> move_arm_command_client_;
    std::string move_arm_command_action_name_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    rcta::MoveArmResult::ConstPtr move_arm_command_result_;
    ///@}

    geometry_msgs::PoseStamped robot_pose_world_frame_;

    rcta_msgs::RepositionBaseCommandGoal::ConstPtr current_goal_;
    RepositionBaseExecutionStatus::Status status_;
    RepositionBaseExecutionStatus::Status last_status_;
    Eigen::Affine3d m_rob_pose;
    Eigen::Affine3d m_obj_pose;
    Eigen::Affine3d m_T_model_grid;
    Eigen::Affine3d m_T_grid_model;

    /// \name Initialization
    ///@{
    bool initGraspPlanner(ros::NodeHandle& nh);
    bool downloadGraspingParameters(ros::NodeHandle& nh);
    bool downloadMarkerParameters();
    ///@}

    /// \name State Monitoring
    ///@{
    const moveit::core::RobotState& currentRobotState() const;

    void processSceneUpdate(
        planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);
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
    ///@}

    /// \name Grasp Candidate Selection
    ///@{
    bool generateFilteredGraspCandidates(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose,
        std::vector<rcta::GraspCandidate>& candidates);

    void pruneGraspCandidates(
        std::vector<rcta::GraspCandidate>& candidates,
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& camera_pose,
        double marker_incident_angle_threshold_rad) const;

    void pruneGraspCandidatesIK(
        std::vector<rcta::GraspCandidate>& candidates,
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

    int checkIK(
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
        std::vector<RepositionBaseCandidate::candidate>& cands);
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
        const std::vector<rcta::GraspCandidate>& grasp,
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

    uint8_t execution_status_to_feedback_status(
        RepositionBaseExecutionStatus::Status status);

    void move_arm_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const rcta::MoveArmResult::ConstPtr& result);

    void aMetricIDontHaveTimeToMaintain();
    void anotherMetricIDontHaveTimeToMaintain();
};

#endif
