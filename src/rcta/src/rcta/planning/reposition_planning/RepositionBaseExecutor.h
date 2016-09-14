#ifndef RepositionBaseExecutor_h
#define RepositionBaseExecutor_h

// standard includes
#include <signal.h>
#include <memory>
#include <sstream>
#include <stdlib.h>

// system includes
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rcta_msgs/RepositionBaseCommandAction.h>
#include <ros/ros.h>
#include <spellbook/geometry/nurb/NURB.h>
#include <spellbook/grid/grid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/Marker.h>

// project includes
#include <rcta/common/hdt_description/RobotModel.h>
#include <rcta/MoveArmCommandAction.h>

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

/// \brief Describes the space of discrete poses with respect to another pose
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

class RepositionBaseExecutor
{
public:

    RepositionBaseExecutor();
    bool initialize();
    int run();

    enum MainResult
    {
        SUCCESS = 0, FAILED_TO_INITIALIZE
    };

private:

    struct GraspCandidate
    {
        Eigen::Affine3d grasp_candidate_transform;
        Eigen::Affine3d T_object_grasp;
        double u;

        GraspCandidate(
            const Eigen::Affine3d& grasp_candidate_transform = Eigen::Affine3d::Identity(),
            const Eigen::Affine3d& T_object_grasp = Eigen::Affine3d::Identity(),
            double u = -1.0) :
                grasp_candidate_transform(grasp_candidate_transform), T_object_grasp(T_object_grasp), u(u)
        {
        }
    };

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

    std::string action_name_;
    typedef actionlib::SimpleActionServer<rcta_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
    std::unique_ptr<RepositionBaseCommandActionServer> as_;

    robot_model_loader::RobotModelLoaderPtr rml_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::JointModelGroup* manip_group_;
    std::string manip_name_;

    // TODO: monitor state to get the transform between the camera and the wrist
    planning_scene_monitor::PlanningSceneMonitorPtr m_scene_monitor;
    moveit::core::RobotStatePtr robot_state_;

    std::string camera_view_frame_;

    Eigen::Translation2d T_mount_robot_;

    /// \name Planar Collision Constraints
    ///@{
    std::unique_ptr<XYThetaCollisionChecker> cc_;

    double m_arm_offset_x;
    double m_arm_offset_y;
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
    std::string gas_can_mesh_path_;
    double gas_can_scale_;

    int max_grasp_candidates_;
    double pregrasp_to_grasp_offset_m_;
    Eigen::Affine3d wrist_to_tool_;
    Eigen::Affine3d grasp_to_pregrasp_;
    std::unique_ptr<Nurb<Eigen::Vector3d>> grasp_spline_;
    ///@}

    /// \name Arm Planning Constraints
    /// @{
    typedef actionlib::SimpleActionClient<rcta::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    std::string move_arm_command_action_name_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    rcta::MoveArmCommandResult::ConstPtr move_arm_command_result_;
    ///@}

    geometry_msgs::PoseStamped robot_pose_world_frame_;

    rcta_msgs::RepositionBaseCommandGoal::ConstPtr current_goal_;
    RepositionBaseExecutionStatus::Status status_;
    RepositionBaseExecutionStatus::Status last_status_;

    /// \name Initialization
    ///@{
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
    std::vector<GraspCandidate> generateFilteredGraspCandidates(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose);

    std::vector<GraspCandidate> sampleGraspCandidates(
        const Eigen::Affine3d& T_grasp_object,
        int num_candidates) const;

    void filterGraspCandidates(
        std::vector<GraspCandidate>& candidates,
        const Eigen::Affine3d& T_grasp_robot,
        const Eigen::Affine3d& T_camera_robot,
        double marker_incident_angle_threshold_rad) const;

    void filterGraspCandidatesIK(
        std::vector<GraspCandidate>& candidates,
        const Eigen::Affine3d& T_grasp_robot) const;

    void filterGraspCandidatesVisibility(
        std::vector<GraspCandidate>& candidates,
        const Eigen::Affine3d& T_camera_robot,
        double marker_incident_angle_threshold_rad) const;
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

    void visualizeGraspCandidates(
        const std::vector<GraspCandidate>& grasps,
        const Eigen::Affine3d& T_world_grasp,
        const std::string& ns) const;

    void visualizeGraspCandidates(
        const std::vector<GraspCandidate>& grasp,
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
    ///@}

    void goal_callback();
    void preempt_callback();

    uint8_t execution_status_to_feedback_status(
        RepositionBaseExecutionStatus::Status status);

    void move_arm_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const rcta::MoveArmCommandResult::ConstPtr& result);

    void aMetricIDontHaveTimeToMaintain();
    void anotherMetricIDontHaveTimeToMaintain();
};

#endif
