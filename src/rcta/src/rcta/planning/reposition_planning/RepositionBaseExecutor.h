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

    struct SearchSpaceParams
    {
        int nDist;
        double distMin;
        double distStep;

        int nAng;
        double angMin;
        double angStep;

        int nYaw;
        double yawMin;
        double yawStep;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher viz_pub_;
    ros::Publisher pgrasp_map_pub_;
    ros::Publisher pobs_map_pub_;

    tf::TransformListener listener_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<rcta_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
    std::unique_ptr<RepositionBaseCommandActionServer> as_;

    robot_model_loader::RobotModelLoaderPtr rml_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::JointModelGroup* manip_group_;
    std::string manip_name_;

    // TODO: monitor state to get the transform between the camera and the wrist
    moveit::core::RobotStatePtr robot_state_;

    std::string robot_frame_;
    std::string camera_view_frame_;

    Eigen::Translation2d T_mount_robot_;

    /// \name Planar Collision Constraints
    ///@{
    std::unique_ptr<XYThetaCollisionChecker> cc_;
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

    bool downloadGraspingParameters(ros::NodeHandle& nh);

    /// \name 2D and 3D Pose Type Conversions
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
    ///@}

    void computeRobotPlanarPose(
        const Eigen::Affine3d& robot_pose,
        Eigen::Affine2d& planar_pose) const;
    Pose2D poseEigen3ToSimpleGascan(const Eigen::Affine3d& object_pose) const;

    bool tryFeasibleArmCheck(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose) const;

    int checkFeasibleMoveToPregraspTrajectory(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose);

    int checkFeasibleMoveToPregraspTrajectory(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PoseStamped& object_pose);

    std::vector<GraspCandidate> generateFilteredGraspCandidates(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose);
    std::vector<GraspCandidate> sampleGraspCandidates(
        const Eigen::Affine3d& robot_to_object,
        int num_candidates) const;

    void filterGraspCandidates(
        std::vector<GraspCandidate>& candidates,
        const Eigen::Affine3d& T_camera_robot,
        double marker_incident_angle_threshold_rad) const;

    void filterGraspCandidatesIK(std::vector<GraspCandidate>& candidates) const;
    void filterGraspCandidatesVisibility(
        std::vector<GraspCandidate>& candidates,
        const Eigen::Affine3d& T_camera_robot,
        double marker_incident_angle_threshold_rad) const;

    bool download_marker_params();

    void move_arm_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const rcta::MoveArmCommandResult::ConstPtr& result);
    void visualizeGraspCandidates(
        const std::vector<GraspCandidate>& grasps,
        Eigen::Affine3d T_robot_to_map,
        std::string ns) const;

    void computeGraspProbabilities(
        const SearchSpaceParams& ss,
        const au::grid<3, Pose2D>& rob,
        const Pose2D& obj,
        double pTotThr,
        au::grid<3, double>& pGrasp,
        au::grid<3, bool>& bTotMax);

    void projectProbabilityMap(
        const nav_msgs::OccupancyGrid& map,
        const SearchSpaceParams& ss,
        const Pose2D& obj_pose,
        const au::grid<3, double>& prob,
        const au::grid<3, bool>& valid,
        nav_msgs::OccupancyGrid& grid);

    void generateCandidatePoseSamples(
        const Pose2D& obj_pose,
        const SearchSpaceParams& params,
        au::grid<3, Pose2D>& g) const;

    bool computeRobPose(
        const nav_msgs::OccupancyGrid& map,
        const Pose2D& robot_pose,
        const Pose2D& object_pose,
        std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);

    void goal_callback();
    void preempt_callback();

    uint8_t execution_status_to_feedback_status(
        RepositionBaseExecutionStatus::Status status);

    int checkIK(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose);
    bool computeRobPoseExhaustive(
        const nav_msgs::OccupancyGrid& map,
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose,
        std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);

    void visualizeRobot(
        double x,
        double y,
        double yaw,
        int hue,
        const std::string& ns,
        int& id);

    void visualizeRobot(
        const Eigen::Affine2d& pose,
        int hue,
        const std::string& ns,
        int& id);

    void visualizeRobot(
        const Eigen::Affine3d& pose,
        int hue,
        const std::string& ns,
        int& id);
};

#endif
