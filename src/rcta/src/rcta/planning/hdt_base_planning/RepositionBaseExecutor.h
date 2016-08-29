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
#include <rcta_msgs/RepositionBaseCommandAction.h>
#include <ros/ros.h>
#include <spellbook/geometry/nurb/NURB.h>
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

    moveit::core::RobotModelPtr robot_model_;
    moveit::core::JointModelGroup* manipulator_group_;
    std::string manipulator_group_name_;

    std::string robot_frame_;
    std::string kinematics_frame_;
    std::string camera_view_frame_;

    //visualizations
//    HDTViz viz;

    //xytheta collision checking
    std::unique_ptr<XYThetaCollisionChecker> cc_;

    Eigen::Affine3d T_robot_to_kinematics;

    std::vector<AttachedMarker> attached_markers_;

    // pretend that these aren't required by the grasp planner...
    // the hand-generated grasping spline is done in model coordinates
    // and requires at least the scale to be brought into world coordinates
    std::string gas_can_mesh_path_;
    double gas_can_scale_;

    /// \name Grasping
    ///@{
    int max_grasp_candidates_;
    double pregrasp_to_grasp_offset_m_;
    Eigen::Affine3d wrist_to_tool_;
    Eigen::Affine3d grasp_to_pregrasp_;
    std::unique_ptr<Nurb<Eigen::Vector3d>> grasp_spline_;
    ///@}

    bool generated_grasps_;
    bool pending_move_arm_command_;
    bool sent_move_arm_goal_;
    typedef actionlib::SimpleActionClient<rcta::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    std::string move_arm_command_action_name_;
    rcta::MoveArmCommandGoal last_move_arm_pregrasp_goal_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    rcta::MoveArmCommandResult::ConstPtr move_arm_command_result_;
    tf::TransformListener listener_;
    ros::Publisher marker_arr_pub_;

    geometry_msgs::PoseStamped robot_pose_world_frame_;

    bool bComputedRobPose_;
    nav_msgs::OccupancyGrid map_;
    std::string action_name_;
    typedef actionlib::SimpleActionServer<rcta_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
    std::unique_ptr<RepositionBaseCommandActionServer> as_;
    rcta_msgs::RepositionBaseCommandGoal::ConstPtr current_goal_;
    RepositionBaseExecutionStatus::Status status_;
    RepositionBaseExecutionStatus::Status last_status_;

    bool downloadGraspingParameters(ros::NodeHandle& nh);

    void computeRobotPlanarPose(const Eigen::Affine3d& robot_pose, Eigen::Affine2d& planar_pose) const;
    void computeObjectPlanarPose(const Eigen::Affine3d& object_pose, Eigen::Affine2d& planar_pose) const;

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
    std::vector<GraspCandidate> sample_grasp_candidates(
        const Eigen::Affine3d& robot_to_object,
        int num_candidates) const;

    void filter_grasp_candidates(
        std::vector<GraspCandidate>& candidates,
        const Eigen::Affine3d& T_kinematics_robot,
        const Eigen::Affine3d& T_camera_robot,
        double marker_incident_angle_threshold_rad) const;

    bool download_marker_params();

    void move_arm_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const rcta::MoveArmCommandResult::ConstPtr& result);
    void visualize_grasp_candidates(
        const std::vector<GraspCandidate>& grasps,
        Eigen::Affine3d T_robot_to_map,
        std::string ns) const;

    template <typename ActionType>
    bool wait_for_action_server(
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

    bool computeRobPose(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose,
        std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);

    void goal_callback();
    void preempt_callback();

    uint8_t execution_status_to_feedback_status(RepositionBaseExecutionStatus::Status status);

    int checkIK(const geometry_msgs::PoseStamped& gas_can_pose, const geometry_msgs::PoseStamped& candidate_base_pose);
    int checkPLAN(
        const geometry_msgs::PoseStamped& gas_can_pose,
        const geometry_msgs::PoseStamped& candidate_base_pose);
    bool computeRobPoseExhaustive(
        const Eigen::Affine3d& robot_pose,
        const Eigen::Affine3d& object_pose,
        std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);
};

#endif
