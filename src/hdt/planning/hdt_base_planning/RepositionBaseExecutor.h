#ifndef RepositionBaseExecutor_h
#define RepositionBaseExecutor_h


#include <ros/ros.h>
#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <sstream>
#include <Eigen/Dense>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <hdt_msgs/RepositionBaseCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <hdt/common/geometry/nurb/NURB.h>
#include <tf/transform_listener.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <hdt/MoveArmCommandAction.h>


namespace RepositionBaseCandidate
{

struct candidate
{
	int i;
	int j;
	int k;
	double pTot;

	bool operator < (const candidate& cand2) const
	{
		return pTot > cand2.pTot;
	}
};

}


namespace RepositionBaseExecutionStatus
{

enum Status
{
	INVALID = -1,
	IDLE = 0,
	FAULT,
	COMPUTING_REPOSITION_BASE,
// 	GENERATING_SEARCH_SPACE,
// 	CHECKING_COLLISION,
// 	CHECKING_KINEMATICS,
// 	SELECTING_CANDIDATES,
	COMPLETING_GOAL
};

std::string to_string(Status status);

}


class RepositionBaseExecutor
{
public:

	RepositionBaseExecutor();
	bool initialize();
	int run();

	enum MainResult
	{
		SUCCESS = 0,
		FAILED_TO_INITIALIZE
	};


private:

    struct GraspCandidate
    {
        Eigen::Affine3d grasp_candidate_transform;
        double u;
        GraspCandidate(const Eigen::Affine3d& grasp_candidate_transform = Eigen::Affine3d::Identity(), double u = -1.0) :
            grasp_candidate_transform(grasp_candidate_transform),
            u(u) { }
    };
    ros::NodeHandle ph_;
    int max_grasp_candidates_;
    double gas_can_scale_;
    double pregrasp_to_grasp_offset_m_;
    bool generated_grasps_;
    bool pending_move_arm_command_;
    bool sent_move_arm_goal_;
    std::string gas_can_mesh_path_;
    Eigen::Affine3d wrist_to_tool_;
    Eigen::Affine3d grasp_to_pregrasp_;
    std::vector<GraspCandidate> reachable_grasp_candidates_;
    std::unique_ptr<Nurb<Eigen::Vector3d>> grasp_spline_;
    hdt::RobotModelConstPtr robot_model_;
    typedef actionlib::SimpleActionClient<hdt::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    std::string move_arm_command_action_name_;
    hdt::MoveArmCommandGoal last_move_arm_pregrasp_goal_;
    actionlib::SimpleClientGoalState move_arm_command_goal_state_;
    hdt::MoveArmCommandResult::ConstPtr move_arm_command_result_;
    tf::TransformListener listener_;
    ros::Publisher marker_arr_pub_;

    geometry_msgs::PoseStamped robot_pose_world_frame_;

	int checkIKPLAN();
	int checkIKPLAN(const geometry_msgs::PoseStamped& candidate_base_pose);
    std::vector<GraspCandidate> sample_grasp_candidates(const Eigen::Affine3d& robot_to_object, int num_candidates) const;
    void move_arm_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::MoveArmCommandResult::ConstPtr& result);
	void visualize_grasp_candidates(const std::vector<GraspCandidate>& grasps) const;

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
            ROS_INFO("Waited %0.3f seconds for action server '%s'...", (ros::Time::now() - start).toSec(), action_name.c_str());
        }
        return false;
    }


	double sign(double val);
	double wrapAngle(double ang);
	bool computeRobPose(double objx, double objy, double objY,  double robx0, double roby0, double robY0,  std::vector<geometry_msgs::PoseStamped>& candidate_base_poses);
	bool bComputedRobPose_;
	

	ros::NodeHandle nh_;
	nav_msgs::OccupancyGrid map_;

	std::string action_name_;
	typedef actionlib::SimpleActionServer<hdt_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
	std::unique_ptr<RepositionBaseCommandActionServer> as_;
	hdt_msgs::RepositionBaseCommandGoal::ConstPtr current_goal_;

	RepositionBaseExecutionStatus::Status status_;
	RepositionBaseExecutionStatus::Status last_status_;

	void goal_callback();
	void preempt_callback();

    uint8_t execution_status_to_feedback_status(RepositionBaseExecutionStatus::Status status);
};

#endif
