#include <memory>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <hdt_msgs/GraspObjectCommandAction.h>
#include <hdt_msgs/RepositionBaseCommandAction.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/TeleportAndaliteCommandAction.h>

namespace RetrieveObjectExecutionStatus
{

enum Status
{
    INVALID = -1,
    INITIALIZING,
    PLANNING_REPOSITION_BASE,
    EXECUTING_REPOSITION_BASE,
    GRASPING_OBJECT,
    COMPLETE
};

std::string to_string(Status status)
{
    switch (status) {
    case INVALID:
        return "Invalid";
    case INITIALIZING:
        return "Idle";
    case PLANNING_REPOSITION_BASE:
        return "PlanningRepositionBase";
    case EXECUTING_REPOSITION_BASE:
        return "ExecutingRepositionBase";
    case GRASPING_OBJECT:
        return "GraspingObject";
    case COMPLETE:
        return "Complete";
    default:
        return "InvalidStatus";
    }
}

} // namespace RetrieveObjectExecutionStatus

class RetrieveObjectSimulator
{
public:

    RetrieveObjectSimulator();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };

    bool initialize();
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    RetrieveObjectExecutionStatus::Status last_status_;
    RetrieveObjectExecutionStatus::Status status_;

    // the initial pose of the robot in the map frame to be used for every reposition base query
    geometry_msgs::PoseStamped initial_robot_pose_;
    std::string world_frame_;

    typedef actionlib::SimpleActionClient<hdt_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionClient;
    std::unique_ptr<RepositionBaseCommandActionClient> reposition_base_command_client_;
    bool sent_reposition_base_command_;
    bool pending_reposition_base_command_;

    actionlib::SimpleClientGoalState last_reposition_base_goal_state_;
    hdt_msgs::RepositionBaseCommandResult::ConstPtr last_reposition_base_result_;

    typedef actionlib::SimpleActionClient<hdt::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionClient;
    std::unique_ptr<TeleportAndaliteCommandActionClient> teleport_andalite_command_client_;
    bool sent_teleport_andalite_command_;
    bool pending_teleport_andalite_command_;

    actionlib::SimpleClientGoalState last_teleport_andalite_goal_state_;
    hdt::TeleportAndaliteCommandResult::ConstPtr last_teleport_andalite_result_;

    typedef actionlib::SimpleActionClient<hdt_msgs::GraspObjectCommandAction> GraspObjectCommandActionClient;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool sent_grasp_object_command_;
    bool pending_grasp_object_command_;

    actionlib::SimpleClientGoalState last_grasp_object_goal_state_;
    hdt_msgs::GraspObjectCommandResult::ConstPtr last_grasp_object_result_;

    std::vector<geometry_msgs::PoseStamped> sample_object_poses_;
    std::vector<geometry_msgs::PoseStamped> candidate_base_poses_;

    geometry_msgs::PoseStamped current_sample_object_pose_;
    geometry_msgs::PoseStamped current_candidate_base_pose_;

    void reposition_base_active_cb();
    void reposition_base_feedback_cb(const hdt_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback);
    void reposition_base_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::RepositionBaseCommandResult::ConstPtr& result);

    void teleport_andalite_active_cb();
    void teleport_andalite_feedback_cb(const hdt::TeleportAndaliteCommandFeedback::ConstPtr& feedback);
    void teleport_andalite_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::TeleportAndaliteCommandResult::ConstPtr& result);

    void grasp_object_active_cb();
    void grasp_object_feedback_cb(const hdt_msgs::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::GraspObjectCommandResult::ConstPtr& result);

    std::vector<geometry_msgs::PoseStamped> create_sample_object_poses();
};

RetrieveObjectSimulator::RetrieveObjectSimulator() :
    nh_(),
    ph_("~"),
    last_status_(RetrieveObjectExecutionStatus::INVALID),
    status_(RetrieveObjectExecutionStatus::INITIALIZING),
    reposition_base_command_client_(),
    sent_reposition_base_command_(false),
    pending_reposition_base_command_(false),
    last_reposition_base_goal_state_(actionlib::SimpleClientGoalState::ABORTED),
    last_reposition_base_result_(),
    teleport_andalite_command_client_(),
    sent_teleport_andalite_command_(false),
    pending_teleport_andalite_command_(false),
    last_teleport_andalite_goal_state_(actionlib::SimpleClientGoalState::ABORTED),
    last_teleport_andalite_result_(),
    grasp_object_command_client_(),
    sent_grasp_object_command_(false),
    pending_grasp_object_command_(false),
    last_grasp_object_goal_state_(actionlib::SimpleClientGoalState::ABORTED),
    last_grasp_object_result_(),
    sample_object_poses_()
{
}

bool RetrieveObjectSimulator::initialize()
{
    // TODO: grab initial robot pose and environment from config

    double robot_initial_x;
    double robot_initial_y;
    double robot_initial_yaw;

    std::vector<double> robot_initial_joint_values;

    double room_frame_x_m;
    double room_frame_y_m;
    double room_frame_yaw_deg;
    double room_length_m;
    double room_width_m;
    std::string world_frame_name;

    if (!msg_utils::download_param(ph_, "root_frame_x_m", room_frame_x_m) ||
        !msg_utils::download_param(ph_, "root_frame_y_m", room_frame_y_m) ||
        !msg_utils::download_param(ph_, "root_frame_yaw_deg", room_frame_yaw_deg))
    {

    }
    return true;
}

int RetrieveObjectSimulator::run()
{
    /**
     * function test_retrieve_object(robot, environment):
     *     object_poses = sample_object_poses()
     *     for pose in object_poses do
     *         result, candidate_poses = reposition_base(robot, environment, pose)
     *         log_reposition_result(result, candidate_poses, pose
     *         for candidate in candidate_poses do
     *             success = teleport_andalite(robot, candidate)
     *             result = grasp_object(robot, candidate, pose) then
     *             log_result(robot, candidate, pose, result)
     *         end
     *     end
     * end
     */

    // Test suite to conduct a series of queries to the retrieve-object pipeline for robustness
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(2);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });
        ros::spinOnce();

        if (status_ != last_status_) {
            ROS_INFO("Transitioning '%s' -> '%s'", to_string(last_status_).c_str(), to_string(status_).c_str());
            last_status_ = status_;
        }

        switch (status_) {
        case RetrieveObjectExecutionStatus::INITIALIZING:
        {
            sample_object_poses_ = create_sample_object_poses();
            ROS_INFO("Sampled %zd object poses", sample_object_poses_.size());
            if (sample_object_poses_.empty()) {
                status_ = RetrieveObjectExecutionStatus::COMPLETE;
            }
            else {
                status_ = RetrieveObjectExecutionStatus::PLANNING_REPOSITION_BASE;
            }
        }   break;
        case RetrieveObjectExecutionStatus::PLANNING_REPOSITION_BASE:
        {
            if (sample_object_poses_.empty()) {
                status_ = RetrieveObjectExecutionStatus::COMPLETE;
                break;
            }

            // plan for the next object pose
            if (!sent_reposition_base_command_) {
                current_sample_object_pose_ = sample_object_poses_.back();
                sample_object_poses_.pop_back(); // consume next object poses

                static int reposition_base_goal_id = 0;

                hdt_msgs::RepositionBaseCommandGoal goal;
                goal.id = reposition_base_goal_id++;
                goal.gas_can_in_map = current_sample_object_pose_;
                goal.base_link_in_map = initial_robot_pose_;
                goal.map; // TODO: as usual

                auto result_cb = boost::bind(&RetrieveObjectSimulator::reposition_base_result_cb, this, _1, _2);
                reposition_base_command_client_->sendGoal(goal, result_cb);
                sent_reposition_base_command_ = true;
                break;
            }

            if (!pending_reposition_base_command_) {
                // command finished
                if (last_reposition_base_goal_state_ != actionlib::SimpleClientGoalState::SUCCEEDED ||
                    !last_reposition_base_result_ ||
                    last_reposition_base_result_->result == hdt_msgs::RepositionBaseCommandResult::SUCCESS ||
                    last_reposition_base_result_->candidate_base_poses.empty())
                {
                    // candidate generation for this object pose was not successful, attempt the next object pose
                    sent_reposition_base_command_ = false;
                }
                else {
                    candidate_base_poses_ = last_reposition_base_result_->candidate_base_poses;
                    status_ = RetrieveObjectExecutionStatus::EXECUTING_REPOSITION_BASE;
                }

                sent_reposition_base_command_ = false; // reset for future
                break;
            }
        }   break;
        case RetrieveObjectExecutionStatus::EXECUTING_REPOSITION_BASE:
        {
            if (candidate_base_poses_.empty()) {
                status_ = RetrieveObjectExecutionStatus::PLANNING_REPOSITION_BASE;
                break;
            }

            if (!sent_teleport_andalite_command_) {
                current_candidate_base_pose_ = candidate_base_poses_.back();
                candidate_base_poses_.pop_back();

                static int teleport_andalite_goal_id = 0;
                hdt::TeleportAndaliteCommandGoal goal;
                goal.global_pose.header.seq = teleport_andalite_goal_id++;
                goal.global_pose.header.stamp = ros::Time::now();
                goal.global_pose.header.frame_id = world_frame_;
                goal.global_pose.pose = current_candidate_base_pose_.pose;

                auto result_cb = boost::bind(&RetrieveObjectSimulator::teleport_andalite_result_cb, this, _1, _2);
                teleport_andalite_command_client_->sendGoal(goal, result_cb);
                sent_teleport_andalite_command_ = true;
                break;
            }

            if (!pending_teleport_andalite_command_) {
                // command finished
                if (last_teleport_andalite_goal_state_ != actionlib::SimpleClientGoalState::SUCCEEDED ||
                    !last_teleport_andalite_result_)
                {
                    sent_teleport_andalite_command_ = false;
                }
                else {
                    status_ = RetrieveObjectExecutionStatus::GRASPING_OBJECT;
                }

                sent_teleport_andalite_command_ = false;
                break;
            }
        }   break;
        case RetrieveObjectExecutionStatus::GRASPING_OBJECT:
        {
            if (!sent_grasp_object_command_) {
                static int grasp_object_command_goal_id = 0;
                hdt_msgs::GraspObjectCommandGoal goal;
                goal.id = grasp_object_command_goal_id++;
                goal.retry_count = 0;
                goal.gas_can_in_map = current_sample_object_pose_;
                static const std::string robot_frame_ = "base_footprint"; // TODO:
                goal.gas_can_in_base_link.header.seq = 0;
                goal.gas_can_in_base_link.header.stamp = ros::Time::now();
                goal.gas_can_in_base_link.header.frame_id = robot_frame_;

                Eigen::Affine3d world_to_robot, world_to_object, robot_to_object;
                tf::poseMsgToEigen(current_sample_object_pose_.pose, world_to_object);
                tf::poseMsgToEigen(current_candidate_base_pose_.pose, world_to_robot);
                robot_to_object = world_to_robot.inverse() * world_to_object;
                tf::poseEigenToMsg(robot_to_object, goal.gas_can_in_base_link.pose);
                goal.octomap; // TODO: as usual
                break;

                auto result_cb = boost::bind(&RetrieveObjectSimulator::grasp_object_result_cb, this, _1, _2);
                grasp_object_command_client_->sendGoal(goal, result_cb);
                sent_grasp_object_command_ = true;
                break;
            }

            if (!pending_grasp_object_command_) {
                if (last_grasp_object_goal_state_ != actionlib::SimpleClientGoalState::SUCCEEDED ||
                    !last_grasp_object_result_ ||
                    last_grasp_object_result_->result != hdt_msgs::GraspObjectCommandResult::SUCCESS)
                {
                    // log failure
                }
                else {
                    // log success
                }

                // TODO: reset arm?

                sent_grasp_object_command_ = false;
                status_ = RetrieveObjectExecutionStatus::EXECUTING_REPOSITION_BASE;
                break;
            }
        }   break;
        case RetrieveObjectExecutionStatus::COMPLETE:
        {
            ROS_INFO("Completing Retrieve Object Simulator");
        }   break;
        default:
        {
            ROS_INFO("Invalid Retrieve Object Simulator status '%s'", to_string(status_).c_str());
            ros::shutdown();
        }   break;
        }
    }

    return SUCCESS;
}

void RetrieveObjectSimulator::reposition_base_active_cb()
{
    ROS_INFO("Reposition Base Command Active!");
}

void RetrieveObjectSimulator::reposition_base_feedback_cb(const hdt_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback)
{
    ROS_INFO("Reposition Base Command Feedback!");
}

void RetrieveObjectSimulator::reposition_base_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt_msgs::RepositionBaseCommandResult::ConstPtr& result)
{
    ROS_INFO("Reposition Base Command Result!");
    last_reposition_base_goal_state_ = state;
    last_reposition_base_result_ = result;
    pending_reposition_base_command_ = false;
}

void RetrieveObjectSimulator::teleport_andalite_active_cb()
{
    ROS_INFO("Teleport Andalite Command Active!");
}

void RetrieveObjectSimulator::teleport_andalite_feedback_cb(const hdt::TeleportAndaliteCommandFeedback::ConstPtr& feedback)
{
    ROS_INFO("Teleport Andalite Command Feedback!");
}

void RetrieveObjectSimulator::teleport_andalite_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::TeleportAndaliteCommandResult::ConstPtr& result)
{
    ROS_INFO("Teleport Andalite Command Result!");
    last_teleport_andalite_goal_state_ = state;
    last_teleport_andalite_result_ = result;
    pending_teleport_andalite_command_ = false;
}

void RetrieveObjectSimulator::grasp_object_active_cb()
{
    ROS_INFO("Grasp Object Command Active!");
}

void RetrieveObjectSimulator::grasp_object_feedback_cb(const hdt_msgs::GraspObjectCommandFeedback::ConstPtr& feedback)
{
    ROS_INFO("Grasp Object Command Feedback!");
}

void RetrieveObjectSimulator::grasp_object_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt_msgs::GraspObjectCommandResult::ConstPtr& result)
{
    ROS_INFO("Grasp Object Command Result!");
    last_grasp_object_goal_state_ = state;
    last_grasp_object_result_ = result;
    pending_grasp_object_command_ = false;
}

std::vector<geometry_msgs::PoseStamped> RetrieveObjectSimulator::create_sample_object_poses()
{
    return { };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retrieve_object_simulator");
    return RetrieveObjectSimulator().run();
}
