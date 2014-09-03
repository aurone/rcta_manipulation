#include "GraspObjectExecutor.h"

#include <eigen_conversions/eigen_msg.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/msg_utils/msg_utils.h>

namespace GraspObjectExecutionStatus
{

std::string to_string(Status status)
{
    switch (status) {
    case IDLE:
        return "Idle";
    case PLANNING_ARM_MOTION_TO_PREGRASP:
        return "PlanningArmMotionToPregrasp";
    case EXECUTING_ARM_MOTION_TO_PREGRASP:
        return "ExecutingArmMotionToPregrasp";
    case EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return "ExecutingVisualServoMotionToPregrasp";
    case EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return "ExecutingVisualServoMotionToGrasp";
    case GRASPING_OBJECT:
        return "GraspingObject";
    case PLANNING_ARM_MOTION_TO_STOW_POSITION:
        return "PlanningArmMotionToStowPosition";
    case EXECUTING_ARM_MOTION_TO_STOW_POSITION:
        return "ExecutingArmMotionToStowPosition";
    case COMPLETING_GOAL:
        return "CompletingGoal";
    default:
        return "Invalid";
    }
}

} // namespace GraspObjectExecutionStatus

GraspObjectExecutor::GraspObjectExecutor() :
    nh_(),
    ph_("~"),
    action_name_("grasp_object_command"),
    as_(),

    move_arm_command_action_name_("move_arm_command"),
    move_arm_command_client_(),
    sent_move_arm_goal_(false),
    pending_move_arm_command_(false),

    viservo_command_action_name_("viservo_command"),
    viservo_command_client_(),
    sent_viservo_command_(false),
    pending_viservo_command_(false),

    gripper_command_action_name_("gripper_controller/gripper_command_action"),
    gripper_command_client_(),
    sent_gripper_command_(false),
    pending_gripper_command_(false),

    current_goal_(),
    status_(GraspObjectExecutionStatus::INVALID),
    last_status_(GraspObjectExecutionStatus::INVALID),
    grasp_spline_(),
    gas_can_scale_(0.1),
    wrist_to_tool_(),
    pregrasp_to_grasp_offset_m_(0.0)
{
}

bool GraspObjectExecutor::initialize()
{
    std::vector<geometry_msgs::Point> control_points;
    int degree;
    if (!msg_utils::download_param(ph_, "degree", degree) ||
        !msg_utils::download_param(ph_, "control_points", control_points))
    {
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

    move_arm_command_client_.reset(new MoveArmCommandActionClient(move_arm_command_action_name_, false));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Client");
        return false;
    }

    viservo_command_client_.reset(new ViservoCommandActionClient(viservo_command_action_name_, false));
    if (!viservo_command_client_) {
        ROS_ERROR("Failed to instantiate Viservo Command Client");
        return false;
    }

    gripper_command_client_.reset(new GripperCommandActionClient(gripper_command_action_name_, false));
    if (!gripper_command_client_) {
        ROS_ERROR("Failed to instantiate Gripper Command Client");
        return false;
    }

    as_.reset(new GraspObjectActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Grasp Object Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&GraspObjectExecutor::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&GraspObjectExecutor::preempt_callback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    return true;
}

int GraspObjectExecutor::run()
{
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

    status_ = GraspObjectExecutionStatus::IDLE;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        RunUponDestruction rod([&](){ loop_rate.sleep(); });

        ros::spinOnce();

        if (status_ != last_status_) {
            ROS_INFO("Grasp Job Executor Transitioning: %s -> %s", to_string(last_status_).c_str(), to_string(status_).c_str());
            last_status_ = status_;
        }

        switch (status_) {
        case GraspObjectExecutionStatus::IDLE:
        {
            if (as_->isActive()) {
                status_ = GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP;
            }
        }   break;
        case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP:
        {
            if (!sent_move_arm_goal_) {
                ROS_INFO("Sending Move Arm Goal to pregrasp pose");
                hdt::GraspObjectFeedback feedback;
                feedback.status = execution_status_to_feedback_status(status_);
                as_->publishFeedback(feedback);
#if !TEST_STATE_MACHINE

                Eigen::Affine3d base_link_to_gas_canister;
                tf::poseMsgToEigen(current_goal_->gas_can_in_base_link.pose, base_link_to_gas_canister);

                int num_candidates = 100;
                std::vector<Eigen::Affine3d> grasp_candidates(num_candidates);
                const double min_u = 0.0;
                const double max_u = 1.0;

                // sample uniformly the position and derivative of the gas canister grasp spline
                for (int i = 0; i < num_candidates; ++i) {
                    double u = (max_u - min_u) * i / (num_candidates - 1);
                    Eigen::Vector3d sample_spline_point = (*grasp_spline_)(u);
                    Eigen::Vector3d sample_spline_deriv = grasp_spline_->deriv(u);

                    Eigen::Vector3d sample_spline_point_robot_frame =
                            base_link_to_gas_canister * Eigen::Scaling(gas_can_scale_) * sample_spline_point;

                    Eigen::Vector3d sample_spline_deriv_robot_frame =
                            base_link_to_gas_canister.rotation() * sample_spline_deriv.normalized();

                    // compute the normal to the grasp spline that most points "up" in the robot frame
                    Eigen::Vector3d up_bias(Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d grasp_dir = up_bias - up_bias.dot(sample_spline_deriv_robot_frame) * sample_spline_deriv_robot_frame;
                    grasp_dir.normalize();
                    grasp_dir *= -1.0;

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
                    Eigen::Affine3d grasp_candidate_rotation = Eigen::Translation3d(sample_spline_point_robot_frame) * grasp_rotation_matrix;

                    Eigen::Affine3d candidate_wrist_transform = grasp_candidate_rotation * wrist_to_tool_.inverse();
                }

                // TODO:
                //     1. generate grasp candidates from object pose
                //     2. filter unreachable grasp candidates
                //     3. sort grasp candidates by desirability
                //     4. send a move arm goal for the best grasp
                //     5. open gripper to prepare for visual servoing (which requires the gripper to be fully open)

                if (!wait_for_action_server(
                        move_arm_command_client_,
                        move_arm_command_action_name_,
                        ros::Duration(1.0),
                        ros::Duration(5.0)))
                {
                    ROS_WARN("Failed to connect to 'move_arm_command' action server");
                    // TODO: return something
                }

                hdt::MoveArmCommandGoal move_arm_goal;
                move_arm_goal.type = hdt::MoveArmCommandGoal::EndEffectorGoal;
                move_arm_goal.goal_pose; // FIXME: fill me out

                pending_move_arm_command_ = true;
#endif
                sent_move_arm_goal_ = true;
            }
            else if (!pending_move_arm_command_) {
                ROS_INFO("Move Arm Goal is no longer pending");
                // NOTE: short-circuiting "EXECUTING_ARM_MOTION_TO_PREGRASP" for
                // now since the move_arm action handles execution and there is
                // presently no feedback to distinguish planning vs. execution
                status_ = GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
                sent_move_arm_goal_ = false; // reset for future move arm goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_PREGRASP:
        {
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        {
            if (!sent_viservo_command_) {
                ROS_INFO("Sending Viservo Goal to pregrasp pose");
#if !TEST_STATE_MACHINE
                // TODO:
                //     send viservo goal to successful pregrasp from PLANNIG_ARM_MOTION_TO_PREGRASP phase
                pending_viservo_command_ = true;
#endif
                sent_viservo_command_ = true;
            }
            else if (!pending_viservo_command_) {
                ROS_INFO("Viservo Goal to pregrasp is no longer pending");
                status_ = GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
                sent_viservo_command_ = false; // reset for future viservo goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        {
            if (!sent_viservo_command_) {
                ROS_INFO("Sending Viservo Goal to grasp pose");
#if !TEST_STATE_MACHINE
                // TODO:
                //     send viservo goal to grasp associated with pregrasp from previous phase
                pending_viservo_command_ = true;
#endif
                sent_viservo_command_ = true;
            }
            else if (!pending_viservo_command_) {
                ROS_INFO("Viservo Goal to grasp is no longer pending");
                status_ = GraspObjectExecutionStatus::GRASPING_OBJECT;
                sent_viservo_command_ = false; // reset for future viservo goals
            }
        }   break;
        case GraspObjectExecutionStatus::GRASPING_OBJECT:
        {
            if (!sent_gripper_command_) {
                ROS_INFO("Sending Gripper Goal to close gripper");
#if !TEST_STATE_MACHINE
                pending_gripper_command_ = true;
#endif
                sent_gripper_command_ = true;
            }
            else if (!pending_gripper_command_) {
                ROS_INFO("Gripper Goal to close gripper is no longer pending");
                status_ = GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_STOW_POSITION;
                sent_gripper_command_ = false; // reset for future close gripper goals
            }
        }   break;
        case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_STOW_POSITION:
        {
            if (!sent_move_arm_goal_) {
                ROS_INFO("Sending Move Arm Goal to stow position");
#if !TEST_STATE_MACHINE
                pending_move_arm_command_ = true;
#endif
                sent_move_arm_goal_ = true;
            }
            else if (!pending_move_arm_command_) {
                // NOTE: short-circuiting
                // "EXECUTING_ARM_MOTION_TO_STOW_POSITION" for now since hte
                // move arm action handles execution and there is presently no
                // feedback to distinguish planning vs. execution
                status_ = GraspObjectExecutionStatus::COMPLETING_GOAL;
                sent_move_arm_goal_ = false; // reset for future move arm goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_STOW_POSITION:
        {
        }   break;
        case GraspObjectExecutionStatus::COMPLETING_GOAL:
        {
            hdt::GraspObjectResult result;
            result.result = hdt::GraspObjectResult::SUCCESS;
            as_->setSucceeded(result);
            status_ = GraspObjectExecutionStatus::IDLE;
        }   break;
        default:
            break;
        }
    }

    return SUCCESS;
}

void GraspObjectExecutor::goal_callback()
{
    ROS_INFO("Received a new goal");
    current_goal_ = as_->acceptNewGoal();
    ROS_INFO("    Goal ID: %u", current_goal_->id);
    ROS_INFO("    Retry Count: %d", current_goal_->retryCount);
    ROS_INFO("    Gas Can Pose [map]", to_string(current_goal_->gas_can_in_map.pose).c_str());
    ROS_INFO("    Gas Can Pose [base_link]", to_string(current_goal_->gas_can_in_base_link.pose).c_str());
    ROS_INFO("    Octomap ID: %s", current_goal_->octomap.id.c_str());

    sent_move_arm_goal_ = false;
    pending_move_arm_command_ = false;

    sent_viservo_command_ = false;
    pending_viservo_command_ = false;

    sent_gripper_command_ = false;
    pending_gripper_command_ = false;
}

void GraspObjectExecutor::preempt_callback()
{

}

void GraspObjectExecutor::move_arm_command_active_cb()
{

}

void GraspObjectExecutor::move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback)
{

}

void GraspObjectExecutor::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::MoveArmCommandResult::ConstPtr& result)
{
    pending_move_arm_command_ = false;
}

void GraspObjectExecutor::viservo_command_active_cb()
{

}

void GraspObjectExecutor::viservo_command_feedback_cb(const hdt::ViservoCommandFeedback::ConstPtr& feedback)
{

}

void GraspObjectExecutor::viservo_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::ViservoCommandResult::ConstPtr& result)
{
    pending_viservo_command_ = false;
}

void GraspObjectExecutor::gripper_command_active_cb()
{

}

void GraspObjectExecutor::gripper_command_feedback_cb(const control_msgs::GripperCommandFeedback::ConstPtr& feedback)
{
}

void GraspObjectExecutor::gripper_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::GripperCommandResult& result)
{
    pending_gripper_command_ = false;
}

uint8_t GraspObjectExecutor::execution_status_to_feedback_status(GraspObjectExecutionStatus::Status status)
{
    switch (status) {
    case GraspObjectExecutionStatus::IDLE:
        return -1;
    case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP:
        return hdt::GraspObjectFeedback::EXECUTING_ARM_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_PREGRASP:
        return hdt::GraspObjectFeedback::EXECUTING_ARM_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return hdt::GraspObjectFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return hdt::GraspObjectFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
    case GraspObjectExecutionStatus::GRASPING_OBJECT:
        return hdt::GraspObjectFeedback::GRASPING_OBJECT;
    case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_STOW_POSITION:
        return hdt::GraspObjectFeedback::PLANNING_ARM_MOTION_TO_STOW;
    case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_STOW_POSITION:
        return hdt::GraspObjectFeedback::EXECUTING_ARM_MOTION_TO_STOW;
    case GraspObjectExecutionStatus::COMPLETING_GOAL:
        return -1;
    default:
        return -1;
    }
}
