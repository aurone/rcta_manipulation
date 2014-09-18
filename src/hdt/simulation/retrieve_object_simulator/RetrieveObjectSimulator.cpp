#include "RetrieveObjectSimulator.h"

#include <sbpl/utils/utils.h>
#include <sbpl_geometry_utils/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/utils/utils.h>
#include <hdt/common/hdt_description/RobotModel.h>

RetrieveObjectSimulator::RetrieveObjectSimulator() :
    nh_(),
    ph_("~"),
    last_status_(RetrieveObjectExecutionStatus::INVALID),
    status_(RetrieveObjectExecutionStatus::INITIALIZING),
    initial_robot_pose_(),
    world_frame_(),
    world_to_room_(),
    num_disc_x_(0),
    num_disc_y_(0),
    num_disc_yaw_(0),
    gas_can_scale_(0.12905),
    goal_object_pose_marker_pub_(),
    occupancy_grid_sub_(),
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
    sent_teleport_hdt_command_(false),
    pending_teleport_hdt_command_(false),
    last_teleport_hdt_goal_state_(actionlib::SimpleClientGoalState::ABORTED),
    last_teleport_hdt_result_(),
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
    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model from the URDF");
        return false;
    }

    double object_footprint_x_m;
    double object_footprint_y_m;
    double object_footprint_z_m;
    double object_footprint_roll_deg;
    double object_footprint_pitch_deg;
    double object_footprint_yaw_deg;
    if (!msg_utils::download_param(ph_, "object_footprint_x_m", object_footprint_x_m) ||
        !msg_utils::download_param(ph_, "object_footprint_y_m", object_footprint_y_m) ||
        !msg_utils::download_param(ph_, "object_footprint_z_m", object_footprint_z_m) ||
        !msg_utils::download_param(ph_, "object_footprint_roll_deg", object_footprint_roll_deg) ||
        !msg_utils::download_param(ph_, "object_footprint_pitch_deg", object_footprint_pitch_deg) ||
        !msg_utils::download_param(ph_, "object_footprint_yaw_deg", object_footprint_yaw_deg) )
    {
        ROS_ERROR("Failed to retrieve parameters for the footprint frame of the object");
        return false;
    }

    object_to_footprint_ =
        Eigen::Translation3d(object_footprint_x_m, object_footprint_y_m, object_footprint_z_m) *
        Eigen::AngleAxisd(sbpl::utils::ToRadians(object_footprint_yaw_deg), Eigen::Vector3d(0.0, 0.0, 1.0)) *
        Eigen::AngleAxisd(sbpl::utils::ToRadians(object_footprint_pitch_deg), Eigen::Vector3d(0.0, 1.0, 0.0)) *
        Eigen::AngleAxisd(sbpl::utils::ToRadians(object_footprint_roll_deg), Eigen::Vector3d(1.0, 0.0, 0.0));

    double robot_initial_x_m;
    double robot_initial_y_m;
    double robot_initial_yaw_deg;
    if (!msg_utils::download_param(ph_, "robot_initial_x_m", robot_initial_x_m) ||
        !msg_utils::download_param(ph_, "robot_initial_y_m", robot_initial_y_m) ||
        !msg_utils::download_param(ph_, "robot_initial_yaw_deg", robot_initial_yaw_deg))
    {
        ROS_ERROR("Failed to retrieve parameters for the initial pose of the robot");
        return false;
    }

    std::vector<double> robot_initial_joint_values;
    if (!msg_utils::download_param(ph_, "robot_initial_joint_values", robot_initial_joint_values)) {
        ROS_ERROR("Failed to retrieve 'robot_initial_joint_values' from the param server");
        return false;
    }

    std::string world_frame_name;
    double room_frame_x_m;
    double room_frame_y_m;
    double room_frame_yaw_deg;

    if (!msg_utils::download_param(ph_, "room_frame_x_m", room_frame_x_m) ||
        !msg_utils::download_param(ph_, "room_frame_y_m", room_frame_y_m) ||
        !msg_utils::download_param(ph_, "room_frame_yaw_deg", room_frame_yaw_deg) ||
        !msg_utils::download_param(ph_, "world_frame_name", world_frame_name))
    {
        ROS_ERROR("Failed to retrieve parameters for map -> room transform");
        return false;
    }

    double room_length_m;
    double room_width_m;

    if (!msg_utils::download_param(ph_, "room_length_m", room_length_m) ||
        !msg_utils::download_param(ph_, "room_width_m", room_width_m))
    {
        ROS_ERROR("Failed to retrieve parameters for the dimensions of the room");
        return false;
    }

    int num_disc_x, num_disc_y, num_disc_yaw;
    if (!msg_utils::download_param(ph_, "num_disc_x", num_disc_x) ||
        !msg_utils::download_param(ph_, "num_disc_y", num_disc_y) ||
        !msg_utils::download_param(ph_, "num_disc_yaw", num_disc_yaw))
    {
        ROS_ERROR("Failed to retrieve parameters for object pose sampling");
        return false;
    }

    if (num_disc_x < 2 || num_disc_y < 2) {
        ROS_ERROR("Linear discretizations should be greater than 1");
        return false;
    }

    if (num_disc_yaw < 1) {
        ROS_ERROR("Angular discretizations should be greater than 0");
        return false;
    }

    num_disc_x_ = num_disc_x;
    num_disc_y_ = num_disc_y;
    num_disc_yaw_ = num_disc_yaw;

    room_length_m_ = room_length_m;
    room_width_m_ = room_width_m;

    world_frame_ = world_frame_name;

    world_to_room_ =
            Eigen::Translation3d(room_frame_x_m, room_frame_y_m, 0.0) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(room_frame_yaw_deg), Eigen::Vector3d(0.0, 0.0, 1.0));

    initial_robot_pose_.header.seq = 0;
    initial_robot_pose_.header.stamp = ros::Time(0);
    initial_robot_pose_.header.frame_id = world_frame_name;
    Eigen::Affine3d initial_robot_transform =
            Eigen::Translation3d(robot_initial_x_m, robot_initial_y_m, 0.0) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(robot_initial_yaw_deg), Eigen::Vector3d(0.0, 0.0, 1.0));
    initial_robot_transform = world_to_room_ * initial_robot_transform;
    tf::poseEigenToMsg(initial_robot_transform, initial_robot_pose_.pose);

    initial_robot_joint_values_ = robot_initial_joint_values;

    ROS_WARN("Waiting for occupancy grid message...");
    occupancy_grid_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("fixed_occupancy_grid", 5, &RetrieveObjectSimulator::occupancy_grid_cb, this);
    while (!last_occupancy_grid_) {
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO(" -> Received occupancy grid. Shutting down subscriber...");
    occupancy_grid_sub_.shutdown();

    goal_object_pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 5);

    return true;
}

int RetrieveObjectSimulator::run()
{
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(2);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });
        ros::spinOnce();

        ros::Time now = ros::Time::now();

        if (status_ != last_status_) {
            ROS_INFO("Transitioning '%s' -> '%s'", to_string(last_status_).c_str(), to_string(status_).c_str());
            last_status_ = status_;
        }

        switch (status_) {
        case RetrieveObjectExecutionStatus::INITIALIZING:
        {
            sample_object_poses_ = create_sample_object_poses();

            sample_object_poses_ = collision_check_object_poses(sample_object_poses_);

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

                Eigen::Affine3d room_to_object;
                tf::poseMsgToEigen(current_sample_object_pose_.pose, room_to_object);
                Eigen::Affine3d world_to_object = world_to_room_ * room_to_object;

                geometry_msgs::PoseStamped object_pose_world_frame;
                object_pose_world_frame.header.seq = 0;
                object_pose_world_frame.header.stamp = ros::Time::now();
                object_pose_world_frame.header.frame_id = world_frame_;
                tf::poseEigenToMsg(world_to_object, object_pose_world_frame.pose);
                publish_gas_canister_marker(object_pose_world_frame);

                static int reposition_base_goal_id = 0;

                hdt_msgs::RepositionBaseCommandGoal goal;
                goal.id = reposition_base_goal_id++;
                goal.gas_can_in_map.header.seq = 0;
                goal.gas_can_in_map.header.stamp = now;
                goal.gas_can_in_map.header.frame_id = world_frame_;
                tf::poseEigenToMsg(world_to_object, goal.gas_can_in_map.pose);
                goal.base_link_in_map = initial_robot_pose_;
                assert((bool)last_occupancy_grid_);
                goal.map = *last_occupancy_grid_;

                if (!wait_for_action_server(
                        reposition_base_command_client_,
                        "reposition_base_command",
                        ros::Duration(0.5),
                        ros::Duration(5.0)))
                {
                    ROS_WARN("   -> Skipping sample object pose. Failed to connect to action server");
                    sent_reposition_base_command_ = false;
                    break; ///< skip planning for this object pose
                }

                ROS_WARN("  Repositioning Base around %s", to_string(current_sample_object_pose_.pose).c_str());

                auto result_cb = boost::bind(&RetrieveObjectSimulator::reposition_base_result_cb, this, _1, _2);
                reposition_base_command_client_->sendGoal(goal, result_cb);
                sent_reposition_base_command_ = true;
                pending_reposition_base_command_ = true;
                break;
            }

            if (!pending_reposition_base_command_) {
                // command finished
                if (last_reposition_base_goal_state_ != actionlib::SimpleClientGoalState::SUCCEEDED ||
                    !last_reposition_base_result_ ||
                    last_reposition_base_result_->result != hdt_msgs::RepositionBaseCommandResult::SUCCESS ||
                    last_reposition_base_result_->candidate_base_poses.empty())
                {
                    // candidate generation for this object pose was not successful, attempt the next object pose
                    ROS_WARN("    Reposition Base Command was not successful");
                    ROS_WARN("     -> Goal State: %s", last_reposition_base_goal_state_.toString().c_str());
                    ROS_WARN("     -> Last Result Ptr: %p", last_reposition_base_result_.get());
                    ROS_WARN("     -> Last Result Success: %s",
                            boolstr(last_reposition_base_result_ && last_reposition_base_result_->result == hdt_msgs::RepositionBaseCommandResult::SUCCESS));
                    ROS_WARN("     -> Last Result Candidates: %zd",
                            last_reposition_base_result_ ? last_reposition_base_result_->candidate_base_poses.size() : 0);
                    sent_reposition_base_command_ = false;
                }
                else {
                    ROS_INFO("    Reposition Base Command finished with %zd candidate base poses",
                            last_reposition_base_result_->candidate_base_poses.size());
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

                ////////////////////////////////////////////////////////////////////////////////
                // Send Teleport Andalite Command
                ////////////////////////////////////////////////////////////////////////////////

                // TODO: get the current base pose from tf instead of the ideal pose here (not that it's hardly any different)
                static int teleport_andalite_goal_id = 0;
                hdt::TeleportAndaliteCommandGoal goal;
                goal.global_pose.header.seq = teleport_andalite_goal_id++;
                goal.global_pose.header.stamp = now;
                goal.global_pose.header.frame_id = world_frame_;
                goal.global_pose.pose = current_candidate_base_pose_.pose;

                if (!wait_for_action_server(
                        teleport_andalite_command_client_,
                        "teleport_andalite_command",
                        ros::Duration(0.5),
                        ros::Duration(5.0)))
                {
                    ROS_WARN(" -> Skipping candidate base pose. Failed to connect to action server");
                    sent_teleport_andalite_command_ = false;
                    break; ///< skip planning for this object pose
                }

                ROS_WARN("  Teleporting Andalite to %s", to_string(current_candidate_base_pose_.pose).c_str());

                auto result_cb = boost::bind(&RetrieveObjectSimulator::teleport_andalite_result_cb, this, _1, _2);
                teleport_andalite_command_client_->sendGoal(goal, result_cb);
                sent_teleport_andalite_command_ = true;
                pending_teleport_andalite_command_ = true;

                ////////////////////////////////////////////////////////////////////////////////
                // Send Teleport HDT Command
                ////////////////////////////////////////////////////////////////////////////////

                static int teleport_hdt_goal_id = 0;
                hdt::TeleportHDTCommandGoal hdt_goal;
                hdt_goal.joint_state.name = robot_model_->joint_names();
                hdt_goal.joint_state.position = initial_robot_joint_values_;
                hdt_goal.joint_state.velocity = std::vector<double>(robot_model_->joint_names().size(), 0.0);
                hdt_goal.joint_state.effort = std::vector<double>(robot_model_->joint_names().size(), 0.0);

                if (!wait_for_action_server(
                        teleport_hdt_command_client_,
                        "teleport_hdt_command",
                        ros::Duration(0.5),
                        ros::Duration(5.0)))
                {
                    ROS_WARN(" -> Skipping candidate base pose. Failed to connect to action server");
                    sent_teleport_andalite_command_ = false;
                    sent_teleport_hdt_command_ = false;
                    break;
                }

                ROS_WARN("  Teleporting HDT to %s", to_string(initial_robot_joint_values_).c_str());

                auto hdt_result_cb = boost::bind(&RetrieveObjectSimulator::teleport_hdt_result_cb, this, _1, _2);
                teleport_hdt_command_client_->sendGoal(hdt_goal, hdt_result_cb);
                sent_teleport_hdt_command_ = true;
                pending_teleport_hdt_command_ = true;

                break;
            }

            if (!pending_teleport_andalite_command_ && !pending_teleport_hdt_command_) {
                if (last_teleport_andalite_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    last_teleport_andalite_result_ &&
                    last_teleport_hdt_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    last_teleport_hdt_result_)
                {
                    status_ = RetrieveObjectExecutionStatus::GRASPING_OBJECT;
                }

                sent_teleport_andalite_command_ = false;
                sent_teleport_hdt_command_ = false;
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

                goal.gas_can_in_map.header.seq = 0;
                goal.gas_can_in_map.header.stamp = now;
                goal.gas_can_in_map.header.frame_id = world_frame_;
                Eigen::Affine3d room_to_object;
                tf::poseMsgToEigen(current_sample_object_pose_.pose, room_to_object);
                Eigen::Affine3d world_to_object = world_to_room_ * room_to_object;
                tf::poseEigenToMsg(world_to_object, goal.gas_can_in_map.pose);

                goal.gas_can_in_base_link.header.seq = 0;
                goal.gas_can_in_base_link.header.stamp = now;
                static const std::string robot_frame_ = "base_footprint"; // TODO:
                goal.gas_can_in_base_link.header.frame_id = robot_frame_;
                Eigen::Affine3d world_to_robot,robot_to_object;
                tf::poseMsgToEigen(current_candidate_base_pose_.pose, world_to_robot);
                robot_to_object = world_to_robot.inverse() * world_to_object;
                tf::poseEigenToMsg(robot_to_object, goal.gas_can_in_base_link.pose);

                goal.octomap; // TODO: as usual

                if (!wait_for_action_server(
                        grasp_object_command_client_,
                        "grasp_object_command",
                        ros::Duration(0.5),
                        ros::Duration(5.0)))
                {
                    ROS_WARN(" -> Skipping grasp from this candidate base pose. Failed to connect to action server");
                    sent_grasp_object_command_ = false;
                    status_ = RetrieveObjectExecutionStatus::EXECUTING_REPOSITION_BASE;
                    break; ///< skip planning for this object pose
                }

                ROS_INFO("  Grasping Object at:");
                ROS_INFO("    [world]: %s", to_string(current_sample_object_pose_.pose).c_str());
                ROS_INFO("    [robot]: %s", to_string(goal.gas_can_in_base_link.pose).c_str());

                auto result_cb = boost::bind(&RetrieveObjectSimulator::grasp_object_result_cb, this, _1, _2);
                grasp_object_command_client_->sendGoal(goal, result_cb);
                sent_grasp_object_command_ = true;
                pending_grasp_object_command_ = true;
                break;
            }

            if (!pending_grasp_object_command_) {
                if (last_grasp_object_goal_state_ != actionlib::SimpleClientGoalState::SUCCEEDED ||
                    !last_grasp_object_result_ ||
                    last_grasp_object_result_->result != hdt_msgs::GraspObjectCommandResult::SUCCESS)
                {
                    // log failure
                    ROS_WARN("    Grasp Object Command failed");
                }
                else {
                    // log success
                    ROS_INFO("    Grasp Object Command succeeded");
                }

                sent_grasp_object_command_ = false;
                status_ = RetrieveObjectExecutionStatus::EXECUTING_REPOSITION_BASE;
                break;
            }
        }   break;
        case RetrieveObjectExecutionStatus::COMPLETE:
        {
            ROS_INFO("Completing Retrieve Object Simulator");
            ros::shutdown();
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
    ROS_INFO("  Reposition Base Command Result!");
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
    ROS_INFO("  Teleport Andalite Command Result!");
    last_teleport_andalite_goal_state_ = state;
    last_teleport_andalite_result_ = result;
    pending_teleport_andalite_command_ = false;
}

void RetrieveObjectSimulator::teleport_hdt_active_cb()
{
    ROS_INFO("Teleport HDT Command Active!");
}

void RetrieveObjectSimulator::teleport_hdt_feedback_cb(const hdt::TeleportHDTCommandFeedback::ConstPtr& feedback)
{
    ROS_INFO("Teleport HDT Command Feedback");
}

void RetrieveObjectSimulator::teleport_hdt_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::TeleportHDTCommandResult::ConstPtr& result)
{
    ROS_INFO("  Teleport HDT Command Result!");
    last_teleport_hdt_goal_state_ = state;
    last_teleport_hdt_result_ = result;
    pending_teleport_hdt_command_ = false;
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
    ROS_INFO("  Grasp Object Command Result!");
    last_grasp_object_goal_state_ = state;
    last_grasp_object_result_ = result;
    pending_grasp_object_command_ = false;
}

std::vector<geometry_msgs::PoseStamped> RetrieveObjectSimulator::create_sample_object_poses() const
{
    std::vector<geometry_msgs::PoseStamped> sample_poses;

    std::size_t num_samples = num_disc_x_ * num_disc_y_ * num_disc_yaw_;
    sample_poses.reserve(num_samples);

    for (int x = 0; x < num_disc_x_; ++x) {
        for (int y = 0; y < num_disc_y_; ++y) {
            for (int yaw = 0; yaw < num_disc_yaw_; ++yaw) {
                double object_x = 0.0 + (x * room_length_m_ / (num_disc_x_ - 1));
                double object_y = 0.0 + (y * room_width_m_ / (num_disc_y_ - 1));
                double object_yaw = (yaw * 2.0 * M_PI) / num_disc_yaw_;

                Eigen::Affine3d room_to_footprint =
                        Eigen::Translation3d(object_x, object_y, 0.0) *
                        Eigen::AngleAxisd(object_yaw, Eigen::Vector3d(0.0, 0.0, 1.0));

                Eigen::Affine3d room_to_object = room_to_footprint * object_to_footprint_.inverse();

                geometry_msgs::PoseStamped object_pose;
                object_pose.header.seq = 0;
                object_pose.header.stamp = ros::Time(0);
                object_pose.header.frame_id = "room";
                tf::poseEigenToMsg(room_to_object, object_pose.pose);
                sample_poses.push_back(object_pose);
            }
        }
    }

    std::reverse(sample_poses.begin(), sample_poses.end());

    return sample_poses;
}

void RetrieveObjectSimulator::occupancy_grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    last_occupancy_grid_ = msg;
}

std::vector<geometry_msgs::PoseStamped>
RetrieveObjectSimulator::collision_check_object_poses(const std::vector<geometry_msgs::PoseStamped>& sample_poses)
{
    CollisionModel2 collision_model;
    collision_model.set_occupancy_grid(last_occupancy_grid_);

    std::vector<geometry_msgs::PoseStamped> valid_samples;
    const double object_radius_m = 6.52 * 2.54 * 0.01; // 6.52 inches
    ROS_INFO("Estimated Object Radius: %0.3f", object_radius_m);
    for (const geometry_msgs::PoseStamped& sample : sample_poses) {
        Circle sample_circle;

        Eigen::Affine3d room_to_object;
        tf::poseMsgToEigen(sample.pose, room_to_object);
        Eigen::Affine3d world_to_object = world_to_room_ * room_to_object;
        geometry_msgs::Pose object_pose_in_world;
        tf::poseEigenToMsg(world_to_object, object_pose_in_world);

        sample_circle.x = object_pose_in_world.position.x;
        sample_circle.y = object_pose_in_world.position.y;
        sample_circle.radius = object_radius_m;
        if (collision_model.check_obstacle(sample_circle)) {
            valid_samples.push_back(sample);
        }
    }

    ROS_INFO("%zd of %zd samples are valid", valid_samples.size(), sample_poses.size());

    return valid_samples;
}

void RetrieveObjectSimulator::publish_gas_canister_marker(const geometry_msgs::PoseStamped& pose)
{
    visualization_msgs::Marker object_marker;
    object_marker.header.seq = 0;
    object_marker.header.stamp = ros::Time::now();
    object_marker.header.frame_id = pose.header.frame_id;
    object_marker.ns = "goal_object_pose";
    object_marker.id = 0;
    object_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    object_marker.action = visualization_msgs::Marker::ADD;
    object_marker.pose = pose.pose;
    object_marker.scale = geometry_msgs::CreateVector3(gas_can_scale_, gas_can_scale_, gas_can_scale_);
    object_marker.color = std_msgs::CreateColorRGBA(0.2f, 1.0f, 0.2f, 0.9f);
    object_marker.lifetime = ros::Duration(0);
    object_marker.frame_locked = false;
    object_marker.mesh_resource = "package://hdt/resource/meshes/gastank/clean_small_gastank.obj";
    object_marker.mesh_use_embedded_materials = false;
    goal_object_pose_marker_pub_.publish(object_marker);
}
