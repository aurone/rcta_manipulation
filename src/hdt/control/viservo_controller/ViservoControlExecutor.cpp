#include "ViservoControlExecutor.h"

#include <hdt/common/msg_utils/msg_utils.h>

ViservoControlExecutor::ViservoControlExecutor() :
    nh_(),
    ph_("~"),
    action_name_("viservo_command"),
    as_(),
    joint_command_pub_(),
    joint_states_sub_()
{
}

int ViservoControlExecutor::run()
{
    as_.reset(new ViservoCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_WARN("Failed to instantiated Viservo Command Action Server");
        return FAILED_TO_INITIALIZE;
    }

    as_->registerGoalCallback(boost::bind(&ViservoControlExecutor::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&ViservoControlExecutor::preempt_callback, this));

    double marker_to_wrist_x;
    double marker_to_wrist_y;
    double marker_to_wrist_z;
    double marker_to_wrist_roll;
    double marker_to_wrist_pitch;
    double marker_to_wrist_yaw;

    if (!ph_.getParam("marker_to_wrist_x", marker_to_wrist_x) ||
        !ph_.getParam("marker_to_wrist_y", marker_to_wrist_y) ||
        !ph_.getParam("marker_to_wrist_z", marker_to_wrist_z) ||
        !ph_.getParam("marker_to_wrist_roll", marker_to_wrist_roll) ||
        !ph_.getParam("marker_to_wrist_pitch", marker_to_wrist_pitch) ||
        !ph_.getParam("marker_to_wrist_yaw", marker_to_wrist_yaw))
    {
        ROS_ERROR("Failed to retrieve marker-to-wrist offset parameters");
        return FAILED_TO_INITIALIZE;
    }

    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return FAILED_TO_INITIALIZE;
    }

    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to instantiate Robot Model");
        return FAILED_TO_INITIALIZE;
    }

    joint_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    joint_states_sub_ = nh_.subscribe("/joint_states", 1, &ViservoControlExecutor::joint_states_cb, this);
    ar_marker_sub_ = nh_.subscribe("/ar_pose_marker", 1, &ViservoControlExecutor::ar_markers_cb, this);

    ros::Rate executive_rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();

        if (!as_->isActive()) {
            executive_rate.sleep();
            continue;
        }

        // when receiving goal
        //     1. make sure that the markers are in view of the camera


        // incorporate the newest marker measurement and estimate the current
        // wrist pose based off of that and the last two joints

        // check goal stati
        //     1. lost track of the marker
        //     2. check whether the estimated wrist pose has reached the goal within the specified tolerance
        //     3. check whether the wrist has moved too far from the goal (and the canonical path follower should retry)

        void update_wrist_pose_estimate();

        hdt::ViservoCommandResult result;
        if (!reached_goal()) {
            result.result = hdt::ViservoCommandResult::SUCCESS;
            as_->setSucceeded(result);
        }

        if (lost_marker()) {
            result.result = hdt::ViservoCommandResult::LOST_MARKER;
            as_->setAborted(result);
        }

        if (moved_too_far()) {
            result.result = hdt::ViservoCommandResult::MOVED_TOO_FAR;
            as_->setAborted(result);
        }

        // TODO: Attach markers to both the gripper and the forearm
        //       Use the forearm plus the last few joints as one measurement for the current pose of the end effector
        //       Use the gripper markers as another measurement for the current pose of the end effector
        //       Have some filtering mechanism to determine the most confident current pose

        // overview:
        //
        //     receive a goal pose in the frame of the camera for the tracked marker bundle to achieve
        //     the pose is the pose of the representative marker in the bundle
        //     translate this pose into the wrist frame of the arm
        //     apply P control to determine the direction that the wrist should travel
        //     apply the jacobian in the direction of the error to derive a joint state that moves the arm toward the goal
        //     send that joint state to the arm controller

        Eigen::Affine3d wrist_transform_estimate; // in camera frame
        tf::poseMsgToEigen(wrist_pose_estimate_, wrist_transform_estimate);

        Eigen::Affine3d goal_wrist_transform; // in camera frame
        tf::poseMsgToEigen(current_goal_->goal_pose, goal_wrist_transform);

        Eigen::Affine3d error = msg_utils::transform_diff(goal_wrist_transform, wrist_transform_estimate);

        Eigen::AngleAxisd aa_error(error.rotation());
        double angle_error = aa_error.angle();

        Eigen::Vector3d verror(error.translation());

        // FOR NOW: disallow motions if the joints will not be able to reach their target positions within a reasonable amount of time

        trajectory_msgs::JointTrajectory traj_cmd;
        traj_cmd.points = std::vector<double>(7, 0.0); // TODO: fill out the real deal here
        joint_command_pub_.publish(traj_cmd);

        tf::StampedTransform transform;
        const std::string& target_frame = camera_frame_;
        const std::string& source_frame = wrist_frame_;
        try {
            listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("I hate TF");

        }

        executive_rate.sleep();
    }
    return SUCCESS;
}

void ViservoControlExecutor::goal_callback()
{
    current_goal_ = as_->acceptNewGoal();
}

void ViservoControlExecutor::preempt_callback()
{
    // TODO: something important, like preempt the goal
}

void ViservoControlExecutor::joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg)
{

}

void ViservoControlExecutor::ar_markers_cb(const ar_track_alvar::AlvarMarkers& msg)
{

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "viservo_control_executor");

    ViservoControlExecutor executor;
    return executor.run();
}
