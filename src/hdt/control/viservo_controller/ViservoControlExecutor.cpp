#include "ViservoControlExecutor.h"

ViservoControlExecutor::ViservoControlExecutor() :
    nh_(),
    ph_("~"),
    as_(),
    action_name_("viservo_command"),
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

    joint_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    joint_states_sub_ = nh_.subscribe("/joint_states", 1, &ViservoControlExecutor::joint_states_cb, this);

    ros::Rate executive_rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();

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

        ROS_INFO("Last AR Markers message had %zd markers", last_ar_markers_msg_->markers.size());

        trajectory_msgs::JointTrajectory traj_cmd;
        traj_cmd.points = std::vector<double>(7, 0.0); // TODO: fill out the real deal here
        joint_command_pub_.publish(traj_cmd);

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
