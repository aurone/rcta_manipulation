#ifndef ViservoControlExecutor_h
#define ViservoControlExecutor_h

#include <memory>
#include <mutex>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <hdt/ViservoCommandAction.h>
#include <hdt/common/hdt_description/RobotModel.h>

class ViservoControlExecutor
{
public:

    ViservoControlExecutor();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    hdt::RobotModelPtr robot_model_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<hdt::ViservoCommandAction> ViservoCommandActionServer;
    std::unique_ptr<ViservoCommandActionServer> as_;

    ros::Publisher joint_command_pub_;

    ros::Subscriber joint_states_sub_;
    ros::Subscriber ar_marker_sub_;

    std::mutex msg_mutex;

    hdt::ViservoCommandGoal::ConstPtr current_goal_;

    sensor_msgs::JointState::ConstPtr last_joint_state_msg_;
    ar_track_alvar::AlvarMarkers::ConstPtr last_ar_markers_msg_;

    bool marker_validity_timeout_;

    geometry_msgs::Pose wrist_pose_estimate_;

    /// maximum velocities in workspace and in joint space
    double max_translational_velocity_mps_;
    double max_rotational_velocity_rps_;
    std::vector<double> max_joint_velocities_rps_;

    tf::TransformListener listener_;

    std::string camera_frame_;
    std::string wrist_frame_;

    void goal_callback();
    void preempt_callback();
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg);
    void ar_markers_cb(const ar_track_alvar::AlvarMarkers& msg);

    bool lost_marker() const;
    bool reached_goal() const;
    bool moved_too_far() const;

    void update_wrist_pose_estimate();
};

#endif
