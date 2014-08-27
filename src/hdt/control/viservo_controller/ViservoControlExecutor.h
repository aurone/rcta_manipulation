#ifndef ViservoControlExecutor_h
#define ViservoControlExecutor_h

#include <memory>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <hdt/ViservoCommandAction.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>

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

    typedef std::unique_ptr<sbpl_arm_planner::KDLRobotModel> KDLRobotModelPtr;
    KDLRobotModelPtr kdl_robot_model_;

    std::string action_name_;
    typedef actionlib::SimpleActionServer<hdt::ViservoCommandAction> ViservoCommandActionServer;
    std::unique_ptr<ViservoCommandActionServer> as_;

    ros::Publisher joint_command_pub_;
    ros::Publisher corrected_wrist_goal_pub_;

    ros::Subscriber joint_states_sub_;
    ros::Subscriber ar_marker_sub_;

    hdt::ViservoCommandGoal::ConstPtr current_goal_;

    sensor_msgs::JointState::ConstPtr last_joint_state_msg_;
    ar_track_alvar::AlvarMarkers::ConstPtr last_ar_markers_msg_;

    double marker_validity_timeout_;

    Eigen::Affine3d wrist_transform_estimate_;
    Eigen::Affine3d eef_transform_from_joint_state_;

    /// maximum velocities in workspace and in joint space
    double max_translational_velocity_mps_;
    double max_rotational_velocity_rps_;
    std::vector<double> deadband_joint_velocities_rps_; // velocities at which and below velocities will be truncated to 0
    std::vector<double> minimum_joint_velocities_rps_; // velocities between the deadband and this will be clamped to these values
    std::vector<double> max_joint_velocities_rps_; // velocities greater than these will be clamped to these values

    tf::TransformListener listener_;

    std::string camera_frame_;
    std::string wrist_frame_;
    std::string mount_frame_;

    struct AttachedMarker
    {
        int marker_id;
        std::string attached_link;
        Eigen::Affine3d link_to_marker;
    };

    AttachedMarker attached_marker_;

    Eigen::Vector3d goal_pos_tolerance_;
    double goal_rot_tolerance_;

    std::vector<double> last_curr_;
    std::vector<double> last_diff_;

    void goal_callback();
    void preempt_callback();
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr& msg);
    void ar_markers_cb(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

    bool lost_marker();
    bool reached_goal() const;
    bool moved_too_far() const;

    // incorporate the most recent joint state and ar marker data to come up with the best estimate for
    // the actual wrist frame in the frame of the camera, using the ar marker with hardcoded offsets to an
    // attached link and joint data for the remaining links between the attached link and the end effector
    bool update_wrist_pose_estimate();

    bool download_marker_params();

    bool get_tracked_marker_pose(Eigen::Affine3d& marker_pose);

    bool safe_joint_delta(const std::vector<double>& from, const std::vector<double>& to) const;

    void correct_joint_velocity_cmd(const std::vector<double>& from, std::vector<double>& to, double dt);
};

#endif
