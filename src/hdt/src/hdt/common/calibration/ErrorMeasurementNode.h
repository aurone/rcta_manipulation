#ifndef ErrorMeasurementNode_h
#define ErrorMeasurementNode_h

#include <deque>
#include <Eigen/Dense>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hdt_description/RobotModel.h>
#include <tf/transform_listener.h>

class ErrorMeasurementNode
{
public:

    ErrorMeasurementNode();

    bool initialize();
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    hdt::RobotModelPtr robot_model_;

    ros::Subscriber ar_marker_sub_;
    ros::Subscriber joint_state_sub_;

    uint32_t tracked_marker_id_;

    std::list<ar_track_alvar_msgs::AlvarMarker::ConstPtr> marker_msgs_;
    std::list<sensor_msgs::JointState::ConstPtr> joint_state_msgs_;

    ar_track_alvar_msgs::AlvarMarker::ConstPtr last_processed_marker_msg_;
    sensor_msgs::JointState::ConstPtr last_processed_joint_state_msg_;

    tf::TransformListener listener_;

    std::string mount_frame_;
    std::string base_frame_;
    std::string camera_frame_;
    std::string gripper_frame_;
    std::string wrist_frame_;

    bool transforms_initialized_;
    Eigen::Affine3d base_frame_to_mount_frame_;
    Eigen::Affine3d base_frame_to_camera_frame_;
    Eigen::Affine3d wrist_frame_to_gripper_frame_;
    Eigen::Affine3d gripper_frame_to_marker_frame_;

    Eigen::Affine3d total_mean_error_;

    Eigen::Vector3d trans_variance_;
    double rot_variance_;

    double p_; // filter/interpolation factor

    int num_measurements_;

    void alvar_markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    // return the interpolated transform of the marker frame in the common frame
    Eigen::Affine3d interpolated_marker_pose(
        const ar_track_alvar_msgs::AlvarMarker::ConstPtr& first,
        const ar_track_alvar_msgs::AlvarMarker::ConstPtr& second,
        double alpha);

    // return the interpolated transform of the tool frame in the common frame
    Eigen::Affine3d interpolated_joint_state_pose(
        const sensor_msgs::JointState::ConstPtr& first,
        const sensor_msgs::JointState::ConstPtr& second,
        double alpha);

    // linearly interpolate between two transforms
    Eigen::Affine3d interpolate(const Eigen::Affine3d& a, const Eigen::Affine3d& b, double alpha);

    // compute the pose of the tool frame in the common frame given a joint state
    Eigen::Affine3d compute_joint_state_pose(const sensor_msgs::JointState::ConstPtr& msg);

    // compute the pose of the marker frame in the common frame given a joint state
    Eigen::Affine3d compute_marker_pose(const ar_track_alvar_msgs::AlvarMarker::ConstPtr& msg);

    // compute the difference (error) between two poses
    Eigen::Affine3d compute_pose_diff(const Eigen::Affine3d& a, const Eigen::Affine3d& b);

    Eigen::Affine3d tf_to_eigen(const tf::StampedTransform& transform);

    void updateVariance(const Eigen::Affine3d& error);

    double interp_factor();
};

#endif
