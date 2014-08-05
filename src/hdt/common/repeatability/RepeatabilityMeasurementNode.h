#ifndef RepeatabilityMeasurementNode_h
#define RepeatabilityMeasurementNode_h

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ar_track_alvar/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <ros/ros.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <hdt/common/hdt_description/RobotModel.h>

namespace hdt
{

struct JointState
{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
    double a6;

    JointState(
        double a0 = double(),
        double a1 = double(),
        double a2 = double(),
        double a3 = double(),
        double a4 = double(),
        double a5 = double(),
        double a6 = double())
    :
        a0(a0), a1(a1), a2(a2), a3(a3), a4(a4), a5(a5), a6(a6) { }

    typedef double* iterator;
    typedef const double* const_iterator;

    iterator begin() { return &a0; }
    iterator end() { return &a6 + 1; }

    const_iterator cbegin() const { return &a0; }
    const_iterator cend() const { return &a6 + 1; }

    double& operator[](int index) { return *(&a0 + index); }
    const double& operator[](int index) const { return *(&a0 + index); }
};

inline JointState::const_iterator begin(const JointState& js) { return js.cbegin(); }
inline JointState::const_iterator end(const JointState& js) { return js.cend(); }
inline JointState::iterator begin(JointState& js) { return js.begin(); }
inline JointState::iterator end(JointState& js) { return js.end(); }

inline JointState operator-(const JointState& a, const JointState& b)
{
    return JointState(a.a0 - b.a0, a.a1 - b.a1, a.a2 - b.a2, a.a3 - b.a3, a.a4 - b.a4, a.a5 - b.a5, a.a6 - b.a6);
}

std::string to_string(const JointState& js);

} // namespace hdt

class RepeatabilityMeasurementNode
{
public:

    RepeatabilityMeasurementNode();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE = 1
    };
    int run();

private:


    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    Eigen::Vector3d workspace_min_, workspace_max_;
    double roll_offset_, pitch_offset_, yaw_offset_;

    Eigen::Vector3i num_samples_;
    Eigen::Vector3d sample_res_;

    int num_roll_samples_;
    int num_pitch_samples_;
    int num_yaw_samples_;

    double roll_res_;
    double pitch_res_;
    double yaw_res_;

    tf::TransformListener listener_;
    ros::Publisher joint_cmd_pub_;

    ros::Subscriber joint_states_sub_;

    std::vector<hdt::JointState> egress_positions_;

    std::vector<geometry_msgs::Pose> sample_eef_poses_;

    std::string camera_frame_;
    std::string mount_frame_;

    Eigen::Affine3d camera_frame_to_tool_frame_rotation_;

    ar_track_alvar::AlvarMarkers::ConstPtr last_markers_msg_;
    sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

    std::string urdf_description_;
    hdt::RobotModel robot_model_;

    std::unique_ptr<distance_field::PropagationDistanceField> distance_field_;
    std::unique_ptr<sbpl_arm_planner::OccupancyGrid> grid_;
    std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace> collision_checker_;

    void ar_markers_callback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool download_params();

    static const char* to_cstring(XmlRpc::XmlRpcValue::Type type);

    std::vector<geometry_msgs::Pose> generate_sample_poses();

    bool move_to_position(const hdt::JointState& position);

    bool track_marker_pose(const ros::Duration& listen_duration, geometry_msgs::Pose& pose);
};

#endif
