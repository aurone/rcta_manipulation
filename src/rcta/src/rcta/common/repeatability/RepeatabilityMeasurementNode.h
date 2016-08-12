#ifndef RepeatabilityMeasurementNode_h
#define RepeatabilityMeasurementNode_h

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <rcta/MoveArmCommandAction.h>
#include <rcta/common/hdt_description/RobotModel.h>

inline std::string to_string(const std::tuple<int, int, int, int, int, int>& coord)
{
    std::stringstream ss;
    ss << "("
       << std::get<0>(coord) << ", "
       << std::get<1>(coord) << ", "
       << std::get<2>(coord) << ", "
       << std::get<3>(coord) << ", "
       << std::get<4>(coord) << ", "
       << std::get<5>(coord) << ")";
    return ss.str();
}

namespace std
{

// awful, i know
template <>
struct hash<tuple<int, int, int, int, int, int>>
{
    size_t operator()(const std::tuple<int, int, int, int, int, int>& t) const
    {
        size_t seed = std::hash<int>()(std::get<5>(t));
        boost::hash_combine(seed, std::hash<int>()(std::get<4>(t)));
        boost::hash_combine(seed, std::hash<int>()(std::get<3>(t)));
        boost::hash_combine(seed, std::hash<int>()(std::get<2>(t)));
        boost::hash_combine(seed, std::hash<int>()(std::get<1>(t)));
        boost::hash_combine(seed, std::hash<int>()(std::get<0>(t)));
        return seed;
    }
};

} // namespace std

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

    const_iterator begin() const { return &a0; }
    const_iterator end() const { return &a6 + 1; }

    const_iterator cbegin() const { return &a0; }
    const_iterator cend() const { return &a6 + 1; }

    double& operator[](int index) { return *(&a0 + index); }
    const double& operator[](int index) const { return *(&a0 + index); }

    std::vector<double> to_joint_vector() const
    { return std::vector<double>(&this->operator[](0), &this->operator[](7)); }
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

    /// orientation offsets from the camera frame
    double roll_offset_degs_;
    double pitch_offset_degs_;
    double yaw_offset_degs_;

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

    ros::Publisher sample_eef_pose_marker_pub_;
    ros::Publisher bounds_markers_pub_;

    ros::Subscriber joint_states_sub_;
    ros::Subscriber ar_markers_sub_;

    std::vector<hdt::JointState> egress_positions_;

    std::vector<geometry_msgs::Pose> sample_eef_poses_;

    std::string camera_frame_;
    std::string mount_frame_;

    Eigen::Affine3d camera_frame_to_tool_frame_rotation_;

    ar_track_alvar_msgs::AlvarMarkers::ConstPtr last_markers_msg_;
    sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

    typedef actionlib::SimpleActionClient<rcta::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    std::string move_arm_command_action_name_;
    bool pending_command_;

    std::string urdf_description_;
    hdt::RobotModelPtr robot_model_;

    unsigned int tracked_marker_id_;

    typedef std::tuple<int, int, int, int, int, int> DiscCoord;
    struct SamplePose
    {
        DiscCoord coord;
        geometry_msgs::Pose pose;

        bool operator==(const SamplePose& rhs) const
        {
            return coord == rhs.coord;
        }
    };

    struct SamplePoseHash
    {
        std::size_t operator()(const SamplePose& sp) const
       {
            return std::hash<std::tuple<int, int, int, int, int, int>>()(sp.coord);
        }
    };

    std::unordered_map<SamplePose, Eigen::Affine3d, SamplePoseHash> offset_means_;
    std::unordered_map<SamplePose, Eigen::Affine3d, SamplePoseHash> offset_variance_;

    void ar_markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool download_params();
    bool download_egress_positions();
    bool connect_to_move_arm_client();

    static const char* to_cstring(XmlRpc::XmlRpcValue::Type type);

    std::vector<SamplePose> generate_sample_poses();

    bool move_to_position(const hdt::JointState& position);

    bool track_marker_pose(const ros::Duration& listen_duration, geometry_msgs::Pose& pose);
    bool track_eef_pose(const ros::Duration& listen_duration, geometry_msgs::Pose& pose);

    void move_arm_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const rcta::MoveArmCommandResult::ConstPtr& result);

    void publish_triad(const Eigen::Affine3d& mount_to_eef);

    std::vector<double> gather_joint_values(const sensor_msgs::JointState& joint_state) const;

    bool write_variances_to_file();
};

#endif
