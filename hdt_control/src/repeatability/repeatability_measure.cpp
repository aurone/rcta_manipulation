#include <ros/ros.h>

// standard includes
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <hdt_kinematics/RobotModel.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <smpl/angles.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <rcta/MoveArmAction.h>

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

    typedef actionlib::SimpleActionClient<rcta::MoveArmAction> MoveArmActionClient;
    std::unique_ptr<MoveArmActionClient> move_arm_command_client_;
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
            const rcta::MoveArmResult::ConstPtr& result);

    void publish_triad(const Eigen::Affine3d& mount_to_eef);

    std::vector<double> gather_joint_values(const sensor_msgs::JointState& joint_state) const;

    bool write_variances_to_file();
};

std::string parseandpad(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    const int max_chars = 150 + 1;
    char buffer[max_chars] = { 0 };
    int retVal = vsnprintf(buffer, max_chars, fmt, args);

    if (retVal >= 0 && retVal < max_chars) {
        for (int r = retVal; r < max_chars - 1; ++r) {
            buffer[r] = ' ';
        }
    }
    buffer[max_chars - 1] = '\0';
    va_end(args);
    return std::string(buffer);
}

#define AU_DEBUG(fmt, ...) ROS_DEBUG("%s", parseandpad(fmt, ##__VA_ARGS__).c_str())
#define AU_INFO(fmt, ...) ROS_INFO("%s", parseandpad(fmt, ##__VA_ARGS__).c_str())
#define AU_WARN(fmt, ...) ROS_WARN("%s", parseandpad(fmt, ##__VA_ARGS__).c_str())

namespace hdt
{

std::string to_string(const hdt::JointState& js)
{
    std::stringstream ss;
    ss << "[ ";
    for (const double d : js) {
        ss << d << ' ';
    }
    ss << ']';
    return ss.str();
}

} // namespace hdt

RepeatabilityMeasurementNode::RepeatabilityMeasurementNode() :
    nh_(),
    ph_("~"),
    workspace_min_(),
    workspace_max_(),
    roll_offset_degs_(),
    pitch_offset_degs_(),
    yaw_offset_degs_(),
    num_samples_(),
    sample_res_(),
    num_roll_samples_(),
    num_pitch_samples_(),
    num_yaw_samples_(),
    roll_res_(),
    pitch_res_(),
    yaw_res_(),
    listener_(),
    joint_cmd_pub_(),
    joint_states_sub_(),
    egress_positions_(),
    sample_eef_poses_(),
    camera_frame_(),
    mount_frame_(),
    camera_frame_to_tool_frame_rotation_(),
    last_markers_msg_(),
    last_joint_state_msg_(),
    urdf_description_(),
    robot_model_(),
    pending_command_(false)
{
}

int RepeatabilityMeasurementNode::run()
{
    // 1. Create a number of known safe configurations that we can
    //    reasonably interpolate to and from for most poses in the visible
    //    workspace
    if (!download_params()) { // errors printed within
        return FAILED_TO_INITIALIZE;
    }

    if (!nh_.getParam("robot_description", urdf_description_)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from param server");
        return FAILED_TO_INITIALIZE;
    }
    if (!(robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_description_))) {
        ROS_ERROR("Failed to load Robot Model from URDF");
        return FAILED_TO_INITIALIZE;
    }

    if (!connect_to_move_arm_client()) {
        ROS_ERROR("Failed to connect to move arm client");
        return FAILED_TO_INITIALIZE;
    }

    const double ninety_rads = M_PI / 2.0;
    camera_frame_to_tool_frame_rotation_ = Eigen::Affine3d(Eigen::AngleAxisd(ninety_rads, Eigen::Vector3d::UnitZ()));

    // subscribe to joint states and ar markers
    joint_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    sample_eef_pose_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sample_pose_markers", 1);
    bounds_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("repeatability_bounds_markers", 5);
    joint_states_sub_ = nh_.subscribe("joint_states", 10, &RepeatabilityMeasurementNode::joint_states_callback, this);
    ar_markers_sub_ = nh_.subscribe("ar_pose_marker", 5, &RepeatabilityMeasurementNode::ar_markers_callback, this);

    std::vector<Eigen::Vector3d> line_vertices = {
        Eigen::Vector3d(workspace_min_(0), workspace_min_(1), workspace_min_(2)),
        Eigen::Vector3d(workspace_min_(0), workspace_min_(1), workspace_max_(2)),
        Eigen::Vector3d(workspace_min_(0), workspace_max_(1), workspace_min_(2)),
        Eigen::Vector3d(workspace_min_(0), workspace_max_(1), workspace_max_(2)),
        Eigen::Vector3d(workspace_max_(0), workspace_min_(1), workspace_min_(2)),
        Eigen::Vector3d(workspace_max_(0), workspace_min_(1), workspace_max_(2)),
        Eigen::Vector3d(workspace_max_(0), workspace_max_(1), workspace_min_(2)),
        Eigen::Vector3d(workspace_max_(0), workspace_max_(1), workspace_max_(2))
    };

    // each consecutive pair of two indices represents one line
    std::vector<int> line_indices = {
        0, 1, 0, 2, 2, 3, 3, 1, 2, 6, 3, 7, 1, 5, 0, 4, 6, 4, 4, 5, 5, 7, 6, 7
    };

    // publish bounds markers
    visualization_msgs::MarkerArray bounds_markers;

    visualization_msgs::Marker lines_marker;
    lines_marker.header.seq = 0;
    lines_marker.header.stamp = ros::Time::now();
    lines_marker.header.frame_id = camera_frame_;
    lines_marker.ns = "repeatability_bounds";
    lines_marker.id = 0;
    lines_marker.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker.action = visualization_msgs::Marker::ADD;
    lines_marker.scale.x = 0.01;
    lines_marker.scale.y = 0.01;
    lines_marker.scale.z = 0.01;

    int num_lines = line_indices.size() >> 1;
    ROS_INFO("Adding %d lines for bounds marker", num_lines);
    for (int i = 0; i < num_lines; ++i) {
        const Eigen::Vector3d& line_a = line_vertices[line_indices[2 * i]];
        const Eigen::Vector3d& line_b = line_vertices[line_indices[2 * i + 1]];

        geometry_msgs::Point point_a;
        point_a.x = line_a(0);
        point_a.y = line_a(1);
        point_a.z = line_a(2);

        geometry_msgs::Point point_b;
        point_b.x = line_b(0);
        point_b.y = line_b(1);
        point_b.z = line_b(2);

        lines_marker.points.push_back(point_a);
        lines_marker.points.push_back(point_b);

        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.a = 0.8;
        lines_marker.colors.push_back(color);
        lines_marker.colors.push_back(color);
    }

    bounds_markers.markers.push_back(lines_marker);
    ROS_INFO("Publishing marker array with %zd markers", bounds_markers.markers.size());
    ROS_INFO("    First marker has %zd points", bounds_markers.markers.front().points.size());
    bounds_markers_pub_.publish(bounds_markers);

    ros::Time then = ros::Time::now();
    while (ros::Time::now() < then + ros::Duration(5.0)) {
        ros::spinOnce();
    }

    // 2. Create a sparse sampling of visible/reachable poses for the end effector
    std::vector<SamplePose> sample_poses = generate_sample_poses();

    AU_INFO("Minimum x is %0.3f", std::min_element(sample_poses.begin(), sample_poses.end(),
            [](const SamplePose& p, const SamplePose& q)
            { return p.pose.position.x < q.pose.position.x; })->pose.position.x);
    AU_INFO("Minimum y is %0.3f", std::min_element(sample_poses.begin(), sample_poses.end(),
            [](const SamplePose& p, const SamplePose& q)
            { return p.pose.position.y < q.pose.position.y; })->pose.position.y);
    AU_INFO("Minimum z is %0.3f", std::min_element(sample_poses.begin(), sample_poses.end(),
            [](const SamplePose& p, const SamplePose& q)
            { return p.pose.position.z < q.pose.position.z; })->pose.position.z);

    AU_INFO("Generated %zd sample end effector locations within view of the camera", sample_poses.size());

    // 3. For each sample end effector pose, try to move to each ik solution
    for (const auto& sample : sample_poses)
    {
        // Transform this sample pose into the mounting frame for visualization
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(0);
        pose_stamped.header.seq = 0;
        pose_stamped.header.frame_id = camera_frame_;
        pose_stamped.pose = sample.pose;
        geometry_msgs::PoseStamped eef_pose_mount_frame;

        if (!listener_.waitForTransform(mount_frame_, camera_frame_, ros::Time::now(), ros::Duration(5.0))) {
            ROS_WARN("Unable to transform sample pose in the camera frame to the mounting frame");
            continue;
        }

        try {
            listener_.transformPose(mount_frame_, pose_stamped, eef_pose_mount_frame);
        }
        catch (const tf::TransformException& ex) {
            ROS_WARN("Failed to transform pose from frame '%s' to '%s'. Fuck TF!", camera_frame_.c_str(), mount_frame_.c_str());
        }

        ROS_DEBUG("Transformed %s (%s -> %s) = %s",
                to_string(pose_stamped.pose).c_str(), pose_stamped.header.frame_id.c_str(),
                eef_pose_mount_frame.header.frame_id.c_str(), to_string(eef_pose_mount_frame.pose).c_str());

        Eigen::Affine3d mount_to_eef;
        tf::poseMsgToEigen(eef_pose_mount_frame.pose, mount_to_eef);

        publish_triad(mount_to_eef);

        // 4. For each known safe configuration
        // move back and forth between the egress position and the ik solution until all transitions have been attempted
        for (const auto& egress_position : egress_positions_) {
            // Generate IK Solutions
            std::vector<double> seed = egress_position.to_joint_vector();
            ROS_DEBUG("Generating IK solutions starting from seed state %s", to_string(seed).c_str());
            hdt::IKSolutionGenerator iksols =
                    robot_model_->search_all_ik_solutions(mount_to_eef, seed, sbpl::angles::to_radians(1.0));

            // Retrieve all IK Solutions
            std::vector<std::vector<double>> all_solutions;
            std::vector<double> iksol;
            while (iksols(iksol)) {
                all_solutions.push_back(std::move(iksol));
            }

            ROS_DEBUG("Gathered %zd IK Solutions", all_solutions.size());

            // Pick the top most-different IK Solutions
            std::vector<std::pair<int, double>> dists(all_solutions.size());
            for (std::size_t i = 0; i < all_solutions.size(); ++i) {
                dists[i] = std::make_pair(i, 0.0);
                for (std::size_t j = 0; j < all_solutions.size(); ++j) {
                    if (i != j) {
                        dists[i].second += hdt::ComputeJointStateL2NormSqrd(all_solutions[i], all_solutions[j]);
                    }
                }
            }

            std::sort(dists.begin(), dists.end(),
                    [](const std::pair<int, double>& a, const std::pair<int, double>& b)
                    { return b.second < a.second; });

            const int max_attempts = 4;
            std::vector<std::vector<double>> solutions_to_attempt;
            const std::size_t num_attempts = std::min(all_solutions.size(), (std::size_t)max_attempts);
            for (std::size_t i = 0; i < num_attempts; ++i) {
                solutions_to_attempt.push_back(std::move(all_solutions[dists[i].first]));
            }

            std::vector<Eigen::Affine3d> differences;
            differences.reserve(num_attempts);

            int num_ik_solutions_attempted = 0;
            for (const std::vector<double>& sol : solutions_to_attempt)
            {
                publish_triad(mount_to_eef);
                AU_DEBUG("  Moving to egress position %s", to_string(egress_position).c_str());
                bounds_markers_pub_.publish(bounds_markers);
                if (!move_to_position(egress_position))
                {
                    ROS_ERROR("    Failed to move to egress position");
                    continue;
                }

                // move to the ik position
                hdt::JointState js;
                std::memcpy(&js, sol.data(), sol.size() * sizeof(double));
                AU_DEBUG("  Moving to IK solution %s", to_string(js).c_str());
                bounds_markers_pub_.publish(bounds_markers);
                if (!move_to_position(js))
                {
                    ROS_WARN("    Failed to move to ik solution");
                    continue;
                }

                // the controller currently reports success as soon as it gets within its capture radius, even if the
                // arm is still moving because the underlying joint controller doesnt think its reached the target yet;
                // sleep here for a couple seconds to let it finish and stabilize
                ros::Duration(2.0).sleep();

                const double record_time_s = 3.0;

                ////////////////////////////////////////////////////////////////////////////////
                // Pose Measurements. SKIP THIS POSITION IF EITHER FAILS
                ////////////////////////////////////////////////////////////////////////////////

                // record the marker pose
                AU_DEBUG("  Recording marker pose at this joint position over the next %0.3f seconds", record_time_s);
                geometry_msgs::Pose marker_pose_camera_frame;
                if (!track_marker_pose(ros::Duration(record_time_s), marker_pose_camera_frame)) {
                    ROS_WARN("Unable to detect marker at pose %s", to_string(sample.pose).c_str());
                    continue;
                }
                Eigen::Affine3d marker_transform(Eigen::Affine3d::Identity());
                tf::poseMsgToEigen(marker_pose_camera_frame, marker_transform);

                // record the wrist pose via forward kinematics
                AU_INFO("  Recording end effector position from joint states over the next %0.3f seconds", record_time_s);
                geometry_msgs::Pose eef_pose;
                if (!track_eef_pose(ros::Duration(record_time_s), eef_pose)) {
                    ROS_WARN("Unable to track end effector at pose %s", to_string(sample.pose).c_str());
                    continue;
                }
                Eigen::Affine3d eef_transform(Eigen::Affine3d::Identity());
                tf::poseMsgToEigen(eef_pose, eef_transform);

                ////////////////////////////////////////////////////////////////////////////////
                // Incorporate difference measurement into average
                ////////////////////////////////////////////////////////////////////////////////

                differences.push_back(msg_utils::transform_diff(marker_transform, eef_transform));

                // 5. Incorporate the difference measurement between the end effector and marker into the average
                auto omit = offset_means_.find(sample);
                if (omit == offset_means_.end()) {
                    offset_means_[sample] = differences.back();
                    ROS_INFO("  Average pose delta is now %s", to_string(differences.back()).c_str());
                }
                else {
                    double alpha = 1.0 / (num_ik_solutions_attempted + 1);
                    omit->second = msg_utils::interp(omit->second, differences.back(), 1.0 - alpha);
                    ROS_INFO("  Average pose delta is now %s", to_string(omit->second).c_str());
                }

                ++num_ik_solutions_attempted;
            }

            // 6. compute a variance matrix
            if (!differences.empty())
            {
                Eigen::Affine3d variance_matrix;
                const Eigen::Affine3d& average_diff = offset_means_[sample];
                AU_INFO("The average offset between the marker and the wrist is %s", to_string(average_diff).c_str());
                int num_added = 0;
                for (const Eigen::Affine3d& diff : differences) {
                    if (num_added == 0) {
                        variance_matrix = msg_utils::transform_diff(diff, average_diff);
                        Eigen::AngleAxisd aa(variance_matrix.rotation());
                        aa.angle();

                    }
                    else {
                        double alpha = 1.0 / (num_added + 1);
                        variance_matrix = msg_utils::interp(variance_matrix, msg_utils::transform_diff(diff, average_diff), 1.0 - alpha);
                    }
                    ++num_added;
                }

                AU_INFO("Variance in pose for coord %s: %s", to_string(sample.coord).c_str(), to_string(variance_matrix).c_str());
                offset_variance_[sample] = variance_matrix;
            }

            if (num_ik_solutions_attempted != 0) {
                AU_WARN("  Attempted %d IK Solutions", num_ik_solutions_attempted);
            }
        }
    }

    write_variances_to_file();

    return SUCCESS;
}

void RepeatabilityMeasurementNode::ar_markers_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    last_markers_msg_ = msg;
}

void RepeatabilityMeasurementNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    last_joint_state_msg_ = msg;
}

bool RepeatabilityMeasurementNode::download_params()
{
    std::vector<double> workspace_min_coords;
    std::vector<double> workspace_max_coords;

    if (!ph_.getParam("workspace_min", workspace_min_coords) ||
        !ph_.getParam("workspace_max", workspace_max_coords))
    {
        ROS_ERROR("Failed to retrieve 'workspace_min' or 'workspace_max' parameters from config");
        return false;
    }

    if (workspace_min_coords.size() != 3 || workspace_max_coords.size() != 3) {
        ROS_ERROR("Invalid workspace coordinates from config");
        return false;
    }

    workspace_min_(0) = workspace_min_coords[0];
    workspace_min_(1) = workspace_min_coords[1];
    workspace_min_(2) = workspace_min_coords[2];

    workspace_max_(0) = workspace_max_coords[0];
    workspace_max_(1) = workspace_max_coords[1];
    workspace_max_(2) = workspace_max_coords[2];

    if (!ph_.getParam("roll_offset_degs", roll_offset_degs_) ||
        !ph_.getParam("pitch_offset_degs", pitch_offset_degs_) ||
        !ph_.getParam("yaw_offset_degs", yaw_offset_degs_))
    {
        ROS_ERROR("Failed to retrieve angle offset parameters from config");
        return false;
    }

    if (!ph_.getParam("sample_resolution_x", sample_res_(0)) ||
        !ph_.getParam("sample_resolution_y", sample_res_(1)) ||
        !ph_.getParam("sample_resolution_z", sample_res_(2)))
    {
        ROS_ERROR("Failed to retrieve sampling parameters");
        return false;
    }

    int tracked_marker_id = -1;
    if (!ph_.getParam("tracked_marker_id", tracked_marker_id)) {
        ROS_ERROR("Failed to retrieve 'tracked_marker_id' from the param server");
        return false;
    }

    if (tracked_marker_id < 0) {
        ROS_ERROR("Param 'tracked_marker_id' must be non-negative");
        return false;
    }

    tracked_marker_id_ = (unsigned)tracked_marker_id;

    if (!download_egress_positions()) {
        return false;
    }

    if (!ph_.getParam("camera_frame", camera_frame_)) {
        ROS_ERROR("Failed to retrieve 'camera_frame' from config");
        return false;
    }

    mount_frame_ = "arm_mount_panel_dummy";

    return true;
}

bool RepeatabilityMeasurementNode::download_egress_positions()
{
    XmlRpc::XmlRpcValue egress_positions;
    if (!ph_.getParam("egress_positions", egress_positions) ||
        egress_positions.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }

    // read in each egress position
    for (int i = 0; i < egress_positions.size(); ++i) {
        XmlRpc::XmlRpcValue joint_vector = egress_positions[i];
        // assert the size of the joint vector
        if (joint_vector.size() != 7) {
            ROS_WARN("Joint vector has incorrect size (%d). Expected 7.", joint_vector.size());
            return false;
        }

        // assert the types of the joint element
        for (int i = 0; i < 7; ++i) {
            XmlRpc::XmlRpcValue::Type type = joint_vector[i].getType();
            if (type != XmlRpc::XmlRpcValue::TypeDouble) {
                ROS_WARN("Joint vector element has incorrect type. (%s) Expected double", to_cstring(type));
                return false;
            }
        }

        // create the joint stat
        hdt::JointState joint_state(
            (double)joint_vector[0],
            (double)joint_vector[1],
            (double)joint_vector[2],
            (double)joint_vector[3],
            (double)joint_vector[4],
            (double)joint_vector[5],
            (double)joint_vector[6]);

        egress_positions_.push_back(joint_state);

        AU_INFO("Read in egress position %s", to_string(joint_state).c_str());
    }

    return true;
}

bool RepeatabilityMeasurementNode::connect_to_move_arm_client()
{
    move_arm_command_action_name_ = "move_arm_command";
    move_arm_command_client_.reset(new MoveArmActionClient(move_arm_command_action_name_, false));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Action Client");
        return false;
    }

    ros::Rate waitLoopRate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!move_arm_command_client_->isServerConnected()) {
            AU_INFO("Waiting to connect to action server %s", move_arm_command_action_name_.c_str());
        }
        else {
            break;
        }
        waitLoopRate.sleep();
    }

    AU_INFO("Connected to action server '%s'", move_arm_command_action_name_.c_str());

    return true;
}

const char* RepeatabilityMeasurementNode::to_cstring(XmlRpc::XmlRpcValue::Type type)
{
    switch (type)
    {
    case XmlRpc::XmlRpcValue::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::TypeDouble:
        return "Double";
    case XmlRpc::XmlRpcValue::TypeInt:
        return "Int";
    case XmlRpc::XmlRpcValue::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::TypeStruct:
        return "Struct";
    }
}

auto RepeatabilityMeasurementNode::generate_sample_poses() -> std::vector<SamplePose>
{
    const double sample_resolution_m = 0.10; // ideal linear discretization
    const double sample_angle_res_degs = 30.0; // ideal angular discretization

    Eigen::Vector3d workspace_size = workspace_max_ - workspace_min_;

    num_samples_(0) = (int)std::round(workspace_size(0) / sample_resolution_m) + 1;
    num_samples_(1) = (int)std::round(workspace_size(1) / sample_resolution_m) + 1;
    num_samples_(2) = (int)std::round(workspace_size(2) / sample_resolution_m) + 1;

    sample_res_(0) = workspace_size(0) / (num_samples_(0) - 1);
    sample_res_(1) = workspace_size(1) / (num_samples_(1) - 1);
    sample_res_(2) = workspace_size(2) / (num_samples_(2) - 1);

    num_roll_samples_ = (int)std::round(2 * roll_offset_degs_ / sample_angle_res_degs) + 1;
    roll_res_ = 2 * roll_offset_degs_ / (num_roll_samples_ - 1);

    num_pitch_samples_ = (int)std::round(2 * pitch_offset_degs_ / sample_angle_res_degs) + 1;
    pitch_res_ = 2 * pitch_offset_degs_ / (num_pitch_samples_ - 1);

    num_yaw_samples_ = (int)std::round(2 * yaw_offset_degs_ / sample_angle_res_degs) + 1;
    yaw_res_ = 2 * yaw_offset_degs_ / (num_yaw_samples_ - 1);

    ROS_INFO("Sampling x locations in [%0.3f, %0.3f] at %0.3f m with %d samples", workspace_min_(0), workspace_max_(0), sample_res_(0), num_samples_(0));
    ROS_INFO("Sampling y locations in [%0.3f, %0.3f] at %0.3f m with %d samples", workspace_min_(1), workspace_max_(1), sample_res_(1), num_samples_(1));
    ROS_INFO("Sampling z locations in [%0.3f, %0.3f] at %0.3f m with %d samples", workspace_min_(2), workspace_max_(2), sample_res_(2), num_samples_(2));

    std::vector<SamplePose> sample_poses;
    for (int x = 0; x < num_samples_(0); ++x)
    {
        double sample_x = workspace_min_(0) + x * sample_res_(0);
        for (int y = 0; y < num_samples_(1); ++y)
        {
            double sample_y = workspace_min_(1) + y * sample_res_(1);
            for (int z = 0; z < num_samples_(2); ++z)
            {
                double sample_z = workspace_min_(2) + z * sample_res_(2);
                for (int roll = 0; roll < num_roll_samples_; ++roll)
                {
                    double sample_roll = -roll_offset_degs_ + roll * roll_res_;
                    for (int pitch = 0; pitch < num_pitch_samples_; ++pitch)
                    {
                        double sample_pitch = -pitch_offset_degs_ + pitch * pitch_res_;
                        for (int yaw = 0; yaw < num_yaw_samples_; ++yaw)
                        {
                            double sample_yaw = -yaw_offset_degs_ + yaw * yaw_res_;

                            Eigen::Affine3d sample_transform_camera_frame =
                                Eigen::Translation3d(sample_x, sample_y, sample_z) *
                                Eigen::AngleAxisd(sbpl::angles::to_radians(sample_roll), Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(sbpl::angles::to_radians(sample_pitch), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(sbpl::angles::to_radians(sample_yaw), Eigen::Vector3d::UnitZ()) *
                                camera_frame_to_tool_frame_rotation_;

                            SamplePose sample;
                            sample.coord = std::make_tuple(x,y, z, roll, pitch, yaw);
                            tf::poseEigenToMsg(sample_transform_camera_frame, sample.pose);

                            ROS_INFO("Adding sample pose %s", to_string(sample.pose).c_str());
                            sample_poses.push_back(sample);
                        }
                    }
                }
            }
        }
    }

    return sample_poses;
}

bool RepeatabilityMeasurementNode::move_to_position(const hdt::JointState& position)
{
    if (!move_arm_command_client_->isServerConnected()) {
        ROS_WARN("Move Arm Command Client (%s) is not connected", move_arm_command_action_name_.c_str());
        return false;
    }

    std::vector<double> joint_vector =
            { position.a0, position.a1, position.a2, position.a3, position.a4, position.a5, position.a6 };

    rcta::MoveArmGoal goal;

    goal.type = rcta::MoveArmGoal::JointGoal;
    goal.goal_joint_state.header.stamp = ros::Time::now();
    goal.goal_joint_state.header.frame_id = "";
    goal.goal_joint_state.header.seq = 0;
    goal.goal_joint_state.name = robot_model_->joint_names();
    goal.goal_joint_state.position = joint_vector;

    auto result_callback = boost::bind(&RepeatabilityMeasurementNode::move_arm_command_result_cb, this, _1, _2);
    move_arm_command_client_->sendGoal(goal, result_callback);
    pending_command_ = true;
    ros::Rate lr(2);
    while (ros::ok() && pending_command_)
    {
        // TODO: manually check for timeout here
        ros::spinOnce();
        lr.sleep();
    }

    return true;
}

bool RepeatabilityMeasurementNode::track_marker_pose(const ros::Duration& listen_duration, geometry_msgs::Pose& pose)
{
    ROS_INFO("    Recording marker position...");
    last_markers_msg_.reset(); // only listen for fresh markers

    ar_track_alvar_msgs::AlvarMarkers::ConstPtr last_processed_msg;
    Eigen::Affine3d marker_transform(Eigen::Affine3d::Identity());

    ros::Time start = ros::Time::now();
    while (ros::ok() && ros::Time::now() < start + listen_duration) {
        ros::spinOnce(); // wait for marker message
        if (!last_markers_msg_) {
            continue;
        }

        if (!last_processed_msg) {
            // skip empty messages or messages that don't have the marker we're looking for (assume only one marker for the time being)
            if (last_markers_msg_->markers.empty() || last_markers_msg_->markers.front().id != tracked_marker_id_) {
                continue;
            }

            // initial reading of the pose
            tf::poseMsgToEigen(last_markers_msg_->markers.front().pose.pose, marker_transform);
        }
        else {
            // skip empty messages or messages that don't have the marker we're looking for (assume only one marker for the time being)
            if (last_markers_msg_->markers.empty() || last_markers_msg_->markers.front().id != tracked_marker_id_) {
                continue;
            }
            // start filtering that bad boy
            Eigen::Affine3d curr_marker_transform;
            tf::poseMsgToEigen(last_markers_msg_->markers.front().pose.pose, curr_marker_transform);
            const double pose_gain = 0.1;
            marker_transform = msg_utils::interp(marker_transform, curr_marker_transform, pose_gain);
        }

        //
        last_processed_msg = last_markers_msg_;
    }

    // no messages were processed
    if (!last_processed_msg) {
        return false;
    }

    tf::poseEigenToMsg(marker_transform, pose);
    return true;
}

bool RepeatabilityMeasurementNode::track_eef_pose(const ros::Duration& listen_duration, geometry_msgs::Pose& pose)
{
    ROS_INFO("    Recording wrist position via forward kinematics...");
    last_joint_state_msg_.reset(); // only listen for joint states

    sensor_msgs::JointState::ConstPtr last_incorp_msg;
    Eigen::Affine3d eef_transform;
    ros::Time start = ros::Time::now();
    while (ros::ok() && ros::Time::now() < start + listen_duration) {
        ros::spinOnce(); // wait for marker message

        if (last_joint_state_msg_) {
            if (!last_incorp_msg) {
                // initial reading of the pose
                std::vector<double> joint_vals = gather_joint_values(*last_joint_state_msg_);
                if (!joint_vals.empty()) {
                    robot_model_->compute_fk(joint_vals, eef_transform);
                }
            }
            else {
                // start filtering that bad boy
                Eigen::Affine3d curr_marker_transform;
                std::vector<double> joint_vals = gather_joint_values(*last_joint_state_msg_);
                const double pose_gain = 0.1;
                eef_transform = msg_utils::interp(eef_transform, curr_marker_transform, pose_gain);
            }

            last_incorp_msg = last_joint_state_msg_;
        }
    }

    if (!last_incorp_msg) {
        return false;
    }

    tf::poseEigenToMsg(eef_transform, pose);
    return true;
}

void RepeatabilityMeasurementNode::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::MoveArmResult::ConstPtr& result)
{
    AU_DEBUG("Received a Move Arm Command Result!");
    pending_command_ = false;
}

void RepeatabilityMeasurementNode::publish_triad(const Eigen::Affine3d& mount_to_eef)
{
    // publish the new pose marker
    geometry_msgs::Vector3 marker_scale;
    marker_scale.x = 0.1;
    marker_scale.y = marker_scale.z = 0.01;
    visualization_msgs::MarkerArray pose_markers = msg_utils::create_triad_marker_arr(marker_scale);
    for (visualization_msgs::Marker& marker : pose_markers.markers) {
        Eigen::Affine3d marker_pose_eef_frame;
        tf::poseMsgToEigen(marker.pose, marker_pose_eef_frame);
        Eigen::Affine3d mount_to_marker = mount_to_eef * marker_pose_eef_frame;
        tf::poseEigenToMsg(mount_to_marker, marker.pose);
        marker.header.frame_id = mount_frame_;
    }
    sample_eef_pose_marker_pub_.publish(pose_markers);
}

std::vector<double> RepeatabilityMeasurementNode::gather_joint_values(const sensor_msgs::JointState& joint_state) const
{
    if (msg_utils::contains_only_joints(joint_state, robot_model_->joint_names())) {
        sensor_msgs::JointState js = joint_state;
        msg_utils::reorder_joints(js, robot_model_->joint_names());
        return js.position;
    }
    else {
        return std::vector<double>();
    }
}

bool RepeatabilityMeasurementNode::write_variances_to_file()
{
    FILE* f = fopen("variances.txt", "w");
    if (!f) {
        ROS_ERROR("Failed to open variances.txt for writing");
        return false;
    }

    for (const auto& variance : offset_variance_) {
        std::stringstream ss;
        ss << "Coord: " << to_string(variance.first.pose) << ", Variance: " << ::to_string(variance.second) << "\n";
        fprintf(f, "%s", ss.str().c_str());
    }

    fclose(f);
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "repeatability_measure");
    return RepeatabilityMeasurementNode().run();
}
