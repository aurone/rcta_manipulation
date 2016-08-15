// standard includes
#include <cmath>
#include <memory>

// system includes
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <rcta_msgs/RepositionBaseCommandAction.h>
#include <ros/ros.h>
#include <sbpl_geometry_utils/utils.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/utils/RunUponDestruction.h>

class RepositionBaseSimulator
{
public:

    RepositionBaseSimulator();

    enum MainResult
    {
        SUCCESS,
        FAILED_TO_INITIALIZE
    };

    int run();

private:

    bool initialize();

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::string world_frame_name_;

    typedef actionlib::SimpleActionServer<rcta_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionServer;
    std::unique_ptr<RepositionBaseCommandActionServer> as_;
    std::string action_name_;

    int max_out_candidates_;

    void goal_callback();
    void preempt_callback();
};

RepositionBaseSimulator::RepositionBaseSimulator() :
    nh_(),
    ph_("~"),
    world_frame_name_("abs_nwu"),
    as_(),
    action_name_("reposition_base_command")
{
}

int RepositionBaseSimulator::run()
{
    if (!initialize()) {
        ROS_ERROR("Failed to initialize Localization Simulator");
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(1.0);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });
        ros::spinOnce();
    }

    return SUCCESS;
}

bool RepositionBaseSimulator::initialize()
{
    if (!ph_.getParam("max_output_candidates", max_out_candidates_)) {
        ROS_ERROR("Failed to retrieve 'max_output_candidates' from the param server");
        return false;
    }

    if (max_out_candidates_ < 0) {
        ROS_ERROR("Yeah, because negative numbers make sense. Who invented those guys anyway?");
        return false;
    }

    as_.reset(new RepositionBaseCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Teleport Andalite Command Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&RepositionBaseSimulator::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&RepositionBaseSimulator::preempt_callback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    return true;
}

void RepositionBaseSimulator::goal_callback()
{
    auto current_goal = as_->acceptNewGoal();

    std::vector<geometry_msgs::PoseStamped> candidate_base_poses;

    // initialize uniform candidate pose attributes
    geometry_msgs::PoseStamped candidate_pose;
    candidate_pose.header.frame_id = world_frame_name_;
    candidate_pose.header.seq = 0;
    candidate_pose.header.stamp = ros::Time::now();

    // sample candidate base positions uniformly in a circle around the object
    // with the heading of the base aimed directly at the object
    const double radius_m = 0.8; // canonical distance from the object
    const int num_samples = 32;
    for (int i = 0; i < num_samples; ++i) {
        double yaw = (2.0 * M_PI * i) / num_samples;
        double object_pos_x = current_goal->gas_can_in_map.pose.position.x;
        double object_pos_y = current_goal->gas_can_in_map.pose.position.y;
        Eigen::Vector2d object_pos(object_pos_x, object_pos_y);

        double candidate_x = object_pos.x() + radius_m * cos(yaw);
        double candidate_y = object_pos.y() + radius_m * sin(yaw);

        Eigen::Vector2d candidate_pos(candidate_x, candidate_y);
        Eigen::Vector2d candidate_heading(object_pos - candidate_pos);
        candidate_heading.normalize();
        double candidate_yaw = atan2(candidate_heading.y(), candidate_heading.x());

        Eigen::Affine3d candidate_transform =
                Eigen::Translation3d(candidate_pos.x(), candidate_pos.y(), 0.0) *
                Eigen::AngleAxisd(candidate_yaw, Eigen::Vector3d(0, 0, 1));

        tf::poseEigenToMsg(candidate_transform, candidate_pose.pose);
        candidate_base_poses.push_back(candidate_pose);
    }

    // Retrieve the object's heading for prioritizing base candidates
    Eigen::Affine3d object_transform;
    tf::poseMsgToEigen(current_goal->gas_can_in_map.pose, object_transform);
    double object_yaw, pitch, roll;
    msg_utils::get_euler_ypr(object_transform, object_yaw, pitch, roll);

    ROS_INFO("Object Yaw: %0.3f degs", sbpl::utils::ToDegrees(object_yaw));

    // we want the heading of the object to be offset 60 degrees from the heading of the base
    const double canonical_heading_diff_rad = sbpl::utils::ToRadians(60.0);

    // sort candidate base poses to get the pose whose heading is most ideally offset from the object's heading
    auto compare_poses = [&] (const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b)
    {
        Eigen::Affine3d a_transform, b_transform;
        tf::poseMsgToEigen(a.pose, a_transform);
        tf::poseMsgToEigen(b.pose, b_transform);

        double a_yaw, b_yaw, tmp_pitch, tmp_roll;
        msg_utils::get_euler_ypr(a_transform, a_yaw, tmp_pitch, tmp_roll);
        msg_utils::get_euler_ypr(b_transform, b_yaw, tmp_pitch, tmp_roll);

        double canonical_heading_diff_a = fabs(canonical_heading_diff_rad - sbpl::utils::ShortestAngleDiff(object_yaw, a_yaw));
        double canonical_heading_diff_b = fabs(canonical_heading_diff_rad - sbpl::utils::ShortestAngleDiff(object_yaw, b_yaw));
        return canonical_heading_diff_a < canonical_heading_diff_b;
    };

    std::sort(candidate_base_poses.begin(), candidate_base_poses.end(), compare_poses);

    ROS_INFO("Sorted Candidates (%zd):", candidate_base_poses.size());
    for (const geometry_msgs::PoseStamped& pose : candidate_base_poses) {
        Eigen::Affine3d candidate_transform;
        tf::poseMsgToEigen(pose.pose, candidate_transform);
        double r, p, y;
        msg_utils::get_euler_ypr(candidate_transform, y, p, r);
        ROS_INFO(" -> Yaw: %0.3f", sbpl::utils::ToDegrees(y));
    }

    while (candidate_base_poses.size() > max_out_candidates_) {
        candidate_base_poses.pop_back();
    }

    rcta_msgs::RepositionBaseCommandResult result;
    result.result = rcta_msgs::RepositionBaseCommandResult::SUCCESS;
    result.candidate_base_poses = std::move(candidate_base_poses);
    as_->setSucceeded(result);
}

void RepositionBaseSimulator::preempt_callback()
{
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "reposition_base_simulator");
    return RepositionBaseSimulator().run();
}
