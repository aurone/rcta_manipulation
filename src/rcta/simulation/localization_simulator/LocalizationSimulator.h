#ifndef LocalizationSimulator_h
#define LocalizationSimulator_h

// standard includes
#include <cmath>
#include <memory>

// system includes
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/utils/RunUponDestruction.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// project includes
#include <rcta/TeleportAndaliteCommandAction.h>

class LocalizationSimulator
{
public:

    LocalizationSimulator();

    enum MainResult
    {
        SUCCESS,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    bool initialize();
    void publish_costmap();

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::string action_name_;

    std::string world_frame_nwu_name_;
    std::string world_frame_ned_name_;
    std::string robot_frame_nwu_name_;
    std::string robot_frame_ned_name_;
    std::string robot_odometry_frame_ned_name_;

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    Eigen::Affine3d world_to_robot_;
    Eigen::Affine3d footprint_to_robot_;

    typedef actionlib::SimpleActionServer<rcta::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionServer;
    std::unique_ptr<TeleportAndaliteCommandActionServer> as_;

    bool publish_costmap_;
    std::string costmap_bagfile_;

    nav_msgs::OccupancyGrid::Ptr last_costmap_msg_;
    ros::Publisher costmap_pub_;

    ros::Time last_costmap_pub_time_;
    ros::Rate costmap_pub_rate_;

    void goal_callback();
    void preempt_callback();

    bool transform(const std::string& target, const std::string& source, Eigen::Affine3d& transform);
};

#endif
