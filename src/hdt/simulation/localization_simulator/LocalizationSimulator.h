#ifndef LocalizationSimulator_h
#define LocalizationSimulator_h

#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/TeleportAndaliteCommandAction.h>
#include <hdt/common/msg_utils/msg_utils.h>

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

    typedef actionlib::SimpleActionServer<hdt::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionServer;
    std::unique_ptr<TeleportAndaliteCommandActionServer> as_;

    bool publish_costmap_;
    std::string costmap_bagfile_;
    ros::Publisher costmap_pub_;

    void goal_callback();
    void preempt_callback();

    bool transform(const std::string& target, const std::string& source, Eigen::Affine3d& transform);
};

#endif
