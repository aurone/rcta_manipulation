#include "LocalizationSimulator.h"

#include <nav_msgs/OccupancyGrid.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

LocalizationSimulator::LocalizationSimulator() :
    nh_(),
    ph_("~"),
    action_name_("teleport_andalite_command"),
    world_frame_nwu_name_("abs_nwu"),
    world_frame_ned_name_("abs_ned"),
    robot_frame_nwu_name_("base_footprint"),
    robot_frame_ned_name_("robot_ned"),
    robot_odometry_frame_ned_name_("robot"),
    broadcaster_(),
    listener_(),
    world_to_robot_(Eigen::Affine3d::Identity()),
    as_(),
    costmap_pub_()
{
}

int LocalizationSimulator::run()
{
    if (!initialize()) {
        ROS_ERROR("Failed to initialize Localization Simulator");
        return FAILED_TO_INITIALIZE;
    }

    publish_costmap();

    ros::Rate loop_rate(10.0);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });

        ros::spinOnce();

        ros::Time now = ros::Time::now();

        tf::Quaternion ned_to_nwu_rotation(tf::Vector3(1.0, 0.0, 0.0), M_PI);
        tf::Transform ned_to_nwu(ned_to_nwu_rotation, tf::Vector3(0.0, 0.0, 0.0));

        tf::Transform footprint_ned_to_robot_ned;
        msg_utils::convert(footprint_to_robot_.inverse(), footprint_ned_to_robot_ned);
        tf::StampedTransform footprint_ned_to_robot_ned_stamped(footprint_ned_to_robot_ned, now, robot_odometry_frame_ned_name_, robot_frame_ned_name_);
        broadcaster_.sendTransform(footprint_ned_to_robot_ned_stamped);

        // publish robot coordinate frame using NED conventions for convenience
        tf::StampedTransform robot_ned_to_nwu_stamped(ned_to_nwu, now, robot_frame_ned_name_, robot_frame_nwu_name_);
        broadcaster_.sendTransform(robot_ned_to_nwu_stamped);

        // publish world coordinate frame to robot coordinate frame transform (global pose)
        tf::Transform world_ned_to_robot_ned;
        msg_utils::convert(world_to_robot_, world_ned_to_robot_ned);
        tf::StampedTransform world_ned_to_robot_ned_stamped(world_ned_to_robot_ned, now, world_frame_ned_name_, robot_frame_ned_name_);
        broadcaster_.sendTransform(world_ned_to_robot_ned_stamped);

        // publish world coordinate frame using NWU conventions for convenience
        tf::StampedTransform world_ned_to_nwu_stamped(ned_to_nwu, now, world_frame_ned_name_, world_frame_nwu_name_);
        broadcaster_.sendTransform(world_ned_to_nwu_stamped);

        // TODO: publish /laser transform?
    }

    return SUCCESS;
}

bool LocalizationSimulator::initialize()
{
    as_.reset(new TeleportAndaliteCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Teleport Andalite Command Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&LocalizationSimulator::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&LocalizationSimulator::preempt_callback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    ROS_INFO("Waiting for transform '%s' -> '%s'", robot_frame_nwu_name_.c_str(), "base_link");
    if (!listener_.waitForTransform(robot_frame_nwu_name_, "base_link", ros::Time(0), ros::Duration(10.0))) {
        ROS_ERROR(" -> Failed to lookup transform");
        return false;
    }
    tf::StampedTransform footprint_frame_to_robot_frame;
    listener_.lookupTransform(robot_frame_nwu_name_, "base_link", ros::Time(0), footprint_frame_to_robot_frame);
    msg_utils::convert(footprint_frame_to_robot_frame, footprint_to_robot_);

    ph_.param("publish_costmap", publish_costmap_, false);
    ph_.param<std::string>("costmap_bagfile", costmap_bagfile_, "");

    return true;
}

void LocalizationSimulator::publish_costmap()
{
    if (!publish_costmap_) {
        return;
    }

    if (costmap_bagfile_.empty()) {
        ROS_WARN("~publish_costmap is true but ~costmap_bagfile is empty");
        return;
    }

    try {
        rosbag::Bag bag;
        bag.open(costmap_bagfile_, rosbag::bagmode::Read);
        rosbag::View view(bag);

        // find the first topic with type nav_msgs/OccupancyGrid
        std::string occupancy_grid_topic;
        for (const rosbag::ConnectionInfo* conn : view.getConnections()) {
            if (conn->md5sum == std::string(ros::message_traits::MD5Sum<nav_msgs::OccupancyGrid>::value())) {
                occupancy_grid_topic = conn->topic;
                break;
            }
        }

        if (occupancy_grid_topic.empty()) {
            ROS_WARN("bagfile '%s' does not contain a topic with type nav_msgs/OccupancyGrid", costmap_bagfile_.c_str());
            return;
        }

        // create a view of the rosbag that only contains the nav_msgs/OccupancyGrid
        rosbag::View grid_only_view(bag, rosbag::TopicQuery(std::vector<std::string>({occupancy_grid_topic})));
        nav_msgs::OccupancyGrid::Ptr last_costmap_msg;
        for (const rosbag::MessageInstance m : grid_only_view) {
            nav_msgs::OccupancyGrid::Ptr s = m.instantiate<nav_msgs::OccupancyGrid>();
            if (s) {
                last_costmap_msg = s;
            }
        }

        if (last_costmap_msg) {
            ROS_WARN("Failed to find a valid costmap message. Weird...");
            return;
        }

        // publish costmap message on latched topic "fixed_costmap_sim"
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("fixed_costmap_sim", 1, true);
    }
    catch (const rosbag::BagException& ex) {
        ROS_WARN("Failed to publish costmap from bagfile (%s)", ex.what());
    }
}

void LocalizationSimulator::goal_callback()
{
    auto current_goal = as_->acceptNewGoal();

    try {
        geometry_msgs::PoseStamped robot_pose_ned_frame;
        robot_pose_ned_frame.header.stamp = ros::Time(0);
        robot_pose_ned_frame.header.seq = 0;

        geometry_msgs::PoseStamped goal_pose = current_goal->global_pose;
        goal_pose.header.stamp = ros::Time(0);

        listener_.transformPose(world_frame_ned_name_, goal_pose, robot_pose_ned_frame);

        Eigen::Affine3d world_ned_to_robot_nwu;
        tf::poseMsgToEigen(robot_pose_ned_frame.pose, world_ned_to_robot_nwu);

        // world_ned -> robot_ned = world_ned -> robot_nwu * robot_nwu -> robot_ned
        world_to_robot_ = world_ned_to_robot_nwu * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0));
        as_->setSucceeded();
    }
    catch (const tf::TransformException& ex) {
        ROS_WARN("Unable to transform from '%s' to '%s' (%s)",
                current_goal->global_pose.header.frame_id.c_str(), world_frame_ned_name_.c_str(), ex.what());
        as_->setAborted();
    }
}

void LocalizationSimulator::preempt_callback()
{
    // shouldn't ever need to do this...I mean it
}
