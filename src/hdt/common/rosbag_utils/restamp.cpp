#include <cstdio>
#include <cstring>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

enum MainResult
{
    SUCCESS = 0,
    ERROR = 1
};

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <bag file>\n", argv[0]);
        return ERROR;
    }

    ros::init(argc, argv, "restamp");
    ros::NodeHandle nh;

    // open up the rosbag file
    std::string rosbag_fname(argv[1]);
    rosbag::Bag bag;
    try {
        bag.open(rosbag_fname, rosbag::bagmode::Read);
    }
    catch (const rosbag::BagException& ex) {
        fprintf(stderr, "Failed to open bag file for reading (%s)", ex.what());
        return ERROR;
    }

    // get the md5 sums for the message types that care about
    std::string occupancy_grid_md5(ros::message_traits::MD5Sum<nav_msgs::OccupancyGrid>::value());
    std::string odometry_md5(ros::message_traits::MD5Sum<nav_msgs::Odometry>::value());
    std::string pose_stamped_md5(ros::message_traits::MD5Sum<geometry_msgs::PoseStamped>::value());

    // gather all the topics that have one of the above md5sums
    rosbag::View view(bag);
    std::vector<std::string> interested_topics;
    std::vector<std::string> topic_md5s;
    for (const rosbag::ConnectionInfo* conn : view.getConnections()) {
        if (conn->md5sum == occupancy_grid_md5 || conn->md5sum == odometry_md5 || conn->md5sum == pose_stamped_md5) {
            interested_topics.push_back(conn->topic);
            topic_md5s.push_back(conn->md5sum);
        }
    }

    ROS_INFO("Interested Topics:");
    for (const std::string& topic : interested_topics) {
        ROS_INFO("   Topic: %s", topic.c_str());
    }

    // create publishers for forwarding
    std::map<std::string, ros::Publisher> publishers;
    for (std::size_t i = 0; i < interested_topics.size(); ++i) {
        const std::string& topic = interested_topics[i];
        const std::string& md5 = topic_md5s[i];
        if (md5 == occupancy_grid_md5) {
            publishers[topic] = nh.advertise<nav_msgs::OccupancyGrid>(topic, 10);
        }
        else if (md5 == odometry_md5) {
            publishers[topic] = nh.advertise<nav_msgs::Odometry>(topic, 10);
        }
        else if (md5 == pose_stamped_md5) {
            publishers[topic] = nh.advertise<geometry_msgs::PoseStamped>(topic, 10);
        }
        else {
            return ERROR;
        }
    }

    tf::TransformBroadcaster broadcaster;

    while (ros::ok()) {
        // overwrite the header information in each message to fake coming from 'now'
        ros::Time last_stamp(0, 0);
        bool isValid = false;
        rosbag::View simplified_view(bag, rosbag::TopicQuery(interested_topics));
        for (rosbag::MessageInstance const m : simplified_view) {
            if (!ros::ok()) {
                ros::shutdown();
                break;
            }

            if (isValid && last_stamp.isValid()) {
                ros::Duration wait_duration(m.getTime() - last_stamp);
                wait_duration.sleep();
            }
            last_stamp = m.getTime();
            isValid = true;

            last_stamp = m.getTime();
            if (m.getMD5Sum() == occupancy_grid_md5) {
                nav_msgs::OccupancyGrid::Ptr s = m.instantiate<nav_msgs::OccupancyGrid>();
                s->header.stamp = ros::Time::now();
                publishers[m.getTopic()].publish(s);
            }
            else if (m.getMD5Sum() == odometry_md5) {
                nav_msgs::Odometry::Ptr s = m.instantiate<nav_msgs::Odometry>();
                s->header.stamp = ros::Time::now();
                publishers[m.getTopic()].publish(s);

                // publish odometry
                tf::Transform transform(
                        tf::Quaternion(s->pose.pose.orientation.x, s->pose.pose.orientation.y, s->pose.pose.orientation.z, s->pose.pose.orientation.w),
                        tf::Vector3(s->pose.pose.position.x, s->pose.pose.position.y, s->pose.pose.position.z));
                tf::StampedTransform transform_stamped(transform, ros::Time::now(), s->header.frame_id, s->child_frame_id);
                broadcaster.sendTransform(transform_stamped);
            }
            else if (m.getMD5Sum() == pose_stamped_md5) {
                geometry_msgs::PoseStamped::Ptr s = m.instantiate<geometry_msgs::PoseStamped>();
                s->header.stamp = ros::Time::now();
                publishers[m.getTopic()].publish(s);
            }
            else {
                return ERROR;
            }
        }
    }

    return SUCCESS;
}
