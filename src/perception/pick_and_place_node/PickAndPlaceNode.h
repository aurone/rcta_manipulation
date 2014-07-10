#ifndef PickAndPlaceNode_h
#define PickAndPlaceNode_h

#include <cstdio>
#include <string>
#include <memory>
#include <vector>
#include <actionlib/server/simple_action_server.h>
#include <hdt/ObjectDetectionAction.h>
#include <message_filters/subscriber.h>
#include <pr2_vfh_database/ObjectFinder.h>
#include <pr2_vfh_database/pr2_grasp_database.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <pr2_vfh_database/ObjectFinder.h>

/// Basic node that subscribes to point clouds and returns the best match in the
/// given object database for each received point cloud.
class PickAndPlaceNode
{
public:

    PickAndPlaceNode();

    bool initialize();
    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    typedef actionlib::SimpleActionServer<hdt::ObjectDetectionAction> ObjectDetectionActionServer;
    std::unique_ptr<ObjectDetectionActionServer> action_server_;

    std::string point_cloud_topic_;
    // message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
    // std::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> tf_filter_;
    ros::Subscriber point_cloud_sub_;

    std::unique_ptr<ObjectFinder> object_detector_;

    sensor_msgs::PointCloud2::ConstPtr last_point_cloud_;
    tf::TransformListener listener_;

    std::string camera_frame_;
    std::string root_frame_;

    std::string last_database_filename_;
    std::string last_features_filename_;
    std::string last_kdtree_indices_filename_;

    ros::Publisher pregrasp_pub_;

    hdt::ObjectDetectionFeedback feedback_;
    hdt::ObjectDetectionResult result_;

    void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void print_database_contents(const std::string& database_filename) const;

    void object_detection_callback(const hdt::ObjectDetectionGoal::ConstPtr& goal);

    void publish_grasp_markers(const geometry_msgs::PoseArray& grasps, const geometry_msgs::PoseArray& pregrasps) const;
};

#endif
