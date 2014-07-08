#ifndef PickAndPlaceNode_h
#define PickAndPlaceNode_h

#include <cstdio>
#include <string>
#include <memory>
#include <vector>
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

    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
    std::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> tf_filter_;

    std::unique_ptr<ObjectFinder> object_detector_;

    tf::TransformListener listener_;

    std::string camera_frame_;
    std::string root_frame_;

    std::string database_filename_;
    std::string features_filename_;
    std::string kdtree_index_filename_;

    void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void print_database_contents() const;
};

#endif
