#include <boost/date_time/posix_time/posix_time.hpp>
#include <visualization_msgs/MarkerArray.h>
#include "PickAndPlaceNode.h"

PickAndPlaceNode::PickAndPlaceNode() :
    nh_(),
    ph_("~"),
    point_cloud_sub_(),
    tf_filter_()
{

}

bool PickAndPlaceNode::initialize()
{
    if (!ph_.getParam("database_filename", database_filename_)) {
        ROS_ERROR("Failed to obtain 'database_filename' from the param server");
        return false;
    }

    if (!ph_.getParam("features_filename", features_filename_)) {
        ROS_ERROR("Failed to obtain 'features_filename' from the param server");
        return false;
    }

    if (!ph_.getParam("kdtree_index_filename", kdtree_index_filename_)) {
        ROS_ERROR("Failed to obtain 'kdtree_index_filename' from the param server");
        return false;
    }

    std::string point_cloud_topic;
    if (!ph_.getParam("point_cloud_topic", point_cloud_topic)) {
        ROS_ERROR("Failed to extract 'point_cloud_topic' from the param server");
        return false;
    }

    if (!ph_.getParam("camera_frame", camera_frame_)) {
        ROS_ERROR("Failed to extract 'camera_frame' from the param server");
        return false;
    }

    if (!ph_.getParam("root_frame", root_frame_)) {
        ROS_ERROR("Failed to extract 'root_frame' from the param server");
        return false;
    }

    // subscribe to point cloud topic
    point_cloud_sub_.subscribe(nh_, point_cloud_topic, 1);
    tf_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(point_cloud_sub_, listener_, root_frame_, 2));
    tf_filter_->registerCallback(std::bind(&PickAndPlaceNode::point_cloud_callback, this, std::placeholders::_1));

    ObjectFinder::VisualizationOptions visopts;
    visopts.visualize_culled_cloud = true;
    visopts.visualize_best_cluster = true;
    visopts.visualize_best_match = true;
    visopts.visualize_segmented_plane = true;
    visopts.visualize_all_clusters = true;
    visopts.visualize_fixed_match = true;
    object_detector_.reset(new ObjectFinder(visopts));
    if (!object_detector_) {
        ROS_ERROR("Failed to instantiate Object Finder");
        return false;
    }

    if (!object_detector_->initialize(database_filename_, features_filename_, kdtree_index_filename_)) {
        ROS_ERROR("Failed to initialize Object Finder");
        return false;
    }

    ROS_INFO("Successfully initialized Pick And Place Node");
    return true;
}

int PickAndPlaceNode::run()
{
    ros::spin();
    return 0;
}

void PickAndPlaceNode::point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("Received a point cloud at %s", boost::posix_time::to_simple_string(ros::Time::now().toBoost()).c_str());
    // wait for transforms between root <-> camera <-> point cloud
    bool transforms_available = true;
    transforms_available &= listener_.waitForTransform(root_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(5.0));
    transforms_available &= listener_.waitForTransform(msg->header.frame_id, camera_frame_, msg->header.stamp, ros::Duration(5.0));
    transforms_available &= listener_.waitForTransform(camera_frame_, root_frame_, msg->header.stamp, ros::Duration(5.0));

    ObjectFinder::CullingOptions cull_options;
    cull_options.cull = true;
    cull_options.from.x = 0.3;
    cull_options.from.y = -0.6;
    cull_options.from.z = -1.6;
    cull_options.to.x = 2.5;
    cull_options.to.y = 0.6;
    cull_options.to.z = 1.6;

    ObjectFinder::MatchResult match_result = object_detector_->match(*msg, camera_frame_, root_frame_, listener_, cull_options);
    bool success = match_result.valid();

    if (success) {
        ROS_INFO("Match succeeded");

        geometry_msgs::PoseArray grasps_out, pregrasps_out;
        object_detector_->get_grasps(match_result, camera_frame_, root_frame_, grasps_out, pregrasps_out, listener_);

        ROS_INFO("Retrieved %zd grasps and %zd pre-grasps!", grasps_out.poses.size(), pregrasps_out.poses.size());
        static ros::Publisher pregrasp_pub = nh_.advertise<visualization_msgs::MarkerArray>("grasp_markers", 5);

        int id = 0;
        visualization_msgs::MarkerArray arr;
        for (const geometry_msgs::Pose& grasp_pose : grasps_out.poses) {
            visualization_msgs::Marker m;
            m.header.seq = 0;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = grasps_out.header.frame_id;
            m.ns = "grasp";
            m.id = id++;
            m.type = visualization_msgs::Marker::ARROW;
            m.action = visualization_msgs::Marker::ADD;
            m.pose = grasp_pose;
            m.scale.x = 0.1;
            m.scale.y = 0.02;
            m.scale.z = 0.02;
            m.color.a = 1.0;
            m.lifetime = ros::Duration(0);
            arr.markers.push_back(m);
        }

        pregrasp_pub.publish(arr);
    }
    else {
        ROS_WARN("Match failed");
    }
}

void PickAndPlaceNode::print_database_contents() const
{
    ROS_INFO("Opening database located at %s", database_filename_.c_str());
    pr2_grasp_database database(database_filename_);

    ROS_INFO("The database has size %d", database.size());

    for (int i = 1; i <= database.size(); ++i) {
        sensor_msgs::PointCloud2 msg;
        if (!database.GetPointCloud2(i, msg)) {
            ROS_WARN("Failed to retrieve point cloud at row %d", i);
            continue;
        }

        std::string object_name = database.GetObjectName(i);
        ROS_INFO("Object %s at row %d", object_name.c_str(), i);
    }
}
