#include <boost/date_time/posix_time/posix_time.hpp>
#include <visualization_msgs/MarkerArray.h>
#include "PickAndPlaceNode.h"

PickAndPlaceNode::PickAndPlaceNode() :
    nh_(),
    ph_("~"),
    action_server_(),
    point_cloud_sub_(),
    tf_filter_(),
    object_detector_(),
    last_point_cloud_(),
    listener_(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), false),
    camera_frame_(),
    root_frame_(),
    last_database_filename_(),
    last_features_filename_(),
    last_kdtree_indices_filename_(),
    pregrasp_pub_(),
    feedback_(),
    result_()
{
}

bool PickAndPlaceNode::initialize()
{
    std::string point_cloud_topic;
    if (!ph_.getParam("point_cloud_topic", point_cloud_topic)) {
        ROS_ERROR("Failed to extract 'point_cloud_topic' from the param server");
        return false;
    }

    // subscribe to point cloud topic
    point_cloud_sub_.subscribe(nh_, point_cloud_topic, 1);
    tf_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(point_cloud_sub_, listener_, root_frame_, 2));
    tf_filter_->registerCallback(std::bind(&PickAndPlaceNode::point_cloud_callback, this, std::placeholders::_1));

    pregrasp_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("grasp_markers", 5);

    action_server_.reset(new ObjectDetectionActionServer(
            "object_detection_action", boost::bind(&PickAndPlaceNode::object_detection_callback, this, _1), false)),
    action_server_->start();

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
    last_point_cloud_ = msg;
}

void PickAndPlaceNode::print_database_contents(const std::string& database_filename) const
{
    ROS_INFO("Opening database located at %s", database_filename.c_str());
    pr2_grasp_database database(database_filename);

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

void PickAndPlaceNode::object_detection_callback(const hdt::ObjectDetectionGoal::ConstPtr& goal)
{
    ROS_INFO("Received an object detection goal request at %s from %s",
            boost::posix_time::to_simple_string(ros::Time::now().toBoost()).c_str(),
            boost::posix_time::to_simple_string(goal->header.stamp.toBoost()).c_str());

    // lazily (re)initialize object detector
    const std::string& database_filename = goal->object_database;
    const std::string& features_filename = goal->training_features;
    const std::string& kdtree_indices_filename = goal->training_kdtree;
    if (database_filename != last_database_filename_ ||
        features_filename != last_features_filename_ ||
        kdtree_indices_filename != last_kdtree_indices_filename_)
    {
        ROS_INFO("Recreating object detector");
        ROS_INFO("    Object Database: %s", database_filename.c_str());
        ROS_INFO("    Features: %s", features_filename.c_str());
        ROS_INFO("    KD-Tree Indices: %s", kdtree_indices_filename.c_str());

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
            action_server_->setAborted();
            return;
        }

        if (!object_detector_->initialize(database_filename, features_filename, kdtree_indices_filename)) {
            ROS_ERROR("Failed to initialize Object Finder");
            action_server_->setAborted();
            return;
        }

        last_database_filename_ = database_filename;
        last_features_filename_ = features_filename;
        last_kdtree_indices_filename_ = kdtree_indices_filename;
        ROS_INFO("Successfully recreated object detector");
    }

    const std::string& camera_frame = goal->camera_frame;
    const std::string& root_frame = goal->camera_frame;

    bool have_transforms = true;
    const std::string& point_cloud_frame = last_point_cloud_->header.frame_id;
    have_transforms &= listener_.canTransform(root_frame, point_cloud_frame, last_point_cloud_->header.stamp);
    have_transforms &= listener_.canTransform(point_cloud_frame, camera_frame_, last_point_cloud_->header.stamp);
    have_transforms &= listener_.canTransform(camera_frame_, root_frame, last_point_cloud_->header.stamp);

    if (!have_transforms) {
        ROS_ERROR("Transforms not available between all relevant frames");
        ROS_ERROR("    Root Frame: %s", root_frame.c_str());
        ROS_ERROR("    Camera Frame: %s", camera_frame_.c_str());
        ROS_ERROR("    Point Cloud Frame: %s", last_point_cloud_->header.frame_id.c_str());
        return;
    }

    ObjectFinder::CullingOptions cull_options;
    cull_options.cull = goal->cull_snapshot;
    cull_options.from.x = goal->cull_from.x;
    cull_options.from.y = goal->cull_from.y;
    cull_options.from.z = goal->cull_from.z;
    cull_options.to.x = goal->cull_to.x;
    cull_options.to.y = goal->cull_to.y;
    cull_options.to.z = goal->cull_to.z;

    ObjectFinder::MatchResult match_result = object_detector_->match(
            *last_point_cloud_, camera_frame, root_frame, listener_, cull_options);

    bool success = match_result.valid();

    if (success) {
        ROS_INFO("Match succeeded");

        result_.match_score = object_detector_->score_match(match_result, camera_frame);

        geometry_msgs::PoseArray grasps_out, pregrasps_out;
        object_detector_->get_grasps(match_result, camera_frame_, root_frame, grasps_out, pregrasps_out, listener_);
        ROS_INFO("Retrieved %zd grasps and %zd pre-grasps!", grasps_out.poses.size(), pregrasps_out.poses.size());

        publish_grasp_markers(grasps_out, pregrasps_out);

        result_.success = true;
        if (goal->request_snapshot) {
            result_.snapshot_cloud = *last_point_cloud_;
        }
        result_.best_cluster;
        result_.pregrasps = pregrasps_out;
        result_.grasps = grasps_out;
        action_server_->setSucceeded(result_);
    }
    else {
        ROS_WARN("Match failed");
        result_.success = false;
        action_server_->setSucceeded(result_);
    }
}

void PickAndPlaceNode::publish_grasp_markers(const geometry_msgs::PoseArray& grasps, const geometry_msgs::PoseArray& pregrasps) const
{
    int id = 0;
    visualization_msgs::MarkerArray arr;
    for (const geometry_msgs::Pose& grasp_pose : grasps.poses) {
        visualization_msgs::Marker m;
        m.header.seq = 0;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = grasps.header.frame_id;
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

    pregrasp_pub_.publish(arr);
}
