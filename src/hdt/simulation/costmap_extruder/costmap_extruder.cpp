#include <cassert>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionMap.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdt/common/msg_utils/msg_utils.h>

nav_msgs::OccupancyGrid::ConstPtr TheOnlyOccupancyGrid;

void occupancy_grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    TheOnlyOccupancyGrid = msg;
}

void WaitForOctomap()
{
    // wait for an occupancy grid to arrive
    while (ros::ok() && !TheOnlyOccupancyGrid) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
}

moveit_msgs::CollisionMap::ConstPtr extrude_occupancy_grid(const nav_msgs::OccupancyGrid& grid, double extrusion)
{
    ROS_INFO("Extruding Occupancy Grid");
    moveit_msgs::CollisionMap::Ptr cmap(new moveit_msgs::CollisionMap);
    if (!cmap) {
        return cmap;
    }

    cmap->header.seq = 0;
    cmap->header.stamp = ros::Time(0);
    cmap->header.frame_id = grid.header.frame_id;

    const std::uint32_t width = grid.info.width;
    const std::uint32_t height = grid.info.height;
    const float res = grid.info.resolution;

    const std::int8_t obs_thresh = 100;
    int num_occupied_cells = 0;
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            // only add occuped cells to the collision map
            bool occupied = (grid.data[y * width + x] >= obs_thresh);
            if (!occupied) {
                continue;
            }

            ++num_occupied_cells;

            double map_x = grid.info.origin.position.x + x * res + 0.5 * res;
            double map_y = grid.info.origin.position.y + y * res + 0.5 * res;
            for (double z = 0.0; z <= extrusion; z += res) {
                const double& map_z = z + 0.5 * res;

                geometry_msgs::Point box_pos(geometry_msgs::CreatePoint(map_x, map_y, map_z));
                geometry_msgs::Quaternion box_rot(geometry_msgs::IdentityQuaternion());
                geometry_msgs::Pose box_pose(geometry_msgs::CreatePose(box_pos, box_rot));
                geometry_msgs::Point32 box_extents(geometry_msgs::CreatePoint32(res, res, res));

                moveit_msgs::OrientedBoundingBox bb;
                bb.pose = box_pose;
                bb.extents = box_extents;
                cmap->boxes.push_back(bb);
            }
        }
    }

    ROS_INFO("Extruded %d occupied cells", num_occupied_cells);

    double min_x, min_y, min_z, max_x, max_y, max_z;
    for (const auto& obbx : cmap->boxes) {
        if (obbx.pose.position.x < min_x) {
            min_x = obbx.pose.position.x;
        }
        if (obbx.pose.position.y < min_y) {
            min_y = obbx.pose.position.y;
        }
        if (obbx.pose.position.z < min_z) {
            min_z = obbx.pose.position.z;
        }

        if (obbx.pose.position.x > max_x) {
            max_x = obbx.pose.position.x;
        }
        if (obbx.pose.position.y > max_y) {
            max_y = obbx.pose.position.y;
        }
        if (obbx.pose.position.z > max_z) {
            max_z = obbx.pose.position.z;
        }
    }

    ROS_INFO("Max Point: %0.3f, %0.3f, %0.3f", min_x, min_y, min_z);
    ROS_INFO("Max Point: %0.3f, %0.3f, %0.3f", max_x, max_y, max_z);

    return cmap;
}

void LogOctomap(const octomap::OcTree& octree)
{
    size_t num_nodes = octree.calcNumNodes();
    ROS_INFO("  Num Nodes: %zd", num_nodes);
    ROS_INFO("  Memory Usage: %zd bytes", octree.memoryUsage());
    ROS_INFO("  Num Leaf Nodes: %zd", octree.getNumLeafNodes());

    unsigned num_thresholded, num_other;
    octree.calcNumThresholdedNodes(num_thresholded, num_other);
    ROS_INFO("  Num Thresholded Nodes: %u", num_thresholded);
    ROS_INFO("  Num Other Nodes: %u", num_other);

    const octomap::point3d octomap_min = octree.getBBXMin();
    const octomap::point3d octomap_max = octree.getBBXMax();
    const octomap::point3d octomap_center = octree.getBBXCenter();
    double clamping_thresh_min = octree.getClampingThresMin();
    double clamping_thresh_max = octree.getClampingThresMax();

    ROS_INFO("  Bounding Box Set: %s", octree.bbxSet() ? "TRUE" : "FALSE");
    ROS_INFO("  Bounding Box Min: (%0.3f, %0.3f, %0.3f)", octomap_min.x(), octomap_min.y(), octomap_min.z());
    ROS_INFO("  Bounding Box Max: (%0.3f, %0.3f, %0.3f)", octomap_max.x(), octomap_max.y(), octomap_max.z());
    ROS_INFO("  Bounding Box Center: (%0.3f, %0.3f, %0.3f)", octomap_center.x(), octomap_center.y(), octomap_center.z());
    ROS_INFO("  Clamping Threshold Min: %0.3f", clamping_thresh_min);
    ROS_INFO("  Clamping Threshold Max: %0.3f", clamping_thresh_max);

    double metric_min_x, metric_min_y, metric_min_z;
    double metric_max_x, metric_max_y, metric_max_z;
    double metric_size_x, metric_size_y, metric_size_z;
    octree.getMetricMin(metric_min_x, metric_min_y, metric_min_z);
    octree.getMetricMax(metric_max_x, metric_max_y, metric_max_z);

    ROS_INFO("  Metric Min: (%0.3f, %0.3f, %0.3f)", metric_min_x, metric_min_y, metric_min_z);
    ROS_INFO("  Metric Max: (%0.3f, %0.3f, %0.3f)", metric_max_x, metric_max_y, metric_max_z);

    octree.getMetricSize(metric_size_x, metric_size_y, metric_size_z);
    ROS_INFO("  Metric Size: (%0.3f, %0.3f, %0.3f)", metric_size_x, metric_size_y, metric_size_z);

    ROS_INFO("  Node Size (max depth): %0.6f", octree.getNodeSize(octree.getTreeDepth()));
    ROS_INFO("  Occupancy Threshold: %0.3f", octree.getOccupancyThres());
    ROS_INFO("  Probability Hit: %0.3f", octree.getProbHit());
    ROS_INFO("  Probability Miss: %0.3f", octree.getProbMiss());
    ROS_INFO("  Resolution: %0.3f", octree.getResolution());
    ROS_INFO("  Depth: %u", octree.getTreeDepth());
    ROS_INFO("  Tree Type: %s", octree.getTreeType().c_str());
}

octomap_msgs::Octomap::ConstPtr convert_to_octomap(const moveit_msgs::CollisionMap& cmap)
{
    // convert the collision map to a point cloud
    ROS_INFO("Creating Octomap from Collision Map");
    ROS_INFO("  %zd oriented bounding box cells", cmap.boxes.size());

    octomap::Pointcloud octomap_cloud;
    octomap_cloud.reserve(cmap.boxes.size());
    for (const moveit_msgs::OrientedBoundingBox& box : cmap.boxes) {
        octomap_cloud.push_back(box.pose.position.x, box.pose.position.y, box.pose.position.z);
    }

    const double res = cmap.boxes.front().extents.x;
    ROS_INFO("  Resolution: %0.3f", res);

    octomap::OcTree octree(res);
    octree.insertScan(octomap_cloud, octomap::point3d(0.0, 0.0, 0.0));

    LogOctomap(octree);

    octomap_msgs::Octomap::Ptr octomap(new octomap_msgs::Octomap);
    if (!octomap) {
        return octomap;
    }

    if (!octomap_msgs::fullMapToMsg(octree, *octomap)) {
        return octomap;
    }

    octomap->header = cmap.header;
    return octomap;
}

/// @bright Lightweight node to:
///     1. Receive a costmap (only one, assumed to be on a latched topic)
///     2. Extrude that costmap to some height to create a 3D costmap
///     3. Convert the extruded costmap into an octree (octomap_msgs/Octomap)
///     4. Publish the octomap on a latched topic
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "costmap_extruder");

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Subscriber occupancy_grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>("occupancy_grid", 1, occupancy_grid_cb);

    WaitForOctomap();
    if (!ros::ok()) {
        return 0;
    }

    assert(TheOnlyOccupancyGrid);

    const double unconfigured_height = 2.0;
    moveit_msgs::CollisionMap::ConstPtr cmap = extrude_occupancy_grid(*TheOnlyOccupancyGrid, unconfigured_height);
    if (!cmap) {
        ROS_ERROR("Failed to extrude nav_msgs/OccupancyGrid");
        return 1;
    }

    octomap_msgs::Octomap::ConstPtr octomap = convert_to_octomap(*cmap);
    if (!octomap) {
        ROS_ERROR("Failed to convert moveit_msgs/CollisionMap to octomap_msgs/Octomap");
    }

    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("fixed_octomap", 1, true);
    octomap_pub.publish(octomap);
    ros::spin();

    return 0;
}
