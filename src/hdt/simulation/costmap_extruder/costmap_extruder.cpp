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
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            // only add occuped cells to the collision map
            bool occupied = (grid.data[y * width + x] >= obs_thresh);
            if (!occupied) {
                continue;
            }

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

    return cmap;
}

octomap_msgs::Octomap::ConstPtr convert_to_octomap(const moveit_msgs::CollisionMap& cmap)
{
    // convert the collision map to a point cloud
    octomap::Pointcloud octomap_cloud;
    octomap_cloud.reserve(cmap.boxes.size());
    for (const moveit_msgs::OrientedBoundingBox& box : cmap.boxes) {
        octomap_cloud.push_back(box.pose.position.x, box.pose.position.y, box.pose.position.z);
    }

    const double res = cmap.boxes.front().extents.x;

    octomap::OcTree octree(res);
    octree.insertScan(octomap_cloud, octomap::point3d(0.0, 0.0, 0.0));

    std::vector<std::int8_t> octomap_msg_data;
    if (!octomap_msgs::binaryMapToMsgData(octree, octomap_msg_data)) {
        return octomap_msgs::Octomap::ConstPtr();
    }

    octomap_msgs::Octomap::Ptr octomap(new octomap_msgs::Octomap);
    if (!octomap) {
        return octomap;
    }

    octomap->header = cmap.header;
    octomap->binary = true;
    octomap->id = "extruded_collision_map";
    octomap->resolution = res;
    octomap->data = octomap_msg_data;
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
