#include "CostmapExtruder.h"
#include <leatherman/print.h>
#include <octomap_msgs/conversions.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/msg_utils/msg_utils.h>

CostmapExtruder::CostmapExtruder(std::int8_t obs_threshold, bool unknown_obstacles) :
    unknown_obstacles_(unknown_obstacles),
    obs_threshold_(obs_threshold)
{
}

octomap_msgs::Octomap::ConstPtr CostmapExtruder::extrude(const nav_msgs::OccupancyGrid& grid, double extrusion)
{
    moveit_msgs::CollisionMap collision_map = extrude_to_collision_map(grid, extrusion);

    // convert the collision map to a point cloud
    ROS_INFO_PRETTY("Creating Octomap from Collision Map");
    ROS_INFO_PRETTY("  %zd oriented bounding box cells", collision_map.boxes.size());
    const double res = collision_map.boxes.front().extents.x;
    ROS_INFO_PRETTY("  Resolution: %0.3f", res);

    octomap::Pointcloud octomap_cloud;
    convert(collision_map, octomap_cloud);

    octomap::OcTree octree(res);
    octree.insertScan(octomap_cloud, octomap::point3d(0.0, 0.0, 0.0));

    log_octomap(octree);

    octomap_msgs::Octomap::Ptr octomap(new octomap_msgs::Octomap);
    if (!octomap) {
        ROS_ERROR_PRETTY("Failed to instantiate octomap_msgs/Octomap");
        return octomap;
    }

    if (!octomap_msgs::fullMapToMsg(octree, *octomap)) {
        ROS_ERROR("Failed to convert OcTree to octomap_msgs/Octomap");
        return octomap;
    }

    octomap->header = collision_map.header;
    return octomap;
}

moveit_msgs::CollisionMap
CostmapExtruder::extrude_to_collision_map(const nav_msgs::OccupancyGrid& grid, double extrusion) const
{
    ROS_INFO_PRETTY("Extruding Occupancy Grid");

    moveit_msgs::CollisionMap cmap;

    cmap.header.seq = 0;
    cmap.header.stamp = ros::Time(0);
    cmap.header.frame_id = grid.header.frame_id;

    const std::uint32_t width = grid.info.width;
    const std::uint32_t height = grid.info.height;
    const float res = grid.info.resolution;

    int num_occupied_cells = 0;
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            // only add occuped cells to the collision map
            bool obstacle =
                (grid.data[y * width + x] >= obs_threshold_) || (unknown_obstacles_ && grid.data[y * width + x] < 0);

            if (!obstacle) {
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
                cmap.boxes.push_back(bb);
            }
        }
    }

    ROS_INFO_PRETTY("Extruded %d occupied cells", num_occupied_cells);

    moveit_msgs::OrientedBoundingBox bbx = get_bbx(cmap);
    ROS_INFO_PRETTY("  Collision Map Bounding Box: %s", to_string(bbx).c_str());

    return cmap;
}

moveit_msgs::OrientedBoundingBox CostmapExtruder::get_bbx(const moveit_msgs::CollisionMap& collision_map) const
{
    moveit_msgs::OrientedBoundingBox bbx;

    double min_x, min_y, min_z, max_x, max_y, max_z;
    for (const auto& obbx : collision_map.boxes) {
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

    bbx.extents.x = max_x - min_x;
    bbx.extents.y = max_y - min_y;
    bbx.extents.z = max_z - min_z;
    bbx.pose.position.x = 0.5 * (max_x + min_x);
    bbx.pose.position.y = 0.5 * (max_y + min_y);
    bbx.pose.position.z = 0.5 * (max_z + min_z);
    bbx.pose.orientation = geometry_msgs::IdentityQuaternion();

    return bbx;
}

void CostmapExtruder::log_octomap(const octomap::OcTree& octree) const
{
    size_t num_nodes = octree.calcNumNodes();
    ROS_INFO_PRETTY("  Num Nodes: %zd", num_nodes);
    ROS_INFO_PRETTY("  Memory Usage: %zd bytes", octree.memoryUsage());
    ROS_INFO_PRETTY("  Num Leaf Nodes: %zd", octree.getNumLeafNodes());

    unsigned num_thresholded, num_other;
    octree.calcNumThresholdedNodes(num_thresholded, num_other);
    ROS_INFO_PRETTY("  Num Thresholded Nodes: %u", num_thresholded);
    ROS_INFO_PRETTY("  Num Other Nodes: %u", num_other);

    const octomap::point3d octomap_min = octree.getBBXMin();
    const octomap::point3d octomap_max = octree.getBBXMax();
    const octomap::point3d octomap_center = octree.getBBXCenter();
    double clamping_thresh_min = octree.getClampingThresMin();
    double clamping_thresh_max = octree.getClampingThresMax();

    ROS_INFO_PRETTY("  Bounding Box Set: %s", octree.bbxSet() ? "TRUE" : "FALSE");
    ROS_INFO_PRETTY("  Bounding Box Min: (%0.3f, %0.3f, %0.3f)", octomap_min.x(), octomap_min.y(), octomap_min.z());
    ROS_INFO_PRETTY("  Bounding Box Max: (%0.3f, %0.3f, %0.3f)", octomap_max.x(), octomap_max.y(), octomap_max.z());
    ROS_INFO_PRETTY("  Bounding Box Center: (%0.3f, %0.3f, %0.3f)", octomap_center.x(), octomap_center.y(), octomap_center.z());
    ROS_INFO_PRETTY("  Clamping Threshold Min: %0.3f", clamping_thresh_min);
    ROS_INFO_PRETTY("  Clamping Threshold Max: %0.3f", clamping_thresh_max);

    double metric_min_x, metric_min_y, metric_min_z;
    double metric_max_x, metric_max_y, metric_max_z;
    double metric_size_x, metric_size_y, metric_size_z;
    octree.getMetricMin(metric_min_x, metric_min_y, metric_min_z);
    octree.getMetricMax(metric_max_x, metric_max_y, metric_max_z);

    ROS_INFO_PRETTY("  Metric Min: (%0.3f, %0.3f, %0.3f)", metric_min_x, metric_min_y, metric_min_z);
    ROS_INFO_PRETTY("  Metric Max: (%0.3f, %0.3f, %0.3f)", metric_max_x, metric_max_y, metric_max_z);

    octree.getMetricSize(metric_size_x, metric_size_y, metric_size_z);
    ROS_INFO_PRETTY("  Metric Size: (%0.3f, %0.3f, %0.3f)", metric_size_x, metric_size_y, metric_size_z);

    ROS_INFO_PRETTY("  Node Size (max depth): %0.6f", octree.getNodeSize(octree.getTreeDepth()));
    ROS_INFO_PRETTY("  Occupancy Threshold: %0.3f", octree.getOccupancyThres());
    ROS_INFO_PRETTY("  Probability Hit: %0.3f", octree.getProbHit());
    ROS_INFO_PRETTY("  Probability Miss: %0.3f", octree.getProbMiss());
    ROS_INFO_PRETTY("  Resolution: %0.3f", octree.getResolution());
    ROS_INFO_PRETTY("  Depth: %u", octree.getTreeDepth());
    ROS_INFO_PRETTY("  Tree Type: %s", octree.getTreeType().c_str());
}

void CostmapExtruder::convert(const moveit_msgs::CollisionMap& collision_map, octomap::Pointcloud& octomap_cloud) const
{
    octomap_cloud.clear();
    octomap_cloud.reserve(collision_map.boxes.size());
    for (const moveit_msgs::OrientedBoundingBox& box : collision_map.boxes) {
        octomap_cloud.push_back(box.pose.position.x, box.pose.position.y, box.pose.position.z);
    }
}
