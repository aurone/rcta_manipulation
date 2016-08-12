#ifndef extrude_occupancy_grid_h
#define extrude_occupancy_grid_h

#include <cstdint>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <moveit_msgs/CollisionMap.h>
#include <nav_msgs/OccupancyGrid.h>

/// @example
/// CostmapExtruder extruder(100, false);
/// octomap_pub.publish(extruder.extrude(grid, 3.0));

/// @brief Class used to convert nav_msgs/OccupancyGrid messages to octomap_msgs/Octomap messages via extrusion along the z-axis.
class CostmapExtruder
{
public:

    CostmapExtruder(std::int8_t obs_threshold, bool unknown_obstacles);

    octomap_msgs::Octomap::ConstPtr extrude(const nav_msgs::OccupancyGrid& grid, double extrusion);

private:

    bool unknown_obstacles_;
    int obs_threshold_;

    moveit_msgs::CollisionMap extrude_to_collision_map(const nav_msgs::OccupancyGrid& grid, double extrusion) const;
    moveit_msgs::OrientedBoundingBox get_bbx(const moveit_msgs::CollisionMap& collision_map) const;
    void log_octomap(const octomap::OcTree& octree) const;
    void convert(const moveit_msgs::CollisionMap& collision_map, octomap::Pointcloud& point_cloud) const;
};

#endif
