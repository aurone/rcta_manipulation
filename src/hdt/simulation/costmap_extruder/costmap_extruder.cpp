#include <cassert>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include "CostmapExtruder.h"

nav_msgs::OccupancyGrid::ConstPtr TheOnlyOccupancyGrid;

void occupancy_grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    TheOnlyOccupancyGrid = msg;
}

void WaitForOccupancyGrid()
{
    // wait for an occupancy grid to arrive
    while (ros::ok() && !TheOnlyOccupancyGrid) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
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

    WaitForOccupancyGrid();
    assert((bool)TheOnlyOccupancyGrid);

    if (!ros::ok()) {
        return 0;
    }

    assert(TheOnlyOccupancyGrid);

    std::int8_t obs_threshold = 100;
    CostmapExtruder extruder(obs_threshold, false);

    const double unconfigured_height = 2.0;
    octomap_msgs::Octomap::ConstPtr octomap = extruder.extrude(*TheOnlyOccupancyGrid, unconfigured_height);
    if (!octomap) {
        ROS_ERROR("Failed to extrude nav_msgs/OccupancyGrid to octomap_msgs/Octomap");
        return 1;
    }

    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("fixed_octomap", 1, true);
    octomap_pub.publish(octomap);
    ros::spin();

    return 0;
}
