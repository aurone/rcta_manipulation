#include "CollisionModel2.h"
#include <cassert>
#include <limits>
#include <ros/ros.h>
#include <sbpl/utils/utils.h>

CollisionModel2::CollisionModel2() :
    occupancy_grid_(),
    obs_distance_transform_(),
    nonfree_distance_transform_()
{
}

CollisionModel2::~CollisionModel2()
{
    clear();
}

bool CollisionModel2::set_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid)
{
    clear();
    occupancy_grid_ = occupancy_grid;

    if (!occupancy_grid_) {
        return true;
    }

    // initialize grid from costmap 2d
    Grid<unsigned char> grid;
    fill_grid_from_occupancy_grid(grid, *occupancy_grid_, true);

    const std::uint32_t width = occupancy_grid_->info.width;
    const std::uint32_t height = occupancy_grid_->info.height;
    obs_distance_transform_.resize(width, height);
    nonfree_distance_transform_.resize(width, height);

    // count number of obstacles
    int num_obstacles = 0;
    for (auto it = grid.begin(); it != grid.end(); ++it) {
        num_obstacles += *it > 0 && *it < (std::uint8_t)-1 ? 1 : 0;
    }

    ROS_INFO("%d / %d (%0.3f %%) obstacles", num_obstacles, (int)(width * height), (double)num_obstacles / (width * height));

    // create histogram of values
    std::map<unsigned char, int> vhistogram;
    for (auto it = grid.begin(); it != grid.end(); ++it) {
        unsigned char value = *it;
        if (vhistogram.find(value) != vhistogram.end()) {
            ++vhistogram[value];
        }
        else {
            vhistogram[value] = 1;
        }
    }

    ROS_INFO("Value Histogram:");
    for (const auto& entry : vhistogram) {
        ROS_INFO("    value = %d: %d", (int)entry.first, entry.second);
    }

    computeDistancestoNonfreeAreas(
            grid.data_nocompact(),
            width,
            height,
            10,
            obs_distance_transform_.data_nocompact(),
            nonfree_distance_transform_.data_nocompact());

    int num_0 = 0;
    for (auto it = obs_distance_transform_.begin(); it != obs_distance_transform_.end(); ++it) {
        if (*it == 0.0) {
            ++num_0;
        }
    }

    ROS_INFO("Distance Transform says there are %d obstacle cells", num_0);

    FILE* fuck = fopen("fuck.txt", "w");
    for (int y = obs_distance_transform_.height() - 1; y >= 0; --y) {
        for (int x = 0; x < obs_distance_transform_.width(); ++x) {
            fprintf(fuck, "%0.3f ", obs_distance_transform_(x, y));
        }
        fprintf(fuck, "\n");
    }

    return true;
}

bool CollisionModel2::check_obstacle(const Circle& circle)
{
    if (!occupancy_grid_) {
        ROS_WARN("check_obstacle called without setting occupancy grid");
        return false;
    }

    int grid_x, grid_y;
    Result res = map_to_grid_coords(circle.x, circle.y, grid_x, grid_y);
    if (res != SUCCESS) {
        ROS_WARN("map_to_grid_coords returned %s", to_string(res));
        return false;
    }

    ROS_DEBUG("(%0.3f, %0.3f) = (%d, %d)", circle.x, circle.y, grid_x, grid_y);

    float dist_to_obs = occupancy_grid_->info.resolution * obs_distance_transform_(grid_x, grid_y);
    ROS_DEBUG("Nearest obstacle to (%0.3f, %0.3f) is %0.3f away", circle.x, circle.y, dist_to_obs);
    return circle.radius < dist_to_obs;
}

double CollisionModel2::find_nearest_obs_dist(const Circle& circle)
{
    if (!occupancy_grid_) {
        ROS_WARN("find_nearest_obs_dist called without setting occupancy grid");
        return std::numeric_limits<double>::quiet_NaN();
    }

    int grid_x, grid_y;
    Result res = map_to_grid_coords(circle.x, circle.y, grid_x, grid_y);
    if (res != SUCCESS) {
        ROS_WARN("map_to_grid_coords returned %s", to_string(res));
        return std::numeric_limits<double>::quiet_NaN();
    }

    return occupancy_grid_->info.resolution * obs_distance_transform_(grid_x, grid_y);
}

CollisionModel2::Result CollisionModel2::map_to_grid_coords(double x, double y, int& x_out, int& y_out)
{
    if (!occupancy_grid_) {
        return NULL_OCCUPANCY_GRID;
    }

    const double map_min_x = occupancy_grid_->info.origin.position.x;
    const double map_min_y = occupancy_grid_->info.origin.position.y;
    const double map_max_x = map_min_x + occupancy_grid_->info.width * occupancy_grid_->info.resolution;
    const double map_max_y = map_min_y + occupancy_grid_->info.height * occupancy_grid_->info.resolution;

    const bool within_bounds = (x >= map_min_x && x <= map_max_x && y >= map_min_y && y <= map_max_y);
    if (!within_bounds) {
        return OUT_OF_BOUNDS;
    }

    double grid_x = x - map_min_x;
    double grid_y = y - map_min_y;

    int grid_coord_x = grid_x / occupancy_grid_->info.resolution;
    int grid_coord_y = grid_y / occupancy_grid_->info.resolution;

    x_out = grid_coord_x;
    y_out = grid_coord_y;
    return SUCCESS;
}

void CollisionModel2::fill_grid_from_occupancy_grid(
    Grid<unsigned char>& grid,
    const nav_msgs::OccupancyGrid& occupancy_grid,
    bool unknown_as_free)
{
    const int width = (int)occupancy_grid.info.width;
    const int height = (int)occupancy_grid.info.height;
    grid.resize(width, height);
    for (std::size_t x = 0; x < width; ++x) {
        for (std::size_t y = 0; y < height; ++y) {
            unsigned char value = occupancy_grid_->data[y * width + x];
            if (unknown_as_free && value == /*unknown*/ 0xFF) {
                grid(x, y) = 0;
            }
            else {
                grid(x, y) = occupancy_grid_->data[y * width + x];
            }
        }
    }
}

void CollisionModel2::clear()
{
    occupancy_grid_.reset();
    obs_distance_transform_.clear();
    nonfree_distance_transform_.clear();
}
