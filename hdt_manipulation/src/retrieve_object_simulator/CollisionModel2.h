#ifndef CollisionModel2_h
#define CollisionModel2_h

#include <nav_msgs/OccupancyGrid.h>
#include "Grid.h"

struct Circle
{
    double x, y, radius;
};

class CollisionModel2
{
public:

    CollisionModel2();
    ~CollisionModel2();

    bool set_occupancy_grid(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid);
    bool check_obstacle(const Circle& circle);
    double find_nearest_obs_dist(const Circle& circle);

private:

    nav_msgs::OccupancyGrid::ConstPtr occupancy_grid_;
    Grid<float> obs_distance_transform_;
    Grid<float> nonfree_distance_transform_;

    enum Result
    {
        SUCCESS = 0,
        NULL_OCCUPANCY_GRID,
        OUT_OF_BOUNDS
    };
    static const char* to_string(Result r)
    {
        switch (r) {
        case SUCCESS:
            return "SUCCESS";
        case NULL_OCCUPANCY_GRID:
            return "NULL_OCCUPANCY_GRID";
        case OUT_OF_BOUNDS:
            return "OUT_OF_BOUNDS";
        }
    }

    Result map_to_grid_coords(double x, double y, int& grid_x, int& grid_y);

    void fill_grid_from_occupancy_grid(
            Grid<unsigned char>& grid,
            const nav_msgs::OccupancyGrid& occupancy_grid,
            bool unknown_as_free);

    void clear();
};

#endif
