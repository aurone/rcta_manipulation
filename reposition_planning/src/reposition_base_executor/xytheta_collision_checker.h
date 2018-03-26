#ifndef _XYTHETA_COLLISION_CHECKER_H_
#define _XYTHETA_COLLISION_CHECKER_H_

#include <sbpl/headers.h>
#include <ros/ros.h>
#include <spellbook/grid/grid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

class XYThetaCollisionChecker
{
public:

    XYThetaCollisionChecker(
        const std::vector<sbpl_2Dpt_t>& footprint_polygon,
        int obs_thresh,
        int num_heading_disc = 16);

    ~XYThetaCollisionChecker();

    bool updateOccupancyGrid(const nav_msgs::OccupancyGrid& new_occ_grid);

    bool isValidState(double x, double y, double theta);
    bool isValidState(int x, int y, int theta);

    double getCellObstacleDistance(int x, int y);
    double getCellObstacleDistance(double x, double y);

    double getCollisionProbability(double x, double y, double theta);
    double getCollisionProbability(int x, int y, int theta);

    visualization_msgs::MarkerArray
    getFootprintVisualization(double x, double y, double theta) const;

    visualization_msgs::MarkerArray
    getDiscreteFootprintVisualization(double x, double y, double theta) const;

private:

    nav_msgs::OccupancyGrid m_grid;
    std::vector<sbpl_2Dpt_t> m_footprint_polygon;
    int m_num_heading_disc;
    int m_obs_thresh;

    // indexed by discrete heading value
    double m_footprint_res;
    std::vector<std::vector<sbpl_2Dcell_t>> m_precomputed_footprint_cells;

    au::grid<2, double> m_distance_map;

    bool updateDistanceMap(const nav_msgs::OccupancyGrid& grid);

    bool reinitFootprintCells(double new_res);

    bool precomputeFootprints(double res);

    bool calculateFootprintForPose(
        EnvNAVXYTHETALAT3Dpt_t pose,
        std::vector<sbpl_2Dcell_t> &footprint_cells,
        const std::vector<sbpl_2Dpt_t>& FootprintPolygon,
        double res);

    bool isValidCell(int x, int y);
    double getCellColllisionProbability(int x, int y);

    int ipow(int b, int e);

    bool initialized() const;
};

#endif
