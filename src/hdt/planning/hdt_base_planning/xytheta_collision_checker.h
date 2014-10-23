
#ifndef _XYTHETA_COLLISION_CHECKER_H_
#define _XYTHETA_COLLISION_CHECKER_H_

#include <sbpl/headers.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class XYThetaCollisionChecker {

public:
    XYThetaCollisionChecker(const std::vector<sbpl_2Dpt_t> &footprint_polygon, int obs_thresh, int num_heading_disc=16);
    ~XYThetaCollisionChecker();

    void UpdateOccupancyGrid(nav_msgs::OccupancyGrid new_occ_grid);

    bool isValidState(double x, double y, double theta);

    bool isValidState(int x, int y, int theta); //discrete coords version

    double getCollisionProbability(int x, int y, int theta);

    double getCollisionProbability(double x, double y, double theta);

private:
    nav_msgs::OccupancyGrid *grid_;
    std::vector<sbpl_2Dpt_t> footprint_polygon_;
    int num_heading_disc_;
    int obs_thresh_;

    std::vector< std::vector<sbpl_2Dcell_t> > precomputed_footprint_cells; //indexed by discrete heading value
    bool footprints_initialized_;

    bool reinit();

    bool PreComputeFootprints();

    bool CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, std::vector<sbpl_2Dcell_t> &footprint_cells, const std::vector<sbpl_2Dpt_t>& FootprintPolygon);

    bool IsValidCell(int x, int y);
    double getCellColllisionProbability(int x, int y);
};

#endif
