#ifndef SplineVisualizer_h
#define SplineVisualizer_h

#include <cstdio>
#include <string>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include "NURB.h"

class SplineVisualizer
{
public:

    SplineVisualizer();
    virtual ~SplineVisualizer();

    enum MainResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE,
        FAILED_TO_VISUALIZE_GAS_CANISTER,
        FAILED_TO_VISUALIZE_GRASP_SPLINE
    };

    int run();

    void display();
    void reshape(int w, int h);

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    Nurb<Eigen::Vector3d> nurb_;
    std::unique_ptr<Nurb<Eigen::Vector3d>> grasp_spline_;

    ros::Publisher marker_pub_;
    ros::Publisher spline_marker_pub_;

    std::vector<Eigen::Vector3d> grasp_spline_control_points_;

    Eigen::Affine3d gascanister_to_basefootprint_;
    Eigen::Affine3d base_footprint_to_gascanister_;

    visualization_msgs::Marker canister_marker_;
    visualization_msgs::Marker spline_marker_;

    bool load_curve();

    void plot(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);

    void plot_arrows(
            const std::vector<double>& x,
            const std::vector<double>& y,
            const std::vector<double>& z,
            const std::vector<double>& dx,
            const std::vector<double>& dy,
            const std::vector<double>& dz,
            const std::string& color = "red");

    bool init_canister_marker();
    bool init_spline_marker();
};

#endif
