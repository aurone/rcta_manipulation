#ifndef SplineVisualizer_h
#define SplineVisualizer_h

// standard includes
#include <cstdio>
#include <string>
#include <memory>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <spellbook/geometry/nurb/NURB.h>
#include <visualization_msgs/Marker.h>

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
    ros::Publisher control_vertices_marker_pub_;
    ros::Publisher spline_marker_pub_;

    std::vector<Eigen::Vector3d> grasp_spline_control_points_;

    Eigen::Affine3d gas_canister_to_base_footprint_;
    Eigen::Affine3d base_footprint_to_gas_canister_;

    visualization_msgs::Marker canister_marker_;
    visualization_msgs::Marker control_vertices_marker_;
    visualization_msgs::Marker spline_marker_;

    double gas_can_scale_;

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
    bool init_control_vertex_marker();
    bool init_spline_marker();
};

#endif
