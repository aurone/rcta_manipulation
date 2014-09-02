#include "SplineVisualizer.h"

#include <cmath>
#include <limits>
#include <string>
#include <Eigen/Dense>
#include <GL/freeglut.h>
#include <eigen_conversions/eigen_msg.h>
#include <rospack/rospack.h>
#include <visualization_msgs/Marker.h>
#include <sbpl_geometry_utils/utils.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/matplotpp/matplotpp.h>
#include <hdt/common/msg_utils/msg_utils.h>

using namespace Eigen;

SplineVisualizer::SplineVisualizer() :
    nh_(),
    ph_("~"),
    nurb_({
        Vector3d(0.0, 0.0, 0),
        Vector3d(1, 2, 0),
        Vector3d(2, 1.75, 0),
        Vector3d(3, 1.5, 0),
        Vector3d(5, -1, 0),
        Vector3d(6, 1, 0) },
        2),
    marker_pub_(),
    spline_marker_pub_(),
    gas_can_scale_(0.1)
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
}

SplineVisualizer::~SplineVisualizer()
{
}

int SplineVisualizer::run()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 5, false);
    control_vertices_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 5, false);
    spline_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 5, false);

    if (!load_curve()) {
        ROS_ERROR("Failed to load curve from parameter server");
        return FAILED_TO_INITIALIZE;
    }

    if (!init_canister_marker()) {
        ROS_ERROR("Failed to visualize gas canister");
        return FAILED_TO_VISUALIZE_GAS_CANISTER;
    }

    if (!init_control_vertex_marker()) {
        return FAILED_TO_INITIALIZE;
    }

    if (!init_spline_marker()) {
        ROS_ERROR("Failed to visualize grasp spline");
        return FAILED_TO_VISUALIZE_GRASP_SPLINE;
    }

    ros::Rate vis_rate(1.0);
    while (ros::ok()) {
        marker_pub_.publish(canister_marker_);
        control_vertices_marker_pub_.publish(control_vertices_marker_);
        spline_marker_pub_.publish(spline_marker_);
        ros::spinOnce();
        vis_rate.sleep();
    }

    ROS_INFO("Spline Visualizer returned successfully");
    return SUCCESS;
}

void SplineVisualizer::display()
{
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        printf("Displaying...\n");
    const int num_samples = 100;
    const double min_u = 0.0;
    const double max_u = 1.0;

    std::vector<double> xs(num_samples), ys(num_samples), zs(num_samples);
    std::vector<double> dxs(num_samples), dys(num_samples), dzs(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        double u = (max_u - min_u) / (num_samples - 1) * i;

        Vector3d p = nurb_(u);
        xs[i] = p.x();
        ys[i] = p.y();
        zs[i] = p.z();

        p = nurb_.deriv(u);
        dxs[i] = p.x();
        dys[i] = p.y();
        dzs[i] = p.z();
    }

    plot(xs, ys, zs);
    plot_arrows(xs, ys, zs, dxs, dys, dzs);

    const Vector3d bias(0, 1, 0);
    for (std::size_t i = 0; i < xs.size(); ++i) {
        Vector3d x(xs[i], ys[i], zs[i]);
        Vector3d d(dxs[i], dys[i], dzs[i]);

        d.normalize();
        Vector3d grasp_dir = bias - bias.dot(d) * d;
        grasp_dir.normalize();
        grasp_dir *= 0.3;

        Vector3d grasp_pos = x + grasp_dir;
        grasp_dir = -grasp_dir;

        xs[i] = grasp_pos.x();
        ys[i] = grasp_pos.y();
        zs[i] = grasp_pos.z();

        dxs[i] = grasp_dir.x();
        dys[i] = grasp_dir.y();
        dzs[i] = grasp_dir.z();

        printf("Grasp dir: %s\n", to_string(grasp_dir).c_str());
    }

    plot_arrows(xs, ys, zs, dxs, dys, dzs, "blue");

    glutSwapBuffers();
}

void SplineVisualizer::reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//        gluOrtho2D(-1.05, 1.05, -1.05, 1.05); // for basis functions
    gluOrtho2D(-1.0, 7.0, -3.0, 3.0); // for the points
//        gluOrtho2D(-10.0, 16.0, -12.0, 12.0); // for the points
    glMatrixMode(GL_MODELVIEW);
}

bool SplineVisualizer::load_curve()
{
    std::vector<geometry_msgs::Point> control_points;
    int degree;
    if (!msg_utils::download_param(ph_, "degree", degree) ||
        !msg_utils::download_param(ph_, "control_points", control_points))
    {
        ROS_ERROR("Failed to retrieve parameters");
        return false;
    }

    grasp_spline_control_points_.reserve(control_points.size());
    for (const geometry_msgs::Point& p : control_points) {
        grasp_spline_control_points_.push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    grasp_spline_.reset(new Nurb<Eigen::Vector3d>(grasp_spline_control_points_, degree));
    if (!grasp_spline_) {
        ROS_ERROR("Failed to instantiate Nurb");
        return false;
    }

    ROS_INFO("Control Points:");
    for (const Vector3d& control_vertex : grasp_spline_->control_points()) {
        ROS_INFO("    %s", to_string(control_vertex).c_str());
    }

    ROS_INFO("Knot Vector: %s", to_string(grasp_spline_->knots()).c_str());
    ROS_INFO("Degree: %s", std::to_string(grasp_spline_->degree()).c_str());

    ROS_INFO("Loaded curve with %zd points and degree %d", grasp_spline_control_points_.size(), degree);
    return true;
}

void SplineVisualizer::plot(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z)
{
    glColor3d(0.0, 0.0, 0.0);
    glBegin(GL_POINTS);
    for (std::size_t i = 0; i < x.size(); ++i) {
        glVertex3d(x[i], y[i], z[i]);
    }
    glEnd();
}

void SplineVisualizer::plot_arrows(
    const std::vector<double>& x,
    const std::vector<double>& y,
    const std::vector<double>& z,
    const std::vector<double>& dx,
    const std::vector<double>& dy,
    const std::vector<double>& dz,
    const std::string& color)
{
    if (color == "red") {
        glColor3d(1.0, 0.0, 0.0);
    }
    else if (color == "blue") {
        glColor3f(0.0, 0.0, 1.0);
    }
    glBegin(GL_LINES);
    for (std::size_t i = 0; i < x.size(); ++i) {
        Vector3d d(dx[i], dy[i], dz[i]);
        d.normalize();
        glVertex3d(x[i], y[i], z[i]);
        glVertex3d(x[i] + 0.3 * d.x(), y[i] + 0.3 * d.y(), z[i] + 0.3 * d.z());
    }
    glEnd();
}

bool SplineVisualizer::init_canister_marker()
{
    rospack::Rospack rpack;
    std::vector<std::string> search_path;
    if (!rpack.getSearchPathFromEnv(search_path)) {
        ROS_ERROR("Failed to load ROS package search path from environment");
        return false;
    }

    rpack.crawl(search_path, false);

    std::string hdt_package_path;
    rpack.find("hdt", hdt_package_path);

    ROS_INFO("Found package 'hdt' at %s", hdt_package_path.c_str());

    // publish a visualization marker with the gas canister
    canister_marker_.header.seq = 0;
    canister_marker_.header.stamp = ros::Time::now();
    canister_marker_.header.frame_id = "base_footprint";
    canister_marker_.ns = "gas_canister";
    canister_marker_.id = 0;
    canister_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    canister_marker_.action = visualization_msgs::Marker::ADD;

    base_footprint_to_gas_canister_ =
            Eigen::Translation3d(1, 0, 0.1) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(60.0), Eigen::Vector3d(0, 0, 1)) *
            Eigen::AngleAxisd(sbpl::utils::ToRadians(90.0), Eigen::Vector3d(1, 0, 0));

    tf::poseEigenToMsg(base_footprint_to_gas_canister_, canister_marker_.pose);

    canister_marker_.scale.x = canister_marker_.scale.y = canister_marker_.scale.z = gas_can_scale_;
    canister_marker_.color.r = canister_marker_.color.g = canister_marker_.color.b = 0.5;
    canister_marker_.color.a = 0.5;
    canister_marker_.lifetime = ros::Duration(0);
    canister_marker_.frame_locked = false;
    canister_marker_.mesh_resource = "file://" + hdt_package_path + "/resource/meshes/gastank/clean_small_gastank.obj";
    canister_marker_.mesh_use_embedded_materials = false;

    return true;
}

bool SplineVisualizer::init_control_vertex_marker()
{
    control_vertices_marker_.header.seq = 0;
    control_vertices_marker_.header.stamp = ros::Time::now();
    control_vertices_marker_.header.frame_id = "base_footprint";

    control_vertices_marker_.ns = "control_vertices";
    control_vertices_marker_.id = 0;
    control_vertices_marker_.type = visualization_msgs::Marker::POINTS;
    control_vertices_marker_.action = visualization_msgs::Marker::ADD;
    for (const Vector3d& cv : grasp_spline_control_points_) {
        Vector3d control_vertex_robot_frame = base_footprint_to_gas_canister_ * Eigen::Scaling(gas_can_scale_) * cv;
        geometry_msgs::Point p;
        p.x = control_vertex_robot_frame.x();
        p.y = control_vertex_robot_frame.y();
        p.z = control_vertex_robot_frame.z();
        control_vertices_marker_.points.push_back(p);
    }
    control_vertices_marker_.scale.x = 0.01;
    control_vertices_marker_.color.r = 1.0;
    control_vertices_marker_.color.g = 0.5;
    control_vertices_marker_.color.b = 0.0;
    control_vertices_marker_.color.a = 1.0;
    control_vertices_marker_.lifetime = ros::Duration(0);
    control_vertices_marker_.frame_locked = false;
    return true;
}

bool SplineVisualizer::init_spline_marker()
{
    spline_marker_.header.seq = 0;
    spline_marker_.header.stamp = ros::Time::now();
    spline_marker_.header.frame_id = "base_footprint";
    spline_marker_.ns = "grasp_spline";
    spline_marker_.id = 0;
    spline_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    spline_marker_.action = visualization_msgs::Marker::ADD;

    // evaluate the grasp spline at a number of points for visualizatio
    double max_samples = 100;
    double min_u = 0.0;
    double max_u = 1.0;
    for (int i = 0; i < max_samples; ++i)
    {
        double u = (max_u - min_u) * i / (max_samples - 1);

        ROS_INFO("Spline point %d (u = %0.8f)", i, u);

        Eigen::Vector3d spline_sample = (*grasp_spline_)(u);

        ROS_INFO("    [canister frame]: (%0.3f, %0.3f, %0.3f)", spline_sample.x(), spline_sample.y(), spline_sample.z());

        spline_sample = base_footprint_to_gas_canister_ * Eigen::Scaling(gas_can_scale_) * spline_sample;

        geometry_msgs::Point sample_point;
        sample_point.x = spline_sample.x();
        sample_point.y = spline_sample.y();
        sample_point.z = spline_sample.z();

        ROS_INFO("    [base_footprint]: (%0.3f, %0.3f, %0.3f)", sample_point.x, sample_point.y, sample_point.z);
        spline_marker_.points.push_back(sample_point);
    }

    spline_marker_.scale.x = 0.01; // line width

    spline_marker_.color.r = 1.0;
    spline_marker_.color.g = 0.0;
    spline_marker_.color.b = 1.0;
    spline_marker_.color.a = 1.0;
    spline_marker_.lifetime = ros::Duration(0);
    spline_marker_.frame_locked = false;
    return true;
}
