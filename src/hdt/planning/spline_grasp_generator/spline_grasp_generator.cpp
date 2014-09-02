#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <Eigen/Dense>
#include <GL/freeglut.h>
#include <ros/ros.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/matplotpp/matplotpp.h>

#include "SplineVisualizer.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "spline_grasp_generator");
    return SplineVisualizer().run();
}
