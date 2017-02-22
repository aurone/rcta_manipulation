// standard includes
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <GL/freeglut.h>
#include <ros/ros.h>
#include <spellbook/stringifier/stringifier.h>

// project includes
#include <rcta/common/matplotpp/matplotpp.h>

// module includes
#include "SplineVisualizer.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "spline_grasp_generator");
    return SplineVisualizer().run();
}
