#ifndef GRASP_PLANNER_INTERFACE_GRASP_H
#define GRASP_PLANNER_INTERFACE_GRASP_H

#include <Eigen/Dense>

namespace rcta {

struct Grasp
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Affine3d pose;
    double u;
};

} // namespace rcta

#endif

