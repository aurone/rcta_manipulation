#ifndef GRASP_PLANNER_INTERFACE_GRASP_UTILS_H
#define GRASP_PLANNER_INTERFACE_GRASP_UTILS_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include <grasp_planner_interface/grasp.h>

namespace rcta {

auto GetGraspCandidatesVisualization(
    const std::vector<Grasp>& grasps,
    const std::string& frame_id,
    const std::string& ns)
    -> visualization_msgs::MarkerArray;


void PruneGraspsByVisibility(
    std::vector<Grasp>& grasps,
    const EigenSTL::vector_Affine3d& marker_poses,
    const Eigen::Affine3d& camera_pose,
    double ang_thresh);

} // namespace rcta

#endif

