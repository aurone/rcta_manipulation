#ifndef GASCAN_GRASP_PLANNER_H
#define GASCAN_GRASP_PLANNER_H

// standard includes
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <ros/ros.h>
#include <spellbook/geometry/nurb/NURB.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

namespace rcta {

struct GraspCandidate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Affine3d pose;
    Eigen::Affine3d pose_in_object;
    double u;

    GraspCandidate(
        const Eigen::Affine3d& pose = Eigen::Affine3d::Identity(),
        const Eigen::Affine3d& pose_in_object = Eigen::Affine3d::Identity(),
        double u = -1.0)
    :
        pose(pose),
        pose_in_object(pose_in_object),
        u(u)
    {
    }
};

class GascanGraspPlanner
{
public:

    GascanGraspPlanner();

    bool init(
        const std::vector<Eigen::Vector3d>& cp,
        int degree,
        double gascan_scale = 1.0);

    bool init(ros::NodeHandle& nh);

    void setWristToToolTransform(const Eigen::Affine3d& T_wrist_tool)
    {
        m_T_wrist_tool = T_wrist_tool;
    }

    void setGraspToPregraspTransform(const Eigen::Affine3d& T_grasp_pregrasp)
    {
        m_T_grasp_pregrasp = T_grasp_pregrasp;
        m_T_pregrasp_grasp = m_T_grasp_pregrasp.inverse();
    }

    bool sampleGrasps(
        const Eigen::Affine3d& T_grasp_object,
        int max_samples,
        std::vector<GraspCandidate>& candidates);

    const Nurb<Eigen::Vector3d>& spline() const { return m_grasp_spline; }
    const Eigen::Affine3d& wristToTool() const { return m_T_wrist_tool; }
    const Eigen::Affine3d& graspToPregrasp() const { return m_T_grasp_pregrasp; }
    const Eigen::Affine3d& pregraspToGrasp() const { return m_T_pregrasp_grasp; }

    bool transformToPreGrasp() const { return m_transform_to_pregrasp; }
    void setTransformToPreGrasp(bool on) { m_transform_to_pregrasp = on; }

private:

    bool m_initialized;
    Nurb<Eigen::Vector3d> m_grasp_spline;
    double m_gascan_scale;

    bool m_transform_to_pregrasp = false;
    Eigen::Affine3d m_T_wrist_tool;
    Eigen::Affine3d m_T_grasp_pregrasp;
    Eigen::Affine3d m_T_pregrasp_grasp;
};

} // namespace rcta

#endif
