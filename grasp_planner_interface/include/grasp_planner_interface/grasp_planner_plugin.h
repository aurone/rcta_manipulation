#ifndef GRASP_PLANNER_INTERFACE_GRASP_PLANNER_PLUGIN_H
#define GRASP_PLANNER_INTERFACE_GRASP_PLANNER_PLUGIN_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <grasp_planner_interface/grasp.h>

// gascan:
//  inputs:
//    model of the object
//    pose of the object (in some frame)
//  outputs:
//    set of grasps
//      graspability : real
//      pose : pose of the wrist in the frame the object pose is specified in
// other:
//  inputs:
//    robot state (pose + configuration) (if using inverse kinematics)
//    inverse kinematics solver
//    point cloud of interest
//  outputs:
//    * can be filtered by inverse kinematics
//    set of grasps
//      graspability : real
//      pose : pose of the tool frame in the frame of the point cloud
//

namespace rcta {

class GraspPlannerPlugin
{
public:

    virtual ~GraspPlannerPlugin() { }

    /// Initialize the grasp planner plugin. Derived classes should call this
    /// method to initialize the grasp-to-pregrasp offset.
    virtual bool init(ros::NodeHandle& nh, ros::NodeHandle& gh);

    /// Construct a set of potential grasps. Grasp poses should be constructed
    /// in whatever frame the object is specified in, i.e. relative to
    /// $object_pose and should correspond to a target pose for the wrist to be
    /// placed.
    ///
    /// \param object_id The name of the object
    /// \param object_pose The pose of the object in some frame
    /// \param cloud An optional point cloud containing the object
    /// \param max_grasps The maximum number of grasps to generate
    /// \param grasps The generated grasps
    virtual bool planGrasps(
        const std::string& object_id,
        const Eigen::Affine3d& object_pose,
        const pcl::PointCloud<pcl::PointXYZ>* cloud,
        int max_grasps,
        std::vector<Grasp>& grasps) = 0;

    auto graspToPregrasp() const -> const Eigen::Affine3d& { return m_T_grasp_pregrasp; }
    auto pregraspToGrasp() const -> const Eigen::Affine3d& { return m_T_pregrasp_grasp; }

    void setGraspToPregraspTransform(const Eigen::Affine3d& transform)
    {
        m_T_grasp_pregrasp = transform;
        m_T_pregrasp_grasp = transform.inverse();
    }

private:

    Eigen::Affine3d m_T_grasp_pregrasp = Eigen::Affine3d::Identity();
    Eigen::Affine3d m_T_pregrasp_grasp = Eigen::Affine3d::Identity();
};

} // namespace rcta

#endif

