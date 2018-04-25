#include <grasp_planner_interface/grasp_planner_plugin.h>

#include <gascan_grasp_planning/gascan_grasp_planner.h>

namespace rcta {

class GascanGraspPlannerPlugin : public GraspPlannerPlugin
{
public:

    GascanGraspPlanner grasp_planner;

    bool init(ros::NodeHandle& nh, ros::NodeHandle& gh) override
    {
        if (!GraspPlannerPlugin::init(nh, gh)) {
            return false;
        }
        return grasp_planner.init(gh);
    }

    bool planGrasps(
        const std::string& object_id,
        const Eigen::Affine3d& T_grasp_object,
        const pcl::PointCloud<pcl::PointXYZ>* cloud,
        int max_grasps,
        std::vector<Grasp>& grasps) override
    {
        if (object_id != "gascan") {
            ROS_ERROR("This planner is for the gascan");
            return false;
        }

        std::vector<GraspCandidate> tmp;
        if (!grasp_planner.sampleGrasps(T_grasp_object, max_grasps, tmp)) {
            return false;
        }

        grasps.reserve(tmp.size());
        for (auto& grasp : tmp) {
            grasps.push_back(Grasp{ grasp.pose, grasp.u });
        }
        return true;
    }
};

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GascanGraspPlannerPlugin, rcta::GraspPlannerPlugin);

