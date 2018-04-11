#include "grasp_planner_plugin.h"

namespace rcta {

bool GraspPlannerPlugin::init(ros::NodeHandle& nh, ros::NodeHandle& gh)
{
    double pregrasp_to_grasp_offset_x_m;
    if (!gh.getParam("pregrasp_to_grasp_offset_m", pregrasp_to_grasp_offset_x_m)) {
        ROS_ERROR("Failed to retrieve 'pregrasp_to_grasp_offset_m' from the param server");
        return false;
    }

    Eigen::Affine3d T_grasp_pregrasp(
            Eigen::Translation3d(-pregrasp_to_grasp_offset_x_m, 0.0, 0.0));
    setGraspToPregraspTransform(T_grasp_pregrasp);

    return true;
}

} // namespace rcta
