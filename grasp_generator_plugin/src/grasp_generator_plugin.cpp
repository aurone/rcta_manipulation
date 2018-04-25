// standard includes
#include <memory>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <grasp_planner_interface/grasp_planner_plugin.h>
#include <grasp_generator_msgs/GenerateGraspAction.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rcta {

class GraspGeneratorPlugin : public GraspPlannerPlugin
{
public:

    using GenerateGraspActionClient =
            actionlib::SimpleActionClient<grasp_generator_msgs::GenerateGraspAction>;

    std::unique_ptr<GenerateGraspActionClient> m_generate_grasp_client;

    bool init(ros::NodeHandle& nh, ros::NodeHandle& gh) override
    {
        auto action_name = "generate_grasp";
        m_generate_grasp_client.reset(new GenerateGraspActionClient(action_name, false));
        return true;
    }

    bool planGrasps(
        const std::string& object_id,
        const Eigen::Affine3d& T_grasp_object,
        const pcl::PointCloud<pcl::PointXYZ>* cloud,
        int max_grasps,
        std::vector<Grasp>& grasps) override
    {
        if (!cloud) {
            ROS_WARN("NEED CLOUD FOR GRASP PLANNING");
            return false;
        }

        grasp_generator_msgs::GenerateGraspGoal req;
        req.task = "";
        req.type = grasp_generator_msgs::GenerateGraspGoal::POWER;

        pcl::toROSMsg(*cloud, req.goal_pc);
//        req.goal_pc = *cloud;
        req.selected_arms = grasp_generator_msgs::GenerateGraspGoal::RIGHT;
        ros::Duration timeout(10.0);
        auto state = m_generate_grasp_client->sendGoalAndWait(req, timeout);
        if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ERROR("Action returned %s", state.getText().c_str());
            return false;
        }

        auto res = m_generate_grasp_client->getResult();
        if (!res) {
            ROS_ERROR("res is null");
            return false;
        }

        if (!res->error_string.empty()) {
            ROS_ERROR("Grasp generator failed to generate grasps (%s)", res->error_string.c_str());
            return false;
        }


        if (res->goal_poses.size() != 1) {
            ROS_ERROR("Received %zu pose arrays. Why not 1?", res->goal_poses.size());
            return false;
        }

        grasps.resize(res->goal_poses.size());
        for (size_t i = 0; i < res->goal_poses[0].poses.size(); ++i) {
            tf::poseMsgToEigen(res->goal_poses[0].poses[i], grasps[i].pose);
            grasps[i].u = 1.0;
        }

        return true;
    }
};

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GraspGeneratorPlugin, rcta::GraspPlannerPlugin);

