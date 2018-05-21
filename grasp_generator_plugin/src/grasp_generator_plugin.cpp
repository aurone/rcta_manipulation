// standard includes
#include <memory>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <grasp_planner_interface/grasp_planner_plugin.h>
#include <grasp_planner_msgs/GraspPlannerAction.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rcta {

class GraspGeneratorPlugin : public GraspPlannerPlugin
{
public:

    using GraspPlannerActionClient =
            actionlib::SimpleActionClient<grasp_planner_msgs::GraspPlannerAction>;

    std::unique_ptr<GraspPlannerActionClient> m_generate_grasp_client;

    Eigen::Affine3d T_wrist_tool;
    Eigen::Affine3d T_tool_wrist;

    bool init(ros::NodeHandle& nh, ros::NodeHandle& gh) override
    {
        if (!GraspPlannerPlugin::init(nh, gh)) {
            return false;
        }

        auto action_name = "jpl_grasp_planner";
        m_generate_grasp_client.reset(new GraspPlannerActionClient(action_name, false));

        geometry_msgs::Pose tool_pose_wrist_frame;
        if (!msg_utils::download_param(
                gh, "wrist_to_tool_transform", tool_pose_wrist_frame))
        {
            ROS_ERROR("Failed to retrieve 'wrist_to_tool_transform' from the param server");
            return false;
        }

        Eigen::Affine3d T_wrist_tool_;
        tf::poseMsgToEigen(tool_pose_wrist_frame, T_wrist_tool_);
//        ROS_INFO("Wrist-to-Tool Transform: %s", to_string(T_wrist_tool_).c_str());

        this->T_wrist_tool = T_wrist_tool_;
        T_tool_wrist = T_wrist_tool.inverse();
        return true;
    }

    bool planGrasps(
        const std::string& object_id,
        const Eigen::Affine3d& object_pose,
        const Eigen::Vector3d& object_bbx,
        const pcl::PointCloud<pcl::PointXYZ>* cloud,
        int max_grasps,
        std::vector<Grasp>& grasps) override
    {
        if (!cloud) {
            ROS_WARN("NEED CLOUD FOR GRASP PLANNING");
            return false;
        }

        grasp_planner_msgs::GraspPlannerGoal req;
        req.task = "";
        req.type = grasp_planner_msgs::GraspPlannerGoal::POWER;

        req.roi_pose.header.frame_id = cloud->header.frame_id;
        tf::poseEigenToMsg(object_pose, req.roi_pose.pose);

        req.roi_dimensions.header.frame_id = "huh";
        tf::pointEigenToMsg(object_bbx, req.roi_dimensions.point);

        pcl::toROSMsg(*cloud, req.point_cloud);
        req.selected_arms = grasp_planner_msgs::GraspPlannerGoal::RIGHT;
        ros::Duration timeout(30.0);
        ROS_INFO("SEND GRASP GOAL AND WAIT FOR TIME");
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

        grasps.reserve(res->result_grasps.poses.size());
        for (size_t i = 0; i < res->result_grasps.poses.size(); ++i) {
            auto& grasp_msg = res->result_grasps.poses[i];
            ROS_INFO("Grasp %zu: { %f, %f, %f, %f, %f, %f, %f }",
                    i,
                    grasp_msg.position.x,
                    grasp_msg.position.y,
                    grasp_msg.position.z,
                    grasp_msg.orientation.w,
                    grasp_msg.orientation.x,
                    grasp_msg.orientation.y,
                    grasp_msg.orientation.z);

            Eigen::Affine3d grasp_pose;
            tf::poseMsgToEigen(res->result_grasps.poses[i], grasp_pose);

            // Compute poses for the wrist from poses for the tool frame. The
            // extra pi/2 rotation around +x is because the wrist-to-tool
            // transform is currently configured incorrectly.
            // T_planning_wrist = T_planning_tool * T_tool_wrist
            grasp_pose =
                    grasp_pose *
                    T_tool_wrist *
                    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());

            // T_map_wrist = T_map_basefootprint * T_basefootprint_wrist

            // Remove grasp poses where the +x axis of the wrist frame is not
            // pointed downwards within some cone.
            Eigen::Quaterniond p(grasp_pose.rotation());

            Eigen::Affine3d T_nominal_grasp(
                    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));

            // TODO: configurate me
            double angle_thresh = 45.0 * M_PI / 180.0;
            Eigen::Quaterniond q(T_nominal_grasp.rotation());

            double angle = 2.0 * acos(std::fabs(p.dot(q)));

            if (angle > angle_thresh) {
                ROS_INFO(" -> Filtered (%f)", angle);
                continue;
            }

            Grasp g;
            g.pose = grasp_pose;
            g.u = 1.0;
            grasps.push_back(g);

#if 0
            {
                Grasp g;
                tf::poseMsgToEigen(res->result_grasps.poses[i], g.pose);
                g.u = 1.0;
                grasps.push_back(g);
            }
#endif
        }

        return true;
    }
};

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GraspGeneratorPlugin, rcta::GraspPlannerPlugin);

