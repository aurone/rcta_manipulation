#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/debug/marker_utils.h>
#include <smpl_urdf_robot_model/robot_state_visualization.h>
#include <moveit_msgs/RobotState.h>
#include <urdf/model.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "draw_kinematics");
    ros::NodeHandle nh;

    smpl::VisualizerROS visualizer;
    smpl::visual::set_visualizer(&visualizer);

    auto filepath = argv[1];
    auto model = urdf::Model();
    if (!model.initFile(filepath)) {
        return 1;
    }

    auto robot_model = smpl::urdf::RobotModel();
    if (!InitRobotModel(&robot_model, &model)) {
        return 2;
    }

    auto robot_state = smpl::urdf::RobotState{ };
    if (!InitRobotState(&robot_state, &robot_model, false, false)) {
        return 3;
    }

    using RobotStateCallback = boost::function<void(const moveit_msgs::RobotState::ConstPtr& msg)>;
    RobotStateCallback cb = [&](const moveit_msgs::RobotState::ConstPtr& msg)
    {
        // update the robot state
        // make line visualization
        for (auto i = 0; i < msg->joint_state.name.size(); ++i) {
            auto& name = msg->joint_state.name[i];
            auto* var = GetVariable(&robot_model, name.c_str());
            if (var == NULL) continue;

            auto position = msg->joint_state.position[i];
            SetVariablePosition(&robot_state, var, position);
        }

        UpdateTransforms(&robot_state);

        auto* link3 = GetLink(&robot_model, "limb_right_link1");
        auto* link4 = GetLink(&robot_model, "limb_right_link3");
        auto* link5 = GetLink(&robot_model, "limb_right_link5");

        auto l3_pos = smpl::Vector3(GetLinkTransform(&robot_state, link3)->translation());
        auto l4_pos = smpl::Vector3(GetLinkTransform(&robot_state, link4)->translation());
        auto l5_pos = smpl::Vector3(GetLinkTransform(&robot_state, link5)->translation());

        auto* j1 = GetVariable(&robot_model, "limb_right_joint1");
        auto* j2 = GetVariable(&robot_model, "limb_right_joint2");
        auto* j3 = GetVariable(&robot_model, "limb_right_joint3");
        auto* j4 = GetVariable(&robot_model, "limb_right_joint4");
        auto* j5 = GetVariable(&robot_model, "limb_right_joint5");
        auto* j6 = GetVariable(&robot_model, "limb_right_joint6");
        auto* j7 = GetVariable(&robot_model, "limb_right_joint7");

        auto pj1 = GetVariablePosition(&robot_state, j1);
        auto pj2 = GetVariablePosition(&robot_state, j2);
        auto pj3 = GetVariablePosition(&robot_state, j3);
        auto pj4 = GetVariablePosition(&robot_state, j4);
        auto pj5 = GetVariablePosition(&robot_state, j5);
        auto pj6 = GetVariablePosition(&robot_state, j6);
        auto pj7 = GetVariablePosition(&robot_state, j7);

        auto L_shoulder_wrist = (l3_pos - l5_pos).norm();
        auto L_shoulder_elbow = (l4_pos - l3_pos).norm();
        auto L_elbow_wrist = (l5_pos - l4_pos).norm();

        auto sqrd = [](double d) { return d * d; };

        auto t4 = acos((sqrd(L_shoulder_elbow) + sqrd(L_elbow_wrist) - sqrd(L_shoulder_wrist)) / (2 * L_shoulder_elbow * L_elbow_wrist));

        {
            auto b2 = sqrd(2.0 * 0.117);
            auto C2 = sqrd(L_shoulder_wrist);
            auto a2 = C2 - b2;
            auto t4 = acos((sqrd(L_shoulder_elbow) + sqrd(L_elbow_wrist) - a2) / (2 * L_shoulder_elbow * L_elbow_wrist));
            ROS_INFO("L_s_w = %f, t4 = %f", L_shoulder_wrist, t4);
        }

        SV_SHOW_INFO(smpl::visual::MakeLineMarker(l3_pos, l5_pos, "map", "axis"));
    };

    auto sub = nh.subscribe("command_robot_state", 1, cb);

    ros::spin();
    return 0;
}
