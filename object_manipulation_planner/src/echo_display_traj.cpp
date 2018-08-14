#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>

ros::Publisher robot_state_pub;
std::unique_ptr<robot_trajectory::RobotTrajectory> traj;
moveit::core::RobotModelConstPtr THE_MODEL;
ros::Publisher marker_pub;

void DisplayTrajectoryCallback(const moveit_msgs::DisplayTrajectory& msg)
{
    ROS_INFO("GOT A TRAJECTORY");
    traj.reset(new robot_trajectory::RobotTrajectory(THE_MODEL, "right_arm_torso_base"));
    moveit::core::RobotState state(THE_MODEL);
    moveit::core::robotStateMsgToRobotState(msg.trajectory_start, state);
    traj->setRobotTrajectoryMsg(state, msg.trajectory.front());//.trajectory_start, msg.trajectory);

    for (int i = 0; i < traj->getWayPointCount(); ++i) {
        if (!ros::ok()) break;

        auto state = traj->getWayPoint(i);

        moveit_msgs::RobotState state_msg;
        moveit::core::robotStateToRobotStateMsg(state, state_msg);
        robot_state_pub.publish(state_msg);

        visualization_msgs::MarkerArray ma;
        state.update();
        state.getRobotMarkers(ma, THE_MODEL->getLinkModelNames());
        auto id = 0;
        for (auto& m : ma.markers) {
            m.id = id++;
            if (!m.mesh_use_embedded_materials) {
            m.color.r = 0.5f;
            m.color.g = 0.5f;
            m.color.b = 0.5f;
            m.color.a = 1.0f;
            }
            m.ns = "cabinet_plan";
            m.lifetime = ros::Duration(0);
            m.mesh_use_embedded_materials = true;
//            m.scale.x = m.scale.y = m.scale.z = 1.0;
            m.header.frame_id = "map";
        }
        marker_pub.publish(ma);

        ros::Duration(0.05).sleep();
    }

    ROS_INFO("FINISH TRAJECTORY");
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "echo_display_traj");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("move_group/display_planned_path", 1, DisplayTrajectoryCallback);

    robot_state_pub = nh.advertise<moveit_msgs::RobotState>("plan_state", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);

    robot_model_loader::RobotModelLoader loader;
    THE_MODEL = loader.getModel();

    ros::spin();
}
