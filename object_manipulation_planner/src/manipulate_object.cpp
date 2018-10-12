#include <Eigen/Dense>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <object_manipulation_planner/ManipulateObjectAction.h>
#include <smpl/angles.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manipulate_object");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    using ManipulateObjectActionClient =
            actionlib::SimpleActionClient<object_manipulation_planner::ManipulateObjectAction>;

    ManipulateObjectActionClient client("manipulate_object");

    ros::Duration(1.0).sleep();

    std::map<std::string, double> start_variables;
    (void)ph.getParam("start_variables", start_variables);

    for (auto& var : start_variables) {
        auto vv = var.first;
        for (auto i = 0; i < vv.size(); ++i) {
            if (vv[i] == '\\') vv[i] = '/';
        }

        auto value = var.second;
        if (vv != "world_joint/x" && vv != "world_joint/y") {
            value = smpl::to_radians(value);
        }
    }

    auto object_start_state = 0.0;
    (void)ph.getParam("object_start_position", object_start_state);

    auto object_goal_state = 1.0;
    (void)ph.getParam("object_goal_position", object_goal_state);

    auto allowed_time = 0.0;
    (void)ph.getParam("allowed_planning_time", allowed_time);

    auto execute = false;
    ph.param("execute", execute, false);

    object_manipulation_planner::ManipulateObjectGoal goal;
    goal.allowed_planning_time = allowed_time;

    goal.object_pose.position.x = 0.85;
    Eigen::Quaterniond oq(Eigen::AngleAxisd(1.570796, Eigen::Vector3d::UnitZ()));
    goal.object_pose.orientation.w = oq.w();
    goal.object_pose.orientation.x = oq.x();
    goal.object_pose.orientation.y = oq.y();
    goal.object_pose.orientation.z = oq.z();

    goal.object_goal = object_goal_state;
    goal.object_start = object_start_state;

    // TODO:
    goal.object_id = "crate";

    goal.plan_only = !execute;

    goal.start_state.is_diff = true;

    goal.start_state.joint_state.name =
    {
        "limb_right_joint1",
        "limb_right_joint2",
        "limb_right_joint3",
        "limb_right_joint4",
        "limb_right_joint5",
        "limb_right_joint6",
        "limb_right_joint7",
        "limb_left_joint1",
        "limb_left_joint2",
        "limb_left_joint3",
        "limb_left_joint4",
        "limb_left_joint5",
        "limb_left_joint6",
        "limb_left_joint7",
    };

    goal.start_state.joint_state.position =
    {
        smpl::to_radians(135.0),
        smpl::to_radians(0.0),
        smpl::to_radians(180.0),
        smpl::to_radians(45.0),
        smpl::to_radians(30.0),
        smpl::to_radians(90.0),
        smpl::to_radians(-135.0),
        smpl::to_radians(-90.0),
        smpl::to_radians(90.0),
        smpl::to_radians(90.0),
        smpl::to_radians(180.0),
        smpl::to_radians(-90.0),
        smpl::to_radians(0.0),
        smpl::to_radians(0.0),
    };

    goal.start_state.multi_dof_joint_state.header.frame_id = "map";
    goal.start_state.multi_dof_joint_state.joint_names = { "world_joint" };

    goal.start_state.multi_dof_joint_state.transforms.resize(1);
    goal.start_state.multi_dof_joint_state.transforms[0].rotation.w = 1.0;

    auto state = client.sendGoalAndWait(goal);
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Success!");
    } else {
        ROS_WARN("Failure!");
    }

    return 0;
}
