#include <Eigen/Dense>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <cmu_manipulation_msgs/ManipulateObjectAction.h>
#include <smpl/angles.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manipulate_object");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    using ManipulateObjectActionClient =
            actionlib::SimpleActionClient<cmu_manipulation_msgs::ManipulateObjectAction>;

    ManipulateObjectActionClient client("manipulate_object");

    ros::Duration(1.0).sleep();

    std::map<std::string, double> start_variables;
    (void)ph.getParam("start_state", start_variables);

    std::vector<std::pair<std::string, double>> wtf;

    for (auto& var : start_variables) {
        auto vv = var.first;
        for (auto i = 0; i < vv.size(); ++i) {
            if (vv[i] == '\\') vv[i] = '/';
        }

        auto value = var.second;
        if (vv != "world_joint/x" && vv != "world_joint/y") {
            value = smpl::to_radians(value);
        }

        wtf.emplace_back(vv, value);
    }

    auto object_start_state = 0.0;
    (void)ph.getParam("object_start_position", object_start_state);

    auto object_goal_state = 1.0;
    (void)ph.getParam("object_goal_position", object_goal_state);

    auto allowed_time = 0.0;
    (void)ph.getParam("allowed_planning_time", allowed_time);

    auto execute = false;
    ph.param("execute", execute, false);

    cmu_manipulation_msgs::ManipulateObjectGoal goal;
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

    goal.start_state.multi_dof_joint_state.header.frame_id = "map";
    goal.start_state.multi_dof_joint_state.joint_names = { "world_joint" };

    goal.start_state.multi_dof_joint_state.transforms.resize(1);
    goal.start_state.multi_dof_joint_state.transforms[0].rotation.w = 1.0;

    for (auto& var : wtf) {
        auto pos = var.first.find("world_joint");
        if (pos == std::string::npos) {
            goal.start_state.joint_state.name.push_back(var.first);
            goal.start_state.joint_state.position.push_back(var.second);
        } else {
            auto local = var.first.substr(strlen("world_joint"));
            if (local == "/x") {
                goal.start_state.multi_dof_joint_state.transforms[0].translation.x = var.second;
            } else if (local == "/y") {
                goal.start_state.multi_dof_joint_state.transforms[0].translation.y = var.second;
            } else if (local == "/theta") {
                Eigen::Quaterniond q(Eigen::AngleAxisd(var.second, Eigen::Vector3d::UnitZ()));
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.w = q.w();
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.x = q.x();
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.y = q.y();
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.z = q.z();
            }
        }
    }

    client.waitForServer();

    auto state = client.sendGoalAndWait(goal);
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Success!");
    } else {
        ROS_WARN("Failure!");
    }

    return 0;
}
