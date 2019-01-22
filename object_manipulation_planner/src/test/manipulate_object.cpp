#include <Eigen/Dense>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <cmu_manipulation_msgs/ManipulateObjectAction.h>
#include <smpl/angles.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "manipulate_object");
    auto nh = ros::NodeHandle();
    auto ph = ros::NodeHandle("~");

    //////////////////////////////
    // Parse Test Configuration //
    //////////////////////////////

    auto start_variables = std::map<std::string, double>();
    (void)ph.getParam("start_state", start_variables);

    auto object_start_state = 0.0;
    (void)ph.getParam("object_start_position", object_start_state);

    auto object_goal_state = 1.0;
    (void)ph.getParam("object_goal_position", object_goal_state);

    auto allowed_time = 10.0;
    (void)ph.getParam("allowed_planning_time", allowed_time);

    auto execute = false;
    ph.param("execute", execute, false);

    auto object_pose = std::vector<double>();
    (void)ph.getParam("object_pose", object_pose);

    /////////////////////////////////
    // Build ManipulateObject Goal //
    /////////////////////////////////

    auto goal = cmu_manipulation_msgs::ManipulateObjectGoal();

    goal.start_state.is_diff = true;

    goal.start_state.multi_dof_joint_state.header.frame_id = "map";
    goal.start_state.multi_dof_joint_state.joint_names = { "world_joint" };

    goal.start_state.multi_dof_joint_state.transforms.resize(1);
    goal.start_state.multi_dof_joint_state.transforms[0].rotation.w = 1.0;

    auto wtf = std::vector<std::pair<std::string, double>>();

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
                auto q = Eigen::Quaterniond(Eigen::AngleAxisd(var.second, Eigen::Vector3d::UnitZ()));
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.w = q.w();
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.x = q.x();
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.y = q.y();
                goal.start_state.multi_dof_joint_state.transforms[0].rotation.z = q.z();
            }
        }
    }

    goal.object_id = "crate"; // TODO:

    if (object_pose.size() > 0) {
        goal.object_pose.position.x = object_pose[0];
    }
    if (object_pose.size() > 1) {
        goal.object_pose.position.y = object_pose[1];
    }
    if (object_pose.size() > 2) {
        goal.object_pose.position.z = object_pose[2];
    }
    auto goal_qz = 0.0;
    auto goal_qy = 0.0;
    auto goal_qx = 0.0;
    if (object_pose.size() > 3) {
        goal_qz = smpl::to_radians(object_pose[3]);
    }
    if (object_pose.size() > 4) {
        goal_qy = smpl::to_radians(object_pose[4]);
    }
    if (object_pose.size() > 5) {
        goal_qx = smpl::to_radians(object_pose[5]);
    }

    auto oq = Eigen::Quaterniond(
        Eigen::AngleAxisd(goal_qz, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(goal_qy, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(goal_qx, Eigen::Vector3d::UnitX()));
    goal.object_pose.orientation.w = oq.w();
    goal.object_pose.orientation.x = oq.x();
    goal.object_pose.orientation.y = oq.y();
    goal.object_pose.orientation.z = oq.z();

    goal.object_start = object_start_state;
    goal.object_goal = object_goal_state;

    goal.allowed_planning_time = allowed_time;

    goal.plan_only = !execute;

    ///////////////
    // Send Goal //
    ///////////////

    using cmu_manipulation_msgs::ManipulateObjectAction;
    using ManipulateObjectActionClient = actionlib::SimpleActionClient<ManipulateObjectAction>;
    ManipulateObjectActionClient client("manipulate_object");
    client.waitForServer();

    auto state = client.sendGoalAndWait(goal);
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Success!");
        return 0;
    } else {
        ROS_WARN("Failure!");
        return 1;
    }

    return 0;
}
