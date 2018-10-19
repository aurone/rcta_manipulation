#ifndef RCTA_MANIPULATION_COMMON_MOVE_GROUP_GOAL_H
#define RCTA_MANIPULATION_COMMON_MOVE_GROUP_GOAL_H

// standard includes
#include <stdint.h>

// system includes
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningOptions.h>
#include <moveit_msgs/RobotState.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/JointState.h>

namespace rcta {

struct MoveArmGoal
{
    static const uint8_t EndEffectorGoal = 0;
    static const uint8_t JointGoal = 1;
    static const uint8_t CartesianGoal = 2;

    uint8_t type;
    geometry_msgs::Pose goal_pose;
    sensor_msgs::JointState goal_joint_state;

    moveit_msgs::RobotState start_state;

    octomap_msgs::Octomap octomap;

    bool execute_path;

    moveit_msgs::PlanningOptions planning_options;
};

} // namespace rcta

#endif
