#ifndef hdt_ArmPlanningNode_h
#define hdt_ArmPlanningNode_h

// standard includes
#include <iomanip>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

// system includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <ros/ros.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/arm_planner_interface.h>
#include <sbpl_arm_planner/collision_checker.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <spellbook/stringifier/stringifier.h>
#include <tf/transform_listener.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

// project includes
#include <rcta/MoveArmCommandAction.h>
#include <rcta/common/hdt_description/RobotModel.h>

// module includes
#include "HDTRobotModel.h"

namespace hdt {

/// @brief Implements a ROS node to provide planning and execution of paths
///
/// The ROS node maintains the state of the robot and the world to plan and
/// and execute paths that avoid obstacles in the environment and self
/// collisions with the robot itself.
class ArmPlanningNode
{
public:

    ArmPlanningNode();

    bool init();

    int run();

private:

    ros::NodeHandle m_nh;
    ros::NodeHandle m_ph;

    ros::Publisher m_marker_array_pub;
    ros::Publisher m_joint_trajectory_pub;
    ros::Subscriber m_joint_states_sub;

    typedef actionlib::SimpleActionServer<rcta::MoveArmCommandAction> MoveArmActionServer;
    std::unique_ptr<MoveArmActionServer> m_move_arm_command_server;

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JTAC;
    JTAC m_follow_trajectory_client;

    bool m_use_action_server;

    std::string m_action_set_filename;
    std::string m_kinematics_frame;
    std::string m_planning_frame;
    std::string m_object_filename;
    std::string m_urdf_string;

    std::string robot_name_;
    std::string robot_local_frame_;
    hdt::RobotModelPtr robot_model_;

    HDTRobotModel m_planner_robot_model;
    sbpl::OccupancyGridPtr grid_;
    sbpl::collision::CollisionSpacePtr m_collision_checker;
    sbpl::manip::ActionSet m_action_set;
    std::unique_ptr<sbpl::manip::ArmPlannerInterface> m_planner;

    sensor_msgs::JointState last_joint_state_;
    std::vector<ros::Time> received_joint_state_;
    tf::TransformListener listener_;

    ros::Duration joint_staleness_threshold_;

    boost::shared_ptr<urdf::ModelInterface> urdf_model_;

    std::vector<std::string> statistic_names_;

    moveit_msgs::PlanningScenePtr planning_scene_;

    bool apply_shortcutting_;

    bool init_robot();
    bool init_collision_model();
    bool init_sbpl();

    bool reinit(
            const Eigen::Affine3d& T_kinematics_planning,
            const std::string& planning_frame,
            const octomap_msgs::Octomap& octomap);

    bool reinit_robot(const Eigen::Affine3d& T_kinematics_planning);
    bool reinit_collision_model(
        const std::string& planning_frame,
        const octomap_msgs::Octomap& octomap);
    bool reinit_sbpl();

    void move_arm(const rcta::MoveArmCommandGoal::ConstPtr& goal);

    void fill_constraint(
            const geometry_msgs::Pose& pose,
            const std::string& frame_id,
            moveit_msgs::Constraints& goals);

    moveit_msgs::CollisionObject
    get_collision_cube(
            const geometry_msgs::Pose& pose,
            const std::vector<double>& dims,
            const std::string& frame_id,
            const std::string& id);

    std::vector<moveit_msgs::CollisionObject>
    get_collision_cubes(
            const std::vector<std::vector<double>>& objects,
            const std::vector<std::string>& object_ids,
            const std::string& frame_id);

    std::vector<moveit_msgs::CollisionObject>
    get_collision_objects(const std::string& filename, const std::string& frame_id);

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool add_interpolation_to_plan(trajectory_msgs::JointTrajectory& res_traj) const;
    void publish_trajectory(const trajectory_msgs::JointTrajectory& joint_trajectory);

    std::vector<double> convert_to_sbpl_goal(const geometry_msgs::Pose& pose);

    bool plan_to_eef_goal(
            const moveit_msgs::PlanningScenePtr& scene,
            const geometry_msgs::PoseStamped& goal_pose,
            trajectory_msgs::JointTrajectory& traj);

    bool plan_to_joint_goal(
            const moveit_msgs::PlanningScenePtr& scene,
            const moveit_msgs::RobotState& start,
            const rcta::MoveArmCommandGoal& goal,
            trajectory_msgs::JointTrajectory& traj);

    void clamp_to_joint_limits(
            std::vector<double>& joint_vector,
            const std::vector<double>& min_limits,
            const std::vector<double>& max_limits);
};

} // namespace hdt

#endif
