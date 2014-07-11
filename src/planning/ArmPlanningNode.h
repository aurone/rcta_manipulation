#ifndef hdt_ArmPlanningNode_h
#define hdt_ArmPlanningNode_h

#include <iomanip>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <ros/ros.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_geometry_utils/shortcut.h>
#include <sbpl_manipulation_components/collision_checker.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <hdt/MoveArmCommandAction.h>

namespace hdt
{

template <typename T>
std::string to_string(const std::vector<T>& vec)
{
    std::stringstream ss;
    ss << "[ ";
    for (int i = 0; i < (int)vec.size(); ++i) {
        ss << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << ']';
    return ss.str();
}

template <> inline std::string to_string<double>(const std::vector<double>& vec)
{
    std::stringstream ss;
    ss << "[ ";
    for (int i = 0; i < (int)vec.size(); ++i) {
        // output doubles in fixed-point notation with 3 digits after the
        // decimal point; allow space for the decimal point, leading 0, and
        // possible - sign
        ss << std::fixed << std::setprecision(3) << std::setw(6) << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << " ]";
    return ss.str();
}

class JointInterpolationPathGenerator : public sbpl::shortcut::PathGenerator<trajectory_msgs::JointTrajectoryPoint, int>
{
public:

    JointInterpolationPathGenerator();

    bool initialize(const std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace>& collision_checker,
                    const std::vector<double>& min_limits, const std::vector<double>& max_limits,
                    const std::vector<bool>& continuous);

    bool generate_path(const trajectory_msgs::JointTrajectoryPoint& start,
                       const trajectory_msgs::JointTrajectoryPoint& end,
                       std::vector<trajectory_msgs::JointTrajectoryPoint>& path_out,
                       int& cost_out) const;

private:

    std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace> collision_checker_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<bool> continuous_;
};

/// @brief Class that provides a simple ROS API to the SBPL arm planner for the HDT arm
class ArmPlanningNode
{
public:

    ArmPlanningNode();

    bool init();

    int run();

private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher marker_array_pub_;
    ros::Publisher joint_trajectory_pub_;
    ros::Subscriber joint_states_sub_;

    typedef actionlib::SimpleActionServer<hdt::MoveArmCommandAction> MoveArmActionServer;
    std::unique_ptr<MoveArmActionServer> move_command_server_;

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JTAC;
    JTAC action_client_;

    bool use_action_server_;

    std::string action_set_filename_;
    std::string group_name_;
    std::string kinematics_frame_;
    std::string planning_frame_;
    std::string planning_link_;
    std::string chain_tip_link_;
    std::string object_filename_;
    std::string urdf_;
    std::vector<std::string> planning_joints_;

    std::unique_ptr<sbpl_arm_planner::RobotModel> robot_model_;
    std::unique_ptr<distance_field::PropagationDistanceField> distance_field_;
    std::unique_ptr<sbpl_arm_planner::OccupancyGrid> grid_;
    std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace> collision_checker_;
    std::unique_ptr<sbpl_arm_planner::ActionSet> sbpl_action_set_;
    std::unique_ptr<sbpl_arm_planner::SBPLArmPlannerInterface> planner_;

    sensor_msgs::JointState last_joint_state_;

    std::vector<std::string> manipulator_joint_names_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<bool> continuous_;

    boost::shared_ptr<urdf::ModelInterface> urdf_model_;

    std::vector<std::string> statistic_names_;

    bool init_robot();
    bool init_collision_model();
    bool init_sbpl();

    void move_arm(const hdt::MoveArmCommandGoal::ConstPtr& goal);

    void fill_constraint(const std::vector<double>& pose,
                         const std::string& frame_id,
                         moveit_msgs::Constraints& goals);

    moveit_msgs::CollisionObject
    get_collision_cube(const geometry_msgs::Pose& pose,
                       const std::vector<double>& dims,
                       const std::string& frame_id,
                       const std::string& id);

    std::vector<moveit_msgs::CollisionObject>
    get_collision_cubes(const std::vector<std::vector<double>>& objects,
                        const std::vector<std::string>& object_ids,
                        const std::string& frame_id);

    std::vector<moveit_msgs::CollisionObject>
    get_collision_objects(const std::string& filename, const std::string& frame_id);

    bool get_initial_configuration(ros::NodeHandle& nh, moveit_msgs::RobotState& state);

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool add_interpolation_to_plan(moveit_msgs::GetMotionPlan::Response& res) const;

    void publish_trajectory(const moveit_msgs::RobotTrajectory& trajectory);

    void apply_shortcutting(moveit_msgs::GetMotionPlan::Response& res) const;
};

} // namespace hdt

#endif
