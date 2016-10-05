// standard includes
#include <memory>

// system includes
#include <moveit/distance_field/propagation_distance_field.h>
#include <ros/ros.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <sbpl_arm_planner/planner_interface.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>

enum MainResult
{
    SUCCESS = 0,
    FAILED_TO_FIND_PACKAGE,
    INSUFFICIENT_START_ANGLES,
    MISSING_URDF,
    FAILED_TO_INITIALIZE_ROBOT_MODEL,
    FAILED_TO_INITIALIZE_SBPL,
    FAILED_TO_INITIALIZE_COLLISION_CHECKER,
    FAILED_TO_LOAD_ACTION_SET
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hdt_manipulation_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Duration(1.0).sleep();

    const std::string planning_frame = "base_link";

    /////////////////
    // Robot Model //
    /////////////////

    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return MISSING_URDF;
    }

    const std::vector<std::string> planning_joints = {
        "arm_1_shoulder_twist",
        "arm_2_shoulder_lift",
        "arm_3_elbow_twist",
        "arm_4_elbow_lift",
        "arm_5_wrist_twist",
        "arm_6_wrist_lift",
        "arm_7_gripper_lift"
    };

    const std::string planning_link = "arm_7_gripper_lift_link";

    const std::string kinematics_frame = "base_link";
    // planning for the wrist rather than the gripper base as in the
    // sbpl_arm_planner_test - Andrew
    const std::string chain_tip_link = planning_link;

    const int free_angle_index = 2;
    sbpl::manip::KDLRobotModel robot_model(
        kinematics_frame,
        chain_tip_link,
        free_angle_index);

    if (!robot_model.init(robot_description, planning_joints)) {
        ROS_ERROR("Failed to initialize KDL Robot Model");
        return false;
    }

    robot_model.setPlanningLink(planning_link);

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    const double minx = 0.0, miny = 0.0, minz = 0.0;
    const double maxx = 3.0, maxy = 3.0, maxz = 3.0;
    const double cellres_m = 0.02;
    const double origin_x = -0.75, origin_y = -1.25, origin_z = -1.0;
    const double max_distance_m = 0.2;
    const bool propagate_negative_distances = false;

    sbpl::OccupancyGrid grid(
            maxx - minx, maxy - miny, maxz - minz,
            cellres_m,
            origin_x, origin_y, origin_z,
            max_distance_m,
            propagate_negative_distances);
    grid.setReferenceFrame(planning_frame);

    ///////////////////////
    // Collision Checker //
    ///////////////////////

    sbpl::collision::CollisionModelConfig cm_cfg;
    if (!sbpl::collision::CollisionModelConfig::Load(ph, cm_cfg)) {
        ROS_ERROR("Failed to load collision model config");
        return FAILED_TO_INITIALIZE_COLLISION_CHECKER;
    }

    // group of collision links to collision check as specified via collision
    // model config
    const std::string group_name = "hdt_arm";

    sbpl::collision::CollisionSpaceBuilder builder;
    auto cspace = builder.build(&grid, robot_description, cm_cfg, group_name, planning_joints);
    if (!cspace) {
        ROS_ERROR("Failed to initialize collision checking for HDT arm group %s", group_name.c_str());
        return FAILED_TO_INITIALIZE_COLLISION_CHECKER;
    }

    // set the initial state of the robot (not strictly needed here since all
    // the robot variables are included as planning joints and will be updated
    // with calls to isStateValid)
    std::vector<double> start_angles(planning_joints.size(), 0);
    if (start_angles.size() < 7) {
        ROS_ERROR("Insufficent joint position variables. expected: 7, actual: %zu", start_angles.size());
        return INSUFFICIENT_START_ANGLES;
    }

    for (size_t jidx = 0; jidx < planning_joints.size(); ++jidx) {
        const std::string& joint_name = planning_joints[jidx];
        double joint_position = start_angles[jidx];
        if (!cspace->setJointPosition(joint_name, joint_position)) {
            ROS_ERROR("Failed to set the initial state for joint '%s'", joint_name.c_str());
            return FAILED_TO_INITIALIZE_COLLISION_CHECKER;
        }
    }

    std::string action_set_filename;
    if (!ph.getParam("action_set_filename", action_set_filename)) {
        ROS_ERROR("Failed to retrieve 'action_set_filename' from the param server");
        return 1;
    }

    ////////////////////////
    // Planning Interface //
    ////////////////////////

    sbpl::manip::PlannerInterface planner(&robot_model, cspace.get(), &grid);

    sbpl::manip::PlanningParams params;
    params.action_filename = action_set_filename;
    // TODO: fill in planning params
    if (!planner.init(params)) {
        ROS_ERROR("Failed to initialize SBPL Arm Planner Interface");
        return FAILED_TO_INITIALIZE_SBPL;
    }

    return SUCCESS;
}
