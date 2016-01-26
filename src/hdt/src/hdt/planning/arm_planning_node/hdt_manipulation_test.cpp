#include <memory>
#include <moveit/distance_field/propagation_distance_field.h>
#include <ros/ros.h>
#include <sbpl_manipulation_components/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_arm_planner/action_set.h>
#include <sbpl_arm_planner/sbpl_arm_planner_interface.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include "HDTRobotModel.h"

std::vector<std::string> ReadPlanningJointsParam()
{
    ros::NodeHandle ph("~");

    XmlRpc::XmlRpcValue xlist;
    ph.getParam("planning/planning_joints", xlist);

    std::string joint_list = std::string(xlist);
    std::stringstream joint_name_stream(joint_list);
    std::vector<std::string> planning_joints;
    while (joint_name_stream.good() && !joint_name_stream.eof())
    {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.size() == 0) {
            continue;
        }
        planning_joints.push_back(jname);
    }

    return planning_joints;
}

enum MainResult
{
    SUCCESS = 0,
    FAILED_TO_FIND_PACKAGE,
    INSUFFICIENT_START_ANGLES,
    MISSING_URDF,
    FAILED_TO_INITIALIZE_ROBOT_MODEL,
    FAILED_TO_INITIALIZE_SBPL,
    FAILED_TO_INITIALIZE_COLLISION_CHECKER
};

int main(int argc, char** argv)
{
    using distance_field::PropagationDistanceField;

    ros::init(argc, argv, "hdt_manipulation_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    const std::string kinematics_frame = "base_link";
    const std::string planning_frame = "base_link";
    // planning for the wrist rather than the gripper base as in the sbpl_arm_planner_test - Andrew
    const std::string planning_link = "arm_7_gripper_lift_link";
    const std::string chain_tip_link = "arm_7_gripper_lift_link";
    const std::string group_name = "hdt_arm";

    // Andrew - There was a hardcoded path to the action set here...apparently
    // because rospack was refusing to compile with distance_field. Sounds bogus
    // to me.
    std::string action_set_filename;
    if (!ph.getParam("action_set_filename", action_set_filename)) {
        ROS_ERROR("Failed to retrieve 'action_set_filename' from the param server");
        return 1;
    }

    std::vector<double> goal = { 1.1, 0.0, 0.6, 0.0, 0.0,  0.0 };

    ///////////////////////////
    // Set up the robot model
    ///////////////////////////

    std::vector<std::string> planning_joints = {
        "arm_1_shoulder_twist",
        "arm_2_shoulder_lift",
        "arm_3_elbow_twist",
        "arm_4_elbow_lift",
        "arm_5_wrist_twist",
        "arm_6_wrist_lift",
        "arm_7_gripper_lift"
    };

//    std::vector<std::string> planning_joints = ReadPlanningJointsParam();
    if (planning_joints.empty()) {
        ROS_WARN("Planning joint names empty");
    }

    std::vector<double> start_angles(planning_joints.size(), 0);
    if (start_angles.size() < 7) {
        ROS_ERROR("Insufficent joint position variables. expected: 7, actual: %zu", start_angles.size());
        return INSUFFICIENT_START_ANGLES;
    }

    std::string urdf;
    if (!nh.getParam("robot_description", urdf)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return MISSING_URDF;
    }

    // NOTE: It looks looks a vanilla KDLRobotModel can be instantiated here instead of needing to subclass for the HDT - Andrew
//    std::unique_ptr<sbpl_arm_planner::RobotModel> robot_model(new HdtKdlRobotModel);
    std::unique_ptr<sbpl_arm_planner::KDLRobotModel> robot_model(
            new sbpl_arm_planner::KDLRobotModel(kinematics_frame, chain_tip_link));
    if (!robot_model) {
        ROS_ERROR("Failed to instantiate KDL Robot Model");
        return FAILED_TO_INITIALIZE_ROBOT_MODEL;
    }

    if (!robot_model->init(urdf, planning_joints)) {
        ROS_ERROR("Failed to initialize RobotModel");
        return FAILED_TO_INITIALIZE_ROBOT_MODEL;
    }

    robot_model->setPlanningLink(planning_link);

    KDL::Frame f;

    ///////////////////////////////////////////////////
    // Set up the environment and its collision model
    ///////////////////////////////////////////////////

    double minx = 0.0, miny = 0.0, minz = 0.0;
    double maxx = 3.0, maxy = 3.0, maxz = 3.0;
    double cellres_m = 0.02;
    double center_x = -0.75, center_y = -1.25, center_z = -1.0;
//    double max_distance_m = 1.0;
    double max_distance_m = 0.2;
    bool propagate_negative_distances = false;

    std::unique_ptr<PropagationDistanceField> distance_field;
    distance_field.reset(new PropagationDistanceField(
            maxx - minx, maxy - miny, maxz - minz,
            cellres_m,
            center_x, center_y, center_z,
            max_distance_m,
            propagate_negative_distances));

    std::unique_ptr<sbpl_arm_planner::OccupancyGrid> grid(
            new sbpl_arm_planner::OccupancyGrid(distance_field.get()));
    grid->setReferenceFrame(planning_frame);

    std::unique_ptr<sbpl::collision::SBPLCollisionSpace> cc(
            new sbpl::collision::SBPLCollisionSpace(grid.get()));
    if (!cc) {
        ROS_ERROR("Failed to instantiate collision checker");
        return FAILED_TO_INITIALIZE_COLLISION_CHECKER;
    }

    sbpl::collision::CollisionModelConfig cm_config;
    if (!sbpl::collision::CollisionModelConfig::Load(ros::NodeHandle(), cm_config)) {
        ROS_ERROR("Failed to load collision model config");
        return FAILED_TO_INITIALIZE_COLLISION_CHECKER;
    }

    if (!cc->init(urdf, group_name, cm_config, planning_joints)) {
        ROS_ERROR("Failed to initialize collision checking for HDT arm group %s", group_name.c_str());
        return FAILED_TO_INITIALIZE_COLLISION_CHECKER;
    }

    ///////////////////////////
    // Set up the arm planner
    ///////////////////////////

    sbpl_arm_planner::ActionSetPtr action_set(new sbpl_arm_planner::ActionSet);
    if (!sbpl_arm_planner::ActionSet::Load(action_set_filename, *action_set)) {
        ROS_ERROR("Failed to load Action Set");
        return FAILED_TO_INITIALIZE_SBPL;
    }

    std::unique_ptr<sbpl_arm_planner::SBPLArmPlannerInterface> planner;
    planner.reset(new sbpl_arm_planner::SBPLArmPlannerInterface(
            robot_model.get(), cc.get(), action_set.get(), distance_field.get()));
    if (!planner) {
        ROS_ERROR("Failed to instantiate SBPL Arm Planner Interface");
        return FAILED_TO_INITIALIZE_SBPL;
    }

    if (!planner->init()) {
        ROS_ERROR("Failed to initialize SBPL Arm Planner Interface");
        return FAILED_TO_INITIALIZE_SBPL;
    }

    return SUCCESS;
}
