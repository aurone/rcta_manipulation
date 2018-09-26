#include <algorithm>
#include <ros/ros.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

struct ControlledRobot : public hardware_interface::RobotHW
{
    hardware_interface::JointStateInterface     i_joint_state;
    hardware_interface::PositionJointInterface  i_position_command;
    hardware_interface::VelocityJointInterface  i_velocity_command;
    hardware_interface::EffortJointInterface    i_effort_command;

    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    std::vector<double> position_commands;
    std::vector<double> velocity_commands;
    std::vector<double> effort_commands;

    void read(const ros::Time& time, const ros::Duration& dt) override
    {
    }

    void write(const ros::Time& time, const ros::Duration& dt) override
    {
        std::copy(begin(position_commands), end(position_commands), begin(joint_positions));
    }
};

bool InitControlledRobot(
    ControlledRobot* robot,
    const smpl::urdf::RobotModel* model,
    const smpl::urdf::RobotState* state)
{
    auto internal_variable_count =
            smpl::urdf::GetVariableCount(model) -
            GetVariableCount(smpl::urdf::GetRootJoint(model));

    // TODO: non-mimic joints?
    robot->joint_positions.resize(internal_variable_count);
    robot->joint_velocities.resize(internal_variable_count);
    robot->joint_efforts.resize(internal_variable_count);
    robot->position_commands.resize(internal_variable_count);
    robot->velocity_commands.resize(internal_variable_count);
    robot->effort_commands.resize(internal_variable_count);

    // create and register handles to joint states and joint commands
    auto ii = 0;
    for (auto i = 0; i < smpl::urdf::GetVariableCount(model); ++i) {
        auto* var = smpl::urdf::GetVariable(model, i);
        if (var->joint == smpl::urdf::GetRootJoint(model)) continue;

        ROS_INFO("Make joint state interface for joint variable '%s'", var->name.c_str());

        hardware_interface::JointStateHandle h_joint_state(
                var->name,
                &robot->joint_positions[ii],
                &robot->joint_velocities[ii],
                &robot->joint_efforts[ii]);
        robot->i_joint_state.registerHandle(h_joint_state);

        ROS_INFO("Make joint command interface for joint variable '%s'", var->name.c_str());

        hardware_interface::JointHandle h_joint_pos(robot->i_joint_state.getHandle(var->name), &robot->position_commands[ii]);
        hardware_interface::JointHandle h_joint_vel(robot->i_joint_state.getHandle(var->name), &robot->velocity_commands[ii]);
        hardware_interface::JointHandle h_joint_eff(robot->i_joint_state.getHandle(var->name), &robot->effort_commands[ii]);

        robot->i_position_command.registerHandle(h_joint_pos);
        robot->i_velocity_command.registerHandle(h_joint_vel);
        robot->i_effort_command.registerHandle(h_joint_eff);
        ++ii;
    }

    robot->registerInterface(&robot->i_joint_state);
    robot->registerInterface(&robot->i_position_command);
    robot->registerInterface(&robot->i_velocity_command);
    robot->registerInterface(&robot->i_effort_command);

    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    std::string robot_description_key;
    if (!ph.searchParam("robot_description", robot_description_key)) {
        ROS_ERROR("Failed to find 'robot_description' on the param server");
        return -1;
    }

    std::string robot_description;
    if (!ph.getParam(robot_description_key, robot_description)) {
        ROS_ERROR("Failed to retrieve '%s' from the param server", robot_description.c_str());
        return -1;
    }

    auto urdf = urdf::parseURDF(robot_description);
    if (urdf == NULL) {
        ROS_ERROR("Failed to parse URDF");
        return -2;
    }

    smpl::urdf::RobotModel robot_model;
    smpl::urdf::JointSpec world_joint;
    world_joint.name = "world_joint";
    world_joint.type = smpl::urdf::JointType::Floating;
    if (!InitRobotModel(&robot_model, urdf.get(), &world_joint)) {
        ROS_ERROR("Failed to initialize Robot Model");
        return -3;
    }

    // Create a robot state for determining initial joint variable positions
    smpl::urdf::RobotState robot_state;
    auto with_velocities = true;
    auto with_accelerations = true;
    if (!Init(&robot_state, &robot_model, with_velocities, with_accelerations)) {
        ROS_ERROR("Failed to initialize Robot State");
        return -3;
    }

    ControlledRobot robot;
    if (!InitControlledRobot(&robot, &robot_model, &robot_state)) {
        ROS_ERROR("Failed to initialize controlled robot");
        return -4;
    }

    controller_manager::ControllerManager manager(&robot);

    ROS_INFO("ready");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(500.0);

    auto prev_time = ros::Time::now();
    while (ros::ok()) {
        auto now = ros::Time::now();
        auto dt = now - prev_time;
        prev_time = now;

        robot.read(now, dt);
        manager.update(now, dt);
        robot.write(now, dt);

        loop_rate.sleep();
    }

    return 0;
}

