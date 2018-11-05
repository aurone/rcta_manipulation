#include <algorithm>
#include <ros/ros.h>
#define private public
#include <controller_manager/controller_manager.h>
#undef private
#define private private
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <urdf_parser/urdf_parser.h>

struct ControlledRobot : public hardware_interface::RobotHW
{
    hardware_interface::JointStateInterface     i_joint_state;
    hardware_interface::PositionJointInterface  i_position_command;
    hardware_interface::VelocityJointInterface  i_velocity_command;
    hardware_interface::EffortJointInterface    i_effort_command;

    int variable_count = 0;

    enum Mode {
        Position,
        Velocity,
        Effort
    };

    std::vector<int> control_modes;

    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;

    std::vector<double> prev_position_commands;
    std::vector<double> prev_velocity_commands;
    std::vector<double> prev_effort_commands;

    std::vector<double> position_commands;
    std::vector<double> velocity_commands;
    std::vector<double> effort_commands;

#if ROS_KINETIC
    void read(const ros::Time& time, const ros::Duration& dt) override;
#else
    void read();
#endif

#if ROS_KINETIC
    void write(const ros::Time& time, const ros::Duration& dt) override;
#else
    void write();
#endif
};

bool InitControlledRobot(
    ControlledRobot* robot,
    const smpl::urdf::RobotModel* model,
    const smpl::urdf::RobotState* state)
{
    // number of variables, not including those for the root joint
    auto internal_variable_count =
            GetVariableCount(model) - GetVariableCount(GetRootJoint(model));

    ROS_INFO("Root joint has %zu variables", GetVariableCount(GetRootJoint(model)));
    ROS_INFO("Robot Model contains %zu variables", GetVariableCount(model));
    ROS_INFO("Robot Model contains %zu internal variables", internal_variable_count);

    // TODO: non-mimic joints?
    robot->variable_count = internal_variable_count;

    robot->control_modes.resize(internal_variable_count, ControlledRobot::Position);

    // HACK! known that root joint variables come first
    auto* positions_begin = GetVariablePositions(state) + GetVariableCount(GetRootJoint(model));
    auto* positions_end = GetVariablePositions(state) + GetVariableCount(model);
    printf("set from %td variables [%p, %p]\n", positions_end - positions_begin, positions_begin, positions_end);
    robot->joint_positions.assign(positions_begin, positions_end);

    robot->joint_velocities.resize(internal_variable_count, 0.0);
    robot->joint_efforts.resize(internal_variable_count, 0.0);

    robot->position_commands = robot->joint_positions;
    robot->velocity_commands.resize(internal_variable_count, 0.0);
    robot->effort_commands.resize(internal_variable_count, 0.0);

    robot->prev_position_commands = robot->joint_positions;
    robot->prev_velocity_commands.resize(internal_variable_count, 0.0);
    robot->prev_effort_commands.resize(internal_variable_count, 0.0);

    // create and register handles to joint states and joint commands
    auto ii = 0;
    for (auto i = 0; i < GetVariableCount(model); ++i) {
        auto* var = GetVariable(model, i);

        // skip root joint variables
        if (var->joint == GetRootJoint(model)) continue;

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

constexpr auto dt = 1.0 / 500.0;
constexpr auto odt = 500.0;

void Read(ControlledRobot* robot)
{
    for (auto i = 0; i < robot->variable_count; ++i) {
        switch (robot->control_modes[i]) {
        case ControlledRobot::Position:
        {
            auto prev_position = robot->joint_positions[i];
            auto prev_velocity = robot->joint_velocities[i];

            robot->joint_positions[i] = robot->position_commands[i];
            robot->joint_velocities[i] = (robot->joint_positions[i] - prev_position) * odt;
            robot->joint_efforts[i] = (robot->joint_velocities[i] - prev_velocity) * odt;
            break;
        }
        case ControlledRobot::Velocity:
        {
            auto prev_velocity = robot->joint_velocities[i];

            robot->joint_velocities[i] = robot->velocity_commands[i];
            robot->joint_positions[i] += robot->joint_velocities[i] * dt;
            robot->joint_efforts[i] = (robot->joint_velocities[i] - prev_velocity) * odt;
            break;
        }
        case ControlledRobot::Effort:
        {
            break;
        }
        }
    }
}

void Write(ControlledRobot* robot)
{
#if 0
    printf("position command: [");
    for (auto i = 0; i < robot->variable_count; ++i) {
        if (i != 0) {
            printf(", %f", robot->position_commands[i]);
        } else {
            printf("%f", robot->position_commands[i]);
        }
    }
    printf("]\n");
    printf("velocity command: [");
    for (auto i = 0; i < robot->variable_count; ++i) {
        if (i != 0) {
            printf(", %f", robot->velocity_commands[i]);
        } else {
            printf("%f", robot->velocity_commands[i]);
        }
    }
    printf("]\n");
    printf("  effort command: [");
    for (auto i = 0; i < robot->variable_count; ++i) {
        if (i != 0) {
            printf(", %f", robot->effort_commands[i]);
        } else {
            printf("%f", robot->effort_commands[i]);
        }
    }
    printf("]\n");
#endif

    for (auto i = 0; i < robot->variable_count; ++i) {
        if (robot->position_commands[i] != robot->prev_position_commands[i]) {
            robot->control_modes[i] = ControlledRobot::Position;
            robot->prev_position_commands[i] = robot->position_commands[i];
        } else if (robot->velocity_commands[i] != robot->prev_velocity_commands[i]) {
            robot->control_modes[i] = ControlledRobot::Velocity;
            robot->prev_velocity_commands[i] = robot->velocity_commands[i];
        } else if (robot->effort_commands[i] != robot->prev_effort_commands[i]) {
            robot->control_modes[i] = ControlledRobot::Effort;
            robot->prev_effort_commands[i] = robot->effort_commands[i];
        }
    }
}

#if ROS_KINETIC
void ControlledRobot::read(const ros::Time& time, const ros::Duration& dt) override
{
    Read(this);
}
#else
void ControlledRobot::read()
{
    Read(this);
}
#endif

#if ROS_KINETIC
void ControlledRobot::write(const ros::Time& time, const ros::Duration& dt) override
{
    Write(this);
};
#else
void ControlledRobot::write()
{
    Write(this);
}
#endif

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

    auto empty = [&]()
    {
#if 1
        // someone please explain to me how to prevent a controller manager from
        // dying until all externally-loaded controllers have been unloaded if
        // there is no way to query how many controllers exist, without
        // hijacking these private members.
        boost::unique_lock<boost::recursive_mutex> lock(manager.controllers_lock_);
        return manager.controllers_lists_[manager.current_controllers_list_].empty();
#else
        return false;
#endif
    };

    auto prev_time = ros::Time::now();
    while (ros::ok() && !empty()) {
        auto now = ros::Time::now();
        auto dt = now - prev_time;
        prev_time = now;

#if ROS_KINETIC
        robot.read(now, dt);
#else
        robot.read();
#endif

        manager.update(now, dt);

#if ROS_KINETIC
        robot.write(now, dt);
#else
        robot.write();
#endif

        loop_rate.sleep();
    }

    return 0;
}

