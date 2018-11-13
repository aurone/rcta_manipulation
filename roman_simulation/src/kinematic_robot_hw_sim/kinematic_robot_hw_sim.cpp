#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
//#include <gazebo/model.hh>
#include <ros/console.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <pluginlib/class_list_macros.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>

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

    std::vector<gazebo::physics::JointPtr> joints;
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

    std::vector<gazebo::physics::JointPtr> sim_joints;

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
    const smpl::urdf::RobotState* state,
    gazebo::physics::Model* gazebo_model)
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

    robot->joints.resize(internal_variable_count);

    // create and register handles to joint states and joint commands
    auto ii = 0;
    for (auto i = 0; i < GetVariableCount(model); ++i) {
        auto* var = GetVariable(model, i);

        // skip root joint variables
        if (var->joint == GetRootJoint(model)) continue;

        ROS_INFO("Make joint state interface for joint variable '%s'", var->name.c_str());

        auto joint = gazebo_model->GetJoint(var->name);
        if (joint == NULL) {
            ROS_ERROR("No corresponding Gazebo joint exists for variable '%s'", var->name.c_str());
            return false;
        }
        robot->joints[ii] = std::move(joint);

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
            printf("controlling via effort\n");
            break;
        }
        }

        robot->joints[i]->SetPosition(0, robot->joint_positions[i]);
        robot->joints[i]->SetVelocity(0, robot->joint_velocities[i]);
//        robot->joints[i]->SetForce(0, robot->joint_efforts[i]);
    }
}

void Write(ControlledRobot* robot)
{
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

void PrintRobot(const gazebo::physics::Model* model)
{
    printf("gazebo model has %u joints\n", model->GetJointCount());
    for (auto& joint : model->GetJoints()) {
        printf("Joint: %s\n", joint->GetName().c_str());
    }
    printf("gazebo model has %zu links\n", model->GetLinks().size());
    for (auto& link : model->GetLinks()) {
        printf("link: name = %s\n", link->GetName().c_str());
        printf("  gravity = %d\n", (int)link->GetGravityMode());
        printf("  enabled = %d\n", (int)link->GetEnabled());
        printf("  self_collide = %d\n", (int)link->GetSelfCollide());
//        printf("  laser retro = %d\n", (int)link->GetLaserRetro());
        printf("  linear damping = %f\n", link->GetLinearDamping());
        printf("  angular damping = %f\n", link->GetAngularDamping());
    }
    printf("children: %zu\n", model->NestedModels().size());
    for (auto& m : model->NestedModels()) {
        PrintRobot(m.get());
    }
}

class KinematicRobotHardware : public gazebo_ros_control::RobotHWSim
{
public:

    ControlledRobot robot;

    gazebo::physics::ModelPtr model;
    ros::Subscriber odom_sub;

    gazebo::math::Pose initial_pose;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        auto p = gazebo::math::Pose::Zero;
        p.pos.x = msg->pose.pose.position.x;
        p.pos.y = msg->pose.pose.position.y;
        p.pos.z = msg->pose.pose.position.z;
        p.rot.w = msg->pose.pose.orientation.w;
        p.rot.x = msg->pose.pose.orientation.x;
        p.rot.y = msg->pose.pose.orientation.y;
        p.rot.z = msg->pose.pose.orientation.z;
        model->SetLinkWorldPose(initial_pose + p, "base_link");
//        model->SetWorldPose(initial_pose + p);
    }

    bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr gazebo_model,
        const urdf::Model* const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions) override
    {
        model = gazebo_model;
        model->SetSelfCollide(false);
        model->SetGravityMode(false);

        this->initial_pose = gazebo_model->GetWorldPose();

        ros::NodeHandle nh;
        this->odom_sub = nh.subscribe("rcta_velocity_controller/odom", 10, &KinematicRobotHardware::odomCallback, this);

        printf("KinematicRobotHardware::initSim\n");

        for (auto& link : gazebo_model->GetLinks()) {
            link->SetGravityMode(false);
            link->SetSelfCollide(false);
            link->SetKinematic(true);
        }

        PrintRobot(gazebo_model.get());

        smpl::urdf::RobotModel robot_model;
        smpl::urdf::JointSpec world_joint;
        world_joint.name = "world_joint";
        world_joint.type = smpl::urdf::JointType::Floating;
        if (!InitRobotModel(&robot_model, urdf_model, &world_joint)) {
            return false;
        }

        auto with_velocities = true;
        auto with_accelerations = true;
        smpl::urdf::RobotState robot_state;
        if (!InitRobotState(&robot_state, &robot_model, with_velocities, with_accelerations)) {
            return false;
        }

        if (!InitControlledRobot(&robot, &robot_model, &robot_state, gazebo_model.get())) {
            return false;
        }

        this->registerInterface(&robot.i_joint_state);
        this->registerInterface(&robot.i_position_command);
        this->registerInterface(&robot.i_velocity_command);
        this->registerInterface(&robot.i_effort_command);

        return true;
    }

    void readSim(ros::Time time, ros::Duration period) override
    {
#if 0
        printf("KinematicRobotHardware::readSim\n");
#endif
        Read(&robot);
    }

    void writeSim(ros::Time time, ros::Duration period) override
    {
#if 0
        printf("KinematicRobotHardware::writeSim\n");
#endif
        Write(&robot);
    }
};

PLUGINLIB_EXPORT_CLASS(KinematicRobotHardware, gazebo_ros_control::RobotHWSim);
