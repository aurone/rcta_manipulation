#ifndef hdt_TeleopNode_h
#define hdt_TeleopNode_h

// standard includes
#include <functional>
#include <map>
#include <memory>

// system includes
#include <LinuxJoystick.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf_parser/urdf_parser.h>

namespace au
{
class ConfigBlock;
}

namespace hdt
{

class Controller;

class TeleopNode
{
public:

    TeleopNode();
    ~TeleopNode();

    enum RunResult
    {
        SUCCESS = 0,
        FAILED_TO_INITIALIZE
    };

    RunResult run();

private:

    static const double MAX_DELTA_POS_RPS;
    static const double MAX_DELTA_VEL_RPSPS;
    static const double MAX_DELTA_TORQUE_NMPS;
    static const double MAX_DELTA_CURRENT_APS;

    LinuxJoystick joystick_;
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Subscriber hdt_diagnostic_sub_;
    ros::Subscriber joint_state_sub_;

    ros::Publisher command_pub_;
    ros::Publisher diagnostic_pub_;
    ros::Publisher ack_reset_pub_;

    std::map<int, std::function<void()>> button_event_handlers_;
    std::map<int, std::function<void(double)>> axis_event_handlers_;

    bool shutdown_;
    double loop_rate_hz_;

    std::shared_ptr<Controller> controller_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_positions_;
    std::vector<double> joint_max_positions_;

    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    sensor_msgs::JointState::ConstPtr last_joint_state_;

    size_t selected_joint_idx_;
    std::vector<double> joint_pos_target_;
    std::vector<double> joint_vel_target_;
    // std::vector<double> joint_accel_target_; // TODO

    boost::shared_ptr<urdf::ModelInterface> robot_model_;

    bool deadman_on_; // true => can send commands

    bool init();

    typedef void (TeleopNode::*ButtonCommandMemberFn)();
    typedef void (TeleopNode::*AxisCommandMemberFn)(double);

    bool register_button_callback(const au::ConfigBlock& mapping_config, const std::string& command_name, ButtonCommandMemberFn);
    bool register_axis_callback(const au::ConfigBlock& mapping_config, const std::string& command_name, AxisCommandMemberFn);

    void shutdown();

    // Axis Commands
    void no_command(double value);
    void set_position(double value);
    void set_velocity(double value);
    void set_torque(double value);
    void set_current(double value);
    void set_inertia(double value);
    void set_damping(double value);
    void set_stiffness(double value);
    void some_fun(double);

    // Button Commands
    void no_command();
    void quit();
    void increase_current_joint();
    void decrease_current_joint();
    void reset_target_pos_and_vel();
    void reset_target_torque_and_current();
    void send_motion_command();
    void acknowledge_reset();
    void send_impedance_command();
    void send_impedance_off_command();
    void reset_impedance_params();
    void dead_man_switch_on();
    void dead_man_switch_off();

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool get_joint_value(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_value) const;
    bool get_joint_velocity(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_velocity) const;

    bool at_limit(int joint_index) const;
};

} // namespace hdt

#endif
