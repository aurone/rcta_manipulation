#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include "gripper_command_action.h"
#include "gripper_interface.h"
#include "gripper_connection.h"

namespace robotiq
{

std::string GripperCommandActionExecutor::to_string(RunResult r)
{
    switch (r)
    {
    case SUCCESS:
        return "SUCCESS";
    case FAILED_TO_INITIALIZE:
        return "FAILED_TO_INITIALIZE";    
    default:
        return "INVAILD";
    }
}

GripperCommandActionExecutor::GripperCommandActionExecutor(ros::NodeHandle& nh) :
    nh_(nh),
    ph_("~"),
    action_server_name_("gripper_controller/gripper_command_action"),
    feedback_(),
    result_(),
    connection_(),
    gripper_(),
    gripper_throttle_rate_hz_(30),
    joint_state_pub_(),
    gripper_mutex_()
{
}

GripperCommandActionExecutor::~GripperCommandActionExecutor()
{

}

GripperCommandActionExecutor::RunResult GripperCommandActionExecutor::run()
{
    // read in connection parameters

    std::string hostname;
    if (!ph_.getParam("hostname", hostname)) {
        ROS_ERROR("Failed to retrieve 'hostname' parameter");
        return FAILED_TO_INITIALIZE;
    }

    int portno;
    if (!ph_.getParam("port", portno)) {
        ROS_ERROR("Failed to retrieve 'port' parameter");
        return FAILED_TO_INITIALIZE;
    }

    // create gripper connection options

    unsigned long gripper_ip = resolve_to_ipv4(hostname, portno);
    if (!gripper_ip) {
        return FAILED_TO_INITIALIZE;
    }
    else {
        ROS_INFO("Resolved %s to %lx", hostname.c_str(), gripper_ip);
    }

    GripperConnection::ConnectionOptions ops;
    ops.ip_address = gripper_ip;
    ops.portno = (uint16_t)portno;

    // connect to gripper with configured connection

    connection_.reset(new GripperConnection(ops));
    if (!connection_) {
        ROS_ERROR("Failed to instantiate Gripper Connection");
        return FAILED_TO_INITIALIZE;
    }

    ROS_INFO("Connecting to gripper...");

    std::string why;
    if (!connection_->connect(why)) {
        ROS_ERROR("Failed to connect to Robotiq Gripper (%s)", why.c_str());
        return FAILED_TO_INITIALIZE;
    }

    // create an interface to the gripper
    ROS_INFO("Connected to gripper");

    // don't allow this node to rely on sending gripper commands faster than 30hz (force gripper interface to throttle attempts)
    gripper_.reset(new GripperInterface(connection_, gripper_throttle_rate_hz_));
    if (!gripper_) {
        ROS_ERROR("Failed to instantiate Gripper Interface");
        return FAILED_TO_INITIALIZE;
    }

    ROS_INFO("Activating the gripper");
    gripper_->activate();
    if (!gripper_->update()) {
        ROS_WARN("Failed to update gripper after activation request");
    }

    ROS_INFO("Waiting for gripper to activate");
    while (!gripper_->is_activated()) {
        gripper_->update();
    }
    ROS_INFO("Gripper finished activating");

    ROS_INFO("Gripper activated. State is %s", ::to_string(gripper_->get_status()).c_str());

    ROS_INFO("Starting action server...");

//    auto callback = boost::bind(&GripperCommandActionExecutor::execute_callback, this, _1);
    auto goal_cb = boost::bind(&GripperCommandActionExecutor::goal_callback, this);
    auto preempt_cb = boost::bind(&GripperCommandActionExecutor::preempt_callback, this);
    action_server_.reset(new GripperCommandActionServer(action_server_name_/*, callback*/, false));
    if (!action_server_) {
        ROS_ERROR("Failed to instantiate Simple Action Server");
        return FAILED_TO_INITIALIZE;
    }

    action_server_->registerGoalCallback(goal_cb);
    action_server_->registerPreemptCallback(preempt_cb);

    action_server_->start();
    ROS_INFO("Started action server");

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 5);

    const std::vector<std::string> fake_joint_names = { "finger1", "finger2" };

    ros::Rate loop_rate(gripper_throttle_rate_hz_);
    // note: this loop will be throttled by the update rate of the gripper
    // interface, but we'll also enforce that throttling here as well
    while (ros::ok())
    {
        ros::spinOnce();

        gripper_->update();

        ////////////////////////////////////////////////////////////////////////////////
        // Publish gripper state
        ////////////////////////////////////////////////////////////////////////////////

        double gripper_pos = gripper_->get_position();
        double gripper_speed = gripper_->get_speed();
        double gripper_force = gripper_->get_force();

        if (gripper_pos != -1.0) {
            sensor_msgs::JointState joint_state;
            static int seqno = 0;
            joint_state.header.seq = seqno++;
            joint_state.header.stamp = ros::Time::now();
            joint_state.header.frame_id = "";
            joint_state.name = fake_joint_names;
            joint_state.position.resize(2);
            joint_state.position[0] = gripper_pos / 2.0;
            joint_state.position[1] = gripper_pos / 2.0;
            joint_state.velocity.resize(2);
            joint_state.velocity[0] = gripper_speed / 2.0;
            joint_state.velocity[1] = gripper_speed / 2.0;
            joint_state.effort.resize(2);
            joint_state.effort[0] = gripper_force / 2.0;
            joint_state.effort[1] = gripper_force / 2.0;

            joint_state_pub_.publish(joint_state);
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Work on any active goal
        ////////////////////////////////////////////////////////////////////////////////

        if (action_server_->isActive())
        {
            ROS_INFO("Continuing execution of gripper goal to position %0.3f", curr_goal_->command.position);

            feedback_.position = gripper_pos;
            feedback_.effort = gripper_force;
            feedback_.stalled = !gripper_->fingers_in_motion();
            feedback_.reached_goal = gripper_->completed_positioning();

            bool finished = false;
            if (gripper_->completed_positioning()) {
                ROS_INFO("Gripper completed requested positioning");
                finished = true;
            }
            else if (gripper_->made_contact_closing() || gripper_->made_contact_opening()) {
                ROS_INFO("Gripper finished moving due to contact");
                finished = true;
            }

            if (finished) {
                ROS_INFO("Successfully positioned gripper");
                result_.position = feedback_.position;
                result_.effort = feedback_.effort;
                result_.stalled = feedback_.stalled;
                result_.reached_goal = feedback_.reached_goal;
                action_server_->setSucceeded(result_);
            }
        }

        loop_rate.sleep();
    }

    action_server_->shutdown();

    return SUCCESS;
}

void GripperCommandActionExecutor::execute_callback(const control_msgs::GripperCommandGoalConstPtr& goal)
{
}

void GripperCommandActionExecutor::goal_callback()
{
    curr_goal_ = action_server_->acceptNewGoal();

    auto clamp = [](double val, double min, double max) { if (val < min) return min; else if (val > max) return max; else return val; };
    double commanded_position = clamp(curr_goal_->command.position, gripper_->model().minimum_width(), gripper_->model().maximum_width());
    double commanded_force = clamp(curr_goal_->command.max_effort, gripper_->model().minimum_force(), gripper_->model().maximum_force());

    ROS_INFO("Clamping gripper command %0.3f to [%0.3f, %0.3f]", curr_goal_->command.position, gripper_->model().minimum_width(), gripper_->model().maximum_width());
    ROS_INFO("Clamping gripper effort %0.3f to [%0.3f, %0.3f]", curr_goal_->command.max_effort, gripper_->model().minimum_force(), gripper_->model().maximum_force());

    gripper_->set_force(commanded_force);
    gripper_->set_position(commanded_position);

    // give the gripper at least five update cycles to get its shit together
    ros::Duration(1.0).sleep();//5.0 / gripper_->model().update_rate()).sleep();
}

void GripperCommandActionExecutor::preempt_callback()
{
    ROS_INFO("Preempt requested");
    control_msgs::GripperCommandResult result;
    result.position = gripper_->get_position();
    result.effort = gripper_->get_force();
    result.stalled = !gripper_->fingers_in_motion();
    result.reached_goal = gripper_->completed_positioning();
    action_server_->setPreempted(result, "Unconditional preempt accepted");
}

unsigned long GripperCommandActionExecutor::resolve_to_ipv4(const std::string& hostname, uint16_t portno)
{
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::resolver resolver(io_service); // for resolving the given hostname
    std::string service_name(std::to_string(portno));
    boost::asio::ip::tcp::resolver::query query(hostname, service_name);
    boost::asio::ip::tcp::resolver::iterator it = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;
    unsigned long gripper_ip = 0;
    while (it != end) {
        boost::asio::ip::tcp::endpoint endpoint = *it++;
        if (endpoint.address().is_v4()) {
            boost::asio::ip::address_v4 ipv4_addr = endpoint.address().to_v4();
            gripper_ip = ipv4_addr.to_ulong();
            break;
        }
    }
    return gripper_ip;
}

} // namespace robotiq

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gripper_command_action");
    ros::NodeHandle nh;

    robotiq::GripperCommandActionExecutor gca(nh);
    robotiq::GripperCommandActionExecutor::RunResult result = gca.run();
    if (result != robotiq::GripperCommandActionExecutor::SUCCESS) {
        ROS_ERROR("Gripper Command Action terminated with result %s", robotiq::GripperCommandActionExecutor::to_string(result).c_str());
        exit(1);
    }

    return 0;
}

