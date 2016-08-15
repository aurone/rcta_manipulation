#ifndef hdt_ManipulatorInterfaceLiveROS_h
#define hdt_ManipulatorInterfaceLiveROS_h

// standard includes
#include <vector>

// system includes
#include <HDTManipulatorInterface.h>
#include <ros/ros.h>

// project includes
#include <rcta/AcknowledgeReset.h>
#include <rcta/ClearEmergencyStop.h>
#include <rcta/EmergencyStop.h>
#include <hdt_description/RobotModel.h>

// module includes
#include "manipulator_interface_ros.h"

namespace hdt
{

class ManipulatorInterfaceLiveROS : public ManipulatorInterfaceROS
{
public:

    ManipulatorInterfaceLiveROS();
    ~ManipulatorInterfaceLiveROS();

    RunResult run();

    const ManipulatorError& init_error() const { return init_error_; }

private:

    HDTManipulatorInterface manip_;
    ManipulatorError init_error_;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher follow_joint_traj_feedback_pub_;
    ros::Publisher joint_states_pub_;
    ros::Publisher hdt_joint_states_pub_;
    ros::Publisher diagnostic_status_pub_;
    ros::Subscriber joint_traj_sub_;
    ros::Subscriber estop_sub_;
    ros::Subscriber clear_estop_sub_;
    ros::Subscriber ack_reset_sub_;

    std::vector<std::string> joint_names_;

    bool initialized_;

    bool estopped_;

    boost::shared_ptr<const trajectory_msgs::JointTrajectory> last_command_;

    RobotModelPtr robot_model_;

    bool init();

    void joint_trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void emergency_stop_callback(const rcta::EmergencyStop::ConstPtr& msg);
    void clear_emergency_stop_callback(const rcta::ClearEmergencyStop::ConstPtr& msg);
    void acknowledge_reset_callback(const rcta::AcknowledgeReset::ConstPtr& msg);

    void target_position_and_velocity(const std::vector<float>& position, const std::vector<float>& velocity);
    void halt();

    std::vector<double> extract_target_command(const trajectory_msgs::JointTrajectory& command_msg) const;

    bool assert_joint_limits(const ManipulatorParameters& manip_params);
};

} // namespace hdt

#endif
