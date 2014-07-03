#ifndef hdt_ManipulatorInterfaceSimROS_h
#define hdt_ManipulatorInterfaceSimROS_h

#include <random>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "manipulator_interface_ros.h"

namespace hdt
{

class ManipulatorInterfaceSimROS : public ManipulatorInterfaceROS
{
public:

    ManipulatorInterfaceSimROS();
    ~ManipulatorInterfaceSimROS();

    RunResult run();

    const ManipulatorError& init_error() const { return init_error_; }

private:

    ManipulatorError init_error_;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    ros::Publisher follow_joint_traj_feedback_pub_;
    ros::Publisher joint_states_pub_;
    ros::Publisher diagnostic_status_pub_;
    ros::Subscriber joint_traj_sub_;

    std::vector<double> true_joint_positions_;
    std::vector<double> noisy_joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_position_command_;

    ManipulatorParameters manip_params_;

    std::map<std::string, int> joint_indices_;

    std::random_device rd_;
    std::mt19937 rng_;

    bool init();

    void joint_trajectory_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    double get_max_velocity(const std::string& joint_name) const;
    int get_joint_index(const std::string& joint_name) const;

    std::vector<double> neg(const std::vector<double>& values) const;
    std::vector<double> mul(double scalar, const std::vector<double>& values) const;
    std::vector<double> sum(const std::vector<double>& u, const std::vector<double>& v) const;

    std::vector<double> anglediff(const std::vector<double>& u,
                                  const std::vector<double>& v,
                                  const std::vector<double>& min_limits,
                                  const std::vector<double>& max_limits) const;

    const std::vector<double>& min_limits() const;
    const std::vector<double>& max_limits() const;
    const std::vector<double>& min_velocity_limits() const;
    const std::vector<double>& max_velocity_limits() const;

    template <typename T>
    void clamp(T& val, const T& min, const T& max) const
    {
        if (val < min) {
            val = min;
        }
        else if (val > max) {
            val = max;
        }
    }

    template <typename InputIterator>
    void clamp(InputIterator begin, InputIterator end, double min, double max) const
    {
        for (auto it = begin; it != end; ++it) {
            clamp(*it, min, max);
        }
    }

    template <typename Container>
    void clampall(Container& values, const Container& min, const Container& max) const
    {
        if (values.size() != min.size() || values.size() != max.size()) {
            return;
        }

        for (size_t i = 0; i < values.size(); ++i) {
            clamp(values[i], min[i], max[i]);
        }
    }
};

} // namespace hdt

#endif
