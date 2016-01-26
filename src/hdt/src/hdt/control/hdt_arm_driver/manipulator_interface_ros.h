#ifndef hdt_ManipulatorInterfaceROS_h
#define hdt_ManipulatorInterfaceROS_h

#include <string>
#include <vector>
#include <ManipulatorError.h>
#include <ManipulatorParameters.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace hdt
{

template <typename T> std::string to_string(const std::vector<T>& vec);
template <> inline std::string to_string<double>(const std::vector<double>& vec);
template <> inline std::string to_string<float>(const std::vector<float>& vec);

std::string to_string(const ManipulatorError& error);

class ManipulatorInterfaceROS
{
public:

    enum RunResult
    {
        SUCCESS,
        FAILED_TO_INITIALIZE
    };

    static const char *RunResultToString(RunResult result);

    ManipulatorInterfaceROS();
    virtual ~ManipulatorInterfaceROS() { }

    virtual RunResult run() = 0;

    // return the initialization status or NO_ERROR if initialization was never attempted
    virtual const ManipulatorError& init_error() const = 0;

    const std::vector<std::string>& joint_names() const { return joint_names_; }

    bool read_manip_params(const std::string& manip_config_fname, ManipulatorParameters& out);

    bool check_joints(const std::vector<std::string>& injoints) const;

    int find_joint_index(const std::string& joint_name, const trajectory_msgs::JointTrajectory& joint_traj) const;

private:

    std::vector<std::string> joint_names_;
};

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

template <> std::string to_string<double>(const std::vector<double>& vec)
{
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < vec.size(); ++i) {
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

template <> std::string to_string<float>(const std::vector<float>& vec)
{
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < vec.size(); ++i) {
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

} // namespace hdt

#endif
