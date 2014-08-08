#ifndef JointInterpolationPathGenerator_h
#define JointInterpolationPathGenerator_h

#include <memory>
#include <vector>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sbpl_geometry_utils/shortcut.h>

namespace sbpl_arm_planner
{
class SBPLCollisionSpace;
}

namespace hdt
{

class JointInterpolationPathGenerator : public sbpl::shortcut::PathGenerator<trajectory_msgs::JointTrajectoryPoint, int>
{
public:

    JointInterpolationPathGenerator();

    bool initialize(
        const std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace>& collision_checker,
        const std::vector<double>& min_limits,
        const std::vector<double>& max_limits,
        const std::vector<bool>& continuous);

    bool generate_path(
        const trajectory_msgs::JointTrajectoryPoint& start,
        const trajectory_msgs::JointTrajectoryPoint& end,
        std::vector<trajectory_msgs::JointTrajectoryPoint>& path_out,
        int& cost_out) const;

private:

    std::shared_ptr<sbpl_arm_planner::SBPLCollisionSpace> collision_checker_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<bool> continuous_;
};

} // namespace hdt

#endif
