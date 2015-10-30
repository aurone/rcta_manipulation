#include "JointInterpolationPathGenerator.h"

#include <sbpl_collision_checking/sbpl_collision_space.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <hdt/common/stringifier/stringifier.h>

namespace hdt
{

JointInterpolationPathGenerator::JointInterpolationPathGenerator() :
    collision_checker_(),
    min_limits_(),
    max_limits_(),
    continuous_()
{
}

bool JointInterpolationPathGenerator::initialize(
    const std::shared_ptr<sbpl::collision::SBPLCollisionSpace>& collision_checker,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits,
    const std::vector<bool>& continuous)
{
    if (!collision_checker || min_limits.size() != max_limits.size() || min_limits.size() != continuous.size()) {
        return false;
    }

    collision_checker_ = collision_checker;
    min_limits_ = min_limits;
    max_limits_ = max_limits;
    continuous_ = continuous;
    return true;
}

bool JointInterpolationPathGenerator::generate_path(
    const trajectory_msgs::JointTrajectoryPoint& start,
    const trajectory_msgs::JointTrajectoryPoint& end,
    std::vector<trajectory_msgs::JointTrajectoryPoint>& path_out,
    int& cost_out) const
{
    std::vector<double> inc(min_limits_.size(), sbpl::utils::ToRadians(1.0));

    std::vector<std::vector<double>> path;
    if (!sbpl::interp::InterpolatePath(start.positions, end.positions, min_limits_, max_limits_, inc, continuous_, path)) {
        ROS_ERROR("Failed to shortcut between %s and %s with joint interpolation", to_string(start.positions).c_str(), to_string(end.positions).c_str());
        return false;
    }
    else {
        for (std::vector<double>& point : path) {
            double dist;
            if (collision_checker_->isStateValid(point, false, false, dist)) {
                trajectory_msgs::JointTrajectoryPoint jt_point;
                jt_point.positions = std::move(point);
                path_out.push_back(std::move(jt_point));
            }
            else {
                return false;
            }
        }
        cost_out = 0; ///< hacking my way to victory
        return true;
    }
}

} // namespace hdt
