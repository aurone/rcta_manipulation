#include "object_manip_checker.h"

#include "variables.h"

auto ExtractState(const smpl::RobotState& state) -> smpl::RobotState
{
    auto s = state;
    s.pop_back();
    return s;
}

constexpr auto z_thresh = 0.03;

bool ObjectManipChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    if (state[WORLD_JOINT_Z] * state[WORLD_JOINT_Z] > z_thresh * z_thresh) {
        return false;
    }
    return parent->isStateValid(ExtractState(state), verbose);
}

bool ObjectManipChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    if (start[WORLD_JOINT_Z] * start[WORLD_JOINT_Z] > z_thresh * z_thresh) {
        return false;
    }
    if (finish[WORLD_JOINT_Z] * finish[WORLD_JOINT_Z] > z_thresh * z_thresh) {
        return false;
    }
    return parent->isStateToStateValid(ExtractState(start), ExtractState(finish), verbose);
}

bool ObjectManipChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    if (start.back() != finish.back()) {
        path.push_back(start);
        path.push_back(finish);
        return true;
    }

    auto old_size = path.size();
    if (!parent->interpolatePath(ExtractState(start), ExtractState(finish), path)) {
        return false;
    }
    auto new_size = path.size();

    for (auto i = old_size; i != new_size; ++i) {
        path[i].push_back(start.back());
    }

    return true;
}

auto ObjectManipChecker::getCollisionModelVisualization(const smpl::RobotState& state)
    -> std::vector<smpl::visual::Marker>
{
    return parent->getCollisionModelVisualization(ExtractState(state));
}

auto ObjectManipChecker::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<CollisionChecker>()) {
        return this;
    }
    return NULL;
}

