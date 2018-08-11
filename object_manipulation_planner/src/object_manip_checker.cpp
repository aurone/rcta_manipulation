#include "object_manip_checker.h"

auto ExtractState(const smpl::RobotState& state) -> smpl::RobotState
{
    auto s = state;
    s.pop_back();
    return s;
}

bool ObjectManipChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    return parent->isStateValid(ExtractState(state), verbose);
}

bool ObjectManipChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    return parent->isStateToStateValid(ExtractState(start), ExtractState(finish), verbose);
}

bool ObjectManipChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    if (!parent->interpolatePath(ExtractState(start), ExtractState(finish), path)) {
        return false;
    }
    for (auto& s : path) {
        s.push_back(start.back());
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
    return nullptr;
}

