#ifndef OBJECT_MANIPULATION_PLANNER_OBJECT_MANIP_CHECKER_H
#define OBJECT_MANIPULATION_PLANNER_OBJECT_MANIP_CHECKER_H

#include <smpl/collision_checker.h>

class ObjectManipChecker : public smpl::CollisionChecker
{
public:

    smpl::CollisionChecker* parent = NULL;

    bool isStateValid(
        const smpl::RobotState& state,
        bool verbose = false) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        bool verbose = false) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;

    auto getCollisionModelVisualization(const smpl::RobotState& state)
        -> std::vector<smpl::visual::Marker> override;

    auto getExtension(size_t class_code) -> smpl::Extension* override;
};

#endif
