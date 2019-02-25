#ifndef OBJECT_MANIPULATION_PLANNER_OBJECT_MANIP_CHECKER_H
#define OBJECT_MANIPULATION_PLANNER_OBJECT_MANIP_CHECKER_H

#include <smpl/collision_checker.h>
#include <smpl/spatial.h>

namespace smpl {
namespace collision {
class CollisionSpace;
}
}

struct ObjectManipCheckerImpl;

class ObjectManipChecker : public smpl::CollisionChecker
{
public:

    ObjectManipCheckerImpl* impl = NULL;

//    smpl::CollisionChecker* parent = NULL;
    smpl::collision::CollisionSpace* parent = NULL;

    smpl::Affine3 object_pose;

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

bool Init(ObjectManipChecker* checker, smpl::collision::CollisionSpace* parent);
void SetObjectPose(ObjectManipChecker* checker, const smpl::Affine3& pose);

#endif
