#include "roman_robot_model.h"

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>

const char* RomanRobotModel::LOG = "robot.roman";

// I forget why this is required here. Sometimes it's needed because
// getExtension<T>() is shadowed by a concrete type's definition of
// getExtension(size_t), but we're not using a concrete type here, so
// getExtension<T>() should be found.
template <class T>
T* GetExtension(smpl::Extension* extension)
{
    auto* e = extension->getExtension(smpl::GetClassCode<T>());
    return dynamic_cast<T*>(e);
}

bool Init(RomanRobotModel* model, smpl::RobotModel* parent)
{
    model->parent_model = parent;
    model->fk_iface = GetExtension<smpl::ForwardKinematicsInterface>(parent);
    model->ik_iface = GetExtension<smpl::InverseKinematicsInterface>(parent);
    model->rm_iface = GetExtension<smpl::RedundantManipulatorInterface>(parent);

    auto all_joints = parent->getPlanningJoints();
    all_joints.push_back("world_z");
    model->setPlanningJoints(all_joints);
    return true;
}

auto GetZVariableName(const RomanRobotModel* model) -> const std::string&
{
    return model->getPlanningJoints().back();
}

double RomanRobotModel::minPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->minPosLimit(vidx);
    } else {
        return -std::numeric_limits<double>::infinity();
    }
}

double RomanRobotModel::maxPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->maxPosLimit(vidx);
    } else {
        return std::numeric_limits<double>::infinity();
    }
}

bool RomanRobotModel::hasPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->hasPosLimit(vidx);
    } else {
        return false;
    }
}

bool RomanRobotModel::isContinuous(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->isContinuous(vidx);
    } else {
        return false;
    }
}

double RomanRobotModel::velLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->velLimit(vidx);
    } else {
        return 0.0;
    }
}

double RomanRobotModel::accLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->accLimit(vidx);
    } else {
        return 0.0;
    }
}

auto ExtractParentState(const smpl::RobotState& state, smpl::RobotModel* parent_model)
    -> smpl::RobotState
{
    auto small_state = smpl::RobotState();
    std::copy(
            begin(state),
            begin(state) + parent_model->jointVariableCount(),
            std::back_inserter(small_state));
    return small_state;
}

bool RomanRobotModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    auto small_state = ExtractParentState(state, this->parent_model);
    auto ovar = state.back();
    return parent_model->checkJointLimits(small_state, verbose);
}

auto RomanRobotModel::getExtension(size_t class_code)
    -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>()) {
        return (this->fk_iface != NULL) ? this : NULL;
    } else if (class_code == smpl::GetClassCode<smpl::InverseKinematicsInterface>()) {
        return (this->ik_iface != NULL) ? this : NULL;
    } else if (class_code == smpl::GetClassCode<smpl::RedundantManipulatorInterface>()) {
        return (this->rm_iface != NULL) ? this : NULL;
    } else if (class_code == smpl::GetClassCode<smpl::RobotModel>()) {
        return this;
    }

    return NULL;
}

// TODO: need to apply the z position offset
auto RomanRobotModel::computeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    auto small_state = ExtractParentState(state, this->parent_model);
    return this->fk_iface->computeFK(small_state);
}

// TODO: need to apply the z position offset
bool RomanRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& seed,
    smpl::RobotState& solution,
    smpl::ik_option::IkOption option)
{
    SMPL_DEBUG_STREAM_NAMED(LOG, "compute ik(seed = " << seed << ")");
    auto small_seed = ExtractParentState(seed, this->parent_model);
    if (!this->ik_iface->computeIK(pose, small_seed, solution, option)) {
        return false;
    }

    // add the object variable to the solution
    solution.push_back(seed.back());
    return true;
}

// TODO: need to apply the z offset
bool RomanRobotModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    std::vector<smpl::RobotState>& solutions,
    smpl::ik_option::IkOption option)
{
    SMPL_DEBUG_STREAM_NAMED(LOG, "compute multi-ik(seed = " << start << ")");
    auto small_seed = ExtractParentState(start, this->parent_model);
    if (!this->ik_iface->computeIK(pose, small_seed, solutions, option)) {
        return false;
    }

    // add the object variable to all solutions
    for (auto& sol : solutions) {
        sol.push_back(start.back());
    }
    return true;
}

// All the redundant variables of the parent model, +1 for z
auto RomanRobotModel::redundantVariableCount() const -> const int
{
    return this->rm_iface->redundantVariableCount() + 1;
}

auto RomanRobotModel::redundantVariableIndex(int rvidx) const
    -> const int
{
    if (rvidx < this->rm_iface->redundantVariableCount()) {
        return this->rm_iface->redundantVariableIndex(rvidx);
    } else {
        return this->parent_model->jointCount();
    }
}

// TODO: undo the z
bool RomanRobotModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& seed,
    smpl::RobotState& solution)
{
    SMPL_DEBUG_STREAM_NAMED(LOG, "compute fast-ik(seed = " << seed << ")");
    auto small_seed = ExtractParentState(seed, this->parent_model);
    if (!this->rm_iface->computeFastIK(pose, small_seed, solution)) {
        return false;
    }

    // add the object variable to all solutions
    solution.push_back(seed.back());
    return true;
}
