#include "object_manipulation_model.h"

double ObjectManipulationModel::minPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->minPosLimit(vidx);
    } else {
        return this->min_object_pos;
    }
}

double ObjectManipulationModel::maxPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->maxPosLimit(vidx);
    } else {
        return this->max_object_pos;
    }
}

bool ObjectManipulationModel::hasPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->hasPosLimit(vidx);
    } else {
        return true;
    }
}

bool ObjectManipulationModel::isContinuous(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->isContinuous(vidx);
    } else {
        return false;
    }
}

double ObjectManipulationModel::velLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->velLimit(vidx);
    } else {
        return 0.0;
    }
}

double ObjectManipulationModel::accLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->accLimit(vidx);
    } else {
        return 0.0;
    }
}

bool ObjectManipulationModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    auto ovar = state.back();
    return parent_model->checkJointLimits(state, verbose) &
            (ovar >= this->min_object_pos) &
            (ovar <= this->max_object_pos);
}

auto ObjectManipulationModel::getExtension(size_t class_code)
    -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>()) {
        return this->fk_iface ? this : NULL;
    } else if (class_code == smpl::GetClassCode<smpl::InverseKinematicsInterface>()) {
        return this->ik_iface ? this : NULL;
    } else if (class_code == smpl::GetClassCode<smpl::RedundantManipulatorInterface>()) {
        return this->rm_iface ? this : NULL;
    } else if (class_code == smpl::GetClassCode<smpl::RobotModel>()) {
        return this;
    }

    return NULL;
}

auto ExtractParentState(const smpl::RobotState& state, smpl::RobotModel* parent_model)
    -> smpl::RobotState
{
    smpl::RobotState small_state;
    std::copy(
            begin(state),
            begin(state) + parent_model->jointVariableCount(),
            std::back_inserter(small_state));
    return small_state;
}

auto ObjectManipulationModel::computeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    auto small_state = ExtractParentState(state, this->parent_model);
    return this->fk_iface->computeFK(small_state);
}

bool ObjectManipulationModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    smpl::RobotState& solution,
    smpl::ik_option::IkOption option)
{
    auto small_start = ExtractParentState(start, this->parent_model);
    if (!this->ik_iface->computeIK(pose, small_start, solution, option)) {
        return false;
    }

    // add the object variable to the solution
    solution.push_back(start.back());
    return true;
}

bool ObjectManipulationModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& start,
    std::vector<smpl::RobotState>& solutions,
    smpl::ik_option::IkOption option)
{
    auto small_start = ExtractParentState(start, this->parent_model);
    if (!this->ik_iface->computeIK(pose, small_start, solutions, option)) {
        return false;
    }

    // add the object variable to all solutions
    for (auto& sol : solutions) {
        sol.push_back(start.back());
    }
    return true;
}

auto ObjectManipulationModel::redundantVariableCount() const -> const int
{
    return this->rm_iface->redundantVariableCount() + 1;
}

auto ObjectManipulationModel::redundantVariableIndex(int rvidx) const
    -> const int
{
    if (rvidx < this->rm_iface->redundantVariableCount()) {
        return this->rm_iface->redundantVariableIndex(rvidx);
    } else {
        // the object variable is always the last variable in the robot state
        return this->parent_model->jointCount();
    }
}

bool ObjectManipulationModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& state,
    smpl::RobotState& solution)
{
    auto small_start = ExtractParentState(state, this->parent_model);
    if (!this->rm_iface->computeFastIK(pose, small_start, solution)) {
        return false;
    }

    // add the object variable to all solutions
    solution.push_back(state.back());
    return true;
}

template <class T>
T* GetExtension(smpl::Extension* extension)
{
    auto* e = extension->getExtension(smpl::GetClassCode<T>());
    return dynamic_cast<T*>(e);
}

bool Init(
    ObjectManipulationModel* model,
    smpl::RobotModel* parent,
    double object_min,
    double object_max)
{
    model->parent_model = parent;
    model->fk_iface = GetExtension<smpl::ForwardKinematicsInterface>(parent);
    model->ik_iface = GetExtension<smpl::InverseKinematicsInterface>(parent);
    model->min_object_pos = object_min;
    model->max_object_pos = object_max;

    auto parent_joints = parent->getPlanningJoints();
    parent_joints.push_back("hinge");
    model->setPlanningJoints(parent_joints);
    return true;
}

