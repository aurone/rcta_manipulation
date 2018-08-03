#include "object_manipulation_model.h"

double ObjectManipModel::minPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->minPosLimit(vidx);
    } else {
        return this->min_object_pos;
    }
}

double ObjectManipModel::maxPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->maxPosLimit(vidx);
    } else {
        return this->max_object_pos;
    }
}

bool ObjectManipModel::hasPosLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->hasPosLimit(vidx);
    } else {
        return true;
    }
}

bool ObjectManipModel::isContinuous(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->isContinuous(vidx);
    } else {
        return false;
    }
}

double ObjectManipModel::velLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->velLimit(vidx);
    } else {
        return 0.0;
    }
}

double ObjectManipModel::accLimit(int vidx) const
{
    if (vidx < this->parent_model->jointCount()) {
        return this->parent_model->accLimit(vidx);
    } else {
        return 0.0;
    }
}

bool ObjectManipModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    auto ovar = state.back();
    return parent_model->checkJointLimits(state, verbose) &
            (ovar >= this->min_object_pos) &
            (ovar <= this->max_object_pos);
}

auto ObjectManipModel::getExtension(size_t class_code)
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

auto ObjectManipModel::objectJointName() const -> const std::string&
{
    return this->getPlanningJoints().back();
}

// we should just be able to use the underlying robot model's forward
// kinematics
auto ObjectManipModel::computeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    auto small_state = ExtractParentState(state, this->parent_model);
    return this->fk_iface->computeFK(small_state);
}

// same for ik...
bool ObjectManipModel::computeIK(
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

bool ObjectManipModel::computeIK(
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

// the redundant manipulator interface will be a little different...
// we need to tell smpl that the object dimension is a 'free angle'
auto ObjectManipModel::redundantVariableCount() const -> const int
{
    return this->rm_iface->redundantVariableCount() + 1;
}

auto ObjectManipModel::redundantVariableIndex(int rvidx) const
    -> const int
{
    if (rvidx < this->rm_iface->redundantVariableCount()) {
        return this->rm_iface->redundantVariableIndex(rvidx);
    } else {
        // the object variable is always the last variable in the robot state
        return this->parent_model->jointCount();
    }
}

bool ObjectManipModel::computeFastIK(
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
    ObjectManipModel* model,
    smpl::RobotModel* parent,
    const std::string& object_joint_name,
    double object_min,
    double object_max)
{
    model->parent_model = parent;
    model->fk_iface = GetExtension<smpl::ForwardKinematicsInterface>(parent);
    model->ik_iface = GetExtension<smpl::InverseKinematicsInterface>(parent);
    model->rm_iface = GetExtension<smpl::RedundantManipulatorInterface>(parent);
    model->min_object_pos = object_min;
    model->max_object_pos = object_max;

    auto parent_joints = parent->getPlanningJoints();
    parent_joints.push_back(object_joint_name);
    model->setPlanningJoints(parent_joints);
    return true;
}

