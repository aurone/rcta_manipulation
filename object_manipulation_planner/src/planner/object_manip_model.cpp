#include "object_manip_model.h"

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>

static const char* R_LOG = "robot";

double ObjectManipModel::minPosLimit(int vidx) const
{
    return this->min_positions[vidx];
}

double ObjectManipModel::maxPosLimit(int vidx) const
{
    return this->max_positions[vidx];
}

bool ObjectManipModel::hasPosLimit(int vidx) const
{
    return this->pos_limited[vidx];
}

bool ObjectManipModel::isContinuous(int vidx) const
{
    return this->continuous[vidx];
}

double ObjectManipModel::velLimit(int vidx) const
{
    return this->vel_limits[vidx];
}

double ObjectManipModel::accLimit(int vidx) const
{
    return this->acc_limits[vidx];
}

// TODO: copypasta with object_manip_checker
static
auto ExtractParentState(const smpl::RobotState& state, smpl::RobotModel* parent_model)
    -> smpl::RobotState
{
    auto small_state = smpl::RobotState(parent_model->jointVariableCount());
    for (auto& p : RobotToParentVariablePairs) {
        small_state[p.second] = state[p.first];
    }
    return small_state;
}

static
auto FuseWithParent(
    const smpl::RobotState& full_state,
    const smpl::RobotState& parent_state)
    -> smpl::RobotState
{
    auto s = full_state;
    for (auto& p : RobotToParentVariablePairs) {
        s[p.first] = parent_state[p.second];
    }
    return s;
}

bool ObjectManipModel::checkJointLimits(
    const smpl::RobotState& state,
    bool verbose)
{
    // TODO: do we need to check bounds on object min and max position? we're
    // never going to generate an out-of-bounds index
    auto small_state = ExtractParentState(state, this->parent_model);
    auto ovar = state[HINGE];
    return parent_model->checkJointLimits(small_state, verbose) &
            (ovar >= this->min_positions[HINGE]) &
            (ovar <= this->max_positions[HINGE]);
}

auto ObjectManipModel::getExtension(size_t class_code)
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

auto ObjectManipModel::objectJointName() const -> const std::string&
{
    return this->getPlanningJoints()[HINGE];
}

auto ObjectManipModel::computeFK(const smpl::RobotState& state)
    -> Eigen::Affine3d
{
    // get the transformation from the world frame, dropping the z position and
    // roll and pitch orientation components
    auto small_state = ExtractParentState(state, this->parent_model);
    auto A = this->fk_iface->computeFK(small_state);

    // undo the contribution of the world -> robot transformation
    auto T_px_py_qz = smpl::Affine3(
            smpl::Translation3(state[WORLD_JOINT_X], state[WORLD_JOINT_Y], 0.0) *
            smpl::AngleAxis(state[WORLD_JOINT_YAW], smpl::Vector3::UnitZ()));
    auto Ap = smpl::Affine3(T_px_py_qz.inverse() * A);

    // apply our own world -> robot transformation
    return smpl::MakeAffine(
            state[WORLD_JOINT_X],
            state[WORLD_JOINT_Y],
            state[WORLD_JOINT_Z],
            state[WORLD_JOINT_YAW],
            state[WORLD_JOINT_PITCH],
            state[WORLD_JOINT_ROLL]) * Ap;
}

// TODO: Technically, we should fix up the input transform to remove the z,
// roll, and pitch components. However, we're really only calling IK AFAIK
// on valid states, which always have z = roll = pitch = 0, since demonstration
// states are fully known and not determined by IK.
bool ObjectManipModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& seed,
    smpl::RobotState& solution,
    smpl::ik_option::IkOption option)
{
    SMPL_DEBUG_STREAM_NAMED(R_LOG, "compute ik(seed = " << seed << ")");
    auto small_seed = ExtractParentState(seed, this->parent_model);
    if (!this->ik_iface->computeIK(pose, small_seed, solution, option)) {
        return false;
    }

    // add the object variable to the solution
    solution = FuseWithParent(seed, solution);
    return true;
}

bool ObjectManipModel::computeIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& seed,
    std::vector<smpl::RobotState>& solutions,
    smpl::ik_option::IkOption option)
{
    SMPL_DEBUG_STREAM_NAMED(R_LOG, "compute multi-ik(seed = " << seed << ")");
    auto small_seed = ExtractParentState(seed, this->parent_model);
    if (!this->ik_iface->computeIK(pose, small_seed, solutions, option)) {
        return false;
    }

    // add the object variable to all solutions
    for (auto& sol : solutions) {
        sol = FuseWithParent(seed, sol);
    }
    return true;
}

// the redundant manipulator interface will be a little different...
// we need to tell smpl that the object dimension is a 'free angle'
auto ObjectManipModel::redundantVariableCount() const -> const int
{
    // include z, roll, pitch, and object
    return this->rm_iface->redundantVariableCount() + 4;
}

auto ObjectManipModel::redundantVariableIndex(int rvidx) const
    -> const int
{
    // TODO: compute this map on Init
    int indices[] = {
        WORLD_JOINT_X,
        WORLD_JOINT_Y,
        WORLD_JOINT_Z,
        WORLD_JOINT_YAW,
        WORLD_JOINT_PITCH,
        WORLD_JOINT_ROLL,
        TORSO_JOINT1,
        LIMB_JOINT3,
        HINGE
    };
    return indices[rvidx];
#if 0
    if (rvidx < this->rm_iface->redundantVariableCount()) {
        return this->rm_iface->redundantVariableIndex(rvidx);
    } else {
        // the object variable is always the last variable in the robot state
        return this->parent_model->jointCount();
    }
#endif
}

bool ObjectManipModel::computeFastIK(
    const Eigen::Affine3d& pose,
    const smpl::RobotState& seed,
    smpl::RobotState& solution)
{
    SMPL_DEBUG_STREAM_NAMED(R_LOG, "compute fast-ik(seed = " << seed << ")");
    auto small_seed = ExtractParentState(seed, this->parent_model);
    if (!this->rm_iface->computeFastIK(pose, small_seed, solution)) {
        return false;
    }

    // add the object variable to all solutions
    solution = FuseWithParent(seed, solution);
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

    model->min_positions[WORLD_JOINT_X] = parent->minPosLimit(PMV_WORLD_JOINT_X);
    model->min_positions[WORLD_JOINT_Y] = parent->minPosLimit(PMV_WORLD_JOINT_Y);
    model->min_positions[WORLD_JOINT_Z] = -std::numeric_limits<double>::infinity();
    model->min_positions[WORLD_JOINT_YAW] = parent->minPosLimit(PMV_WORLD_JOINT_THETA);
    model->min_positions[WORLD_JOINT_PITCH] = -0.5 * M_PI;
    model->min_positions[WORLD_JOINT_ROLL] = -M_PI;
    model->min_positions[TORSO_JOINT1] = parent->minPosLimit(PMV_TORSO_JOINT1);
    model->min_positions[LIMB_JOINT1] = parent->minPosLimit(PMV_LIMB_JOINT1);
    model->min_positions[LIMB_JOINT2] = parent->minPosLimit(PMV_LIMB_JOINT2);
    model->min_positions[LIMB_JOINT3] = parent->minPosLimit(PMV_LIMB_JOINT3);
    model->min_positions[LIMB_JOINT4] = parent->minPosLimit(PMV_LIMB_JOINT4);
    model->min_positions[LIMB_JOINT5] = parent->minPosLimit(PMV_LIMB_JOINT5);
    model->min_positions[LIMB_JOINT6] = parent->minPosLimit(PMV_LIMB_JOINT6);
    model->min_positions[LIMB_JOINT7] = parent->minPosLimit(PMV_LIMB_JOINT7);
    model->min_positions[HINGE] = object_min;

    model->max_positions[WORLD_JOINT_X] = parent->maxPosLimit(PMV_WORLD_JOINT_X);
    model->max_positions[WORLD_JOINT_Y] = parent->maxPosLimit(PMV_WORLD_JOINT_Y);
    model->max_positions[WORLD_JOINT_Z] = std::numeric_limits<double>::infinity();
    model->max_positions[WORLD_JOINT_YAW] = parent->maxPosLimit(PMV_WORLD_JOINT_THETA);
    model->max_positions[WORLD_JOINT_PITCH] = 0.5 * M_PI;
    model->max_positions[WORLD_JOINT_ROLL] = M_PI;
    model->max_positions[TORSO_JOINT1] = parent->maxPosLimit(PMV_TORSO_JOINT1);
    model->max_positions[LIMB_JOINT1] = parent->maxPosLimit(PMV_LIMB_JOINT1);
    model->max_positions[LIMB_JOINT2] = parent->maxPosLimit(PMV_LIMB_JOINT2);
    model->max_positions[LIMB_JOINT3] = parent->maxPosLimit(PMV_LIMB_JOINT3);
    model->max_positions[LIMB_JOINT4] = parent->maxPosLimit(PMV_LIMB_JOINT4);
    model->max_positions[LIMB_JOINT5] = parent->maxPosLimit(PMV_LIMB_JOINT5);
    model->max_positions[LIMB_JOINT6] = parent->maxPosLimit(PMV_LIMB_JOINT6);
    model->max_positions[LIMB_JOINT7] = parent->maxPosLimit(PMV_LIMB_JOINT7);
    model->max_positions[HINGE] = object_max;

    model->pos_limited[WORLD_JOINT_X] = parent->hasPosLimit(PMV_WORLD_JOINT_X);
    model->pos_limited[WORLD_JOINT_Y] = parent->hasPosLimit(PMV_WORLD_JOINT_Y);
    model->pos_limited[WORLD_JOINT_Z] = false;
    model->pos_limited[WORLD_JOINT_YAW] = parent->hasPosLimit(PMV_WORLD_JOINT_THETA);
    model->pos_limited[WORLD_JOINT_PITCH] = true;
    model->pos_limited[WORLD_JOINT_ROLL] = true;
    model->pos_limited[TORSO_JOINT1] = parent->hasPosLimit(PMV_TORSO_JOINT1);
    model->pos_limited[LIMB_JOINT1] = parent->hasPosLimit(PMV_LIMB_JOINT1);
    model->pos_limited[LIMB_JOINT2] = parent->hasPosLimit(PMV_LIMB_JOINT2);
    model->pos_limited[LIMB_JOINT3] = parent->hasPosLimit(PMV_LIMB_JOINT3);
    model->pos_limited[LIMB_JOINT4] = parent->hasPosLimit(PMV_LIMB_JOINT4);
    model->pos_limited[LIMB_JOINT5] = parent->hasPosLimit(PMV_LIMB_JOINT5);
    model->pos_limited[LIMB_JOINT6] = parent->hasPosLimit(PMV_LIMB_JOINT6);
    model->pos_limited[LIMB_JOINT7] = parent->hasPosLimit(PMV_LIMB_JOINT7);
    model->pos_limited[HINGE] = true;

    model->continuous[WORLD_JOINT_X] = parent->isContinuous(PMV_WORLD_JOINT_X);
    model->continuous[WORLD_JOINT_Y] = parent->isContinuous(PMV_WORLD_JOINT_Y);
    model->continuous[WORLD_JOINT_Z] = false;
    model->continuous[WORLD_JOINT_YAW] = parent->isContinuous(PMV_WORLD_JOINT_THETA);
    model->continuous[WORLD_JOINT_PITCH] = false;
    model->continuous[WORLD_JOINT_ROLL] = false;
    model->continuous[TORSO_JOINT1] = parent->isContinuous(PMV_TORSO_JOINT1);
    model->continuous[LIMB_JOINT1] = parent->isContinuous(PMV_LIMB_JOINT1);
    model->continuous[LIMB_JOINT2] = parent->isContinuous(PMV_LIMB_JOINT2);
    model->continuous[LIMB_JOINT3] = parent->isContinuous(PMV_LIMB_JOINT3);
    model->continuous[LIMB_JOINT4] = parent->isContinuous(PMV_LIMB_JOINT4);
    model->continuous[LIMB_JOINT5] = parent->isContinuous(PMV_LIMB_JOINT5);
    model->continuous[LIMB_JOINT6] = parent->isContinuous(PMV_LIMB_JOINT6);
    model->continuous[LIMB_JOINT7] = parent->isContinuous(PMV_LIMB_JOINT7);
    model->continuous[HINGE] = false;

    model->vel_limits[WORLD_JOINT_X] = parent->velLimit(PMV_WORLD_JOINT_X);
    model->vel_limits[WORLD_JOINT_Y] = parent->velLimit(PMV_WORLD_JOINT_Y);
    model->vel_limits[WORLD_JOINT_Z] = 0.0;
    model->vel_limits[WORLD_JOINT_YAW] = parent->velLimit(PMV_WORLD_JOINT_THETA);
    model->vel_limits[WORLD_JOINT_PITCH] = 0.0;
    model->vel_limits[WORLD_JOINT_ROLL] = 0.0;
    model->vel_limits[TORSO_JOINT1] = parent->velLimit(PMV_TORSO_JOINT1);
    model->vel_limits[LIMB_JOINT1] = parent->velLimit(PMV_LIMB_JOINT1);
    model->vel_limits[LIMB_JOINT2] = parent->velLimit(PMV_LIMB_JOINT2);
    model->vel_limits[LIMB_JOINT3] = parent->velLimit(PMV_LIMB_JOINT3);
    model->vel_limits[LIMB_JOINT4] = parent->velLimit(PMV_LIMB_JOINT4);
    model->vel_limits[LIMB_JOINT5] = parent->velLimit(PMV_LIMB_JOINT5);
    model->vel_limits[LIMB_JOINT6] = parent->velLimit(PMV_LIMB_JOINT6);
    model->vel_limits[LIMB_JOINT7] = parent->velLimit(PMV_LIMB_JOINT7);
    model->vel_limits[HINGE] = 0.0;

    model->acc_limits[WORLD_JOINT_X] = parent->accLimit(PMV_WORLD_JOINT_X);
    model->acc_limits[WORLD_JOINT_Y] = parent->accLimit(PMV_WORLD_JOINT_Y);
    model->acc_limits[WORLD_JOINT_Z] = 0.0;
    model->acc_limits[WORLD_JOINT_YAW] = parent->accLimit(PMV_WORLD_JOINT_THETA);
    model->acc_limits[WORLD_JOINT_PITCH] = 0.0;
    model->acc_limits[WORLD_JOINT_ROLL] = 0.0;
    model->acc_limits[TORSO_JOINT1] = parent->accLimit(PMV_TORSO_JOINT1);
    model->acc_limits[LIMB_JOINT1] = parent->accLimit(PMV_LIMB_JOINT1);
    model->acc_limits[LIMB_JOINT2] = parent->accLimit(PMV_LIMB_JOINT2);
    model->acc_limits[LIMB_JOINT3] = parent->accLimit(PMV_LIMB_JOINT3);
    model->acc_limits[LIMB_JOINT4] = parent->accLimit(PMV_LIMB_JOINT4);
    model->acc_limits[LIMB_JOINT5] = parent->accLimit(PMV_LIMB_JOINT5);
    model->acc_limits[LIMB_JOINT6] = parent->accLimit(PMV_LIMB_JOINT6);
    model->acc_limits[LIMB_JOINT7] = parent->accLimit(PMV_LIMB_JOINT7);
    model->acc_limits[HINGE] = 0.0;

    auto variable_names = std::vector<std::string>(VARIABLE_COUNT);
    variable_names[WORLD_JOINT_X] = parent->getPlanningJoints()[PMV_WORLD_JOINT_X];
    variable_names[WORLD_JOINT_Y] = parent->getPlanningJoints()[PMV_WORLD_JOINT_Y];
    variable_names[WORLD_JOINT_Z] = "world_joint/z";
    variable_names[WORLD_JOINT_YAW] = parent->getPlanningJoints()[PMV_WORLD_JOINT_THETA];
    variable_names[WORLD_JOINT_PITCH] = "world_joint/pitch";
    variable_names[WORLD_JOINT_ROLL] = "world_joint/roll";
    variable_names[TORSO_JOINT1] = parent->getPlanningJoints()[PMV_TORSO_JOINT1];
    variable_names[LIMB_JOINT1] = parent->getPlanningJoints()[PMV_LIMB_JOINT1];
    variable_names[LIMB_JOINT2] = parent->getPlanningJoints()[PMV_LIMB_JOINT2];
    variable_names[LIMB_JOINT3] = parent->getPlanningJoints()[PMV_LIMB_JOINT3];
    variable_names[LIMB_JOINT4] = parent->getPlanningJoints()[PMV_LIMB_JOINT4];
    variable_names[LIMB_JOINT5] = parent->getPlanningJoints()[PMV_LIMB_JOINT5];
    variable_names[LIMB_JOINT6] = parent->getPlanningJoints()[PMV_LIMB_JOINT6];
    variable_names[LIMB_JOINT7] = parent->getPlanningJoints()[PMV_LIMB_JOINT7];
    variable_names[HINGE] = object_joint_name;

    model->setPlanningJoints(variable_names);
    return true;
}

