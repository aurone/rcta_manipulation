#ifndef OBJECT_MANIPULATION_PLANNER_OBJECT_MANIPULATION_MODEL_H
#define OBJECT_MANIPULATION_PLANNER_OBJECT_MANIPULATION_MODEL_H

#include <smpl/robot_model.h>

// A RobotModel that references an existing RobotModel and extends it by
// adding a single degree-of-freedom to represent an articulated object.
struct ObjectManipulationModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface,
    public virtual smpl::InverseKinematicsInterface,
    public virtual smpl::RedundantManipulatorInterface
{
    smpl::RobotModel* parent_model = NULL;

    // cache the underlying interfaces that we wrap
    smpl::ForwardKinematicsInterface* fk_iface = NULL;
    smpl::InverseKinematicsInterface* ik_iface = NULL;
    smpl::RedundantManipulatorInterface* rm_iface = NULL;

    double min_object_pos = 0.0;
    double max_object_pos = 0.0;

    double minPosLimit(int vidx) const override;
    double maxPosLimit(int vidx) const override;
    bool hasPosLimit(int vidx) const override;
    bool isContinuous(int vidx) const override;
    double velLimit(int vidx) const override;
    double accLimit(int vidx) const override;

    bool checkJointLimits(
        const smpl::RobotState& state,
        bool verbose = false) override;

    auto getExtension(size_t class_code) -> smpl::Extension* override;

    // we should just be able to use the underlying robot model's forward
    // kinematics
    auto computeFK(const smpl::RobotState& state) -> Eigen::Affine3d override;

    // same for ik...
    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) override;

    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        std::vector<smpl::RobotState>& solutions,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) override;

    // the redundant manipulator interface will be a little different...
    // we need to tell smpl that the object dimension is a 'free angle'
    const int redundantVariableCount() const override;
    const int redundantVariableIndex(int rvidx) const override;

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& state,
        smpl::RobotState& solution);
};

bool Init(
    ObjectManipulationModel* model,
    smpl::RobotModel* parent,
    double object_min,
    double object_max);

#endif
