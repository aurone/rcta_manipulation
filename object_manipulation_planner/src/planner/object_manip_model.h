#ifndef OBJECT_MANIPULATION_PLANNER_OBJECT_MANIPULATION_MODEL_H
#define OBJECT_MANIPULATION_PLANNER_OBJECT_MANIPULATION_MODEL_H

#include <smpl/robot_model.h>
#include "variables.h"

// A RobotModel that references an existing RobotModel and extends it by
// adding a single degree-of-freedom to represent an articulated object.
class ObjectManipModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface,
    public virtual smpl::InverseKinematicsInterface,
    public virtual smpl::RedundantManipulatorInterface
{
public:

    smpl::RobotModel* parent_model = NULL;

    // cache the underlying interfaces that we wrap
    smpl::ForwardKinematicsInterface* fk_iface = NULL;
    smpl::InverseKinematicsInterface* ik_iface = NULL;
    smpl::RedundantManipulatorInterface* rm_iface = NULL;

    double min_positions[VARIABLE_COUNT] = { };
    double max_positions[VARIABLE_COUNT] = { };
    bool   pos_limited[VARIABLE_COUNT] = { };
    bool   continuous[VARIABLE_COUNT] = { };
    double vel_limits[VARIABLE_COUNT] = { };
    double acc_limits[VARIABLE_COUNT] = { };

    auto objectJointName() const -> const std::string&;

    /// \name ForwardKinematicsInterface
    ///@{
    auto computeFK(const smpl::RobotState& state) -> Eigen::Affine3d override;
    ///@}

    /// \name InverseKinematicsInterface
    ///@{

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
    ///@}

    /// \name RedundantManipulatorInterface
    ///@{
    const int redundantVariableCount() const override;
    const int redundantVariableIndex(int rvidx) const override;

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& state,
        smpl::RobotState& solution) override;
    ///@}

    /// \name RobotModel Interface
    ///@{
    double minPosLimit(int vidx) const override;
    double maxPosLimit(int vidx) const override;
    bool hasPosLimit(int vidx) const override;
    bool isContinuous(int vidx) const override;
    double velLimit(int vidx) const override;
    double accLimit(int vidx) const override;

    bool checkJointLimits(
        const smpl::RobotState& state,
        bool verbose = false) override;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> smpl::Extension* override;
    ///@}
};

bool Init(
    ObjectManipModel* model,
    smpl::RobotModel* parent,
    const std::string& object_joint_name,
    double object_min,
    double object_max);

#endif
