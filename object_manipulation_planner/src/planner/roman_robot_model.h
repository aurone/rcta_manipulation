#ifndef OBJECT_MANIPULATION_PLANNER_ROMAN_ROBOT_MODEL_H
#define OBJECT_MANIPULATION_PLANNER_ROMAN_ROBOT_MODEL_H

// standard includes
#include <string>

// system includes
#include <smpl/robot_model.h>

class RomanRobotModel;

bool Init(RomanRobotModel* model, smpl::RobotModel* parent_model);

auto GetZVariableName(const RomanRobotModel* model) -> const std::string&;

// RobotModel that takes a z position variable onto the end of state vectors.
class RomanRobotModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface,
    public virtual smpl::InverseKinematicsInterface,
    public virtual smpl::RedundantManipulatorInterface
{
public:

    static const char* LOG;

    smpl::RobotModel* parent_model = NULL;

    smpl::ForwardKinematicsInterface* fk_iface = NULL;
    smpl::InverseKinematicsInterface* ik_iface;
    smpl::RedundantManipulatorInterface* rm_iface;

    /// \name ForwardKinematicsInterface
    ///@{
    auto computeFK(const smpl::RobotState& state) -> Eigen::Affine3d final;
    ///@}

    /// \name InverseKinematicsInterface
    ///@{

    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        smpl::RobotState& solution,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) final;

    bool computeIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& start,
        std::vector<smpl::RobotState>& solutions,
        smpl::ik_option::IkOption option = smpl::ik_option::UNRESTRICTED) final;
    ///@}

    /// \name RedundantManipulatorInterface
    ///@{
    auto redundantVariableCount() const -> const int final;
    auto redundantVariableIndex(int rvidx) const -> const int final;

    bool computeFastIK(
        const Eigen::Affine3d& pose,
        const smpl::RobotState& state,
        smpl::RobotState& solution) final;
    ///@}

    /// \name RobotModel Interface
    ///@{
    auto minPosLimit(int vidx) const -> double final;
    auto maxPosLimit(int vidx) const -> double final;
    bool hasPosLimit(int vidx) const final;
    bool isContinuous(int vidx) const final;
    auto velLimit(int vidx) const -> double final;
    auto accLimit(int vidx) const -> double final;

    bool checkJointLimits(const smpl::RobotState& state, bool verbose = false) final;
    ///@}

    /// \name Extension Interface
    ///@{
    auto getExtension(size_t class_code) -> smpl::Extension* final;
    ///@}
};

#endif

