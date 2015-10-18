#include "MoveArmCommandModel.h"

#include <ros/console.h>

MoveArmCommandModel::MoveArmCommandModel(QObject* parent) :
    QObject(parent),
    m_robot_description(),
    m_rm_loader(),
    m_robot_model(),
    m_robot_state()
{
}

bool MoveArmCommandModel::loadRobot(const std::string& robot_description)
{
    if (robot_description == m_robot_description) {
        return true;
    }

    // load a new robot
    robot_model_loader::RobotModelLoaderPtr rm_loader(
            new robot_model_loader::RobotModelLoader(robot_description, true));

    if (!rm_loader->getModel()) {
        // failed to load robot from URDF
        return false;
    }

    m_robot_description = robot_description;
    m_rm_loader = rm_loader;
    m_robot_model = m_rm_loader->getModel();
    m_robot_state.reset(new moveit::core::RobotState(m_robot_model));

    m_robot_state->setToDefaultValues();
    m_robot_state->updateLinkTransforms();
    m_robot_state->updateCollisionBodyTransforms();

    Q_EMIT robotLoaded();
    return true;
}

bool MoveArmCommandModel::isRobotLoaded() const
{
    return (bool)m_robot_model.get();
}

const std::string& MoveArmCommandModel::robotDescription() const
{
    return m_robot_description;
}

moveit::core::RobotModelConstPtr MoveArmCommandModel::robotModel() const
{
    return m_robot_model;
}

moveit::core::RobotStateConstPtr MoveArmCommandModel::robotState() const
{
    return m_robot_state;
}

void MoveArmCommandModel::setJointVariable(int jidx, double value)
{
    if (!isRobotLoaded()) {
        return;
    }

    if (jidx < 0 || jidx >= m_robot_model->getVariableCount()) {
        ROS_WARN("Index passed to setJointVariable out of bounds: jidx = %d, variable count = %zu", jidx, m_robot_model->getVariableCount());
        return;
    }

    if (m_robot_state->getVariablePosition(jidx) != value) {
        m_robot_state->setVariablePosition(jidx, value);
        if (m_robot_state->getVariablePosition(jidx) != value) {
            ROS_WARN("Attempt to set joint variable %d to %0.3f failed", jidx, value);
        }
        m_robot_state->updateLinkTransforms();
        m_robot_state->updateCollisionBodyTransforms();
        Q_EMIT robotStateChanged();
    }
}
