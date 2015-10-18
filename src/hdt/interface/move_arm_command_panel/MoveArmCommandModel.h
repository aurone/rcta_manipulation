#ifndef MoveArmCommandModel_h
#define MoveArmCommandModel_h

#include <string>

#include <QtGui>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

class MoveArmCommandModel : public QObject
{
    Q_OBJECT

public:

    MoveArmCommandModel(QObject* parent = 0);

    /// \brief Load a robot into the command model.
    ///
    /// If the robot model fails load from the given robot_description, the
    /// previous model remains and no robotLoaded signal is emitted.
    ///
    /// \param robot_description The name of the ROS parameter containing the
    ///     URDF and SRDF. The SRDF is derived from robot_description +
    ///     "_semantic".
    bool loadRobot(const std::string& robot_description);

    bool isRobotLoaded() const;

    const std::string& robotDescription() const;
    moveit::core::RobotModelConstPtr robotModel() const;
    moveit::core::RobotStateConstPtr robotState() const;

    std::map<std::string, double>
    getRightArmTorques(
        double fx, double fy, double fz,
        double ta, double tb, double tc) const;

public Q_SLOTS:

    void setJointVariable(int jidx, double value);

Q_SIGNALS:

    void robotLoaded();
    void robotStateChanged();

private:

    std::string m_robot_description;

    robot_model_loader::RobotModelLoaderPtr m_rm_loader;
    moveit::core::RobotModelPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    void logRobotModelInfo(const moveit::core::RobotModel& rm) const;
};

#endif
