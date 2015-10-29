#ifndef MoveArmCommandModel_h
#define MoveArmCommandModel_h

#include <string>

#include <QtGui>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetMotionPlan.h>

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

    bool readyToPlan() const;
    bool planToPosition(const std::string& group_name);

    bool copyCurrentState();

public Q_SLOTS:

    void setJointVariable(int jidx, double value);

Q_SIGNALS:

    void robotLoaded();
    void robotStateChanged();
    void readyStatusChanged();

private:

    ros::NodeHandle m_nh;
    ros::Subscriber m_joint_states_sub;
    ros::ServiceClient m_plan_path_client;

    sensor_msgs::JointState::ConstPtr m_last_joint_state_msg;

    std::string m_robot_description;

    robot_model_loader::RobotModelLoaderPtr m_rm_loader;
    moveit::core::RobotModelPtr m_robot_model;
    moveit::core::RobotStatePtr m_robot_state;

    ros::Publisher m_collision_object_pub;

    void logRobotModelInfo(const moveit::core::RobotModel& rm) const;

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void clearMoveGroupRequest();

    bool fillWorkspaceParameters(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req);
    bool fillStartState(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillGoalConstraints(
        const ros::Time& now,
        const std::string& group,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillPathConstraints(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;
    bool fillTrajectoryConstraints(
        const ros::Time& now,
        const std::string& group_name,
        moveit_msgs::MotionPlanRequest& req) const;
};

#endif
