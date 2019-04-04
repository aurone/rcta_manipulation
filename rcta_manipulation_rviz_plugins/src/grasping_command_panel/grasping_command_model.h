#ifndef RCTA_MANIPULATION_RVIZ_PLUGINS_GRASPING_COMMAND_MODEL_H
#define RCTA_MANIPULATION_RVIZ_PLUGINS_GRASPING_COMMAND_MODEL_H

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <QObject>
#ifndef Q_MOC_RUN
#include <actionlib/client/simple_action_client.h>
#include <cmu_manipulation_msgs/GraspObjectCommandAction.h>
#include <cmu_manipulation_msgs/ManipulateObjectAction.h>
#include <cmu_manipulation_msgs/RepositionBaseCommandAction.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <rviz/config.h>
#include <tf/transform_listener.h>
#endif

namespace rcta {

using GraspObjectCommandActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::GraspObjectCommandAction>;
using RepositionBaseCommandActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::RepositionBaseCommandAction>;
using ManipulateObjectActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::ManipulateObjectAction>;

// Maintains the state of an object manipulation command to be sent and the
// results of any previous commands sent to the robot
class GraspingCommandModel : public QObject
{
    Q_OBJECT

public:

    ros::NodeHandle m_nh;
    ros::Subscriber m_occupancy_grid_sub;
    tf::TransformListener m_listener;

    std::unique_ptr<GraspObjectCommandActionClient> m_grasp_object_command_client;
    std::unique_ptr<RepositionBaseCommandActionClient> m_reposition_base_command_client;
    std::unique_ptr<ManipulateObjectActionClient> m_manipulate_object_client;

    /// \name Robot Model
    ///@{

    // We need a robot for a few things. In general, object manipulation
    // commands accept a pose for the object in the planning frame, as
    // determined by RobotModelLoader. To know what that frame is, we have to
    // load the robot.
    std::string m_robot_description;
    robot_model_loader::RobotModelLoaderPtr m_rml;
    robot_model::RobotModelPtr m_robot_model;

    ///@}

    /// \name Object Model
    ///@{

    std::string m_obj_mesh_resource;
    double m_obj_scale_x;
    double m_obj_scale_y;
    double m_obj_scale_z;

    ///@}

    /// \name Robot State
    ///@{

    std::unique_ptr<robot_state::RobotState> m_robot_state;

    // TODO: can this be maintained via the transform of the root joint or is
    // there a reason this is being maintained externally?
    Eigen::Affine3d m_T_world_robot;

    ///@}

    /// \name Object State
    ///@{

    // shared state of object, for all object manipulation commands
    Eigen::Affine3d m_T_world_object;

    ///@}

    // current RepositionBase command
    nav_msgs::OccupancyGrid::ConstPtr m_occupancy_grid;

    // current ManipulateObject command
    std::string m_object_id;
    double m_obj_start;
    double m_obj_goal;
    double m_allowed_planning_time;
    bool m_plan_only;

    // results of previous RepositionBaseCommand
    int m_base_candidate_idx;
    std::vector<geometry_msgs::PoseStamped> m_candidate_base_poses;

    GraspingCommandModel();

    bool loadRobot(const std::string& urdf_param, std::string* why = NULL);
    bool robotLoaded() const;

    void load(const rviz::Config& config);
    void save(rviz::Config& config) const;

    void repositionBaseCommandResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const cmu_manipulation_msgs::RepositionBaseCommandResult::ConstPtr& result);

    void manipulateObjectCommandResultCallback(
            const actionlib::SimpleClientGoalState& state,
            const cmu_manipulation_msgs::ManipulateObjectResult::ConstPtr& result);

    void graspObjectCommandResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const cmu_manipulation_msgs::GraspObjectCommandResult::ConstPtr& result);

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

public Q_SLOTS:

    void setObjectMeshResource(const std::string& path);
    void setObjectMeshScaleX(double x);
    void setObjectMeshScaleY(double y);
    void setObjectMeshScaleZ(double z);

    void setRobotPoseToCurrent();
    void setRobotPoseX(double val);
    void setRobotPoseY(double val);
    void setRobotPoseYaw(double val);

    void setObjectPoseX(double val);
    void setObjectPoseY(double val);
    void setObjectPoseZ(double val);
    void setObjectPoseYaw(double val);
    void setObjectPosePitch(double val);
    void setObjectPoseRoll(double val);

    void sendRepositionBaseCommand();

    void setAllowedPlanningTime(double val);
    void setObjectStart(double val);
    void setObjectGoal(double val);
    void setPlanOnly(bool plan_only);
    void sendManipulateObjectCommand();

    void sendGraspObjectCommand();

Q_SIGNALS:

    void robotLoaded(const QString&);

    void objectMeshResourceChanged(const QString&);
    void objectMeshScaleChanged(double x, double y, double z);

    void robotStateChanged();

    void objectPoseXChanged(double val);
    void objectPoseYChanged(double val);
    void objectPoseZChanged(double val);
    void objectPoseYawChanged(double val);
    void objectPosePitchChanged(double val);
    void objectPoseRollChanged(double val);

    void sentGraspObjectCommand();
    void sentRepositionBaseCommand();
    void sentManipulateObjectCommand();
    void receivedGraspObjectResult();
    void receivedRepositionBaseResult();
    void receivedManipulateObjectResult();

    void objectStartChanged(double val);
    void objectGoalChanged(double val);
    void allowedPlanningTimeChanged(double val);
    void planOnlyChanged(bool val);

    void errorEncountered(const QString&);
};

} // namespace rcta

#endif
