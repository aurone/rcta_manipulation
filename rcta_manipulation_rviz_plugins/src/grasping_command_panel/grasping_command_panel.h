#ifndef RCTA_MANIPULATION_RVIZ_PLUGINS_GRASPING_COMMAND_PANEL_H
#define RCTA_MANIPULATION_RVIZ_PLUGINS_GRASPING_COMMAND_PANEL_H

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <rviz/panel.h>
#ifndef Q_MOC_RUN
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#endif

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QCheckBox;

namespace rcta {

class GraspingCommandModel;

class GraspingCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    GraspingCommandPanel(QWidget *parent = 0);
    ~GraspingCommandPanel();

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    void refreshRobotDescription();
    void refreshObjectMeshResource();

    void notifyRobotLoaded(const QString&);
    void updateRobotState();

    void updateBasePoseYaw(double yaw);
    void updateBasePoseCandidate(int index);

    // interpret values in line edit and pass to model
    void updateMeshScaleX();
    void updateMeshScaleY();
    void updateMeshScaleZ();

    void updateObjectPoseYawDegs(double val);
    void updateObjectPosePitchDegs(double val);
    void updateObjectPoseRollDegs(double val);

    void setObjectPoseX(double val);
    void setObjectPoseY(double val);
    void setObjectPoseZ(double val);
    void setObjectPoseYaw(double val);
    void setObjectPosePitch(double val);
    void setObjectPoseRoll(double val);

    // hooks to reinitialize interactive marker
    void updateObjectMeshResourceChanged(const QString& path);
    void updateMeshScale(double x, double y, double z);

    // Signals to grab the current text and pass values to the model
    void updateObjectStart();
    void updateObjectGoal();

    void updateObjectStart(double val);
    void updateObjectGoal(double val);

    void displayErrorMessageBox(const QString& s);

    void setPlanOnlyFromCheckbox(int state);
    void updatePlanOnly(bool plan_only);

Q_SIGNALS:

private:

    ros::NodeHandle m_nh;

    /// \name Interface
    ///@{

    // Global Settings Widgets
    QLineEdit*      m_robot_description_line_edit = NULL;
    QPushButton*    m_refresh_robot_desc_button = NULL;

    // Object controls
    QDoubleSpinBox* m_obj_pose_x_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_y_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_z_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_Y_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_P_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_R_spinbox = NULL;
    QLineEdit*      m_obj_mesh_resource_line_edit = NULL;

    // Object view
    QPushButton*    m_refresh_obj_mesh_resource_button = NULL;

    QLineEdit*      m_obj_mesh_scale_x_line_edit = NULL;
    QLineEdit*      m_obj_mesh_scale_y_line_edit = NULL;
    QLineEdit*      m_obj_mesh_scale_z_line_edit = NULL;

    // Reposition Base Command Widgets
    QPushButton*    m_copy_current_base_pose_button = NULL;
    QDoubleSpinBox* m_teleport_base_command_x_box = NULL;
    QDoubleSpinBox* m_teleport_base_command_y_box = NULL;
    QDoubleSpinBox* m_teleport_base_command_yaw_box = NULL;
    QPushButton*    m_send_reposition_base_command_button = NULL;
    QSpinBox*       m_update_candidate_spinbox = NULL;
    QLabel*         m_num_candidates_label = NULL;

    // Manipulate Object Command Widgets
    QPushButton*    m_send_manipulate_object_button = NULL;
    QLineEdit*      m_object_start_line_edit = NULL;
    QLineEdit*      m_object_goal_line_edit = NULL;
    QDoubleSpinBox* m_allowed_planning_time_spinbox = NULL;
    QCheckBox*      m_plan_only_checkbox = NULL;

    // Grasp Object Command Widgets
    QPushButton*    m_send_grasp_object_command_button = NULL;

    ros::Publisher m_marker_pub;

    interactive_markers::InteractiveMarkerServer m_server;
    std::string m_gascan_interactive_marker_name = "gas_canister_fixture";

    ///@}

    /// \name Model
    ///@{
    std::unique_ptr<GraspingCommandModel> m_model;
    ///@}

    // The set utilities have the effect of changing the text in the line edit
    // box as well as applying the same action as the corresponding refresh
    // button; the get utilities retrieve the last 'refreshed' value
    bool setRobotDescription(const std::string& robot_description, std::string& why);

    void processObjectMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);

    bool updateObjectInteractiveMarker();
    void updateObjectMarkerPose();
    void updateGUI();
};

} // namespace rcta

#endif
