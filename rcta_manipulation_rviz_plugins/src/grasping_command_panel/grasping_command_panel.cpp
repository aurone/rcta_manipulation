#include "grasping_command_panel.h"

// standard includes
#include <cassert>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>

// system includes
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QGridLayout>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/TransformStamped.h>
#include <rcta_manipulation_common/comms/actionlib.h>
#include <robotiq_controllers/gripper_model.h>
#include <smpl/angles.h>
#include <sensor_msgs/JointState.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/std_msgs/std_msgs.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/utils/utils.h>
#include <std_msgs/ColorRGBA.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#define USE_MANIPULATE_ACTION 0

namespace rcta {

static const char* LOG = "grasping_command_panel";

GraspingCommandPanel::GraspingCommandPanel(QWidget *parent) :
    rviz::Panel(parent),
    m_server("grasping_commands")
{
    ///////////////////
    // Construct GUI //
    ///////////////////

    auto* parent_layout = new QVBoxLayout;
    auto* scroll_area = new QScrollArea;
    auto* scroll_area_widget = new QWidget;
    auto* main_layout = new QVBoxLayout;

    // general settings
    auto* general_settings_group = new QGroupBox(tr("General Settings"));
    auto* general_settings_layout = new QVBoxLayout;

    auto* robot_description_layout = new QHBoxLayout;
    auto* robot_description_label = new QLabel(tr("Robot Description:"));
    m_robot_description_line_edit = new QLineEdit;
    m_refresh_robot_desc_button = new QPushButton(tr("Refresh"));
    robot_description_layout->addWidget(robot_description_label);
    robot_description_layout->addWidget(m_robot_description_line_edit);
    robot_description_layout->addWidget(m_refresh_robot_desc_button);

    auto* global_frame_layout = new QHBoxLayout;
    auto* global_frame_label = new QLabel(tr("Global Frame:"));
    m_global_frame_line_edit = new QLineEdit;
    m_global_frame_line_edit->setEnabled(false);
    m_refresh_global_frame_button = new QPushButton(tr("Refresh"));
    global_frame_layout->addWidget(global_frame_label);
    global_frame_layout->addWidget(m_global_frame_line_edit);
    global_frame_layout->addWidget(m_refresh_global_frame_button);

    auto* obj_mesh_resource_layout = new QHBoxLayout;
    auto* obj_mesh_resource_label = new QLabel(tr("Object Mesh Resource:"));
    m_obj_mesh_resource_line_edit = new QLineEdit;
    m_refresh_obj_mesh_resource_button = new QPushButton(tr("Refresh"));

    m_obj_mesh_scale_x_line_edit = new QLineEdit;
    m_obj_mesh_scale_y_line_edit = new QLineEdit;
    m_obj_mesh_scale_z_line_edit = new QLineEdit;

    obj_mesh_resource_layout->addWidget(obj_mesh_resource_label);
    obj_mesh_resource_layout->addWidget(m_obj_mesh_resource_line_edit);
    obj_mesh_resource_layout->addWidget(m_refresh_obj_mesh_resource_button);
    obj_mesh_resource_layout->addWidget(m_obj_mesh_scale_x_line_edit);
    obj_mesh_resource_layout->addWidget(m_obj_mesh_scale_y_line_edit);
    obj_mesh_resource_layout->addWidget(m_obj_mesh_scale_z_line_edit);

    auto* obj_pose_layout = new QGridLayout;
    m_obj_pose_x_spinbox = new QDoubleSpinBox;
    m_obj_pose_y_spinbox = new QDoubleSpinBox;
    m_obj_pose_z_spinbox = new QDoubleSpinBox;
    m_obj_pose_Y_spinbox = new QDoubleSpinBox;
    m_obj_pose_P_spinbox = new QDoubleSpinBox;
    m_obj_pose_R_spinbox = new QDoubleSpinBox;

    m_obj_pose_x_spinbox->setMinimum(-100.0);
    m_obj_pose_y_spinbox->setMinimum(-100.0);
    m_obj_pose_z_spinbox->setMinimum(-100.0);
    m_obj_pose_x_spinbox->setMaximum(100.0);
    m_obj_pose_y_spinbox->setMaximum(100.0);
    m_obj_pose_z_spinbox->setMaximum(100.0);
    m_obj_pose_x_spinbox->setSingleStep(0.01);
    m_obj_pose_y_spinbox->setSingleStep(0.01);
    m_obj_pose_z_spinbox->setSingleStep(0.01);

    m_obj_pose_Y_spinbox->setMaximum(359.0);
    m_obj_pose_P_spinbox->setMinimum(-90.0);
    m_obj_pose_P_spinbox->setMaximum(90.0);
    m_obj_pose_R_spinbox->setMaximum(359.0);
    obj_pose_layout->addWidget(m_obj_pose_x_spinbox, 0, 0);
    obj_pose_layout->addWidget(m_obj_pose_y_spinbox, 0, 1);
    obj_pose_layout->addWidget(m_obj_pose_z_spinbox, 0, 2);
    obj_pose_layout->addWidget(m_obj_pose_Y_spinbox, 1, 0);
    obj_pose_layout->addWidget(m_obj_pose_P_spinbox, 1, 1);
    obj_pose_layout->addWidget(m_obj_pose_R_spinbox, 1, 2);

    general_settings_layout->addLayout(robot_description_layout);
    general_settings_layout->addLayout(global_frame_layout);
    general_settings_layout->addLayout(obj_mesh_resource_layout);
    general_settings_layout->addLayout(obj_pose_layout);
    general_settings_group->setLayout(general_settings_layout);

    // reposition planner command

    auto* reposition_planner_group = new QGroupBox(tr("Reposition Base"));

    auto* reposition_planner_group_layout = new QVBoxLayout;

    m_copy_current_base_pose_button = new QPushButton(tr("Copy Current Base Pose"));

    auto* base_pose_spinbox_layout = new QHBoxLayout;
    auto* x_label = new QLabel(tr("X:"));
    m_teleport_base_command_x_box = new QDoubleSpinBox;
    m_teleport_base_command_x_box->setMinimum(-100.0);
    m_teleport_base_command_x_box->setMaximum(100.0);
    m_teleport_base_command_x_box->setSingleStep(0.05);
    auto* y_label = new QLabel(tr("Y:"));
    m_teleport_base_command_y_box = new QDoubleSpinBox;
    m_teleport_base_command_y_box->setMinimum(-100.0);
    m_teleport_base_command_y_box->setMaximum(100.0);
    m_teleport_base_command_y_box->setSingleStep(0.05);
    auto* z_label = new QLabel(tr("Z:"));
    m_teleport_base_command_z_box = new QDoubleSpinBox;
    auto* yaw_label = new QLabel(tr("Yaw:"));
    m_teleport_base_command_yaw_box = new QDoubleSpinBox;
    m_teleport_base_command_yaw_box->setMinimum(0.0);
    m_teleport_base_command_yaw_box->setMaximum(359.0);
    m_teleport_base_command_yaw_box->setSingleStep(1.0);
    m_teleport_base_command_yaw_box->setWrapping(true);
    base_pose_spinbox_layout->addWidget(x_label);
    base_pose_spinbox_layout->addWidget(m_teleport_base_command_x_box);
    base_pose_spinbox_layout->addWidget(y_label);
    base_pose_spinbox_layout->addWidget(m_teleport_base_command_y_box);
    base_pose_spinbox_layout->addWidget(yaw_label);
    base_pose_spinbox_layout->addWidget(m_teleport_base_command_yaw_box);

    send_reposition_base_command_button_ = new QPushButton(tr("Reposition Base"));

    auto* candidates_layout = new QHBoxLayout;
    update_candidate_spinbox_ = new QSpinBox;
    update_candidate_spinbox_->setEnabled(false);
    num_candidates_label_ = new QLabel(tr("of 0 Candidates"));
    candidates_layout->addWidget(update_candidate_spinbox_);
    candidates_layout->addWidget(num_candidates_label_);

    reposition_planner_group_layout->addWidget(m_copy_current_base_pose_button);
    reposition_planner_group_layout->addLayout(base_pose_spinbox_layout);
    reposition_planner_group_layout->addWidget(send_reposition_base_command_button_);
    reposition_planner_group_layout->addLayout(candidates_layout);

    reposition_planner_group->setLayout(reposition_planner_group_layout);

    // manipulate object planner command

    auto* manip_object_command_group = new QGroupBox(tr("Manipulate Object"));

    auto* manip_object_command_layout = new QVBoxLayout;

    send_manipulate_object_command_button_ = new QPushButton(tr("Manipulate Object"));

    m_allowed_planning_time_spinbox = new QDoubleSpinBox;
    m_allowed_planning_time_spinbox->setMinimum(0.0);

    auto* allowed_planning_time_layout = new QHBoxLayout;
    allowed_planning_time_layout->addWidget(new QLabel(tr("Allowed Planning Time:")));
    allowed_planning_time_layout->addWidget(m_allowed_planning_time_spinbox);

    auto* manip_object_settings_layout = new QHBoxLayout;
    m_object_start_line_edit = new QLineEdit;
    m_object_goal_line_edit = new QLineEdit;
    manip_object_settings_layout->addWidget(new QLabel(tr("Object Start:")));
    manip_object_settings_layout->addWidget(m_object_start_line_edit);
    manip_object_settings_layout->addWidget(new QLabel(tr("Object Goal:")));
    manip_object_settings_layout->addWidget(m_object_goal_line_edit);
    manip_object_command_layout->addWidget(send_manipulate_object_command_button_);
    manip_object_command_layout->addLayout(manip_object_settings_layout);
    manip_object_command_layout->addLayout(allowed_planning_time_layout);

    manip_object_command_group->setLayout(manip_object_command_layout);

    // grasping object executor

    auto* grasp_object_command_group = new QGroupBox(tr("Grasp Object"));

    auto* grasp_object_command_layout = new QVBoxLayout;

    send_grasp_object_command_button_ = new QPushButton(tr("Grasp Object"));

    grasp_object_command_layout->addWidget(send_grasp_object_command_button_);

    grasp_object_command_group->setLayout(grasp_object_command_layout);

    // Build the main layout

    main_layout->addWidget(general_settings_group);
    main_layout->addWidget(reposition_planner_group);
    main_layout->addWidget(manip_object_command_group);
    main_layout->addWidget(grasp_object_command_group);
    main_layout->addStretch();

    scroll_area_widget->setLayout(main_layout);
    scroll_area->setWidget(scroll_area_widget);
    scroll_area->setWidgetResizable(true);
    parent_layout->addWidget(scroll_area);
    setLayout(parent_layout);

    //////////////////////////
    // Connect GUI to Model //
    //////////////////////////

    // note: do not connect any outgoing signals from general settings line edits; force users to use refresh button
    connect(m_refresh_robot_desc_button, SIGNAL(clicked()), this, SLOT(refreshRobotDescription()));
    connect(m_refresh_global_frame_button, SIGNAL(clicked()), this, SLOT(refreshGlobalFrame()));
    connect(m_refresh_obj_mesh_resource_button, SIGNAL(clicked()), this, SLOT(refreshObjectMeshResource()));

    // base commands
    connect(m_copy_current_base_pose_button, SIGNAL(clicked()), this, SLOT(copyCurrentBasePose()));
    connect(m_teleport_base_command_x_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseX(double)));
    connect(m_teleport_base_command_y_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseY(double)));
    connect(m_teleport_base_command_z_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseZ(double)));
    connect(m_teleport_base_command_yaw_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseYaw(double)));

    connect(m_allowed_planning_time_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateAllowedPlanningTime(double)));

    // object interaction commands
    connect(send_grasp_object_command_button_, SIGNAL(clicked()), this, SLOT(sendGraspObjectCommand()));
    connect(send_reposition_base_command_button_, SIGNAL(clicked()), this, SLOT(sendRepositionBaseCommand()));
    connect(send_manipulate_object_command_button_, SIGNAL(clicked()), this, SLOT(sendManipulateObjectCommand()));
    connect(update_candidate_spinbox_, SIGNAL(valueChanged(int)), this, SLOT(updateBasePoseCandidate(int)));

    connect(m_obj_mesh_scale_x_line_edit, SIGNAL(editingFinished()), this, SLOT(updateMeshScaleX()));
    connect(m_obj_mesh_scale_y_line_edit, SIGNAL(editingFinished()), this, SLOT(updateMeshScaleY()));
    connect(m_obj_mesh_scale_z_line_edit, SIGNAL(editingFinished()), this, SLOT(updateMeshScaleZ()));

#if 1
    connect(m_obj_pose_x_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateObjectPoseX(double)));
    connect(m_obj_pose_y_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateObjectPoseY(double)));
    connect(m_obj_pose_z_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateObjectPoseZ(double)));
    connect(m_obj_pose_Y_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateObjectPoseYaw(double)));
    connect(m_obj_pose_P_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateObjectPosePitch(double)));
    connect(m_obj_pose_R_spinbox, SIGNAL(valueChanged(double)), this, SLOT(updateObjectPoseRoll(double)));

    connect(this, SIGNAL(objectPoseXUpdated(double)),       m_obj_pose_x_spinbox, SLOT(setValue(double)));
    connect(this, SIGNAL(objectPoseYUpdated(double)),       m_obj_pose_y_spinbox, SLOT(setValue(double)));
    connect(this, SIGNAL(objectPoseZUpdated(double)),       m_obj_pose_z_spinbox, SLOT(setValue(double)));
    connect(this, SIGNAL(objectPoseYawUpdated(double)),     m_obj_pose_Y_spinbox, SLOT(setValue(double)));
    connect(this, SIGNAL(objectPosePitchUpdated(double)),   m_obj_pose_P_spinbox, SLOT(setValue(double)));
    connect(this, SIGNAL(objectPoseRollUpdated(double)),    m_obj_pose_R_spinbox, SLOT(setValue(double)));
#endif

    connect(m_object_start_line_edit, SIGNAL(editingFinished()), this, SLOT(updateObjectStart()));
    connect(m_object_goal_line_edit, SIGNAL(editingFinished()), this, SLOT(updateObjectGoal()));

    //////////////////////////
    // Initialize the Model //
    //////////////////////////

    m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 1);
    m_occupancy_grid_sub = m_nh.subscribe(
            "map", 1, &GraspingCommandPanel::occupancyGridCallback, this);
}

GraspingCommandPanel::~GraspingCommandPanel()
{
}

void GraspingCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);

    ROS_DEBUG_NAMED(LOG, "Loading config for '%s'", this->getName().toStdString().c_str());

    auto robot_description = QString();
    auto global_frame = QString();
    auto obj_mesh_resource = QString();
    auto obj_scale_x = 0.0f;
    auto obj_scale_y = 0.0f;
    auto obj_scale_z = 0.0f;
    auto base_x = 0.0f;
    auto base_y = 0.0f;
    auto base_yaw = 0.0f;
    auto object_x = 0.0f;
    auto object_y = 0.0f;
    auto object_z = 0.0f;
    auto object_yaw = 0.0f;
    auto object_pitch = 0.0f;
    auto object_roll = 0.0f;
    auto object_start = 0.0f;
    auto object_goal = 1.0f;
    config.mapGetString("robot_description", &robot_description);
    config.mapGetString("global_frame", &global_frame);
    config.mapGetString("object_mesh_resource", &obj_mesh_resource);
    config.mapGetFloat("object_scale_x", &obj_scale_x);
    config.mapGetFloat("object_scale_y", &obj_scale_y);
    config.mapGetFloat("object_scale_z", &obj_scale_z);
    config.mapGetFloat("base_x", &base_x);
    config.mapGetFloat("base_y", &base_y);
    config.mapGetFloat("base_yaw", &base_yaw);
    config.mapGetFloat("object_x", &object_x);
    config.mapGetFloat("object_y", &object_y);
    config.mapGetFloat("object_z", &object_z);
    config.mapGetFloat("object_yaw", &object_yaw);
    config.mapGetFloat("object_pitch", &object_pitch);
    config.mapGetFloat("object_roll", &object_roll);
    config.mapGetFloat("object_start", &object_start);
    config.mapGetFloat("object_goal", &object_goal);

    ROS_DEBUG_NAMED(LOG, "Robot Description: %s", robot_description.toStdString().c_str());
    ROS_DEBUG_NAMED(LOG, "Global Frame: %s", global_frame.toStdString().c_str());
    ROS_DEBUG_NAMED(LOG, "Object Mesh Resource: %s", obj_mesh_resource.toStdString().c_str());

    m_obj_mesh_resource = obj_mesh_resource.toStdString();
    updateMeshScaleX(obj_scale_x);
    updateMeshScaleY(obj_scale_y);
    updateMeshScaleZ(obj_scale_z);

    m_obj_start = object_start;
    m_obj_goal = object_goal;

    // note: set the robot description before the global frame so we don't flag
    // virtual joint parent frames as unacceptable
    if (!robot_description.isEmpty()) {
        // attempt to initalize the robot using this robot description
        std::string why;
        if (!setRobotDescription(robot_description.toStdString(), why)) {
            QMessageBox::warning(
                    this,
                    tr("Config Failure"),
                    tr("Failed to load 'robot_description' from panel config (%1)").arg(QString::fromStdString(why)));
        }
    }

    if (!global_frame.isEmpty()) {
        // attempt to set the global frame
        std::string why;
        if (!setGlobalFrame(global_frame.toStdString(), why)) {
            QMessageBox::warning(
                    this,
                    tr("Config Failure"),
                    tr("Failed to load 'global_frame' from panel config (%1)").arg(QString::fromStdString(why)));
        }
    }

    m_T_world_robot =
            Eigen::Translation3d(base_x, base_y, 0.0) *
            Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ());

    updateBasePoseSpinBoxes();

    updateObjectPoseX(object_x);
    updateObjectPoseY(object_y);
    updateObjectPoseZ(object_z);
    updateObjectPoseYaw(object_yaw);
    updateObjectPosePitch(object_pitch);
    updateObjectPoseRoll(object_roll);

    updateGUI();
}

void GraspingCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    ROS_DEBUG_NAMED(LOG, "Saving config for '%s'", this->getName().toStdString().c_str());

    config.mapSetValue("robot_description", QString::fromStdString(robot_description_));
    config.mapSetValue("global_frame", QString::fromStdString(global_frame_));
    config.mapSetValue("object_mesh_resource", QString::fromStdString(m_obj_mesh_resource));

    config.mapSetValue("object_scale_x", m_obj_scale_x);
    config.mapSetValue("object_scale_y", m_obj_scale_y);
    config.mapSetValue("object_scale_z", m_obj_scale_z);

    config.mapSetValue("base_x", m_T_world_robot.translation()(0, 0));
    config.mapSetValue("base_y", m_T_world_robot.translation()(1, 0));
    double yaw, pitch, roll;
    msg_utils::get_euler_ypr(m_T_world_robot, yaw, pitch, roll);
    config.mapSetValue("base_yaw", yaw);

    config.mapSetValue("object_x", m_T_world_object.translation()(0, 0));
    config.mapSetValue("object_y", m_T_world_object.translation()(1, 0));
    config.mapSetValue("object_z", m_T_world_object.translation()(2, 0));
    msg_utils::get_euler_ypr(m_T_world_object, yaw, pitch, roll);
    config.mapSetValue("object_yaw", yaw);
    config.mapSetValue("object_pitch", pitch);
    config.mapSetValue("object_roll", roll);

    config.mapSetValue("object_start", m_obj_start);
    config.mapSetValue("object_goal", m_obj_goal);
}

void GraspingCommandPanel::refreshRobotDescription()
{
    auto user_robot_description = m_robot_description_line_edit->text().toStdString();
    if (user_robot_description.empty()) {
        QMessageBox::information(this, tr("Robot Description"), tr("Please enter a valid ROS parameter for the URDF"));
        return;
    }

    std::string why;
    if (!setRobotDescription(user_robot_description, why)) {
        QMessageBox::warning(
                this,
                tr("Robot Description"),
                tr("Failed to set the robot description to '%1' (%2)")
                    .arg(QString::fromStdString(user_robot_description), QString::fromStdString(why)));
    }

    updateGUI();
}

void GraspingCommandPanel::refreshGlobalFrame()
{
    auto user_global_frame = m_global_frame_line_edit->text().toStdString();

    std::string why;
    if (!setGlobalFrame(user_global_frame, why)) {
        QMessageBox::warning(
                this,
                tr("Global Frame"),
                tr("Failed to set the global frame to  '%1' (%2)")
                    .arg(QString::fromStdString(user_global_frame), QString::fromStdString(why)));
    }

    updateGUI();
}

void GraspingCommandPanel::refreshObjectMeshResource()
{
    QMessageBox::warning(this, tr("Object Mesh Resource"), tr("You set the object mesh resource"));
    m_obj_mesh_resource = m_obj_mesh_resource_line_edit->text().toStdString();
    reinitObjectInteractiveMarker();
}

void GraspingCommandPanel::copyCurrentBasePose()
{
    try {
        tf::StampedTransform world_to_robot;
        m_listener.lookupTransform(global_frame_, robot_model_->getModelFrame(), ros::Time(0), world_to_robot);
        tf::transformTFToEigen(world_to_robot, m_T_world_robot);

        m_teleport_base_command_x_box->setValue(m_T_world_robot.translation()[0]);
        m_teleport_base_command_y_box->setValue(m_T_world_robot.translation()[1]);
        m_teleport_base_command_z_box->setValue(m_T_world_robot.translation()[2]);

        double roll, pitch, yaw;
        msg_utils::get_euler_ypr(m_T_world_robot, yaw, pitch, roll);
        m_teleport_base_command_yaw_box->setValue(smpl::angles::to_degrees(yaw));

        publishPhantomRobotVisualization();
    } catch (const tf::TransformException& ex) {
        QMessageBox::critical(this, tr("Transform Exception"), tr("%1").arg(QString(ex.what())));
    }
}

void GraspingCommandPanel::updateBasePoseX(double x)
{
    m_T_world_robot.translation()(0, 0) = x;
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseY(double y)
{
    m_T_world_robot.translation()(1, 0) = y;
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseZ(double z)
{
    m_T_world_robot.translation()(2, 0) = z;
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseYaw(double yaw_deg)
{
    double yaw_rad = smpl::angles::to_radians(yaw_deg);
    m_T_world_robot =
            Eigen::Translation3d(m_T_world_robot.translation()) *
            Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d(0.0, 0.0, 1.0));
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseCandidate(int index)
{
    if (index > 0) {
        base_candidate_idx_ = index - 1;
        assert(base_candidate_idx_ >= 0 && base_candidate_idx_ < candidate_base_poses_.size());
        publishBasePoseCandidateVisualization(candidate_base_poses_[base_candidate_idx_]);
    }
}

void GraspingCommandPanel::updateAllowedPlanningTime(double val)
{
    if (m_allowed_planning_time != val) {
        m_allowed_planning_time = val;
    }
}

void GraspingCommandPanel::updateMeshScaleX()
{
    auto ok = false;
    auto scale = m_obj_mesh_scale_x_line_edit->text().toDouble(&ok);
    if (ok) {
        updateMeshScaleX(scale);
    }
}

void GraspingCommandPanel::updateMeshScaleY()
{
    auto ok = false;
    auto scale = m_obj_mesh_scale_y_line_edit->text().toDouble(&ok);
    if (ok) {
        updateMeshScaleY(scale);
    }
}

void GraspingCommandPanel::updateMeshScaleZ()
{
    auto ok = false;
    auto scale = m_obj_mesh_scale_z_line_edit->text().toDouble(&ok);
    if (ok) {
        updateMeshScaleZ(scale);
    }
}

void GraspingCommandPanel::updateObjectPoseX(double val)
{
    if (m_T_world_object.translation().x() != val) {
        m_T_world_object.translation().x() = val;
        updateObjectMarkerPose();
        Q_EMIT objectPoseXUpdated(val);
    }
}

void GraspingCommandPanel::updateObjectPoseY(double val)
{
    if (m_T_world_object.translation().y() != val) {
        m_T_world_object.translation().y() = val;
        updateObjectMarkerPose();
        Q_EMIT objectPoseYUpdated(val);
    }
}

void GraspingCommandPanel::updateObjectPoseZ(double val)
{
    if (m_T_world_object.translation().z() != val) {
        m_T_world_object.translation().z() = val;
        updateObjectMarkerPose();
        Q_EMIT objectPoseZUpdated(val);
    }
}

void GraspingCommandPanel::updateObjectPoseYaw(double val)
{
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    auto new_yaw = smpl::to_radians(val);
    if (new_yaw != yaw) {
        yaw = new_yaw;
        Eigen::Matrix3d new_rot;
        smpl::from_euler_zyx(yaw, pitch, roll, new_rot);
        m_T_world_object = Eigen::Translation3d(m_T_world_object.translation()) * Eigen::Quaterniond(new_rot);
        updateObjectMarkerPose();
        Q_EMIT objectPoseYawUpdated(val);
    }
}

void GraspingCommandPanel::updateObjectPosePitch(double val)
{
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    auto new_pitch = smpl::to_radians(val);
    if (new_pitch != pitch) {
        pitch = new_pitch;
        Eigen::Matrix3d new_rot;
        smpl::from_euler_zyx(yaw, pitch, roll, new_rot);
        m_T_world_object = Eigen::Translation3d(m_T_world_object.translation()) * Eigen::Quaterniond(new_rot);
        updateObjectMarkerPose();
        Q_EMIT objectPosePitchUpdated(val);
    }
}

void GraspingCommandPanel::updateObjectPoseRoll(double val)
{
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    auto new_roll = smpl::to_radians(val);
    if (new_roll != roll) {
        roll = new_roll;
        Eigen::Matrix3d new_rot;
        smpl::from_euler_zyx(yaw, pitch, roll, new_rot);
        m_T_world_object = Eigen::Translation3d(m_T_world_object.translation()) * Eigen::Quaterniond(new_rot);
        updateObjectMarkerPose();
        Q_EMIT objectPoseRollUpdated(val);
    }
}

void GraspingCommandPanel::updateMeshScaleX(double scale)
{
    m_obj_scale_x = scale;
    reinitObjectInteractiveMarker();
}

void GraspingCommandPanel::updateMeshScaleY(double scale)
{
    m_obj_scale_y = scale;
    reinitObjectInteractiveMarker();
}

void GraspingCommandPanel::updateMeshScaleZ(double scale)
{
    m_obj_scale_z = scale;
    reinitObjectInteractiveMarker();
}

void GraspingCommandPanel::updateObjectStart()
{
    auto ok = false;
    auto start = m_object_start_line_edit->text().toDouble(&ok);
    if (ok) {
        m_obj_start = start;
    }
}

void GraspingCommandPanel::updateObjectGoal()
{
    auto ok = false;
    auto goal = m_object_goal_line_edit->text().toDouble(&ok);
    if (ok) {
        m_obj_goal = goal;
    }
}

void GraspingCommandPanel::sendGraspObjectCommand()
{
    visualization_msgs::InteractiveMarker gas_can_interactive_marker;
    if (!m_server.get(m_gascan_interactive_marker_name, gas_can_interactive_marker)) {
        QMessageBox::warning(
                this,
                tr("Command Failure"),
                tr("Unable to send Grasp Object Command (no interactive marker named '%1'")
                    .arg(QString::fromStdString(m_gascan_interactive_marker_name)));
        return;
    }

    // robot -> object = robot -> world * world -> object
    Eigen::Affine3d robot_to_object = m_T_world_robot.inverse() * m_T_world_object;

    ROS_DEBUG_NAMED(LOG, "Robot -> Marker: %s", to_string(m_T_world_robot.inverse()).c_str());
    ROS_DEBUG_NAMED(LOG, "Marker -> Object: %s", to_string(m_T_world_object).c_str());
    ROS_DEBUG_NAMED(LOG, "Robot -> Object: %s", to_string(robot_to_object).c_str());

#if USE_MANIPULATE_ACTION
    if(!ReconnectActionClient(manipulate_client_, "manipulate")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Manipulate (server is not available)"));
        return;
    }

    cmu_manipulation_msgs::ManipulateGoal grasp_object_goal;
    grasp_object_goal.task = "pickup";
    grasp_object_goal.selected_arms |= cmu_manipulation_msgs::ManipulateGoal::RIGHT;

    grasp_object_goal.goal_poses.resize(1);
    grasp_object_goal.goal_poses[0].header.frame_id = robot_model_->getModelFrame();
    tf::poseEigenToMsg(m_T_world_object, grasp_object_goal.goal_poses[0].pose);

    auto result_callback = boost::bind(&GraspingCommandPanel::manipulate_result_cb, this, _1, _2);
    manipulate_client_->sendGoal(grasp_object_goal, result_callback);

    pending_manipulate_command_ = true;
#else
    if (!ReconnectActionClient(grasp_object_command_client_, "grasp_object_command")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Grasp Object Command (server is not available)"));
        return;
    }

    cmu_manipulation_msgs::GraspObjectCommandGoal grasp_object_goal;
    grasp_object_goal.gas_can_in_base_link.header.frame_id = robot_model_->getModelFrame();
    tf::poseEigenToMsg(robot_to_object, grasp_object_goal.gas_can_in_base_link.pose);

    static int grasp_object_goal_id = 0;
    grasp_object_goal.id = grasp_object_goal_id++;
    grasp_object_goal.retry_count = 0;

    grasp_object_goal.gas_can_in_map.header.frame_id = global_frame_;
    tf::poseEigenToMsg(m_T_world_object, grasp_object_goal.gas_can_in_map.pose);

    auto result_callback = boost::bind(&GraspingCommandPanel::grasp_object_command_result_cb, this, _1, _2);
    grasp_object_command_client_->sendGoal(grasp_object_goal, result_callback);

    pending_grasp_object_command_ = true;
#endif

    updateGUI();
}

void GraspingCommandPanel::sendRepositionBaseCommand()
{
    if (!ReconnectActionClient(
            reposition_base_command_client_, "reposition_base_command"))
    {
        QMessageBox::warning(
                this,
                tr("Command Failure"),
                tr("Unable to send Reposition Base Command (server is not available)"));
        return;
    }

    if (!occupancy_grid_) {
        QMessageBox::warning(this, tr("Command Failure"), tr("No map data received"));
        return;
    }

    cmu_manipulation_msgs::RepositionBaseCommandGoal reposition_base_goal;

    static int reposition_base_goal_id = 0;
    reposition_base_goal.id = reposition_base_goal_id++;
    reposition_base_goal.retry_count = 0;

    tf::poseEigenToMsg(m_T_world_object, reposition_base_goal.gas_can_in_map.pose);
    reposition_base_goal.gas_can_in_map.header.frame_id = global_frame_;

    tf::poseEigenToMsg(m_T_world_robot, reposition_base_goal.base_link_in_map.pose);
    reposition_base_goal.base_link_in_map.header.frame_id = global_frame_;

    reposition_base_goal.map = *occupancy_grid_;

    auto result_callback = boost::bind(&GraspingCommandPanel::reposition_base_command_result_cb, this, _1, _2);
    reposition_base_command_client_->sendGoal(reposition_base_goal, result_callback);

    pending_reposition_base_command_ = true;
    updateGUI();
}

void GraspingCommandPanel::sendManipulateObjectCommand()
{
    if (!ReconnectActionClient(manipulate_object_client_, "manipulate_object")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Manipulate Object command (server is not available)"));
        return;
    }

    cmu_manipulation_msgs::ManipulateObjectGoal goal;

    static auto goal_id = 0;
    goal.object_id = "crate";
    tf::poseEigenToMsg(m_T_world_object, goal.object_pose);
    goal.object_start = m_obj_start;
    goal.object_goal = m_obj_goal;
    goal.plan_only = false;
    goal.allowed_planning_time = m_allowed_planning_time;
    goal.start_state.is_diff = true;

    auto result_callback = boost::bind(&GraspingCommandPanel::manipulateObjectResultCallback, this, _1, _2);
    manipulate_object_client_->sendGoal(goal, result_callback);

    pending_manipulate_command_ = true;
    updateGUI();
}

bool GraspingCommandPanel::setRobotDescription(
    const std::string& robot_description,
    std::string& why)
{
    // attempt to reinitialize the robot from this robot description
    if (reinit(robot_description, why)) {
        ROS_DEBUG_NAMED(LOG, "Successfully reinitialized robot from '%s'", robot_description.c_str());

        if (robot_description_ != robot_description) {
            // robot description changed to something different, not just
            // refreshed
            Q_EMIT(configChanged());
        }

        robot_description_ = robot_description;

        // set the line edit, since we may have called this function via methods
        // other than the refresh button. note: ok to do this here without
        // disabling signals since setRobotDescription is invoked via refresh
        // button callback
        QString q_robot_desc = QString::fromStdString(robot_description_);
        m_robot_description_line_edit->setText(q_robot_desc);

        ROS_DEBUG_NAMED(LOG, "Robot Description set to '%s'", robot_description.c_str());
        return true;
    } else {
        QMessageBox::warning(
                this,
                tr("Refresh Robot Description"),
                tr("Failed to Reinitialize (%1)").arg(QString::fromStdString(why)));

        // revert to previous robot_description
        auto q_robot_desc = QString::fromStdString(robot_description_);
        m_robot_description_line_edit->setText(q_robot_desc);
        return false;
    }
}

bool GraspingCommandPanel::setGlobalFrame(
    const std::string& global_frame,
    std::string& why)
{
    if (!isValidGlobalFrame(global_frame)) {
        why = global_frame + " is not a valid frame";
        m_global_frame_line_edit->setText(QString::fromStdString(global_frame_));
        return false;
    }

    // TODO: consider transforming the robot and object transforms so that they
    // remain in the same positions

    if (global_frame != global_frame_) {
        Q_EMIT(configChanged());
    }

    global_frame_ = global_frame;
    m_global_frame_line_edit->setText(QString::fromStdString(global_frame_));

    updateObjectMarkerPose();

    ROS_DEBUG_NAMED(LOG, "Global Frame set to %s", global_frame.c_str());
    return true;
}

bool GraspingCommandPanel::isValidGlobalFrame(const std::string& frame) const
{
    // TODO: remove these...check for existing tf frames that are not part of
    // the robot and refresh periodically?...use a dropdown for this
    auto valid_global_frames = std::vector<std::string>{
        "abs_nwu",
        "abs_ned",
        "/abs_nwu",
        "/abs_ned",
        "map",
    };

    if (robot_model_) {
        auto srdf = robot_model_->getSRDF();
        if (srdf) {
            const auto& vjs = srdf->getVirtualJoints();
            for (const auto& vj : vjs) {
                valid_global_frames.push_back(vj.parent_frame_);
            }
        }
    }

    auto fit = std::find(valid_global_frames.begin(), valid_global_frames.end(), frame);
    if (fit == valid_global_frames.end()) {
        ROS_DEBUG_NAMED(LOG, "'%s' is not a valid frame. Available candidates are:", frame.c_str());
        for (const auto& f : valid_global_frames) {
            ROS_DEBUG_NAMED(LOG, "  %s", f.c_str());
        }
    }

    return fit != valid_global_frames.end();
}

bool GraspingCommandPanel::robotModelLoaded() const
{
    return (bool)(robot_model_);
}

void GraspingCommandPanel::processObjectMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        tf::poseMsgToEigen(feedback->pose, m_T_world_object);
        double yaw, pitch, roll;
        msg_utils::get_euler_ypr(m_T_world_object, yaw, pitch, roll);
        Q_EMIT objectPoseXUpdated(m_T_world_object.translation().x());
        Q_EMIT objectPoseYUpdated(m_T_world_object.translation().y());
        Q_EMIT objectPoseZUpdated(m_T_world_object.translation().z());
        Q_EMIT objectPoseYawUpdated(yaw);
        Q_EMIT objectPosePitchUpdated(pitch);
        Q_EMIT objectPoseRollUpdated(roll);
    }
}

void GraspingCommandPanel::publishPhantomRobotVisualization()
{
    if (!robotModelLoaded()) {
        return;
    }

    robot_state_->update();

    auto& link_names = robot_model_->getLinkModelNames();
    auto color = std_msgs::CreateColorRGBA(0.94, 0.44, 0.44, 1.0);
    std::string ns = "phantom_robot_link";
    ros::Duration d(0);

    visualization_msgs::MarkerArray marker_array;
    robot_state_->getRobotMarkers(marker_array, link_names, color, ns, d);

    // transform all markers from the robot frame into the global frame
    for (auto& marker : marker_array.markers) {
        Eigen::Affine3d root_to_marker;
        tf::poseMsgToEigen(marker.pose, root_to_marker);
        // world -> marker = world -> robot * robot -> marker
        Eigen::Affine3d world_to_marker = m_T_world_robot * root_to_marker;
        tf::poseEigenToMsg(world_to_marker, marker.pose);
        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time(0);
    }

    m_marker_pub.publish(marker_array);
}

void GraspingCommandPanel::publishBasePoseCandidateVisualization(
    const geometry_msgs::PoseStamped& candidate_pose)
{
    if (!robotModelLoaded()) {
        return;
    }

    robot_state_->update();

    auto& link_names = robot_model_->getLinkModelNames();
    auto color = std_msgs::CreateColorRGBA(0.44, 0.94, 0.44, 1.0);
    std::string ns = "base_pose_candidate";
    ros::Duration d(0);

    visualization_msgs::MarkerArray marker_array;
    robot_state_->getRobotMarkers(marker_array, link_names, color, ns, d);

    Eigen::Affine3d T_world_candidate;
    tf::poseMsgToEigen(candidate_pose.pose, T_world_candidate);

    // transform all markers from the robot frame into the global frame
    for (auto& marker : marker_array.markers) {
        Eigen::Affine3d root_to_marker;
        tf::poseMsgToEigen(marker.pose, root_to_marker);
        // world -> marker = world -> robot * robot -> marker
        Eigen::Affine3d world_to_marker = T_world_candidate * root_to_marker;
        tf::poseEigenToMsg(world_to_marker, marker.pose);
        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time(0);
    }

    m_marker_pub.publish(marker_array);
}

auto GraspingCommandPanel::createSixDOFControls() const
    -> std::vector<visualization_msgs::InteractiveMarkerControl>
{
    auto controls = std::vector<visualization_msgs::InteractiveMarkerControl>(6);

    visualization_msgs::InteractiveMarkerControl control;

    control.name = "";
    control.orientation.w = 1.0;
    control.orientation.x = control.orientation.y = control.orientation.z = 0.0;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    control.always_visible = false;
    control.description = "";

    // rotate and translate about and on the x-axis
    control.name = "rotate_x";
    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls[0] = control;

    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls[1] = control;

    // rotate and translate about and on the z-axis
    control.name = "rotate_z";
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 1.0;
    control.orientation.z = 0.0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls[2] = control;

    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls[3] = control;

    // rotate and translate about and on the y-axis
    control.name = "rotate_y";
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 1.0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    controls[4] = control;

    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls[5] = control;

    return controls;
}

void GraspingCommandPanel::occupancyGridCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    occupancy_grid_ = msg;
}

bool GraspingCommandPanel::reinit(
    const std::string& robot_description,
    std::string& why)
{
    if (!reinitRobotModels(robot_description, why)) {
        return false;
    }

    if (!reinitObjectInteractiveMarker()) {
        return false;
    }

    publishPhantomRobotVisualization();
    return true;
}

bool GraspingCommandPanel::reinitRobotModels(
    const std::string& robot_description,
    std::string& why)
{
    if (!m_nh.hasParam(robot_description) ||
        !m_nh.hasParam(robot_description + "_semantic"))
    {
        std::stringstream ss;
        ss << "Failed to retrieve '" << robot_description << "' and '" <<
                (robot_description + "_semantic") << "' from the param server";
        why = ss.str();
        return false;
    }

    std::string urdf_string;
    if (!m_nh.getParam(robot_description, urdf_string)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << robot_description <<
                "' from the param server";
        why = ss.str();
        return false;
    }

    rml_.reset(new robot_model_loader::RobotModelLoader(robot_description, true));

    robot_model::RobotModelPtr robot_model = rml_->getModel();
    if (!robot_model) {
        why = "Robot Model Loader was unable to construct Robot Model";
        return false;
    }

    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    if (!robot_state) {
        why = "Failed to instantiate Robot State";
        return false;
    }

    // All lights are green from above
    robot_model_ = robot_model;
    robot_state_ =  robot_state;

    ROS_DEBUG_NAMED(LOG, "MoveIt model is consistent with HDT Robot Model");

    ROS_DEBUG_NAMED(LOG, "Root link name: %s", robot_model_->getRootLinkName().c_str());
    ROS_DEBUG_NAMED(LOG, "Robot Joints:");
    for (const std::string& joint_name : robot_model_->getJointModelNames()) {
        ROS_DEBUG_NAMED(LOG, "    %s", joint_name.c_str());
    }

    robot_state_->setToDefaultValues();
    robot_state_->update();
    return true;
}

bool GraspingCommandPanel::reinitObjectInteractiveMarker()
{
    ROS_DEBUG_NAMED(LOG, "Inserting marker '%s'", m_gascan_interactive_marker_name.c_str());

    // TODO: grab the gas canister mesh and scale from the parameter server
    // (does this mean those parameters have to be global?)

    // initializer an interactive marker for the gas canister object
    visualization_msgs::InteractiveMarker gascan_imarker;
    gascan_imarker.header.frame_id = global_frame_;
    tf::poseEigenToMsg(m_T_world_object, gascan_imarker.pose);
//    gascan_imarker.pose = geometry_msgs::IdentityPose();     // TODO: object pose?
    gascan_imarker.name = m_gascan_interactive_marker_name;
    gascan_imarker.description = "Gas Canister Positioning";

    // the scale defines the inner diameter of the interactive marker circle
    gascan_imarker.scale = 2.0 * 0.3; //1.0f; //0.25f;
    gascan_imarker.menu_entries.clear();
    gascan_imarker.controls.clear();

    gascan_imarker.controls = createSixDOFControls();

    // Construct mesh control
    visualization_msgs::InteractiveMarkerControl gas_can_mesh_control;
    gas_can_mesh_control.name = "mesh_control";
    gas_can_mesh_control.orientation = geometry_msgs::IdentityQuaternion();
    gas_can_mesh_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    gas_can_mesh_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    gas_can_mesh_control.always_visible = true;

    // add mesh marker
    visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "";
    mesh_marker.ns = "gas_can_mesh_marker";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.pose = geometry_msgs::IdentityPose();
    mesh_marker.scale = geometry_msgs::CreateVector3(m_obj_scale_x, m_obj_scale_y, m_obj_scale_z);
    mesh_marker.color = std_msgs::WhiteColorRGBA(0.5f);
    mesh_marker.lifetime = ros::Duration(0);
    mesh_marker.frame_locked = false;

    mesh_marker.mesh_resource = m_obj_mesh_resource;
    mesh_marker.mesh_use_embedded_materials = false;
    gas_can_mesh_control.markers.push_back(mesh_marker);

    gas_can_mesh_control.independent_marker_orientation = false;
    gas_can_mesh_control.description = "";

    gascan_imarker.controls.push_back(gas_can_mesh_control);

    m_server.insert(gascan_imarker);
    auto gascan_feedback_cb = boost::bind(&GraspingCommandPanel::processObjectMarkerFeedback, this, _1);
    m_server.setCallback(gascan_imarker.name, gascan_feedback_cb);
    ROS_DEBUG_NAMED(LOG, "Inserted marker '%s'", m_gascan_interactive_marker_name.c_str());

    m_server.applyChanges();
    return true;
}

void GraspingCommandPanel::manipulate_active_cb()
{
}

void GraspingCommandPanel::manipulate_feedback_cb(
    const cmu_manipulation_msgs::ManipulateFeedback::ConstPtr& feedback)
{
}

void GraspingCommandPanel::manipulate_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::ManipulateResult::ConstPtr& result)
{
    ROS_DEBUG_NAMED(LOG, "Received Result from Grasp Object Command Action");
    pending_manipulate_command_ = false;
    updateGUI();
}

void GraspingCommandPanel::grasp_object_command_active_cb()
{

}

void GraspingCommandPanel::grasp_object_command_feeback_cb(const cmu_manipulation_msgs::GraspObjectCommandFeedback::ConstPtr& feedback)
{

}

void GraspingCommandPanel::grasp_object_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::GraspObjectCommandResult::ConstPtr& result)
{
    ROS_DEBUG_NAMED(LOG, "Received Result from Grasp Object Command Action");
    pending_grasp_object_command_ = false;
    updateGUI();
}

void GraspingCommandPanel::reposition_base_command_active_cb()
{

}

void GraspingCommandPanel::reposition_base_command_feedback_cb(const cmu_manipulation_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback)
{

}

void GraspingCommandPanel::reposition_base_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::RepositionBaseCommandResult::ConstPtr& result)
{

    ROS_DEBUG_NAMED(LOG, "Received Result from Reposition Base Command Action");
    pending_reposition_base_command_ = false;

    if (result && result->result == cmu_manipulation_msgs::RepositionBaseCommandResult::SUCCESS) {
        candidate_base_poses_ = result->candidate_base_poses;
        ROS_DEBUG_NAMED(LOG, "Reposition Base Command returned %zd candidate poses", candidate_base_poses_.size());
    }

    updateGUI();
}

void GraspingCommandPanel::manipulateObjectResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::ManipulateObjectResult::ConstPtr& result)
{
    ROS_DEBUG_NAMED(LOG, "Received result from Manipulate Object action");
}

void GraspingCommandPanel::updateBasePoseSpinBoxes()
{
    disconnect(m_teleport_base_command_x_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseX(double)));
    disconnect(m_teleport_base_command_y_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseY(double)));
    disconnect(m_teleport_base_command_yaw_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseYaw(double)));
    m_teleport_base_command_x_box->setValue(m_T_world_robot.translation()(0, 0));
    m_teleport_base_command_y_box->setValue(m_T_world_robot.translation()(1, 0));
    double yaw, pitch, roll;
    msg_utils::get_euler_ypr(m_T_world_robot, yaw, pitch, roll);
    m_teleport_base_command_yaw_box->setValue(smpl::angles::to_degrees(yaw));
    connect(m_teleport_base_command_x_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseX(double)));
    connect(m_teleport_base_command_y_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseY(double)));
    connect(m_teleport_base_command_yaw_box, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseYaw(double)));
}

// Update GUI elements to reflect the current model:
//  0. Whether a robot model has been loaded yet
//  1. State of each commands (pending or not)
//  2. Result of most recent reposition base command.
void GraspingCommandPanel::updateGUI()
{
    ROS_DEBUG_NAMED(LOG, "    Pending Grasp Object Command: %s", pending_grasp_object_command_ ? "TRUE" : "FALSE");
    ROS_DEBUG_NAMED(LOG, "    Pending Reposition Base Command: %s", pending_reposition_base_command_ ? "TRUE" : "FALSE");
    ROS_DEBUG_NAMED(LOG, "    Pending Manipulate: %s", pending_manipulate_command_ ? "TRUE" : "FALSE");

    auto pending_motion_command =
            pending_grasp_object_command_       ||
            pending_reposition_base_command_    ||
            pending_manipulate_command_         ||
            pending_manipulate_object_command_;

    // because actionlib isn't always as friendly as you might think
    pending_motion_command = false;

    m_obj_mesh_resource_line_edit->setText(QString::fromStdString(m_obj_mesh_resource));
    m_obj_mesh_scale_x_line_edit->setText(tr("%1").arg(m_obj_scale_x));
    m_obj_mesh_scale_y_line_edit->setText(tr("%1").arg(m_obj_scale_y));
    m_obj_mesh_scale_z_line_edit->setText(tr("%1").arg(m_obj_scale_z));

    m_object_start_line_edit->setText(tr("%1").arg(m_obj_start));
    m_object_goal_line_edit->setText(tr("%1").arg(m_obj_goal));

    m_allowed_planning_time_spinbox->setValue(m_allowed_planning_time);

    m_global_frame_line_edit->setEnabled(true);

    m_copy_current_base_pose_button->setEnabled(robotModelLoaded());
    m_teleport_base_command_x_box->setEnabled(robotModelLoaded());
    m_teleport_base_command_y_box->setEnabled(robotModelLoaded());
    m_teleport_base_command_z_box->setEnabled(robotModelLoaded());
    m_teleport_base_command_yaw_box->setEnabled(robotModelLoaded());

    send_grasp_object_command_button_->setEnabled(robotModelLoaded() && !pending_motion_command);
    send_reposition_base_command_button_->setEnabled(robotModelLoaded() && !pending_motion_command);

    auto num_candidates_label_text =
            "of " + std::to_string((int)candidate_base_poses_.size()) + " Candidates";

    if (candidate_base_poses_.empty()) {
        update_candidate_spinbox_->setMinimum(0);
        update_candidate_spinbox_->setMaximum(0);
        update_candidate_spinbox_->setValue(0);
        update_candidate_spinbox_->setEnabled(false);
    }
    else {
        update_candidate_spinbox_->setMinimum(1);
        update_candidate_spinbox_->setMaximum((int)candidate_base_poses_.size());
        update_candidate_spinbox_->setEnabled(true);
    }

    num_candidates_label_->setText(QString::fromStdString(num_candidates_label_text));
}

// Update the pose of the interactive marker to reflect m_T_world_object;
void GraspingCommandPanel::updateObjectMarkerPose()
{
    visualization_msgs::InteractiveMarker object_marker;
    if (!m_server.get(m_gascan_interactive_marker_name, object_marker)) {
        return;
    }

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = ros::Time(0);
    header.frame_id = global_frame_;
    geometry_msgs::Pose new_pose;
    tf::poseEigenToMsg(m_T_world_object, new_pose);
    m_server.setPose(m_gascan_interactive_marker_name, new_pose, header);
    m_server.applyChanges();
}

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GraspingCommandPanel, rviz::Panel)
