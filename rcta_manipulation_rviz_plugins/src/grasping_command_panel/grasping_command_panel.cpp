#include "grasping_command_panel.h"

// standard includes
#include <cassert>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>

// system includes
#include <QCheckBox>
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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <smpl/angles.h>
#include <smpl/stl/memory.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/std_msgs/std_msgs.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include "grasping_command_model.h"

namespace rcta {

static const char* LOG = "grasping_command_panel";

static
auto CreateSixDOFControls() -> std::vector<visualization_msgs::InteractiveMarkerControl>
{
    auto controls = std::vector<visualization_msgs::InteractiveMarkerControl>(6);

    auto control = visualization_msgs::InteractiveMarkerControl{ };

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

static
auto MakeTransformedRobotVisualization(
    const moveit::core::RobotModel& robot,
    moveit::core::RobotState& state,
    const Eigen::Affine3d& T_world_robot,
    const std_msgs::ColorRGBA& color,
    const std::string& ns)
    -> visualization_msgs::MarkerArray
{
    auto& link_names = robot.getLinkModelNames();
    auto d = ros::Duration(0);
    auto marker_array = visualization_msgs::MarkerArray();
    state.getRobotMarkers(marker_array, link_names, color, ns, d);

    // transform all markers from the robot frame into the global frame
    for (auto& marker : marker_array.markers) {
        auto root_to_marker = Eigen::Affine3d{ };
        tf::poseMsgToEigen(marker.pose, root_to_marker);
        auto world_to_marker = Eigen::Affine3d(T_world_robot * root_to_marker);
        tf::poseEigenToMsg(world_to_marker, marker.pose);
        marker.header.frame_id = robot.getModelFrame();
        marker.header.stamp = ros::Time(0);
    }

    return marker_array;
}

GraspingCommandPanel::GraspingCommandPanel(QWidget *parent) :
    rviz::Panel(parent),
    m_server("grasping_commands")
{
    m_model = smpl::make_unique<GraspingCommandModel>();

    ///////////////////
    // Construct GUI //
    ///////////////////

    auto* parent_layout = new QVBoxLayout;
    auto* scroll_area = new QScrollArea;
    auto* scroll_area_widget = new QWidget;
    auto* main_layout = new QVBoxLayout;

    ////////////////////
    // robot settings //
    ////////////////////

    auto* robot_settings_group = new QGroupBox(tr("Robot Settings"));
    auto* robot_settings_layout = new QVBoxLayout;

    auto* robot_description_layout = new QHBoxLayout;
    auto* robot_description_label = new QLabel(tr("Robot Description:"));
    m_robot_description_line_edit = new QLineEdit;
    m_refresh_robot_desc_button = new QPushButton(tr("Refresh"));
    robot_description_layout->addWidget(robot_description_label);
    robot_description_layout->addWidget(m_robot_description_line_edit);
    robot_description_layout->addWidget(m_refresh_robot_desc_button);

    robot_settings_layout->addLayout(robot_description_layout);
    robot_settings_group->setLayout(robot_settings_layout);

    /////////////////////
    // object settings //
    /////////////////////

    auto* object_settings_group = new QGroupBox(tr("Object Settings"));
    auto* object_settings_layout = new QVBoxLayout;

    auto* obj_mesh_resource_layout = new QHBoxLayout;
    auto* obj_mesh_resource_label = new QLabel(tr("Object Mesh Resource:"));
    m_obj_mesh_resource_line_edit = new QLineEdit;
    m_refresh_obj_mesh_resource_button = new QPushButton(tr("Refresh"));

    // TODO: initial value?
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

    object_settings_layout->addLayout(obj_mesh_resource_layout);
    object_settings_layout->addLayout(obj_pose_layout);
    object_settings_group->setLayout(object_settings_layout);

    ////////////////////////////////
    // reposition planner command //
    ////////////////////////////////

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

    m_send_reposition_base_command_button = new QPushButton(tr("Reposition Base"));

    auto* candidates_layout = new QHBoxLayout;
    m_update_candidate_spinbox = new QSpinBox;
    m_update_candidate_spinbox->setEnabled(false);
    m_num_candidates_label = new QLabel(tr("of 0 Candidates"));
    candidates_layout->addWidget(m_update_candidate_spinbox);
    candidates_layout->addWidget(m_num_candidates_label);

    reposition_planner_group_layout->addWidget(m_copy_current_base_pose_button);
    reposition_planner_group_layout->addLayout(base_pose_spinbox_layout);
    reposition_planner_group_layout->addWidget(m_send_reposition_base_command_button);
    reposition_planner_group_layout->addLayout(candidates_layout);

    reposition_planner_group->setLayout(reposition_planner_group_layout);

    ///////////////////////////////////////
    // manipulate object planner command //
    ///////////////////////////////////////

    auto* manip_object_command_group = new QGroupBox(tr("Manipulate Object"));

    auto* manip_object_command_layout = new QVBoxLayout;

    m_send_manipulate_object_button = new QPushButton(tr("Manipulate Object"));

    m_allowed_planning_time_spinbox = new QDoubleSpinBox;
    m_allowed_planning_time_spinbox->setMinimum(0.0);

    auto* allowed_planning_time_layout = new QHBoxLayout;
    allowed_planning_time_layout->addWidget(new QLabel(tr("Allowed Planning Time:")));
    allowed_planning_time_layout->addWidget(m_allowed_planning_time_spinbox);

    m_plan_only_checkbox = new QCheckBox(tr("Plan Only"));
    m_plan_only_checkbox->setCheckState(m_model->m_plan_only ? Qt::Checked : Qt::Unchecked);

    auto* manip_object_settings_layout = new QHBoxLayout;
    m_object_start_line_edit = new QLineEdit;
    m_object_goal_line_edit = new QLineEdit;
    manip_object_settings_layout->addWidget(new QLabel(tr("Object Start:")));
    manip_object_settings_layout->addWidget(m_object_start_line_edit);
    manip_object_settings_layout->addWidget(new QLabel(tr("Object Goal:")));
    manip_object_settings_layout->addWidget(m_object_goal_line_edit);
    manip_object_command_layout->addWidget(m_send_manipulate_object_button);
    manip_object_command_layout->addLayout(manip_object_settings_layout);
    manip_object_command_layout->addLayout(allowed_planning_time_layout);
    manip_object_command_layout->addWidget(m_plan_only_checkbox);

    manip_object_command_group->setLayout(manip_object_command_layout);

    //////////////////////////////
    // grasping object executor //
    //////////////////////////////

    auto* grasp_object_command_group = new QGroupBox(tr("Grasp Object"));

    auto* grasp_object_command_layout = new QVBoxLayout;

    m_send_grasp_object_command_button = new QPushButton(tr("Grasp Object"));

    grasp_object_command_layout->addWidget(m_send_grasp_object_command_button);

    grasp_object_command_group->setLayout(grasp_object_command_layout);

    ///////////////////////////
    // Build the main layout //
    ///////////////////////////

    main_layout->addWidget(robot_settings_group);
    main_layout->addWidget(object_settings_group);
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

    auto* pmodel = m_model.get();

    connect(m_refresh_robot_desc_button, SIGNAL(clicked()),
            this, SLOT(refreshRobotDescription()));
    connect(pmodel, SIGNAL(robotLoaded(const QString&)),
            this, SLOT(notifyRobotLoaded(const QString&)));

    // object model
    connect(m_refresh_obj_mesh_resource_button, SIGNAL(clicked()),
            this, SLOT(refreshObjectMeshResource()));
    connect(pmodel, SIGNAL(objectMeshResourceChanged(const QString&)),
            this, SLOT(updateObjectMeshResourceChanged(const QString&)));
    connect(pmodel, SIGNAL(objectMeshScaleChanged(double, double, double)),
            this, SLOT(updateMeshScale(double, double, double)));
    connect(m_obj_mesh_scale_x_line_edit, SIGNAL(editingFinished()),
            this, SLOT(updateMeshScaleX()));
    connect(m_obj_mesh_scale_y_line_edit, SIGNAL(editingFinished()),
            this, SLOT(updateMeshScaleY()));
    connect(m_obj_mesh_scale_z_line_edit, SIGNAL(editingFinished()),
            this, SLOT(updateMeshScaleZ()));

    // object state
    connect(m_obj_pose_x_spinbox, SIGNAL(valueChanged(double)),
            pmodel, SLOT(setObjectPoseX(double)));
    connect(m_obj_pose_y_spinbox, SIGNAL(valueChanged(double)),
            pmodel, SLOT(setObjectPoseY(double)));
    connect(m_obj_pose_z_spinbox, SIGNAL(valueChanged(double)),
            pmodel, SLOT(setObjectPoseZ(double)));
    connect(m_obj_pose_Y_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(updateObjectPoseYawDegs(double)));
    connect(m_obj_pose_P_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(updateObjectPosePitchDegs(double)));
    connect(m_obj_pose_R_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(updateObjectPoseRollDegs(double)));
    connect(pmodel, SIGNAL(objectPoseXChanged(double)),
            this, SLOT(setObjectPoseX(double)));
    connect(pmodel, SIGNAL(objectPoseYChanged(double)),
            this, SLOT(setObjectPoseY(double)));
    connect(pmodel, SIGNAL(objectPoseZChanged(double)),
            this, SLOT(setObjectPoseZ(double)));
    connect(pmodel, SIGNAL(objectPoseYawChanged(double)),
            this, SLOT(setObjectPoseYaw(double)));
    connect(pmodel, SIGNAL(objectPosePitchChanged(double)),
            this, SLOT(setObjectPosePitch(double)));
    connect(pmodel, SIGNAL(objectPoseRollChanged(double)),
            this, SLOT(setObjectPoseRoll(double)));

    // reposition base commands
    connect(m_copy_current_base_pose_button, SIGNAL(clicked()),
            pmodel, SLOT(setRobotPoseToCurrent()));
    connect(m_teleport_base_command_x_box, SIGNAL(valueChanged(double)),
            pmodel, SLOT(setRobotPoseX(double)));
    connect(m_teleport_base_command_y_box, SIGNAL(valueChanged(double)),
            pmodel, SLOT(setRobotPoseY(double)));
    connect(m_teleport_base_command_yaw_box, SIGNAL(valueChanged(double)),
            this, SLOT(updateBasePoseYaw(double)));
    connect(m_update_candidate_spinbox, SIGNAL(valueChanged(int)),
            this, SLOT(updateBasePoseCandidate(int)));
    connect(m_send_reposition_base_command_button, SIGNAL(clicked()),
            pmodel, SLOT(sendRepositionBaseCommand()));

    // manipulate object commands
    connect(m_object_start_line_edit, SIGNAL(editingFinished()),
            this, SLOT(updateObjectStart()));
    connect(m_object_goal_line_edit, SIGNAL(editingFinished()),
            this, SLOT(updateObjectGoal()));
    connect(pmodel, SIGNAL(objectStartChanged(double)),
            this, SLOT(updateObjectStart(double)));
    connect(pmodel, SIGNAL(objectGoalChanged(double)),
            this, SLOT(updateObjectGoal(double)));
    connect(m_allowed_planning_time_spinbox, SIGNAL(valueChanged(double)),
            pmodel, SLOT(setAllowedPlanningTime(double)));
    connect(pmodel, SIGNAL(allowedPlanningTimeChanged(double)),
            m_allowed_planning_time_spinbox, SLOT(setValue(double)));
    // @plan-only
    connect(m_send_manipulate_object_button, SIGNAL(clicked()),
            pmodel, SLOT(sendManipulateObjectCommand()));

    // grasp object commands
    connect(m_send_grasp_object_command_button, SIGNAL(clicked()),
            pmodel, SLOT(sendGraspObjectCommand()));

    connect(pmodel, SIGNAL(errorEncountered(const QString&)),
            this, SLOT(displayErrorMessageBox(const QString&)));

    connect(m_plan_only_checkbox, SIGNAL(stateChanged(int)), this, SLOT(setPlanOnlyFromCheckbox(int)));
    connect(pmodel, SIGNAL(planOnlyChanged(bool)), this, SLOT(updatePlanOnly(bool)));

    m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 1);
}

GraspingCommandPanel::~GraspingCommandPanel()
{
}

void GraspingCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    ROS_DEBUG_NAMED(LOG, "Loading config for '%s'", this->getName().toStdString().c_str());
    m_model->load(config);
}

void GraspingCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    m_model->save(config);
}

void GraspingCommandPanel::refreshRobotDescription()
{
    auto urdf_param = m_robot_description_line_edit->text().toStdString();
    if (urdf_param.empty()) {
        QMessageBox::information(this, tr("Robot Description"), tr("Please enter a valid ROS parameter for the URDF"));
        return;
    }

    auto why = std::string();
    if (!m_model->loadRobot(urdf_param, &why)) {
        QMessageBox::warning(
                this,
                tr("Refresh Robot Description"),
                tr("Failed to load robot (%1)").arg(QString::fromStdString(why)));
        auto q_robot_desc = QString::fromStdString(m_model->m_robot_description);
        m_robot_description_line_edit->setText(q_robot_desc);
    }
}

void GraspingCommandPanel::refreshObjectMeshResource()
{
    QMessageBox::warning(this, tr("Object Mesh Resource"), tr("You set the object mesh resource"));
    auto obj_mesh_resource = m_obj_mesh_resource_line_edit->text().toStdString();
    m_model->setObjectMeshResource(obj_mesh_resource);
}

void GraspingCommandPanel::notifyRobotLoaded(const QString& urdf_param)
{
    m_robot_description_line_edit->setText(urdf_param);
    m_copy_current_base_pose_button->setEnabled(true);
    m_teleport_base_command_x_box->setEnabled(true);
    m_teleport_base_command_y_box->setEnabled(true);
    m_teleport_base_command_yaw_box->setEnabled(true);
    m_send_grasp_object_command_button->setEnabled(true);
    m_send_reposition_base_command_button->setEnabled(true);
    m_send_manipulate_object_button->setEnabled(true);
    updateObjectInteractiveMarker();
}

void GraspingCommandPanel::updateRobotState()
{
    auto x = m_model->m_T_world_robot.translation().x();
    auto y = m_model->m_T_world_robot.translation().y();
    auto z = m_model->m_T_world_robot.translation().z();
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_model->m_T_world_robot.rotation(), yaw, pitch, roll);

    m_teleport_base_command_x_box->setValue(x);
    m_teleport_base_command_y_box->setValue(y);
    m_teleport_base_command_yaw_box->setValue(smpl::to_degrees(yaw));

    if (!m_model->robotLoaded()) {
        return;
    }

    auto color = std_msgs::CreateColorRGBA(0.94, 0.44, 0.44, 1.0);
    auto ns = "phantom_robot_link";
    auto marker_array = MakeTransformedRobotVisualization(
            *m_model->m_robot_model,
            *m_model->m_robot_state,
            m_model->m_T_world_robot,
            color,
            ns);

    m_marker_pub.publish(marker_array);
}

void GraspingCommandPanel::updateBasePoseYaw(double yaw_deg)
{
    auto yaw_rad = smpl::to_radians(yaw_deg);
    m_model->setRobotPoseYaw(yaw_rad);
}

void GraspingCommandPanel::updateBasePoseCandidate(int index)
{
    if (index > 0) {
        m_model->m_base_candidate_idx = index - 1;
        assert(m_model->m_base_candidate_idx >= 0 && m_model->m_base_candidate_idx < m_model->m_candidate_base_poses.size());

        if (!m_model->robotLoaded()) {
            return;
        }

        auto& candidate_pose = m_model->m_candidate_base_poses[m_model->m_base_candidate_idx];

        auto color = std_msgs::CreateColorRGBA(0.44, 0.94, 0.44, 1.0);
        auto ns = "base_pose_candidate";

        auto T_world_candidate = Eigen::Affine3d{ };
        tf::poseMsgToEigen(candidate_pose.pose, T_world_candidate);

        auto marker_array = MakeTransformedRobotVisualization(
                *m_model->m_robot_model,
                *m_model->m_robot_state,
                T_world_candidate,
                color,
                ns);

        m_marker_pub.publish(marker_array);
    }
}

void GraspingCommandPanel::updateMeshScaleX()
{
    auto ok = false;
    auto scale = m_obj_mesh_scale_x_line_edit->text().toDouble(&ok);
    if (ok) m_model->setObjectMeshScaleX(scale);
}

void GraspingCommandPanel::updateMeshScaleY()
{
    auto ok = false;
    auto scale = m_obj_mesh_scale_y_line_edit->text().toDouble(&ok);
    if (ok) m_model->setObjectMeshScaleY(scale);
}

void GraspingCommandPanel::updateMeshScaleZ()
{
    auto ok = false;
    auto scale = m_obj_mesh_scale_z_line_edit->text().toDouble(&ok);
    if (ok) m_model->setObjectMeshScaleZ(scale);
}

void GraspingCommandPanel::updateObjectMeshResourceChanged(const QString& path)
{
    m_obj_mesh_resource_line_edit->setText(path);
    updateObjectInteractiveMarker();
    m_model->setObjectMeshResource(path.toStdString());
}

void GraspingCommandPanel::updateMeshScale(double x, double y, double z)
{
    // TODO: set mesh scale line edits, then remove the explicit set calls
    m_obj_mesh_scale_x_line_edit->setText(tr("%1").arg(x));
    m_obj_mesh_scale_y_line_edit->setText(tr("%1").arg(y));
    m_obj_mesh_scale_z_line_edit->setText(tr("%1").arg(z));
    updateObjectInteractiveMarker();
}

void GraspingCommandPanel::updateObjectPoseYawDegs(double val)
{
    m_model->setObjectPoseYaw(smpl::to_radians(val));
}

void GraspingCommandPanel::updateObjectPosePitchDegs(double val)
{
    m_model->setObjectPosePitch(smpl::to_radians(val));
}

void GraspingCommandPanel::updateObjectPoseRollDegs(double val)
{
    m_model->setObjectPoseRoll(smpl::to_radians(val));
}

void GraspingCommandPanel::setObjectPoseX(double val)
{
    m_obj_pose_x_spinbox->setValue(val);
    updateObjectMarkerPose();
}

void GraspingCommandPanel::setObjectPoseY(double val)
{
    m_obj_pose_y_spinbox->setValue(val);
    updateObjectMarkerPose();
}

void GraspingCommandPanel::setObjectPoseZ(double val)
{
    m_obj_pose_z_spinbox->setValue(val);
    updateObjectMarkerPose();
}

void GraspingCommandPanel::setObjectPoseYaw(double val)
{
    m_obj_pose_Y_spinbox->setValue(smpl::to_degrees(val));
    updateObjectMarkerPose();
}

void GraspingCommandPanel::setObjectPosePitch(double val)
{
    m_obj_pose_P_spinbox->setValue(smpl::to_degrees(val));
    updateObjectMarkerPose();
}

void GraspingCommandPanel::setObjectPoseRoll(double val)
{
    m_obj_pose_R_spinbox->setValue(smpl::to_degrees(val));
    updateObjectMarkerPose();
}

void GraspingCommandPanel::updateObjectStart()
{
    auto ok = false;
    auto start = m_object_start_line_edit->text().toDouble(&ok);
    if (ok) m_model->setObjectStart(start);
}

void GraspingCommandPanel::updateObjectGoal()
{
    auto ok = false;
    auto goal = m_object_goal_line_edit->text().toDouble(&ok);
    if (ok) m_model->setObjectGoal(goal);
}

void GraspingCommandPanel::updateObjectStart(double val)
{
    m_object_start_line_edit->setText(tr("%1").arg(val));
}

void GraspingCommandPanel::updateObjectGoal(double val)
{
    m_object_goal_line_edit->setText(tr("%1").arg(val));
}

void GraspingCommandPanel::processObjectMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        auto pose = Eigen::Affine3d{ };
        tf::poseMsgToEigen(feedback->pose, pose);
        double yaw, pitch, roll;
        smpl::get_euler_zyx(pose.rotation(), yaw, pitch, roll);
        m_model->setObjectPoseX(feedback->pose.position.x);
        m_model->setObjectPoseY(feedback->pose.position.y);
        m_model->setObjectPoseZ(feedback->pose.position.z);
        m_model->setObjectPoseYaw(yaw);
        m_model->setObjectPosePitch(pitch);
        m_model->setObjectPoseRoll(roll);
    }
}

void GraspingCommandPanel::updateObjectMarkerPose()
{
    auto marker = visualization_msgs::InteractiveMarker();
    if (m_server.get(m_gascan_interactive_marker_name, marker)) {
        tf::poseEigenToMsg(m_model->m_T_world_object, marker.pose);
        m_server.insert(marker);
        m_server.applyChanges();
    }
}

bool GraspingCommandPanel::updateObjectInteractiveMarker()
{
    if (!m_model->robotLoaded()) {
        ROS_INFO_NAMED(LOG, "Can't update object interactive marker. Robot not loaded");
        return false;
    }

    if (m_model->m_obj_mesh_resource.empty()) {
        ROS_INFO_NAMED(LOG, "Can't update object interactive marker. Object mesh resource not configured");
        return false;
    }

    ROS_DEBUG_NAMED(LOG, "Inserting marker '%s'", m_gascan_interactive_marker_name.c_str());

    // TODO: grab the gas canister mesh and scale from the parameter server
    // (does this mean those parameters have to be global?)

    // initializer an interactive marker for the gas canister object
    auto object_imarker = visualization_msgs::InteractiveMarker{ };
    object_imarker.header.frame_id = m_model->m_robot_model->getModelFrame();
    tf::poseEigenToMsg(m_model->m_T_world_object, object_imarker.pose);
//    object_imarker.pose = geometry_msgs::IdentityPose();     // TODO: object pose?
    object_imarker.name = m_gascan_interactive_marker_name;
    object_imarker.description = "Gas Canister Positioning";

    // the scale defines the inner diameter of the interactive marker circle
    object_imarker.scale = 2.0 * 0.3; //1.0f; //0.25f;
    object_imarker.menu_entries.clear();
    object_imarker.controls.clear();

    object_imarker.controls = CreateSixDOFControls();

    // Construct mesh control
    auto gas_can_mesh_control = visualization_msgs::InteractiveMarkerControl{ };
    gas_can_mesh_control.name = "mesh_control";
    gas_can_mesh_control.orientation = geometry_msgs::IdentityQuaternion();
    gas_can_mesh_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    gas_can_mesh_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    gas_can_mesh_control.always_visible = true;

    // add mesh marker
    auto mesh_marker = visualization_msgs::Marker{ };
    mesh_marker.header.frame_id = "";
    mesh_marker.ns = "gas_can_mesh_marker";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.pose = geometry_msgs::IdentityPose();
    mesh_marker.scale = geometry_msgs::CreateVector3(
            m_model->m_obj_scale_x,
            m_model->m_obj_scale_y,
            m_model->m_obj_scale_z);
    mesh_marker.color = std_msgs::WhiteColorRGBA(0.5f);
    mesh_marker.lifetime = ros::Duration(0);
    mesh_marker.frame_locked = false;

    mesh_marker.mesh_resource = m_model->m_obj_mesh_resource;
    mesh_marker.mesh_use_embedded_materials = false;
    gas_can_mesh_control.markers.push_back(mesh_marker);

    gas_can_mesh_control.independent_marker_orientation = false;
    gas_can_mesh_control.description = "";

    object_imarker.controls.push_back(gas_can_mesh_control);

    m_server.insert(object_imarker);
    auto gascan_feedback_cb = boost::bind(&GraspingCommandPanel::processObjectMarkerFeedback, this, _1);
    m_server.setCallback(object_imarker.name, gascan_feedback_cb);
    ROS_DEBUG_NAMED(LOG, "Inserted marker '%s'", m_gascan_interactive_marker_name.c_str());

    m_server.applyChanges();
    return true;
}

// Update GUI elements to reflect the current model:
//  0. Whether a robot model has been loaded yet
//  2. Result of most recent reposition base command.
void GraspingCommandPanel::updateGUI()
{
    auto num_candidates_label_text =
            "of " + std::to_string((int)m_model->m_candidate_base_poses.size()) + " Candidates";

    if (m_model->m_candidate_base_poses.empty()) {
        m_update_candidate_spinbox->setMinimum(0);
        m_update_candidate_spinbox->setMaximum(0);
        m_update_candidate_spinbox->setValue(0);
        m_update_candidate_spinbox->setEnabled(false);
    }
    else {
        m_update_candidate_spinbox->setMinimum(1);
        m_update_candidate_spinbox->setMaximum((int)m_model->m_candidate_base_poses.size());
        m_update_candidate_spinbox->setEnabled(true);
    }

    m_num_candidates_label->setText(
            QString::fromStdString(num_candidates_label_text));
}

void GraspingCommandPanel::displayErrorMessageBox(const QString& s)
{
    QMessageBox::warning(this, tr("Error"), s);
}

void GraspingCommandPanel::setPlanOnlyFromCheckbox(int state)
{
    m_model->setPlanOnly(state == Qt::Checked);
}

void GraspingCommandPanel::updatePlanOnly(bool plan_only)
{
    m_plan_only_checkbox->setCheckState(plan_only ? Qt::Checked : Qt::Unchecked);
}

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GraspingCommandPanel, rviz::Panel)
