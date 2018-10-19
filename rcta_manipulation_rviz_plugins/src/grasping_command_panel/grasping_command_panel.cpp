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
#include <QVBoxLayout>

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

GraspingCommandPanel::GraspingCommandPanel(QWidget *parent) :
    rviz::Panel(parent),
    server_("grasping_commands"),
    base_candidate_idx_(-1)
{
    setup_gui();
    robot_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1);
    occupancy_grid_sub_ = nh_.subscribe("map", 1, &GraspingCommandPanel::occupancyGridCallback, this);
}

GraspingCommandPanel::~GraspingCommandPanel()
{
}

void GraspingCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);

    ROS_INFO("Loading config for '%s'", this->getName().toStdString().c_str());

    QString global_frame;
    QString robot_description;
    float base_x;
    float base_y;
    float base_yaw;
    float object_x;
    float object_y;
    float object_z;
    float object_yaw;
    config.mapGetString("global_frame", &global_frame);
    config.mapGetString("robot_description", &robot_description);
    config.mapGetFloat("base_x", &base_x);
    config.mapGetFloat("base_y", &base_y);
    config.mapGetFloat("base_yaw", &base_yaw);
    config.mapGetFloat("object_x", &object_x);
    config.mapGetFloat("object_y", &object_y);
    config.mapGetFloat("object_z", &object_z);
    config.mapGetFloat("object_yaw", &object_yaw);

    ROS_INFO("Robot Description: %s", robot_description.toStdString().c_str());
    ROS_INFO("Global Frame: %s", global_frame.toStdString().c_str());

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

    T_world_robot_ =
            Eigen::Translation3d(base_x, base_y, 0.0) *
            Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ());

    T_world_object_ =
            Eigen::Translation3d(object_x, object_y, object_z) *
            Eigen::AngleAxisd(object_yaw, Eigen::Vector3d::UnitZ());

    updateObjectMarkerPose();
    updateBasePoseSpinBoxes();

    updateGUI();
}

void GraspingCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    ROS_INFO("Saving config for '%s'", this->getName().toStdString().c_str());

    config.mapSetValue("robot_description", QString::fromStdString(robot_description_));
    config.mapSetValue("global_frame", QString::fromStdString(global_frame_));

    config.mapSetValue("base_x", T_world_robot_.translation()(0, 0));
    config.mapSetValue("base_y", T_world_robot_.translation()(1, 0));
    double yaw, pitch, roll;
    msg_utils::get_euler_ypr(T_world_robot_, yaw, pitch, roll);
    config.mapSetValue("base_yaw", yaw);

    config.mapSetValue("object_x", T_world_object_.translation()(0, 0));
    config.mapSetValue("object_y", T_world_object_.translation()(1, 0));
    config.mapSetValue("object_z", T_world_object_.translation()(2, 0));
    msg_utils::get_euler_ypr(T_world_object_, yaw, pitch, roll);
    config.mapSetValue("object_yaw", yaw);
}

void GraspingCommandPanel::refresh_robot_description()
{
    std::string user_robot_description = robot_description_line_edit_->text().toStdString();
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

void GraspingCommandPanel::refresh_global_frame()
{
    std::string user_global_frame = global_frame_line_edit_->text().toStdString();

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

void GraspingCommandPanel::copyCurrentBasePose()
{
    try {
        tf::StampedTransform world_to_robot;
        listener_.lookupTransform(global_frame_, robot_model_->getModelFrame(), ros::Time(0), world_to_robot);
        tf::transformTFToEigen(world_to_robot, T_world_robot_);

        teleport_base_command_x_box_->setValue(T_world_robot_.translation()[0]);
        teleport_base_command_y_box_->setValue(T_world_robot_.translation()[1]);
        teleport_base_command_z_box_->setValue(T_world_robot_.translation()[2]);

        double roll, pitch, yaw;
        msg_utils::get_euler_ypr(T_world_robot_, yaw, pitch, roll);
        teleport_base_command_yaw_box_->setValue(smpl::angles::to_degrees(yaw));

        publishPhantomRobotVisualization();
    } catch (const tf::TransformException& ex) {
        QMessageBox::critical(this, tr("Transform Exception"), tr("%1").arg(QString(ex.what())));
    }
}

void GraspingCommandPanel::updateBasePoseX(double x)
{
    T_world_robot_.translation()(0, 0) = x;
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseY(double y)
{
    T_world_robot_.translation()(1, 0) = y;
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseZ(double z)
{
    T_world_robot_.translation()(2, 0) = z;
    publishPhantomRobotVisualization();
}

void GraspingCommandPanel::updateBasePoseYaw(double yaw_deg)
{
    double yaw_rad = smpl::angles::to_radians(yaw_deg);
    T_world_robot_ =
            Eigen::Translation3d(T_world_robot_.translation()) *
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

void GraspingCommandPanel::sendGraspObjectCommand()
{
    const std::string gas_can_interactive_marker_name = "gas_canister_fixture";
    visualization_msgs::InteractiveMarker gas_can_interactive_marker;
    if (!server_.get(gas_can_interactive_marker_name, gas_can_interactive_marker)) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Grasp Object Command (no interactive marker named 'gas_canister_fixture'"));
        return;
    }

    // robot -> object = robot -> world * world -> object
    Eigen::Affine3d robot_to_object = T_world_robot_.inverse() * T_world_object_;

    ROS_INFO("Robot -> Marker: %s", to_string(T_world_robot_.inverse()).c_str());
    ROS_INFO("Marker -> Object: %s", to_string(T_world_object_).c_str());
    ROS_INFO("Robot -> Object: %s", to_string(robot_to_object).c_str());

#if USE_MANIPULATE_ACTION
    if(!ReconnectActionClient(manipulate_client_, "manipulate")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Manipulate (server is not connected)"));
        return;
    }

    cmu_manipulation_msgs::ManipulateGoal grasp_object_goal;
    grasp_object_goal.task = "pickup";
    grasp_object_goal.selected_arms |= cmu_manipulation_msgs::ManipulateGoal::RIGHT;

    grasp_object_goal.goal_poses.resize(1);
    grasp_object_goal.goal_poses[0].header.frame_id = robot_model_->getModelFrame();
    tf::poseEigenToMsg(T_world_object_, grasp_object_goal.goal_poses[0].pose);

    auto result_callback = boost::bind(&GraspingCommandPanel::manipulate_result_cb, this, _1, _2);
    manipulate_client_->sendGoal(grasp_object_goal, result_callback);

    pending_manipulate_command_ = true;
#else
    if (!ReconnectActionClient(grasp_object_command_client_, "grasp_object_command")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Grasp Object Command (server is not connected)"));
        return;
    }

    cmu_manipulation_msgs::GraspObjectCommandGoal grasp_object_goal;
    grasp_object_goal.gas_can_in_base_link.header.frame_id = robot_model_->getModelFrame();
    tf::poseEigenToMsg(robot_to_object, grasp_object_goal.gas_can_in_base_link.pose);

    static int grasp_object_goal_id = 0;
    grasp_object_goal.id = grasp_object_goal_id++;
    grasp_object_goal.retry_count = 0;

    grasp_object_goal.gas_can_in_map.header.frame_id = global_frame_;
    tf::poseEigenToMsg(T_world_object_, grasp_object_goal.gas_can_in_map.pose);

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
                tr("Unable to send Reposition Base Command (server is not connected)"));
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

    tf::poseEigenToMsg(T_world_object_, reposition_base_goal.gas_can_in_map.pose);
    reposition_base_goal.gas_can_in_map.header.frame_id = global_frame_;

    tf::poseEigenToMsg(T_world_robot_, reposition_base_goal.base_link_in_map.pose);
    reposition_base_goal.base_link_in_map.header.frame_id = global_frame_;

    reposition_base_goal.map = *occupancy_grid_;

    auto result_callback = boost::bind(&GraspingCommandPanel::reposition_base_command_result_cb, this, _1, _2);
    reposition_base_command_client_->sendGoal(reposition_base_goal, result_callback);

    pending_reposition_base_command_ = true;
    updateGUI();
}

void GraspingCommandPanel::setup_gui()
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    // general settings
    auto* general_settings_group = new QGroupBox(tr("General Settings"));
    auto* general_settings_layout = new QVBoxLayout;
    auto* robot_description_layout = new QHBoxLayout;
    auto* robot_description_label = new QLabel(tr("Robot Description:"));
    robot_description_line_edit_ = new QLineEdit;
    refresh_robot_desc_button_ = new QPushButton(tr("Refresh"));
    robot_description_layout->addWidget(robot_description_label);
    robot_description_layout->addWidget(robot_description_line_edit_);
    robot_description_layout->addWidget(refresh_robot_desc_button_);
    auto* global_frame_layout = new QHBoxLayout;
    auto* global_frame_label = new QLabel(tr("Global Frame:"));
    global_frame_line_edit_ = new QLineEdit;
    global_frame_line_edit_->setEnabled(false);
    refresh_global_frame_button_ = new QPushButton(tr("Refresh"));
    global_frame_layout->addWidget(global_frame_label);
    global_frame_layout->addWidget(global_frame_line_edit_);
    global_frame_layout->addWidget(refresh_global_frame_button_);
    general_settings_layout->addLayout(robot_description_layout);
    general_settings_layout->addLayout(global_frame_layout);
    general_settings_group->setLayout(general_settings_layout);

    // base commands
    auto* base_commands_group = new QGroupBox(tr("Base Commands"));
    auto* base_commands_layout = new QVBoxLayout;
    copy_current_base_pose_button_ = new QPushButton(tr("Copy Current Base Pose"));
    auto* base_pose_spinbox_layout = new QHBoxLayout;
    auto* x_label = new QLabel(tr("X:"));
    teleport_base_command_x_box_ = new QDoubleSpinBox;
    teleport_base_command_x_box_->setMinimum(-100.0);
    teleport_base_command_x_box_->setMaximum(100.0);
    teleport_base_command_x_box_->setSingleStep(0.05);
    auto* y_label = new QLabel(tr("Y:"));
    teleport_base_command_y_box_ = new QDoubleSpinBox;
    teleport_base_command_y_box_->setMinimum(-100.0);
    teleport_base_command_y_box_->setMaximum(100.0);
    teleport_base_command_y_box_->setSingleStep(0.05);
    auto* z_label = new QLabel(tr("Z:"));
    teleport_base_command_z_box_ = new QDoubleSpinBox;
    auto* yaw_label = new QLabel(tr("Yaw:"));
    teleport_base_command_yaw_box_ = new QDoubleSpinBox;
    teleport_base_command_yaw_box_->setMinimum(0.0);
    teleport_base_command_yaw_box_->setMaximum(359.0);
    teleport_base_command_yaw_box_->setSingleStep(1.0);
    teleport_base_command_yaw_box_->setWrapping(true);
    base_pose_spinbox_layout->addWidget(x_label);
    base_pose_spinbox_layout->addWidget(teleport_base_command_x_box_);
    base_pose_spinbox_layout->addWidget(y_label);
    base_pose_spinbox_layout->addWidget(teleport_base_command_y_box_);
    base_pose_spinbox_layout->addWidget(yaw_label);
    base_pose_spinbox_layout->addWidget(teleport_base_command_yaw_box_);
    base_commands_layout->addWidget(copy_current_base_pose_button_);
    base_commands_layout->addLayout(base_pose_spinbox_layout);
    base_commands_group->setLayout(base_commands_layout);

    // object interaction commands
    auto* object_interaction_commands_group = new QGroupBox(tr("Object Interaction Commands"));
    auto* object_interaction_commands_layout = new QVBoxLayout;
    send_grasp_object_command_button_ = new QPushButton(tr("Grasp Object"));
    send_reposition_base_command_button_ = new QPushButton(tr("Reposition Base"));
    send_manipulate_object_command_button_ = new QPushButton(tr("Manipulate Object"));
    auto* candidates_layout = new QHBoxLayout;
    update_candidate_spinbox_ = new QSpinBox;
    update_candidate_spinbox_->setEnabled(false);
    num_candidates_label_ = new QLabel(tr("of 0 Candidates"));
    candidates_layout->addWidget(update_candidate_spinbox_);
    candidates_layout->addWidget(num_candidates_label_);
    object_interaction_commands_layout->addWidget(send_grasp_object_command_button_);
    object_interaction_commands_layout->addWidget(send_reposition_base_command_button_);
    object_interaction_commands_layout->addWidget(send_manipulate_object_command_button_);
    object_interaction_commands_layout->addLayout(candidates_layout);
    object_interaction_commands_group->setLayout(object_interaction_commands_layout);

    main_layout->addWidget(general_settings_group);
    main_layout->addWidget(base_commands_group);
    main_layout->addWidget(object_interaction_commands_group);
    setLayout(main_layout);

    // note: do not connect any outgoing signals from general settings line edits; force users to use refresh button
    connect(refresh_robot_desc_button_, SIGNAL(clicked()), this, SLOT(refresh_robot_description()));
    connect(refresh_global_frame_button_, SIGNAL(clicked()), this, SLOT(refresh_global_frame()));

    // base commands
    connect(copy_current_base_pose_button_, SIGNAL(clicked()), this, SLOT(copyCurrentBasePose()));
    connect(teleport_base_command_x_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseX(double)));
    connect(teleport_base_command_y_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseY(double)));
    connect(teleport_base_command_z_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseZ(double)));
    connect(teleport_base_command_yaw_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseYaw(double)));

    // object interaction commands
    connect(send_grasp_object_command_button_, SIGNAL(clicked()), this, SLOT(sendGraspObjectCommand()));
    connect(send_reposition_base_command_button_, SIGNAL(clicked()), this, SLOT(sendRepositionBaseCommand()));
    connect(update_candidate_spinbox_, SIGNAL(valueChanged(int)), this, SLOT(updateBasePoseCandidate(int)));
}

bool GraspingCommandPanel::setRobotDescription(
    const std::string& robot_description,
    std::string& why)
{
    // attempt to reinitialize the robot from this robot description
    if (reinit(robot_description, why)) {
        ROS_INFO("Successfully reinitialized robot from '%s'", robot_description.c_str());

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
        robot_description_line_edit_->setText(q_robot_desc);

        ROS_INFO("Robot Description set to '%s'", robot_description.c_str());
        return true;
    }
    else {
        QMessageBox::warning(
                this,
                tr("Refresh Robot Description"),
                tr("Failed to Reinitialize (%1)").arg(QString::fromStdString(why)));

        // revert to previous robot_description
        auto q_robot_desc = QString::fromStdString(robot_description_);
        robot_description_line_edit_->setText(q_robot_desc);
        return false;
    }
}

bool GraspingCommandPanel::setGlobalFrame(
    const std::string& global_frame,
    std::string& why)
{
    if (!isValidGlobalFrame(global_frame)) {
        why = global_frame + " is not a valid frame";
        global_frame_line_edit_->setText(QString::fromStdString(global_frame_));
        return false;
    }

    // TODO: consider transforming the robot and object transforms so that they
    // remain in the same positions

    if (global_frame != global_frame_) {
        Q_EMIT(configChanged());
    }

    global_frame_ = global_frame;
    global_frame_line_edit_->setText(QString::fromStdString(global_frame_));

    updateObjectMarkerPose();

    ROS_INFO("Global Frame set to %s", global_frame.c_str());
    return true;
}

bool GraspingCommandPanel::isValidGlobalFrame(const std::string& frame) const
{
    // TODO: remove these...check for existing tf frames that are not part of
    // the robot and refresh periodically?...use a dropdown for this
    std::vector<std::string> valid_global_frames = {
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
        ROS_INFO("'%s' is not a valid frame. Available candidates are:", frame.c_str());
        for (const auto& f : valid_global_frames) {
            ROS_INFO("  %s", f.c_str());
        }
    }

    return fit != valid_global_frames.end();
}

bool GraspingCommandPanel::initialized() const
{
    return (bool)(robot_model_);
}

void GraspingCommandPanel::processGascanMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        tf::poseMsgToEigen(feedback->pose, T_world_object_);
    }
}

void GraspingCommandPanel::publishPhantomRobotVisualization()
{
    if (!initialized()) {
        return;
    }

    robot_state_->update();

    const std::vector<std::string>& link_names = robot_model_->getLinkModelNames();
    std_msgs::ColorRGBA color = std_msgs::CreateColorRGBA(0.94, 0.44, 0.44, 1.0);
    std::string ns = "phantom_robot_link";
    ros::Duration d(0);

    visualization_msgs::MarkerArray marker_array;
    robot_state_->getRobotMarkers(marker_array, link_names, color, ns, d);

    // transform all markers from the robot frame into the global frame
    for (visualization_msgs::Marker& marker : marker_array.markers) {
        Eigen::Affine3d root_to_marker;
        tf::poseMsgToEigen(marker.pose, root_to_marker);
        // world -> marker = world -> robot * robot -> marker
        Eigen::Affine3d world_to_marker = T_world_robot_ * root_to_marker;
        tf::poseEigenToMsg(world_to_marker, marker.pose);
        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time(0);
    }

    robot_markers_pub_.publish(marker_array);
}

void GraspingCommandPanel::publishBasePoseCandidateVisualization(
    const geometry_msgs::PoseStamped& candidate_pose)
{
    if (!initialized()) {
        return;
    }

    robot_state_->update();

    const std::vector<std::string>& link_names = robot_model_->getLinkModelNames();
    std_msgs::ColorRGBA color = std_msgs::CreateColorRGBA(0.44, 0.94, 0.44, 1.0);
    std::string ns = "base_pose_candidate";
    ros::Duration d(0);

    visualization_msgs::MarkerArray marker_array;
    robot_state_->getRobotMarkers(marker_array, link_names, color, ns, d);

    Eigen::Affine3d T_world_candidate;
    tf::poseMsgToEigen(candidate_pose.pose, T_world_candidate);

    // transform all markers from the robot frame into the global frame
    for (visualization_msgs::Marker& marker : marker_array.markers) {
        Eigen::Affine3d root_to_marker;
        tf::poseMsgToEigen(marker.pose, root_to_marker);
        // world -> marker = world -> robot * robot -> marker
        Eigen::Affine3d world_to_marker = T_world_candidate * root_to_marker;
        tf::poseEigenToMsg(world_to_marker, marker.pose);
        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time(0);
    }

    robot_markers_pub_.publish(marker_array);
}

auto GraspingCommandPanel::createSixDOFControls() const
    -> std::vector<visualization_msgs::InteractiveMarkerControl>
{
    std::vector<visualization_msgs::InteractiveMarkerControl> controls(6);

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
    if (!nh_.hasParam(robot_description) ||
        !nh_.hasParam(robot_description + "_semantic"))
    {
        std::stringstream ss;
        ss << "Failed to retrieve '" << robot_description << "' and '" <<
                (robot_description + "_semantic") << "' from the param server";
        why = ss.str();
        return false;
    }

    std::string urdf_string;
    if (!nh_.getParam(robot_description, urdf_string)) {
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

    ROS_INFO("MoveIt model is consistent with HDT Robot Model");

    ROS_INFO("Root link name: %s", robot_model_->getRootLinkName().c_str());
    ROS_INFO("Robot Joints:");
    for (const std::string& joint_name : robot_model_->getJointModelNames()) {
        ROS_INFO("    %s", joint_name.c_str());
    }

    robot_state_->setToDefaultValues();
    robot_state_->update();
    return true;
}

bool GraspingCommandPanel::reinitObjectInteractiveMarker()
{
    ROS_INFO("Inserting marker 'gas_canister_fixture'");

    // TODO: grab the gas canister mesh and scale from the parameter server
    // (does this mean those parameters have to be global?)

    // initializer an interactive marker for the gas canister object
    visualization_msgs::InteractiveMarker gascan_imarker;
    gascan_imarker.header.frame_id = global_frame_;
    gascan_imarker.pose = geometry_msgs::IdentityPose();     // TODO: object pose?
    gascan_imarker.name = "gas_canister_fixture";
    gascan_imarker.description = "Gas Canister Positioning";
    gascan_imarker.scale = 0.25f;
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
    const double gas_can_mesh_scale = 1.0; //0.12905;
    mesh_marker.scale = geometry_msgs::CreateVector3(gas_can_mesh_scale, gas_can_mesh_scale, gas_can_mesh_scale);
    mesh_marker.color = std_msgs::WhiteColorRGBA(0.5f);
    mesh_marker.lifetime = ros::Duration(0);
    mesh_marker.frame_locked = false;

    mesh_marker.mesh_resource = "package://gascan_description/meshes/rcta_gastank.ply";
    mesh_marker.mesh_use_embedded_materials = false;
    gas_can_mesh_control.markers.push_back(mesh_marker);

    gas_can_mesh_control.independent_marker_orientation = false;
    gas_can_mesh_control.description = "";

    gascan_imarker.controls.push_back(gas_can_mesh_control);

    server_.insert(gascan_imarker);
    auto gascan_feedback_cb = boost::bind(&GraspingCommandPanel::processGascanMarkerFeedback, this, _1);
    server_.setCallback(gascan_imarker.name, gascan_feedback_cb);
    ROS_INFO("Inserted marker 'gas_canister_fixture'");

    server_.applyChanges();
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
    ROS_INFO("Received Result from Grasp Object Command Action");
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
    ROS_INFO("Received Result from Grasp Object Command Action");
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

    ROS_INFO("Received Result from Reposition Base Command Action");
    pending_reposition_base_command_ = false;

    if (result && result->result == cmu_manipulation_msgs::RepositionBaseCommandResult::SUCCESS) {
        candidate_base_poses_ = result->candidate_base_poses;
        ROS_INFO("Reposition Base Command returned %zd candidate poses", candidate_base_poses_.size());
    }

    updateGUI();
}

void GraspingCommandPanel::updateBasePoseSpinBoxes()
{
    disconnect(teleport_base_command_x_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseX(double)));
    disconnect(teleport_base_command_y_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseY(double)));
    disconnect(teleport_base_command_yaw_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseYaw(double)));
    teleport_base_command_x_box_->setValue(T_world_robot_.translation()(0, 0));
    teleport_base_command_y_box_->setValue(T_world_robot_.translation()(1, 0));
    double yaw, pitch, roll;
    msg_utils::get_euler_ypr(T_world_robot_, yaw, pitch, roll);
    teleport_base_command_yaw_box_->setValue(smpl::angles::to_degrees(yaw));
    connect(teleport_base_command_x_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseX(double)));
    connect(teleport_base_command_y_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseY(double)));
    connect(teleport_base_command_yaw_box_, SIGNAL(valueChanged(double)), this, SLOT(updateBasePoseYaw(double)));
}

void GraspingCommandPanel::updateGUI()
{
    ROS_INFO("    Pending Grasp Object Command: %s", pending_grasp_object_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Reposition Base Command: %s", pending_reposition_base_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Manipulate: %s", pending_manipulate_command_ ? "TRUE" : "FALSE");

    bool pending_motion_command =
        pending_grasp_object_command_      ||
        pending_reposition_base_command_ ||
        pending_manipulate_command_;

    // because actionlib isn't always as friendly as you might think
    pending_motion_command = false;

    global_frame_line_edit_->setEnabled(true);

    copy_current_base_pose_button_->setEnabled(initialized());
    teleport_base_command_x_box_->setEnabled(initialized());
    teleport_base_command_y_box_->setEnabled(initialized());
    teleport_base_command_z_box_->setEnabled(initialized());
    teleport_base_command_yaw_box_->setEnabled(initialized());

    send_grasp_object_command_button_->setEnabled(initialized() && !pending_motion_command);
    send_reposition_base_command_button_->setEnabled(initialized() && !pending_motion_command);

    std::string num_candidates_label_text =
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

void GraspingCommandPanel::updateObjectMarkerPose()
{
    visualization_msgs::InteractiveMarker object_marker;
    if (server_.get("gas_canister_fixture", object_marker)) {
        std_msgs::Header header;
        header.seq = 0;
        header.stamp = ros::Time(0);
        header.frame_id = global_frame_;
        geometry_msgs::Pose new_pose;
        tf::poseEigenToMsg(T_world_object_, new_pose);
        server_.setPose("gas_canister_fixture", new_pose, header);
        server_.applyChanges();
    }
}

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GraspingCommandPanel, rviz::Panel)
