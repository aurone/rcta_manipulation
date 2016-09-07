#include "ManipulatorCommandPanel.h"

// standard includes
#include <cassert>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <sbpl_geometry_utils/geometry.h>
#include <sensor_msgs/JointState.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/utils/utils.h>
#include <std_msgs/ColorRGBA.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shape_operations.h>

// project includes
#include <rcta/control/robotiq_controllers/gripper_model.h>

namespace hdt
{

ManipulatorCommandPanel::ManipulatorCommandPanel(QWidget *parent) :
    rviz::Panel(parent),
    nh_(),
    move_arm_command_client_(),
    pending_move_arm_command_(false),
    grasp_object_command_client_(),
    pending_grasp_object_command_(false),
    reposition_base_command_client_(),
    pending_reposition_base_command_(false),
    teleport_andalite_command_client_(),
    pending_teleport_andalite_command_(false),
    teleport_hdt_command_client_(),
    pending_teleport_hdt_command_(false),
    robot_description_line_edit_(nullptr),
    refresh_robot_desc_button_(nullptr),
    global_frame_line_edit_(nullptr),
    refresh_global_frame_button_(nullptr),
    copy_current_base_pose_button_(nullptr),
    teleport_base_command_x_box_(nullptr),
    teleport_base_command_y_box_(nullptr),
    teleport_base_command_z_box_(nullptr),
    teleport_base_command_yaw_box_(nullptr),
    send_teleport_andalite_command_button_(nullptr),
    send_grasp_object_command_button_(nullptr),
    send_reposition_base_command_button_(nullptr),
    world_to_robot_(Eigen::Affine3d::Identity()),
    world_to_object_(Eigen::Affine3d::Identity()),
    robot_model_(),
    manip_name_("right_arm"),
    rm_loader_(),
    rm_(),
    rs_(),
    server_("hdt_control"),
    joint_states_sub_(),
    robot_markers_pub_(),
    last_joint_state_(),
    tip_link_(),
    base_link_(),
    listener_(),
    robot_description_(),
    global_frame_(),
    candidate_base_poses_()
{
    setup_gui();
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &ManipulatorCommandPanel::joint_states_callback, this);
    robot_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    occupancy_grid_sub_ = nh_.subscribe("fixed_costmap_sim", 1, &ManipulatorCommandPanel::occupancy_grid_callback, this);
}

ManipulatorCommandPanel::~ManipulatorCommandPanel()
{
}

void ManipulatorCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);

    ROS_INFO("Loading config for '%s'", this->getName().toStdString().c_str());

    QString global_frame, robot_description;
    bool use_global_frame;
    float base_x, base_y, base_yaw;
    float object_x, object_y, object_yaw;
    config.mapGetString("global_frame", &global_frame);
    config.mapGetString("robot_description", &robot_description);
    config.mapGetBool("use_global_frame", &use_global_frame);
    config.mapGetFloat("base_x", &base_x);
    config.mapGetFloat("base_y", &base_y);
    config.mapGetFloat("base_yaw", &base_yaw);
    config.mapGetFloat("object_x", &object_x);
    config.mapGetFloat("object_y", &object_y);
    config.mapGetFloat("object_yaw", &object_yaw);

    ROS_INFO("Robot Description: %s", robot_description.toStdString().c_str());
    ROS_INFO("Use Global Frame: %s", use_global_frame ? "true" : "false");
    ROS_INFO("Global Frame: %s", global_frame.toStdString().c_str());

    if (!robot_description.isEmpty()) {
        // attempt to initalize the robot via this robot description
        std::string why;
        if (!set_robot_description(robot_description.toStdString(), why)) {
            QMessageBox::warning(
                    this,
                    tr("Config Failure"),
                    tr("Failed to load 'robot_description' from panel config (%1)").arg(QString::fromStdString(why)));
        }
    }

    // Update checkbox without firing off signals
    disconnect(global_frame_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(check_use_global_frame(int)));
    global_frame_checkbox_->setChecked(use_global_frame);
    connect(global_frame_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(check_use_global_frame(int)));

    set_use_global_frame(use_global_frame);

    if (!global_frame.isEmpty()) {
        std::string why;
        if (!set_global_frame(global_frame.toStdString(), why)) {
            QMessageBox::warning(
                    this,
                    tr("Config Failure"),
                    tr("Failed to load 'global_frame' from panel config (%1)").arg(QString::fromStdString(why)));
        }
    }

    world_to_robot_ = Eigen::Translation3d(base_x, base_y, 0.0) * Eigen::AngleAxisd(base_yaw, Eigen::Vector3d(0, 0, 1));
    world_to_object_ = Eigen::Translation3d(object_x, object_y, 0.0) *
            Eigen::AngleAxisd(object_yaw, Eigen::Vector3d(0, 0, 1));

    update_manipulator_marker_pose();
    update_object_marker_pose();

    update_base_pose_spinboxes();
    update_gui();
}

void ManipulatorCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    ROS_INFO("Saving config for '%s'", this->getName().toStdString().c_str());

    config.mapSetValue("robot_description", QString::fromStdString(this->get_robot_description()));
    config.mapSetValue("use_global_frame", use_global_frame());
    config.mapSetValue("global_frame", QString::fromStdString(this->get_global_frame()));
    config.mapSetValue("base_x", robot_transform().translation()(0, 0));
    config.mapSetValue("base_y", robot_transform().translation()(1, 0));
    double yaw, pitch, roll;
    msg_utils::get_euler_ypr(robot_transform(), yaw, pitch, roll);
    config.mapSetValue("base_yaw", yaw);
    config.mapSetValue("object_x", object_transform().translation()(0, 0));
    config.mapSetValue("object_y", object_transform().translation()(1, 0));
    msg_utils::get_euler_ypr(object_transform(), yaw, pitch, roll);
    config.mapSetValue("object_yaw", yaw);
}

void ManipulatorCommandPanel::refresh_robot_description()
{
    std::string user_robot_description = robot_description_line_edit_->text().toStdString();
    if (user_robot_description.empty()) {
        QMessageBox::information(this, tr("Robot Description"), tr("Please enter a valid ROS parameter for the URDF"));
        return;
    }

    std::string why;
    if (!set_robot_description(user_robot_description, why)) {
        QMessageBox::warning(
                this,
                tr("Robot Description"),
                tr("Failed to set the robot description to '%1' (%2)")
                    .arg(QString::fromStdString(user_robot_description), QString::fromStdString(why)));
    }

    update_gui();
}

void ManipulatorCommandPanel::refresh_global_frame()
{
    std::string user_global_frame = global_frame_line_edit_->text().toStdString();

    std::string why;
    if (!set_global_frame(user_global_frame, why)) {
        QMessageBox::warning(
                this,
                tr("Global Frame"),
                tr("Failed to set the global frame to  '%1' (%2)")
                    .arg(QString::fromStdString(user_global_frame), QString::fromStdString(why)));
    }

    update_gui();
}

void ManipulatorCommandPanel::check_use_global_frame(int state)
{
    set_use_global_frame(state);
    update_gui();
}

void ManipulatorCommandPanel::copy_current_base_pose()
{
    tf::StampedTransform world_to_robot;
    listener_.lookupTransform(get_global_frame(), rm_->getModelFrame(), ros::Time(0), world_to_robot);
    msg_utils::convert(world_to_robot, world_to_robot_);
    double roll, pitch, yaw;
    msg_utils::get_euler_ypr(world_to_robot_, yaw, pitch, roll);
    Eigen::Vector3d robot_pos(world_to_robot_.translation());
    teleport_base_command_x_box_->setValue(robot_pos.x());
    teleport_base_command_y_box_->setValue(robot_pos.y());
    teleport_base_command_z_box_->setValue(robot_pos.z());
    teleport_base_command_yaw_box_->setValue(sbpl::utils::ToDegrees(yaw));
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::update_base_pose_x(double x)
{
    world_to_robot_.translation()(0, 0) = x;
    update_manipulator_marker_pose();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::update_base_pose_y(double y)
{
    world_to_robot_.translation()(1, 0) = y;
    update_manipulator_marker_pose();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::update_base_pose_z(double z)
{
    world_to_robot_.translation()(2, 0) = z;
    update_manipulator_marker_pose();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::update_base_pose_yaw(double yaw_deg)
{
    double yaw_rad = sbpl::utils::ToRadians(yaw_deg);
    world_to_robot_ =
            Eigen::Translation3d(world_to_robot_.translation()) *
            Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d(0.0, 0.0, 1.0));
    update_manipulator_marker_pose();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::update_base_pose_candidate(int index)
{
    if (index > 0) {
        base_candidate_idx_ = index - 1;
        assert(base_candidate_idx_ >= 0 && base_candidate_idx_ < candidate_base_poses_.size());

        tf::poseMsgToEigen(candidate_base_poses_[base_candidate_idx_].pose, world_to_robot_);

        update_base_pose_spinboxes();
        update_manipulator_marker_pose();
        publish_phantom_robot_visualizations();
    }
}

void ManipulatorCommandPanel::send_teleport_andalite_command()
{
    if (!reconnect_client(teleport_andalite_command_client_, "teleport_andalite_command")) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Teleport Andalite Command (server is not connected)"));
        return;
    }

    rcta::TeleportAndaliteCommandGoal teleport_andalite_goal;
    teleport_andalite_goal.global_pose.header.seq = 0;
    teleport_andalite_goal.global_pose.header.stamp = ros::Time::now();
    teleport_andalite_goal.global_pose.header.frame_id = interactive_marker_frame();
    tf::poseEigenToMsg(world_to_robot_, teleport_andalite_goal.global_pose.pose);

    auto result_cb = boost::bind(&ManipulatorCommandPanel::teleport_andalite_command_result_cb, this, _1, _2);
    teleport_andalite_command_client_->sendGoal(teleport_andalite_goal, result_cb);

    pending_teleport_andalite_command_ = true;
}

void ManipulatorCommandPanel::send_grasp_object_command()
{
    if (!reconnect_client(grasp_object_command_client_, "grasp_object_command")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Grasp Object Command (server is not connected)"));
        return;
    }

    const std::string gas_can_interactive_marker_name = "gas_canister_fixture";
    visualization_msgs::InteractiveMarker gas_can_interactive_marker;
    if (!server_.get(gas_can_interactive_marker_name, gas_can_interactive_marker)) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Grasp Object Command (no interactive marker named 'gas_canister_fixture'"));
        return;
    }

    rcta_msgs::GraspObjectCommandGoal grasp_object_goal;

    static int grasp_object_goal_id = 0;
    grasp_object_goal.id = grasp_object_goal_id++;
    grasp_object_goal.retry_count = 0;

    // robot -> object = robot -> marker * marker -> object
    Eigen::Affine3d robot_to_object = robot_transform().inverse() * object_transform();
    grasp_object_goal.gas_can_in_base_link.header.frame_id = rm_->getModelFrame();
    tf::poseEigenToMsg(robot_to_object, grasp_object_goal.gas_can_in_base_link.pose);

    ROS_INFO("Robot -> Marker: %s", to_string(robot_transform().inverse()).c_str());
    ROS_INFO("Marker -> Object: %s", to_string(object_transform()).c_str());
    ROS_INFO("Robot -> Object: %s", to_string(robot_to_object).c_str());

    grasp_object_goal.gas_can_in_map.header.frame_id = interactive_marker_frame();
    tf::poseEigenToMsg(object_transform(), grasp_object_goal.gas_can_in_map.pose);

    auto result_callback = boost::bind(&ManipulatorCommandPanel::grasp_object_command_result_cb, this, _1, _2);
    grasp_object_command_client_->sendGoal(grasp_object_goal, result_callback);

    pending_grasp_object_command_ = true;
    update_gui();
}

void ManipulatorCommandPanel::send_reposition_base_command()
{
    if (!reconnect_client(reposition_base_command_client_, "reposition_base_command")) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Unable to send Reposition Base Command (server is not connected)"));
        return;
    }

    rcta_msgs::RepositionBaseCommandGoal reposition_base_goal;

    static int reposition_base_goal_id = 0;
    reposition_base_goal.id = reposition_base_goal_id++;
    reposition_base_goal.retry_count = 0;

    tf::poseEigenToMsg(world_to_object_, reposition_base_goal.gas_can_in_map.pose);
    reposition_base_goal.gas_can_in_map.header.frame_id = interactive_marker_frame();

    tf::poseEigenToMsg(world_to_robot_, reposition_base_goal.base_link_in_map.pose);
    reposition_base_goal.base_link_in_map.header.frame_id = interactive_marker_frame();

    // TODO: occupancy grid
    if (last_occupancy_grid_msg_) {
    	ROS_INFO("Sending occupancy grid to sung");
    	reposition_base_goal.map = *last_occupancy_grid_msg_;
    }

    auto result_callback = boost::bind(&ManipulatorCommandPanel::reposition_base_command_result_cb, this, _1, _2);
    reposition_base_command_client_->sendGoal(reposition_base_goal, result_callback);

    pending_reposition_base_command_ = true;
    update_gui();
}

void ManipulatorCommandPanel::setup_gui()
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    // general settings
    QGroupBox* general_settings_group = new QGroupBox(tr("General Settings"));
        QVBoxLayout* general_settings_layout = new QVBoxLayout;
            QHBoxLayout* robot_description_layout = new QHBoxLayout;
                QLabel* robot_description_label = new QLabel(tr("Robot Description:"));
                robot_description_line_edit_ = new QLineEdit;
                refresh_robot_desc_button_ = new QPushButton(tr("Refresh"));
            robot_description_layout->addWidget(robot_description_label);
            robot_description_layout->addWidget(robot_description_line_edit_);
            robot_description_layout->addWidget(refresh_robot_desc_button_);
            QHBoxLayout* global_frame_layout = new QHBoxLayout;
                global_frame_checkbox_ = new QCheckBox(tr("Use Global Frame"));
                global_frame_checkbox_->setChecked(false);
                QLabel* global_frame_label = new QLabel(tr("Global Frame:"));
                global_frame_line_edit_ = new QLineEdit;
                global_frame_line_edit_->setEnabled(false);
                refresh_global_frame_button_ = new QPushButton(tr("Refresh"));
            global_frame_layout->addWidget(global_frame_checkbox_);
            global_frame_layout->addWidget(global_frame_label);
            global_frame_layout->addWidget(global_frame_line_edit_);
            global_frame_layout->addWidget(refresh_global_frame_button_);
        general_settings_layout->addLayout(robot_description_layout);
        general_settings_layout->addLayout(global_frame_layout);
    general_settings_group->setLayout(general_settings_layout);

    // base commands
    QGroupBox* base_commands_group = new QGroupBox(tr("Base Commands"));
        QVBoxLayout* base_commands_layout = new QVBoxLayout;
            copy_current_base_pose_button_ = new QPushButton(tr("Copy Current Base Pose"));
            QHBoxLayout* base_pose_spinbox_layout = new QHBoxLayout;
                QLabel* x_label = new QLabel(tr("X:"));
                teleport_base_command_x_box_ = new QDoubleSpinBox;
                teleport_base_command_x_box_->setMinimum(-100.0);
                teleport_base_command_x_box_->setMaximum(100.0);
                teleport_base_command_x_box_->setSingleStep(0.05);
                QLabel* y_label = new QLabel(tr("Y:"));
                teleport_base_command_y_box_ = new QDoubleSpinBox;
                teleport_base_command_y_box_->setMinimum(-100.0);
                teleport_base_command_y_box_->setMaximum(100.0);
                teleport_base_command_y_box_->setSingleStep(0.05);
                QLabel* z_label = new QLabel(tr("Z:"));
                teleport_base_command_z_box_ = new QDoubleSpinBox;
                QLabel* yaw_label = new QLabel(tr("Yaw:"));
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
            send_teleport_andalite_command_button_ = new QPushButton(tr("Teleport Andalite"));
        base_commands_layout->addWidget(copy_current_base_pose_button_);
        base_commands_layout->addLayout(base_pose_spinbox_layout);
        base_commands_layout->addWidget(send_teleport_andalite_command_button_);
    base_commands_group->setLayout(base_commands_layout);

    // object interaction commands
    QGroupBox* object_interaction_commands_group = new QGroupBox(tr("Object Interaction Commands"));
        QVBoxLayout* object_interaction_commands_layout = new QVBoxLayout;
            send_grasp_object_command_button_ = new QPushButton(tr("Grasp Object"));
            send_reposition_base_command_button_ = new QPushButton(tr("Reposition Base"));
            QHBoxLayout* candidates_layout = new QHBoxLayout;
                update_candidate_spinbox_ = new QSpinBox;
                update_candidate_spinbox_->setEnabled(false);
                num_candidates_label_ = new QLabel(tr("of 0 Candidates"));
            candidates_layout->addWidget(update_candidate_spinbox_);
            candidates_layout->addWidget(num_candidates_label_);
        object_interaction_commands_layout->addWidget(send_grasp_object_command_button_);
        object_interaction_commands_layout->addWidget(send_reposition_base_command_button_);
        object_interaction_commands_layout->addLayout(candidates_layout);
    object_interaction_commands_group->setLayout(object_interaction_commands_layout);

    main_layout->addWidget(general_settings_group);
    main_layout->addWidget(base_commands_group);
    main_layout->addWidget(object_interaction_commands_group);
    setLayout(main_layout);

    // note: do not connect any outgoing signals from general settings line edits; force users to use refresh button
    connect(refresh_robot_desc_button_, SIGNAL(clicked()), this, SLOT(refresh_robot_description()));
    connect(refresh_global_frame_button_, SIGNAL(clicked()), this, SLOT(refresh_global_frame()));
    connect(global_frame_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(check_use_global_frame(int)));

    // base commands
    connect(copy_current_base_pose_button_, SIGNAL(clicked()), this, SLOT(copy_current_base_pose()));
    connect(teleport_base_command_x_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_x(double)));
    connect(teleport_base_command_y_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_y(double)));
    connect(teleport_base_command_z_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_z(double)));
    connect(teleport_base_command_yaw_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_yaw(double)));
    connect(send_teleport_andalite_command_button_, SIGNAL(clicked()), this, SLOT(send_teleport_andalite_command()));

    // object interaction commands
    connect(send_grasp_object_command_button_, SIGNAL(clicked()), this, SLOT(send_grasp_object_command()));
    connect(send_reposition_base_command_button_, SIGNAL(clicked()), this, SLOT(send_reposition_base_command()));
    connect(update_candidate_spinbox_, SIGNAL(valueChanged(int)), this, SLOT(update_base_pose_candidate(int)));
}

bool ManipulatorCommandPanel::check_robot_model_consistency(
    const hdt::RobotModel& hdt_model,
    const robot_model::RobotModel& moveit_model) const
{
    for (const std::string& joint_name : hdt_model.joint_names()) {
        if (!moveit_model.hasJointModel(joint_name)) {
            ROS_ERROR("MoveIt Robot Model does not contain joint %s", joint_name.c_str());
            return false;
        }
    }

    for (std::size_t i = 1; i < hdt_model.joint_names().size(); ++i) {
        const std::string& parent_joint_name = hdt_model.joint_names()[i - 1];
        const std::string& child_joint_name = hdt_model.joint_names()[i];

        std::vector<const robot_model::JointModel*> child_joints;
        child_joints = moveit_model.getJointModel(parent_joint_name)->getChildLinkModel()->getChildJointModels();
        if (child_joints.size() != 1) {
            ROS_ERROR("Unexpected number of child joints (%zd)", child_joints.size());
            return false;
        }

        if (child_joints.front()->getName() != child_joint_name) {
            ROS_ERROR("Child of joint %s is not %s in the MoveIt model", parent_joint_name.c_str(), child_joint_name.c_str());
            return false;
        }
    }

    return true;
}

bool ManipulatorCommandPanel::set_robot_description(const std::string& robot_description, std::string& why)
{
    // attempt to reinitialize the robot from this robot description
    if (reinit(robot_description, why)) {
        ROS_INFO("Successfully reinitialized robot from '%s'", robot_description.c_str());

        if (robot_description_ != robot_description) { // robot description changed to something different, not just refreshed
            Q_EMIT(configChanged());
        }

        robot_description_ = robot_description;
        // note: ok to do this here without disabling signals since set_robot_description is invoked via refresh button callback
        robot_description_line_edit_->setText(QString::fromStdString(robot_description_));

        ROS_INFO("Robot Description set to '%s'", robot_description.c_str());
        return true;
    }
    else {
        QMessageBox::warning(
                this, tr("Refresh Robot Description"), tr("Failed to Reinitialize (%1)").arg(QString::fromStdString(why)));
        robot_description_line_edit_->setText(QString::fromStdString(robot_description_));
        return false;
    }
}

bool ManipulatorCommandPanel::set_global_frame(const std::string& global_frame, std::string& why)
{
    if (!valid_global_frame(global_frame)) {
        why = global_frame + " is not a valid frame";
        global_frame_line_edit_->setText(QString::fromStdString(global_frame_));
        return false;
    }

    // TODO: consider transforming the robot and object transforms so that they remain in the same positions

    if (global_frame != global_frame_) {
        Q_EMIT(configChanged());
    }

    global_frame_ = global_frame;
    global_frame_line_edit_->setText(QString::fromStdString(global_frame_));

    update_manipulator_marker_pose();
    update_object_marker_pose();

    ROS_INFO("Global Frame set to %s", global_frame.c_str());
    return true;
}

const std::string& ManipulatorCommandPanel::get_robot_description() const
{
    return robot_description_;
}

const std::string& ManipulatorCommandPanel::get_global_frame() const
{
    return global_frame_;
}

bool ManipulatorCommandPanel::use_global_frame() const
{
    return use_global_frame_;
}

void ManipulatorCommandPanel::set_use_global_frame(bool use)
{
    use_global_frame_ = use;
    update_manipulator_marker_pose();
    update_object_marker_pose();
    publish_phantom_robot_visualizations();
    ROS_INFO("Use Global Frame set to %s", use ? "true" : "false");
}

bool ManipulatorCommandPanel::global_frame_active() const
{
    return use_global_frame() && !get_global_frame().empty();
}

std::string ManipulatorCommandPanel::interactive_marker_frame() const
{
    if (global_frame_active()) {
        return get_global_frame();
    }
    else if (rm_) {
        return rm_->getModelFrame();
    }
    else {
        return std::string("");
    }
}

bool ManipulatorCommandPanel::valid_global_frame(const std::string& frame) const
{
    static std::vector<std::string> valid_global_frames = {
            "abs_nwu",
            "abs_ned",
            "/abs_nwu",
            "/abs_ned"

    }; // TODO: please do something more intelligent like look for possible global frames
    return std::find(valid_global_frames.begin(), valid_global_frames.end(), frame) != valid_global_frames.end();
}

const Eigen::Affine3d ManipulatorCommandPanel::robot_transform() const
{
    if (global_frame_active()) {
        return world_to_robot_;
    }
    else {
        return Eigen::Affine3d::Identity();
    }
}

const Eigen::Affine3d ManipulatorCommandPanel::object_transform() const
{
    return world_to_object_;
}

bool ManipulatorCommandPanel::initialized() const
{
    return (bool)(robot_model_);
}

void ManipulatorCommandPanel::do_process_feedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    Eigen::Affine3d new_eef_pose;
    tf::poseMsgToEigen(feedback->pose, new_eef_pose);

    ROS_DEBUG("Marker is in frame %s", feedback->header.frame_id.c_str());

    ROS_DEBUG("Base -> Robot: %s", to_string(rs_->getFrameTransform(base_link_).inverse()).c_str());
    ROS_DEBUG("Robot -> World: %s", to_string(world_to_robot_.inverse()).c_str());
    ROS_DEBUG("World -> End Effector: %s", to_string(new_eef_pose).c_str());

    // (robot -> eef) = (root -> global) * (global -> eef)
    new_eef_pose = robot_transform().inverse() * new_eef_pose;

    ROS_DEBUG("Robot -> End Effector: %s", to_string(new_eef_pose).c_str());

    const auto* jmg = rm_->getJointModelGroup(manip_name_);
    if (jmg) {
        rs_->setFromIK(jmg, new_eef_pose);
        std::vector<double> solution;
        rs_->copyJointGroupPositions(jmg, solution);
        if (!set_phantom_joint_angles(solution)) {
            QMessageBox::warning(
                    this,
                    tr("Interactive Marker Feedback"),
                    tr("Failed to set phantom state from interactive marker-driven IK"));
        }
    }
    else {
        QMessageBox::warning(
                this,
                tr("Interactive Marker Feedback"),
                tr("No joint model group %1 found in Robot Model").arg(QString::fromStdString(manip_name_)));
    }

    // update the position of the marker (in case of ik failure and for synchronization)
    update_manipulator_marker_pose();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::process_gas_canister_marker_feedback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        tf::poseMsgToEigen(feedback->pose, world_to_object_);
    }
}

void ManipulatorCommandPanel::publish_phantom_robot_visualizations()
{
    if (!initialized()) {
        return;
    }

    rs_->updateLinkTransforms();

    const std::vector<std::string>& link_names = rm_->getLinkModelNames();
    std_msgs::ColorRGBA color = std_msgs::CreateColorRGBA(0.94, 0.44, 0.44, 1.0);
    std::string ns = "phantom_robot_link";
    ros::Duration d(0);

    visualization_msgs::MarkerArray marker_array;
    gatherRobotMarkers(*rs_, link_names, color, ns, d, marker_array);

    // transform all markers from the robot frame into the global frame
    for (visualization_msgs::Marker& marker : marker_array.markers) {
        Eigen::Affine3d root_to_marker;
        tf::poseMsgToEigen(marker.pose, root_to_marker);
        // world -> marker = world -> robot * robot -> marker
        Eigen::Affine3d world_to_marker = robot_transform() * root_to_marker;
        tf::poseEigenToMsg(world_to_marker, marker.pose);
        marker.header.frame_id = interactive_marker_frame();
        marker.header.stamp = ros::Time(0);
    }

    robot_markers_pub_.publish(marker_array);
}

std::string ManipulatorCommandPanel::get_tip_link(const robot_model::JointModelGroup& joint_model_group) const
{
    const auto& joint_roots = joint_model_group.getJointRoots();
    if (joint_roots.size() == 1) {
        const robot_model::JointModel* joint = joint_roots.front();
        bool advance = false;
        do
        {
            advance = false;
            // advance until the child link is the last link in the chain
            const auto& child_link = joint->getChildLinkModel();
            if (child_link && child_link->getChildJointModels().size() == 1 &&
                joint_model_group.hasJointModel(child_link->getChildJointModels().front()->getName()))
            {
                joint = joint->getChildLinkModel()->getChildJointModels().front();
                advance = true;
            }
        }
        while (advance);
        return joint->getChildLinkModel()->getName();
    }
    else {
        return std::string("");
    }
}

std::string ManipulatorCommandPanel::get_base_link(
    const robot_model::JointModelGroup& joint_model_group) const
{
    const std::vector<const robot_model::JointModel*>& joint_roots = joint_model_group.getJointRoots();
    if (joint_roots.size() == 1) {
        const robot_model::JointModel* root_joint = joint_roots.front();
        return root_joint->getParentLinkModel()->getName();
    }
    else {
        return std::string();
    }
}

std::vector<visualization_msgs::InteractiveMarkerControl>
ManipulatorCommandPanel::create_sixdof_controls() const
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

bool ManipulatorCommandPanel::get_joint_value(
    const sensor_msgs::JointState& joint_state,
    const std::string& joint,
    double& joint_value) const
{
    if (joint_state.name.size() != joint_state.position.size()) {
        ROS_ERROR("Number of joint names and joint positions differ");
        return false;
    }

    for (int i = 0; i < (int)joint_state.name.size(); ++i) {
        if (joint_state.name[i] == joint) {
            joint_value = joint_state.position[i];
            return true;
        }
    }

    return false;
}

void ManipulatorCommandPanel::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!initialized()) {
        return;
    }

    const std::vector<std::string>& joint_names = rm_->getJointModelNames();

    sensor_msgs::JointState last_joint_state_copy = last_joint_state_;

    last_joint_state_.name = joint_names;
    last_joint_state_.position.resize(joint_names.size());
    last_joint_state_.header = msg->header;

    for (int i = 0; i < (int)joint_names.size(); ++i) {
        // find the ith joint in the message and update our last joint state
        bool found = false;
        double j;
        if (get_joint_value(*msg, joint_names[i], j)) {
            // check this message for the i'th joint
            last_joint_state_.position[i] = j;
        }
        else if (get_joint_value(last_joint_state_copy, joint_names[i], j)) {
            // check the last message for the i'th joint
            last_joint_state_.position[i] = j;
        }
        else {
            // haven't received information for this joint yet
            last_joint_state_.position[i] = 0.0;
        }
    }
}

void ManipulatorCommandPanel::occupancy_grid_callback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	last_occupancy_grid_msg_ = msg;
}

bool ManipulatorCommandPanel::reinit(
    const std::string& robot_description,
    std::string& why)
{
    if (!reinit_robot_models(robot_description, why)) {
        return false;
    }

    if (!reinit_interactive_marker_server()) {
        return false;
    }

    publish_phantom_robot_visualizations();
    return true;
}

bool ManipulatorCommandPanel::reinit_robot_models(
    const std::string& robot_description,
    std::string& why)
{
    if (!nh_.hasParam(robot_description) || !nh_.hasParam(robot_description + "_semantic")) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << robot_description << "' and '" << (robot_description + "_semantic") << "' from the param server";
        why = ss.str();
        return false;
    }

    std::string urdf_string;
    if (!nh_.getParam(robot_description, urdf_string)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << robot_description << "' from the param server";
        why = ss.str();
        return false;
    }

    hdt::RobotModelPtr robot_model = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model) {
        why = "Failed to load robot model from the URDF";
        return false;
    }

    robot_model_loader::RobotModelLoaderPtr rm_loader(new robot_model_loader::RobotModelLoader(robot_description, true));
    if (!rm_loader) {
        why = "Failed to instantiate Robot Model Loader";
        return false;
    }

    robot_model::RobotModelPtr rm = rm_loader->getModel();
    if (!rm) {
        why = ("Robot Model Loader was unable to construct Robot Model");
        return false;
    }

    if (!check_robot_model_consistency(*robot_model, *rm)) {
        why = "Robot models are not consistent";
        return false;
    }

    robot_state::RobotStatePtr rs(new robot_state::RobotState(rm));
    if (!rs) {
        why = "Failed to instantiate Robot State";
        return false;
    }

    // All lights are green from above
    robot_model_ = robot_model;
    rm_loader_ = rm_loader;
    rm_ = rm;
    rs_ =  rs;

    ROS_INFO("MoveIt model is consistent with HDT Robot Model");

    ROS_INFO("Root link name: %s", rm_->getRootLinkName().c_str());
    ROS_INFO("Robot Joints:");
    for (const std::string& joint_name : rm_->getJointModelNames()) {
        ROS_INFO("    %s", joint_name.c_str());
    }

    rs_->setToDefaultValues();
//    rs_->setRootTransform(Eigen::Affine3d::Identity()); // TODO: equivalent
    rs_->updateLinkTransforms();

    return true;
}

bool ManipulatorCommandPanel::reinit_interactive_marker_server()
{
    return reinit_manipulator_interactive_marker() && reinit_object_interactive_marker();
}

bool ManipulatorCommandPanel::reinit_manipulator_interactive_marker()
{
    if (!rm_) {
        ROS_ERROR("Failed to reinitialize manipulator interactive marker. Robot Model is null");
        return false;
    }

    // initialize an interactive marker for the first joint group that's a kinematic chain
    for (const auto& joint_model_group_name : rm_->getJointModelGroupNames()) {
        const robot_model::JointModelGroup* jmg = rm_->getJointModelGroup(joint_model_group_name);
        if (jmg->isChain()) {
            // find the tip link of the arm and the link to which the arm is attached
            tip_link_ = get_tip_link(*jmg);
            base_link_ = get_base_link(*jmg);
            ROS_INFO("Found manipulator attached to %s with tip link %s", base_link_.c_str(), tip_link_.c_str());

            ROS_DEBUG("Joint Group %s:", jmg->getName().c_str());
            ROS_DEBUG("    Joints:");
            for (const std::string& joint_name : jmg->getJointModelNames()) {
                ROS_DEBUG("        %s", joint_name.c_str());
            }
            ROS_DEBUG("    Links:");
            for (const std::string& link_name : jmg->getLinkModelNames()) {
                ROS_DEBUG("        %s", link_name.c_str());
            }

            // insert an interactive marker for the tip link of the arm
            visualization_msgs::InteractiveMarker interactive_marker;
            interactive_marker.header.seq = 0;
            interactive_marker.header.stamp = ros::Time(0);
            interactive_marker.header.frame_id = interactive_marker_frame();
            const robot_state::LinkModel* tip_link_model = rs_->getLinkModel(tip_link_);
            if (!tip_link_model) {
                ROS_ERROR("Failed to get Link State for tip link '%s'", tip_link_.c_str());
                return false;
            }

            Eigen::Affine3d world_to_marker = robot_transform() * rs_->getGlobalLinkTransform(tip_link_);
            tf::poseEigenToMsg(world_to_marker, interactive_marker.pose);
            interactive_marker.name = jmg->getName() + "_control";
            interactive_marker.description = std::string("Control of ") + tip_link_ + std::string(" of manipulator ") + jmg->getName();
            interactive_marker.scale = 0.25f;
            interactive_marker.menu_entries.clear();
            interactive_marker.controls.clear();
            interactive_marker.controls = create_sixdof_controls();

            ROS_INFO("Inserting interactive marker '%s'", interactive_marker.name.c_str());
            server_.insert(interactive_marker);
            server_.setCallback(interactive_marker.name, boost::bind(&ManipulatorCommandPanel::do_process_feedback, this, _1));
            break;
        }
    }

    server_.applyChanges();
    return true;
}

bool ManipulatorCommandPanel::reinit_object_interactive_marker()
{
    ROS_INFO("Inserting marker 'gas_canister_fixture'");

    // initializer an interactive marker for the gas canister object
    visualization_msgs::InteractiveMarker gas_canister_interactive_marker;
    gas_canister_interactive_marker.header.seq = 0;
    gas_canister_interactive_marker.header.stamp = ros::Time(0);
    gas_canister_interactive_marker.header.frame_id = interactive_marker_frame();
    gas_canister_interactive_marker.pose.position.x = 0.0;
    gas_canister_interactive_marker.pose.position.y = 0.0;
    gas_canister_interactive_marker.pose.position.z = 0.0;
    gas_canister_interactive_marker.pose.orientation.w = 1.0;
    gas_canister_interactive_marker.pose.orientation.x = 0.0;
    gas_canister_interactive_marker.pose.orientation.y = 0.0;
    gas_canister_interactive_marker.pose.orientation.z = 0.0;
    gas_canister_interactive_marker.name = "gas_canister_fixture";
    gas_canister_interactive_marker.description = "Gas Canister Positioning";
    gas_canister_interactive_marker.scale = 0.25f;
    gas_canister_interactive_marker.menu_entries.clear();
    gas_canister_interactive_marker.controls.clear();

    gas_canister_interactive_marker.controls = create_sixdof_controls();

    // Construct mesh control
    visualization_msgs::InteractiveMarkerControl gas_can_mesh_control;
    gas_can_mesh_control.name = "mesh_control";
    gas_can_mesh_control.orientation = geometry_msgs::IdentityQuaternion();
    gas_can_mesh_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    gas_can_mesh_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    gas_can_mesh_control.always_visible = true;

    // TODO: somehow grab the gas canister mesh and scale from the parameter server (does this mean those parameters have to be global?)

    visualization_msgs::Marker mesh_marker;
    mesh_marker.header.seq = 0;
    mesh_marker.header.stamp = ros::Time(0);
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
    mesh_marker.points.clear();
    mesh_marker.colors.clear();
    mesh_marker.text = "";
    mesh_marker.mesh_resource = "package://hdt/resource/meshes/gastank/rcta_gastank.ply";
    mesh_marker.mesh_use_embedded_materials = false;
    gas_can_mesh_control.markers.push_back(mesh_marker);
    gas_canister_interactive_marker.controls.push_back(gas_can_mesh_control);

    gas_can_mesh_control.independent_marker_orientation = false;
    gas_can_mesh_control.description = "";
    //

    server_.insert(gas_canister_interactive_marker);
    auto gascan_feedback_cb = boost::bind(&ManipulatorCommandPanel::process_gas_canister_marker_feedback, this, _1);
    server_.setCallback(gas_canister_interactive_marker.name, gascan_feedback_cb);
    ROS_INFO("Inserted marker 'gas_canister_fixture'");

    server_.applyChanges();
    return true;
}

void ManipulatorCommandPanel::move_arm_command_active_cb()
{

}

void ManipulatorCommandPanel::move_arm_command_feedback_cb(const rcta::MoveArmCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::MoveArmCommandResult::ConstPtr& result)
{
    pending_move_arm_command_ = false;
    update_gui();
}

void ManipulatorCommandPanel::grasp_object_command_active_cb()
{

}

void ManipulatorCommandPanel::grasp_object_command_feeback_cb(const rcta_msgs::GraspObjectCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::grasp_object_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta_msgs::GraspObjectCommandResult::ConstPtr& result)
{
    ROS_INFO("Received Result from Grasp Object Command Action");
    pending_grasp_object_command_ = false;
    update_gui();
}


void ManipulatorCommandPanel::reposition_base_command_active_cb()
{

}

void ManipulatorCommandPanel::reposition_base_command_feedback_cb(const rcta_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::reposition_base_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const rcta_msgs::RepositionBaseCommandResult::ConstPtr& result)
{

    ROS_INFO("Received Result from Reposition Base Command Action");
    pending_reposition_base_command_ = false;

    if (result->result == rcta_msgs::RepositionBaseCommandResult::SUCCESS) {
        candidate_base_poses_ = result->candidate_base_poses;
        ROS_INFO("Reposition Base Command returned %zd candidate poses", candidate_base_poses_.size());
    }

    update_gui();
}

void ManipulatorCommandPanel::teleport_andalite_command_active_cb()
{

}

void ManipulatorCommandPanel::teleport_andalite_command_feedback_cb(const rcta::TeleportAndaliteCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::teleport_andalite_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::TeleportAndaliteCommandResult::ConstPtr& result)
{
    ROS_INFO("Received Result from Teleport Andalite Command Action Client");
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Teleport Andalite Command goal was not successful"));
    }

    pending_teleport_andalite_command_ = false;
}

void ManipulatorCommandPanel::teleport_hdt_command_active_cb()
{

}

void ManipulatorCommandPanel::teleport_hdt_command_feedback_cb(const rcta::TeleportHDTCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::teleport_hdt_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::TeleportHDTCommandResult::ConstPtr& result)
{
    ROS_INFO("Received Result from Teleport HDT Command Action Client");
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        QMessageBox::warning(this, tr("Command Failure"), tr("Teleport HDT Command goal was not successful"));
    }

    pending_teleport_hdt_command_ = false;
}

bool ManipulatorCommandPanel::gatherRobotMarkers(
    const robot_state::RobotState& robot_state,
    const std::vector<std::string>& link_names,
    const std_msgs::ColorRGBA& color,
    const std::string& ns,
    const ros::Duration& d,
    visualization_msgs::MarkerArray& markers,
    bool include_attached)
{
    ////////////////////////////////////////////////////////////////////////////////
    // Method derived from moveit_ros_planning/robot_state.cpp on groovy-devel
    // branch version a5f7c1c728
    ////////////////////////////////////////////////////////////////////////////////

    std::size_t num_orig_markers = markers.markers.size();

    ros::Time tm = ros::Time::now();
    for (std::size_t i = 0; i < link_names.size(); ++i) {
        visualization_msgs::Marker mark;
        const robot_state::LinkModel* lm = rs_->getLinkModel(link_names[i]);
        if (!lm) {
            continue;
        }

        // add markers for attached objects
        if (include_attached) {
            std::vector<const robot_state::AttachedBody*> attached_bodies;
            rs_->getAttachedBodies(attached_bodies, lm);
            for (std::size_t j = 0; j < attached_bodies.size(); ++j) {
                if (attached_bodies[j]->getShapes().size() > 0) {
                    visualization_msgs::Marker att_mark;
                    att_mark.header.frame_id = robot_state.getRobotModel()->getModelFrame();
                    att_mark.header.stamp = tm;
                    shapes::constructMarkerFromShape(attached_bodies[j]->getShapes()[0].get(), att_mark);
                    tf::poseEigenToMsg(attached_bodies[j]->getGlobalCollisionBodyTransforms()[0], att_mark.pose);
                    markers.markers.push_back(att_mark);
                }
            }
        }

        if (!lm || lm->getShapes().empty()) {
            continue;
        }

        mark.header.frame_id = robot_state.getRobotModel()->getModelFrame();
        mark.header.stamp = tm;
        // TODO: other collision bodies
        tf::poseEigenToMsg(rs_->getCollisionBodyTransform(link_names[i], 0), mark.pose);

        // we prefer using the visual mesh, if a mesh is available
        const std::string& mesh_resource = lm->getVisualMeshFilename();
        if (mesh_resource.empty()) {
            if (!shapes::constructMarkerFromShape(lm->getShapes().front().get(), mark)) {
                continue;
            }

            // if the object is invisible (0 volume) we skip it
            if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon()) {
                continue;
            }
        }
        else {
            tf::poseEigenToMsg(rs_->getGlobalLinkTransform(link_names[i]), mark.pose);

            mark.type = mark.MESH_RESOURCE;
            mark.mesh_use_embedded_materials = false;
            mark.mesh_resource = mesh_resource;
            const Eigen::Vector3d &mesh_scale = lm->getVisualMeshScale();

            mark.scale.x = mesh_scale[0];
            mark.scale.y = mesh_scale[1];
            mark.scale.z = mesh_scale[2];
        }
        markers.markers.push_back(mark);
    }

    for (std::size_t i = num_orig_markers; i < markers.markers.size(); ++i) {
        visualization_msgs::Marker& m = markers.markers[i];
        m.color = color;
        m.ns = ns;
        m.lifetime = d;
        m.id = i;
    }

    return true;
}

void ManipulatorCommandPanel::update_base_pose_spinboxes()
{
    disconnect(teleport_base_command_x_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_x(double)));
    disconnect(teleport_base_command_y_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_y(double)));
    disconnect(teleport_base_command_yaw_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_yaw(double)));
    teleport_base_command_x_box_->setValue(world_to_robot_.translation()(0, 0));
    teleport_base_command_y_box_->setValue(world_to_robot_.translation()(1, 0));
    double yaw, pitch, roll;
    msg_utils::get_euler_ypr(world_to_robot_, yaw, pitch, roll);
    teleport_base_command_yaw_box_->setValue(sbpl::utils::ToDegrees(yaw));
    connect(teleport_base_command_x_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_x(double)));
    connect(teleport_base_command_y_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_y(double)));
    connect(teleport_base_command_yaw_box_, SIGNAL(valueChanged(double)), this, SLOT(update_base_pose_yaw(double)));
}

void ManipulatorCommandPanel::update_gui()
{
    ROS_INFO("    Pending Move Arm Command: %s", pending_move_arm_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Grasp Object Command: %s", pending_grasp_object_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Reposition Base Command: %s", pending_reposition_base_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Teleport Andalite Command: %s", pending_teleport_andalite_command_ ? "TRUE" : "FALSE");

    bool pending_motion_command =
        pending_move_arm_command_          ||
        pending_grasp_object_command_      ||
        pending_reposition_base_command_   ||
        pending_teleport_andalite_command_ ||
        pending_teleport_hdt_command_;

    pending_motion_command = false;

    global_frame_line_edit_->setEnabled(use_global_frame());

    copy_current_base_pose_button_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_x_box_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_y_box_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_z_box_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_yaw_box_->setEnabled(global_frame_active() && initialized());
    send_teleport_andalite_command_button_->setEnabled(global_frame_active() && initialized() && !pending_motion_command);

    send_grasp_object_command_button_->setEnabled(initialized() && !pending_motion_command);
    send_reposition_base_command_button_->setEnabled(global_frame_active() && initialized() && !pending_motion_command);

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

std::vector<double> ManipulatorCommandPanel::get_current_joint_angles() const
{
    std::vector<double> curr_joint_angles;
    curr_joint_angles.resize(robot_model_->joint_names().size());
    for (std::size_t i = 0; i < robot_model_->joint_names().size(); ++i) {
        const std::string& joint_name = robot_model_->joint_names()[i];
        (void)get_joint_value(last_joint_state_, joint_name, curr_joint_angles[i]);
    }
    return curr_joint_angles;
}

std::vector<double> ManipulatorCommandPanel::get_phantom_joint_angles() const
{
    std::vector<double> curr_joint_angles;
    curr_joint_angles.reserve(robot_model_->joint_names().size());

    for (const std::string& joint_name : robot_model_->joint_names()) {
        curr_joint_angles.push_back(rs_->getVariablePosition(joint_name));
    }

    return curr_joint_angles;
}

bool ManipulatorCommandPanel::set_phantom_joint_angles(const std::vector<double>& joint_angles)
{
    if (joint_angles.size() != robot_model_->joint_names().size()) {
        return false;
    }

    std::size_t i = 0;
    for (const std::string& joint_name : robot_model_->joint_names()) {
        rs_->setVariablePosition(joint_name, joint_angles[i]);
        ++i;
    }

    return true;
}

void ManipulatorCommandPanel::update_manipulator_marker_pose()
{
    visualization_msgs::InteractiveMarker manipulator_marker;
    if (server_.get("hdt_arm_control", manipulator_marker))
    {
        rs_->updateLinkTransforms();
        geometry_msgs::Pose new_marker_pose;
        tf::poseEigenToMsg(robot_transform() * rs_->getGlobalLinkTransform(tip_link_), new_marker_pose);
        std_msgs::Header header;
        header.seq = 0;
        header.frame_id = interactive_marker_frame();
        header.stamp = ros::Time(0);
        server_.setPose("hdt_arm_control", new_marker_pose, header);
        server_.applyChanges();
    }
}

void ManipulatorCommandPanel::update_object_marker_pose()
{
    visualization_msgs::InteractiveMarker object_marker;
    if (server_.get("gas_canister_fixture", object_marker)) {
        std_msgs::Header header;
        header.seq = 0;
        header.stamp = ros::Time(0);
        header.frame_id = interactive_marker_frame();
        geometry_msgs::Pose new_pose;
        tf::poseEigenToMsg(object_transform(), new_pose);
        server_.setPose("gas_canister_fixture", new_pose, header);
        server_.applyChanges();
    }
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::ManipulatorCommandPanel, rviz::Panel)
