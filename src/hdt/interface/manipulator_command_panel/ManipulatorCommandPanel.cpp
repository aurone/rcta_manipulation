#include "ManipulatorCommandPanel.h"

#include <cassert>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shape_operations.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/msg_utils/msg_utils.h>

namespace hdt
{

ManipulatorCommandPanel::ManipulatorCommandPanel(QWidget *parent) :
    rviz::Panel(parent),
    nh_(),
    move_arm_command_client_(),
    pending_move_arm_command_(false),
    viservo_command_client_(),
    pending_viservo_command_(false),
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
    send_teleport_hdt_command_button_(nullptr),
    copy_current_state_button_(nullptr),
    cycle_ik_solutions_button_(nullptr),
    send_move_arm_command_button_(nullptr),
    send_joint_goal_button_(nullptr),
    j1_spinbox_(nullptr),
    j2_spinbox_(nullptr),
    j3_spinbox_(nullptr),
    j4_spinbox_(nullptr),
    j5_spinbox_(nullptr),
    j6_spinbox_(nullptr),
    j7_spinbox_(nullptr),
    send_viservo_command_button_(nullptr),
    send_grasp_object_command_button_(nullptr),
    send_reposition_base_command_button_(nullptr),
    world_to_robot_(Eigen::Affine3d::Identity()),
    world_to_object_(Eigen::Affine3d::Identity()),
    robot_model_(),
    rm_loader_(),
    rm_(),
    rs_(),
    server_("hdt_control"),
    joint_states_sub_(),
    robot_markers_pub_(),
    last_joint_state_(),
    tip_link_(),
    base_link_(),
    mount_frame_to_manipulator_frame_(Eigen::Affine3d::Identity()),
    listener_(),
    robot_description_(),
    global_frame_(),
    candidate_base_poses_()
{
    setup_gui();
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &ManipulatorCommandPanel::joint_states_callback, this);
    robot_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
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
    config.mapGetString("global_frame", &global_frame);
    config.mapGetString("robot_description", &robot_description);
    config.mapGetBool("use_global_frame", &use_global_frame);
    config.mapGetFloat("base_x", &base_x);
    config.mapGetFloat("base_y", &base_y);
    config.mapGetFloat("base_yaw", &base_yaw);

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

    hdt::TeleportAndaliteCommandGoal teleport_andalite_goal;
    teleport_andalite_goal.global_pose.header.seq = 0;
    teleport_andalite_goal.global_pose.header.stamp = ros::Time::now();
    teleport_andalite_goal.global_pose.header.frame_id = interactive_marker_frame();
    tf::poseEigenToMsg(world_to_robot_, teleport_andalite_goal.global_pose.pose);

    auto result_cb = boost::bind(&ManipulatorCommandPanel::teleport_andalite_command_result_cb, this, _1, _2);
    teleport_andalite_command_client_->sendGoal(teleport_andalite_goal, result_cb);

    pending_teleport_andalite_command_ = true;
}

void ManipulatorCommandPanel::send_teleport_hdt_command()
{
    if (!reconnect_client(teleport_hdt_command_client_, "teleport_hdt_command")) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Teleport HDT Command (server is not connected)"));
        return;
    }

    hdt::TeleportHDTCommandGoal teleport_hdt_goal;
    teleport_hdt_goal.joint_state.header.seq = 0;
    teleport_hdt_goal.joint_state.header.stamp = ros::Time(0);
    teleport_hdt_goal.joint_state.header.frame_id = "";
    teleport_hdt_goal.joint_state.name = robot_model_->joint_names();
    teleport_hdt_goal.joint_state.position = get_phantom_joint_angles();

    auto result_cb = boost::bind(&ManipulatorCommandPanel::teleport_hdt_command_result_cb, this, _1, _2);
    teleport_hdt_command_client_->sendGoal(teleport_hdt_goal, result_cb);
    pending_teleport_hdt_command_ = true;
}

void ManipulatorCommandPanel::copy_current_state()
{
    std::vector<double> curr_joint_angles = get_current_joint_angles();
    if (!set_phantom_joint_angles(curr_joint_angles)) {
        QMessageBox::warning(this, tr("Copy Current State"), tr("Failed to copy current state to phantom state"));
        return;
    }

    update_manipulator_marker_pose();
    update_spinboxes();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::cycle_ik_solutions()
{
   Eigen::Affine3d new_eef_pose;
   new_eef_pose = rs_->getLinkState(tip_link_)->getGlobalLinkTransform();

   // manipulator -> eef = manipulator -> base * base -> root * root -> eef
   new_eef_pose = mount_frame_to_manipulator_frame_.inverse() * rs_->getFrameTransform(base_link_).inverse() * new_eef_pose;

   // get the current joint configuration to use as a seed for picking a nearby ik solution
   std::vector<double> curr_joint_angles(7);
   curr_joint_angles[0] = rs_->getJointState("arm_1_shoulder_twist")->getVariableValues()[0];
   curr_joint_angles[1] = rs_->getJointState("arm_2_shoulder_lift")->getVariableValues()[0];
   curr_joint_angles[2] = rs_->getJointState("arm_3_elbow_twist")->getVariableValues()[0];
   curr_joint_angles[3] = rs_->getJointState("arm_4_elbow_lift")->getVariableValues()[0];
   curr_joint_angles[4] = rs_->getJointState("arm_5_wrist_twist")->getVariableValues()[0];
   curr_joint_angles[5] = rs_->getJointState("arm_6_wrist_lift")->getVariableValues()[0];
   curr_joint_angles[6] = rs_->getJointState("arm_7_gripper_lift")->getVariableValues()[0];

   const double ik_search_res = sbpl::utils::ToRadians(1.0);

   std::vector<std::vector<double>> ik_solutions;
   SimpleIKSolutionGenerator solgen = robot_model_->compute_all_ik_solutions(new_eef_pose, curr_joint_angles);

   std::vector<double> iksol;
   while (solgen(iksol)) {
        ik_solutions.push_back(std::move(iksol));
   }

   std::sort(ik_solutions.begin(), ik_solutions.end(), [&curr_joint_angles](const std::vector<double>& j1, const std::vector<double>& j2)
   {
       return ComputeJointStateL2NormSqrd(j1, curr_joint_angles) < ComputeJointStateL2NormSqrd(j2, curr_joint_angles);
   });

   if (ik_solutions.size() > 1) {
       ROS_INFO("BAM, %zd solutions", ik_solutions.size());
       std::vector<std::vector<double>>::const_iterator next = ++ik_solutions.cbegin();
       if (!set_phantom_joint_angles(*next)) {
           QMessageBox::warning(this, tr("Cycle IK Solutions"), tr("Failed to set phantom state from ik solution"));
           return;
       }
       rs_->updateLinkTransforms();
       publish_phantom_robot_visualizations();
   }
   else {
       ROS_WARN("Not enough IK solutions to cycle");
   }
}

void ManipulatorCommandPanel::send_move_arm_command()
{
    if (!reconnect_client(move_arm_command_client_, "move_arm_command")) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Move Arm Command (server is not connected)"));
        return;
    }

    hdt::MoveArmCommandGoal move_arm_goal;

    move_arm_goal.type = hdt::MoveArmCommandGoal::EndEffectorGoal;

    // mounting frame -> eef = mounting_frame -> root * root -> eef
    const Eigen::Affine3d& root_to_mount_frame = rs_->getFrameTransform(base_link_);
    tf::poseEigenToMsg(
            root_to_mount_frame.inverse() * rs_->getLinkState("arm_7_gripper_lift_link")->getGlobalLinkTransform(),
            move_arm_goal.goal_pose);

    // manipulator -> eef = manipulator -> mount * mount -> root * root -> end effector
    Eigen::Affine3d manipulator_frame_to_eef_frame =
            mount_frame_to_manipulator_frame_.inverse() *
            root_to_mount_frame.inverse() *
            rs_->getLinkState("arm_7_gripper_lift_link")->getGlobalLinkTransform();

    geometry_msgs::Pose eef_in_manipulator_frame;
    tf::poseEigenToMsg(manipulator_frame_to_eef_frame, eef_in_manipulator_frame);
    ROS_INFO("eef in manipulator frame: %s", to_string(manipulator_frame_to_eef_frame).c_str());

    auto result_callback = boost::bind(&ManipulatorCommandPanel::move_arm_command_result_cb, this, _1, _2);
    move_arm_command_client_->sendGoal(move_arm_goal, result_callback);

    pending_move_arm_command_ = true;
    update_gui();
}

void ManipulatorCommandPanel::send_joint_goal()
{
    if (!reconnect_client(move_arm_command_client_, "move_arm_command")) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Move Arm Command (server is not connected)"));
        return;
    }

    hdt::MoveArmCommandGoal move_arm_goal;
    move_arm_goal.type = hdt::MoveArmCommandGoal::JointGoal;

    move_arm_goal.goal_joint_state.header.stamp = ros::Time::now();
    move_arm_goal.goal_joint_state.name = robot_model_->joint_names();
    move_arm_goal.goal_joint_state.position.reserve(7);
    for (const std::string& joint_name : robot_model_->joint_names()) {
        move_arm_goal.goal_joint_state.position.push_back(rs_->getJointState(joint_name)->getVariableValues()[0]);
    }

    auto result_callback = boost::bind(&ManipulatorCommandPanel::move_arm_command_result_cb, this, _1, _2);
    move_arm_command_client_->sendGoal(move_arm_goal, result_callback);

    pending_move_arm_command_ = true;
    update_gui();
}

void ManipulatorCommandPanel::update_j1_position(double value)
{
    update_joint_position(0, value);
}

void ManipulatorCommandPanel::update_j2_position(double value)
{
    update_joint_position(1, value);
}

void ManipulatorCommandPanel::update_j3_position(double value)
{
    update_joint_position(2, value);
}

void ManipulatorCommandPanel::update_j4_position(double value)
{
    update_joint_position(3, value);
}

void ManipulatorCommandPanel::update_j5_position(double value)
{
    update_joint_position(4, value);
}

void ManipulatorCommandPanel::update_j6_position(double value)
{
    update_joint_position(5, value);
}

void ManipulatorCommandPanel::update_j7_position(double value)
{
    update_joint_position(6, value);
}

void ManipulatorCommandPanel::send_viservo_command()
{
    if (!reconnect_client(viservo_command_client_, "viservo_command")) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Viservo Command (server is not connected)"));
        return;
    }

    const std::string camera_link_name = "camera_rgb_frame";
    const std::string wrist_link_name = "arm_7_gripper_lift_link";

    if (/*!rs_->hasLinkState(camera_link_name) ||*/ !rs_->hasLinkState(wrist_link_name)) {
        ROS_ERROR("Robot State does not contain transforms for camera or wrist links");
        return;
    }

    geometry_msgs::PoseStamped identity_pose_camera_frame;
    identity_pose_camera_frame.header.frame_id = camera_link_name;
    identity_pose_camera_frame.header.stamp = ros::Time(0);
    identity_pose_camera_frame.header.seq = 0;
    identity_pose_camera_frame.pose.position.x = identity_pose_camera_frame.pose.position.y = identity_pose_camera_frame.pose.position.z = 0.0;
    identity_pose_camera_frame.pose.orientation.x = identity_pose_camera_frame.pose.orientation.y = identity_pose_camera_frame.pose.orientation.z = 0.0;
    identity_pose_camera_frame.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped root_to_camera_pose;
    try {
        listener_.transformPose(rs_->getRobotModel()->getRootLinkName(), identity_pose_camera_frame, root_to_camera_pose);
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to find fixed transform between '%s' and %s'", camera_link_name.c_str(), wrist_link_name.c_str());
        return;
    }

    Eigen::Affine3d root_to_camera;
    tf::poseMsgToEigen(root_to_camera_pose.pose, root_to_camera);

    ROS_INFO("Root to camera: %s", to_string(root_to_camera).c_str());

    // construct the goal wrist pose in the camera frame; the goal pose should be the same pose as whatever pose we think we're currently at.
//    const Eigen::Affine3d& root_to_camera = rs_->getLinkState(camera_link_name)->getGlobalLinkTransform();
    const Eigen::Affine3d& root_to_wrist = rs_->getLinkState(wrist_link_name)->getGlobalLinkTransform();
    Eigen::Affine3d camera_to_wrist = root_to_camera.inverse() * root_to_wrist;

    hdt::ViservoCommandGoal viservo_goal;
    tf::poseEigenToMsg(camera_to_wrist, viservo_goal.goal_pose);
    auto result_callback = boost::bind(&ManipulatorCommandPanel::viservo_command_result_cb, this, _1, _2);
    viservo_command_client_->sendGoal(viservo_goal, result_callback);

    pending_viservo_command_ = true;
    update_gui();
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

    hdt_msgs::GraspObjectCommandGoal grasp_object_goal;

    static int grasp_object_goal_id = 0;
    grasp_object_goal.id = grasp_object_goal_id++;
    grasp_object_goal.retry_count = 0;

    // robot -> object = robot -> marker * marker -> object
    Eigen::Affine3d robot_to_object = robot_transform().inverse() * object_transform();
    grasp_object_goal.gas_can_in_base_link.header.frame_id = rm_->getModelFrame();
    tf::poseEigenToMsg(robot_to_object, grasp_object_goal.gas_can_in_base_link.pose);

    grasp_object_goal.gas_can_in_map.header.frame_id = interactive_marker_frame();
    tf::poseEigenToMsg(object_transform(), grasp_object_goal.gas_can_in_map.pose);

    // TODO: octomap

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

    hdt_msgs::RepositionBaseCommandGoal reposition_base_goal;

    static int reposition_base_goal_id = 0;
    reposition_base_goal.id = reposition_base_goal_id++;
    reposition_base_goal.retry_count = 0;

    tf::poseEigenToMsg(world_to_object_, reposition_base_goal.gas_can_in_map.pose);
    reposition_base_goal.gas_can_in_map.header.frame_id = interactive_marker_frame();

    tf::poseEigenToMsg(world_to_robot_, reposition_base_goal.base_link_in_map.pose);
    reposition_base_goal.base_link_in_map.header.frame_id = interactive_marker_frame();

    // TODO: occupancy grid

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

    // arm commands
    QGroupBox* arm_commands_group = new QGroupBox(tr("Arm Commands"));
        QVBoxLayout* arm_commands_layout = new QVBoxLayout;
            copy_current_state_button_ = new QPushButton(tr("Copy Arm State"));
            cycle_ik_solutions_button_ = new QPushButton(tr("Cycle IK Solution"));
            QGridLayout* joint_state_spinbox_layout = new QGridLayout;
                QLabel* j1_label = new QLabel(tr("J1"));
                j1_spinbox_ = new QDoubleSpinBox;
                QLabel* j2_label = new QLabel(tr("J2"));
                j2_spinbox_ = new QDoubleSpinBox;
                QLabel* j3_label = new QLabel(tr("J3"));
                j3_spinbox_ = new QDoubleSpinBox;
                QLabel* j4_label = new QLabel(tr("J4"));
                j4_spinbox_ = new QDoubleSpinBox;
                QLabel* j5_label = new QLabel(tr("J5"));
                j5_spinbox_ = new QDoubleSpinBox;
                QLabel* j6_label = new QLabel(tr("J6"));
                j6_spinbox_ = new QDoubleSpinBox;
                QLabel* j7_label = new QLabel(tr("J7"));
                j7_spinbox_ = new QDoubleSpinBox;
            joint_state_spinbox_layout->addWidget(j1_label, 0, 0);
            joint_state_spinbox_layout->addWidget(j1_spinbox_, 0, 1);
            joint_state_spinbox_layout->addWidget(j2_label, 0, 2);
            joint_state_spinbox_layout->addWidget(j2_spinbox_, 0, 3);
            joint_state_spinbox_layout->addWidget(j3_label, 0, 4);
            joint_state_spinbox_layout->addWidget(j3_spinbox_, 0, 5);
            joint_state_spinbox_layout->addWidget(j4_label, 0, 6);
            joint_state_spinbox_layout->addWidget(j4_spinbox_, 0, 7);
            joint_state_spinbox_layout->addWidget(j5_label, 1, 0);
            joint_state_spinbox_layout->addWidget(j5_spinbox_, 1, 1);
            joint_state_spinbox_layout->addWidget(j6_label, 1, 2);
            joint_state_spinbox_layout->addWidget(j6_spinbox_, 1, 3);
            joint_state_spinbox_layout->addWidget(j7_label, 1, 4);
            joint_state_spinbox_layout->addWidget(j7_spinbox_, 1, 5);
            send_move_arm_command_button_ = new QPushButton(tr("Move Arm to End Effector Pose"));
            send_joint_goal_button_ = new QPushButton(tr("Move Arm to Joint State"));
            send_teleport_hdt_command_button_ = new QPushButton(tr("Teleport HDT"));
            send_viservo_command_button_ = new QPushButton(tr("Visual Servo"));
        arm_commands_layout->addWidget(copy_current_state_button_);
        arm_commands_layout->addWidget(cycle_ik_solutions_button_);
        arm_commands_layout->addLayout(joint_state_spinbox_layout);
        arm_commands_layout->addWidget(send_move_arm_command_button_);
        arm_commands_layout->addWidget(send_joint_goal_button_);
        arm_commands_layout->addWidget(send_teleport_hdt_command_button_);
        arm_commands_layout->addWidget(send_viservo_command_button_);
    arm_commands_group->setLayout(arm_commands_layout);

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
    main_layout->addWidget(arm_commands_group);
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

    // arm commands
    connect(copy_current_state_button_, SIGNAL(clicked()), this, SLOT(copy_current_state()));
    connect(cycle_ik_solutions_button_, SIGNAL(clicked()), this, SLOT(cycle_ik_solutions()));
    connect(send_move_arm_command_button_, SIGNAL(clicked()), this, SLOT(send_move_arm_command()));
    connect(send_joint_goal_button_, SIGNAL(clicked()), this, SLOT(send_joint_goal()));
    connect(send_teleport_hdt_command_button_, SIGNAL(clicked()), this, SLOT(send_teleport_hdt_command()));
    connect(j1_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j1_position(double)));
    connect(j2_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j2_position(double)));
    connect(j3_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j3_position(double)));
    connect(j4_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j4_position(double)));
    connect(j5_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j5_position(double)));
    connect(j6_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j6_position(double)));
    connect(j7_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j7_position(double)));
    connect(send_viservo_command_button_, SIGNAL(clicked()), this, SLOT(send_viservo_command()));

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

        std::vector<robot_model::JointModel*> child_joints;
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

    ROS_DEBUG("Manipulator -> Base: %s", to_string(mount_frame_to_manipulator_frame_.inverse()).c_str());
    ROS_DEBUG("Base -> Robot: %s", to_string(rs_->getFrameTransform(base_link_).inverse()).c_str());
    ROS_DEBUG("Robot -> World: %s", to_string(world_to_robot_.inverse()).c_str());
    ROS_DEBUG("World -> End Effector: %s", to_string(new_eef_pose).c_str());

    // manipulator -> eef =
    //     manipulator -> mount * mount -> root * root -> global * global -> eef
    new_eef_pose =
            mount_frame_to_manipulator_frame_.inverse() *
            rs_->getFrameTransform(base_link_).inverse() *
            robot_transform().inverse() *
            new_eef_pose;

    ROS_DEBUG("Manipulator -> EndEffector: %s", to_string(new_eef_pose).c_str());

    // get the current joint configuration to use as a seed for picking a nearby ik solution
    std::vector<double> curr_joint_angles = get_phantom_joint_angles();
    const double ik_search_res = sbpl::utils::ToRadians(1.0);
    std::vector<double> iksol;

    // run ik to the pose of the marker
    if (robot_model_->search_nearest_ik(new_eef_pose, curr_joint_angles, iksol, ik_search_res)) {
        if (!set_phantom_joint_angles(iksol)) {
            QMessageBox::warning(this, tr("Interactive Marker Feedback"), tr("Failed to set phantom state from interactive marker-driven IK"));
            return;
        }
    }

    // update the position of the marker (in case of ik failure and for synchronization)
    update_manipulator_marker_pose();
    update_spinboxes();
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

std::string ManipulatorCommandPanel::get_base_link(const robot_model::JointModelGroup& joint_model_group) const
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

void ManipulatorCommandPanel::update_joint_position(int joint_index, double joint_position)
{
    double joint_val = sbpl::utils::ToRadians(joint_position);
    rs_->getJointState(robot_model_->joint_names()[joint_index])->setVariableValues(&joint_val);
    update_manipulator_marker_pose();
    publish_phantom_robot_visualizations();
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

bool ManipulatorCommandPanel::reinit(const std::string& robot_description, std::string& why)
{
    if (!reinit_robot_models(robot_description, why)) {
        return false;
    }

    if (!reinit_interactive_marker_server()) {
        return false;
    }

    // TODO: The following must be done after the interactive marker server has
    // been reinitialized, which is what identifies the base and tip links. It
    // probably makes more sense for this step to be done when the robot is
    // reinitialized

    // compute the transform to the frame that kinematics is done in, which is required for sending arm goals
    const Eigen::Affine3d& root_to_manipulator_frame = rs_->getFrameTransform("arm_1_shoulder_twist_link");
    const Eigen::Affine3d& root_to_base_frame = rs_->getFrameTransform(base_link_);
    mount_frame_to_manipulator_frame_ = root_to_base_frame.inverse() * root_to_manipulator_frame;
    ROS_INFO("Mount Frame -> Manipulator Frame: %s", to_string(mount_frame_to_manipulator_frame_).c_str());

    publish_phantom_robot_visualizations();
    return true;
}

bool ManipulatorCommandPanel::reinit_robot_models(const std::string& robot_description, std::string& why)
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
    rs_->setRootTransform(Eigen::Affine3d::Identity());
    rs_->updateLinkTransforms();

    // update gui parameters to reflect robot
    j1_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[0]));
    j2_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[1]));
    j3_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[2]));
    j4_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[3]));
    j5_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[4]));
    j6_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[5]));
    j7_spinbox_->setMinimum(sbpl::utils::ToDegrees(robot_model_->min_limits()[6]));

    j1_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[0]));
    j2_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[1]));
    j3_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[2]));
    j4_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[3]));
    j5_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[4]));
    j6_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[5]));
    j7_spinbox_->setMaximum(sbpl::utils::ToDegrees(robot_model_->max_limits()[6]));

    j1_spinbox_->setSingleStep(1.0);
    j2_spinbox_->setSingleStep(1.0);
    j3_spinbox_->setSingleStep(1.0);
    j4_spinbox_->setSingleStep(1.0);
    j5_spinbox_->setSingleStep(1.0);
    j6_spinbox_->setSingleStep(1.0);
    j7_spinbox_->setSingleStep(1.0);

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
            robot_state::LinkState* tip_link_state = rs_->getLinkState(tip_link_);
            if (!tip_link_state) {
                ROS_ERROR("Failed to get Link State for tip link '%s'", tip_link_.c_str());
                return false;
            }

            Eigen::Affine3d world_to_marker = robot_transform() * tip_link_state->getGlobalLinkTransform();
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

void ManipulatorCommandPanel::move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::MoveArmCommandResult::ConstPtr& result)
{
    pending_move_arm_command_ = false;
    update_gui();
}

void ManipulatorCommandPanel::viservo_command_active_cb()
{

}

void ManipulatorCommandPanel::viservo_command_feedback_cb(const hdt::ViservoCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::viservo_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::ViservoCommandResult::ConstPtr& result)
{
    ROS_INFO("Received Result from Viservo Command Action");
    pending_viservo_command_ = false;
    update_gui();
}

void ManipulatorCommandPanel::grasp_object_command_active_cb()
{

}

void ManipulatorCommandPanel::grasp_object_command_feeback_cb(const hdt_msgs::GraspObjectCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::grasp_object_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt_msgs::GraspObjectCommandResult::ConstPtr& result)
{
    ROS_INFO("Received Result from Grasp Object Command Action");
    pending_grasp_object_command_ = false;
    update_gui();
}


void ManipulatorCommandPanel::reposition_base_command_active_cb()
{

}

void ManipulatorCommandPanel::reposition_base_command_feedback_cb(const hdt_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::reposition_base_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const hdt_msgs::RepositionBaseCommandResult::ConstPtr& result)
{

    ROS_INFO("Received Result from Reposition Base Command Action");
    pending_reposition_base_command_ = false;

    if (result->result == hdt_msgs::RepositionBaseCommandResult::SUCCESS) {
        candidate_base_poses_ = result->candidate_base_poses;
        ROS_INFO("Reposition Base Command returned %zd candidate poses", candidate_base_poses_.size());
    }

    update_gui();
}

void ManipulatorCommandPanel::teleport_andalite_command_active_cb()
{

}

void ManipulatorCommandPanel::teleport_andalite_command_feedback_cb(const hdt::TeleportAndaliteCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::teleport_andalite_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::TeleportAndaliteCommandResult::ConstPtr& result)
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

void ManipulatorCommandPanel::teleport_hdt_command_feedback_cb(const hdt::TeleportHDTCommandFeedback::ConstPtr& feedback)
{

}

void ManipulatorCommandPanel::teleport_hdt_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::TeleportHDTCommandResult::ConstPtr& result)
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
        const robot_state::LinkState* ls = robot_state.getLinkState(link_names[i]);
        if (!ls) {
            continue;
        }

        // add markers for attached objects
        if (include_attached) {
            std::vector<const robot_state::AttachedBody*> attached_bodies;
            ls->getAttachedBodies(attached_bodies);
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

        if (!ls->getLinkModel() || !ls->getLinkModel()->getShape()) {
            continue;
        }

        mark.header.frame_id = robot_state.getRobotModel()->getModelFrame();
        mark.header.stamp = tm;
        tf::poseEigenToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);

        // we prefer using the visual mesh, if a mesh is available
        const std::string& mesh_resource = ls->getLinkModel()->getVisualMeshFilename();
        if (mesh_resource.empty()) {
            if (!shapes::constructMarkerFromShape(ls->getLinkModel()->getShape().get(), mark)) {
                continue;
            }

            // if the object is invisible (0 volume) we skip it
            if (fabs(mark.scale.x * mark.scale.y * mark.scale.z) < std::numeric_limits<float>::epsilon()) {
                continue;
            }
        }
        else {
            tf::poseEigenToMsg(ls->getGlobalLinkTransform(), mark.pose);

            mark.type = mark.MESH_RESOURCE;
            mark.mesh_use_embedded_materials = false;
            mark.mesh_resource = mesh_resource;
            const Eigen::Vector3d &mesh_scale = ls->getLinkModel()->getVisualMeshScale();

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

void ManipulatorCommandPanel::update_spinboxes()
{
    // note: temporarily disable this guys so that spinboxes can be updated in
    // response to interactive marker changes without publishing a fuckton of
    // display markers from triggering one slot for each joint
    disconnect(j1_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j1_position(double)));
    disconnect(j2_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j2_position(double)));
    disconnect(j3_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j3_position(double)));
    disconnect(j4_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j4_position(double)));
    disconnect(j5_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j5_position(double)));
    disconnect(j6_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j6_position(double)));
    disconnect(j7_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j7_position(double)));
    j1_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[0])->getVariableValues()[0]));
    j2_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[1])->getVariableValues()[0]));
    j3_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[2])->getVariableValues()[0]));
    j4_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[3])->getVariableValues()[0]));
    j5_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[4])->getVariableValues()[0]));
    j6_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[5])->getVariableValues()[0]));
    j7_spinbox_->setValue(sbpl::utils::ToDegrees(rs_->getJointState(robot_model_->joint_names()[6])->getVariableValues()[0]));
    connect(j1_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j1_position(double)));
    connect(j2_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j2_position(double)));
    connect(j3_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j3_position(double)));
    connect(j4_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j4_position(double)));
    connect(j5_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j5_position(double)));
    connect(j6_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j6_position(double)));
    connect(j7_spinbox_, SIGNAL(valueChanged(double)), this, SLOT(update_j7_position(double)));
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
    ROS_INFO("    Pending Viservo Command: %s", pending_viservo_command_ ? "TRUE": "FALSE");
    ROS_INFO("    Pending Grasp Object Command: %s", pending_grasp_object_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Reposition Base Command: %s", pending_reposition_base_command_ ? "TRUE" : "FALSE");
    ROS_INFO("    Pending Teleport Andalite Command: %s", pending_teleport_andalite_command_ ? "TRUE" : "FALSE");

    bool pending_motion_command =
        pending_move_arm_command_          ||
        pending_viservo_command_           ||
        pending_grasp_object_command_      ||
        pending_reposition_base_command_   ||
        pending_teleport_andalite_command_ ||
        pending_teleport_hdt_command_;

    global_frame_line_edit_->setEnabled(use_global_frame());

    copy_current_base_pose_button_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_x_box_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_y_box_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_z_box_->setEnabled(global_frame_active() && initialized());
    teleport_base_command_yaw_box_->setEnabled(global_frame_active() && initialized());
    send_teleport_andalite_command_button_->setEnabled(global_frame_active() && initialized() && !pending_motion_command);

    copy_current_state_button_->setEnabled(initialized());
    cycle_ik_solutions_button_->setEnabled(initialized());
    send_move_arm_command_button_->setEnabled(initialized() && !pending_motion_command);
    send_joint_goal_button_->setEnabled(initialized() && !pending_motion_command);
    send_teleport_hdt_command_button_->setEnabled(initialized() && !pending_motion_command);
    j1_spinbox_->setEnabled(initialized());
    j2_spinbox_->setEnabled(initialized());
    j3_spinbox_->setEnabled(initialized());
    j4_spinbox_->setEnabled(initialized());
    j5_spinbox_->setEnabled(initialized());
    j6_spinbox_->setEnabled(initialized());
    j7_spinbox_->setEnabled(initialized());
    send_viservo_command_button_->setEnabled(initialized() && !pending_motion_command);

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
        curr_joint_angles.push_back(rs_->getJointState(joint_name)->getVariableValues()[0]);
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
        rs_->getJointState(joint_name)->setVariableValues(&joint_angles[i]);
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
        tf::poseEigenToMsg(robot_transform() * rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
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
