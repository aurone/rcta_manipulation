#include "ManipulatorCommandPanel.h"

#include <cassert>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <Eigen/Dense>
#include <QPushButton>
#include <QVBoxLayout>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <hdt/MoveArmCommand.h>
#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometric_shapes/shape_operations.h>

using namespace std;

namespace hdt
{

std::string to_string(const geometry_msgs::Pose& pose)
{
    std::stringstream ss;
    ss << "{ position: {";
    ss << "x: " << pose.position.x << ", ";
    ss << "y: " << pose.position.y << ", ";
    ss << "z: " << pose.position.z << " } ";
    ss << "orientation: {";
    ss << "w: " << pose.orientation.w << ", ";
    ss << "x: " << pose.orientation.x << ", ";
    ss << "y: " << pose.orientation.y << ", ";
    ss << "z: " << pose.orientation.z << "} }";
    return ss.str();
}

ManipulatorCommandPanel::ManipulatorCommandPanel(QWidget *parent) :
    rviz::Panel(parent),
    initialized_(false),
    shutdown_(false),
    nh_(),
    ph_("~"),
    move_arm_client_(),
    send_move_command_(false),
    move_arm_thread_(),
    move_arm_mutex_(),
    move_arm_condvar_(),
    rm_loader_(),
    rm_(),
    rs_(),
    server_("hdt_control"),
    interactive_markers_(),
    tip_link_(),
    send_interp_command_button_(nullptr),
    copy_current_state_button_(nullptr),
    refresh_robot_desc_button_(nullptr),
    send_move_arm_command_button_(nullptr),
    cycle_ik_solutions_button_(nullptr),
    joint_1_slider_(nullptr),
    joint_2_slider_(nullptr),
    joint_3_slider_(nullptr),
    joint_4_slider_(nullptr),
    joint_5_slider_(nullptr),
    joint_6_slider_(nullptr),
    joint_7_slider_(nullptr),
    critical_state_mutex_()
{
    ROS_INFO("Instantiating Manipulator Command Panel");

    send_interp_command_button_ = new QPushButton(tr("Send Interpolate Command"));
    copy_current_state_button_ = new QPushButton(tr("Copy Current State"));
    refresh_robot_desc_button_ = new QPushButton(tr("Refresh Robot Description"));
    send_move_arm_command_button_ = new QPushButton(tr("Send Move Arm Command"));
    cycle_ik_solutions_button_ = new QPushButton(tr("Cycle IK Solution"));

    joint_1_slider_ = new QSlider(Qt::Horizontal);
    joint_2_slider_ = new QSlider(Qt::Horizontal);
    joint_3_slider_ = new QSlider(Qt::Horizontal);
    joint_4_slider_ = new QSlider(Qt::Horizontal);
    joint_5_slider_ = new QSlider(Qt::Horizontal);
    joint_6_slider_ = new QSlider(Qt::Horizontal);
    joint_7_slider_ = new QSlider(Qt::Horizontal);

    QVBoxLayout* layout = new QVBoxLayout;
    // layout->addWidget(send_interp_command_button_);
    layout->addWidget(copy_current_state_button_);
    layout->addWidget(refresh_robot_desc_button_);
    layout->addWidget(send_move_arm_command_button_);
    layout->addWidget(cycle_ik_solutions_button_);
    layout->addWidget(joint_1_slider_);
    layout->addWidget(joint_2_slider_);
    layout->addWidget(joint_3_slider_);
    layout->addWidget(joint_4_slider_);
    layout->addWidget(joint_5_slider_);
    layout->addWidget(joint_6_slider_);
    layout->addWidget(joint_7_slider_);
    setLayout(layout);

    move_arm_client_ = nh_.serviceClient<hdt::MoveArmCommand>("move_arm_command");
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &ManipulatorCommandPanel::joint_states_callback, this);
    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
    robot_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("phantom_robot", 1);

    connect(send_interp_command_button_, SIGNAL(clicked()), this, SLOT(send_interp_command()));
    connect(copy_current_state_button_, SIGNAL(clicked()), this, SLOT(copy_current_state()));
    connect(refresh_robot_desc_button_, SIGNAL(clicked()), this, SLOT(refresh_robot_description()));
    connect(send_move_arm_command_button_, SIGNAL(clicked()), this, SLOT(send_move_arm_command()));
    connect(cycle_ik_solutions_button_, SIGNAL(clicked()), this, SLOT(cycle_ik_solutions()));
    connect(joint_1_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_1(int)));
    connect(joint_2_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_2(int)));
    connect(joint_3_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_3(int)));
    connect(joint_4_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_4(int)));
    connect(joint_5_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_5(int)));
    connect(joint_6_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_6(int)));
    connect(joint_7_slider_, SIGNAL(valueChanged(int)), this, SLOT(change_joint_7(int)));

    ROS_INFO("Initializing Manipulator Command Panel");
    if (!do_init())
    {
        ROS_ERROR("Failed to initialize ManipulatorCommandPanel");
    }
    else
    {
        ROS_INFO("Successfully initialized ManipulatorCommandPanel");
    }
}

ManipulatorCommandPanel::~ManipulatorCommandPanel()
{
    {
        shutdown_ = true;
        std::unique_lock<std::mutex> lock(move_arm_mutex_);
        move_arm_condvar_.notify_one();
    }

    // TODO: figure out why deleting rm_loader_ here causes a core dump
    if (move_arm_thread_.joinable()) {
        move_arm_thread_.join();
    }
}

void ManipulatorCommandPanel::send_interp_command()
{
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

    if (!initialized_) {
        return;
    }

    auto to_string = [](const std::array<double, 7>& joint_state)
    {
        std::stringstream ss;
        ss << std::setprecision(3) << std::setw(8) << '(' << joint_state[0] << ' ' << joint_state[1] << ' ' << joint_state[2] << ' ' << joint_state[3] << ' ' << joint_state[4] << ' ' << joint_state[5] << ' ' << joint_state[6] << ')';
        return ss.str();
    };

    std::array<double, 7> curr_joint_angles;
    get_joint_value(last_joint_state_, "arm_1_shoulder_twist", curr_joint_angles[0]);
    get_joint_value(last_joint_state_, "arm_2_shoulder_lift", curr_joint_angles[1]);
    get_joint_value(last_joint_state_, "arm_3_elbow_twist", curr_joint_angles[2]);
    get_joint_value(last_joint_state_, "arm_4_elbow_lift", curr_joint_angles[3]);
    get_joint_value(last_joint_state_, "arm_5_wrist_twist", curr_joint_angles[4]);
    get_joint_value(last_joint_state_, "arm_6_wrist_lift", curr_joint_angles[5]);
    get_joint_value(last_joint_state_, "arm_7_gripper_lift", curr_joint_angles[6]);

    std::array<double, 7> new_joint_angles;
    new_joint_angles[0] = rs_->getJointState("arm_1_shoulder_twist")->getVariableValues()[0];
    new_joint_angles[1] = rs_->getJointState("arm_2_shoulder_lift")->getVariableValues()[0];
    new_joint_angles[2] = rs_->getJointState("arm_3_elbow_twist")->getVariableValues()[0];
    new_joint_angles[3] = rs_->getJointState("arm_4_elbow_lift")->getVariableValues()[0];
    new_joint_angles[4] = rs_->getJointState("arm_5_wrist_twist")->getVariableValues()[0];
    new_joint_angles[5] = rs_->getJointState("arm_6_wrist_lift")->getVariableValues()[0];
    new_joint_angles[6] = rs_->getJointState("arm_7_gripper_lift")->getVariableValues()[0];

    ROS_INFO("Interpolating from %s to %s", to_string(curr_joint_angles).c_str(), to_string(new_joint_angles).c_str());

    std::vector<double> start(curr_joint_angles.begin(), curr_joint_angles.end());
    std::vector<double> end(new_joint_angles.begin(), new_joint_angles.end());
    std::vector<double> inc(7, sbpl::utils::ToRadians(1.0));
    std::vector<bool> continuous(7, false);
    std::vector<std::vector<double>> path;

    if (sbpl::interp::InterpolatePath(start, end, min_limits_, max_limits_, inc, continuous, path)) {
        ROS_INFO("Interpolation succeeded with %lu points", path.size());
        static ros::Publisher trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
        trajectory_msgs::JointTrajectory traj;
        traj.header.frame_id = "";
        traj.header.seq = 0;
        traj.header.stamp = ros::Time::now();

        const std::vector<std::string>& joint_names = rm_->getJointModelNames();

        const std::vector<std::string> manip_joint_names = { "arm_1_shoulder_twist",
                                                             "arm_2_shoulder_lift",
                                                             "arm_3_elbow_twist",
                                                             "arm_4_elbow_lift",
                                                             "arm_5_wrist_twist",
                                                             "arm_6_wrist_lift",
                                                             "arm_7_gripper_lift" };

        auto manip_joint_index = [&manip_joint_names](const std::string& joint)
        {
            for (int i = 0; i < (int)manip_joint_names.size(); ++i) {
                if (manip_joint_names[i] == joint) {
                    return i;
                }
            }
            return -1;
        };

        traj.joint_names = joint_names;
        for (auto& point : path) {
            trajectory_msgs::JointTrajectoryPoint p;
            p.positions.resize(joint_names.size());
            for (int jidx = 0; jidx < (int)joint_names.size(); ++jidx) {
                int manip_jidx = manip_joint_index(joint_names[jidx]);
                if (manip_jidx != -1) {
                    p.positions[jidx] = point[manip_jidx];
                }
                else {
                    p.positions[jidx] = 0.0;
                }
            }

            const double joint_vel = 4.0 * M_PI / 180.0;
            // create joint velocities and timings
            if (traj.points.empty()) {
                p.velocities = std::vector<double>(joint_names.size(), 0.0);
                p.time_from_start = ros::Duration(0.0);
            }
            else {
                const std::vector<double>& last_point = traj.points.back().positions;
                p.time_from_start = traj.points.back().time_from_start + ros::Duration(0.05);
//                for (int i = 0; i < 7; ++i) {
//                    if (sbpl::utils::ShortestAngleDiff(p.positions[i], last_point[i]) < 0) {
//                        p.velocities.push_back(-joint_vel);
//                    }
//                    else if (sbpl::utils::ShortestAngleDiff(p.positions[i], last_point[i]) > 0) {
//                        p.velocities.push_back(joint_vel);
//                    }
//                    else {
//                        p.velocities.push_back(0.0);
//                    }
//                }
            }
            traj.points.push_back(p);
        }

        trajectory_pub_.publish(traj);
    }
    else
    {
        ROS_WARN("Interpolation unsuccessful");
    }
}

void ManipulatorCommandPanel::copy_current_state()
{
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

    std::array<double, 7> curr_joint_angles;
    get_joint_value(last_joint_state_, "arm_1_shoulder_twist", curr_joint_angles[0]);
    get_joint_value(last_joint_state_, "arm_2_shoulder_lift", curr_joint_angles[1]);
    get_joint_value(last_joint_state_, "arm_3_elbow_twist", curr_joint_angles[2]);
    get_joint_value(last_joint_state_, "arm_4_elbow_lift", curr_joint_angles[3]);
    get_joint_value(last_joint_state_, "arm_5_wrist_twist", curr_joint_angles[4]);
    get_joint_value(last_joint_state_, "arm_6_wrist_lift", curr_joint_angles[5]);
    get_joint_value(last_joint_state_, "arm_7_gripper_lift", curr_joint_angles[6]);

    // run ik to the pose of the marker
    rs_->getJointState("arm_1_shoulder_twist")->setVariableValues(&curr_joint_angles[0]);
    rs_->getJointState("arm_2_shoulder_lift")->setVariableValues(&curr_joint_angles[1]);
    rs_->getJointState("arm_3_elbow_twist")->setVariableValues(&curr_joint_angles[2]);
    rs_->getJointState("arm_4_elbow_lift")->setVariableValues(&curr_joint_angles[3]);
    rs_->getJointState("arm_5_wrist_twist")->setVariableValues(&curr_joint_angles[4]);
    rs_->getJointState("arm_6_wrist_lift")->setVariableValues(&curr_joint_angles[5]);
    rs_->getJointState("arm_7_gripper_lift")->setVariableValues(&curr_joint_angles[6]);

    rs_->updateLinkTransforms();

    // update the position of the marker (in case of ik failure and for synchronization)
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();

    publish_phantom_robot_visualizations();
    update_sliders();
}

void ManipulatorCommandPanel::refresh_robot_description()
{
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

    if (!reinit_robot()){
        ROS_ERROR("Failed to refresh robot description");
    }
}

void ManipulatorCommandPanel::send_move_arm_command()
{
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

    std::unique_lock<std::mutex> move_arm_lock(move_arm_mutex_);
    send_move_command_ = true;
    move_arm_condvar_.notify_one();
}

void ManipulatorCommandPanel::cycle_ik_solutions()
{
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

    if (!initialized_) {
        return;
    }

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
   IKSolutionGenerator solgen = robot_model_.compute_all_ik_solutions(new_eef_pose, curr_joint_angles);

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
       rs_->getJointState("arm_1_shoulder_twist")->setVariableValues(&(*next)[0]);
       rs_->getJointState("arm_2_shoulder_lift")->setVariableValues(&(*next)[1]);
       rs_->getJointState("arm_3_elbow_twist")->setVariableValues(&(*next)[2]);
       rs_->getJointState("arm_4_elbow_lift")->setVariableValues(&(*next)[3]);
       rs_->getJointState("arm_5_wrist_twist")->setVariableValues(&(*next)[4]);
       rs_->getJointState("arm_6_wrist_lift")->setVariableValues(&(*next)[5]);
       rs_->getJointState("arm_7_gripper_lift")->setVariableValues(&(*next)[6]);
       rs_->updateLinkTransforms();
       publish_phantom_robot_visualizations();
   }
   else {
       ROS_WARN("Not enough IK solutions to cycle");
   }

    update_sliders();
}

void ManipulatorCommandPanel::change_joint_1(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_1_shoulder_twist")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::change_joint_2(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_2_shoulder_lift")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::change_joint_3(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_3_elbow_twist")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::change_joint_4(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_4_elbow_lift")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::change_joint_5(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_5_wrist_twist")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::change_joint_6(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_6_wrist_lift")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

void ManipulatorCommandPanel::change_joint_7(int value)
{
    double joint_val = sbpl::utils::ToRadians((double)value);
    rs_->getJointState("arm_7_gripper_lift")->setVariableValues(&joint_val);
    rs_->updateLinkTransforms();
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose("hdt_arm_control", new_marker_pose, header);
    server_.applyChanges();
    publish_phantom_robot_visualizations();
}

bool ManipulatorCommandPanel::do_init()
{
    if (!reinit_robot()) {
        return false;
    }

    move_arm_thread_ = std::thread(std::bind(&ManipulatorCommandPanel::monitor_move_command, this));

    initialized_ = true;
    return initialized_;
}

int ManipulatorCommandPanel::do_run()
{
    if (!initialized_) {
        ROS_ERROR("Teleop Node was not initialized before run");
        return 1;
    }

    ros::Rate loop_rate(60.0);
    while (ros::ok()) {
        // process interactive markers and update joint states from service call
        ros::spinOnce();
        publish_phantom_robot_visualizations();
        loop_rate.sleep();
    }
    return 0;
}

void ManipulatorCommandPanel::do_process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

    if (!initialized_) {
        return;
    }

    Eigen::Affine3d new_eef_pose;
    tf::poseMsgToEigen(feedback->pose, new_eef_pose);

    // manipulator -> eef = manipulator -> base * base -> root * root -> eef
    new_eef_pose = mount_frame_to_manipulator_frame_.inverse() * rs_->getFrameTransform(base_link_).inverse() * new_eef_pose;

    // ROS_INFO("Manipulator -> EndEffector: (%s)", to_string(new_eef_pose).c_str());

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
    std::vector<double> iksol;

    // run ik to the pose of the marker
    if (robot_model_.search_nearest_ik(new_eef_pose, curr_joint_angles, iksol, ik_search_res)) {
        rs_->getJointState("arm_1_shoulder_twist")->setVariableValues(&iksol[0]);
        rs_->getJointState("arm_2_shoulder_lift")->setVariableValues(&iksol[1]);
        rs_->getJointState("arm_3_elbow_twist")->setVariableValues(&iksol[2]);
        rs_->getJointState("arm_4_elbow_lift")->setVariableValues(&iksol[3]);
        rs_->getJointState("arm_5_wrist_twist")->setVariableValues(&iksol[4]);
        rs_->getJointState("arm_6_wrist_lift")->setVariableValues(&iksol[5]);
        rs_->getJointState("arm_7_gripper_lift")->setVariableValues(&iksol[6]);
        rs_->updateLinkTransforms();
    }

    // update the position of the marker (in case of ik failure and for synchronization)
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), new_marker_pose);
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = rm_->getRootLinkName();
    header.stamp = ros::Time(0);
    server_.setPose(feedback->marker_name, new_marker_pose, header);
    server_.applyChanges();

    publish_phantom_robot_visualizations();
    update_sliders();
}

void ManipulatorCommandPanel::publish_phantom_robot_visualizations()
{
    visualization_msgs::MarkerArray marker_array;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.94;
    color.g = 0.44;
    color.b = 0.44;
    std::string ns = "phantom";
    ros::Duration d(0);

    const std::vector<std::string>& link_names = rm_->getLinkModelNames();
//    rs_->getRobotMarkers(marker_array, link_names, color, ns, d, false);
    for (auto& marker : marker_array.markers) {
//        marker.header.frame_id = rm_->getRootLinkName();
    }

    gatherRobotMarkers(*rs_, link_names, color, ns, d, marker_array);
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

bool ManipulatorCommandPanel::get_joint_value(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_value)
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
    std::unique_lock<std::mutex> lock(critical_state_mutex_);

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

bool ManipulatorCommandPanel::reinit_robot()
{
    if (!nh_.hasParam("robot_description") || !nh_.hasParam("robot_description_semantic")) {
        ROS_ERROR("Failed to initialize Manipulator Command Panel; requires \"robot_description\" and \"robot_description_semantic\" parameters");
        return false;
    }

    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    if (!robot_model_.load(urdf_string)) {
        ROS_ERROR("Failed to load robot model from the URDF");
        return false;
    }

    if (rm_loader_) {
        delete rm_loader_;
    }

    rm_loader_ = new robot_model_loader::RobotModelLoader;
    if (!rm_loader_) {
        ROS_ERROR("Failed to instantiate Robot Model Loader");
        return false;
    }

    rm_ = rm_loader_->getModel();
    if (!rm_) {
        ROS_ERROR("Robot Model Loader was unable to construct Robot Model");
        return false;
    }

    ROS_INFO("Root link name: %s", rm_->getRootLinkName().c_str());
    ROS_INFO("Robot Joints:");
    for (const std::string& joint_name : rm_->getJointModelNames()) {
        ROS_INFO("    %s", joint_name.c_str());
    }

    rs_.reset(new robot_state::RobotState(rm_));
    if (!rs_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    rs_->setToDefaultValues();
    rs_->setRootTransform(Eigen::Affine3d::Identity());
    rs_->updateLinkTransforms();

    // initialize an interactive marker for the first joint group that's a kinematic chain
    interactive_markers_.reserve(rm_->getJointModelGroupNames().size());
    for (const auto& joint_model_group_name : rm_->getJointModelGroupNames()) {
        const robot_model::JointModelGroup* jmg = rm_->getJointModelGroup(joint_model_group_name);
        if (jmg->isChain()) {
            // find the tip link of the arm and the link to which the arm is attached
            tip_link_ = get_tip_link(*jmg);
            base_link_ = get_base_link(*jmg);
            ROS_INFO("Found manipulator attached to %s with tip link %s", base_link_.c_str(), tip_link_.c_str());

            ROS_INFO("Joint Group %s:", jmg->getName().c_str());
            ROS_INFO("    Joints:");
            for (const std::string& joint_name : jmg->getJointModelNames()) {
                ROS_INFO("        %s", joint_name.c_str());
            }
            ROS_INFO("    Links:");
            for (const std::string& link_name : jmg->getLinkModelNames()) {
                ROS_INFO("        %s", link_name.c_str());
            }

            // insert an interactive marker for the tip link of the arm
            visualization_msgs::InteractiveMarker interactive_marker;
            interactive_marker.header.seq = 0;
            interactive_marker.header.stamp = ros::Time(0);
            interactive_marker.header.frame_id = rm_->getRootLinkName();
            tf::poseEigenToMsg(rs_->getLinkState(tip_link_)->getGlobalLinkTransform(), interactive_marker.pose);
            interactive_marker.name = jmg->getName() + "_control";
            interactive_marker.description = std::string("Control of ") + tip_link_ + std::string(" of manipulator ") + jmg->getName();
            interactive_marker.scale = 1.0f;
            interactive_marker.menu_entries.clear();
            interactive_marker.controls.clear();
            interactive_marker.controls = create_sixdof_controls();

            ROS_INFO("Inserting interactive marker \"%s\"", interactive_marker.name.c_str());
            server_.insert(interactive_marker);
            server_.setCallback(interactive_marker.name, boost::bind(&ManipulatorCommandPanel::do_process_feedback, this, _1));
            interactive_markers_.push_back(interactive_marker);
            break;
        }
    }


    server_.applyChanges();

    // cache limits for interpolation command
    min_limits_.clear();
    min_limits_.reserve(7);
    min_limits_.push_back(rm_->getJointModel("arm_1_shoulder_twist")->getVariableLimits()[0].min_position);
    min_limits_.push_back(rm_->getJointModel("arm_2_shoulder_lift")->getVariableLimits()[0].min_position);
    min_limits_.push_back(rm_->getJointModel("arm_3_elbow_twist")->getVariableLimits()[0].min_position);
    min_limits_.push_back(rm_->getJointModel("arm_4_elbow_lift")->getVariableLimits()[0].min_position);
    min_limits_.push_back(rm_->getJointModel("arm_5_wrist_twist")->getVariableLimits()[0].min_position);
    min_limits_.push_back(rm_->getJointModel("arm_6_wrist_lift")->getVariableLimits()[0].min_position);
    min_limits_.push_back(rm_->getJointModel("arm_7_gripper_lift")->getVariableLimits()[0].min_position);

    max_limits_.clear();
    max_limits_.reserve(7);
    max_limits_.push_back(rm_->getJointModel("arm_1_shoulder_twist")->getVariableLimits()[0].max_position);
    max_limits_.push_back(rm_->getJointModel("arm_2_shoulder_lift")->getVariableLimits()[0].max_position);
    max_limits_.push_back(rm_->getJointModel("arm_3_elbow_twist")->getVariableLimits()[0].max_position);
    max_limits_.push_back(rm_->getJointModel("arm_4_elbow_lift")->getVariableLimits()[0].max_position);
    max_limits_.push_back(rm_->getJointModel("arm_5_wrist_twist")->getVariableLimits()[0].max_position);
    max_limits_.push_back(rm_->getJointModel("arm_6_wrist_lift")->getVariableLimits()[0].max_position);
    max_limits_.push_back(rm_->getJointModel("arm_7_gripper_lift")->getVariableLimits()[0].max_position);

    joint_1_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[0])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));
    joint_2_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[1])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));
    joint_3_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[2])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));
    joint_4_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[3])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));
    joint_5_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[4])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));
    joint_6_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[5])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));
    joint_7_slider_->setRange((int)round(sbpl::utils::ToDegrees(min_limits_[6])), (int)round(sbpl::utils::ToDegrees(max_limits_[0])));

    update_sliders();

//    root_to_first_link_ = rs_->getFrameTransform("arm_1_shoulder_twist_link");
    const Eigen::Affine3d& root_to_manipulator_frame = rs_->getFrameTransform("arm_1_shoulder_twist_link");
    const Eigen::Affine3d& root_to_base_frame = rs_->getFrameTransform(base_link_);
    mount_frame_to_manipulator_frame_ = root_to_base_frame.inverse() * root_to_manipulator_frame;

    initialized_ = true;
    return initialized_;
}

void ManipulatorCommandPanel::monitor_move_command()
{
    while (ros::ok()) {
        std::unique_lock<std::mutex> lock(move_arm_mutex_);
        ROS_INFO("Suspending move command monitor...");
        move_arm_condvar_.wait(lock, [&send_move_command_, &shutdown_](){ return shutdown_ || send_move_command_; });
        if (shutdown_) {
            ROS_INFO("Shutting down move_arm monitor");
            return;
        }
        ROS_INFO("Awake the move command monitor!");

        hdt::MoveArmCommand::Request req;
        {
            std::unique_lock<std::mutex> lock(critical_state_mutex_);
            // mounting frame -> eef = mounting_frame -> root * root -> eef
            const Eigen::Affine3d& root_to_mount_frame = rs_->getFrameTransform(base_link_);
            tf::poseEigenToMsg(root_to_mount_frame.inverse() * rs_->getLinkState("arm_7_gripper_lift_link")->getGlobalLinkTransform(), req.goal_pose);

            // manipulator -> eef = manipulator -> mount * mount -> root * root -> end effector
            Eigen::Affine3d manipulator_frame_to_eef_frame = mount_frame_to_manipulator_frame_.inverse() * root_to_mount_frame.inverse() * rs_->getLinkState("arm_7_gripper_lift_link")->getGlobalLinkTransform();
            geometry_msgs::Pose eef_in_manipulator_frame;
            tf::poseEigenToMsg(manipulator_frame_to_eef_frame, eef_in_manipulator_frame);
            ROS_INFO("eef in manipulator frame: %s", to_string(manipulator_frame_to_eef_frame).c_str());
        }

        hdt::MoveArmCommand::Response res;

        if (!move_arm_client_.call(req, res)) {
            ROS_ERROR("Call to %s failed", move_arm_client_.getService().c_str());
        }
        else {
            ROS_INFO("Call to %s finished", move_arm_client_.getService().c_str());
        }

        send_move_command_ = false;
    }
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

std::string ManipulatorCommandPanel::to_string(const Eigen::Affine3d& transform)
{
    const Eigen::Vector3d translation(transform.translation());
    const Eigen::Quaterniond rotation(transform.rotation());
    std::stringstream ss;
    ss << "translation(x, y, z): (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")";
    ss << " ";
    ss << "rotation(w, x, y, z): (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")";
    return ss.str();
}

void ManipulatorCommandPanel::update_sliders()
{
    joint_1_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_1_shoulder_twist")->getVariableValues()[0])));
    joint_2_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_2_shoulder_lift")->getVariableValues()[0])));
    joint_3_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_3_elbow_twist")->getVariableValues()[0])));
    joint_4_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_4_elbow_lift")->getVariableValues()[0])));
    joint_5_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_5_wrist_twist")->getVariableValues()[0])));
    joint_6_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_6_wrist_lift")->getVariableValues()[0])));
    joint_7_slider_->setValue((int)round(sbpl::utils::ToDegrees(rs_->getJointState("arm_7_gripper_lift")->getVariableValues()[0])));
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::ManipulatorCommandPanel, rviz::Panel)
