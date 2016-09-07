#include "viservo_command_panel.h"

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>

// project includes
#include <rcta/common/comms/actionlib.h>

namespace rcta {

ViservoCommandPanel::ViservoCommandPanel(QWidget* parent) :
    rviz::Panel(parent),
    nh_(),
    listener_(),
    action_name_("viservo_command"),
    viservo_command_client_(),
    pending_viservo_command_(false),
    send_viservo_command_button_(nullptr)
{
    QVBoxLayout* layout = new QVBoxLayout;
    send_viservo_command_button_ = new QPushButton(tr("Visual Servo"));
    layout->addWidget(send_viservo_command_button_);
    setLayout(layout);

    connect(send_viservo_command_button_, SIGNAL(clicked()), this, SLOT(send_viservo_command()));
}

void ViservoCommandPanel::send_viservo_command()
{
    if (!ReconnectActionClient(viservo_command_client_, action_name_)) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Viservo Command (server is not connected)"));
        return;
    }

    const std::string camera_link_name = "camera_rgb_frame";
    const std::string wrist_link_name = "arm_7_gripper_lift_link";

    geometry_msgs::PoseStamped wrist_in_camera;
    wrist_in_camera.header.frame_id = wrist_link_name;
    wrist_in_camera.header.stamp = ros::Time(0);
    wrist_in_camera.pose = geometry_msgs::IdentityPose();
    geometry_msgs::PoseStamped wrist_in_camera_pose;
    try {
        listener_.transformPose(camera_link_name, wrist_in_camera, wrist_in_camera_pose);
    }
    catch (const tf::TransformException& ex) {
        ROS_ERROR("Failed to find fixed transform between '%s' and %s'", camera_link_name.c_str(), wrist_link_name.c_str());
        return;
    }

    Eigen::Affine3d T_camera_wrist;
    tf::poseMsgToEigen(wrist_in_camera_pose.pose, T_camera_wrist);

    ROS_INFO("T(wrist, camera): %s", to_string(T_camera_wrist).c_str());

    // construct the goal wrist pose in the camera frame; the goal pose should
    // be the same pose as whatever pose we think we're currently at.
    rcta::ViservoCommandGoal viservo_goal;
    tf::poseEigenToMsg(T_camera_wrist, viservo_goal.goal_pose);
    auto result_callback = boost::bind(&ViservoCommandPanel::viservo_command_result_cb, this, _1, _2);
    viservo_command_client_->sendGoal(viservo_goal, result_callback);

    pending_viservo_command_ = true;
    update_gui();
}

void ViservoCommandPanel::viservo_command_active_cb()
{
}

void ViservoCommandPanel::viservo_command_feedback_cb(const rcta::ViservoCommandFeedback::ConstPtr& feedback)
{
}

void ViservoCommandPanel::viservo_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const rcta::ViservoCommandResult::ConstPtr& result)
{
    ROS_INFO("Received Result from Viservo Command Action");
    pending_viservo_command_ = false;
    update_gui();
}

void ViservoCommandPanel::update_gui()
{
    ROS_INFO("    Pending Viservo Command: %s", pending_viservo_command_ ? "TRUE": "FALSE");
    send_viservo_command_button_->setEnabled(!pending_viservo_command_);
}

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::ViservoCommandPanel, rviz::Panel)
