#include "gripper_command_panel.h"

// project includes
#include <rcta/common/comms/actionlib.h>
#include <rcta/control/robotiq_controllers/gripper_model.h>

namespace rcta {

GripperCommandPanel::GripperCommandPanel(QWidget* parent) :
    rviz::Panel(parent),
    nh_(),
    action_name_("gripper_controller/gripper_command_action"),
    gripper_command_client_(),
    pending_gripper_command_(false),
    send_open_gripper_command_button_(nullptr),
    send_close_gripper_command_button_(nullptr)
{
    QVBoxLayout* gripper_commands_layout = new QVBoxLayout;
    send_open_gripper_command_button_ = new QPushButton(tr("Open Gripper"));
    send_close_gripper_command_button_ = new QPushButton(tr("Close Gripper"));
    gripper_commands_layout->addWidget(send_open_gripper_command_button_);
    gripper_commands_layout->addWidget(send_close_gripper_command_button_);
    setLayout(gripper_commands_layout);

    connect(send_open_gripper_command_button_, SIGNAL(clicked()), this, SLOT(sendOpenGripperCommand()));
    connect(send_close_gripper_command_button_, SIGNAL(clicked()), this, SLOT(sendCloseGripperCommand()));
}

void GripperCommandPanel::sendOpenGripperCommand()
{
    if (!ReconnectActionClient(gripper_command_client_, action_name_)) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Gripper Command (server is not connected)"));
        return;
    }

    control_msgs::GripperCommandGoal gripper_command_goal;
    gripper_command_goal.command.position = GripperModel().maximum_width();
    gripper_command_goal.command.max_effort = GripperModel().maximum_force();

    auto result_cb = boost::bind(&GripperCommandPanel::gripperCommandResultCallback, this, _1, _2);
    gripper_command_client_->sendGoal(gripper_command_goal, result_cb);
    pending_gripper_command_ = true;
}

void GripperCommandPanel::sendCloseGripperCommand()
{
    if (!ReconnectActionClient(gripper_command_client_, action_name_)) {
        QMessageBox::warning(this, tr("Connection Failure"), tr("Unable to send Gripper Command (server is not connected)"));
        return;
    }

    control_msgs::GripperCommandGoal gripper_command_goal;
    gripper_command_goal.command.position = GripperModel().minimum_width();
    gripper_command_goal.command.max_effort = GripperModel().maximum_force();

    auto result_cb = boost::bind(&GripperCommandPanel::gripperCommandResultCallback, this, _1, _2);
    gripper_command_client_->sendGoal(gripper_command_goal, result_cb);
    pending_gripper_command_ = true;
}

void GripperCommandPanel::gripperCommandResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::GripperCommandResult::ConstPtr& result)
{
    pending_gripper_command_ = false;
}

} // namespace rcta

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rcta::GripperCommandPanel, rviz::Panel)
