#include "teleop_status_panel.h"
#include "ui_teleop_status_panel.h"

namespace hdt
{

TeleopStatusPanel::TeleopStatusPanel(QWidget* parent) :
    rviz::Panel(parent),
    ui(new Ui::TeleopStatusPanel),
    nh_(),
    teleop_diagnostic_sub_(),
    last_diagnostic_status_(),
    joint_name_to_label_names_({
        { "arm_1_shoulder_twist", { "a1_name", "a1_position", "a1_velocity", "a1_acceleration", "a1_at_limit" }},
        { "arm_2_shoulder_lift",  { "a2_name", "a2_position", "a2_velocity", "a2_acceleration", "a2_at_limit" }},
        { "arm_3_elbow_twist",    { "a3_name", "a3_position", "a3_velocity", "a3_acceleration", "a3_at_limit" }},
        { "arm_4_elbow_lift",     { "a4_name", "a4_position", "a4_velocity", "a4_acceleration", "a4_at_limit" }},
        { "arm_5_wrist_twist",    { "a5_name", "a5_position", "a5_velocity", "a5_acceleration", "a5_at_limit" }},
        { "arm_6_wrist_lift",     { "a6_name", "a6_position", "a6_velocity", "a6_acceleration", "a6_at_limit" }},
        { "arm_7_gripper_lift",   { "a7_name", "a7_position", "a7_velocity", "a7_acceleration", "a7_at_limit" }},
    })
{
    ui->setupUi(this);

    teleop_diagnostic_sub_ = nh_.subscribe("teleop_diagnostics", 1, &TeleopStatusPanel::teleop_diagnostic_callback, this);

    // todo: call teleop_diagnostic_callback
}

TeleopStatusPanel::~TeleopStatusPanel()
{
    delete ui;
}

QLabel* TeleopStatusPanel::find_label(const std::string& label_name)
{
    QLabel* label = this->findChild<QLabel*>(QString::fromStdString(label_name));
    return label;
}

void TeleopStatusPanel::teleop_diagnostic_callback(const hdt::TeleopDiagnosticStatus::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];

        const JointLabelGroup& joint_labels = joint_name_to_label_names_[joint_name];

        // convenience
        const std::string& joint_label_name = joint_labels.joint_label_name;
        const std::string& pos_label_name = joint_labels.pos_label_name;
        const std::string& vel_label_name = joint_labels.vel_label_name;
        const std::string& accel_label_name = joint_labels.accel_label_name;
        const std::string& at_limit_label_name = joint_labels.at_limit_label_name;

        QLabel* joint_label = find_label(joint_label_name);
        QLabel* position_label = find_label(pos_label_name);
        QLabel* velocity_label = find_label(vel_label_name);
        QLabel* accel_label = find_label(accel_label_name);
        QLabel* at_limit_label = find_label(at_limit_label_name);
        if (!joint_label || !position_label || !velocity_label || !accel_label || !at_limit_label) {
            ROS_WARN("Failed to find label for joint '%s' (name: %p, position: %p, velocity, %p, acceleration: %p, at limit: %p", joint_label_name.c_str(), joint_label, position_label, velocity_label, accel_label, at_limit_label);
            continue;
        }

        // color the joint labels appropriately
        const std::string& selected_joint = msg->name[msg->current_selection];
        if (selected_joint == joint_name &&
                (!last_diagnostic_status_ ||
                 selected_joint != last_diagnostic_status_->name[last_diagnostic_status_->current_selection]))
        {
            // update the selected joint

            QPalette old_palette = joint_label->palette();
            old_palette.setColor(QPalette::WindowText, QColor(Qt::green));
            joint_label->setPalette(old_palette);

            // make sure to update the previously selected joint if one exists
            if (last_diagnostic_status_) {
                const std::string& last_selected_joint = last_diagnostic_status_->name[last_diagnostic_status_->current_selection];
                const JointLabelGroup& prev_selected_joint_labels = joint_name_to_label_names_[last_selected_joint];
                QLabel* prev_selected_label = find_label(prev_selected_joint_labels.joint_label_name);
                QPalette prev_selected_label_palette = prev_selected_label->palette();
                prev_selected_label_palette.setColor(QPalette::WindowText, QColor(Qt::blue));
                prev_selected_label->setPalette(prev_selected_label_palette);
            }
        }

        // color unselected joints for the first time
        if (!last_diagnostic_status_ && selected_joint != joint_name) {
            // color other joint labels the first time we receive a message
            QPalette old_palette = joint_label->palette();
            old_palette.setColor(QPalette::WindowText, QColor(Qt::blue));
            joint_label->setPalette(old_palette);
        }

        // set position text
        position_label->setText(QString::fromStdString(std::to_string(msg->position[i])));

        // set at limit text and color
        if (!last_diagnostic_status_ || // first msg
                (last_diagnostic_status_ && // changes on subsequence messages
                (msg->at_limit[i] != last_diagnostic_status_->at_limit[i])))
        {
            QPalette old_palette = at_limit_label->palette();
            old_palette.setColor(QPalette::WindowText, msg->at_limit[i] ? Qt::red : Qt::green);
            at_limit_label->setPalette(old_palette);
        }

        at_limit_label->setText(QString(msg->at_limit[i] ? "YES" : "NO"));

        velocity_label->setText(QString::fromStdString(std::to_string(msg->velocity[i])));
        accel_label->setText(QString::fromStdString(std::to_string(msg->effort[i])));
    }

    // update here so that we can check for "haven't received a message yet" cases above
    last_diagnostic_status_ = msg;
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::TeleopStatusPanel, rviz::Panel)
