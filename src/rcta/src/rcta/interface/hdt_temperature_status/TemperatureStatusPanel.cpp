#include "TemperatureStatusPanel.h"

// system includes
#include <qwt_thermo.h>

// module includes
#include "ui_TemperatureStatusPanelWidget.h"

TemperatureStatusPanel::TemperatureStatusPanel(QWidget *parent) :
    ui_(nullptr),
    nh_(),
    controller_diagnostics_sub_()
{
    ui_ = new Ui::TemperatureStatusPanelWidget;
    ui_->setupUi(this);

//    QwtThermo* thermo;
    ui_->j1_thermo->setMaxValue(100.0);
    ui_->j2_thermo->setMaxValue(100.0);
    ui_->j3_thermo->setMaxValue(100.0);
    ui_->j4_thermo->setMaxValue(100.0);
    ui_->j5_thermo->setMaxValue(100.0);
    ui_->j6_thermo->setMaxValue(100.0);
    ui_->j7_thermo->setMaxValue(100.0);

    ui_->j1_thermo->setAlarmLevel(50.0);
    ui_->j2_thermo->setAlarmLevel(50.0);
    ui_->j3_thermo->setAlarmLevel(50.0);
    ui_->j4_thermo->setAlarmLevel(50.0);
    ui_->j5_thermo->setAlarmLevel(50.0);
    ui_->j6_thermo->setAlarmLevel(50.0);
    ui_->j7_thermo->setAlarmLevel(50.0);

    controller_diagnostics_sub_ = nh_.subscribe(
            "hdt_diagnostics", 1, &TemperatureStatusPanel::controller_diagnostics_callback, this);
}

TemperatureStatusPanel::~TemperatureStatusPanel()
{
    delete ui_;
}

void TemperatureStatusPanel::controller_diagnostics_callback(const rcta::ControllerDiagnosticStatus::ConstPtr &msg)
{
    if (msg->joint_status.size() < 7) {
        ROS_WARN("Insufficient number of joints from Controller Diagnostic Status message");
        return;
    }

    ui_->j1_thermo->setValue(msg->joint_status[0].temperature);
    ui_->j2_thermo->setValue(msg->joint_status[1].temperature);
    ui_->j3_thermo->setValue(msg->joint_status[2].temperature);
    ui_->j4_thermo->setValue(msg->joint_status[3].temperature);
    ui_->j5_thermo->setValue(msg->joint_status[4].temperature);
    ui_->j6_thermo->setValue(msg->joint_status[5].temperature);
    ui_->j7_thermo->setValue(msg->joint_status[6].temperature);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TemperatureStatusPanel, rviz::Panel);
