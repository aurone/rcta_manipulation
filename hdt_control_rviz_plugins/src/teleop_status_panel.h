#ifndef hdt_TeleopStatusPanel_h
#define hdt_TeleopStatusPanel_h

// standard includes
#include <string>

// system includes
#include <QLabel>
#include <QWidget>
#ifndef Q_MOC_RUN
#include <hdt_control_msgs/TeleopDiagnosticStatus.h>
#include <rviz/panel.h>
#include <ros/ros.h>
#endif

namespace Ui
{
class TeleopStatusPanel;
} // namespace Ui

namespace hdt
{

class TeleopStatusPanel : public rviz::Panel
{
    Q_OBJECT

public:

    explicit TeleopStatusPanel(QWidget* parent = 0);
    ~TeleopStatusPanel();

private:

    struct JointLabelGroup
    {
        std::string joint_label_name;
        std::string pos_label_name;
        std::string vel_label_name;
        std::string accel_label_name;
        std::string at_limit_label_name;
    };

    Ui::TeleopStatusPanel* ui;

    ros::NodeHandle nh_;
    ros::Subscriber teleop_diagnostic_sub_;

    hdt_control_msgs::TeleopDiagnosticStatus::ConstPtr last_diagnostic_status_;

    std::map<std::string, JointLabelGroup> joint_name_to_label_names_;

    QLabel* find_label(const std::string& label_name);
    void teleop_diagnostic_callback(const hdt_control_msgs::TeleopDiagnosticStatus::ConstPtr& msg);
};

} // namespace hdt

#endif
