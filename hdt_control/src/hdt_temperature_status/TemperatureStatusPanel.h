#ifndef TemperatureStatusPanel_h
#define TemperatureStatusPanel_h

// system includes
#include <ros/ros.h>
#include <rviz/panel.h>

// project includes
#include <rcta/ControllerDiagnosticStatus.h>

namespace Ui
{
class TemperatureStatusPanelWidget;
}

class TemperatureStatusPanel : public rviz::Panel
{
    Q_OBJECT

public:

    explicit TemperatureStatusPanel(QWidget *parent = 0);
    ~TemperatureStatusPanel();

private:

    Ui::TemperatureStatusPanelWidget *ui_;

    ros::NodeHandle nh_;
    ros::Subscriber controller_diagnostics_sub_;

    void controller_diagnostics_callback(const rcta::ControllerDiagnosticStatus::ConstPtr &msg);
};

#endif
