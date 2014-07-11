#ifndef hdt_ControllerStatusPanelWidget_h
#define hdt_ControllerStatusPanelWidget_h

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <QLabel>
#include <QPushButton>
#include <QWidget>
#include <ros/ros.h>
#include <rospack/rospack.h>
#include <rviz/panel.h>
#include <std_msgs/Empty.h>
#include <hdt/ControllerDiagnosticStatus.h>

namespace Ui {
class ControllerStatusPanelWidget;
}

namespace hdt
{

class ControllerStatusPanelWidget : public rviz::Panel
{
    Q_OBJECT

public:
    explicit ControllerStatusPanelWidget(QWidget* parent = 0);
    ~ControllerStatusPanelWidget();

private Q_SLOTS:

    void on_emergency_stop_button_clicked();

    void on_clear_emergency_stop_button_clicked();

private:

    Ui::ControllerStatusPanelWidget* ui;

    std::vector<std::string> icon_resources_;
    std::vector<std::string> icon_colors_;

    std::map<std::string, QPixmap> icons_;

    ros::NodeHandle nh_;
    ros::Subscriber hdt_diagnostics_sub_;
    rospack::Rospack rospack_;

    std::mutex msg_mutex_;
    hdt::ControllerDiagnosticStatus::ConstPtr last_msg_;

    bool shutdown_watchdog_;
    std::thread watchdog_;
    ros::Publisher staleness_pub_;
    ros::Publisher estop_pub_;
    ros::Publisher clear_estop_pub_;
    ros::Subscriber staleness_sub_;

    bool load_resources();

    void diagnostics_callback(const hdt::ControllerDiagnosticStatus::ConstPtr& msg);
    void staleness_callback(const std_msgs::Empty::ConstPtr& msg);

    QPixmap get_active_icon() const;
    QPixmap get_inactive_icon() const;
    QPixmap get_alert_icon() const;
    QPixmap get_stale_icon() const;

    QLabel* find_label(const std::string& label_name);
    void set_active(const std::string& label_name);
    void set_inactive(const std::string& label_name);
    void set_alert(const std::string& label_name);
    void set_stale(const std::string& label_name);

    QPushButton* find_button(const std::string& button_name);

    void refresh_icons(const hdt::ControllerDiagnosticStatus& msg);

    void watchdog_thread();
};

} // namespace hdt

#endif
