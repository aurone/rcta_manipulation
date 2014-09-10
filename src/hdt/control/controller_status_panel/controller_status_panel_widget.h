#ifndef hdt_ControllerStatusPanelWidget_h
#define hdt_ControllerStatusPanelWidget_h

#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <QLabel>
#include <QPushButton>
#include <QWidget>
#include <boost/circular_buffer.hpp>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <ros/ros.h>
#include <rospack/rospack.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <hdt/ControllerDiagnosticStatus.h>
#include <hdt_description/RobotModel.h>

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

    mutable hdt::RobotModelPtr robot_model_;

    std::mutex msg_mutex_;
    hdt::ControllerDiagnosticStatus::ConstPtr last_msg_;

    bool shutdown_watchdog_;
    std::thread watchdog_;
    ros::Publisher staleness_pub_;
    ros::Publisher estop_pub_;
    ros::Publisher clear_estop_pub_;
    ros::Subscriber staleness_sub_;

    ros::Subscriber raw_joint_states_sub_;
    ros::Subscriber joint_states_sub_;

    std::vector<boost::circular_buffer<double>> past_joint_states_;
    std::vector<boost::circular_buffer<double>> past_raw_joint_states_;
    std::vector<QwtPlotCurve*> joint_states_curves_;
    std::vector<QwtPlotCurve*> raw_joint_states_curves_;

//    std::map<std::string, boost::circular_buffer<double>> past_joint_states_; // circular buffer for each joint state
//    std::map<std::string, boost::circular_buffer<double>> past_raw_joint_states_; // circular buffer for each raw joint state
//    std::map<std::string, QwtPlotCurve*> joint_states_curves_;
//    std::map<std::string, QwtPlotCurve*> raw_joint_states_curves_;

    bool load_resources();

    void diagnostics_callback(const hdt::ControllerDiagnosticStatus::ConstPtr& msg);
    void staleness_callback(const std_msgs::Empty::ConstPtr& msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void raw_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

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

    int get_joint_index(const std::string& joint_name) const;

    hdt::RobotModelPtr lazy_robot_model() const;

    void initialize_gui();
};

} // namespace hdt

#endif
