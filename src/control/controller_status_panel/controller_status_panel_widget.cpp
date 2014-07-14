#include <functional>
#include <boost/algorithm/string.hpp>
#include <hdt/ClearEmergencyStop.h>
#include <hdt/EmergencyStop.h>
#include "controller_status_panel_widget.h"
#include "ui_controller_status_panel_widget.h"

namespace hdt
{

ControllerStatusPanelWidget::ControllerStatusPanelWidget(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::ControllerStatusPanelWidget),
    icon_resources_({ "At.png", "Blank.png", "Cancel.png", "Cash.png",
                      "Chat.png", "CheckMark.png", "Copy.png", "Cut.png",
                      "Doc.png", "Download.png", "Down.png", "Edit.png",
                      "Exclamation.png", "Folder.png", "Hand.png", "Heart.png",
                      "Home.png", "Info.png", "Key.png", "Left.png",
                      "Magnifier.png", "Mail.png", "Minus.png", "MusicalNote.png",
                      "Paint.png", "Paste.png", "Plus.png", "Print.png",
                      "QuestionMark.png", "Recycle.png", "Redo2.png", "Redo.png",
                      "Restart.png", "Right.png", "Save.png", "Shotdown.png",
                      "Sound.png", "Standby.png", "Star.png", "Text.png",
                      "Tools.png", "Trash.png", "Undo.png", "Up.png",
                      "User.png", "Users.png", "Web.png", "ZoomIn.png",
                      "ZoomOut.png" }),
    icon_colors_({ "blue", "green", "red", "smoke" }),
    shutdown_watchdog_(false),
    watchdog_()
{
    std::string ros_root(getenv("ROS_ROOT"));
    if (ros_root.empty()) {
        ROS_ERROR("Failed to initialize 'Controller Status Plugin'. ROS_ROOT is not set");
        return;
    }

    std::string ros_package_path(getenv("ROS_PACKAGE_PATH"));
    if (ros_package_path.empty()) {
        ROS_ERROR("Failed to initialize 'Controller Status Plugin'. ROS_PACKAGE_PATH is not set");
        return;
    }

    std::vector<std::string> search_path;
    boost::split(search_path, ros_package_path, boost::is_any_of(":"));

    rospack_.crawl(search_path, false);

    std::string path;
    const std::string package_name = "hdt";
    ROS_INFO("Searching for '%s' package", package_name.c_str());
    if (!rospack_.find(package_name, path)) {
        ROS_ERROR("Failed to find package '%s'", package_name.c_str());
        return;
    }
    ROS_INFO("Found '%s' at '%s'", package_name.c_str(), path.c_str());

    for (const std::string& color :  icon_colors_) {
        for (const std::string& resource : icon_resources_) {
            const std::string rel_resource = color + "/" + resource;
            const std::string abs_resource = path + "/resource/icons/" + rel_resource;

            if (!icons_[rel_resource].load(QString::fromStdString(abs_resource), "png")) {
                ROS_ERROR("Failed to load resource '%s'", abs_resource.c_str());
                return;
            }
        }
    }

    hdt_diagnostics_sub_ = nh_.subscribe<hdt::ControllerDiagnosticStatus>("hdt_diagnostics", 5, &ControllerStatusPanelWidget::diagnostics_callback, this);
    raw_joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("raw_joint_states", 1, &ControllerStatusPanelWidget::joint_states_callback, this);
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &ControllerStatusPanelWidget::raw_joint_states_callback, this);

    staleness_pub_ = nh_.advertise<std_msgs::Empty>("controller_staleness", 1);
    estop_pub_ = nh_.advertise<hdt::EmergencyStop>("hdt_estop", 1);
    clear_estop_pub_ = nh_.advertise<hdt::ClearEmergencyStop>("clear_hdt_estop", 1);
    staleness_sub_ = nh_.subscribe<std_msgs::Empty>("controller_staleness", 1, &ControllerStatusPanelWidget::staleness_callback, this);

    ui->setupUi(this);

    set_inactive("reset_occurred_icon");
    set_inactive("controllable_icon");
    set_inactive("have_supply_icon");
    set_inactive("have_comms_icon");

    watchdog_ = std::thread(std::bind(&ControllerStatusPanelWidget::watchdog_thread, this));

    // initialize joint state plots

    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return;
    }

    if (!robot_model_.load(urdf_string)) {
        ROS_ERROR("Failed to initialize Robot Model from URDF string");
        return;
    }

    const int buffer_size = 1000;
    past_joint_states_.resize(robot_model_.joint_names().size());
    past_raw_joint_states_.resize(robot_model_.joint_names().size());
    joint_states_curves_.resize(robot_model_.joint_names().size());
    raw_joint_states_curves_.resize(robot_model_.joint_names().size());
    for (std::size_t i = 0; i < robot_model_.joint_names().size(); ++i) {
        const std::string& joint_name = robot_model_.joint_names()[i];

        // create a circular buffer of size
        boost::circular_buffer<double>& past_joint_state = past_joint_states_[i];
        boost::circular_buffer<double>& past_raw_joint_state = past_raw_joint_states_[i];

        past_joint_state.resize(buffer_size, 0.0);
        past_raw_joint_state.resize(buffer_size, 0.0);

        // create curves for filtered joint states and raw joint states

        joint_states_curves_[i] = new QwtPlotCurve(QString("%1 Joint State").arg(QString::fromStdString(joint_name)));
        joint_states_curves_[i]->attach(ui->raw_joint_states_plot);

        raw_joint_states_curves_[i] = new QwtPlotCurve(QString("%1 Raw Joint State").arg(QString::fromStdString(joint_name)));
        raw_joint_states_curves_[i]->attach(ui->raw_joint_states_plot);
    }

    // TODO: set background and curve colors (ROYGBIV on black)
}

ControllerStatusPanelWidget::~ControllerStatusPanelWidget()
{
    shutdown_watchdog_ = true;
    if (watchdog_.joinable())
        watchdog_.join();
    delete ui;
}

bool ControllerStatusPanelWidget::load_resources()
{
    return false;
}

void ControllerStatusPanelWidget::diagnostics_callback(const hdt::ControllerDiagnosticStatus::ConstPtr& msg)
{
    std::unique_lock<std::mutex> lock(msg_mutex_);

    if (last_msg_) {
        if (last_msg_->reset_occurred != msg->reset_occurred ||
            last_msg_->is_controllable != msg->is_controllable ||
            last_msg_->have_supply != msg->have_supply ||
            last_msg_->have_comms != msg->have_comms)
        {
            refresh_icons(*msg);
        }
    }
    else {
        refresh_icons(*msg);
    }

    last_msg_ = msg;
}

void ControllerStatusPanelWidget::staleness_callback(const std_msgs::Empty::ConstPtr& msg)
{
    set_stale("reset_occurred_icon");
    set_stale("controllable_icon");
    set_stale("have_supply_icon");
    set_stale("have_comms_icon");
}

void ControllerStatusPanelWidget::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // update the buffers
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        double joint_val = msg->position[i];

        int jidx = get_joint_index(joint_name);
        if (jidx < 0) {
            continue; // not an arm joint
        }

        past_joint_states_[jidx].push_back(joint_val);
    }

    // update the data associated with the curves and refresh the joint state plots
    for (std::size_t i = 0; i < robot_model_.joint_names().size(); ++i) {
        // convert circular buffer to
        QVector<QPointF> plot_data;
        plot_data.reserve(past_joint_states_.size());
        for (std::size_t j = 0; j < past_joint_states_[i].size(); ++j) {
            plot_data.push_back(QPointF((float)j, past_joint_states_[i][j]));
        }

        joint_states_curves_[i]->setSamples(plot_data);
    }

    QwtPlot* raw_joint_states_plot = ui->raw_joint_states_plot;
    raw_joint_states_plot->replot();
}

void ControllerStatusPanelWidget::raw_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // update the buffers
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        double joint_val = msg->position[i];

        int jidx = get_joint_index(joint_name);
        if (jidx < 0) {
            continue; // not an arm joint
        }

        past_raw_joint_states_[jidx].push_back(joint_val);
    }

    // update the data associated with the curves and refresh the joint state plots
    for (std::size_t i = 0; i < robot_model_.joint_names().size(); ++i) {
        // convert circular buffer to
        QVector<QPointF> plot_data;
        plot_data.reserve(past_raw_joint_states_.size());
        for (std::size_t j = 0; j < past_raw_joint_states_[i].size(); ++j) {
            plot_data.push_back(QPointF((float)j, past_raw_joint_states_[i][j]));
        }

        raw_joint_states_curves_[i]->setSamples(plot_data);
    }

    QwtPlot* raw_joint_states_plot = ui->raw_joint_states_plot;
    raw_joint_states_plot->replot();
}

QPixmap ControllerStatusPanelWidget::get_active_icon() const
{
    return icons_.at("green/Blank.png").scaled(48, 48, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
}

QPixmap ControllerStatusPanelWidget::get_inactive_icon() const
{
    return icons_.at("blue/Blank.png").scaled(48, 48, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
}

QPixmap ControllerStatusPanelWidget::get_alert_icon() const
{
    return icons_.at("red/Blank.png").scaled(48, 48, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
}

QPixmap ControllerStatusPanelWidget::get_stale_icon() const
{
    return icons_.at("smoke/Blank.png").scaled(48, 48, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
}

QLabel* ControllerStatusPanelWidget::find_label(const std::string& label_name)
{
    QLabel* label = this->findChild<QLabel*>(QString::fromStdString(label_name));
    return label;
}

void ControllerStatusPanelWidget::set_active(const std::string& label_name)
{
    QLabel* label = find_label(label_name);
    if (label) {
        label->setPixmap(get_active_icon());
    }
}

void ControllerStatusPanelWidget::set_inactive(const std::string& label_name)
{
    QLabel* label = find_label(label_name);
    if (label) {
        label->setPixmap(get_inactive_icon());
    }
}

void ControllerStatusPanelWidget::set_alert(const std::string& label_name)
{
    QLabel* label = find_label(label_name);
    if (label) {
        label->setPixmap(get_alert_icon());
    }
}

void ControllerStatusPanelWidget::set_stale(const std::string& label_name)
{
    QLabel* label = find_label(label_name);
    if (label) {
        label->setPixmap(get_stale_icon());
    }
}

QPushButton* ControllerStatusPanelWidget::find_button(const std::string& button_name)
{
    QPushButton* button = this->findChild<QPushButton*>(QString::fromStdString(button_name));
    return button;
}

void ControllerStatusPanelWidget::refresh_icons(const hdt::ControllerDiagnosticStatus& msg)
{
    // NOTE: reset_occured boolean condition has different semantics from the other status flags
    if (msg.reset_occurred) {
        set_alert("reset_occurred_icon");
    }
    else {
        set_active("reset_occurred_icon");
    }

    if (msg.is_controllable) {
        set_active("controllable_icon");
    }
    else {
        set_alert("controllable_icon");
    }

    if (msg.have_supply) {
        set_active("have_supply_icon");
    }
    else {
        set_alert("have_supply_icon");
    }

    if (msg.have_comms) {
        set_active("have_comms_icon");
    }
    else {
        set_alert("have_comms_icon");
    }
}

void ControllerStatusPanelWidget::watchdog_thread()
{
    double loop_rate_s = 1.0;
    ros::Rate watchdog_rate(loop_rate_s);
    while (ros::ok() && !shutdown_watchdog_) {
        ros::Time now = ros::Time::now();
        msg_mutex_.lock();
        if (last_msg_) {
            if (last_msg_->header.stamp < now - ros::Duration(loop_rate_s)) {
                std_msgs::Empty msg;
                staleness_pub_.publish(msg);
                last_msg_.reset();
            }
        }
        msg_mutex_.unlock();
        watchdog_rate.sleep();
    }
}

int ControllerStatusPanelWidget::get_joint_index(const std::string& joint_name) const
{
    for (int i = 0; i < (int)robot_model_.joint_names().size(); ++i) {
        if (robot_model_.joint_names()[i] == joint_name) {
            return i;
        }
    }
    return -1;
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::ControllerStatusPanelWidget, rviz::Panel)

void hdt::ControllerStatusPanelWidget::on_emergency_stop_button_clicked()
{
    ROS_INFO("Sending Emergency Stop command!");
    hdt::EmergencyStop estop_msg;
    estop_pub_.publish(estop_msg);
}

void hdt::ControllerStatusPanelWidget::on_clear_emergency_stop_button_clicked()
{
    ROS_INFO("Clearing Emergency Stop");
    hdt::ClearEmergencyStop clear_estop_msg;
    clear_estop_pub_.publish(clear_estop_msg);
}

