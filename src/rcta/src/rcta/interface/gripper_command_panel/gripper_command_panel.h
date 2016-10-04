#ifndef rcta_gripper_command_panel_h
#define rcta_gripper_command_panel_h

// standard includes
#include <memory>
#include <string>

// system includes
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/ros.h>
#include <rviz/panel.h>

namespace rcta {

// TODO: emit configChanged() when line edit changes (maybe changes to something
// valid)...a valid response returned from one of action server requests?

class GripperCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    GripperCommandPanel(QWidget* parent = 0);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    void sendOpenGripperCommand();
    void sendCloseGripperCommand();

private:

    ros::NodeHandle nh_;

    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;
    std::string action_name_;
    std::unique_ptr<GripperCommandActionClient> gripper_command_client_;
    bool pending_gripper_command_;

    // Gripper Command Widgets
    QLineEdit* action_name_line_edit_;
    QPushButton* send_open_gripper_command_button_;
    QPushButton* send_close_gripper_command_button_;

    void gripperCommandActiveCallback();
    void gripperCommandFeedbackCallback(
        const control_msgs::GripperCommandFeedback::ConstPtr& feedback);
    void gripperCommandResultCallback(
        const actionlib::SimpleClientGoalState& state,
        const control_msgs::GripperCommandResult::ConstPtr& result);
};

} // namespace rcta

#endif
