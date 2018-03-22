#ifndef rcta_viservo_command_panel_h
#define rcta_viservo_command_panel_h

// standard includes
#include <memory>
#include <string>

// system includes
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <rviz/panel.h>
#include <tf/transform_listener.h>

// project includes
#include <rcta/ViservoCommandAction.h>

namespace rcta {

class ViservoCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    ViservoCommandPanel(QWidget* parent = 0);

public Q_SLOTS:

    void send_viservo_command();

private:

    ros::NodeHandle nh_;

    tf::TransformListener listener_;

    typedef actionlib::SimpleActionClient<rcta::ViservoCommandAction> ViservoCommandActionClient;
    std::string action_name_;
    std::unique_ptr<ViservoCommandActionClient> viservo_command_client_;
    bool pending_viservo_command_;

    QPushButton* send_viservo_command_button_;

    void viservo_command_active_cb();
    void viservo_command_feedback_cb(const rcta::ViservoCommandFeedback::ConstPtr& feedback);
    void viservo_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const rcta::ViservoCommandResult::ConstPtr& result);

    void update_gui();
};

} // namespace rcta

#endif
