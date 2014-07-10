#ifndef hdt_PickAndPlacePanel_h
#define hdt_PickAndPlacePanel_h

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandActionFeedback.h>
#include <control_msgs/GripperCommandFeedback.h>
#include <interactive_markers/interactive_marker_server.h>
#include <message_filters/subscriber.h>
#include <pr2_vfh_database/ObjectFinder.h>
#include <rviz/panel.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <hdt/ObjectDetectionAction.h>

namespace hdt
{

class PickAndPlacePanel : public rviz::Panel
{
    Q_OBJECT

public:

    explicit PickAndPlacePanel(QWidget* parent = 0);
    ~PickAndPlacePanel();

    void load(const rviz::Config& config);
    void save(rviz::Config config) const;

private Q_SLOTS:

    void choose_database();
    void choose_features();
    void choose_kdtree_indices();

    void take_snapshot();
    void update_grasps();
    void send_move_to_pregrasp_command();
    void send_open_gripper_command();
    void send_close_gripper_command();

    void camera_frame_box_activated(int index);
    void camera_frame_box_activated(const QString& text);
    void camera_frame_box_current_index_changed(int index);
    void camera_frame_box_current_index_changed(const QString& text);
    void camera_frame_box_edit_text_changed(const QString& text);
    void camera_frame_box_highlighted(int index);
    void camera_frame_box_highlighted(const QString& text);

    void root_frame_box_activated(int index);

private:

    struct ObjectDetectionQuery
    {
        sensor_msgs::PointCloud2::ConstPtr snapshot;
        std::string camera_frame;
        std::string root_frame;
        ObjectFinder::CullingOptions cull_options;

        geometry_msgs::PoseArray grasps;
        geometry_msgs::PoseArray pregrasps;
    };

    ros::NodeHandle nh_;

    // Training data selection tools
    QPushButton* open_database_button_;
    QPushButton* open_features_button_;
    QPushButton* open_kdtree_indices_button_;

    QLabel* database_fname_label_;
    QLabel* features_fname_label_;
    QLabel* kdtree_indices_fname_label_;

    // Frame data selectiont tools
    QComboBox* camera_frame_selection_;
    QComboBox* root_frame_selection_;

    // Command tools
    QPushButton* snap_point_cloud_button_;
    QPushButton* update_grasps_button_;
    QPushButton* send_move_to_pregrasp_button_;
    QPushButton* send_open_gripper_command_button_;
    QPushButton* send_close_gripper_command_button_;

    // For transforming pre-grasp/grasp goals into the root frame of the arm
    tf::TransformListener listener_;

    // for choosing available frames to send to pick and place node
    ros::Subscriber tf_sub_;
    double frame_timeout_;
    std::map<std::string, ros::Time> seen_frames_;

    // results of the last snapshot
    hdt::ObjectDetectionGoal last_detection_request_;
    hdt::ObjectDetectionResult::ConstPtr last_detection_result_;
    ros::Publisher snapshot_cloud_pub_;

    // for visualizing and selecting available pre-grasps
    interactive_markers::InteractiveMarkerServer grasp_markers_server_;
    typedef std::string InteractiveMarkerHandle;
    InteractiveMarkerHandle selected_marker_;

    ros::ServiceClient move_arm_client_;

    typedef actionlib::SimpleActionClient<hdt::ObjectDetectionAction> ObjectDetectionActionClient;
    std::unique_ptr<ObjectDetectionActionClient> object_detection_client_;

    typedef control_msgs::GripperCommandGoal GripperCommandGoal;
    typedef control_msgs::GripperCommandFeedback GripperCommandFeedback;
    typedef control_msgs::GripperCommandResult GripperCommandResult;
    typedef actionlib::ActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;
    typedef GripperCommandActionClient::GoalHandle GoalHandle;
    std::unique_ptr<GripperCommandActionClient> gripper_command_client_;

    void setup_gui();

    void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void tf_callback(const tf::tfMessage::ConstPtr& msg);

    void update_gui();

    int find_item(const QComboBox& combo_box, const std::string& item) const;
    void update_combo_box(QComboBox& combo_box, const std::map<std::string, ros::Time>& entries);

    void process_feedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback_msg);

    static visualization_msgs::Marker
    create_arrow_marker(const geometry_msgs::Vector3& scale);

    void print_interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback& feedback_msg) const;

    void gripper_command_action_feedback(GoalHandle goalHandle, const GripperCommandFeedback::ConstPtr& msg);
    void gripper_command_action_transition(GoalHandle goalHandle);

    void object_detection_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const hdt::ObjectDetectionResult::ConstPtr& result);

    void object_detection_active_cb();

    void object_detection_feedback_cb(const hdt::ObjectDetectionFeedback::ConstPtr& feedback);
};

} // namespace hdt

#endif
