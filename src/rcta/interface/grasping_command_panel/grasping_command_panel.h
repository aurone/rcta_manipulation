#ifndef rcta_grasping_command_panel_h
#define rcta_grasping_command_panel_h

// standard includes
#include <atomic>
#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// system includes
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <rcta_msgs/GraspObjectCommandAction.h>
#include <rcta_msgs/RepositionBaseCommandAction.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

// project includes
#include <rcta/TeleportAndaliteCommandAction.h>
#include <rcta/TeleportHDTCommandAction.h>

namespace rcta {

class GraspingCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    GraspingCommandPanel(QWidget *parent = 0);
    ~GraspingCommandPanel();

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    void refresh_robot_description();
    void refresh_global_frame();

    void copyCurrentBasePose();
    void update_base_pose_x(double x);
    void update_base_pose_y(double y);
    void update_base_pose_z(double z);
    void update_base_pose_yaw(double yaw);
    void update_base_pose_candidate(int index);
    void send_teleport_andalite_command();

    void send_grasp_object_command();
    void send_reposition_base_command();

private:

    /// @{ ROS Comms

    ros::NodeHandle nh_;

    ros::Publisher robot_markers_pub_;

    ros::Subscriber occupancy_grid_sub_;

    tf::TransformListener listener_;

    typedef actionlib::SimpleActionClient<rcta_msgs::GraspObjectCommandAction> GraspObjectCommandActionClient;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool pending_grasp_object_command_;

    typedef actionlib::SimpleActionClient<rcta_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionClient;
    std::unique_ptr<RepositionBaseCommandActionClient> reposition_base_command_client_;
    bool pending_reposition_base_command_;

    typedef actionlib::SimpleActionClient<rcta::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionClient;
    std::unique_ptr<TeleportAndaliteCommandActionClient> teleport_andalite_command_client_;
    bool pending_teleport_andalite_command_;

    interactive_markers::InteractiveMarkerServer server_;

    /// @}

    ///@{ GUI Interface

    // Global Settings Widgets
    QLineEdit* robot_description_line_edit_;
    QPushButton* refresh_robot_desc_button_;
    QLineEdit* global_frame_line_edit_;
    QPushButton* refresh_global_frame_button_;

    // Base Command Widgets
    QPushButton* copy_current_base_pose_button_;
    QDoubleSpinBox* teleport_base_command_x_box_;
    QDoubleSpinBox* teleport_base_command_y_box_;
    QDoubleSpinBox* teleport_base_command_z_box_;
    QDoubleSpinBox* teleport_base_command_yaw_box_;
    QPushButton* send_teleport_andalite_command_button_;

    // Object Interaction Command Widgets
    QPushButton* send_grasp_object_command_button_;
    QPushButton* send_reposition_base_command_button_;
    QSpinBox* update_candidate_spinbox_;
    QLabel* num_candidates_label_;

    /// @}

    // TODO: can this be maintained via the transform of the root joint or is
    // there a reason this is being maintained externally?
    Eigen::Affine3d T_world_robot_;

    Eigen::Affine3d T_world_object_;

    robot_model_loader::RobotModelLoaderPtr rml_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    nav_msgs::OccupancyGrid::ConstPtr occupancy_grid_;

    std::string robot_description_;
    std::string global_frame_;

    int base_candidate_idx_;
    std::vector<geometry_msgs::PoseStamped> candidate_base_poses_;

    void setup_gui();

    // The set utilities have the effect of changing the text in the line edit
    // box as well as applying the same action as the corresponding refresh
    // button; the get utilities retrieve the last 'refreshed' value
    bool set_robot_description(const std::string& robot_description, std::string& why);
    bool set_global_frame(const std::string& global_frame, std::string& why);

    bool valid_global_frame(const std::string& frame) const;

    bool reinit(const std::string& robot_description, std::string& why);
    bool reinit_robot_models(const std::string& robot_description, std::string& why);
    bool reinit_object_interactive_marker();

    bool initialized() const;

    void processGascanMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);

    void publish_phantom_robot_visualizations();
    void publish_base_pose_candidate_visualization(
        const geometry_msgs::PoseStamped& candidate_pose);

    std::vector<visualization_msgs::InteractiveMarkerControl> create_sixdof_controls() const;

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg);
    void occupancy_grid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void grasp_object_command_active_cb();
    void grasp_object_command_feeback_cb(const rcta_msgs::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const rcta_msgs::GraspObjectCommandResult::ConstPtr& result);

    void reposition_base_command_active_cb();
    void reposition_base_command_feedback_cb(const rcta_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback);
    void reposition_base_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const rcta_msgs::RepositionBaseCommandResult::ConstPtr& result);

    void teleport_andalite_command_active_cb();
    void teleport_andalite_command_feedback_cb(const rcta::TeleportAndaliteCommandFeedback::ConstPtr& feedback);
    void teleport_andalite_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const rcta::TeleportAndaliteCommandResult::ConstPtr& result);

    void update_object_marker_pose();
    void update_base_pose_spinboxes();
    void update_gui();
};

} // namespace rcta

#endif
