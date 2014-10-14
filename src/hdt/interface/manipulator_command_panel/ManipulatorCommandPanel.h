#ifndef hdt_ManipulatorCommandPanel_h
#define hdt_ManipulatorCommandPanel_h

#include <atomic>
#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <hdt_description/RobotModel.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <hdt/MoveArmCommandAction.h>
#include <hdt/ViservoCommandAction.h>
#include <hdt_msgs/GraspObjectCommandAction.h>
#include <hdt_msgs/RepositionBaseCommandAction.h>
#include <hdt/TeleportAndaliteCommandAction.h>
#include <hdt/TeleportHDTCommandAction.h>

namespace hdt
{

class ManipulatorCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    ManipulatorCommandPanel(QWidget *parent = 0);
    ~ManipulatorCommandPanel();

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    void refresh_robot_description();
    void refresh_global_frame();
    void check_use_global_frame(int);

    void copy_current_base_pose();
    void update_base_pose_x(double x);
    void update_base_pose_y(double y);
    void update_base_pose_z(double z);
    void update_base_pose_yaw(double yaw);
    void update_base_pose_candidate(int index);
    void send_teleport_andalite_command();
    void send_teleport_hdt_command();
    void send_open_gripper_command();
    void send_close_gripper_command();

    void check_send_octomap(int);
    void subscribe_to_octomap();
    void copy_current_state();
    void cycle_ik_solutions();
    void send_move_arm_command();
    void send_joint_goal();
    void update_j1_position(double value);
    void update_j2_position(double value);
    void update_j3_position(double value);
    void update_j4_position(double value);
    void update_j5_position(double value);
    void update_j6_position(double value);
    void update_j7_position(double value);
    void send_viservo_command();

    void send_grasp_object_command();
    void send_reposition_base_command();

private:

    ros::NodeHandle nh_;

    /// @{ Action Clients

    typedef actionlib::SimpleActionClient<hdt::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_command_client_;
    bool pending_move_arm_command_;

    typedef actionlib::SimpleActionClient<hdt::ViservoCommandAction> ViservoCommandActionClient;
    std::unique_ptr<ViservoCommandActionClient> viservo_command_client_;
    bool pending_viservo_command_;

    typedef actionlib::SimpleActionClient<hdt_msgs::GraspObjectCommandAction> GraspObjectCommandActionClient;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool pending_grasp_object_command_;

    typedef actionlib::SimpleActionClient<hdt_msgs::RepositionBaseCommandAction> RepositionBaseCommandActionClient;
    std::unique_ptr<RepositionBaseCommandActionClient> reposition_base_command_client_;
    bool pending_reposition_base_command_;

    typedef actionlib::SimpleActionClient<hdt::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionClient;
    std::unique_ptr<TeleportAndaliteCommandActionClient> teleport_andalite_command_client_;
    bool pending_teleport_andalite_command_;

    typedef actionlib::SimpleActionClient<hdt::TeleportHDTCommandAction> TeleportHDTCommandActionClient;
    std::unique_ptr<TeleportHDTCommandActionClient> teleport_hdt_command_client_;
    bool pending_teleport_hdt_command_;

    typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperCommandActionClient;
    std::unique_ptr<GripperCommandActionClient> gripper_command_client_;
    bool pending_gripper_command_;

    /// @}

    ///@{ GUI Interface

    // Global Settings Widgets
    QLineEdit* robot_description_line_edit_;
    QPushButton* refresh_robot_desc_button_;
    QLineEdit* global_frame_line_edit_;
    QPushButton* refresh_global_frame_button_;
    QCheckBox* global_frame_checkbox_;

    // Base Command Widgets
    QPushButton* copy_current_base_pose_button_;
    QDoubleSpinBox* teleport_base_command_x_box_;
    QDoubleSpinBox* teleport_base_command_y_box_;
    QDoubleSpinBox* teleport_base_command_z_box_;
    QDoubleSpinBox* teleport_base_command_yaw_box_;
    QPushButton* send_teleport_andalite_command_button_;

    // Arm Command Widgets
    QCheckBox* octomap_checkbox_;
    QLineEdit* octomap_topic_line_edit_;
    QPushButton* copy_current_state_button_;
    QPushButton* cycle_ik_solutions_button_;
    QPushButton* send_move_arm_command_button_;
    QPushButton* send_joint_goal_button_;
    QPushButton* send_teleport_hdt_command_button_;
    QDoubleSpinBox* j1_spinbox_;
    QDoubleSpinBox* j2_spinbox_;
    QDoubleSpinBox* j3_spinbox_;
    QDoubleSpinBox* j4_spinbox_;
    QDoubleSpinBox* j5_spinbox_;
    QDoubleSpinBox* j6_spinbox_;
    QDoubleSpinBox* j7_spinbox_;
    QPushButton* send_viservo_command_button_;

    // Gripper Command Widgets
    QPushButton* send_open_gripper_command_button_;
    QPushButton* send_close_gripper_command_button_;

    // Object Interaction Command Widgets
    QPushButton* send_grasp_object_command_button_;
    QPushButton* send_reposition_base_command_button_;
    QSpinBox* update_candidate_spinbox_;
    QLabel* num_candidates_label_;

    /// @}

    Eigen::Affine3d world_to_robot_;
    Eigen::Affine3d world_to_object_;

    hdt::RobotModelPtr robot_model_;

    robot_model_loader::RobotModelLoaderPtr rm_loader_;
    robot_model::RobotModelPtr rm_;
    robot_state::RobotStatePtr rs_;

    interactive_markers::InteractiveMarkerServer server_;

    ros::Subscriber joint_states_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher robot_markers_pub_;

    sensor_msgs::JointState last_joint_state_;
    octomap_msgs::Octomap::ConstPtr last_octomap_msg_;

    std::string tip_link_;
    std::string base_link_;

    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    tf::TransformListener listener_;

    std::string robot_description_;
    bool use_global_frame_;
    std::string global_frame_;

    std::size_t base_candidate_idx_;
    std::vector<geometry_msgs::PoseStamped> candidate_base_poses_;

    void setup_gui();

    // The set utilities have the effect of changing the text in the line edit
    // box as well as applying the same action as the corresponding refresh
    // button; the get utilities retrieve the last 'refreshed' value
    bool set_robot_description(const std::string& robot_description, std::string& why);
    bool set_global_frame(const std::string& global_frame, std::string& why);
    const std::string& get_robot_description() const;
    const std::string& get_global_frame() const;

    bool use_global_frame() const;
    void set_use_global_frame(bool use);

    bool global_frame_active() const;
    std::string interactive_marker_frame() const;

    bool valid_global_frame(const std::string& frame) const;

    /// @{
    ///
    ///    Marker frame transform accessors. The marker frame is the frame that
    ///    the interactive markers are expressed in: either the local frame of
    ///    the robot or a specific global frame (an ancestor of the robot local frame)
    ///
    /// @ brief Return the transform of the robot in the marker frame
    const Eigen::Affine3d robot_transform() const;
    /// @brief Return the transform of the object in the marker frame
    const Eigen::Affine3d object_transform() const;
    ///@}

    bool reinit(const std::string& robot_description, std::string& why);
    bool reinit_robot_models(const std::string& robot_description, std::string& why);
    bool reinit_interactive_marker_server();
    bool reinit_manipulator_interactive_marker();
    bool reinit_object_interactive_marker(); // TODO: the object interactive marker can technically be around while the robot marker is not
    bool check_robot_model_consistency(const hdt::RobotModel& hdt_model, const robot_model::RobotModel& moveit_model) const;

    /// @brief Return the tip link of a joint model group if the joint model group has only one root
    std::string get_tip_link(const robot_model::JointModelGroup& joint_model_group) const;
    /// @brief Return the base link of a joint model group if the joint model group has only one root
    std::string get_base_link(const robot_model::JointModelGroup& joint_model_group) const;

    bool initialized() const;

    void do_process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void process_gas_canister_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);

    void publish_phantom_robot_visualizations();

    std::vector<visualization_msgs::InteractiveMarkerControl> create_sixdof_controls() const;

    /// @brief Get the value of a particular joint in a joint state.
    /// @return true if the joint was found; false otherwise
    bool get_joint_value(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_value) const;

    void update_joint_position(int joint_index, double joint_position);

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg);

    void move_arm_command_active_cb();
    void move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback);
    void move_arm_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::MoveArmCommandResult::ConstPtr& result);

    void viservo_command_active_cb();
    void viservo_command_feedback_cb(const hdt::ViservoCommandFeedback::ConstPtr& feedback);
    void viservo_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::ViservoCommandResult::ConstPtr& result);

    void grasp_object_command_active_cb();
    void grasp_object_command_feeback_cb(const hdt_msgs::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::GraspObjectCommandResult::ConstPtr& result);

    void reposition_base_command_active_cb();
    void reposition_base_command_feedback_cb(const hdt_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback);
    void reposition_base_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt_msgs::RepositionBaseCommandResult::ConstPtr& result);

    void teleport_andalite_command_active_cb();
    void teleport_andalite_command_feedback_cb(const hdt::TeleportAndaliteCommandFeedback::ConstPtr& feedback);
    void teleport_andalite_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::TeleportAndaliteCommandResult::ConstPtr& result);

    void teleport_hdt_command_active_cb();
    void teleport_hdt_command_feedback_cb(const hdt::TeleportHDTCommandFeedback::ConstPtr& feedback);
    void teleport_hdt_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::TeleportHDTCommandResult::ConstPtr& result);

    void gripper_command_active_cb();
    void gripper_command_feedback_cb(const control_msgs::GripperCommandFeedback::ConstPtr& feedback);
    void gripper_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const control_msgs::GripperCommandResult::ConstPtr& result);

    bool gatherRobotMarkers(
            const robot_state::RobotState& robot_state,
            const std::vector<std::string>& link_names,
            const std_msgs::ColorRGBA& color,
            const std::string& ns,
            const ros::Duration& d,
            visualization_msgs::MarkerArray& markers,
            bool include_attached = false);

    std::vector<double> get_current_joint_angles() const;
    std::vector<double> get_phantom_joint_angles() const;

    bool set_phantom_joint_angles(const std::vector<double>& joint_angles);

    void update_manipulator_marker_pose();
    void update_object_marker_pose();
    void update_base_pose_spinboxes();
    void update_spinboxes();
    void update_gui();

    template <typename ActionType>
    bool reconnect_client(
        std::unique_ptr<actionlib::SimpleActionClient<ActionType>>& client,
        const std::string& action_name,
        ros::Rate poll_frequency = ros::Rate(10.0),
        const ros::Duration& timeout = ros::Duration(1.0))
    {
        if (!client) {
            client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
        }

        ROS_DEBUG("Waiting for action server '%s'", action_name.c_str());

        if (!client) {
            ROS_WARN("Action client is null");
            return false;
        }

        ros::Time start = ros::Time::now();
        while (ros::ok() && (timeout == ros::Duration(0) || ros::Time::now() < start + timeout)) {
            ros::spinOnce();
            if (!client->isServerConnected()) {
                client.reset(new actionlib::SimpleActionClient<ActionType>(action_name, false));
                if (!client) {
                    ROS_WARN("Failed to reinstantiate action client '%s'", action_name.c_str());
                    return false;
                }
            }

            if (client->isServerConnected()) {
                return true;
            }

            poll_frequency.sleep();

            ROS_DEBUG("Waited %0.3f seconds for action server '%s'...", (ros::Time::now() - start).toSec(), action_name.c_str());
        }

        return false;
    }
};

} // namespace hdt

#endif
