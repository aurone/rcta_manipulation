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
#include <rospack/rospack.h>
#include <actionlib/client/simple_action_client.h>
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
#include <hdt/GraspObjectCommandAction.h>
#include <hdt/RepositionBaseCommandAction.h>
#include <hdt/TeleportAndaliteCommandAction.h>

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

    void copy_current_state();
    void refresh_robot_description();

    void send_move_arm_command();
    void send_joint_goal();
    void send_viservo_command();
    void send_grasp_object_command();
    void send_reposition_base_command();
    void send_teleport_andalite_command();

    void cycle_ik_solutions();

    void change_joint_1(int value);
    void change_joint_2(int value);
    void change_joint_3(int value);
    void change_joint_4(int value);
    void change_joint_5(int value);
    void change_joint_6(int value);
    void change_joint_7(int value);

private:

    bool initialized_;

    ros::NodeHandle nh_;

    typedef actionlib::SimpleActionClient<hdt::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_client_;
    bool pending_move_arm_command_;

    typedef actionlib::SimpleActionClient<hdt::ViservoCommandAction> ViservoCommandActionClient;
    std::unique_ptr<ViservoCommandActionClient> viservo_command_client_;
    bool pending_viservo_command_;

    typedef actionlib::SimpleActionClient<hdt::GraspObjectCommandAction> GraspObjectCommandActionClient;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool pending_grasp_object_command_;

    typedef actionlib::SimpleActionClient<hdt::RepositionBaseCommandAction> RepositionBaseCommandActionClient;
    std::unique_ptr<RepositionBaseCommandActionClient> reposition_base_command_client_;
    bool pending_reposition_base_command_;

    typedef actionlib::SimpleActionClient<hdt::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionClient;
    std::unique_ptr<TeleportAndaliteCommandActionClient> teleport_andalite_command_client_;
    bool pending_teleport_andalite_command_;

    hdt::RobotModelPtr robot_model_;

    std::unique_ptr<robot_model_loader::RobotModelLoader> rm_loader_;
    robot_model::RobotModelPtr rm_;
    robot_state::RobotStatePtr rs_;

    interactive_markers::InteractiveMarkerServer server_;

    ros::Subscriber joint_states_sub_;
    ros::Publisher robot_markers_pub_;

    sensor_msgs::JointState last_joint_state_;

    std::vector<visualization_msgs::InteractiveMarker> interactive_markers_;
    std::string tip_link_;
    std::string base_link_;

    QLabel* robot_description_label_;
    QLineEdit* robot_description_line_edit_;
    QPushButton* refresh_robot_desc_button_;

    QLabel* global_frame_label_;
    QLineEdit* global_frame_line_edit_;

    QPushButton* copy_current_state_button_;
    QPushButton* copy_current_base_pose_;
    QPushButton* send_move_arm_command_button_;
    QPushButton* send_joint_goal_button_;
    QPushButton* cycle_ik_solutions_button_;
    QPushButton* send_viservo_command_button_;
    QPushButton* send_grasp_object_command_button_;
    QPushButton* send_reposition_base_command_button_;
    QSlider* joint_1_slider_;
    QSlider* joint_2_slider_;
    QSlider* joint_3_slider_;
    QSlider* joint_4_slider_;
    QSlider* joint_5_slider_;
    QSlider* joint_6_slider_;
    QSlider* joint_7_slider_;
    QDoubleSpinBox* j1_spinbox_;
    QDoubleSpinBox* j2_spinbox_;
    QDoubleSpinBox* j3_spinbox_;
    QDoubleSpinBox* j4_spinbox_;
    QDoubleSpinBox* j5_spinbox_;
    QDoubleSpinBox* j6_spinbox_;
    QDoubleSpinBox* j7_spinbox_;

    QDoubleSpinBox* teleport_base_command_x_box_;
    QDoubleSpinBox* teleport_base_command_y_box_;
    QDoubleSpinBox* teleport_base_command_z_box_;
    QPushButton* send_teleport_andalite_command_button_;

    Eigen::Affine3d root_to_first_link_;
    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    tf::TransformListener listener_;

    void setup_gui();

    bool do_init();
    bool check_robot_model_consistency();

    void do_process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void process_gas_canister_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);

    void publish_transform_network();
    void publish_joint_states();
    void publish_phantom_robot_visualizations();

    /// @brief Return the tip link of a joint model group if the joint model group has only one root
    std::string get_tip_link(const robot_model::JointModelGroup& joint_model_group) const;

    /// @brief Return the base link of a joint model group if the joint model group has only one root
    std::string get_base_link(const robot_model::JointModelGroup& joint_model_group) const;

    std::vector<visualization_msgs::InteractiveMarkerControl> create_sixdof_controls() const;

    /// @brief Get the value of a particular joint in a joint state.
    /// @return true if the joint was found; false otherwise
    bool get_joint_value(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_value) const;

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool reinit_robot();

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
    void grasp_object_command_feeback_cb(const hdt::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::GraspObjectCommandResult::ConstPtr& result);

    void reposition_base_command_active_cb();
    void reposition_base_command_feedback_cb(const hdt::RepositionBaseCommandFeedback::ConstPtr& feedback);
    void reposition_base_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::RepositionBaseCommandResult::ConstPtr& result);

    void teleport_andalite_command_active_cb();
    void teleport_andalite_command_feedback_cb(const hdt::TeleportAndaliteCommandFeedback::ConstPtr& feedback);
    void teleport_andalite_command_result_cb(
            const actionlib::SimpleClientGoalState& state,
            const hdt::TeleportAndaliteCommandResult::ConstPtr& result);

    bool gatherRobotMarkers(
            const robot_state::RobotState& robot_state,
            const std::vector<std::string>& link_names,
            const std_msgs::ColorRGBA& color,
            const std::string& ns,
            const ros::Duration& d,
            visualization_msgs::MarkerArray& markers,
            bool include_attached = false);

    static void joint_state_to_joint_array(const sensor_msgs::JointState&);

    std::vector<double> get_current_joint_angles() const;
    std::vector<double> get_phantom_joint_angles() const;

    bool set_phantom_joint_angles(const std::vector<double>& joint_angles);

    void update_sliders();
    void update_gui();
};

} // namespace hdt

#endif
