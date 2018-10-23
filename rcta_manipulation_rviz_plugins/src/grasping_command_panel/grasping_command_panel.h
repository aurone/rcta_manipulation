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
#ifndef Q_MOC_RUN
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <cmu_manipulation_msgs/RepositionBaseCommandAction.h>
#include <cmu_manipulation_msgs/ManipulateAction.h>
#include <cmu_manipulation_msgs/GraspObjectCommandAction.h>
#include <cmu_manipulation_msgs/ManipulateObjectAction.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#endif

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

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

    void refreshRobotDescription();
    void refreshGlobalFrame();
    void refreshObjectMeshResource();

    void copyCurrentBasePose();
    void updateBasePoseX(double x);
    void updateBasePoseY(double y);
    void updateBasePoseZ(double z);
    void updateBasePoseYaw(double yaw);
    void updateBasePoseCandidate(int index);

    void updateMeshScaleX();
    void updateMeshScaleY();
    void updateMeshScaleZ();

    void updateObjectStart();
    void updateObjectGoal();

    void sendGraspObjectCommand();
    void sendRepositionBaseCommand();
    void sendManipulateObjectCommand();

private:

    /// @{ ROS Comms

    ros::NodeHandle nh_;

    ros::Publisher robot_markers_pub_;

    ros::Subscriber occupancy_grid_sub_;

    tf::TransformListener listener_;

    using ManipulateActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::ManipulateAction>;
    std::unique_ptr<ManipulateActionClient> manipulate_client_;
    bool pending_manipulate_command_ = false;

    using GraspObjectCommandActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::GraspObjectCommandAction>;
    std::unique_ptr<GraspObjectCommandActionClient> grasp_object_command_client_;
    bool pending_grasp_object_command_ = false;

    using RepositionBaseCommandActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::RepositionBaseCommandAction>;
    std::unique_ptr<RepositionBaseCommandActionClient> reposition_base_command_client_;
    bool pending_reposition_base_command_ = false;

    using ManipulateObjectActionClient = actionlib::SimpleActionClient<cmu_manipulation_msgs::ManipulateObjectAction>;
    std::unique_ptr<ManipulateObjectActionClient> manipulate_object_client_;
    bool pending_manipulate_object_command_ = false;

    interactive_markers::InteractiveMarkerServer server_;

    /// @}

    ///@{ GUI Interface

    // Global Settings Widgets
    QLineEdit* robot_description_line_edit_ = NULL;
    QPushButton* refresh_robot_desc_button_ = NULL;
    QLineEdit* global_frame_line_edit_ = NULL;
    QPushButton* refresh_global_frame_button_ = NULL;
    QLineEdit* m_obj_mesh_resource_line_edit = NULL;
    QPushButton* m_refresh_obj_mesh_resource_button = NULL;
    QLineEdit* m_obj_mesh_scale_x_line_edit = NULL;
    QLineEdit* m_obj_mesh_scale_y_line_edit = NULL;
    QLineEdit* m_obj_mesh_scale_z_line_edit = NULL;

    // Base Command Widgets
    QPushButton* copy_current_base_pose_button_ = NULL;
    QDoubleSpinBox* teleport_base_command_x_box_ = NULL;
    QDoubleSpinBox* teleport_base_command_y_box_ = NULL;
    QDoubleSpinBox* teleport_base_command_z_box_ = NULL;
    QDoubleSpinBox* teleport_base_command_yaw_box_ = NULL;

    // Object Interaction Command Widgets
    QPushButton* send_grasp_object_command_button_ = NULL;
    QPushButton* send_reposition_base_command_button_ = NULL;
    QPushButton* send_manipulate_object_command_button_ = NULL;
    QLineEdit* m_object_start_line_edit = NULL;
    QLineEdit* m_object_goal_line_edit = NULL;
    QSpinBox* update_candidate_spinbox_ = NULL;
    QLabel* num_candidates_label_ = NULL;

    QDoubleSpinBox* m_obj_pose_x_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_y_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_z_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_Y_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_P_spinbox = NULL;
    QDoubleSpinBox* m_obj_pose_R_spinbox = NULL;

    /// @}

    // TODO: can this be maintained via the transform of the root joint or is
    // there a reason this is being maintained externally?
    Eigen::Affine3d T_world_robot_ = Eigen::Affine3d::Identity();
    Eigen::Affine3d T_world_object_ = Eigen::Affine3d::Identity();

    robot_model_loader::RobotModelLoaderPtr rml_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    nav_msgs::OccupancyGrid::ConstPtr occupancy_grid_;

    std::string robot_description_;
    std::string global_frame_;

    int base_candidate_idx_ = -1;
    std::vector<geometry_msgs::PoseStamped> candidate_base_poses_;

    std::string m_gascan_interactive_marker_name = "gas_canister_fixture";
    std::string m_obj_mesh_resource = "package://gascan_description/meshes/rcta_gastank.ply";

    double m_obj_scale_x = 1.0;
    double m_obj_scale_y = 1.0;
    double m_obj_scale_z = 1.0;

    double m_obj_start = 0.0;
    double m_obj_goal = 0.0;

    void setupGUI();

    // The set utilities have the effect of changing the text in the line edit
    // box as well as applying the same action as the corresponding refresh
    // button; the get utilities retrieve the last 'refreshed' value
    bool setRobotDescription(const std::string& robot_description, std::string& why);
    bool setGlobalFrame(const std::string& global_frame, std::string& why);

    bool isValidGlobalFrame(const std::string& frame) const;

    bool reinit(const std::string& robot_description, std::string& why);
    bool reinitRobotModels(const std::string& robot_description, std::string& why);
    bool reinitObjectInteractiveMarker();

    bool robotModelLoaded() const;

    void processGascanMarkerFeedback(
        const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);

    void publishPhantomRobotVisualization();
    void publishBasePoseCandidateVisualization(
        const geometry_msgs::PoseStamped& candidate_pose);

    auto createSixDOFControls() const -> std::vector<visualization_msgs::InteractiveMarkerControl>;

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void manipulate_active_cb();
    void manipulate_feedback_cb(const cmu_manipulation_msgs::ManipulateFeedback::ConstPtr& feedback);
    void manipulate_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const cmu_manipulation_msgs::ManipulateResult::ConstPtr& result);

    void grasp_object_command_active_cb();
    void grasp_object_command_feeback_cb(const cmu_manipulation_msgs::GraspObjectCommandFeedback::ConstPtr& feedback);
    void grasp_object_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const cmu_manipulation_msgs::GraspObjectCommandResult::ConstPtr& result);

    void reposition_base_command_active_cb();
    void reposition_base_command_feedback_cb(const cmu_manipulation_msgs::RepositionBaseCommandFeedback::ConstPtr& feedback);
    void reposition_base_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const cmu_manipulation_msgs::RepositionBaseCommandResult::ConstPtr& result);

    void manipulateObjectResultCallback(
            const actionlib::SimpleClientGoalState& state,
            const cmu_manipulation_msgs::ManipulateObjectResult::ConstPtr& result);

    void updateObjectMarkerPose();
    void updateBasePoseSpinBoxes();
    void updateGUI();
};

} // namespace rcta

#endif
