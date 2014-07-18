#include <unordered_set>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include "PickAndPlacePanel.h"
#include "SimpleInteractiveMarker.h"
#include "GraspMarkerSelectionMarker.h"

namespace hdt
{

PickAndPlacePanel::PickAndPlacePanel(QWidget* parent) :
    rviz::Panel(parent),
    nh_(),
    open_database_button_(nullptr),
    open_features_button_(nullptr),
    open_kdtree_indices_button_(nullptr),
    database_fname_label_(nullptr),
    features_fname_label_(nullptr),
    kdtree_indices_fname_label_(nullptr),
    camera_frame_selection_(nullptr),
    root_frame_selection_(nullptr),
    snap_point_cloud_button_(nullptr),
    update_grasps_button_(nullptr),
    send_move_to_pregrasp_button_(nullptr),
    send_move_to_flipped_pregrasp_button_(nullptr),
    send_move_to_grasp_button_(nullptr),
    send_move_to_flipped_grasp_button_(nullptr),
    send_open_gripper_command_button_(nullptr),
    send_close_gripper_command_button_(nullptr),
    listener_(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), false),
    tf_sub_(),
    frame_timeout_(10.0),
    seen_frames_(),
    pending_detection_request_(false),
    last_detection_request_(),
    last_detection_result_(),
    snapshot_cloud_pub_(),
    grasp_markers_server_("grasp_markers"),
    selection_(),
    move_arm_command_client_(),
    pending_move_arm_command_(false),
    object_detection_client_(),
    gripper_command_client_()
{
    setup_gui();

    snapshot_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("snapshot_cloud", 1, true);

    tf_sub_ = nh_.subscribe("tf", 10, &PickAndPlacePanel::tf_callback, this);

    selection_.reset(new GraspMarkerSelection);
    if (!selection_) {
        ROS_ERROR("Failed to instantiate Grasp Marker Selection");
    }

    move_arm_command_client_.reset(new MoveArmCommandActionClient("move_arm_command"));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Action Client");
    }

    object_detection_client_.reset(new ObjectDetectionActionClient("object_detection_action", false));
    if (!object_detection_client_) {
        ROS_ERROR("Failed to instantiate Object Detection Action Client");
    }

    gripper_command_client_.reset(new GripperCommandActionClient("gripper_controller/gripper_command_action", false));
    if (!gripper_command_client_) {
        ROS_ERROR("Failed to instantiate Gripper Command Action Client");
    }

    grasp_markers_server_.set_feedback_callback([this]() { return this->update_gui(); });
    update_gui();
}

PickAndPlacePanel::~PickAndPlacePanel()
{
}

void PickAndPlacePanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString database_fname;
    QString features_fname;
    QString kdtree_indices_fname;
    if (config.mapGetString("DatabaseFilename", &database_fname)) {
        database_fname_label_->setText(database_fname);
    }
    if (config.mapGetString("FeaturesFilename", &features_fname)) {
        features_fname_label_->setText(features_fname);
    }
    if (config.mapGetString("KDTreeIndicesFilename", &kdtree_indices_fname)) {
        kdtree_indices_fname_label_->setText(kdtree_indices_fname);
    }

    update_gui();
}

void PickAndPlacePanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("DatabaseFilename", database_fname_label_->text());
    config.mapSetValue("FeaturesFilename", features_fname_label_->text());
    config.mapSetValue("KDTreeIndicesFilename", kdtree_indices_fname_label_->text());
}

void PickAndPlacePanel::choose_database()
{
    QString database_fname = QFileDialog::getOpenFileName(this, tr("Choose Object Database"), QString(), tr("Object Databases (*.db)"));

    if (!database_fname.isEmpty()) {
        QFileInfo database_file_info(database_fname);
        database_fname_label_->setText(database_file_info.fileName());
    }
    update_gui();
}

void PickAndPlacePanel::choose_features()
{
    QString features_fname = QFileDialog::getOpenFileName(this, tr("Choose Training Features"), QString(), tr("Training Features (*.h5)"));
    if (!features_fname.isEmpty()) {
        QFileInfo features_file_info(features_fname);
        features_fname_label_->setText(features_file_info.fileName());
    }
    update_gui();
}

void PickAndPlacePanel::choose_kdtree_indices()
{
    QString kdtree_indices_fname = QFileDialog::getOpenFileName(this, tr("Choose KD-Tree Indices"), QString(), tr("KD-Tree Indices (*.idx)"));
    if (!kdtree_indices_fname.isEmpty()) {
        QFileInfo indices_file_info(kdtree_indices_fname);
        kdtree_indices_fname_label_->setText(indices_file_info.fileName());
    }
    update_gui();
}

void PickAndPlacePanel::take_snapshot()
{
    if (!object_detection_client_->isServerConnected()) {
        QMessageBox::warning(this, tr("Action Client Failure"), tr("Unable to  send Object Detection (server is not connected")));
        return;
    }

    // object detection parameters
    std::string database_fname = database_fname_label_->text().toStdString();
    std::string features_fname = features_fname_label_->text().toStdString();
    std::string kdtree_indices_fname = kdtree_indices_fname_label_->text().toStdString();

    // frames used for cleaning up the point cloud and matching
    const std::string root_frame = root_frame_selection_->currentText().toStdString();
    const std::string camera_frame = camera_frame_selection_->currentText().toStdString();

    static int object_detection_seqno = 0;
    last_detection_request_.header.seq = ++object_detection_seqno;
    last_detection_request_.header.stamp = ros::Time::now();
    last_detection_request_.header.frame_id = "";
    last_detection_request_.object_database = database_fname;
    last_detection_request_.training_features = features_fname;
    last_detection_request_.training_kdtree = kdtree_indices_fname;
    last_detection_request_.camera_frame = camera_frame;
    last_detection_request_.root_frame = root_frame;

    // TODO: configure cull parameters through the GUI
    last_detection_request_.cull_snapshot = true;
    last_detection_request_.cull_from.x = 0.3;
    last_detection_request_.cull_from.y = -0.6;
    last_detection_request_.cull_from.z = -1.6;
    last_detection_request_.cull_to.x = 2.5;
    last_detection_request_.cull_to.y = 0.6;
    last_detection_request_.cull_to.z = 1.6;

    last_detection_request_.request_snapshot = true;

    ROS_INFO("Sent goal to action server object_detection_action");

    object_detection_client_->sendGoal(
            last_detection_request_, boost::bind(&PickAndPlacePanel::object_detection_result_cb, this, _1, _2));
    pending_detection_request_ = true;

    update_gui();
}

void PickAndPlacePanel::update_grasps()
{
    // clear the selected pregrasp/grasp
    selection_->clear();

    if (last_detection_result_->success) {
        grasp_markers_server_.clear();

        ros::Time now = ros::Time::now();

        // add pregrasp markers
        int pregrasp_num = 1;
        for (const geometry_msgs::Pose pregrasp : last_detection_result_->pregrasps.poses) {
            insert_grasp_marker(std::string("pregrasp_") + std::to_string(pregrasp_num++), pregrasp, now);
        }

        // add grasp markers
        int grasp_num = 1;
        for (const geometry_msgs::Pose grasp : last_detection_result_->grasps.poses) {
            insert_grasp_marker(std::string("grasp_") + std::to_string(grasp_num++), grasp, now);
        }

        grasp_markers_server_.flush();
    }

    update_gui();
}

void PickAndPlacePanel::send_move_to_pregrasp_command()
{
    move_arm_to_marker_pose(selection_->get_pregrasp(), false);
}

void PickAndPlacePanel::send_move_to_flipped_pregrasp_command()
{
    move_arm_to_marker_pose(selection_->get_pregrasp(), true);
}

void PickAndPlacePanel::send_move_to_grasp_command()
{
    move_arm_to_marker_pose(selection_->get_grasp(), false);
}

void PickAndPlacePanel::send_move_to_flipped_grasp_command()
{
    move_arm_to_marker_pose(selection_->get_grasp(), true);
}

void PickAndPlacePanel::move_arm_to_marker_pose(const std::string& marker_name, bool flipped)
{
    if (marker_name.empty()) {
        ROS_WARN("Attempt to move to pregrasp without a pregrasp marker selected");
        return;
    }

    SimpleInteractiveMarkerPtr selected_grasp_marker = grasp_markers_server_.get(marker_name);
    if (!selected_grasp_marker) {
        QMessageBox::warning(this, tr("Move Arm To Marker Pose"), tr("No interactive marker named '%1'").arg(QString::fromStdString(marker_name)));
        return;
    }

    geometry_msgs::PoseStamped pregrasp_marker_pose;
    pregrasp_marker_pose.header = selected_grasp_marker->interactive_marker().header;
    pregrasp_marker_pose.header.stamp = ros::Time(0);
    pregrasp_marker_pose.pose = selected_grasp_marker->interactive_marker().pose;

    geometry_msgs::PoseStamped goal_wrist_pose_mount_frame;
    if (!wrist_pose_from_pregrasp_pose(pregrasp_marker_pose, goal_wrist_pose_mount_frame))
    {
        ROS_WARN("Failed to obtain goal wrist pose from pregrasp marker pose");
        return;
    }

    if (flipped) {
        Eigen::Affine3d goal_wrist_transform_mount_frame;
        tf::poseMsgToEigen(goal_wrist_pose_mount_frame.pose, goal_wrist_transform_mount_frame);

        Eigen::Affine3d goal_wrist_transform_flipped =
                goal_wrist_transform_mount_frame * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1.0, 0.0, 0.0));

        tf::poseEigenToMsg(goal_wrist_transform_flipped, goal_wrist_pose_mount_frame.pose);
    }

    hdt::MoveArmCommandGoal goal;
    goal.goal_pose = goal_wrist_pose_mount_frame.pose;
    move_arm_command_client_->sendGoal(goal, boost::bind(&PickAndPlacePanel::move_arm_command_result_cb, this, _1, _2));
    pending_move_arm_command_ = true;
    update_gui();
}

void PickAndPlacePanel::send_open_gripper_command()
{
    ROS_INFO("Opening the gripper!");
    if (gripper_command_client_->isServerConnected()) {
        GripperCommandGoal goal_msg;
        goal_msg.command.position = 0.0854;
        gripper_command_client_->sendGoal(goal_msg, boost::bind(&PickAndPlacePanel::gripper_command_result_cb, this, _1, _2));
    }
    else {
        ROS_INFO("Gripper Command Client is not yet connected");
    }
}

void PickAndPlacePanel::send_close_gripper_command()
{
    ROS_INFO("Closing the gripper!");
    if (gripper_command_client_->isServerConnected()) {
        GripperCommandGoal goal_msg;
        goal_msg.command.position = 0.0;
        gripper_command_client_->sendGoal(goal_msg, boost::bind(&PickAndPlacePanel::gripper_command_result_cb, this, _1, _2));
    }
    else {
        ROS_INFO("Gripper Command Client is not yet connected");
    }
}

void PickAndPlacePanel::camera_frame_box_current_index_changed(int index)
{
    ROS_INFO("Camera Frame Box Current Index Changed!");
    update_gui();
}

void PickAndPlacePanel::camera_frame_box_edit_text_changed(const QString& text)
{
    ROS_INFO("Camera Frame Box Edit Text Changed!");
    update_gui();
}

void PickAndPlacePanel::root_frame_box_current_index_changed(int index)
{
    ROS_INFO("Root Frame Box Current Index Changed!");
    update_gui();
}

void PickAndPlacePanel::root_frame_box_edit_text_changed(const QString& text)
{
    ROS_INFO("Root Frame Box Edit Text Changed!");
    update_gui();
}

void PickAndPlacePanel::tf_callback(const tf::tfMessage::ConstPtr& msg)
{
    bool changed = false;
    for (const geometry_msgs::TransformStamped& transform : msg->transforms) {
        // check if we have a recent frame for the parent frame
        auto it = seen_frames_.find(transform.header.frame_id);
        if (it == seen_frames_.end()) {
            ROS_INFO("Adding %s to available frames", transform.header.frame_id.c_str());
            seen_frames_.insert(std::make_pair(transform.header.frame_id, transform.header.stamp));
            changed = true;
        }
        else {
            it->second = transform.header.stamp;
        }

        // check if we have a recent frame for the child frame
        it = seen_frames_.find(transform.child_frame_id);
        if (it == seen_frames_.end()) {
            ROS_INFO("Adding %s to available frames", transform.child_frame_id.c_str());
            seen_frames_.insert(std::make_pair(transform.child_frame_id, transform.header.stamp));
            changed = true;
        }
        else {
            it->second = transform.header.stamp;
        }
    }

    // remove old frames
//    ros::Time now = ros::Time::now();
//    auto it = seen_frames_.begin();
//    while (it != seen_frames_.end()) {
//        bool erase = (it->second < now - ros::Duration(frame_timeout_));
//        if (erase) {
//            auto eit = it;
//            ++it;
//            changed = true;
//            ROS_INFO("Removing %s from available frames", eit->first.c_str());
//            seen_frames_.erase(eit);
//        }
//        else {
//            ++it;
//        }
//    }

    if (changed) {
        // update combo box options
        update_combo_box(*camera_frame_selection_, seen_frames_);
        update_combo_box(*root_frame_selection_, seen_frames_);
    }
}

void PickAndPlacePanel::update_gui()
{
    snap_point_cloud_button_->setEnabled(
            !database_fname_label_->text().isEmpty() &&
            !features_fname_label_->text().isEmpty() &&
            !kdtree_indices_fname_label_->text().isEmpty() &&
            !camera_frame_selection_->currentText().isEmpty() &&
            !root_frame_selection_->currentText().isEmpty() &&
            !pending_detection_request_);
    update_grasps_button_->setEnabled(last_detection_result_ && last_detection_result_->success);
    send_move_to_pregrasp_button_->setEnabled(selection_->pregrasp_selected() && !pending_move_arm_command_);
    send_move_to_flipped_pregrasp_button_->setEnabled(selection_->pregrasp_selected() && !pending_move_arm_command_);
    send_move_to_grasp_button_->setEnabled(selection_->grasp_selected() && !pending_move_arm_command_);
    send_move_to_flipped_grasp_button_->setEnabled(selection_->grasp_selected() && !pending_move_arm_command_);
}

int PickAndPlacePanel::find_item(const QComboBox& combo_box, const std::string& item) const
{
    for (int i = 0; i < combo_box.count(); ++i) {
        if (combo_box.itemText(i).toStdString() == item) {
            return i;
        }
    }
    return -1;
}

void PickAndPlacePanel::update_combo_box(QComboBox& combo_box, const std::map<std::string, ros::Time>& entries)
{
    std::unordered_set<std::string> searched;
    for (int i = 0; i < combo_box.count(); ) {
        // remove items from the combo box that are not part of entries
        QString item_text = combo_box.itemText(i);
        if (entries.find(item_text.toStdString()) == entries.end()) {
            combo_box.removeItem(i);
        }
        else {
            ++i;
        }
        searched.insert(item_text.toStdString());
    }

    // add new elements from entries (elements that were checked in the entry pass above but were not removed)
    for (const auto& entry : entries) {
        if (searched.find(entry.first) == searched.end()) {
            combo_box.addItem(QString::fromStdString(entry.first));
        }
    }
}

visualization_msgs::Marker PickAndPlacePanel::create_arrow_marker(const geometry_msgs::Vector3 &scale)
{
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.seq = 0;
    arrow_marker.header.stamp = ros::Time(0);
    arrow_marker.header.frame_id = "";
    arrow_marker.ns = "";
    arrow_marker.id = 0;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.position.x = 0.0;
    arrow_marker.pose.position.y = 0.0;
    arrow_marker.pose.position.z = 0.0;
    arrow_marker.pose.orientation.w = 1.0;
    arrow_marker.pose.orientation.x = 0.0;
    arrow_marker.pose.orientation.y = 0.0;
    arrow_marker.pose.orientation.z = 0.0;
    arrow_marker.scale.x = scale.x;
    arrow_marker.scale.y = scale.y;
    arrow_marker.scale.z = scale.z;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 0.7;
    arrow_marker.lifetime = ros::Duration(0);
    arrow_marker.frame_locked = false;
    return arrow_marker;
}

visualization_msgs::MarkerArray PickAndPlacePanel::create_triad_markers(const geometry_msgs::Vector3& scale)
{
    visualization_msgs::MarkerArray markers;

    // create an arrow marker for the x-axis
    visualization_msgs::Marker m = create_arrow_marker(scale);
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.7;
    markers.markers.push_back(m);

    // create an arrow marker for the y-axis
    Eigen::AngleAxisd rotate_z(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::Quaterniond q = rotate_z * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.r = 0.0;
    m.color.g = 1.0;
    markers.markers.push_back(m);

    // create an arrow marker for the z-axis
    Eigen::AngleAxisd rotate_y(-M_PI / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0));
    q = rotate_y * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.g = 0.0;
    m.color.b = 1.0;
    markers.markers.push_back(m);

    visualization_msgs::Marker origin_marker;
    origin_marker.header.seq = 0;
    origin_marker.header.stamp = ros::Time(0);
    origin_marker.header.frame_id = "";
    origin_marker.ns = "";
    origin_marker.id = 0;
    origin_marker.type = visualization_msgs::Marker::CUBE;
    origin_marker.action = visualization_msgs::Marker::ADD;
    origin_marker.pose.position.x = origin_marker.pose.position.y = origin_marker.pose.position.z = 0.0;
    origin_marker.pose.orientation.w = 1.0;
    origin_marker.pose.orientation.x = origin_marker.pose.orientation.y = origin_marker.pose.orientation.z = 0.0;
    origin_marker.scale.x = origin_marker.scale.y = origin_marker.scale.z = scale.y; // note: assume that scale.y and scale.z are the same
    origin_marker.color.r = origin_marker.color.g = origin_marker.color.b = 0.9;
    origin_marker.color.a = 1.0;
    origin_marker.lifetime = ros::Duration(0);
    origin_marker.frame_locked = false;
    markers.markers.push_back(origin_marker);

    return markers;
}

void PickAndPlacePanel::gripper_command_active_cb()
{

}

void PickAndPlacePanel::gripper_command_feedback_cb(const GripperCommandFeedback::ConstPtr& feedback)
{

}

void PickAndPlacePanel::gripper_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const GripperCommandResult::ConstPtr& result)
{
    ROS_INFO("Gripper command completed");
}

void PickAndPlacePanel::move_arm_command_active_cb()
{

}

void PickAndPlacePanel::move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback)
{

}

void PickAndPlacePanel::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::MoveArmCommandResult::ConstPtr& result)
{
    pending_move_arm_command_ = false;
    update_gui();
}

void PickAndPlacePanel::object_detection_active_cb()
{
    update_gui();
}

void PickAndPlacePanel::object_detection_feedback_cb(const hdt::ObjectDetectionFeedback::ConstPtr& feedback)
{
}

void PickAndPlacePanel::object_detection_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::ObjectDetectionResult::ConstPtr& result)
{
    ROS_INFO("Received object detection result");
    if (!result) {
        ROS_WARN("Object Detection Result is null");
        return;
    }

    if (last_detection_request_.request_snapshot) {
        snapshot_cloud_pub_.publish(result->snapshot_cloud);
    }

    if (result->success) {
        ROS_INFO("A successful match was found");
        if (result->match_score != result->match_score) {
             QMessageBox::warning(this, tr("Match Results"), tr("Match succeeded but detector was unable to score it"));
        }
        else {
             QMessageBox::information(this, tr("Match Results"), tr("Match succeeded with score %1").arg(result->match_score));
        }
    }
    else {
        ROS_WARN("No matches found!");
    }

    last_detection_result_ = result;
    pending_detection_request_ = false;

    update_gui();
}

void PickAndPlacePanel::setup_gui()
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    QLabel* training_data_label = new QLabel(tr("Training Data"), this);
    main_layout->addWidget(training_data_label);

    QGridLayout* detection_data_layout = new QGridLayout;

    open_database_button_ = new QPushButton(tr("Choose Object Database"), this);
    open_features_button_ = new QPushButton(tr("Choose Training Data (features)"), this);
    open_kdtree_indices_button_ = new QPushButton(tr("Choose Training Data (kdtree)"), this);

    database_fname_label_ = new QLabel(this);
    features_fname_label_ = new QLabel(this);
    kdtree_indices_fname_label_ = new QLabel(this);

    detection_data_layout->addWidget(open_database_button_, 0, 0);
    detection_data_layout->addWidget(database_fname_label_, 0, 1);
    detection_data_layout->addWidget(open_features_button_, 1, 0);
    detection_data_layout->addWidget(features_fname_label_, 1, 1);
    detection_data_layout->addWidget(open_kdtree_indices_button_, 2, 0);
    detection_data_layout->addWidget(kdtree_indices_fname_label_, 2, 1);

    main_layout->addLayout(detection_data_layout);

    QLabel* frame_data_label = new QLabel(tr("Frame Selection"), this);
    main_layout->addWidget(frame_data_label);

    QHBoxLayout* frame_data_layout = new QHBoxLayout;

    QLabel* camera_frame_box_label = new QLabel(tr("Camera Frame: "), this);
    camera_frame_selection_ = new QComboBox(this);
    QLabel* root_frame_box_label = new QLabel(tr("Root Frame: "), this);
    root_frame_selection_ = new QComboBox(this);

    frame_data_layout->addWidget(camera_frame_box_label);
    frame_data_layout->addWidget(camera_frame_selection_);
    frame_data_layout->addWidget(root_frame_box_label);
    frame_data_layout->addWidget(root_frame_selection_);

    main_layout->addLayout(frame_data_layout);

    QLabel* commands_label = new QLabel(tr("Commands"), this);
    main_layout->addWidget(commands_label);

    snap_point_cloud_button_ = new QPushButton("Take Snapshot");
    main_layout->addWidget(snap_point_cloud_button_);

    update_grasps_button_ = new QPushButton("Update Grasps");
    main_layout->addWidget(update_grasps_button_);

    send_move_to_pregrasp_button_ = new QPushButton("Move to Pre-Grasp");
    main_layout->addWidget(send_move_to_pregrasp_button_);

    send_move_to_flipped_pregrasp_button_ = new QPushButton("Move to Flipped Pre-Grasp");
    main_layout->addWidget(send_move_to_flipped_pregrasp_button_);

    send_move_to_grasp_button_ = new QPushButton("Move to Grasp");
    main_layout->addWidget(send_move_to_grasp_button_);

    send_move_to_flipped_grasp_button_ = new QPushButton("Move to Flipped Grasp");
    main_layout->addWidget(send_move_to_flipped_grasp_button_);

    send_open_gripper_command_button_ = new QPushButton("Open Gripper");
    main_layout->addWidget(send_open_gripper_command_button_);

    send_close_gripper_command_button_ = new QPushButton("Close Gripper");
    main_layout->addWidget(send_close_gripper_command_button_);

    setLayout(main_layout);

    connect(open_database_button_, SIGNAL(clicked()), this, SLOT(choose_database()));
    connect(open_features_button_, SIGNAL(clicked()), this, SLOT(choose_features()));
    connect(open_kdtree_indices_button_, SIGNAL(clicked()), this, SLOT(choose_kdtree_indices()));

    connect(camera_frame_selection_,    SIGNAL(currentIndexChanged(int)),           this, SLOT(camera_frame_box_current_index_changed(int)));
    connect(root_frame_selection_,      SIGNAL(currentIndexChanged(int)),           this, SLOT(root_frame_box_current_index_changed(int)));
    connect(camera_frame_selection_,    SIGNAL(editTextChanged(const QString&)),    this, SLOT(camera_frame_box_edit_text_changed(const QString&)));
    connect(root_frame_selection_,      SIGNAL(editTextChanged(const QString&)),    this, SLOT(camera_frame_box_edit_text_changed(const QString&)));

    connect(snap_point_cloud_button_,               SIGNAL(clicked()), this, SLOT(take_snapshot()));
    connect(update_grasps_button_,                  SIGNAL(clicked()), this, SLOT(update_grasps()));
    connect(send_move_to_pregrasp_button_,          SIGNAL(clicked()), this, SLOT(send_move_to_pregrasp_command()));
    connect(send_move_to_flipped_pregrasp_button_,  SIGNAL(clicked()), this, SLOT(send_move_to_flipped_pregrasp_command()));
    connect(send_move_to_grasp_button_,             SIGNAL(clicked()), this, SLOT(send_move_to_grasp_command()));
    connect(send_move_to_flipped_grasp_button_,     SIGNAL(clicked()), this, SLOT(send_move_to_flipped_grasp_command()));
    connect(send_open_gripper_command_button_,      SIGNAL(clicked()), this, SLOT(send_open_gripper_command()));
    connect(send_close_gripper_command_button_,     SIGNAL(clicked()), this, SLOT(send_close_gripper_command()));
}

tf::Transform PickAndPlacePanel::geomsgs_pose_to_tf_transform(const geometry_msgs::Pose& pose) const
{
    return tf::Transform(
            tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::Pose PickAndPlacePanel::tf_transform_to_geomsgs_pose(const tf::Transform& transform) const
{
    geometry_msgs::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.w = transform.getRotation().w();
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    return pose;
}

bool PickAndPlacePanel::wrist_pose_from_pregrasp_pose(
    const geometry_msgs::PoseStamped& pregrasp_pose,
    geometry_msgs::PoseStamped& out) const
{
    // transform the gripper pose to match the frame orientation of the hdt gripper
    Eigen::Affine3d pr2_pregrasp_transform;
    tf::poseMsgToEigen(pregrasp_pose.pose, pr2_pregrasp_transform);
    Eigen::AngleAxisd pr2_to_hdt_gripper_correction(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));
    Eigen::Affine3d hdt_pregrasp_transform = pr2_pregrasp_transform * pr2_to_hdt_gripper_correction;

    // transform the gripper in the camera frame into the mount frame
    if (!listener_.canTransform(pregrasp_pose.header.frame_id, "arm_mount_panel_dummy", ros::Time(0))) {
        ROS_WARN("Unable to transform from %s to %s", "arm_mount_panel_dummy", pregrasp_pose.header.frame_id.c_str());
        return false;
    }

    // hdt grasp pose in the camera frame
    geometry_msgs::PoseStamped hdt_pregrasp_pose;
    hdt_pregrasp_pose.header = pregrasp_pose.header;
    tf::poseEigenToMsg(hdt_pregrasp_transform, hdt_pregrasp_pose.pose);

    // mount -> gripper = mount -> camera * camera -> gripper
    geometry_msgs::PoseStamped gripper_in_mount_frame;
    try {
        listener_.transformPose("arm_mount_panel_dummy", hdt_pregrasp_pose, gripper_in_mount_frame);
    }
    catch (const tf::TransformException& ex) {
        ROS_WARN("Unable to transform the gripper from '%s' to 'arm_mount_panel_dummy'", hdt_pregrasp_pose.header.frame_id.c_str());
        return false;
    }

    tf::Transform mount_to_gripper = geomsgs_pose_to_tf_transform(gripper_in_mount_frame.pose);

    // gripper -> wrist
//    tf::StampedTransform gripper_to_wrist;
//    const std::string wrist_frame = "arm_7_gripper_lift_link";
//    const std::string gripper_frame = "gripper_base";
//    try {
//        listener_.lookupTransform(gripper_frame, wrist_frame, ros::Time(0), gripper_to_wrist);
//    }
//    catch (const tf::TransformException& ex) {
//        ROS_WARN("Unable to lookup transform from '%s' to '%s'", wrist_frame.c_str(), gripper_frame.c_str());
//        return false;
//    }

    // mount -> wrist = mount -> gripper * gripper -> wrist
    tf::Transform mount_to_wrist = mount_to_gripper; //* gripper_to_wrist;

    geometry_msgs::PoseStamped wrist_pose;
    wrist_pose.header.seq = 0;
    wrist_pose.header.stamp = ros::Time(0);
    wrist_pose.header.frame_id = "arm_mount_panel_dummy";
    wrist_pose.pose = tf_transform_to_geomsgs_pose(mount_to_wrist);
    out = wrist_pose;
    return true;
}

void PickAndPlacePanel::insert_grasp_marker(
    const std::string& name,
    const geometry_msgs::Pose& grasp_pose,
    const ros::Time& time)
{
    visualization_msgs::InteractiveMarker grasp_marker;
    grasp_marker.header.seq = 0;
    grasp_marker.header.stamp = time;
    grasp_marker.header.frame_id = last_detection_request_.root_frame;
    grasp_marker.pose = grasp_pose;
    grasp_marker.name = name;
    grasp_marker.description = "";
    grasp_marker.scale = 1.0f;

    visualization_msgs::InteractiveMarkerControl select_control;
    select_control.name = "Pregrasp Selector";
    select_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    select_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    select_control.always_visible = true;

    geometry_msgs::Vector3 arrow_scale;
    arrow_scale.x = 0.1;
    arrow_scale.y = 0.01;
    arrow_scale.z = 0.01;

    for (const visualization_msgs::Marker& marker : create_triad_markers(arrow_scale).markers) {
        select_control.markers.push_back(marker);
    }
    grasp_marker.controls.push_back(select_control);

    SimpleInteractiveMarkerPtr interactive_marker(
            new GraspMarkerSelectionMarker(grasp_markers_server_, grasp_marker, selection_));
    grasp_markers_server_.insert(interactive_marker);
}

bool PickAndPlacePanel::is_grasp_marker(const std::string& marker_name) const
{
    assert(marker_name.size() >= 5);
    return marker_name.substr(0, 5) == std::string("grasp");
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::PickAndPlacePanel, rviz::Panel);
