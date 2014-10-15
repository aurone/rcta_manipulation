#include "ArmPlanningNode.h"
#include <cassert>
#include <cstdio>
#include <queue>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <octomap_msgs/conversions.h>
#include <sbpl_geometry_utils/utils.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/utils/utils.h>
#include "HDTRobotModel.h"

namespace hdt
{

ArmPlanningNode::ArmPlanningNode() :
    nh_(),
    ph_("~"),
    marker_array_pub_(),
    joint_trajectory_pub_(),
    joint_states_sub_(),
    move_arm_command_server_(),
    follow_trajectory_client_("arm_controller/joint_trajectory_action"),
    use_action_server_(false),
    action_set_filename_(),
    kinematics_frame_(),
    planning_frame_(),
    object_filename_(),
    urdf_string_(),
    robot_name_(),
    robot_local_frame_(),
    robot_model_(),
    planner_robot_model_(),
    distance_field_(),
    grid_(),
    collision_checker_(),
    sbpl_action_set_(),
    planner_(),
    last_joint_state_(),
    received_joint_state_(),
    listener_(),
    joint_staleness_threshold_(10.0),
    urdf_model_(),
    statistic_names_({
        "initial solution planning time",
        "initial epsilon",
        "initial solution expansions",
        "final epsilon planning time",
        "final epsilon",
        "solution epsilon",
        "expansions",
        "solution cost" }),
    planning_scene_()
{
}

bool ArmPlanningNode::init()
{
    if (!init_robot()) {
        ROS_ERROR_PRETTY("Failed to initialize robot model");
        return false;
    }

    if (!init_collision_model()) {
        ROS_ERROR_PRETTY("Failed to initialize collision model");
        return false;
    }

    if (!init_sbpl()) {
        ROS_ERROR_PRETTY("Failed to initialize SBPL");
        return false;
    }

    ph_.param("use_action_server", use_action_server_, true);
    if (use_action_server_) {
        ROS_INFO_PRETTY("Using action server 'arm_controller/joint_trajectory_action'");
    }
    else {
        ROS_INFO_PRETTY("Publishing joint trajectories to command");
    }

    ph_.param("apply_shortcutting", apply_shortcutting_, true);
    ROS_INFO_PRETTY("Apply Shortcutting: %s", boolstr(apply_shortcutting_));

    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
    joint_trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &ArmPlanningNode::joint_states_callback, this);

    auto move_command_callback = boost::bind(&ArmPlanningNode::move_arm, this, _1);
    move_arm_command_server_.reset(new MoveArmActionServer("move_arm_command", move_command_callback, false));
    if (!move_arm_command_server_) {
        ROS_ERROR_PRETTY("Failed to instantiate Move Arm Action Server");
        return false;
    }

    move_arm_command_server_->start();

    ROS_INFO_PRETTY("Waiting for action client \"arm_controller/joint_trajectory_action\"");
    follow_trajectory_client_.waitForServer();

    return true;
}

int ArmPlanningNode::run()
{
    ROS_INFO_PRETTY("Spinning...");
    ros::spin();
    ROS_INFO_PRETTY("Done spinning");
    return 0;
}

bool ArmPlanningNode::reinit(
    const Eigen::Affine3d& T_kinematics_planning,
    const std::string& planning_frame,
    octomap::OcTree* octree)
{
    ROS_INFO_PRETTY("Reinitializing Planner");
    return reinit_robot(T_kinematics_planning) && reinit_collision_model(planning_frame, octree) && reinit_sbpl();
}

bool ArmPlanningNode::reinit_robot(const Eigen::Affine3d& T_kinematics_planning)
{
    ROS_INFO_PRETTY("Reinitializing Robot");
    KDL::Frame F_kinematics_planning;
    tf::transformEigenToKDL(T_kinematics_planning, F_kinematics_planning);
    planner_robot_model_->setKinematicsToPlanningTransform(F_kinematics_planning, kinematics_frame_); // todo: is this what string its asking for?
    return true;
}

bool ArmPlanningNode::reinit_collision_model(const std::string& planning_frame, octomap::OcTree* octomap)
{
    ROS_INFO_PRETTY("Reinitializing collision model");

    // if we didn't receive an octomap, we better be planning in the robot's base frame
    assert(((bool)octomap) ? (true) : (planning_frame == robot_local_frame_));

    const double max_dist_m = 0.2;
    const double cell_res_m = 0.02;

    // initialize the distance field
    std::unique_ptr<distance_field::PropagationDistanceField> distance_field;
    if (!octomap) {
        // still need to instantiate a distance field to check for collisions against other collision groups
        ROS_WARN_PRETTY("  Octomap from message does not support OcTree type requirements");
        ROS_WARN_PRETTY("  Planning with empty collision map in the robot frame");

        const double size_x = 3.0, size_y = 3.0, size_z = 3.0;
        const double origin_x = -0.75, origin_y = -1.25, origin_z = -1.0;
        distance_field.reset(new distance_field::PropagationDistanceField(
                size_x, size_y, size_z, cell_res_m, origin_x, origin_y, origin_z, max_dist_m));
        if (!distance_field) {
            ROS_ERROR_PRETTY("Failed to instantiate Propagation Distance Field");
            return false;
        }

        distance_field->reset();
    }
    else {
        ROS_INFO_PRETTY("  Octree: %p", octomap);
        size_t num_nodes = octomap->calcNumNodes();
        ROS_INFO_PRETTY("    Num Nodes: %zd", num_nodes);
        ROS_INFO_PRETTY("    Memory Usage: %zd bytes", octomap->memoryUsage());
        ROS_INFO_PRETTY("    Num Leaf Nodes: %zd", octomap->getNumLeafNodes());

        unsigned num_thresholded, num_other;
        octomap->calcNumThresholdedNodes(num_thresholded, num_other);
        ROS_INFO_PRETTY("    Num Thresholded Nodes: %u", num_thresholded);
        ROS_INFO_PRETTY("    Num Other Nodes: %u", num_other);

        const octomap::point3d octomap_min = octomap->getBBXMin();
        const octomap::point3d octomap_max = octomap->getBBXMax();
        const octomap::point3d octomap_center = octomap->getBBXCenter();
        double clamping_thresh_min = octomap->getClampingThresMin();
        double clamping_thresh_max = octomap->getClampingThresMax();

        ROS_INFO_PRETTY("    Bounding Box Set: %s", octomap->bbxSet() ? "TRUE" : "FALSE");
        ROS_INFO_PRETTY("    Bounding Box Min: (%0.3f, %0.3f, %0.3f)", octomap_min.x(), octomap_min.y(), octomap_min.z());
        ROS_INFO_PRETTY("    Bounding Box Max: (%0.3f, %0.3f, %0.3f)", octomap_max.x(), octomap_max.y(), octomap_max.z());
        ROS_INFO_PRETTY("    Bounding Box Center: (%0.3f, %0.3f, %0.3f)", octomap_center.x(), octomap_center.y(), octomap_center.z());
        ROS_INFO_PRETTY("    Clamping Threshold Min: %0.3f", clamping_thresh_min);
        ROS_INFO_PRETTY("    Clamping Threshold Max: %0.3f", clamping_thresh_max);

        double metric_min_x, metric_min_y, metric_min_z;
        double metric_max_x, metric_max_y, metric_max_z;
        double metric_size_x, metric_size_y, metric_size_z;
        octomap->getMetricMin(metric_min_x, metric_min_y, metric_min_z);
        octomap->getMetricMax(metric_max_x, metric_max_y, metric_max_z);

        ROS_INFO_PRETTY("    Metric Min: (%0.3f, %0.3f, %0.3f)", metric_min_x, metric_min_y, metric_min_z);
        ROS_INFO_PRETTY("    Metric Max: (%0.3f, %0.3f, %0.3f)", metric_max_x, metric_max_y, metric_max_z);

        octomap->getMetricSize(metric_size_x, metric_size_y, metric_size_z);
        ROS_INFO_PRETTY("    Metric Size: (%0.3f, %0.3f, %0.3f)", metric_size_x, metric_size_y, metric_size_z);

        ROS_INFO_PRETTY("    Node Size (max depth): %0.6f", octomap->getNodeSize(octomap->getTreeDepth()));
        ROS_INFO_PRETTY("    Occupancy Threshold: %0.3f", octomap->getOccupancyThres());
        ROS_INFO_PRETTY("    Probability Hit: %0.3f", octomap->getProbHit());
        ROS_INFO_PRETTY("    Probability Miss: %0.3f", octomap->getProbMiss());
        ROS_INFO_PRETTY("    Resolution: %0.3f", octomap->getResolution());
        ROS_INFO_PRETTY("    Depth: %u", octomap->getTreeDepth());
        ROS_INFO_PRETTY("    Tree Type: %s", octomap->getTreeType().c_str());

        // gather octree statistics
        int num_occupied_octree_cells = 0;
        std::map<double, int> occupancy_hist;
        std::map<float, int> value_hist;
        std::map<bool, int> occupied_hist;
        for (auto tit = octomap->begin(); tit != octomap->end(); ++tit) {
            const octomap::OcTreeNode& node = *tit;

            double occupancy = node.getOccupancy();
            if (occupancy_hist.find(occupancy) == occupancy_hist.end()) {
                occupancy_hist[occupancy] = 0;
            }
            else {
                ++occupancy_hist[occupancy];
            }

            float value = node.getValue();
            if (value_hist.find(value) == value_hist.end()) {
                value_hist[value] = 0;
            }
            else {
                ++value_hist[value];
            }

            bool occupied = octomap->isNodeOccupied(node);
            if (occupied_hist.find(occupied) == occupied_hist.end()) {
                occupied_hist[occupied] = 0;
            }
            else {
                ++occupied_hist[occupied];
            }
        }

        ROS_INFO_PRETTY("    Occupancy Histogram:");
        for (const auto& entry : occupancy_hist) {
            ROS_INFO_PRETTY("      %0.3f: %d", entry.first, entry.second);
        }

        ROS_INFO_PRETTY("    Value Histogram:");
        for (const auto& entry : value_hist) {
            ROS_INFO_PRETTY("      %0.3f: %d", entry.first, entry.second);
        }

        ROS_INFO_PRETTY("    Occupied Histogram:");
        for (const auto& entry : occupied_hist) {
            ROS_INFO_PRETTY("      %s: %d", boolstr(entry.first), entry.second);
        }

        octomap::point3d metric_min(metric_min_x, metric_min_y, metric_min_z);
        octomap::point3d metric_max(metric_max_x, metric_max_y, metric_max_z);

        distance_field.reset(new distance_field::PropagationDistanceField(*octomap, metric_min, metric_max, max_dist_m));
        if (!distance_field) {
            ROS_ERROR_PRETTY("Failed to instantiate Propagation Distance Field");
            return false;
        }

        distance_field->reset();
        this->addOcTreeToField(distance_field.get(), octomap);

        ROS_INFO_PRETTY("  Distance Field:");
        ROS_INFO_PRETTY("    Pointer: %p", distance_field.get());
        ROS_INFO_PRETTY("    Size (m): (%0.3f, %0.3f, %0.3f)", distance_field->getSizeX(), distance_field->getSizeY(), distance_field->getSizeZ());
        ROS_INFO_PRETTY("    Size (cells): (%d", distance_field->getXNumCells(), distance_field->getYNumCells(), distance_field->getZNumCells());
        ROS_INFO_PRETTY("    Origin (m): (%0.3f, %0.3f, %0.3f)", distance_field->getOriginX(), distance_field->getOriginY(), distance_field->getOriginZ());
        ROS_INFO_PRETTY("    Resolution (m): %0.3f", distance_field->getResolution());
        ROS_INFO_PRETTY("    Uninitialized Distance: %0.3f", distance_field->getUninitializedDistance());

        int num_invalid_cells = 0;
        for (int x = 0; x < distance_field->getXNumCells(); ++x) {
            for (int y = 0; y < distance_field->getYNumCells(); ++y) {
                for (int z = 0; z < distance_field->getZNumCells(); ++z) {
                    if (!distance_field->getDistance(x, y, z) <= 0.0) {
                        ++num_invalid_cells;
                    }
                }
            }
        }

        int num_cells = distance_field->getXNumCells() * distance_field->getYNumCells() * distance_field->getZNumCells();

        ROS_INFO_PRETTY("    Num Cells: %d", num_cells);
        ROS_INFO_PRETTY("      Num Invalid Cells: %d", num_invalid_cells);
        ROS_INFO_PRETTY("      Num Valid Cells: %d", num_cells - num_invalid_cells);
    }

    distance_field_ = std::move(distance_field);

    ROS_INFO_PRETTY("Initializing Occupancy Grid");
    grid_.reset(new sbpl_arm_planner::OccupancyGrid(distance_field_.get()));
    if (!grid_) {
        ROS_ERROR_PRETTY("Failed to instantiate Occupancy Grid");
        return false;
    }

    grid_->setReferenceFrame(planning_frame);

    ROS_INFO_PRETTY("  OccupancyGrid:");
    int num_cells_x, num_cells_y, num_cells_z;
    grid_->getGridSize(num_cells_x, num_cells_y, num_cells_z);
    ROS_INFO_PRETTY("    Dimensions (cells): (%d, %d, %d)", num_cells_x, num_cells_y, num_cells_z);
    double grid_size_x, grid_size_y, grid_size_z;
    grid_->getWorldSize(grid_size_x, grid_size_y, grid_size_z);
    ROS_INFO_PRETTY("    Dimensions (m): (%0.3f, %0.3f, %0.3f)", grid_size_x, grid_size_y, grid_size_z);
    double grid_origin_x, grid_origin_y, grid_origin_z;
    grid_->getOrigin(grid_origin_x, grid_origin_y, grid_origin_z);
    ROS_INFO_PRETTY("    Origin (m): (%0.3f, %0.3f, %0.3f)", grid_origin_x, grid_origin_y, grid_origin_z);
    ROS_INFO_PRETTY("    Resolution: %0.3f", grid_->getResolution());
    ROS_INFO_PRETTY("    Reference Frame: %s", grid_->getReferenceFrame().c_str());

    collision_checker_.reset(new sbpl_arm_planner::SBPLCollisionSpace(grid_.get()));
    if (!collision_checker_) {
        ROS_ERROR_PRETTY("  Failed to instantiate SBPL Collision Space");
        return false;
    }

    const std::string group_name = "manipulator"; // This has something to do with the collision checking config file?
    if (!collision_checker_->init(urdf_string_, group_name) ||
        !collision_checker_->setPlanningJoints(robot_model_->joint_names()))
    {
        ROS_ERROR_PRETTY("  Failed to initialize SBPL Collision Checker");
        return false;
    }

    auto distance_field_markers = collision_checker_->getVisualization("distance_field");
    ROS_INFO_PRETTY("Publishing %zd distance field visualization markers", distance_field_markers.markers.size());
    ROS_INFO_PRETTY("Visualizing marker types:");
    for (const auto& marker : distance_field_markers.markers) {
        ROS_INFO_PRETTY("  %s, Num Points: %zd", visualization_msgs_marker_type_to_cstr(marker.type), marker.points.size());
    }
    marker_array_pub_.publish(distance_field_markers);

    ROS_INFO_PRETTY("Successfully reinitialized collision model");
    return true;
}

bool ArmPlanningNode::reinit_sbpl()
{
    ROS_INFO_PRETTY("Reinitializing SBPL");
    planner_.reset(new sbpl_arm_planner::SBPLArmPlannerInterface(
            planner_robot_model_.get(), collision_checker_.get(), sbpl_action_set_.get(), distance_field_.get()));

    if (!planner_) {
        ROS_ERROR_PRETTY("Failed to instantiate SBPL Arm Planner Interface");
        return false;
    }

    if (!planner_->init()) {
        ROS_ERROR_PRETTY("Failed to initialize SBPL Arm Planner Interface");
        return false;
    }

    return true;
}

bool ArmPlanningNode::init_robot()
{
    ROS_INFO_PRETTY("Initializing Robot Models");

    if (!nh_.hasParam("robot_description")) {
        ROS_ERROR_PRETTY("Missing parameter \"robot_description\"");
        return false;
    }

    nh_.getParam("robot_description", urdf_string_);

    boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(urdf_string_);
    if (!urdf_model) {
        ROS_ERROR_PRETTY("Failed to parse URDF");
        return false;
    }

    std::vector<std::string> all_joints;
    all_joints.reserve(urdf_model->joints_.size());

    std::queue<boost::shared_ptr<const urdf::Link>> links;
    links.push(urdf_model->getRoot());
    // traverse the tree in breadth-first fashion to gather all the joints for this model
    while (!links.empty()) {
        // grab the next link
        boost::shared_ptr<const urdf::Link> link = links.front();
        links.pop();

        // gather the child joints of this link
        for (const boost::shared_ptr<urdf::Joint>& child_joint : link->child_joints) {
            if (child_joint->type != urdf::Joint::FIXED) {
                all_joints.push_back(child_joint->name);
            }
        }

        // add all children to the queue
        for (const boost::shared_ptr<urdf::Link>& child_link : link->child_links) {
            links.push(child_link);
        }
    }

    ROS_INFO_PRETTY("  All Joints (%zd):", all_joints.size());
    for (const std::string& joint : all_joints) {
        ROS_INFO_PRETTY("    %s", joint.c_str());
    }

    robot_name_ = urdf_model->getName();
    robot_local_frame_ = urdf_model->getRoot()->name;
    ROS_INFO_PRETTY("  Robot Name: %s", robot_name_.c_str());
    ROS_INFO_PRETTY("  Robot Local Frame: %s", robot_local_frame_.c_str());

    size_t num_nonfixed_joints = all_joints.size();
    last_joint_state_.header.frame_id = "";
    last_joint_state_.header.seq = 0;
    last_joint_state_.header.stamp = ros::Time(0);
    last_joint_state_.name = std::move(all_joints);
    last_joint_state_.position = std::vector<double>(num_nonfixed_joints, 0.0);
    last_joint_state_.velocity = std::vector<double>(num_nonfixed_joints, 0.0);
    last_joint_state_.effort = std::vector<double>(num_nonfixed_joints, 0.0);
    received_joint_state_ = std::vector<ros::Time>(num_nonfixed_joints, ros::Time(0));

    // Initialize Robot Model
    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string_);
    if (!robot_model_) {
        ROS_ERROR_PRETTY("Failed to load Robot Model from URDF");
        return false;
    }

    // Initialize Planning Robot Model
    HDTRobotModel* hdt_robot_model = new HDTRobotModel;
    if (!hdt_robot_model || !hdt_robot_model->init(urdf_string_)) {
        ROS_ERROR_PRETTY("Failed to initialize HDT Robot Model");
        return false;
    }
    planner_robot_model_.reset(hdt_robot_model);
    if (!planner_robot_model_) {
        ROS_ERROR_PRETTY("Failed to instantiate KDL Robot Model");
    }

    std::vector<std::string> planning_joints = robot_model_->joint_names();
    if (!planner_robot_model_->init(urdf_string_, planning_joints)) {
        ROS_ERROR_PRETTY("Failed to initialize KDL Robot Model");
        return false;
    }
    planner_robot_model_->setPlanningLink(robot_model_->joint_names().back());

    kinematics_frame_ = "arm_mount_panel_dummy";

    return true;
}

bool ArmPlanningNode::init_collision_model()
{
    return true;
}

bool ArmPlanningNode::init_sbpl()
{
    if (!ph_.hasParam("action_set_filename")) {
        ROS_ERROR_PRETTY("Missing parameter \"action_set_filename\"");
        return false;
    }

    ph_.getParam("action_set_filename", action_set_filename_);

    sbpl_action_set_.reset(new sbpl_arm_planner::ActionSet(action_set_filename_));
    if (!sbpl_action_set_) {
        ROS_ERROR_PRETTY("Failed to instantiate Action Set");
        return false;
    }

    planning_scene_.reset(new moveit_msgs::PlanningScene);
    if (!planning_scene_) {
        ROS_ERROR_PRETTY("Failed to instantiate Planning Scene");
        return false;
    }

    return true;
}

void ArmPlanningNode::move_arm(const hdt::MoveArmCommandGoal::ConstPtr& request)
{
    ros::Time now = ros::Time::now();
    ROS_INFO_PRETTY("Received Move Arm Command Goal at %s", boost::posix_time::to_simple_string(now.toBoost()).c_str());

    // create the octomap here
    std::shared_ptr<octomap::AbstractOcTree> octomap(octomap_msgs::fullMsgToMap(request->octomap));
    octomap::OcTree* octree(dynamic_cast<octomap::OcTree*>(octomap.get()));
    bool valid_octomap = (bool)octree;

    std::string planning_frame;
    if (valid_octomap) {
        planning_frame = request->octomap.header.frame_id;
        ROS_INFO_PRETTY("Valid Octomap. Planning in frame '%s'", planning_frame.c_str());
    }
    else {
        planning_frame = robot_local_frame_;
        ROS_INFO_PRETTY("Invalid Octomap. Planning in frame '%s'", planning_frame.c_str());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Check for up-to-date state
    ////////////////////////////////////////////////////////////////////////////////

    // check for recent joint information
    for (size_t i = 0; i < received_joint_state_.size(); ++i) {
        const ros::Time& time = received_joint_state_[i];
        if (time < now - joint_staleness_threshold_) {
            hdt::MoveArmCommandResult result;
            result.success = false;
            std::stringstream ss;
            ss << "Joint information for joint '" << last_joint_state_.name[i]
               << "' is more than " << joint_staleness_threshold_.toSec() << " seconds old";
            ROS_WARN_PRETTY("%s", ss.str().c_str());
            move_arm_command_server_->setAborted(result, ss.str());
            return;
        }
    }

    ROS_INFO_PRETTY("  Joint States are up-to-date (within %0.3f seconds)", joint_staleness_threshold_.toSec());

    // check for recent transform from the robot to the planning frame
    geometry_msgs::Pose robot_pose_planning_frame = geometry_msgs::IdentityPose();
    if (planning_frame == robot_local_frame_) {
        ROS_WARN_PRETTY("Unidentfied octomap received. Planning in robot's local frame without obstacles");
    }
    else {
        tf::StampedTransform T_map_robot_tf;
        try {
            listener_.lookupTransform(planning_frame, robot_local_frame_, ros::Time(0), T_map_robot_tf);
            Eigen::Affine3d T_map_to_robot;
            msg_utils::convert(T_map_robot_tf, T_map_to_robot);
            tf::poseEigenToMsg(T_map_to_robot, robot_pose_planning_frame);
        }
        catch (const tf::TransformException& ex) {
            hdt::MoveArmCommandResult result;
            result.success = false;
            move_arm_command_server_->setAborted(result, ex.what());
            return;
        }
    }

    ROS_INFO_PRETTY("  Robot Pose [%s]: %s", planning_frame.c_str(), to_string(robot_pose_planning_frame).c_str());

    // look up the transform from the kinematics frame (the frame that goals are specified in) to the planning frame
    tf::StampedTransform kinematics_to_planning_tf;
    Eigen::Affine3d T_kinematics_planning;
    try {
        listener_.lookupTransform(kinematics_frame_, planning_frame, ros::Time(0), kinematics_to_planning_tf);
        msg_utils::convert(kinematics_to_planning_tf, T_kinematics_planning);
    }
    catch (const tf::TransformException& ex) {
        hdt::MoveArmCommandResult result;
        result.success = false;
        move_arm_command_server_->setAborted(result, ex.what());
        return;
    }

    ROS_INFO_PRETTY("  Kinematics -> Planning: %s", to_string(T_kinematics_planning).c_str());

    ////////////////////////////////////////////////////////////////////////////////
    // Set up the planning scene
    ////////////////////////////////////////////////////////////////////////////////

    planning_scene_->name = "manipulation_planning_scene";

    moveit_msgs::RobotState robot_state;
    robot_state.joint_state = last_joint_state_;
    robot_state.multi_dof_joint_state.joint_names = { "robot_pose" };
    robot_state.multi_dof_joint_state.joint_transforms.resize(1);
    robot_state.multi_dof_joint_state.joint_transforms[0].translation.x = robot_pose_planning_frame.position.x;
    robot_state.multi_dof_joint_state.joint_transforms[0].translation.y = robot_pose_planning_frame.position.y;
    robot_state.multi_dof_joint_state.joint_transforms[0].translation.z = robot_pose_planning_frame.position.z;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.w = robot_pose_planning_frame.orientation.w;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.x = robot_pose_planning_frame.orientation.x;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.y = robot_pose_planning_frame.orientation.y;
    robot_state.multi_dof_joint_state.joint_transforms[0].rotation.z = robot_pose_planning_frame.orientation.z;
    planning_scene_->robot_state = robot_state;

    planning_scene_->robot_model_name = robot_name_;
    planning_scene_->robot_model_root = robot_local_frame_;

    planning_scene_->fixed_frame_transforms.clear();  // todo: are these necessary?
    planning_scene_->allowed_collision_matrix;        // todo: covered by sbpl_collision_model?
    planning_scene_->link_padding.clear();            // todo: covered by sbpl_collision_model?
    planning_scene_->link_scale.clear();              // todo: covered by sbpl_collision_model?
    planning_scene_->object_colors.clear();           // todo: also probably not used

    // Fill out header information for the collision map so that sbpl::manipulation doesn't get pissed off
    planning_scene_->world.collision_objects.clear();
    // planning_scene_->world.octomap = request->octomap; // todo: covered by sbpl_collision_model/occupancy grid?
    planning_scene_->world.collision_map.header = std_msgs::CreateHeader(0, ros::Time(0), planning_frame);

    planning_scene_->is_diff = false;

    ////////////////////////////////////////////////////////////////////////////////
    // Reinitialize for updated collision model
    ////////////////////////////////////////////////////////////////////////////////

    if (!reinit(T_kinematics_planning, planning_frame, octree)) {
        hdt::MoveArmCommandResult result;
        result.success = false;
        move_arm_command_server_->setAborted(result, "Failed to initialize Arm Planner");
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Plan to the received goal state
    ////////////////////////////////////////////////////////////////////////////////

    collision_checker_->setPlanningScene(*planning_scene_);

    // transform the wrist goal from the kinematics frame to the planning frame
    Eigen::Affine3d T_kinematics_wrist_goal;
    tf::poseMsgToEigen(request->goal_pose, T_kinematics_wrist_goal);

    Eigen::Affine3d T_planning_kinematics(T_kinematics_planning.inverse());
    Eigen::Affine3d T_planning_wrist_goal(T_planning_kinematics * T_kinematics_wrist_goal);

    geometry_msgs::PoseStamped wrist_goal_planning_frame;
    wrist_goal_planning_frame.header.seq = 0;
    wrist_goal_planning_frame.header.stamp = ros::Time(0);
    wrist_goal_planning_frame.header.frame_id = planning_frame;
    tf::poseEigenToMsg(T_planning_wrist_goal, wrist_goal_planning_frame.pose);

    bool success = false;
    trajectory_msgs::JointTrajectory result_traj;
    if (request->type == hdt::MoveArmCommandGoal::JointGoal) {
        ROS_INFO_PRETTY("Received a joint goal");
        success = plan_to_joint_goal(planning_scene_, planning_scene_->robot_state, *request, result_traj);
    }
    else if (request->type == hdt::MoveArmCommandGoal::EndEffectorGoal) {
        ROS_INFO_PRETTY("Received an end effector goal");
        success = plan_to_eef_goal(planning_scene_, wrist_goal_planning_frame, result_traj);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Post-process the plan and send to the joint trajectory follower
    ////////////////////////////////////////////////////////////////////////////////

    if (!success) {
        hdt::MoveArmCommandResult result;
        result.success = false;
        result.trajectory;
        move_arm_command_server_->setAborted(result, "Failed to plan path");
        return;
    }

    ROS_INFO_PRETTY("Original joint path (%zd points):", result_traj.points.size());
    for (int i = 0; i < (int)result_traj.points.size(); ++i) {
        const trajectory_msgs::JointTrajectoryPoint& joint_state = result_traj.points[i];
        ROS_INFO_PRETTY("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
    }

    if (apply_shortcutting_) {
        apply_shortcutting(result_traj);
    }

    ROS_INFO_PRETTY("Shortcut trajectory (%zd points):", result_traj.points.size());
    for (int i = 0; i < (int)result_traj.points.size(); ++i) {
        const trajectory_msgs::JointTrajectoryPoint& joint_state = result_traj.points[i];
        ROS_INFO_PRETTY("    Point %3d: %s", i, to_string(joint_state.positions).c_str());
    }

    bool interp_res = add_interpolation_to_plan(result_traj);
    if (!interp_res) {
        ROS_ERROR_PRETTY("Failed to interpolate joint trajectory");
        hdt::MoveArmCommandResult result;
        result.success = false;
        move_arm_command_server_->setAborted(result, "Failed to interpolate joint trajectory");
        return;
    }

    if (request->execute_path) {
        publish_trajectory(result_traj);
    }

    hdt::MoveArmCommandResult result;
    result.success = true;
    result.trajectory = result_traj;
    move_arm_command_server_->setSucceeded(result);
}

void ArmPlanningNode::fill_constraint(
    const std::vector<double>& pose,
    const std::string& frame_id,
    moveit_msgs::Constraints& goals)
{
    if(pose.size() < 6)
        return;

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

    //  goals.position_constraints[0].position.x = pose[0];
    //  goals.position_constraints[0].position.y = pose[1];
    //  goals.position_constraints[0].position.z = pose[2];

    leatherman::rpyToQuatMsg(pose[3], pose[4], pose[5], goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO_PRETTY("Done packing the goal constraints message.");
}

moveit_msgs::CollisionObject ArmPlanningNode::get_collision_cube(
    const geometry_msgs::Pose& pose,
    const std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

std::vector<moveit_msgs::CollisionObject>
ArmPlanningNode::get_collision_cubes(
    const std::vector<std::vector<double>>& objects,
    const std::vector<std::string>& object_ids,
    const std::string& frame_id)
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO_PRETTY("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(get_collision_cube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

std::vector<moveit_msgs::CollisionObject>
ArmPlanningNode::get_collision_objects(const std::string& filename, const std::string& frame_id)
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO_PRETTY("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg, "%s", sTemp) < 1)
        printf("Parsed string has length < 1.\n");

    num_obs = atoi(sTemp);

    ROS_INFO_PRETTY("%i objects in file", num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i = 0; i < num_obs; ++i) {
        if (fscanf(fCfg, "%s", sTemp) < 1)
            printf("Parsed string has length < 1.\n");
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j = 0; j < 6; ++j) {
            if (fscanf(fCfg, "%s", sTemp) < 1)
                printf("Parsed string has length < 1.\n");
            if (!feof(fCfg) && strlen(sTemp) != 0)
                objects[i][j] = atof(sTemp);
        }
    }

    return get_collision_cubes(objects, object_ids, frame_id);
}

void ArmPlanningNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        double joint_position = msg->position[i];
        double joint_velocity = msg->velocity[i];
        double joint_effort = msg->effort[i];

        int jidx = msg_utils::get_joint_index(last_joint_state_, joint_name);
        if (jidx == -1) {
            ROS_WARN_PRETTY("Ignoring unrecognized joint '%s' from joint state message", joint_name.c_str());
            continue;
        }

        received_joint_state_[jidx] = now;

        last_joint_state_.header.stamp = msg->header.stamp;
        last_joint_state_.position[jidx] = joint_position;
        last_joint_state_.velocity[jidx] = joint_velocity;
        last_joint_state_.effort[jidx] = joint_effort;
    }
}

bool ArmPlanningNode::add_interpolation_to_plan(trajectory_msgs::JointTrajectory& res_traj) const
{
    if (robot_model_->min_limits().size() != robot_model_->num_joints() ||
        robot_model_->max_limits().size() != robot_model_->num_joints() ||
        robot_model_->continuous().size() != robot_model_->num_joints())
    {
        ROS_ERROR_PRETTY("Joint variable descriptions do not match the number of planning joints");
        ROS_ERROR_PRETTY("  Num Joints: %zd", robot_model_->num_joints());
        ROS_ERROR_PRETTY("  Num Min Limits: %zd", robot_model_->min_limits().size());
        ROS_ERROR_PRETTY("  Num Max Limits: %zd", robot_model_->max_limits().size());
        ROS_ERROR_PRETTY("  Num Continuous: %zd", robot_model_->continuous().size());
        return false;
    }

    ROS_INFO_PRETTY("Interpolating trajectory of size %zd", res_traj.points.size());

    if (res_traj.joint_names.empty()) {
        res_traj.joint_names = robot_model_->joint_names();
    }

    // find the index of each planning joint in the returned message
    std::map<std::string, int> planning_joint_indices;
    std::vector<std::string> index_to_planning_joint;
    for (int i = 0; i < (int)res_traj.joint_names.size(); ++i) {
        const std::string& joint_name = res_traj.joint_names[i];
        planning_joint_indices[joint_name] = i;
    }

    // interpolate between each consecutive pair of waypoints
    trajectory_msgs::JointTrajectory interp_traj;
    interp_traj.joint_names = res_traj.joint_names;
    for (int i = 0; i < (int)res_traj.points.size() - 1; ++i) {
        const trajectory_msgs::JointTrajectoryPoint& curr_point = res_traj.points[i];
        const trajectory_msgs::JointTrajectoryPoint& next_point = res_traj.points[i + 1];

        if (curr_point.positions.size() != robot_model_->joint_names().size() || next_point.positions.size() != robot_model_->joint_names().size()) {
            ROS_WARN_PRETTY("Intermediate joint trajectory point does not have as many joints as the number of planning joints (%zd)", curr_point.positions.size());
            return false;
        }

        // grab the start and end angles in the order of the planning joints
        std::vector<double> start(robot_model_->joint_names().size());
        std::vector<double> end(robot_model_->joint_names().size());
        for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
            const std::string& planning_joint = robot_model_->joint_names()[j];
            start[j]  = curr_point.positions[planning_joint_indices[planning_joint]];
            end[j]    = next_point.positions[planning_joint_indices[planning_joint]];
        }

        // add the first point
        if (i == 0) {
            trajectory_msgs::JointTrajectoryPoint first_point;
            first_point.positions.resize(robot_model_->joint_names().size());
            for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
                const std::string& planning_joint = robot_model_->joint_names()[j];
                first_point.positions[planning_joint_indices[planning_joint]] = start[j];
            }
            interp_traj.points.push_back(first_point);
        }

        // find the largest angle difference from start to end to determine the number of increments
        std::vector<double> angle_distances(robot_model_->joint_names().size());
        for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
            angle_distances[j] = sbpl::utils::ShortestAngleDistWithLimits(start[j], end[j], robot_model_->min_limits()[j], robot_model_->max_limits()[j]);
        }
        double largest_angle_dist = *std::max_element(angle_distances.begin(), angle_distances.end());
        // interpolate all angles at the resolution necessary to make sure no
        // angle moves more than 1 degree between points and all angles start
        // and stop moving at the same time
        double num_increments = largest_angle_dist / sbpl::utils::ToRadians(1.0);

        std::vector<double> inc(robot_model_->joint_names().size(), sbpl::utils::ToRadians(1.0)); // this is variable depending on transition
//        std::vector<double> inc(robot_model_->joint_names().size()); // this is variable depending on transition
//        for (int j = 0; j < (int)robot_model_->joint_names().size(); ++j) {
//            inc[j] = angle_distances[j] / num_increments;
//        }

        std::vector<std::vector<double>> path;
        if (!sbpl::interp::InterpolatePath(start, end, robot_model_->min_limits(), robot_model_->max_limits(), inc, robot_model_->continuous(), path)) {
            ROS_ERROR_PRETTY("Failed to interpolate planned path");
            return false;
        }

        // for each point in the interpolated path
        for (int j = 0; j < (int)path.size(); ++j) {
            // except the first point (already covered by the end point of the previous interpolated waypoint trajectory
            if (j != 0) {
                const std::vector<double>& interm_point = path[j];
                trajectory_msgs::JointTrajectoryPoint interm_jtpt;
                interm_jtpt.positions.resize(robot_model_->joint_names().size());
                for (int k = 0; k < robot_model_->joint_names().size(); ++k) {
                    const std::string& planning_joint = robot_model_->joint_names()[k];
                    interm_jtpt.positions[planning_joint_indices[planning_joint]] = interm_point[k];
                }
                interp_traj.points.push_back(interm_jtpt);
            }
        }
    }

    ROS_INFO_PRETTY("Interpolated joint trajectory contains %zd points", interp_traj.points.size());

    res_traj = interp_traj;

    return true;
}

void ArmPlanningNode::publish_trajectory(const trajectory_msgs::JointTrajectory& joint_trajectory)
{
    const std::vector<std::string> joint_names = {
            "arm_1_shoulder_twist",
            "arm_2_shoulder_lift",
            "arm_3_elbow_twist",
            "arm_4_elbow_lift",
            "arm_5_wrist_twist",
            "arm_6_wrist_lift",
            "arm_7_gripper_lift",
    };

    // map from planning joints to index in resulting message
    std::map<std::string, int> planning_joint_indices;
    for (int i = 0; i < (int)joint_trajectory.joint_names.size(); ++i) {
        const std::string& joint_name = joint_trajectory.joint_names[i];
        planning_joint_indices.insert({joint_name, i});
    }

    if (joint_trajectory.points.empty()) {
        ROS_INFO_PRETTY("Not publishing empty trajectory");
        return;
    }

    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = "";
    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();

    traj.joint_names = joint_names;
    traj.points.resize(joint_trajectory.points.size());
    int pidx = 0;
    double t = 0.0;
    for (const auto& point : joint_trajectory.points) {
        trajectory_msgs::JointTrajectoryPoint& traj_point = traj.points[pidx];
        traj_point.positions.resize(traj.joint_names.size());
        traj_point.velocities.resize(traj.joint_names.size());
        int jidx = 0;
        for (const auto& joint_name : traj.joint_names) {
            if (std::find(robot_model_->joint_names().begin(), robot_model_->joint_names().end(), joint_name) != robot_model_->joint_names().end()) {
                // if this joint is a planning joint grab it from the planner trajectory
                traj_point.positions[jidx] = point.positions[planning_joint_indices[joint_name]];
            }
            else {
                // just set to 0 for now; search should be seeded with the initial non-variable state
                traj_point.positions[jidx] = 0.0;
            }
            traj_point.time_from_start = ros::Duration(t);
            traj_point.velocities[jidx] = sbpl::utils::ToRadians(20.0);
            ++jidx;
        }

        t += 0.10;
        ++pidx;
    }

    if (!use_action_server_) {
        ROS_INFO_PRETTY("Publishing trajectory to the arm controller");
        joint_trajectory_pub_.publish(traj);
    }
    else {
        ROS_INFO_PRETTY("Sending trajectory goal via actionlib");
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = std::move(traj);
        follow_trajectory_client_.sendGoal(goal);
        follow_trajectory_client_.waitForResult();
        if (follow_trajectory_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO_PRETTY("Yay! The dishes are now clean");
        }
        ROS_INFO_PRETTY("Current State: %s", follow_trajectory_client_.getState().toString().c_str());
    }
}

void ArmPlanningNode::apply_shortcutting(trajectory_msgs::JointTrajectory& joint_trajectory) const
{
    std::vector<int> costs(joint_trajectory.points.size() - 1, 1);

    JointInterpolationPathGenerator generator;
    if (!generator.initialize(collision_checker_, robot_model_->min_limits(), robot_model_->max_limits(), robot_model_->continuous())) {
        ROS_ERROR_PRETTY("Failed to initialize Joint Interpolation Path Generator");
        return;
    }

    std::vector<JointInterpolationPathGenerator> generators;
    generators.push_back(generator);

    std::vector<trajectory_msgs::JointTrajectoryPoint> new_points;
    bool shortcut_res = sbpl::shortcut::ShortcutPath(joint_trajectory.points, costs, generators, new_points);

    if (!shortcut_res) {
        ROS_ERROR_PRETTY("Failed to shortcut trajectory");
    }
    else {
        joint_trajectory.points = std::move(new_points);
    }
}

bool ArmPlanningNode::plan_to_eef_goal(
    const moveit_msgs::PlanningScenePtr& scene,
    const geometry_msgs::PoseStamped& goal_pose,
    trajectory_msgs::JointTrajectory& traj)
{
    // fill goal state
    moveit_msgs::GetMotionPlan::Request req;
    req.motion_plan_request.goal_constraints.resize(1);
    std::vector<double> goal_vector = convert_to_sbpl_goal(goal_pose.pose);
    fill_constraint(goal_vector, goal_pose.header.frame_id, req.motion_plan_request.goal_constraints[0]);
    ROS_WARN_PRETTY("Created a goal in the '%s' frame", req.motion_plan_request.goal_constraints.front().position_constraints[0].header.frame_id.c_str());
    req.motion_plan_request.allowed_planning_time = 10.0; //2.0;
    req.motion_plan_request.start_state = scene->robot_state;

    // plan
    ROS_INFO_PRETTY("Calling solve...");
    moveit_msgs::GetMotionPlan::Response res;
    bool plan_result = planner_->solve(scene, req, res);
    if (!plan_result) {
        ROS_ERROR_PRETTY("Failed to plan.");
    }
    else {
        ROS_INFO_PRETTY("Planning succeeded");
    }

    // print statistics
    std::map<std::string, double> planning_stats = planner_->getPlannerStats();

    ROS_INFO_PRETTY("Planning statistics");
    for (const auto& statistic : statistic_names_) {
        auto it = planning_stats.find(statistic);
        if (it != planning_stats.end()) {
            ROS_INFO_PRETTY("    %s: %0.3f", statistic.c_str(), it->second);
        }
        else {
            ROS_WARN_PRETTY("Did not find planning statistic \"%s\"", statistic.c_str());
        }
    }

    // visualizations
    marker_array_pub_.publish(collision_checker_->getVisualization("bounds"));
    auto distance_field_markers = collision_checker_->getVisualization("distance_field");
    ROS_INFO_PRETTY("Distance Field Visualization contains %zd markers", distance_field_markers.markers.front().points.size());
    marker_array_pub_.publish(distance_field_markers);
    marker_array_pub_.publish(collision_checker_->getVisualization("collision_objects"));
    marker_array_pub_.publish(planner_->getVisualization("goal"));

    // visualize, filter, and publish plan
    if (plan_result) {
        marker_array_pub_.publish(planner_->getCollisionModelTrajectoryMarker());
        traj = res.motion_plan_response.trajectory.joint_trajectory;
    }

    return plan_result;
}

bool ArmPlanningNode::plan_to_joint_goal(
    const moveit_msgs::PlanningScenePtr& scene,
    const moveit_msgs::RobotState& start,
    const hdt::MoveArmCommandGoal& goal,
    trajectory_msgs::JointTrajectory& traj)
{
    sensor_msgs::JointState goal_joint_state = goal.goal_joint_state;
    if (!msg_utils::reorder_joints(goal_joint_state, robot_model_->joint_names())) {
        ROS_WARN_PRETTY("Goal state contains joints other than manipulator joints");
        return false;
    }

    // fill goal state
    moveit_msgs::GetMotionPlan::Request req;
    req.motion_plan_request.goal_constraints.resize(1);

    std::vector<double> goal_vector = goal_joint_state.position;
    std::vector<double> goal_tolerances (7, 0.05); 

    req.motion_plan_request.goal_constraints.resize(1);
    req.motion_plan_request.goal_constraints.front().joint_constraints.resize(goal_vector.size());
    for(int i = 0; i < goal_vector.size(); i++){
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].position = goal_joint_state.position[i];
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].joint_name = goal_joint_state.name[i];
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_above = 3.0 * M_PI / 180.0;
      req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_below = -3.0 * M_PI / 180.0;
      ROS_INFO_PRETTY("%s: %.3f tol(%.3f, %.3f)", goal_joint_state.name[i].c_str(), goal_joint_state.position[i], req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_below, req.motion_plan_request.goal_constraints.front().joint_constraints[i].tolerance_above);
    }

    ROS_WARN_PRETTY("Created a 7DoF goal!");

    req.motion_plan_request.allowed_planning_time = 10.0; //2.0;
    req.motion_plan_request.start_state = scene->robot_state;

    // plan
    ROS_INFO_PRETTY("Calling solve...");
    moveit_msgs::GetMotionPlan::Response res;
    bool plan_result = planner_->solve(scene, req, res);
    if (!plan_result) {
        ROS_ERROR_PRETTY("Failed to plan.");
    }
    else {
        ROS_INFO_PRETTY("Planning succeeded");
    }

    // print statistics
    std::map<std::string, double> planning_stats = planner_->getPlannerStats();

    ROS_INFO_PRETTY("Planning statistics");
    for (const auto& statistic : statistic_names_) {
        auto it = planning_stats.find(statistic);
        if (it != planning_stats.end()) {
            ROS_INFO_PRETTY("    %s: %0.3f", statistic.c_str(), it->second);
        }
        else {
            ROS_WARN_PRETTY("Did not find planning statistic \"%s\"", statistic.c_str());
        }
    }

    // visualizations
    marker_array_pub_.publish(collision_checker_->getVisualization("bounds"));
    auto distance_field_markers = collision_checker_->getVisualization("distance_field");
    ROS_INFO_PRETTY("Distance Field Visualization contains %zd markers", distance_field_markers.markers.front().points.size());
    marker_array_pub_.publish(distance_field_markers);    marker_array_pub_.publish(collision_checker_->getVisualization("collision_objects"));
    marker_array_pub_.publish(planner_->getVisualization("goal"));

    // visualize, filter, and publish plan
    if (plan_result) {
        marker_array_pub_.publish(planner_->getCollisionModelTrajectoryMarker());
        traj = res.motion_plan_response.trajectory.joint_trajectory;
    }

    return plan_result;
}

std::vector<double> ArmPlanningNode::convert_to_sbpl_goal(const geometry_msgs::Pose& pose)
{
    tf::Quaternion goal_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 goal_rotation_matrix(goal_quat);
    double goal_roll, goal_pitch, goal_yaw;
    goal_rotation_matrix.getEulerYPR(goal_yaw, goal_pitch, goal_roll);
    return { pose.position.x, pose.position.y, pose.position.z, goal_roll, goal_pitch, goal_yaw };
}

void ArmPlanningNode::clamp_to_joint_limits(
    std::vector<double>& joint_vector,
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits)
{
    auto clamp = [](double d, double min, double max) { if (d < min) return min; else if (d > max) return max; else return d; };
    for (std::size_t i = 0; i < joint_vector.size(); ++i) {
        joint_vector[i] = clamp(joint_vector[i], robot_model_->min_limits()[i], robot_model_->max_limits()[i]);
    }
}

bool ArmPlanningNode::valid_octomap(const octomap_msgs::Octomap& msg)
{
    return true;
}

void ArmPlanningNode::addOcTreeToField(distance_field::DistanceField* df, const octomap::OcTree* octree)
{
    //lower extent
    double min_x, min_y, min_z;
    df->gridToWorld(0,0,0, min_x, min_y, min_z);

    octomap::point3d bbx_min(min_x, min_y, min_z);

    int num_x = df->getXNumCells();
    int num_y = df->getYNumCells();
    int num_z = df->getZNumCells();

    //upper extent
    double max_x, max_y, max_z;
    df->gridToWorld(num_x, num_y, num_z, max_x, max_y, max_z);

    octomap::point3d bbx_max(max_x, max_y, max_z);

    EigenSTL::vector_Vector3d points;

    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbx_min,bbx_max), end=octree->end_leafs_bbx(); it!= end; ++it)
    {
        if (octree->isNodeOccupied(*it))
        {
            if(it.getSize() <= df->getResolution()) {
                Eigen::Vector3d point(it.getX(), it.getY(), it.getZ());
                points.push_back(point);
            }
            else {
                double ceil_val = ceil(it.getSize()/df->getResolution())*df->getResolution();
                for(double x = it.getX()-ceil_val; x < it.getX()+ceil_val; x += df->getResolution()) {
                for(double y = it.getY()-ceil_val; y < it.getY()+ceil_val; y += df->getResolution()) {
                for(double z = it.getZ()-ceil_val; z < it.getZ()+ceil_val; z += df->getResolution()) {
                    points.push_back(Eigen::Vector3d(x,y,z));
                }
                }
                }
            }
        }
    }

    ROS_INFO_PRETTY("Adding %zd points to the distance field", points.size());
    df->addPointsToField(points);
}

} //namespace hdt

