#include "grasping_command_model.h"

// system includes
#include <rcta_manipulation_common/comms/actionlib.h>
#include <smpl/stl/memory.h>
#include <smpl/angles.h>
#include <smpl/spatial.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace rcta {

static const char* LOG = "grasping_command_model";

GraspingCommandModel::GraspingCommandModel()
{
    m_obj_mesh_resource = "package://gascan_description/meshes/rcta_gastank.ply";
    m_obj_scale_x = 1.0;
    m_obj_scale_y = 1.0;
    m_obj_scale_z = 1.0;
    m_occupancy_grid_sub = m_nh.subscribe("map", 1, &GraspingCommandModel::occupancyGridCallback, this);
    m_T_world_robot = Eigen::Affine3d::Identity();
    m_T_world_object = Eigen::Affine3d::Identity();
    m_object_id = "crate";
    m_obj_start = 0.0;
    m_obj_goal = 0.0;
    m_allowed_planning_time = 0.0;
    m_plan_only = false;
    m_base_candidate_idx = -1;
}

void GraspingCommandModel::load(const rviz::Config& config)
{
    auto robot_description = QString();
    auto obj_mesh_resource = QString();
    auto obj_scale_x = 0.0f;
    auto obj_scale_y = 0.0f;
    auto obj_scale_z = 0.0f;
    auto base_x = 0.0f;
    auto base_y = 0.0f;
    auto base_yaw = 0.0f;
    auto object_x = 0.0f;
    auto object_y = 0.0f;
    auto object_z = 0.0f;
    auto object_yaw = 0.0f;
    auto object_pitch = 0.0f;
    auto object_roll = 0.0f;
    auto allowed_planning_time = 0.0f;
    auto object_start = 0.0f;
    auto object_goal = 1.0f;
    config.mapGetString("robot_description", &robot_description);
    config.mapGetString("object_mesh_resource", &obj_mesh_resource);
    config.mapGetFloat("object_scale_x", &obj_scale_x);
    config.mapGetFloat("object_scale_y", &obj_scale_y);
    config.mapGetFloat("object_scale_z", &obj_scale_z);
    config.mapGetFloat("base_x", &base_x);
    config.mapGetFloat("base_y", &base_y);
    config.mapGetFloat("base_yaw", &base_yaw);
    config.mapGetFloat("object_x", &object_x);
    config.mapGetFloat("object_y", &object_y);
    config.mapGetFloat("object_z", &object_z);
    config.mapGetFloat("object_yaw", &object_yaw);
    config.mapGetFloat("object_pitch", &object_pitch);
    config.mapGetFloat("object_roll", &object_roll);
    config.mapGetFloat("object_start", &object_start);
    config.mapGetFloat("object_goal", &object_goal);
    config.mapGetFloat("allowed_planning_time", &allowed_planning_time);
    // @plan-only

    ROS_INFO_NAMED(LOG, "Object Manipulation Panel Configuration:");
    ROS_INFO_NAMED(LOG, "  robot_description: %s", robot_description.toStdString().c_str());
    ROS_INFO_NAMED(LOG, "  object_mesh_resource: %s", obj_mesh_resource.toStdString().c_str());
    ROS_INFO_NAMED(LOG, "  object_scale_x: %f", obj_scale_x);
    ROS_INFO_NAMED(LOG, "  object_scale_y: %f", obj_scale_y);
    ROS_INFO_NAMED(LOG, "  object_scale_z: %f", obj_scale_z);
    ROS_INFO_NAMED(LOG, "  base_x: %f", base_x);
    ROS_INFO_NAMED(LOG, "  base_y: %f", base_y);
    ROS_INFO_NAMED(LOG, "  base_yaw: %f", base_yaw);
    ROS_INFO_NAMED(LOG, "  object_x: %f", object_x);
    ROS_INFO_NAMED(LOG, "  object_y: %f", object_y);
    ROS_INFO_NAMED(LOG, "  object_z: %f", object_z);
    ROS_INFO_NAMED(LOG, "  object_yaw: %f", object_yaw);
    ROS_INFO_NAMED(LOG, "  object_pitch: %f", object_pitch);
    ROS_INFO_NAMED(LOG, "  object_roll: %f", object_roll);
    ROS_INFO_NAMED(LOG, "  object_start: %f", object_start);
    ROS_INFO_NAMED(LOG, "  object_goal: %f", object_goal);
    ROS_INFO_NAMED(LOG, "  allowed_planning_time: %f", allowed_planning_time);

    if (!robot_description.isEmpty()) {
        // attempt to initalize the robot using this robot description
        auto why = std::string();
        (void)loadRobot(robot_description.toStdString(), &why);
    }

    setObjectMeshResource(obj_mesh_resource.toStdString());
    setObjectMeshScaleX(obj_scale_x);
    setObjectMeshScaleY(obj_scale_y);
    setObjectMeshScaleZ(obj_scale_z);

    setRobotPoseX(base_x);
    setRobotPoseY(base_y);
    setRobotPoseYaw(base_yaw);

    setObjectPoseX(object_x);
    setObjectPoseY(object_y);
    setObjectPoseZ(object_z);
    setObjectPoseYaw(object_yaw);
    setObjectPosePitch(object_pitch);
    setObjectPoseRoll(object_roll);

    setObjectStart(object_start);
    setObjectGoal(object_goal);
    setAllowedPlanningTime(allowed_planning_time);
}

void GraspingCommandModel::save(rviz::Config& config) const
{
    config.mapSetValue("robot_description", QString::fromStdString(m_robot_description));
    config.mapSetValue("object_mesh_resource", QString::fromStdString(m_obj_mesh_resource));

    config.mapSetValue("object_scale_x", m_obj_scale_x);
    config.mapSetValue("object_scale_y", m_obj_scale_y);
    config.mapSetValue("object_scale_z", m_obj_scale_z);

    config.mapSetValue("base_x", m_T_world_robot.translation()(0, 0));
    config.mapSetValue("base_y", m_T_world_robot.translation()(1, 0));
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_robot.rotation(), yaw, pitch, roll);
    config.mapSetValue("base_yaw", yaw);

    config.mapSetValue("object_x", m_T_world_object.translation()(0, 0));
    config.mapSetValue("object_y", m_T_world_object.translation()(1, 0));
    config.mapSetValue("object_z", m_T_world_object.translation()(2, 0));
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    config.mapSetValue("object_yaw", yaw);
    config.mapSetValue("object_pitch", pitch);
    config.mapSetValue("object_roll", roll);

    config.mapSetValue("object_start", m_obj_start);
    config.mapSetValue("object_goal", m_obj_goal);

    config.mapSetValue("allowed_planning_time", m_allowed_planning_time);
}

bool GraspingCommandModel::loadRobot(const std::string& urdf_param, std::string* why)
{
    // Check for URDF and SRDF parameters
    auto srdf_param = urdf_param + "_semantic";
    if (!m_nh.hasParam(urdf_param) || !m_nh.hasParam(srdf_param)) {
        if (why != NULL) {
            std::stringstream ss;
            ss << "Failed to retrieve '" << urdf_param << "' and '" <<
                    (urdf_param + "_semantic") << "' from the param server";
            *why = ss.str();
        }
        return false;
    }

    auto urdf_string = std::string();
    if (!m_nh.getParam(urdf_param, urdf_string)) {
        if (why != NULL) {
            std::stringstream ss;
            ss << "Failed to retrieve '" << urdf_param <<
                    "' from the param server";
            *why = ss.str();
        }
        return false;
    }

    auto load_kinematics_solvers = true;
    auto rml = robot_model_loader::RobotModelLoaderPtr(
            new robot_model_loader::RobotModelLoader(
                    urdf_param, load_kinematics_solvers));

    auto robot_model = rml->getModel();
    if (!robot_model) {
        if (why != NULL) {
            *why = "Robot Model Loader was unable to construct Robot Model";
        }
        return false;
    }

    auto robot_state = smpl::make_unique<robot_state::RobotState>(robot_model);

    // All lights are green from above
    m_robot_description = urdf_param;
    m_rml = std::move(rml);
    m_robot_model = std::move(robot_model);
    m_robot_state = std::move(robot_state);

    Q_EMIT robotLoaded(QString::fromStdString(urdf_param));

    m_robot_state->setToDefaultValues();

    Q_EMIT robotStateChanged();
}

bool GraspingCommandModel::robotLoaded() const
{
    return !m_robot_description.empty();
}

void GraspingCommandModel::graspObjectCommandResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::GraspObjectCommandResult::ConstPtr& result)
{
    ROS_DEBUG_NAMED(LOG, "Received Result from Grasp Object Command Action");
    Q_EMIT receivedGraspObjectResult();
}

void GraspingCommandModel::repositionBaseCommandResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::RepositionBaseCommandResult::ConstPtr& result)
{

    ROS_DEBUG_NAMED(LOG, "Received Result from Reposition Base Command Action");

    if (result && result->result == cmu_manipulation_msgs::RepositionBaseCommandResult::SUCCESS) {
        m_candidate_base_poses = result->candidate_base_poses;
        ROS_DEBUG_NAMED(LOG, "Reposition Base Command returned %zd candidate poses", m_candidate_base_poses.size());
    }

    Q_EMIT receivedRepositionBaseResult();
}

void GraspingCommandModel::manipulateObjectCommandResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const cmu_manipulation_msgs::ManipulateObjectResult::ConstPtr& result)
{
    ROS_DEBUG_NAMED(LOG, "Received result from Manipulate Object action");

    Q_EMIT receivedManipulateObjectResult();
}

void GraspingCommandModel::occupancyGridCallback(
    const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    m_occupancy_grid = msg;
}

void GraspingCommandModel::setObjectMeshResource(const std::string& path)
{
    if (path != m_obj_mesh_resource) {
        m_obj_mesh_resource = path;
        Q_EMIT objectMeshResourceChanged(QString::fromStdString(path));
    }
}

void GraspingCommandModel::setObjectMeshScaleX(double x)
{
    if (x != m_obj_scale_x) {
        m_obj_scale_x = x;
        Q_EMIT objectMeshScaleChanged(m_obj_scale_x, m_obj_scale_y, m_obj_scale_z);
    }
}

void GraspingCommandModel::setObjectMeshScaleY(double y)
{
    if (y != m_obj_scale_y) {
        m_obj_scale_y = y;
        Q_EMIT objectMeshScaleChanged(m_obj_scale_x, m_obj_scale_y, m_obj_scale_z);
    }
}

void GraspingCommandModel::setObjectMeshScaleZ(double z)
{
    if (z != m_obj_scale_z) {
        m_obj_scale_z = z;
        Q_EMIT objectMeshScaleChanged(m_obj_scale_x, m_obj_scale_y, m_obj_scale_z);
    }
}

void GraspingCommandModel::setRobotPoseToCurrent()
{
    try {
        auto world_to_robot = tf::StampedTransform();
        auto model_frame = m_robot_model->getModelFrame();
        auto robot_frame = m_robot_model->getRootLinkName();
        m_listener.lookupTransform(model_frame, robot_frame, ros::Time(0), world_to_robot);
        tf::transformTFToEigen(world_to_robot, m_T_world_robot);

        double roll, pitch, yaw;
        smpl::get_euler_zyx(m_T_world_robot.rotation(), yaw, pitch, roll);
    } catch (const tf::TransformException& ex) {
        Q_EMIT errorEncountered(tr("Transform exception: %1").arg(ex.what()));
    }
}

void GraspingCommandModel::setRobotPoseX(double val)
{
    m_T_world_robot.translation()(0,0) = val;
    Q_EMIT robotStateChanged();
}

void GraspingCommandModel::setRobotPoseY(double val)
{
    m_T_world_robot.translation()(1,0) = val;
    Q_EMIT robotStateChanged();
}

void GraspingCommandModel::setRobotPoseYaw(double val)
{
    m_T_world_robot =
            Eigen::Translation3d(m_T_world_robot.translation()) *
            Eigen::AngleAxisd(val, Eigen::Vector3d::UnitZ());
    Q_EMIT robotStateChanged();
}

void GraspingCommandModel::sendGraspObjectCommand()
{
    // robot -> object = robot -> world * world -> object
    auto robot_to_object = Eigen::Affine3d(m_T_world_robot.inverse() * m_T_world_object);

    if (!ReconnectActionClient(m_grasp_object_command_client, "grasp_object_command")) {
        // TODO: there should be a GUI printout or terminal or something for this stuff
        Q_EMIT errorEncountered(tr("Unable to send Grasp Object Command (server is not available)"));
        return;
    }

    auto grasp_object_goal = cmu_manipulation_msgs::GraspObjectCommandGoal{ };
    grasp_object_goal.gas_can_in_base_link.header.frame_id = m_robot_model->getModelFrame();
    tf::poseEigenToMsg(robot_to_object, grasp_object_goal.gas_can_in_base_link.pose);

    static int grasp_object_goal_id = 0;
    grasp_object_goal.id = grasp_object_goal_id++;
    grasp_object_goal.retry_count = 0;

    grasp_object_goal.gas_can_in_map.header.frame_id = m_robot_model->getModelFrame();
    tf::poseEigenToMsg(m_T_world_object, grasp_object_goal.gas_can_in_map.pose);

    auto result_callback = boost::bind(
            &GraspingCommandModel::graspObjectCommandResultCallback, this, _1, _2);
    m_grasp_object_command_client->sendGoal(grasp_object_goal, result_callback);
}

void GraspingCommandModel::sendRepositionBaseCommand()
{
    if (!ReconnectActionClient(m_reposition_base_command_client, "reposition_base_command")) {
        Q_EMIT errorEncountered(tr("Unable to send Reposition Base Command (server is not available)"));
        return;
    }

    if (!m_occupancy_grid) {
        Q_EMIT errorEncountered(tr("No map data received"));
        return;
    }

    cmu_manipulation_msgs::RepositionBaseCommandGoal reposition_base_goal;

    static int reposition_base_goal_id = 0;
    reposition_base_goal.id = reposition_base_goal_id++;
    reposition_base_goal.retry_count = 0;

    tf::poseEigenToMsg(m_T_world_object, reposition_base_goal.gas_can_in_map.pose);
    reposition_base_goal.gas_can_in_map.header.frame_id = m_robot_model->getModelFrame();

    tf::poseEigenToMsg(m_T_world_robot, reposition_base_goal.base_link_in_map.pose);
    reposition_base_goal.base_link_in_map.header.frame_id = m_robot_model->getModelFrame();

    reposition_base_goal.map = *m_occupancy_grid;

    auto result_callback = boost::bind(&GraspingCommandModel::repositionBaseCommandResultCallback, this, _1, _2);
    m_reposition_base_command_client->sendGoal(reposition_base_goal, result_callback);
}

void GraspingCommandModel::setAllowedPlanningTime(double val)
{
    if (m_allowed_planning_time != val) {
        m_allowed_planning_time = val;
        Q_EMIT allowedPlanningTimeChanged(val);
    }
}

void GraspingCommandModel::setObjectStart(double val)
{
    if (m_obj_start != val) {
        m_obj_start = val;
        Q_EMIT objectStartChanged(val);
    }
}

void GraspingCommandModel::setObjectGoal(double val)
{
    if (m_obj_goal != val) {
        m_obj_goal = val;
        Q_EMIT objectGoalChanged(val);
    }
}

void GraspingCommandModel::sendManipulateObjectCommand()
{
    if (!ReconnectActionClient(m_manipulate_object_client, "manipulate_object")) {
        Q_EMIT errorEncountered(tr("Unable to send Manipulate Object command (server is not available)"));
        return;
    }

    auto goal = cmu_manipulation_msgs::ManipulateObjectGoal{ };

    static auto goal_id = 0;
    goal.object_id = m_object_id;
    tf::poseEigenToMsg(m_T_world_object, goal.object_pose);
    goal.object_start = m_obj_start;
    goal.object_goal = m_obj_goal;
    goal.plan_only = m_plan_only;
    goal.allowed_planning_time = m_allowed_planning_time;
    goal.start_state.is_diff = true;

    auto result_callback = boost::bind(&GraspingCommandModel::manipulateObjectCommandResultCallback, this, _1, _2);
    m_manipulate_object_client->sendGoal(goal, result_callback);
}

void GraspingCommandModel::setObjectPoseX(double val)
{
    if (m_T_world_object.translation().x() != val) {
        m_T_world_object.translation().x() = val;
        Q_EMIT objectPoseXChanged(val);
    }
}

void GraspingCommandModel::setObjectPoseY(double val)
{
    if (m_T_world_object.translation().y() != val) {
        m_T_world_object.translation().y() = val;
        Q_EMIT objectPoseYChanged(val);
    }
}

void GraspingCommandModel::setObjectPoseZ(double val)
{
    if (m_T_world_object.translation().z() != val) {
        m_T_world_object.translation().z() = val;
        Q_EMIT objectPoseZChanged(val);
    }
}

void GraspingCommandModel::setObjectPoseYaw(double val)
{
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    if (val != yaw) {
        yaw = val;
        auto x = m_T_world_object.translation().x();
        auto y = m_T_world_object.translation().y();
        auto z = m_T_world_object.translation().z();
        m_T_world_object = smpl::MakeAffine(x, y, z, yaw, pitch, roll);
        Q_EMIT objectPoseYawChanged(val);
    }
}

void GraspingCommandModel::setObjectPosePitch(double val)
{
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    if (val != pitch) {
        pitch = val;
        auto x = m_T_world_object.translation().x();
        auto y = m_T_world_object.translation().y();
        auto z = m_T_world_object.translation().z();
        m_T_world_object = smpl::MakeAffine(x, y, z, yaw, pitch, roll);
        Q_EMIT objectPosePitchChanged(val);
    }
}

void GraspingCommandModel::setObjectPoseRoll(double val)
{
    double yaw, pitch, roll;
    smpl::get_euler_zyx(m_T_world_object.rotation(), yaw, pitch, roll);
    if (val != roll) {
        roll = val;
        auto x = m_T_world_object.translation().x();
        auto y = m_T_world_object.translation().y();
        auto z = m_T_world_object.translation().z();
        m_T_world_object = smpl::MakeAffine(x, y, z, yaw, pitch, roll);
        Q_EMIT objectPoseRollChanged(val);
    }
}

} // namespace rcta
