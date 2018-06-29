#include <math.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <leatherman/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/GetStateValidity.h>

////////////////////
// STEALING DEFER //
////////////////////

namespace scdl {

template <class Callable>
struct CallOnDestruct
{
    Callable c;
    CallOnDestruct(Callable c) : c(c) { }
    ~CallOnDestruct() { c(); }
};

template <class Callable>
CallOnDestruct<Callable> MakeCallOnDestruct(Callable c) {
    return CallOnDestruct<Callable>(c);
}

} // namespace scdl

////////////////////

// preprocessor magic to get a prefix to concatenate with the __LINE__ macro
#define MAKE_LINE_IDENT_JOIN_(a, b) a##b
#define MAKE_LINE_IDENT_JOIN(a, b) MAKE_LINE_IDENT_JOIN_(a, b)
#define MAKE_LINE_IDENT(prefix) MAKE_LINE_IDENT_JOIN(prefix, __LINE__)

// create an obscurely named CallOnDestruct with an anonymous lambda that
// executes the given statement sequence
#define DEFER(fun) auto MAKE_LINE_IDENT(tmp_call_on_destruct_) = scdl::MakeCallOnDestruct([&](){ fun; })


#define RIGHT_ARM 1

#if RIGHT_ARM
static const auto door_min_pos = -M_PI;
static const auto door_max_pos = -1.0 * M_PI / 4.0;
#else
static const auto door_min_pos = -3.0 * M_PI / 4.0;
static const auto door_max_pos = 0.0;
#endif

struct CabinetModel
{
    double depth;
    double width;
    double height;
    double thickness;

    double handle_offset_y; // how far from the center of the door is the handle positioned
    double handle_offset_x; // how much space between the door and the handle
    double handle_height; // how tall is the handle
    double handle_radius; // how thick is the handle (and its supports)
};

auto GetCabinetToHingeTransform(CabinetModel* model)
    -> Eigen::Affine3d
{
#if RIGHT_ARM
    Eigen::Affine3d T_cabinet_hinge(Eigen::Translation3d(
            0.5 * model->depth + 0.5 * model->thickness,
            0.5 * model->width - 0.5 * model->thickness,
            0.0));
#else
    Eigen::Affine3d T_cabinet_hinge(Eigen::Translation3d(
            0.5 * model->depth + 0.5 * model->thickness,
            -0.5 * model->width + 0.5 * model->thickness,
            0.0));
#endif
    return T_cabinet_hinge;
}

// hinge_pos in [0, 1]
auto GetHingeTransform(CabinetModel* model, double hinge_pos) -> Eigen::Affine3d
{
#if RIGHT_ARM
    return Eigen::Affine3d(
            Eigen::AngleAxisd(
                3.0 * M_PI / 4.0 * hinge_pos,
                Eigen::Vector3d::UnitZ()));
#else
    return Eigen::Affine3d(
            Eigen::AngleAxisd(
                -3.0 * M_PI / 4.0 * hinge_pos,
                Eigen::Vector3d::UnitZ()));
#endif
}

auto GetHingeToDoorTransform(CabinetModel* model) -> Eigen::Affine3d
{
#if RIGHT_ARM
    return Eigen::Affine3d(Eigen::Translation3d(
            0.0,
            -(0.5 * model->width - 0.5 * model->thickness),
            0.0));
#else
    return Eigen::Affine3d(Eigen::Translation3d(
            0.0,
            0.5 * model->width - 0.5 * model->thickness,
            0.0));
#endif
}

auto GetHandleRotationRadius(CabinetModel* model) -> double
{
    auto dx = 0.5 * model->thickness + model->handle_offset_x;
    auto dy = 0.5 * model->width - 0.5 * model->thickness + model->handle_offset_y;
    return std::sqrt(dx * dx + dy * dy);
}

auto GetHandleRotationOffset(CabinetModel* model) -> double
{
    auto dx = 0.5 * model->thickness + model->handle_offset_x;
#if RIGHT_ARM
    auto dy = 0.5 * model->width - 0.5 * model->thickness - model->handle_offset_y;
#else
    auto dy = 0.5 * model->width - 0.5 * model->thickness + model->handle_offset_y;
#endif
    return atan2(fabs(dx), fabs(dy));
//    return std::sqrt(dx * dx + dy * dy);
}

auto GetHingeFrame(CabinetModel* model, const Eigen::Affine3d* pose)
    -> Eigen::Affine3d
{
    Eigen::Affine3d T_cabinet_hinge = GetCabinetToHingeTransform(model);
    return (*pose) * T_cabinet_hinge;
}

auto GetHandlePose(CabinetModel* model, const Eigen::Affine3d* pose, double door_pos)
    -> Eigen::Affine3d
{
    Eigen::Affine3d T_cabinet_hinge = GetCabinetToHingeTransform(model);

    Eigen::Affine3d door_pose;
    door_pose = (*pose)
            * T_cabinet_hinge
            * GetHingeTransform(model, door_pos)
            * GetHingeToDoorTransform(model);

    Eigen::Affine3d handle_pose;
    handle_pose =
            door_pose *
            Eigen::Translation3d(
                    0.5 * model->thickness + model->handle_offset_x + model->handle_radius,
                    model->handle_offset_y,
                    0.0);

    return handle_pose;
}

auto MakeCabinetMarkers(
    CabinetModel* model,
    double door_pos,
    const Eigen::Affine3d* pose,
    const char* ns,
    bool contact)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;
    ma.markers.reserve(9);

    visualization_msgs::Marker m;

    // common marker settings
    m.header.frame_id = "map";
    m.ns = ns;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.color = leatherman::colors::AntiqueWhite();
    m.lifetime = ros::Duration(0.0);

    /////////////////
    // bottom wall //
    /////////////////

    m.id = 0;

    Eigen::Affine3d b_wall_pose;
    b_wall_pose = (*pose) * Eigen::Translation3d(
            0.0, 0.0, -0.5 * model->height + 0.5 * model->thickness);

    tf::poseEigenToMsg(b_wall_pose, m.pose);

    m.scale.x = model->depth;
    m.scale.y = model->width;
    m.scale.z = model->thickness;

    ma.markers.push_back(m);

    //////////////
    // top wall //
    //////////////

    m.id = 1;

    Eigen::Affine3d t_wall_pose;
    t_wall_pose = (*pose) * Eigen::Translation3d(
            0.0, 0.0, 0.5 * model->height - 0.5 * model->thickness);
    tf::poseEigenToMsg(t_wall_pose, m.pose);

    m.scale.x = model->depth;
    m.scale.y = model->width;
    m.scale.z = model->thickness;

    ma.markers.push_back(m);

    ///////////////
    // left wall //
    ///////////////

    m.id = 2;

    Eigen::Affine3d l_wall_pose;
    l_wall_pose = (*pose) * Eigen::Translation3d(
            0.0, -0.5 * model->width + 0.5 * model->thickness, 0.0);
    tf::poseEigenToMsg(l_wall_pose, m.pose);

    m.scale.x = model->depth;
    m.scale.y = model->thickness;
    m.scale.z = model->height - 2.0 * model->thickness;

    ma.markers.push_back(m);

    ////////////////
    // right wall //
    ////////////////

    m.id = 3;

    Eigen::Affine3d r_wall_pose;
    r_wall_pose = (*pose) * Eigen::Translation3d(
            0.0, 0.5 * model->width - 0.5 * model->thickness, 0.0);
    tf::poseEigenToMsg(r_wall_pose, m.pose);

    m.scale.x = model->depth;
    m.scale.y = model->thickness;
    m.scale.z = model->height - 2.0 * model->thickness;

    ma.markers.push_back(m);

    ///////////////
    // back wall //
    ///////////////

    m.id = 4;

    Eigen::Affine3d back_wall_pose;
    back_wall_pose = (*pose)
        * Eigen::Translation3d(
                -0.5 * model->depth - 0.5 * model->thickness,
                0.0,
                0.0);
    tf::poseEigenToMsg(back_wall_pose, m.pose);

    m.scale.x = model->thickness;
    m.scale.y = model->width;
    m.scale.z = model->height;// - 2.0 * model->thickness;

    ma.markers.push_back(m);

    //////////
    // door //
    //////////

    Eigen::Affine3d T_cabinet_hinge = GetCabinetToHingeTransform(model);

    m.id = 5;

    Eigen::Affine3d door_pose;
    door_pose = (*pose)
            * T_cabinet_hinge
            * GetHingeTransform(model, door_pos)
            * GetHingeToDoorTransform(model);

    tf::poseEigenToMsg(door_pose, m.pose);

    m.scale.x = model->thickness;
    m.scale.y = model->width;
    m.scale.z = model->height;

    ma.markers.push_back(m);

    //////////////////
    // handle shaft //
    //////////////////

    if (contact) {
        m.color = leatherman::colors::Red();
    } else {
        m.color = leatherman::colors::DarkSlateGray();
    }
    m.type = visualization_msgs::Marker::CYLINDER;

    m.id = 6;

    Eigen::Affine3d shaft_pose;
    shaft_pose =
            door_pose *
            Eigen::Translation3d(
                    0.5 * model->thickness + model->handle_offset_x + model->handle_radius,
                    model->handle_offset_y,
                    0.0);

    tf::poseEigenToMsg(shaft_pose, m.pose);

    m.scale.x = 2.0 * model->handle_radius;
    m.scale.y = 2.0 * model->handle_radius;
    m.scale.z = model->handle_height + 4.0 * model->handle_radius;

    ma.markers.push_back(m);

    ////////////////////
    // handle support //
    ////////////////////

    m.id = 7;

    Eigen::Affine3d t_support_pose;
    t_support_pose =
            door_pose *
            Eigen::Translation3d(
                    0.5 * (0.5 * model->thickness + model->handle_offset_x + model->handle_radius),
                    model->handle_offset_y,
                    0.5 * model->handle_height + model->handle_radius) *
            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());

    tf::poseEigenToMsg(t_support_pose, m.pose);

    m.scale.x = 2.0 * model->handle_radius;
    m.scale.y = 2.0 * model->handle_radius;
    m.scale.z = (0.5 * model->thickness + model->handle_offset_x + model->handle_radius);

    ma.markers.push_back(m);

    ////////////////////
    // handle support //
    ////////////////////

    m.id = 8;

    Eigen::Affine3d b_support_pose;
    b_support_pose =
            door_pose *
            Eigen::Translation3d(
                    0.5 * (0.5 * model->thickness + model->handle_offset_x + model->handle_radius),
                    model->handle_offset_y,
                    -(0.5 * model->handle_height + model->handle_radius)) *
            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());

    tf::poseEigenToMsg(b_support_pose, m.pose);

    m.scale.x = 2.0 * model->handle_radius;
    m.scale.y = 2.0 * model->handle_radius;
    m.scale.z = (0.5 * model->thickness + model->handle_offset_x + model->handle_radius);

    ma.markers.push_back(m);

    return ma;
}

bool g_state_received = false;
moveit_msgs::RobotState::ConstPtr g_curr_robot_state;

void RobotStateCallback(const moveit_msgs::RobotState::ConstPtr& msg)
{
    g_curr_robot_state = msg;
    g_state_received = true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "door_demonstrator", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ROS_INFO("argc: %d", argc);

    auto cabinet_id = (argc > 1) ? atoi(argv[1]) : 0;
    bool record = argc > 2 ? true : false;
    bool record_object = argc > 3 ? true: false;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 1);

    // to perform usual validity checking
    ros::ServiceClient check_state_service =
            nh.serviceClient<moveit_msgs::GetStateValidity>(
                    "check_state_validity");

    ros::Subscriber state_sub =
            nh.subscribe("robot_state", 1, RobotStateCallback);

    robot_model_loader::RobotModelLoader loader;
    auto model = loader.getModel();

    moveit::core::RobotState robot_state(model);

    auto group_name = "right_arm_torso_base";
    if (!model->hasJointModelGroup(group_name)) {
        ROS_ERROR("Demonstration group '%s' does not exist in the robot model", group_name);
        return 1;
    }

    auto* demo_group = model->getJointModelGroup(group_name);

    FILE* fdemo = NULL;
    if (record) {
        auto demo_filename = "cabinet_demo.csv";
        fdemo = fopen("cabinet_demo.csv", "w");
        if (!fdemo) {
            ROS_ERROR("Failed to open '%s' for writing", demo_filename);
            return 1;
        }

        // write group variable names to the header
        for (size_t vidx = 0; vidx < demo_group->getVariableCount(); ++vidx) {
            auto& name = demo_group->getVariableNames()[vidx];
            fprintf(fdemo, "%s", name.c_str());
            if (record_object) {
                fprintf(fdemo, ",");
            } else {
                if (vidx != demo_group->getVariableCount() - 1) {
                    fprintf(fdemo, ",");
                } else {
                    fprintf(fdemo, "\n");
                }
            }
        }
        if (record_object) {
            fprintf(fdemo, "hinge\n");
        }
    }
    // close the demonstration file when we kill the program
    DEFER(if (record && fdemo != NULL) {
            fclose(fdemo);
            fdemo = NULL;
    });

    /////////////////////////////////////
    // define the model of the cabinet //
    /////////////////////////////////////

    CabinetModel cabinet;
    cabinet.width = 0.50;
    cabinet.height = 0.80;
    cabinet.depth = 0.50;
    cabinet.thickness = 0.02;
#if RIGHT_ARM
    cabinet.handle_offset_y = -0.4 * cabinet.width;
#else
    cabinet.handle_offset_y = 0.4 * cabinet.width;
#endif
    cabinet.handle_offset_x = 0.08;
    cabinet.handle_height = 0.20;
    cabinet.handle_radius = 0.01;

    Eigen::Affine3d cabinet_pose;
    switch (cabinet_id) {
    case 0:
        cabinet_pose =
                Eigen::Translation3d(2.0, 0.0, 0.5 * cabinet.height) *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        break;
    case 1:
        cabinet_pose =
                Eigen::Translation3d(2.0, -1.0, 0.5 * cabinet.height) *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        break;
    case 2:
        cabinet_pose =
                Eigen::Translation3d(1.0, 2.0, 0.5 * cabinet.height) *
                Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
        break;
    default:
        return 1;
    }

    auto prev_door_pos = 0.0;
    auto door_pos = 0.0;
    auto t = 0.0;

    auto tip_link = "limb_right_tool0";

    auto contact_error_z = 0.5 * cabinet.handle_height;
    auto contact_error = 0.03;
    bool contacted = false; // ever came into contact?
    bool contact = false;

    auto IsStateValid = [&](
        moveit_msgs::GetStateValidity::Request& req,
        moveit_msgs::GetStateValidity::Response& res)
        -> bool
    {
        if (contacted) {
            bool result = check_state_service.call(req, res);
            if (result) {
                res.valid = res.valid && contact;
            }
            return result;
        } else {
            return check_state_service.call(req, res);
        }
        return true; //check_state_service.call(req, res);
    };

    boost::function<bool(moveit_msgs::GetStateValidity::Request& req, moveit_msgs::GetStateValidity::Response& res)> service_fun;
    service_fun = IsStateValid;

    ros::ServiceServer check_state_server = nh.advertiseService(
            "check_state_validity_manipulation", service_fun);

    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();

//        door_pos = 0.5 * sin(t) + 0.5;

        if (door_pos != prev_door_pos) {
            ROS_INFO("door position = %f", door_pos);
            prev_door_pos = door_pos;
        }

        bool robot_moved = g_state_received;
        if (robot_moved && record) {
            assert(g_curr_robot_state && "State shouldn't be null if we received it");
            // TODO we should probably just always do this when a state is received to avoid
            // doing it over and over again
            moveit::core::robotStateMsgToRobotState(*g_curr_robot_state, robot_state);

            std::vector<double> group_state;
            robot_state.copyJointGroupPositions(demo_group, group_state);

            for (size_t vidx = 0; vidx < demo_group->getVariableCount(); ++vidx) {
                auto pos = group_state[vidx];
                fprintf(fdemo, "%f", pos);
                if (record_object) {
                    fprintf(fdemo, ",");
                } else {
                    if (vidx != demo_group->getVariableCount() - 1) {
                        fprintf(fdemo, ",");
                    } else {
                        fprintf(fdemo, "\n");
                    }
                }
            }
            if (record_object) {
                fprintf(fdemo, "%f\n", door_pos);
            }
        }
        g_state_received = false;

        //////////////////////////////////////////////////////////
        // check for contact with the door in the current state //
        //////////////////////////////////////////////////////////

        auto handle_pose = GetHandlePose(&cabinet, &cabinet_pose, door_pos);
        ROS_DEBUG("  handle @ (%f, %f, %f)",
                handle_pose.translation().x(),
                handle_pose.translation().y(),
                handle_pose.translation().z());

        contact = false;
        Eigen::Affine3d curr_tool_pose;
        if (g_curr_robot_state) {
            moveit::core::robotStateMsgToRobotState(*g_curr_robot_state, robot_state);
            curr_tool_pose = robot_state.getGlobalLinkTransform(tip_link);
            ROS_DEBUG("  curr tool @ (%f, %f, %f)",
                    curr_tool_pose.translation().x(),
                    curr_tool_pose.translation().y(),
                    curr_tool_pose.translation().z());

            Eigen::Vector3d dp = curr_tool_pose.translation() - handle_pose.translation();
            if (std::fabs(dp.z()) < contact_error_z &&
                dp.x() * dp.x() + dp.y() * dp.y() < contact_error * contact_error)
            {
                contact = true;
                contacted = true; // never reset to false
            }
        }

        ///////////////////////////////////////////////////////
        // update door pose to follow the robot's tool frame //
        ///////////////////////////////////////////////////////

        if (contact) {
            auto hinge_frame =
                    GetHingeFrame(&cabinet, &cabinet_pose) *
                    Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());

            Eigen::Affine3d tool_in_hinge = hinge_frame.inverse() * curr_tool_pose;

            ROS_INFO("tool position (hinge frame) = (%f, %f, %f)",
                    tool_in_hinge.translation().x(),
                    tool_in_hinge.translation().y(),
                    tool_in_hinge.translation().z());

            auto radius = GetHandleRotationRadius(&cabinet);

            Eigen::Vector3d nearest = radius * tool_in_hinge.translation().normalized();

#if RIGHT_ARM
            ROS_INFO("  theta = %f", atan2(nearest.y(), nearest.x()));
            auto theta = atan2(nearest.y(), nearest.x()) - GetHandleRotationOffset(&cabinet);
            ROS_INFO("   theta adjusted = %f", theta);
            if (theta < door_min_pos) theta = door_min_pos;
#else
            auto theta = atan2(nearest.y(), nearest.x()) + GetHandleRotationOffset(&cabinet);
#endif

            if (theta < door_min_pos || theta > door_max_pos) {
                auto new_theta = theta;
                new_theta = std::max(new_theta, door_min_pos);
                new_theta = std::min(new_theta, door_max_pos);
                ROS_INFO("Clamped door position %f -> %f", theta, new_theta);
                theta = new_theta;
            }

#if RIGHT_ARM
            door_pos = (theta - door_min_pos) / (door_max_pos - door_min_pos);
#else
            door_pos = 1.0 - (door_min_pos - theta) / door_min_pos;
#endif

            // get the position of the tool in the hinge frame
        }

        auto markers = MakeCabinetMarkers(&cabinet, door_pos, &cabinet_pose, "cabinet", contact);
        for (auto& marker : markers.markers) {
            marker.id += cabinet_id * markers.markers.size();
        }

        marker_pub.publish(markers);
        loop_rate.sleep();
        t += loop_rate.expectedCycleTime().toSec();
    }

    return 0;
}
