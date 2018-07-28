// standard includes
#include <math.h>

// system includes
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
#include <smpl/angles.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/marker_conversions.h>

// project includes
#include "cabinet_model.h"

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

    auto b_wall_pose = GetCabinetBottomGeometryPose(model, door_pos);
    b_wall_pose = (*pose) * b_wall_pose;
    tf::poseEigenToMsg(b_wall_pose, m.pose);

    tf::vectorEigenToMsg(GetCabinetBottomGeometrySize(model), m.scale);

    ma.markers.push_back(m);

    //////////////
    // top wall //
    //////////////

    m.id = 1;

    auto t_wall_pose = GetCabinetTopGeometryPose(model, door_pos);
    t_wall_pose = (*pose) * t_wall_pose;
    tf::poseEigenToMsg(t_wall_pose, m.pose);

    tf::vectorEigenToMsg(GetCabinetTopGeometrySize(model), m.scale);

    ma.markers.push_back(m);

    ///////////////
    // left wall //
    ///////////////

    m.id = 2;

    auto l_wall_pose = GetCabinetLeftGeometryPose(model, door_pos);
    l_wall_pose = (*pose) * l_wall_pose;
    tf::poseEigenToMsg(l_wall_pose, m.pose);

    tf::vectorEigenToMsg(GetCabinetLeftGeometrySize(model), m.scale);

    ma.markers.push_back(m);

    ////////////////
    // right wall //
    ////////////////

    m.id = 3;

    Eigen::Affine3d r_wall_pose;
    r_wall_pose = (*pose) * GetCabinetRightGeometryPose(model, door_pos);
    tf::poseEigenToMsg(r_wall_pose, m.pose);

    tf::vectorEigenToMsg(GetCabinetRightGeometrySize(model), m.scale);

    ma.markers.push_back(m);

    ///////////////
    // back wall //
    ///////////////

    m.id = 4;

    auto back_wall_pose = GetCabinetBackGeometryPose(model, door_pos);
    back_wall_pose = (*pose) * back_wall_pose;
    tf::poseEigenToMsg(back_wall_pose, m.pose);

    tf::vectorEigenToMsg(GetCabinetBackGeometrySize(model), m.scale);

    ma.markers.push_back(m);

    //////////
    // door //
    //////////

    Eigen::Affine3d T_cabinet_hinge = GetCabinetToHingeTransform(model);

    m.id = 5;

    auto door_pose = GetDoorGeometryPose(model, door_pos);
    door_pose = (*pose) * door_pose;
    tf::poseEigenToMsg(door_pose, m.pose);

    tf::vectorEigenToMsg(GetDoorGeometrySize(model), m.scale);

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

    auto shaft_pose = GetHandlePose(model, door_pos);
    shaft_pose = (*pose) * shaft_pose;
    tf::poseEigenToMsg(shaft_pose, m.pose);

    m.scale.x = 2.0 * model->handle_radius;
    m.scale.y = 2.0 * model->handle_radius;
    m.scale.z = model->handle_height + 4.0 * model->handle_radius;

    ma.markers.push_back(m);

    ////////////////////
    // handle support //
    ////////////////////

    m.id = 7;

    auto t_support_pose = GetHandleUpperGeometryPose(model, door_pos);
    t_support_pose = (*pose) * t_support_pose;
    tf::poseEigenToMsg(t_support_pose, m.pose);

    m.scale.x = 2.0 * model->handle_radius;
    m.scale.y = 2.0 * model->handle_radius;
    m.scale.z = (0.5 * model->thickness + model->handle_offset_x + model->handle_radius);

    ma.markers.push_back(m);

    ////////////////////
    // handle support //
    ////////////////////

    m.id = 8;

    auto b_support_pose = GetHandleLowerGeometryPose(model, door_pos);
    b_support_pose = (*pose) * b_support_pose;
    tf::poseEigenToMsg(b_support_pose, m.pose);

    m.scale.x = 2.0 * model->handle_radius;
    m.scale.y = 2.0 * model->handle_radius;
    m.scale.z = (0.5 * model->thickness + model->handle_offset_x + model->handle_radius);

    ma.markers.push_back(m);

    return ma;
}

int main(int argc, char* argv[])
{
    ////////////////
    // Parameters //
    ////////////////

    auto group_name = "right_arm_torso_base";
    auto demo_filename = "cabinet_demo.csv";
    auto tip_link = "limb_right_tool0";

    auto right_arm = true;

    CabinetModel cabinet;
    cabinet.right = right_arm;
    cabinet.width = 0.50;
    cabinet.height = 0.80;
    cabinet.depth = 0.50;
    cabinet.thickness = 0.02;
    cabinet.handle_offset_y = 0.4 * cabinet.width;
    cabinet.handle_offset_x = 0.08;
    cabinet.handle_height = 0.20;
    cabinet.handle_radius = 0.01;

    auto contact_error_z = 0.5 * cabinet.handle_height;
    auto contact_error = 0.03;

    ///////////////
    // Arguments //
    ///////////////

    ros::init(argc, argv, "door_demonstrator", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    printf("Usage: door_demonstrator [cabinet_id] [record] [record_object]\n");
    ROS_INFO("argc: %d", argc);

    auto cabinet_id = (argc > 1) ? atoi(argv[1]) : 0;
    auto record = argc > 2 ? true : false;
    auto record_object = argc > 3 ? true: false;

    ROS_INFO("Cabinet ID: %d", cabinet_id);
    ROS_INFO("Record: %s", record ? "true" : "false");
    ROS_INFO("Record Object: %s", record_object ? "true" : "false");

    ////////////////////
    // Initialization //
    ////////////////////

    // initialize cabinet pose based on id...
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

    // initialize robot model...
    robot_model_loader::RobotModelLoader loader;
    auto model = loader.getModel();

    // initialize robot state...
    moveit::core::RobotState robot_state(model);

    // initialize joint group...
    if (!model->hasJointModelGroup(group_name)) {
        ROS_ERROR("Demonstration group '%s' does not exist in the robot model", group_name);
        return 1;
    }

    auto* demo_group = model->getJointModelGroup(group_name);

    // initialize the demonstration recording...
    FILE* fdemo = NULL;
    if (record) {
        fdemo = fopen(demo_filename, "w");
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

    // close the demonstration file when we kill the program...
    DEFER(
        if (record && fdemo != NULL) {
            fclose(fdemo);
            fdemo = NULL;
        }
    );

    // state variables...
    auto prev_door_pos = GetHingeDefaultPosition(&cabinet);
    auto door_pos = GetHingeDefaultPosition(&cabinet);
//    auto door_pos = 0.5 * (GetHingeLowerLimit(&cabinet) + GetHingeUpperLimit(&cabinet));

    auto contacted = false; // ever came into contact?
    auto contact = false;   // in contact at the last frame

    auto state_received = false;
    moveit_msgs::RobotState::ConstPtr curr_robot_state;

    //////////////////////
    // Fire up the node //
    //////////////////////

    boost::function<void(const moveit_msgs::RobotState::ConstPtr&)>
    robot_state_callback = [&](const moveit_msgs::RobotState::ConstPtr& msg) {
        curr_robot_state = msg;
        state_received = true;
    };

    auto state_sub = nh.subscribe("robot_state", 1, robot_state_callback);

    // to perform usual validity checking
    auto check_state_service = nh.serviceClient<moveit_msgs::GetStateValidity>(
            "check_state_validity");

    auto marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 1);

    boost::function<bool(moveit_msgs::GetStateValidity::Request& req, moveit_msgs::GetStateValidity::Response& res)> service_fun;

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
    };

    service_fun = IsStateValid;

    auto check_state_server =
            nh.advertiseService("check_state_validity_manipulation", service_fun);

    ////////////////////////
    // demonstration loop //
    ////////////////////////

    auto t = 0.0;
    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
//        t += 1.0 / 30.0;

        // 0 to 3*pi/4
        auto amplitude = 0.5 * GetHingeSpan(&cabinet);
        auto median = 0.5 * (GetHingeLowerLimit(&cabinet) + GetHingeUpperLimit(&cabinet));

//        door_pos += (amplitude * std::cos(t)) / 30.0;

        if (door_pos != prev_door_pos) {
            ROS_INFO("door position = %f", sbpl::angles::to_degrees(door_pos));
            prev_door_pos = door_pos;
        }

        ////////////////////////////////////////////////////////////////
        // record the motion of the robot and the state of the object //
        ////////////////////////////////////////////////////////////////

        auto robot_moved = state_received;
        if (robot_moved && record) {
            assert(curr_robot_state && "State shouldn't be null if we received it");
            // TODO we should probably just always do this when a state is received to avoid
            // doing it over and over again
            moveit::core::robotStateMsgToRobotState(*curr_robot_state, robot_state);

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
        state_received = false;

        //////////////////////////////////////////////////////////
        // check for contact with the door in the current state //
        //////////////////////////////////////////////////////////

        auto handle_pose = cabinet_pose * GetHandlePose(&cabinet, door_pos);
        ROS_DEBUG("  handle @ (%f, %f, %f)",
                handle_pose.translation().x(),
                handle_pose.translation().y(),
                handle_pose.translation().z());

        contact = false;
        Eigen::Affine3d curr_tool_pose;
        if (curr_robot_state) {
            moveit::core::robotStateMsgToRobotState(*curr_robot_state, robot_state);
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

            auto markers = sbpl::visual::MakeFrameMarkers(curr_tool_pose, "map", "tool");
            visualization_msgs::MarkerArray ma;
            for (auto& marker : markers) {
                visualization_msgs::Marker m;
                ConvertMarkerToMarkerMsg(marker, m);
                ma.markers.push_back(std::move(m));
            }
            marker_pub.publish(ma);
        }

        ///////////////////////////////////////////////////////
        // update door pose to follow the robot's tool frame //
        ///////////////////////////////////////////////////////

        if (true || contact) {
            // T_world_hinge, rotated so x is to the cabinet's left and y is to
            // the cabinet's back
            Eigen::Affine3d hinge_frame =
                    cabinet_pose *              // T_world_cabinet
                    GetHingeOrigin(&cabinet);// *  // T_cabinet_hinge
//                    Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());

            // T_hinge_world * T_world_tool
            Eigen::Affine3d tool_in_hinge = hinge_frame.inverse() * curr_tool_pose;

            ROS_INFO("tool position (hinge frame) = (%f, %f, %f)",
                    tool_in_hinge.translation().x(),
                    tool_in_hinge.translation().y(),
                    tool_in_hinge.translation().z());

            auto radius = GetHandleRotationRadius(&cabinet);

            // closest point on the circle mapped by the handle radius
            Eigen::Vector3d nearest = radius * tool_in_hinge.translation().normalized();

            auto ccw_dist = [](double ai, double af)
            {
                auto diff = sbpl::angles::shortest_angle_diff(af, ai);
                if (diff >= 0.0) {
                    return diff;
                } else {
                    return 2.0 * M_PI - std::fabs(diff);
                }
            };

            double theta;
            if (right_arm) {
                ROS_INFO("  theta = %f", atan2(nearest.y(), nearest.x()));
                theta = atan2(nearest.y(), nearest.x());// - GetHandleRotationOffset(&cabinet);

                // clamp to the boundaries for the hinge, theta is now the angle
                // the handle
                theta = std::max(theta, -0.5 * M_PI + GetHandleRotationOffset(&cabinet));
                theta = std::min(theta, 0.25 * M_PI + GetHandleRotationOffset(&cabinet));

                theta -= GetHandleRotationOffset(&cabinet);
                theta += 0.5 * M_PI;

                ROS_INFO("  theta adjusted = %f", theta);
            } else {
                theta = atan2(nearest.y(), nearest.x()) + GetHandleRotationOffset(&cabinet);
            }

            door_pos = theta;
        }

        /////////////////////////////////////////////
        // visualize the cabinet and contact state //
        /////////////////////////////////////////////

        auto markers = MakeCabinetMarkers(&cabinet, door_pos, &cabinet_pose, "cabinet", contact);
        for (auto& marker : markers.markers) {
            marker.id += cabinet_id * markers.markers.size();
        }

        marker_pub.publish(markers);
        loop_rate.sleep();
    }

    return 0;
}

