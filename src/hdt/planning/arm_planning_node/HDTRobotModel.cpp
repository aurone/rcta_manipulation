#include "HDTRobotModel.h"
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <sbpl_geometry_utils/utils.h>
#include <eigen_conversions/eigen_kdl.h>
#include <leatherman/print.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

namespace hdt
{

HDTRobotModel::HDTRobotModel() :
    RobotModel(),
    robot_model_()
{
    setPlanningJoints({ "arm_1_shoulder_twist",
                        "arm_2_shoulder_lift",
                        "arm_3_elbow_twist",
                        "arm_4_elbow_lift",
                        "arm_5_wrist_twist",
                        "arm_6_wrist_lift",
                        "arm_7_gripper_lift" });
    setPlanningLink("arm_7_gripper_lift_link");
}

HDTRobotModel::~HDTRobotModel()
{
}

bool HDTRobotModel::init(const std::string& robot_description)
{
    return (bool)(robot_model_ = hdt::RobotModel::LoadFromURDF(robot_description, true));
}

bool HDTRobotModel::checkJointLimits(const std::vector<double>& angles)
{
    return robot_model_->within_safety_joint_limits(angles);
//    return robot_model_->within_joint_limits(angles);
}

bool HDTRobotModel::computePlanningLinkFK(const std::vector<double>& angles, std::vector<double>& pose)
{
    Eigen::Affine3d res_transform;
    bool res = robot_model_->compute_fk(angles, res_transform);
    if (!res) {
        ROS_WARN("Failed to compute forward kinematics");
        return false;
    }

    Eigen::Affine3d T_planning_kinematics;
    tf::transformKDLToEigen(this->T_planning_to_kinematics_, T_planning_kinematics);
    // kinematics -> eef = planning -> kinematics * mount_frame -> manipulator_frame * manipulator -> eef
    res_transform = T_planning_kinematics * robot_model_->mount_to_manipulator_transform() * res_transform;

    Eigen::Vector3d eef_pos(res_transform.translation());
    Eigen::Quaterniond eef_orient(res_transform.rotation());
    tf::Quaternion tfQuat(eef_orient.x(), eef_orient.y(), eef_orient.z(), eef_orient.w());
    tf::Matrix3x3 tfRotMat(tfQuat);
    double roll, pitch, yaw;
    tfRotMat.getEulerYPR(yaw, pitch, roll);

    pose.resize(6);
    pose[0] = eef_pos.x();
    pose[1] = eef_pos.y();
    pose[2] = eef_pos.z();
    pose[3] = roll;
    pose[4] = pitch;
    pose[5] = yaw;

    return true;
}

std::string to_string(const Eigen::Affine3d& transform)
{
    const Eigen::Vector3d translation(transform.translation());
    const Eigen::Quaterniond rotation(transform.rotation());
    std::stringstream ss;
    ss << "translation(x, y, z): (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")";
    ss << " ";
    ss << "rotation(w, x, y, z): (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")";
    return ss.str();
}

bool HDTRobotModel::computeIK(const std::vector<double>& pose, const std::vector<double>& start, std::vector<double>& solution, int option)
{
    Eigen::Affine3d eef_transform(Eigen::Affine3d::Identity());
    if (pose.size() == 6) {
        double roll = pose[3];
        double pitch = pose[4];
        double yaw = pose[5];

        eef_transform = Eigen::Translation3d(pose[0], pose[1], pose[2]) *
                        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }
    else {
        eef_transform = Eigen::Translation3d(pose[0], pose[1], pose[2]) *
                        Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);
    }

    Eigen::Affine3d T_kinematics_planning;
    tf::transformKDLToEigen(this->T_kinematics_to_planning_, T_kinematics_planning);

    // kinematics -> end effector = kinematics -> planning * planning -> end effector
    eef_transform = T_kinematics_planning * robot_model_->mount_to_manipulator_transform().inverse() * eef_transform;
    ROS_DEBUG("eef in manipulator frame: %s", to_string(eef_transform).c_str());

    bool res = robot_model_->search_nearest_ik(eef_transform, start, solution, sbpl::utils::ToRadians(1.0));
    if (res) {
        ROS_DEBUG("IK Succeeded");
    }
    else {
        ROS_WARN_PRETTY("IK Failed");
    }
    return res;
}

} // namespace hdt

