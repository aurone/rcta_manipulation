#include <hdt_description/RobotModel.h>
#include <hdt_kinematics/kinematics.h>
#include <ros/ros.h>
#include <sbpl_geometry_utils/utils.h>
#include <urdf_parser/urdf_parser.h>

#define VALIDATE_WITH_FK 1

namespace hdt
{

double ComputeJointStateL2NormSqrd(const std::vector<double>& joints1, const std::vector<double>& joints2)
{
    assert(joints1.size() == 7 && joints2.size() == 7);

    auto squared = [](double x) { return x * x; };
    double dist = 0.0;
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[0], joints2[0]));
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[1], joints2[1]));
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[2], joints2[2]));
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[3], joints2[3]));
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[4], joints2[4]));
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[5], joints2[5]));
    dist += squared(sbpl::utils::ShortestAngleDist(joints1[6], joints2[6]));
    return dist;
}

IKSolutionGenerator::IKSolutionGenerator(const std::vector<std::vector<double>>& solutions) :
    solutions_(solutions),
    csol_(-1)
{
}

bool IKSolutionGenerator::operator()(std::vector<double>& solution)
{
    csol_++;
    if (csol_ >= solution.size())
    {
        return false;
    }
    else
    {
        solution = std::move(solutions_[csol_]);
        return true;
    }
}

RobotModel::RobotModel() :
    joint_names_({ "arm_1_shoulder_twist",
                   "arm_2_shoulder_lift",
                   "arm_3_elbow_twist",
                   "arm_4_elbow_lift",
                   "arm_5_wrist_twist",
                   "arm_6_wrist_lift",
                   "arm_7_gripper_lift" }),
    min_limits_(),
    max_limits_(),
    free_angle_index_()
{
    free_angle_index_ = GetFreeParameters()[0];
}

bool RobotModel::load(const std::string& urdf_string)
{
    boost::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDF(urdf_string);
    if (!urdf) {
        ROS_ERROR("Failed to parse URDF");
        return false;
    }

    std::vector<bool> continuous;
    std::string why;
    if (!extract_joint_info(urdf, joint_names_, min_limits_, max_limits_, continuous, why)) {
        ROS_ERROR("Failed to extract joint info (%s)", why.c_str());
        return false;
    }

    boost::shared_ptr<const urdf::Joint> first_joint = urdf->getJoint("arm_1_shoulder_twist");
    if (!first_joint) {
        return false;
    }
    urdf::Pose joint_origin_pose = first_joint->parent_to_joint_origin_transform;

    const auto& joint_origin_pos = joint_origin_pose.position;
    const auto& joint_origin_rot = joint_origin_pose.rotation;

    Eigen::Translation3d joint_origin_translation(joint_origin_pos.x, joint_origin_pos.y, joint_origin_pos.z);
    Eigen::Quaterniond joint_origin_rotation(joint_origin_rot.w, joint_origin_rot.x, joint_origin_rot.y, joint_origin_rot.z);
    Eigen::Affine3d joint_origin_transform = joint_origin_translation * joint_origin_rotation;

    mount_frame_to_manipulator_frame_ = joint_origin_transform;

    ROS_INFO("Joint origin translation: (%0.3f, %0.3f, %0.3f)", joint_origin_translation.x(), joint_origin_translation.y(), joint_origin_translation.z());
    ROS_INFO("Joint origin rotation: (%0.3f, %0.3f, %0.3f, %0.3f)", joint_origin_rotation.w(), joint_origin_rotation.x(), joint_origin_rotation.y(), joint_origin_rotation.z());

    return true;
}

bool RobotModel::within_joint_limits(const std::vector<double>& joint_vals) const
{
    if (!sbpl::utils::AreJointsWithinLimits(joint_vals, min_limits_, max_limits_)) {
        std::vector<double> angles_copy = joint_vals;
        if (sbpl::utils::NormalizeAnglesIntoRange(angles_copy, min_limits_, max_limits_)) {
            ROS_WARN("Joint angles are not within limits when unnormalized");
            return true;
        }
        else {
            return false;
        }
    }
    return true;
}

bool RobotModel::compute_fk(const std::vector<double>& joint_values, Eigen::Affine3d& eef_transform_out) const
{
    if (joint_values.size() < 7) {
        ROS_WARN("Insufficient joint values given to forward kinematics");
        return false;
    }

    double trans_res[3];
    double rot_res[9];
    ComputeFk(joint_values.data(), trans_res, rot_res);

    convert(trans_res, rot_res, eef_transform_out);
    return true;
}

std::size_t RobotModel::free_angle_index() const
{
    return free_angle_index_;
}

bool RobotModel::compute_nearest_ik(
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed,
    std::vector<double>& solution_out) const
{
    if (seed.size() < 7) {
        return false;
    }

    double solver_trans[3];
    double solver_rot[9];
    convert(eef_transform, solver_trans, solver_rot);

    double free_angle = seed[free_angle_index_];

    ikfast::IkSolutionList<double> ik_solutions;
    if (!ComputeIk(solver_trans, solver_rot, &free_angle, ik_solutions)) {
        return false;
    }

    // check ik solutions and pick the closest one
    std::size_t num_valid = 0;
    int best_solution = -1;             // index of closest solution
    double best_joint_dist = -1.0;      // joint distance from seed state
    for (size_t i = 0; i < ik_solutions.GetNumSolutions(); ++i) {
        double solution[7];
        ik_solutions.GetSolution(i).GetSolution(solution, nullptr);

#if VALIDATE_WITH_FK 
        const bool close = check_ik_solution(solution, eef_transform);
#else
        const bool close = true;
#endif

        std::vector<double> vsolution(solution, solution + sizeof(solution) / sizeof(double));
        const bool within_limits = within_joint_limits(vsolution);

        if (close && within_limits) {
            // compare this solution with our current best found
            double joint_dist = ComputeJointStateL2NormSqrd(vsolution, seed);
            if (best_solution == -1 || joint_dist < best_joint_dist)
            {
                solution_out = std::move(vsolution);
                best_solution = (int)i;
                best_joint_dist = joint_dist;
            }
            ++num_valid;
        }
    }

//    if (num_valid != ik_solutions.GetNumSolutions() && ik_solutions.GetNumSolutions() != 0) {
//        ROS_WARN("%zd out of %zd solutions were invalid", ik_solutions.GetNumSolutions() - num_valid, ik_solutions.GetNumSolutions());
//    }

    return best_solution != -1;
}

bool RobotModel::search_nearest_ik(
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed,
    std::vector<double>& solution_out,
    double free_angle_search_res) const
{
    double free_angle_seed = seed[free_angle_index_];

    double free_angle_min = min_limits_[free_angle_index_];
    double free_angle_max = max_limits_[free_angle_index_];

    std::vector<double> cseed = seed;

    bool in_bounds = true;
    int num_iterations = 0;
    bool found_solution = false;
    while (in_bounds && !found_solution) {
        if (num_iterations > 0) {
            if (free_angle_search_res == 0.0) {
                break;
            }

            in_bounds = false;

            // searching over the free angle
            double up_free_angle = free_angle_seed + ((num_iterations + 1) >> 1) * free_angle_search_res;
            double down_free_angle = free_angle_seed - ((num_iterations + 1) >> 1) * free_angle_search_res;

            if (sbpl::utils::IsJointWithinLimits(up_free_angle, free_angle_min, free_angle_max)) {
                in_bounds = true;
                cseed[free_angle_index_] = up_free_angle;
                if (compute_nearest_ik(eef_transform, seed, solution_out)) {
                    found_solution = true;
                }
            }

            if (!found_solution && sbpl::utils::IsJointWithinLimits(down_free_angle, free_angle_min, free_angle_max)) {
                in_bounds = true;
                cseed[free_angle_index_] = down_free_angle;
                if (compute_nearest_ik(eef_transform, seed, solution_out)) {
                    found_solution = true;
                }
            }
        }
        else {
            // use the free angle seed
            cseed[free_angle_index_] = free_angle_seed;
            if (compute_nearest_ik(eef_transform, seed, solution_out)) {
                found_solution = true;
            }
        }

        num_iterations++;
    }

    return found_solution;
}

IKSolutionGenerator RobotModel::compute_all_ik_solutions(
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed) const
{
    if (seed.size() < 7) {
        return IKSolutionGenerator();
    }

    double solver_trans[3];
    double solver_rot[9];
    convert(eef_transform, solver_trans, solver_rot);

    double free_angle = seed[free_angle_index_];

    ikfast::IkSolutionList<double> ik_solutions;
    if (!ComputeIk(solver_trans, solver_rot, &free_angle, ik_solutions)) {
        return IKSolutionGenerator();
    }

    std::vector<std::vector<double>> solutions;
    solutions.resize(ik_solutions.GetNumSolutions());

    // check ik solutions and pick the closest one
    std::size_t num_valid = 0;
    for (size_t i = 0; i < ik_solutions.GetNumSolutions(); ++i) {
        double solution[7];
        ik_solutions.GetSolution(i).GetSolution(solution, nullptr);

#if VALIDATE_WITH_FK 
        const bool close = check_ik_solution(solution, eef_transform);
#else
        const bool close = true;
#endif

        std::vector<double> vsolution(solution, solution + sizeof(solution) / sizeof(double));
        const bool within_limits = within_joint_limits(vsolution);

        if (close && within_limits) {
            solutions.push_back(std::move(vsolution));
            ++num_valid;
        }
    }

//    if (!solutions.empty() && num_valid != solutions.size()) {
//        ROS_WARN("%zd out of %zd solutions were invalid", solutions.size() - num_valid, solutions.size());
//    }

    return IKSolutionGenerator(solutions);
}

IKSolutionGenerator RobotModel::search_all_ik_solutions(
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed,
    double free_angle_search_res) const
{
    ROS_WARN("Unimplemented");
    return IKSolutionGenerator();
}

void RobotModel::convert(const double* trans, const double* rot, Eigen::Affine3d& transform_out)
{
    transform_out.translation()(0) = trans[0];
    transform_out.translation()(1) = trans[1];
    transform_out.translation()(2) = trans[2];
    transform_out(0, 0) = rot[0];
    transform_out(0, 1) = rot[1];
    transform_out(0, 2) = rot[2];
    transform_out(1, 0) = rot[3];
    transform_out(1, 1) = rot[4];
    transform_out(1, 2) = rot[5];
    transform_out(2, 0) = rot[6];
    transform_out(2, 1) = rot[7];
    transform_out(2, 2) = rot[8];
}

void RobotModel::convert(const Eigen::Affine3d& transform, double* trans_out, double* rot_out)
{
    trans_out[0] = transform.translation()(0);
    trans_out[1] = transform.translation()(1);
    trans_out[2] = transform.translation()(2);
    rot_out[0] = transform(0, 0);
    rot_out[1] = transform(0, 1);
    rot_out[2] = transform(0, 2);
    rot_out[3] = transform(1, 0);
    rot_out[4] = transform(1, 1);
    rot_out[5] = transform(1, 2);
    rot_out[6] = transform(2, 0);
    rot_out[7] = transform(2, 1);
    rot_out[8] = transform(2, 2);
}

bool RobotModel::check_ik_solution(double solution[], const Eigen::Affine3d& eef_transform) const
{
    double trans_res[3];
    double rot_res[9];
    ComputeFk(solution, trans_res, rot_res);

    Eigen::Affine3d res_transform;
    convert(trans_res, rot_res, res_transform);

    // check here against desired transform
    Eigen::Vector3d diff_pos = eef_transform.translation() - res_transform.translation();

    Eigen::Quaterniond eef_rot(eef_transform.rotation());
    Eigen::Quaterniond res_rot(res_transform.rotation());
    Eigen::Quaterniond diff_quat(res_rot.inverse() * eef_rot);
    Eigen::AngleAxisd rotdiff(diff_quat);

    const double pos_thresh = 1e-3;
    const double pos_thresh_sqrd = pos_thresh * pos_thresh;
    const double rot_thresh = 1.0 * M_PI / 180.0;

    const bool close = diff_pos.squaredNorm() < pos_thresh_sqrd && fabs(rotdiff.angle() < rot_thresh);

    return close;
}

bool RobotModel::extract_joint_info(
    const boost::shared_ptr<urdf::ModelInterface>& urdf_model,
    const std::vector<std::string>& joints,
    std::vector<double>& min_limits_out,
    std::vector<double>& max_limits_out,
    std::vector<bool>& continuous_out,
    std::string& why) const
{
    std::vector<double> min_limits, max_limits;
    std::vector<bool> continuous;

    min_limits.reserve(joints.size());
    max_limits.reserve(joints.size());
    continuous.reserve(joints.size());

    bool success = true;
    for (const std::string& joint : joints) {
        boost::shared_ptr<const urdf::Joint> joint_model = urdf_model->getJoint(joint);

        if (!joint_model) {
            why = "No joint '" + joint + "'";
            success = false;
            break;
        }

        if (!joint_model->limits) {
            why = "No limits";
            success = false;
            break;
        }

        min_limits.push_back(joint_model->limits->lower);
        max_limits.push_back(joint_model->limits->upper);
        continuous.push_back(joint_model->type == urdf::Joint::CONTINUOUS ? true : false);
    }

    if (success) {
        min_limits_out = std::move(min_limits);
        max_limits_out = std::move(max_limits);
        continuous_out = std::move(continuous);
    }
    return success;
}

} // namespace hdt
