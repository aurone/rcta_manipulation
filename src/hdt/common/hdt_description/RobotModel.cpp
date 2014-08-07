#include <hdt_description/RobotModel.h>

#include <cassert>
#include <utility>
#include <hdt_kinematics/kinematics.h>
#include <ros/ros.h>
#include <sbpl_geometry_utils/utils.h>
#include <urdf_parser/urdf_parser.h>

#define RM_DEBUG 0
#if RM_DEBUG
#define RM_ASSERT(cond) assert(cond)
#else
#define RM_ASSERT(cond) do { (void) sizeof((cond)); } while(0) // assert implementation borrowed from SDL
#endif

namespace hdt
{

double ComputeJointStateL2NormSqrd(const std::vector<double>& joints1, const std::vector<double>& joints2)
{
    RM_ASSERT(joints1.size() == 7 && joints2.size() == 7);

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

SimpleIKSolutionGenerator::SimpleIKSolutionGenerator(const std::vector<std::vector<double>>& solutions) :
    curr_sols_(solutions),
    curr_sol_idx_(0)
{
}

SimpleIKSolutionGenerator::SimpleIKSolutionGenerator(std::vector<std::vector<double>>&& solutions) :
    curr_sols_(solutions),
    curr_sol_idx_(0)
{
}

bool SimpleIKSolutionGenerator::operator()(std::vector<double>& solution)
{
    if (curr_sol_idx_ >= curr_sols_.size()) {
        return false;
    }
    else {
        solution = std::move(curr_sols_[curr_sol_idx_]);
        ++curr_sol_idx_;
        return true;
    }
}

IKSolutionGenerator::IKSolutionGenerator(
    const RobotModelConstPtr& robot_model,
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed,
    double search_res)
:
    robot_model_(robot_model),
    eef_transform_(eef_transform),
    curr_gen_(),
    seed_(seed),
    search_res_(search_res),
    inside_upper_bound_(true),
    inside_lower_bound_(true),
    curr_iteration_(0),
    found_solution_(false)
{
}

bool IKSolutionGenerator::operator()(std::vector<double>& solution)
{
    // TODO: minimize conversions from eigen matrix to double arrays

    // search for another solution until we search out of bounds or find a solution
    bool in_bounds = inside_lower_bound_ && inside_upper_bound_;
    while (in_bounds) {
        ROS_DEBUG("Generating solution for iteration %d", curr_iteration_);
        // on successive iterations, start searching around the free angle
        if (curr_iteration_ > 0) {
            // unless searching is disabled
            if (search_res_ == 0.0) {
                return false;
            }

            // cycle through the last batch of ik solutions
            std::vector<double> next_sol;
            if (curr_gen_(next_sol)) {
                ROS_DEBUG("Returning solution from the batch!");
                solution = std::move(next_sol);
                ++curr_iteration_;
                return true;
            }

            // advance the free angle search
            const double free_angle_seed = seed_[robot_model_->free_angle_index()];
            const double free_angle_min = robot_model_->min_limits()[robot_model_->free_angle_index()];
            const double free_angle_max = robot_model_->max_limits()[robot_model_->free_angle_index()];

            if (curr_iteration_ % 2 != 0) {
                if (inside_upper_bound_) {
                    // odd iterations search up from the canonical free angle
                    double up_free_angle = free_angle_seed + ((curr_iteration_ + 1) >> 1) * search_res_;
                    ROS_DEBUG("Attempting free angle of %0.3f", up_free_angle);
                    if (sbpl::utils::IsJointWithinLimits(up_free_angle, free_angle_min, free_angle_max)) {
                        seed_[robot_model_->free_angle_index()] = up_free_angle;
                        curr_gen_ = robot_model_->compute_all_ik_solutions(eef_transform_, seed_);
                    }
                    else {
                        ROS_DEBUG("Exited upper limit of %0.3f [%0.3f]", free_angle_max, up_free_angle);
                        inside_upper_bound_ = false;
                    }
                }
            }
            else {
                if (inside_lower_bound_) {
                    // even iterations search down from the canonical free angle
                    double down_free_angle = free_angle_seed - ((curr_iteration_ + 1) >> 1) * search_res_;
                    ROS_DEBUG("Attempting free angle of %0.3f", down_free_angle);
                    if (sbpl::utils::IsJointWithinLimits(down_free_angle, free_angle_min, free_angle_max)) {
                        seed_[robot_model_->free_angle_index()] = down_free_angle;
                        curr_gen_ = robot_model_->compute_all_ik_solutions(eef_transform_, seed_);
                    }
                    else {
                        ROS_DEBUG("Exited lower limit of %0.3f [%0.3f]", free_angle_min, down_free_angle);
                        inside_lower_bound_ = false;
                    }
                }
            }
        }
        else {
            // compute canonical free angle ik solutions if we dont have any
            curr_gen_ = robot_model_->compute_all_ik_solutions(eef_transform_, seed_);
            std::vector<double> next_sol;
            if (curr_gen_(next_sol)) {
                solution = std::move(next_sol);
                ++curr_iteration_; // don't forget this
                return true;
            }
        }

        in_bounds = inside_upper_bound_ || inside_lower_bound_;
        ++curr_iteration_;
    }

    // free angle joint limits have been breached
    return false;
}

RobotModelPtr RobotModel::LoadFromURDF(const std::string& urdf_string)
{
    RobotModelPtr robot_model_ptr(new RobotModel);
    if (robot_model_ptr) {
        if (robot_model_ptr->load(urdf_string)) {
            return robot_model_ptr;
        }
        else {
            return RobotModelPtr();
        }
    }
    return robot_model_ptr;
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
    // get a generator for all valid solutions with the specified free angle
    SimpleIKSolutionGenerator solution_generator = compute_all_ik_solutions(eef_transform, seed);

    std::vector<std::vector<double>> solutions;
    std::vector<double> solution;
    // gather all the solutions
    while (solution_generator(solution)) {
        RM_ASSERT(solution.size() == 7);
        solutions.push_back(std::move(solution));
        RM_ASSERT(solutions.back().size() == 7);
    }

    // find the best solution in terms of the l2 norm on joint angle
    int best_solution = -1;
    double best_joint_dist = -1.0;
    for (std::size_t i = 0; i < solutions.size(); ++i) {
        const std::vector<double>& sol = solutions[i];
        double joint_dist = ComputeJointStateL2NormSqrd(sol, seed);
        RM_ASSERT(joint_dist >= 0.0);
        if (best_solution == -1 || joint_dist < best_joint_dist) {
            best_solution = (int)i;
            best_joint_dist = joint_dist;
        }
    }

    if (best_solution != -1) {
        solution_out = std::move(solutions[best_solution]);
        RM_ASSERT(solution_out.size() == 7);
    }

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

SimpleIKSolutionGenerator RobotModel::compute_all_ik_solutions(
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed) const
{
    if (seed.size() < 7) {
        return SimpleIKSolutionGenerator();
    }

    double solver_trans[3];
    double solver_rot[9];
    convert(eef_transform, solver_trans, solver_rot);

    double free_angle = seed[free_angle_index_];

    ikfast::IkSolutionList<double> ik_solutions;
    if (!ComputeIk(solver_trans, solver_rot, &free_angle, ik_solutions)) {
        ROS_WARN("Failed to compute IK");
        return SimpleIKSolutionGenerator();
    }

    std::vector<std::vector<double>> solutions;
    solutions.reserve(ik_solutions.GetNumSolutions());

    // check ik solutions and pick the closest one
    for (std::size_t i = 0; i < ik_solutions.GetNumSolutions(); ++i) {
        double solution[7];
        ik_solutions.GetSolution(i).GetSolution(solution, nullptr);
        RM_ASSERT(check_ik_solution(solution, eef_transform));

        std::vector<double> vsolution(solution, solution + sizeof(solution) / sizeof(double));
        RM_ASSERT(vsolution.size() == 7);
        const bool within_limits = within_joint_limits(vsolution);

        if (within_limits) {
            solutions.push_back(std::move(vsolution));
        }
    }

    return SimpleIKSolutionGenerator(std::move(solutions));
}

IKSolutionGenerator RobotModel::search_all_ik_solutions(
    const Eigen::Affine3d& eef_transform,
    const std::vector<double>& seed,
    double search_res) const
{
    return IKSolutionGenerator(shared_from_this(), eef_transform, seed, search_res);
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
