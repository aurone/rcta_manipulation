#ifndef hdt_RobotModel_h
#define hdt_RobotModel_h

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <urdf/model.h>

namespace hdt
{

class IKSolutionGenerator
{
public:

    IKSolutionGenerator(const std::vector<std::vector<double>>& solutions = std::vector<std::vector<double>>());

    bool operator()(std::vector<double>& solution);

private:

    std::vector<std::vector<double>> solutions_;
    int csol_;
};

double ComputeJointStateL2NormSqrd(const std::vector<double>& joints1, const std::vector<double>& joints2);

class RobotModel
{
public:

    RobotModel();

    bool load(const std::string& urdf_string);

    const std::vector<std::string>& joint_names() const { return joint_names_; }
    const std::vector<double>& min_limits() const { return min_limits_; }
    const std::vector<double>& max_limits() const { return max_limits_; }
    const std::vector<bool>& continuous() const { return continuous_; }

    bool within_joint_limits(const std::vector<double>& joint_vals) const;

    bool compute_fk(const std::vector<double>& joint_values, Eigen::Affine3d& eef_transform_out) const;

    std::size_t free_angle_index() const;
    const std::string& free_angle_joint_name() const { return joint_names_[free_angle_index_]; }

    bool compute_nearest_ik(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed,
        std::vector<double>& solution) const;

    bool search_nearest_ik(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed,
        std::vector<double>& solution,
        double free_angle_search_res = 0.0) const;

    /// @brief Return a generator that produces all ik solutions with the current free angle
    IKSolutionGenerator compute_all_ik_solutions(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed) const;

    /// @brief Return a generator that produces all ik solutions
    IKSolutionGenerator search_all_ik_solutions(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed,
        double free_angle_search_res = 0.0) const;

    const Eigen::Affine3d& mount_to_manipulator_transform() const { return mount_frame_to_manipulator_frame_; }

private:

    std::vector<std::string> joint_names_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<bool> continuous_;
    std::size_t free_angle_index_;

    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    static void convert(const double* trans, const double* rot, Eigen::Affine3d& transform_out);
    static void convert(const Eigen::Affine3d& transform, double* trans_out, double* rot_out);

    bool check_ik_solution(double solution[], const Eigen::Affine3d& eef_transform) const;

    bool extract_joint_info(
        const boost::shared_ptr<urdf::ModelInterface>& urdf,
        const std::vector<std::string>& joints,
        std::vector<double>& min_limits,
        std::vector<double>& max_limits,
        std::vector<bool>& continuous,
        std::string& why) const;
};

} // namespace hdt

#endif
