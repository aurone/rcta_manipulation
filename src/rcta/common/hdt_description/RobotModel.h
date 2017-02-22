#ifndef hdt_RobotModel_h
#define hdt_RobotModel_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <urdf/model.h>

namespace hdt {

class RobotModel;
typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;

struct IKSolutionGeneratorBase
{
    typedef std::vector<double> JointState;
};

class SimpleIKSolutionGenerator : public IKSolutionGeneratorBase
{
public:

    SimpleIKSolutionGenerator(const std::vector<JointState>& solutions = std::vector<JointState>());
    SimpleIKSolutionGenerator(std::vector<JointState>&& solutions);

    bool operator()(std::vector<double>& solution);

private:

    std::vector<std::vector<double>> curr_sols_;
    std::size_t curr_sol_idx_;
};

class IKSolutionGenerator : public IKSolutionGeneratorBase
{
public:

    IKSolutionGenerator(const std::vector<std::vector<double>>& solutions = std::vector<std::vector<double>>());
    IKSolutionGenerator(std::vector<std::vector<double>>&& solutions);

    IKSolutionGenerator(
            const RobotModelConstPtr& robot_model,
            const Eigen::Affine3d& eef_transform,
            const std::vector<double>& seed,
            double search_res);

    bool operator()(std::vector<double>& solution);

private:

    RobotModelConstPtr robot_model_;

    Eigen::Affine3d eef_transform_;

    SimpleIKSolutionGenerator curr_gen_;

    std::vector<double> seed_;
    double search_res_;

    bool inside_lower_bound_;
    bool inside_upper_bound_;
    int curr_iteration_;
    bool found_solution_;
};

double ComputeJointStateL2NormSqrd(const std::vector<double>& joints1, const std::vector<double>& joints2);

/// \brief Class providing information about the HDT structure and kinematics
class RobotModel : public std::enable_shared_from_this<RobotModel>
{
public:

    static RobotModelPtr LoadFromURDF(const std::string& urdf_string, bool enable_safety_limits = false);

    std::size_t num_joints() const { return joint_names_.size(); }
    const std::vector<std::string>& joint_names() const { return joint_names_; }
    const std::vector<double>& min_limits() const { return min_limits_; }
    const std::vector<double>& max_limits() const { return max_limits_; }
    const std::vector<double>& min_safety_limits() const { return min_safety_limits_; }
    const std::vector<double>& max_safety_limits() const { return max_safety_limits_; }
    const std::vector<double>& max_velocity_limits() const { return max_velocity_limits_; }
    const std::vector<bool>& continuous() const { return continuous_; }

    bool within_joint_limits(const std::vector<double>& joint_vals) const;
    bool within_safety_joint_limits(const std::vector<double>& joint_vals) const;

    bool compute_fk(const std::vector<double>& joint_values, Eigen::Affine3d& eef_transform_out) const;

    std::size_t free_angle_index() const;
    const std::string& free_angle_joint_name() const { return joint_names_[free_angle_index_]; }

    /// @brief Return the nearest IK solution to a given seed using the free angle specified by the seed.
    bool compute_nearest_ik(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed,
        std::vector<double>& solution) const;

    /// @brief Search over the free angle until a valid IK solution is found. If
    ///        multiple IK solutions are found, then return the nearest one.
    /// @param eef_transform The transform of the end effector in the manipulator frame
    /// @param seed The seed joint state used to compare against IK solutions;
    ///             the free angle of this joint state is used to seed the free angle search
    /// @param solution The returned solution if one is found
    /// @param free_angle_search_res The resolution, in radians, to search around the free angle
    /// @return true if a valid solution could be found; false otherwise
    bool search_nearest_ik(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed,
        std::vector<double>& solution,
        double free_angle_search_res = 0.0) const;

    /// @brief Return a generator that produces all ik solutions with the current free angle
    SimpleIKSolutionGenerator compute_all_ik_solutions(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed) const;

    /// @brief Return a generator that produces all ik solutions
    IKSolutionGenerator search_all_ik_solutions(
        const Eigen::Affine3d& eef_transform,
        const std::vector<double>& seed,
        double free_angle_search_res = 0.0) const;

    const Eigen::Affine3d& mount_to_manipulator_transform() const { return mount_frame_to_manipulator_frame_; }

private:

    RobotModel(bool enable_safety_limits);
    bool load(const std::string& urdf_string);

    std::vector<std::string> joint_names_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<double> min_safety_limits_;
    std::vector<double> max_safety_limits_;
    std::vector<double> max_velocity_limits_;
    std::vector<bool> continuous_;
    std::size_t free_angle_index_;

    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    bool safety_limits_enabled_;

    static void convert(const double* trans, const double* rot, Eigen::Affine3d& transform_out);
    static void convert(const Eigen::Affine3d& transform, double* trans_out, double* rot_out);

    bool check_ik_solution(double solution[], const Eigen::Affine3d& eef_transform) const;

    bool extract_joint_info(
        const urdf::ModelInterface& urdf,
        const std::vector<std::string>& joints,
        std::vector<double>& min_limits,
        std::vector<double>& max_limits,
        std::vector<double>& min_safety_limits,
        std::vector<double>& max_safety_limits,
        std::vector<double>& max_velocity_limits,
        std::vector<bool>& continuous,
        std::string& why) const;
};

} // namespace hdt

#endif
