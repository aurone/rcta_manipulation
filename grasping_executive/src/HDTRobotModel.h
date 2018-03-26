#ifndef hdt_HDTRobotModel_h
#define hdt_HDTRobotModel_h

// system includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <smpl/robot_model.h>
#include <urdf/model.h>

// project includes
#include <hdt_kinematics/RobotModel.h>

namespace hdt {

/// \brief Implements the RobotModel interface for use with sbpl planners
class HDTRobotModel :
    public virtual sbpl::motion::RobotModel,
    public virtual sbpl::motion::ForwardKinematicsInterface,
    public virtual sbpl::motion::InverseKinematicsInterface
{
public:

    HDTRobotModel();

    virtual ~HDTRobotModel();

    bool init(const std::string& robot_description);
    double computeIKscore(const std::vector<double>& ik);
    void setPlanningToKinematicsTransform(
        const Eigen::Affine3d& T_planning_kinematics);
    const Eigen::Affine3d& planningToKinematicsTransform() const;

    /// \name Reimplemented Public Functions
    ///@{
    double minPosLimit(int jidx) const override;
    double maxPosLimit(int jidx) const override;
    bool hasPosLimit(int jidx) const override;
    double velLimit(int jidx) const override;
    double accLimit(int jidx) const override;

    /// \namem Reimplemented Public Functions from Extension
    ///@{
    virtual sbpl::motion::Extension* getExtension(size_t class_code) override;
    ///@}

    bool checkJointLimits(
        const std::vector<double>& angles,
        bool verbose = false) override;

    bool computeFK(
        const std::vector<double>& angles,
        const std::string& name,
        std::vector<double>& pose) override;

    bool computePlanningLinkFK(
        const std::vector<double>& angles,
        std::vector<double>& pose) override;

    bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<double>& solution,
        sbpl::motion::ik_option::IkOption option =
                sbpl::motion::ik_option::UNRESTRICTED) override;

    bool computeIK(
        const std::vector<double>& pose,
        const std::vector<double>& start,
        std::vector<std::vector<double>>& solutions,
        sbpl::motion::ik_option::IkOption option =
                sbpl::motion::ik_option::UNRESTRICTED) override;
    ///@}

private:

    ros::NodeHandle nh_;
    ros::Publisher pub_;

    hdt::RobotModelPtr robot_model_;

    Eigen::Affine3d m_T_planning_kinematics;

    std::string m_planning_link;

    // Privatizing these methods since the OpenRAVE IK is generated only for the 7th dof

    // sorts the vector of ik solutions -- best ones should be at the end!
    struct iksortstruct
    {
        // sortstruct needs to know its containing object
        HDTRobotModel* rm_;
        bool sort_asc;
        iksortstruct(HDTRobotModel* rm, bool asc) : rm_(rm), sort_asc(asc) {};

        bool operator()(
            const std::vector<double>& ik1,
            const std::vector<double>& ik2) const
        {
            double s1 = rm_->computeIKscore(ik1);
            double s2 = rm_->computeIKscore(ik2);
            if (sort_asc) {
                return s1 < s2;
            }
            else return s1 > s2;
        }
    };

    void sortIKsolutions(std::vector<std::vector<double>>& solutions);

    void setPlanningLink(const std::string& link_name);
};

inline
void HDTRobotModel::setPlanningToKinematicsTransform(
    const Eigen::Affine3d& T_planning_kinematics)
{
    m_T_planning_kinematics = T_planning_kinematics;
}

inline
const Eigen::Affine3d& HDTRobotModel::planningToKinematicsTransform() const
{
    return m_T_planning_kinematics;
}

} // namespace hdt

#endif

