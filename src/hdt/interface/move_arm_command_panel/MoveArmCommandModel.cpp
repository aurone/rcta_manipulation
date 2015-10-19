#include "MoveArmCommandModel.h"

#include <stack>

#include <ros/console.h>

MoveArmCommandModel::MoveArmCommandModel(QObject* parent) :
    QObject(parent),
    m_robot_description(),
    m_rm_loader(),
    m_robot_model(),
    m_robot_state()
{
}

bool MoveArmCommandModel::loadRobot(const std::string& robot_description)
{
    if (robot_description == m_robot_description) {
        return true;
    }

    // load a new robot
    robot_model_loader::RobotModelLoaderPtr rm_loader(
            new robot_model_loader::RobotModelLoader(robot_description, true));

    if (!rm_loader->getModel()) {
        // failed to load robot from URDF
        return false;
    }

    m_robot_description = robot_description;
    m_rm_loader = rm_loader;
    m_robot_model = m_rm_loader->getModel();
    m_robot_state.reset(new moveit::core::RobotState(m_robot_model));

    m_robot_state->setToDefaultValues();
    m_robot_state->updateLinkTransforms();
    m_robot_state->updateCollisionBodyTransforms();

    logRobotModelInfo(*m_robot_model);

    Q_EMIT robotLoaded();
    return true;
}

bool MoveArmCommandModel::isRobotLoaded() const
{
    return (bool)m_robot_model.get();
}

const std::string& MoveArmCommandModel::robotDescription() const
{
    return m_robot_description;
}

moveit::core::RobotModelConstPtr MoveArmCommandModel::robotModel() const
{
    return m_robot_model;
}

moveit::core::RobotStateConstPtr MoveArmCommandModel::robotState() const
{
    return m_robot_state;
}

std::map<std::string, double>
MoveArmCommandModel::getRightArmTorques(
    double fx, double fy, double fz,
    double ta, double tb, double tc) const
{
    // T = J^T() * forces;
    std::map<std::string, double> torques;

    const moveit::core::JointModelGroup* jmg =
            m_robot_model->getJointModelGroup("right_arm");

    Eigen::MatrixXd J;
    J = m_robot_state->getJacobian(jmg, Eigen::Vector3d(0.17, 0.0, 0.0));

    ROS_DEBUG_STREAM("Jacobian = " << J.rows() << " x " << J.cols() << '\n' << J);

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d f;
    f(0) = fx;
    f(1) = fy;
    f(2) = fz;
    f(3) = ta;
    f(4) = tb;
    f(5) = tc;

    Eigen::VectorXd t = J.transpose() * f;

    ROS_INFO("Torques = (%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f)", t(0), t(1), t(2), t(3), t(4), t(5), t(6));

    const std::vector<std::string> rarm_joint_names =
    {
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint"
    };

    torques =
    {
        { rarm_joint_names[0], t(0) },
        { rarm_joint_names[1], t(1) },
        { rarm_joint_names[2], t(2) },
        { rarm_joint_names[3], t(3) },
        { rarm_joint_names[4], t(4) },
        { rarm_joint_names[5], t(5) },
        { rarm_joint_names[6], t(6) },
    };

    return torques;
}

void MoveArmCommandModel::setJointVariable(int jidx, double value)
{
    if (!isRobotLoaded()) {
        return;
    }

    if (jidx < 0 || jidx >= m_robot_model->getVariableCount()) {
        ROS_WARN("Index passed to setJointVariable out of bounds: jidx = %d, variable count = %zu", jidx, m_robot_model->getVariableCount());
        return;
    }

    if (m_robot_state->getVariablePosition(jidx) != value) {
        m_robot_state->setVariablePosition(jidx, value);
        if (m_robot_state->getVariablePosition(jidx) != value) {
            ROS_WARN("Attempt to set joint variable %d to %0.3f failed", jidx, value);
        }
        m_robot_state->updateLinkTransforms();
        m_robot_state->updateCollisionBodyTransforms();
        Q_EMIT robotStateChanged();
    }
}

void MoveArmCommandModel::logRobotModelInfo(
    const moveit::core::RobotModel& rm) const
{                                                                                                                             
    ROS_INFO("Robot Model Name: %s", rm.getName().c_str());                                                                   
    ROS_INFO("Robot Model Frame: %s", rm.getModelFrame().c_str());                                                            
    ROS_INFO("Root Link Name: %s", rm.getRootLinkName().c_str());                                                             
    ROS_INFO("Root Joint Name: %s", rm.getRootJointName().c_str());                                                           
                                                                                                                              
    ROS_INFO("--- Robot Links ---");                                                                                          
    std::stack<std::pair<int, const moveit::core::LinkModel*>> links;                                                         
    links.push(std::make_pair(0, rm.getRootLink()));                                                                          
    while (!links.empty()) {                                                                                                  
        int depth;                                                                                                            
        const moveit::core::LinkModel* lm;                                                                                    
        std::tie(depth, lm) = links.top();                                                                                    
        links.pop();                                                                                                          
                                                                                                                              
        std::string pad(depth, ' ');                                                                                          
        ROS_INFO("%s%s", pad.c_str(), lm->getName().c_str());                                                                 
                                                                                                                              
        for (const moveit::core::JointModel* jm : lm->getChildJointModels()) {                                                
            links.push(std::make_pair(depth+1, jm->getChildLinkModel()));                                                     
        }                                                                                                                     
    }                                                                                                                         
                                                                                                                              
    ROS_INFO("--- Robot Joints ---");                                                                                         
    std::stack<std::pair<int, const moveit::core::JointModel*>> joints;                                                       
    joints.push(std::make_pair(0, rm.getRootJoint()));                                                                        
    while (!joints.empty()) {                                                                                                 
        int depth;                                                                                                            
        const moveit::core::JointModel* jm;                                                                                   
        std::tie(depth, jm) = joints.top();                                                                                   
        joints.pop();                                                                                                         
                                                                                                                              
        std::string pad(depth, ' ');                                                                                          
        ROS_INFO("%s%s", pad.c_str(), jm->getName().c_str());                                                                 
                                                                                                                              
        const moveit::core::LinkModel* lm = jm->getChildLinkModel();                                                          
        for (const moveit::core::JointModel* j : lm->getChildJointModels()) {                                                 
            joints.push(std::make_pair(depth + 1, j));                                                                        
        }                                                                                                                     
    }                                                                                                                         
                                                                                                                              
    ROS_INFO("--- Robot Joint Groups ---");                                                                                   
                                                                                                                              
    const std::vector<const moveit::core::JointModelGroup*>& jmgs =                                                           
            rm.getJointModelGroups();                                                                                         
    for (const moveit::core::JointModelGroup* jmg : jmgs) {                                                                   
        ROS_INFO("Name: %s", jmg->getName().c_str());                                                                         
        ROS_INFO("  Joints:");                                                                                                
        for (const std::string& name : jmg->getJointModelNames()) {                                                           
            ROS_INFO("    %s", name.c_str());                                                                                 
        }                                                                                                                     
        ROS_INFO("  Chain: %s", jmg->isChain() ? "true" : "false");                                                           
        ROS_INFO("  Only Single-DoF Joints: %s", jmg->isSingleDOFJoints() ? "true" : "false");                                
        ROS_INFO("  End Effector: %s", jmg->isEndEffector() ? "true" : "false");                                              
    }                                                                                                                         

    ROS_INFO("--- Joint Variables ---");

    for (size_t vind = 0; vind < rm.getVariableCount(); ++vind) {
        const std::string& var_name = rm.getVariableNames()[vind];
        const auto& var_bounds = rm.getVariableBounds(var_name);
        ROS_INFO("%s: { min: %f, max: %f, vel: %f, acc: %f }", var_name.c_str(), var_bounds.min_position_, var_bounds.max_position_, var_bounds.max_velocity_, var_bounds.max_acceleration_);
    }
}
