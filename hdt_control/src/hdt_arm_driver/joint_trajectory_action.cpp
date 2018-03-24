// standard includes
#include <cmath>

// system includes
#include <Eigen/Dense>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <ros/ros.h>
#include <spellbook/stringifier/stringifier.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <hdt_kinematics/RobotModel.h>

namespace hdt {

class JointTrajectoryExecutor
{
private:

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
    typedef JTAS::GoalHandle GoalHandle;

public:

    JointTrajectoryExecutor(ros::NodeHandle& n);
    ~JointTrajectoryExecutor();

    bool initialize();

private:

    /// @brief Return whether two sets (represented as vectors) are equal.
    static bool sets_equal(const std::vector<std::string> &a, const std::vector<std::string>& b);

    // void watchdog(const ros::TimerEvent &e);

    void goal_callback(GoalHandle gh);
    void cancel_callback(GoalHandle gh);

    ros::NodeHandle node_;
    ros::NodeHandle ph_;
    std::string action_server_name_;
    JTAS action_server_;
    ros::Publisher pub_controller_command_;
    ros::Subscriber sub_controller_state_;
    // ros::Timer watchdog_timer_;

    bool has_active_goal_;
    GoalHandle active_goal_;
    trajectory_msgs::JointTrajectory current_traj_;
    int current_segment_;

    std::vector<std::string> joint_names_;
    std::map<std::string, double> goal_constraints_;
    std::map<std::string, double> trajectory_constraints_;

    double end_effector_goal_tolerance_;
    double end_effector_path_tolerance_;

    double goal_time_constraint_;
    double stopped_velocity_tolerance_;

    control_msgs::FollowJointTrajectoryFeedback::ConstPtr last_controller_state_;

    hdt::RobotModelPtr robot_model_;

    static const double DEFAULT_GOAL_THRESHOLD;
    static const double DEFAULT_END_EFFECTOR_GOAL_TOLERANCE;
    static const double DEFAULT_END_EFFECTOR_PATH_TOLERANCE;

    bool read_constraints();

    bool ready() const;

    bool within_goal_constraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& msg,
                                 const std::map<std::string, double>& constraints,
                                 const trajectory_msgs::JointTrajectory& traj) const;

    bool within_goal_ee_constraints(const Eigen::Affine3d& current, const Eigen::Affine3d& target) const;

    void controller_state_callback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);

    bool set_new_goal(GoalHandle handle);
    void cancel_curr_goal();
    void abort_curr_goal();
    void complete_curr_goal();
    void clear_active_goal();

    bool active_goal() const;
    bool valid_segment() const;
    bool advance();
    int num_segments() const;

    bool send_command();
    void send_stop_command();

    trajectory_msgs::JointTrajectory create_empty_command() const;

    /// @brief Return the index of a joint or -1 if it is not found
    std::string to_string(const trajectory_msgs::JointTrajectoryPoint& traj_point) const;
    int find_joint_index(const std::string& joint_name, const trajectory_msgs::JointTrajectory& joint_traj) const;
};

const double JointTrajectoryExecutor::DEFAULT_GOAL_THRESHOLD = 1.0 * M_PI / 180.0;
const double JointTrajectoryExecutor::DEFAULT_END_EFFECTOR_GOAL_TOLERANCE = 0.04;
const double JointTrajectoryExecutor::DEFAULT_END_EFFECTOR_PATH_TOLERANCE = 0.04;

JointTrajectoryExecutor::JointTrajectoryExecutor(ros::NodeHandle &n) :
    node_(n),
    ph_("~"),
    action_server_name_("arm_controller/joint_trajectory_action"),
    action_server_(node_,
                   action_server_name_,
                   boost::bind(&JointTrajectoryExecutor::goal_callback, this, _1),
                   boost::bind(&JointTrajectoryExecutor::goal_callback, this, _1),
                   false),
    has_active_goal_(false),
    robot_model_()
{
}

bool JointTrajectoryExecutor::initialize()
{
    using namespace XmlRpc;

    ROS_INFO("Starting action server on topic %s", action_server_name_.c_str());

    std::string urdf_string;
    if (!node_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    if (!(robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string))) {
        ROS_ERROR("Failed to load Robot Model");
        return false;
    }

    joint_names_ = robot_model_->joint_names();

    current_segment_ = 0;

    if (!read_constraints()) {
        ROS_ERROR("Failed to read in constraints");
        return false;
    }

    ROS_INFO("Setting joint commands on topic command");
    pub_controller_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    sub_controller_state_ = node_.subscribe("feedback_states", 1, &JointTrajectoryExecutor::controller_state_callback, this);

    ROS_INFO("Action server started");

    action_server_.start();

    return true;
}

bool JointTrajectoryExecutor::read_constraints()
{
    // read in goal time constraints
    ph_.param("constraints/goal_time", goal_time_constraint_, 0.0);

    auto to_rads = [](double degs) { return degs * M_PI / 180.0; };

    // Gets the goal and trajectory constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        std::string ns = std::string("constraints/") + joint_names_[i];

        // set goal constraints
        double g;
        ph_.param(ns + "/goal_degs", g, DEFAULT_GOAL_THRESHOLD);
        goal_constraints_[joint_names_[i]] = to_rads(g);

        // set trajectory constraints
        double t;
        ph_.param(ns + "/trajectory", t, 1.0);
        trajectory_constraints_[joint_names_[i]] = to_rads(t);
    }

    // get stopped velocity tolerance constraint
    ph_.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);

    auto rads_to_degs = [](double rad) { return rad * 180.0 / M_PI; };

    ROS_WARN("IGNORED: Goal Time Constraint: %0.3f s", goal_time_constraint_);

    ROS_INFO("Goal Constraints:");
    for (const auto& entry : goal_constraints_) {
        ROS_INFO("    %s: %0.3f rads (%0.3f degs)", entry.first.c_str(), entry.second, rads_to_degs(entry.second));
    }

    ROS_WARN("IGNORED: Trajectory Constraints:");
    for (const auto& entry : trajectory_constraints_) {
        ROS_WARN("    %s: %0.3f rads (%0.3f degs)", entry.first.c_str(), entry.second, rads_to_degs(entry.second));
    }

    ROS_WARN("IGNORED: Stopped Velocity Constraint: %0.3f m/s", stopped_velocity_tolerance_);

    ph_.param("constraints/end_effector/trajectory_m", end_effector_path_tolerance_, DEFAULT_END_EFFECTOR_PATH_TOLERANCE);
    ph_.param("constraints/end_effector/goal_m", end_effector_goal_tolerance_, DEFAULT_END_EFFECTOR_GOAL_TOLERANCE);

    ROS_INFO("End Effector Goal Tolerance: %0.6f", end_effector_goal_tolerance_);
    ROS_WARN("End Effector Path Tolerance: %0.6f", end_effector_path_tolerance_);

    return true;
}

bool JointTrajectoryExecutor::ready() const
{
    return (bool)last_controller_state_;
}

JointTrajectoryExecutor::~JointTrajectoryExecutor()
{
    if (active_goal()) {
        abort_curr_goal();
    }

    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
}

static std::vector<double> ToDegrees(const std::vector<double>& angles_rad)
{
    std::vector<double> degrees(angles_rad.size());
    for (std::size_t i = 0; i < degrees.size(); ++i) {
        degrees[i] = angles_rad[i];
    }
    return degrees;
}

bool JointTrajectoryExecutor::within_goal_constraints(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr& msg,
    const std::map<std::string, double>& constraints,
    const trajectory_msgs::JointTrajectory& traj) const
{
    // calculate the errors between the msg positions and the end of the current segment on the trajectory
    int last = current_segment_;
    std::vector<std::pair<double, double>> errors(msg->joint_names.size());
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
        errors[i].first = fabs(msg->actual.positions[i] - traj.points[last].positions[i]);
        errors[i].second = constraints.at(msg->joint_names[i]);
    }

    // compute whether joint goal constraints are satisfied
    bool joints_outside = std::any_of(errors.begin(), errors.end(), [](const std::pair<double, double>& constraint) {
            return (constraint.second >= 0) && (constraint.first > constraint.second);
    });

    Eigen::Affine3d actual_ee_pose;
    Eigen::Affine3d target_ee_pose;
    if (!robot_model_->compute_fk(msg->actual.positions, actual_ee_pose)) {
        ROS_WARN("Failed to compute forward kinematics for current end effector pose");
        return false;
    }

    if (!robot_model_->compute_fk(traj.points[last].positions, target_ee_pose)) {
        ROS_WARN("Failed to compute forward kinematics for target end effector pose");
        return false;
    }

    bool ee_outside = !within_goal_ee_constraints(actual_ee_pose, target_ee_pose);

    // print out the status of the goal constraints
    ROS_INFO("Current Goal Constraint Status:");
    ROS_INFO("    Joint Status: %s", joints_outside ? "OUTSIDE" : "WITHIN");
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
        ROS_INFO("    %s: %0.6f (%0.6f)", msg->joint_names[i].c_str(), 180.0 * errors[i].first / M_PI, 180.0 * errors[i].second / M_PI);
    }
    ROS_INFO("    End Effector Status: %s", ee_outside ? "OUTSIDE" : "WITHIN");

    ROS_INFO("    Actual Joint Positions: %s", ::to_string(ToDegrees(msg->actual.positions)).c_str());
    ROS_INFO("    Target Joint Positions: %s", ::to_string(ToDegrees(traj.points[last].positions)).c_str());
    ROS_INFO("    Actual EE Pose: %s", ::to_string(actual_ee_pose).c_str());
    ROS_INFO("    Target EE Pose: %s", ::to_string(target_ee_pose).c_str());

    return !(joints_outside || ee_outside);
}

bool JointTrajectoryExecutor::within_goal_ee_constraints(const Eigen::Affine3d& current, const Eigen::Affine3d& target) const
{
    const Eigen::Vector3d current_translation(current.translation());
    const Eigen::Vector3d target_translation(target.translation());
    Eigen::Vector3d diff = Eigen::Vector3d::Zero(); //target_translation - current_translation; // FIXME: this is bad and you should feel bad
    return fabs(diff.x()) <= fabs(end_effector_goal_tolerance_) &&
           fabs(diff.y()) <= fabs(end_effector_goal_tolerance_) &&
           fabs(diff.z()) <= fabs(end_effector_goal_tolerance_);
}

bool JointTrajectoryExecutor::sets_equal(const std::vector<std::string>& a, const std::vector<std::string>& b)
{
    if (a.size() != b.size()) {
        return false;
    }

    for (size_t i = 0; i < a.size(); ++i) {
        if (count(b.begin(), b.end(), a[i]) != 1) {
            return false;
        }
    }

    for (size_t i = 0; i < b.size(); ++i) {
        if (count(a.begin(), a.end(), b[i]) != 1) {
            return false;
        }
    }

    return true;
}

void JointTrajectoryExecutor::goal_callback(GoalHandle gh)
{
    // TODO: reject goal if no controller or robot feedback

    // Ensures that the joints in the goal match the joints we are commanding.
    ROS_INFO("Received goal with %zd points", gh.getGoal()->trajectory.points.size());

    if (!sets_equal(joint_names_, gh.getGoal()->trajectory.joint_names)) {
        ROS_ERROR("Joints on incoming goal don't match our joints");
        gh.setRejected();
        return;
    }

    if (active_goal()) {
        ROS_INFO("Received new goal, canceling current goal");
        cancel_curr_goal();
    }

    if (!set_new_goal(gh)) {
        gh.setRejected();
        return;
    }

    gh.setAccepted();

    if (!ready()) {
        ROS_WARN("Accepted goal but still waiting on subsystems");
        return;
    }

    bool finished = advance();
    if (!finished) {
        if (!send_command()) {
            ROS_ERROR("Should probably abort the current goal since a command was unable to be constructed");
        }
    }
}

bool JointTrajectoryExecutor::set_new_goal(GoalHandle handle)
{
    if (handle.getGoal()->trajectory.points.size() < 2) {
        return false;
    }

    active_goal_ = handle;
    has_active_goal_ = true;
    current_traj_ = active_goal_.getGoal()->trajectory;
    current_segment_ = 0;
    current_traj_.points.clear();
    current_traj_.points.push_back(active_goal_.getGoal()->trajectory.points[current_segment_ + 1]);
    return true;
}

void JointTrajectoryExecutor::cancel_curr_goal()
{
    if (!active_goal()) {
        ROS_WARN("Attempt to cancel current goal without an active goal");
        return;
    }

    ROS_INFO("Canceling current goal");
    send_stop_command();
    active_goal_.setCanceled();
    clear_active_goal();
}

void JointTrajectoryExecutor::abort_curr_goal()
{
    if (!active_goal()) {
        ROS_WARN("Attempt to abort current goal without an active goal");
        return;
    }

    ROS_INFO("Aborting current goal");
    send_stop_command();
    active_goal_.setAborted();
    clear_active_goal();
}

void JointTrajectoryExecutor::complete_curr_goal()
{
    if (active_goal()) {
        // all conditions for an active goal should be true except that the current segment index is not out of bounds
        ROS_WARN("Attempt to complete an active goal");
        return;
    }

    ROS_INFO("Completing current goal");
    send_stop_command();
    active_goal_.setSucceeded();
    clear_active_goal();
}

void JointTrajectoryExecutor::cancel_callback(GoalHandle gh)
{
    ROS_DEBUG("Received action cancel request");
    if (active_goal_ == gh) {
        cancel_curr_goal();
    }
}

bool JointTrajectoryExecutor::active_goal() const
{
    return has_active_goal_ &&
           !current_traj_.points.empty() &&
           num_segments() > 0 &&
           valid_segment();
}

void JointTrajectoryExecutor::controller_state_callback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
    if (sets_equal(joint_names_, msg->joint_names)) {
        last_controller_state_ = msg;
    }
    else {
        ROS_DEBUG("Joint names from the controller don't match our joint names");
        std::string joint_string = "[";
        for (std::vector<std::string>::iterator it = joint_names_.begin() ; it != joint_names_.end() ; it++) {
            joint_string = joint_string + " " + *it;
        }
        joint_string = joint_string + " ] vs. [ ";
        for (size_t i = 0; i < msg->joint_names.size() ; i++) {
            joint_string = joint_string + " " + msg->joint_names[i];
        }
        joint_string = joint_string + "]";
        ROS_DEBUG_STREAM(joint_string);
        return;
    }

    if (!active_goal()) {
        return;
    }

    if (!ready()) {
        ROS_WARN("Ready for action but still waiting on subsystems");
        return;
    }

    bool finished = advance();
    if (!finished) {
        if (!send_command()) {
            ROS_ERROR("Should probably abort the current goal since a command was unable to be constructed");
        }
    }
}

int JointTrajectoryExecutor::num_segments() const
{
    return active_goal_.getGoal()->trajectory.points.size() - 1;
}

bool JointTrajectoryExecutor::advance()
{
    if (!valid_segment()) {
        ROS_ERROR("Invalid current segment index");
        return true;
    }

    if (within_goal_constraints(last_controller_state_, goal_constraints_, active_goal_.getGoal()->trajectory)) {
        if (current_segment_ < num_segments()) {
            ROS_INFO("Completed segment %d. Moving on to the next segment", current_segment_);
            ++current_segment_;

            if (current_segment_ == num_segments()) {
                complete_curr_goal();
                return true;
            }
            else {
                return false;
            }
        }
        else { // current_segment_ == num_segments()

            // Additional check for motion stoppage since the controller goal may still
            // be moving.  The current robot driver calls a motion stop if it receives
            // a new trajectory while it is still moving.  If the driver is not publishing
            // the motion state (i.e. old driver), this will still work, but it warns you.
            ROS_INFO("Within goal constraints but robot is still moving");
            complete_curr_goal();
            return true;
        }
    }

    return false;
}

bool JointTrajectoryExecutor::valid_segment() const
{
    return current_segment_ >= 0 && current_segment_ < num_segments();
}

bool JointTrajectoryExecutor::send_command()
{
    if (!valid_segment()) {
        ROS_WARN("Invalid current segment index");
        return false;
    }

    current_traj_ = active_goal_.getGoal()->trajectory; // Andrew: copy over the goal trajectory to get the joint names and header information?
    current_traj_.points.clear();
    current_traj_.points.push_back(active_goal_.getGoal()->trajectory.points[current_segment_ + 1]); // push back the end point of the current segment
    ROS_INFO("Publishing command %s rads", to_string(current_traj_.points.back()).c_str());
    pub_controller_command_.publish(current_traj_);
    return true;
}

void JointTrajectoryExecutor::send_stop_command()
{
    pub_controller_command_.publish(create_empty_command());
}

int JointTrajectoryExecutor::find_joint_index(const std::string& joint_name, const trajectory_msgs::JointTrajectory& joint_traj) const
{
    for (size_t i = 0; i < joint_traj.joint_names.size(); ++i) {
        if (joint_traj.joint_names[i] == joint_name) {
            return i;
        }
    }
    return -1;
}

std::string JointTrajectoryExecutor::to_string(const trajectory_msgs::JointTrajectoryPoint& traj_point) const
{
    std::stringstream ss;
    ss << "(";
    for (size_t i = 0; i < traj_point.positions.size(); ++i) {
        ss << "a" << i << ": " << traj_point.positions[i];
        if (i != traj_point.positions.size() - 1) {
            ss << ", ";
        }
    }
    ss << ") @ (";
    for (size_t i = 0; i < traj_point.velocities.size(); ++i) {
        ss << "a" << i << ": " << traj_point.velocities[i];
        if (i != traj_point.velocities.size() - 1) {
            ss << ", ";
        }
    }
    ss << ")";
    return ss.str();
}

trajectory_msgs::JointTrajectory JointTrajectoryExecutor::create_empty_command() const
{
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    return empty;
}

void JointTrajectoryExecutor::clear_active_goal()
{
    has_active_goal_ = false;
    current_traj_.points.clear();
    current_segment_ = -1;
}

} // namespace hdt

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_action");
    ros::NodeHandle node;
    hdt::JointTrajectoryExecutor jte(node);
    if (!jte.initialize()) {
        ROS_ERROR("Failed to initialize Joint Trajectory Executor");
        ros::shutdown();
        return 1;
    }

    ros::spin();

    return 0;
}

