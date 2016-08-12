/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef hdt_JointTrajectoryExecutor_h
#define hdt_JointTrajectoryExecutor_h

#include <Eigen/Dense>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <rcta/common/hdt_description/RobotModel.h>

namespace hdt
{

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

} // namespace hdt

#endif
