#include "roman_joint_trajectory_controller.h"

// system includes
#include <robot/robot_pub_types.h>
#include <plan/plan_types.h>
#include <plan/plan_utils.h>
#include <roman_client_ros_utils/RomanSpec.h>

// translate roman_spec_t to RomanSpec
void translate_jplspec_to_rosspec(
    roman_spec_t& spec,
    roman_client_ros_utils::RomanSpec& spec_rosmsg)
{
    spec_rosmsg.waypoints.clear();
    spec_rosmsg.valid.clear();

    spec_rosmsg.utime = ros::Time::now().toNSec() * 1e3;
    spec_rosmsg.num_mechanisms = ROBOT_NUM_MECHS;
    for (int m = 0; m < ROBOT_NUM_MECHS; m++) {
        spec_rosmsg.valid.push_back(spec.valid[m]);
    }
    spec_rosmsg.num_waypoints  = spec.num_waypoints;
    spec_rosmsg.waypoints.resize(spec_rosmsg.num_waypoints);

    const int64_t min_wp_time = 80000; // 46000
    for (int w = 0; w<spec_rosmsg.num_waypoints; w++) {
        roman_client_ros_utils::RomanWaypoint& wpt = spec_rosmsg.waypoints[w];
        wpt.utime = (int64_t)spec.waypoints[w].timestamp;
        if (w > 0) {
            roman_client_ros_utils::RomanWaypoint& pwp = spec_rosmsg.waypoints[w - 1];
            const int64_t diff_time = wpt.utime - pwp.utime;
            if (diff_time < min_wp_time) {
                ROS_INFO("Deadbanding time");
                wpt.utime = pwp.utime + min_wp_time;

                // shift all timestamps by the amount we "added in" here
                const int64_t shift_time = wpt.utime - (int64_t)spec.waypoints[w].timestamp;
                for (int ww = w; ww < spec.num_waypoints; ++ww) {
                    spec.waypoints[w].timestamp += shift_time;
                }
            }
        }

        if (w % 10 == 0) {
            ROS_INFO("Waypoint Time (%d): %ld", w, wpt.utime);
            ROS_INFO_STREAM("Waypoint Time (" << w << "): " << wpt.utime);
        }
        wpt.num_joints = ROBOT_NUM_JOINTS;
        for (int j = 0; j < ROBOT_NUM_JOINTS; j++) {
            wpt.positions.push_back(spec.waypoints[w].positions[j]);
        }
        wpt.world2robot.position.x    = spec.waypoints[w].world2robot.pos.x;
        wpt.world2robot.position.y    = spec.waypoints[w].world2robot.pos.y;
        wpt.world2robot.position.z    = spec.waypoints[w].world2robot.pos.z;
        wpt.world2robot.orientation.w = spec.waypoints[w].world2robot.rot.u;
        wpt.world2robot.orientation.x = spec.waypoints[w].world2robot.rot.x;
        wpt.world2robot.orientation.y = spec.waypoints[w].world2robot.rot.y;
        wpt.world2robot.orientation.z = spec.waypoints[w].world2robot.rot.z;
//        spec_rosmsg.waypoints.push_back(wpt);
    }

    for (int w = 0; w<spec_rosmsg.num_waypoints; w++) {
        if (w % 10 == 0) {
            ROS_INFO("Waypoint Time (%d): %ld", w, spec_rosmsg.waypoints[w].utime);
        }
    }

    return;
}


void translate_rosspec_to_jplspec(
    roman_client_ros_utils::RomanSpec& spec_rosmsg,
    roman_spec_t& jplspec)
{
    memcpy(jplspec.valid, &spec_rosmsg.valid[0], sizeof(int) * ROBOT_NUM_MECHS);
    jplspec.num_waypoints =  spec_rosmsg.num_waypoints;
    for (int w = 0; w < jplspec.num_waypoints; w++) {
        jplspec.waypoints[w].timestamp= w * 1e5; //spec_rosmsg.waypoints[w].utime;
        if (w % 10 == 0) {
            ROS_INFO("Waypoint Time (%d): %ld", w, jplspec.waypoints[w].timestamp);
        }
        memcpy(jplspec.waypoints[w].positions, &spec_rosmsg.waypoints[w].positions[0], sizeof(double) * ROBOT_NUM_JOINTS);
        jplspec.waypoints[w].world2robot.pos.x = spec_rosmsg.waypoints[w].world2robot.position.x;
        jplspec.waypoints[w].world2robot.pos.y = spec_rosmsg.waypoints[w].world2robot.position.y;
        jplspec.waypoints[w].world2robot.pos.z = spec_rosmsg.waypoints[w].world2robot.position.z;
        jplspec.waypoints[w].world2robot.rot.u = spec_rosmsg.waypoints[w].world2robot.orientation.w;
        jplspec.waypoints[w].world2robot.rot.x = spec_rosmsg.waypoints[w].world2robot.orientation.x;
        jplspec.waypoints[w].world2robot.rot.y = spec_rosmsg.waypoints[w].world2robot.orientation.y;
        jplspec.waypoints[w].world2robot.rot.z = spec_rosmsg.waypoints[w].world2robot.orientation.z;
    }
}

void spec_update_times(roman_client_ros_utils::RomanSpec& spec_rosmsg)
{
  double max_vel = 0.75;
  double accel = 1.5;
  roman_spec_t spec;
  translate_rosspec_to_jplspec(spec_rosmsg,spec);
  // updates utime according to a joint level trapezoidal profiling scheme
  plan_utils_update_times_roman(&spec, max_vel, accel);
  translate_jplspec_to_rosspec(spec, spec_rosmsg);
}

RomanJointTrajectoryController::RomanJointTrajectoryController(
    const std::string& ns)
:
    m_nh(),
    m_ah(ns),
    m_server(
        ros::NodeHandle(ns), "follow_joint_trajectory", false),
    m_client(m_ah, "child_follow_joint_trajectory"),
    m_roman_spec_pub(),
    m_roman_spec_reply_sub()
{
    m_roman_spec_pub = m_nh.advertise<roman_client_ros_utils::RomanSpec>(
            "roman_spec", 1);
    m_roman_spec_reply_sub = m_nh.subscribe(
            "roman_spec_reply", 1,
            &RomanJointTrajectoryController::romanSpecReplyCallback, this);

    m_server.registerGoalCallback(boost::bind(&RomanJointTrajectoryController::goalCallback, this));
    m_server.registerPreemptCallback(boost::bind(&RomanJointTrajectoryController::preemptCallback, this));

    this->spec_index_to_joint_name =
    {
        { "limb_right_joint1" },
        { "limb_right_joint2" },
        { "limb_right_joint3" },
        { "limb_right_joint4" },
        { "limb_right_joint5" },
        { "limb_right_joint6" },
        { "limb_right_joint7" },
        { "limb_left_joint1" },
        { "limb_left_joint2" },
        { "limb_left_joint3" },
        { "limb_left_joint4", },
        { "limb_left_joint5", },
        { "limb_left_joint6", },
        { "limb_left_joint7", },
        { "torso_joint1", },
        { "track_left_joint ", },
        { "track_right_joint" },
    };

    for (size_t i = 0; i < this->spec_index_to_joint_name.size(); ++i) {
        this->m_joint_name_to_spec_index[this->spec_index_to_joint_name[i]] = i;
    }
}

RomanJointTrajectoryController::~RomanJointTrajectoryController()
{
    if (m_server.isActive()) {
        m_server.setAborted();
    }
    m_server.shutdown();
}

int RomanJointTrajectoryController::run()
{
    m_server.start();
    ros::spin();
    return 0;
}

void RomanJointTrajectoryController::romanStateCallback(
    const roman_client_ros_utils::RomanState::ConstPtr& msg)
{
    m_state = msg;
}

static
void ConvertRomanSpecToFollowJointTrajectory(
    RomanJointTrajectoryController* ctrl,
    const roman_client_ros_utils::RomanSpec* spec,
    control_msgs::FollowJointTrajectoryGoal* goal)
{
    goal->trajectory.joint_names = ctrl->spec_index_to_joint_name;

    goal->trajectory.points.resize(spec->num_waypoints);
    for (size_t i = 0; i < spec->waypoints.size(); ++i) {
        auto& wp_src = spec->waypoints[i];
        auto& wp_dst = goal->trajectory.points[i];

        // copy joint positions
        wp_dst.positions.resize(wp_src.num_joints);
        for (size_t j = 0; j < wp_src.num_joints; ++j) {
            wp_dst.positions[j] = wp_src.positions[j];
        }

        wp_dst.time_from_start = ros::Duration((double)wp_src.utime / 1e3);
    }
}

void RomanJointTrajectoryController::goalCallback()
{
    ROS_INFO("This is probably where i would publish the spec");
    m_goal = m_server.acceptNewGoal();

    // TODO: there's a possibility we might have to check here for whether a
    // preempt has been requested between the time the callback was serviced
    // and the goal was accepted (though this should be a very rare race condition)

    for (auto& s : m_goal->trajectory.joint_names) {
        ROS_INFO("%s", s.c_str());
    }

    if (m_goal->trajectory.points.size() > roman_client_ros_utils::RomanSpec::max_waypoints) {
        ROS_ERROR("that's too many points");
        // we could resample here or something smarter if needed
        m_result.error_code = FollowJointTrajectoryActionServer::Result::INVALID_GOAL;
        m_server.setAborted(m_result);
        return;
    }

    ////////////////////////////////////////////////
    // Convert FollowJointTrajectory to RomanSpec //
    ////////////////////////////////////////////////

    roman_client_ros_utils::RomanSpec path_msg;
    path_msg.utime = ros::Time::now().toNSec() / 1e3;
    path_msg.num_mechanisms = ROBOT_NUM_MECHS;
    for (int m = 0; m < ROBOT_NUM_MECHS; ++m) {
        path_msg.valid.push_back(false);
    }
    path_msg.valid[ROBOT_MECH_LIMB_1] = true;
    path_msg.num_waypoints = m_goal->trajectory.points.size();
    for (const trajectory_msgs::JointTrajectoryPoint& pt : m_goal->trajectory.points) {
        roman_client_ros_utils::RomanWaypoint waypoint;

        waypoint.utime = pt.time_from_start.toNSec() / 1e3;

        waypoint.num_joints = ROBOT_NUM_JOINTS;

        std::vector<double> positions(m_joint_name_to_spec_index.size(), 0.0);

        for (size_t i = 0; i < m_goal->trajectory.joint_names.size(); ++i) {
            const std::string& joint_name = m_goal->trajectory.joint_names[i];
            double pos = pt.positions[i];
            positions[m_joint_name_to_spec_index[joint_name]] = pos;
        }
        waypoint.positions = std::move(positions);

        waypoint.world2robot.orientation.w = 1.0;

        path_msg.waypoints.push_back(waypoint);
    }
//        ROS_INFO_STREAM(path_msg);

    //////////////////////
    // Do the profiling //
    //////////////////////

    spec_update_times(path_msg);

    ///////////////////////////////
    // Send to someone who cares //
    ///////////////////////////////

    const bool publish_spec = true;
    if (publish_spec) {
        m_roman_spec_pub.publish(path_msg);

        if (!path_msg.waypoints.empty()) {
            int duration_us = path_msg.waypoints.back().utime;
            double duration_s = 3.0 * (double)duration_us / 1e6; // 2 inflation
            ROS_INFO("Sleep for %d us (%0.3f seconds)", duration_us, duration_s);
            ros::Duration(duration_s).sleep();
            ROS_INFO("Done sleeping");
        }
    } else {
        control_msgs::FollowJointTrajectoryGoal traj_stamped;
        ConvertRomanSpecToFollowJointTrajectory(this, &path_msg, &traj_stamped);
        auto state = m_client.sendGoalAndWait(traj_stamped);
        ROS_INFO("Action client returned state %s", state.toString().c_str());
    }

    m_result.error_code = FollowJointTrajectoryActionServer::Result::SUCCESSFUL;
    m_server.setSucceeded(m_result);
}

void RomanJointTrajectoryController::preemptCallback()
{
    ROS_INFO("Preempt action goal");
    // TODO: send halt message to the arm
    m_server.setPreempted();
}

void RomanJointTrajectoryController::romanSpecReplyCallback(
    const roman_client_ros_utils::RomanSpecReply::ConstPtr& msg)
{
    ROS_INFO("Received spec reply");
    if (m_server.isActive()) {
        if (!msg->attempted || msg->in_fault) {
            m_result.error_code = FollowJointTrajectoryActionServer::Result::INVALID_GOAL;
            m_server.setAborted(m_result);
        }
        else {
            m_result.error_code = FollowJointTrajectoryActionServer::Result::SUCCESSFUL;
            m_server.setSucceeded(m_result);
        }
    }
}
