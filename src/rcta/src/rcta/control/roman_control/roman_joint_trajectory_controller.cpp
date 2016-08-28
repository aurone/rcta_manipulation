// system includes
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <robot/robot_pub_types.h>
#include <plan/plan_types.h>
#include <plan/plan_utils.h>
#include <roman_client_ros_utils/RomanSpec.h>
#include <roman_client_ros_utils/RomanSpecReply.h>
#include <roman_client_ros_utils/RomanState.h>
#include <ros/ros.h>

// project includes
#include "roman_joint_trajectory_controller.h"

// translate roman_spec_t to RomanSpec
void translate_jplspec_to_rosspec(roman_spec_t& spec, roman_client_ros_utils::RomanSpec& spec_rosmsg)
{
    spec_rosmsg.utime = ros::Time::now().toNSec()*1e3;
    spec_rosmsg.num_mechanisms = ROBOT_NUM_MECHS;
    for(int m=0;m<ROBOT_NUM_MECHS;m++)
        spec_rosmsg.valid.push_back(spec.valid[m]);
    spec_rosmsg.num_waypoints  = spec.num_waypoints;
    for(int w=0;w<spec_rosmsg.num_waypoints;w++)
    {
      roman_client_ros_utils::RomanWaypoint wpt;
      wpt.utime = spec.waypoints[w].timestamp;
      wpt.num_joints = ROBOT_NUM_JOINTS;
      for(int j=0;j<ROBOT_NUM_JOINTS;j++)
        wpt.positions.push_back(spec.waypoints[w].positions[j]);
      wpt.world2robot.position.x    = spec.waypoints[w].world2robot.pos.x;
      wpt.world2robot.position.y    = spec.waypoints[w].world2robot.pos.y;
      wpt.world2robot.position.z    = spec.waypoints[w].world2robot.pos.z;
      wpt.world2robot.orientation.w = spec.waypoints[w].world2robot.rot.u;
      wpt.world2robot.orientation.x = spec.waypoints[w].world2robot.rot.x;
      wpt.world2robot.orientation.y = spec.waypoints[w].world2robot.rot.y;
      wpt.world2robot.orientation.z = spec.waypoints[w].world2robot.rot.z;
      spec_rosmsg.waypoints.push_back(wpt);
    }

   return;
}


void translate_rosspec_to_jplspec(roman_client_ros_utils::RomanSpec& spec_rosmsg,roman_spec_t& jplspec)
{

  memcpy(jplspec.valid,&spec_rosmsg.valid[0],sizeof(int)*ROBOT_NUM_MECHS);
  jplspec.num_waypoints =  spec_rosmsg.num_waypoints;
  for(int w=0;w<jplspec.num_waypoints;w++)
  {
    jplspec.waypoints[w].timestamp= spec_rosmsg.waypoints[w].utime;
    memcpy(jplspec.waypoints[w].positions,&spec_rosmsg.waypoints[w].positions[0],sizeof(int)*ROBOT_NUM_JOINTS);
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
  double  accel = 1.5;
  roman_spec_t spec;
  translate_rosspec_to_jplspec(spec_rosmsg,spec);
  plan_utils_update_times_roman(&spec, max_vel, accel);// updates utime according to a joint level trapezoidal profiling scheme
  translate_jplspec_to_rosspec(spec, spec_rosmsg);
}

class RomanJointTrajectoryController
{
public:

    RomanJointTrajectoryController() :
        m_nh(),
        m_server(
            ros::NodeHandle("right_limb"), "follow_joint_trajectory", false),
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

        m_joint_name_to_spec_index = {
            { "limb_right_joint1", 0 },
            { "limb_right_joint2", 1 },
            { "limb_right_joint3", 2 },
            { "limb_right_joint4", 3 },
            { "limb_right_joint5", 4 },
            { "limb_right_joint6", 5 },
            { "limb_right_joint7", 6 },
            { "limb_left_joint1", 7 },
            { "limb_left_joint2", 8 },
            { "limb_left_joint3", 9 },
            { "limb_left_joint4", 10 },
            { "limb_left_joint5", 11 },
            { "limb_left_joint6", 12 },
            { "limb_left_joint7", 13 },
            { "torso_joint1", 14 },
            { "track_left_joint ", 15 },
            { "track_right_joint", 16 }
        };
    }

    ~RomanJointTrajectoryController()
    {
        if (m_server.isActive()) {
            m_server.setAborted();
        }
        m_server.shutdown();
    }

    int run()
    {
        m_server.start();
        ros::spin();
        return 0;
    }

private:

    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
    FollowJointTrajectoryActionServer;

    ros::NodeHandle m_nh;

    FollowJointTrajectoryActionServer m_server;

    ros::Publisher m_roman_spec_pub;
    ros::Subscriber m_roman_spec_reply_sub;
    ros::Subscriber m_state_sub;

    FollowJointTrajectoryActionServer::GoalConstPtr m_goal;
    FollowJointTrajectoryActionServer::Feedback m_feedback;
    FollowJointTrajectoryActionServer::Result m_result;

    roman_client_ros_utils::RomanState::ConstPtr m_state;

    std::map<std::string, int> m_joint_name_to_spec_index;

    void romanStateCallback(const roman_client_ros_utils::RomanState::ConstPtr& msg)
    {
        m_state = msg;
    }

    void goalCallback()
    {
        ROS_INFO("This is probably where i would publish the spec");
        m_goal = m_server.acceptNewGoal();

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
        spec_update_times(path_msg);
        m_roman_spec_pub.publish(path_msg);
    }

    void preemptCallback()
    {
        ROS_INFO("Preempt action goal");
        // TODO: send halt message to the arm
        m_server.setPreempted();
    }

    void romanSpecReplyCallback(
        const roman_client_ros_utils::RomanSpecReply::ConstPtr& msg)
    {
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
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roman_joint_trajectory_controller");
    return RomanJointTrajectoryController().run();
}
