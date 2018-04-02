// standard includes
#include <unordered_map>

// system includes
#include <class_loader/class_loader.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <eigen_conversions/eigen_msg.h>
#include <plan/plan_types.h>
#include <plan/plan_utils.h>
#include <robot/robot_pub_types.h>
#include <roman_client_ros_utils/RomanSpec.h>

namespace rcta {

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
    for(int w = 0; w<spec_rosmsg.num_waypoints; w++)
    {
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

class JPLTimeParameterizationAdapter :
    public planning_request_adapter::PlanningRequestAdapter
{
public:

    std::unordered_map<std::string, int> joint_name_to_spec_index;
    std::vector<std::string> spec_index_to_joint_name;

    JPLTimeParameterizationAdapter()
    {
        spec_index_to_joint_name = {
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

        for (size_t i = 0; i < spec_index_to_joint_name.size(); ++i) {
            joint_name_to_spec_index[spec_index_to_joint_name[i]] = i;
        }
    }

    std::string getDescription() const override
    {
        return "JPL Time Parameterization";
    }

    bool adaptAndPlan(
            const PlannerFn& planner,
            const planning_scene::PlanningSceneConstPtr& planning_scene,
            const planning_interface::MotionPlanRequest& req,
            planning_interface::MotionPlanResponse& res,
            std::vector<std::size_t>& added_path_index) const override
    {
        bool result = planner(planning_scene, req, res);
        if (!result) return false;

        ROS_INFO("JPLTimeParameterizationAdapter::adaptAndPlan");

        if (!res.trajectory_) {
            ROS_WARN("Empty trajectory");
            return true;
        }

        ROS_INFO("  group: %s", res.trajectory_->getGroupName().c_str());

        if (res.trajectory_->getWayPointCount() > roman_client_ros_utils::RomanSpec::max_waypoints) {
            ROS_ERROR("Too many points for RomanSpec");
            return false;
        }

        //////////////////////////////////////////////////
        // Convert robot_trajectory::RobotTrajectory to //
        // roman_client_ros_utils::RomanSpec            //
        //////////////////////////////////////////////////

        ROS_INFO("RobotTrajectory -> RomanSpec");

        roman_client_ros_utils::RomanSpec path_msg;
        path_msg.utime = ros::Time::now().toNSec() / 1e3;
        path_msg.num_mechanisms = ROBOT_NUM_MECHS;
        path_msg.valid.assign(ROBOT_NUM_MECHS, false);
        path_msg.valid[ROBOT_MECH_LIMB_1] = true;

        auto* group = planning_scene->getRobotModel()->getJointModelGroup(res.trajectory_->getGroupName());

        for (size_t i = 0; i < res.trajectory_->getWayPointCount(); ++i) {
            auto& wp = res.trajectory_->getWayPoint(i);
            ros::Duration time_from_start(res.trajectory_->getWaypointDurationFromStart(i));

            roman_client_ros_utils::RomanWaypoint waypoint;
            waypoint.utime = time_from_start.toNSec() / 1e3;
            waypoint.num_joints = ROBOT_NUM_JOINTS;

            std::vector<double> positions(this->joint_name_to_spec_index.size(), 0.0);

            for (size_t j = 0; j < group->getVariableCount(); ++j) {
                auto& var_name = group->getVariableNames()[j];
                int var_index = group->getVariableIndexList()[j];
                double pos = wp.getVariablePositions()[var_index];
                positions[this->joint_name_to_spec_index.at(var_name)] = pos;
            }

            waypoint.positions = std::move(positions);
            waypoint.world2robot.orientation.w = 1.0;

            path_msg.waypoints.push_back(waypoint);
        }

        ///////////////////////////
        // Do the path profiling //
        ///////////////////////////

        ROS_INFO("Profile path");

        spec_update_times(path_msg);

        //////////////////////////////////////////////////
        // convert roman_client_ros_utils::RomanSpec to //
        // robot_trajectory::RobotTrajectory            //
        //////////////////////////////////////////////////

        ROS_INFO("RomanSpec -> RobotTrajectory");

        robot_trajectory::RobotTrajectory traj_stamped(
                planning_scene->getRobotModel(), group);

        for (auto& wp : path_msg.waypoints) {
            moveit::core::RobotState state(res.trajectory_->getFirstWayPoint());
            for (size_t i = 0; i < this->spec_index_to_joint_name.size(); ++i) {
                auto& name = this->spec_index_to_joint_name[i];
                auto pos = wp.positions[i];
                state.setVariablePosition(name, pos);
            }
            Eigen::Affine3d pose;
            tf::poseMsgToEigen(wp.world2robot, pose);

            traj_stamped.addSuffixWayPoint(state, 1e3 * wp.utime);
        }

        return true;
    }

#if 0
    virtual bool adaptAndPlan(
            const PlannerFn& planner,
            const planning_scene::PlanningSceneConstPtr& planning_scene,
            const planning_interface::MotionPlanRequest& req,
            planning_interface::MotionPlanResponse& res,
            std::vector<std::size_t>& added_path_index) const = 0;
#endif
};

} // namespace rcta

CLASS_LOADER_REGISTER_CLASS(
        rcta::JPLTimeParameterizationAdapter,
        planning_request_adapter::PlanningRequestAdapter);

