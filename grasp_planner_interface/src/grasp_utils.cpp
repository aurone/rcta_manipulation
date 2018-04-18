#include <grasp_planner_interface/grasp_utils.h>

#include <algorithm>
#include <utility>

#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/msg_utils/msg_utils.h>

namespace rcta {

void PruneGraspsByVisibility(
    std::vector<Grasp>& grasps,
    const EigenSTL::vector_Affine3d& marker_poses,
    const Eigen::Affine3d& camera_pose,
    double ang_thresh)
{
    ROS_INFO("Filter %zu grasps via visibility", grasps.size());

    // test if any marker is visible with the wrist at the grasp pose
    auto marker_visible = [&](const Grasp& grasp)
    {
        return !std::any_of(marker_poses.begin(), marker_poses.end(),
                [&](const Eigen::Affine3d& T_wrist_marker)
                {
                    Eigen::Affine3d T_camera_fid =
                            camera_pose.inverse() *
                            grasp.pose *
                            T_wrist_marker;

                    ROS_DEBUG_NAMED("grasping", "  Camera -> Marker: %s", to_string(T_camera_fid).c_str());
                    Eigen::Vector3d camera_view_axis = -Eigen::Vector3d::UnitZ();
                    Eigen::Vector3d marker_plane_normal;
                    marker_plane_normal.x() = T_camera_fid(0, 2);
                    marker_plane_normal.y() = T_camera_fid(1, 2);
                    marker_plane_normal.z() = T_camera_fid(2, 2);

                    ROS_DEBUG_NAMED("grasping", "  Marker Plane Normal [camera view frame]: %s", to_string(marker_plane_normal).c_str());

                    double dp = marker_plane_normal.dot(camera_view_axis);
                    ROS_DEBUG_NAMED("grasping", "  Marker Normal * Camera View Axis: %0.3f", dp);

                    // the optimal situation is when the camera is facing the
                    // marker directly and this angle is 0
                    double angle = acos(dp);
                    ROS_DEBUG_NAMED("grasping", "  Angle: %0.3f", angle);

                    return angle < ang_thresh;
                });
    };

    auto it = std::remove_if(grasps.begin(), grasps.end(),
            [&](const Grasp& g) { return !marker_visible(g); });

    size_t ridx = std::distance(grasps.begin(), it);

    ROS_INFO("%zu/%zu visible grasps", ridx, grasps.size());
    grasps.resize(ridx);
}

// Create a marker array consisting of frame triad markers for each grasp pose
auto GetGraspCandidatesVisualization(
    const std::vector<Grasp>& grasps,
    const std::string& frame_id,
    const std::string& ns)
    -> visualization_msgs::MarkerArray
{
    auto triad_scale = geometry_msgs::CreateVector3(0.1, 0.01, 0.01);

    // create triad markers to be reused for each grasp candidate
    auto triad_markers = msg_utils::create_triad_marker_arr(triad_scale);

    for (auto& marker : triad_markers.markers) {
        marker.header.frame_id = frame_id;
        marker.ns = ns;
    }

    visualization_msgs::MarkerArray all_markers;
    all_markers.markers.resize(grasps.size() * triad_markers.markers.size());

    // save the relative transform of each marker in the triad's frame
    EigenSTL::vector_Affine3d marker_transforms;
    marker_transforms.reserve(triad_markers.markers.size());
    for (auto& marker : triad_markers.markers) {
        Eigen::Affine3d marker_transform;
        tf::poseMsgToEigen(marker.pose, marker_transform);
        marker_transforms.push_back(marker_transform);
    }

    // transform each marker into the world frame
    for (size_t gidx = 0; gidx < grasps.size(); ++gidx) {
        auto& candidate = grasps[gidx];
        for (size_t midx = 0; midx < triad_markers.markers.size(); ++midx) {
            size_t idx = triad_markers.markers.size() * gidx + midx;

            // make a copy of the marker copy transformed into the world frame
            visualization_msgs::Marker m = triad_markers.markers[midx];
            Eigen::Affine3d T_world_marker =
                    candidate.pose *
                    marker_transforms[midx];
            tf::poseEigenToMsg(T_world_marker, m.pose);

            m.id = idx;
            all_markers.markers[idx] = std::move(m);
        }
    }

    return all_markers;
}


} // namespace rcta
