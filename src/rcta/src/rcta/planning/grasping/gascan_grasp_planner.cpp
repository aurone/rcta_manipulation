#include "gascan_grasp_planner.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>

namespace rcta {

visualization_msgs::MarkerArray
GetGraspCandidatesVisualization(
    const std::vector<rcta::GraspCandidate>& grasps,
    const std::string& frame_id,
    const std::string& ns)
{
    ROS_INFO("Visualizing %zd grasp candidates", grasps.size());
    geometry_msgs::Vector3 triad_scale =
            geometry_msgs::CreateVector3(0.1, 0.01, 0.01);

    // create triad markers to be reused for each grasp candidate
    auto triad_markers = msg_utils::create_triad_marker_arr(triad_scale);

    for (visualization_msgs::Marker& marker : triad_markers.markers) {
        marker.header.frame_id = frame_id;
        marker.ns = ns;
    }

    visualization_msgs::MarkerArray all_markers;
    all_markers.markers.resize(grasps.size() * triad_markers.markers.size());

    // save the relative transform of each marker in the triad's frame
    EigenSTL::vector_Affine3d marker_transforms;
    marker_transforms.reserve(triad_markers.markers.size());
    for (const auto& marker : triad_markers.markers) {
        Eigen::Affine3d marker_transform;
        tf::poseMsgToEigen(marker.pose, marker_transform);
        marker_transforms.push_back(marker_transform);
    }

    // transform each marker into the world frame
    for (size_t gidx = 0; gidx < grasps.size(); ++gidx) {
        const rcta::GraspCandidate& candidate = grasps[gidx];
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

/// Remove all grasps at which the wrist is not visible via any fiducials
///
/// The camera pose, and all grasp candidates should be in the same frame. The
/// fiducial poses are considered to be within the frame of each grasp (the link
/// for which the grasp is meant). The camera view direction is assumed to be
/// down the -z axis and the visible surface of the fiducial is assumed to be
/// orthogonal to its +z axis.
void PruneGraspsByVisibility(
    std::vector<rcta::GraspCandidate>& grasps,
    const EigenSTL::vector_Affine3d& marker_poses,
    const Eigen::Affine3d& camera_pose,
    double ang_thresh)
{
    ROS_INFO("Filter %zu grasps via visibility", grasps.size());

    // test if any marker is visible with the wrist at the grasp pose
    auto marker_visible = [&](const rcta::GraspCandidate& grasp_candidate)
    {
        return !std::any_of(marker_poses.begin(), marker_poses.end(),
                [&](const Eigen::Affine3d& T_wrist_marker)
                {
                    Eigen::Affine3d T_camera_fid =
                            camera_pose.inverse() *
                            grasp_candidate.pose *
                            T_wrist_marker;

                    ROS_DEBUG("  Camera -> Marker: %s", to_string(T_camera_fid).c_str());
                    Eigen::Vector3d camera_view_axis = -Eigen::Vector3d::UnitZ();
                    Eigen::Vector3d marker_plane_normal;
                    marker_plane_normal.x() = T_camera_fid(0, 2);
                    marker_plane_normal.y() = T_camera_fid(1, 2);
                    marker_plane_normal.z() = T_camera_fid(2, 2);

                    ROS_DEBUG("  Marker Plane Normal [camera view frame]: %s", to_string(marker_plane_normal).c_str());

                    double dp = marker_plane_normal.dot(camera_view_axis);
                    ROS_DEBUG("  Marker Normal * Camera View Axis: %0.3f", dp);

                    // the optimal situation is when the camera is facing the
                    // marker directly and this angle is 0
                    double angle = acos(dp);
                    ROS_DEBUG("  Angle: %0.3f", angle);

                    return angle < ang_thresh;
                });
    };

    auto it = std::remove_if(grasps.begin(), grasps.end(),
            [&](const rcta::GraspCandidate& g) { return !marker_visible(g); });

    size_t ridx = std::distance(grasps.begin(), it);

    ROS_INFO("%zu/%zu visible grasps", ridx, grasps.size());
    grasps.resize(ridx);
}

// Sort grasp poses so that grasps in the middle of the spline are at the front
void RankGrasps(std::vector<rcta::GraspCandidate>& grasps)
{
    const double min_u = 0.0;
    const double max_u = 1.0;

    std::sort(candidates.begin(), candidates.end(),
            [&](const rcta::GraspCandidate& a, const rcta::GraspCandidate& b) -> bool
            {
                double mid_u = 0.5 * (min_u + max_u);
                return fabs(a.u - mid_u) < fabs(b.u - mid_u);
            });
}

/// \param cp sequence of control points representing the grasp spline
/// \param degree The degree of the grasp spline
GascanGraspPlanner::GascanGraspPlanner() :
    m_initialized(false),
    m_grasp_spline(),
    m_gascan_scale(1.0),
    m_T_wrist_tool(Eigen::Affine3d::Identity()),
    m_T_grasp_pregrasp(Eigen::Affine3d::Identity())
{
}

bool GascanGraspPlanner::init(
    const std::vector<Eigen::Vector3d>& cp,
    int degree,
    double gascan_scale)
{
    if (!m_grasp_spline.initialize(cp, degree)) {
        return false;
    }

    m_gascan_scale = gascan_scale;

    m_initialized = true;
    return m_initialized;
}

bool GascanGraspPlanner::init(ros::NodeHandle& nh)
{
    // gascan scale
    double gascan_scale;
    if (!msg_utils::download_param(nh, "gas_canister_mesh_scale", gascan_scale)) {
        ROS_ERROR("Failed to download gas canister parameters");
        return false;
    }

    // degree
    int degree;
    if (!msg_utils::download_param(nh, "degree", degree)) {
        return false;
    }

    // control points
    std::vector<geometry_msgs::Point> control_points;
    if (!msg_utils::download_param(nh, "control_points", control_points)) {
        ROS_ERROR("Failed to retrieve grasp spline parameters");
        return false;
    }

    std::vector<Eigen::Vector3d> grasp_spline_points(control_points.size());
    for (std::size_t i = 0; i < control_points.size(); ++i) {
        tf::pointMsgToEigen(control_points[i], grasp_spline_points[i]);
    }

    if (!init(grasp_spline_points, degree, gascan_scale)) {
        return false;
    }

    ROS_INFO("Control Points:");
    for (const auto& cp : spline().control_points()) {
        ROS_INFO("    %s", to_string(cp).c_str());
    }

    ROS_INFO("Knot Vector: %s", to_string(spline().knots()).c_str());
    ROS_INFO("Degree: %s", std::to_string(spline().degree()).c_str());

    // load wrist -> tool transform from config

    geometry_msgs::Pose tool_pose_wrist_frame;
    if (!msg_utils::download_param(
            nh, "wrist_to_tool_transform", tool_pose_wrist_frame))
    {
        ROS_ERROR("Failed to retrieve 'wrist_to_tool_transform' from the param server");
        return false;
    }

    Eigen::Affine3d T_wrist_tool;
    tf::poseMsgToEigen(tool_pose_wrist_frame, T_wrist_tool);
    ROS_INFO("Wrist-to-Tool Transform: %s", to_string(T_wrist_tool).c_str());
    setWristToToolTransform(T_wrist_tool);

    // load pregrasp -> grasp transform transform from config

    double pregrasp_to_grasp_offset_x_m;
    if (!msg_utils::download_param(
            nh, "pregrasp_to_grasp_offset_m", pregrasp_to_grasp_offset_x_m))
    {
        ROS_ERROR("Failed to retrieve 'pregrasp_to_grasp_offset_m' from the param server");
        return false;
    }
    Eigen::Affine3d T_grasp_pregrasp(
            Eigen::Translation3d(-pregrasp_to_grasp_offset_x_m, 0, 0));
    setGraspToPregraspTransform(T_grasp_pregrasp);

    return true;
}

/// \brief Sample a set of candidate grasp poses
///
/// Grasp candidates are defined as pregrasp poses for the "wrist" of the robot.
/// The original grasp candidate is computed as follows
///
/// * A spline is specified, in the object frame, that defines possible poses to
///   place the tool frame of a gripper to successfully grasp the object
///
/// * Points are sampled along the spline, as well as the derivative of the
///   spline at that point.
///
/// * A direction for the grasp is chosen, orthogonal to the derivative at the
///   grasp point. The exact grasp direction is the direction that minimizes the
///   distance to the +z axis in the <grasp frame>
///
/// * The tool frame is chosen such that the +x axis is in line with the grasp
///   direction, the +y direction follows the spline derivative, and the +z is
///   chosen so as to create a right-hand coordinate system
///
/// * The wrist (grasp) frame is determined via a fixed offset from the tool
///   frame
///
/// * The pregrasp frame is determined via a fixed offset from the grasp frame
bool GascanGraspPlanner::sampleGrasps(
    const Eigen::Affine3d& T_grasp_object,
    int max_samples,
    std::vector<GraspCandidate>& candidates)
{
    if (!m_initialized) {
        return false;
    }

    const double min_u = 0.0;
    const double max_u = 1.0;

//    auto ipow = [](int b, int e)
//    {
//        if (e == 0) {
//            return 1;
//        } else if (e % 2 == 0) {
//            int r = ipow(b, e >> 1);
//            return r * r;
//        } else {
//            return b * ipow(b, e - 1);
//        }
//    };
//
//    int l = 0;
//    int sample_count = ipow(2, l) + 1;
//    int sidx = 0;
//    auto next_u = [&]()
//    {
//        if (sidx == sample_count) {
//            ++l;
//            sample_count = ipow(2, l) + 1;
//            sidx = 0;
//            while (l > 0 && sidx % 2 == 0 && sidx != sample_count) {
//                ++sidx;
//            }
//        }
//    };

    candidates.reserve(max_samples);
    for (int i = 0; i < max_samples; ++i) {
        ROS_DEBUG("Candidate Pregrasp %3d", i);
        // sample uniformly the position and derivative of the gas canister
        // grasp spline
        double u = (max_u - min_u) * i / (max_samples - 1);

        int knot_num = -1;
        for (int j = 0; j < m_grasp_spline.knots().size() - 1; ++j) {
            double curr_knot = m_grasp_spline.knot(j);
            double next_knot = m_grasp_spline.knot(j + 1);
            if (u >= curr_knot && u < next_knot) {
                knot_num = j;
                break;
            }
        }

        if (knot_num < m_grasp_spline.degree() ||
            knot_num >= m_grasp_spline.knots().size() - m_grasp_spline.degree())
        {
            ROS_DEBUG("Skipping grasp_spline(%0.3f) [point governed by same knot more than once]", u);
            continue;
        }

        Eigen::Vector3d object_pos_robot_frame(T_grasp_object.translation());

        Eigen::Vector3d sample_spline_point =
                Eigen::Affine3d(Eigen::Scaling(m_gascan_scale)) *
                m_grasp_spline(u);
        Eigen::Vector3d sample_spline_deriv = m_grasp_spline.deriv(u);

        ROS_DEBUG("    Sample Spline Point [object frame]: %s", to_string(sample_spline_point).c_str());
        ROS_DEBUG("    Sample Spline Deriv [object frame]: %s", to_string(sample_spline_deriv).c_str());

        Eigen::Vector3d sample_spline_point_robot_frame =
                T_grasp_object * sample_spline_point;

        Eigen::Vector3d sample_spline_deriv_robot_frame =
                T_grasp_object.rotation() * sample_spline_deriv.normalized();

        ROS_DEBUG("    Sample Spline Point [target frame]: %s", to_string(sample_spline_point_robot_frame).c_str());
        ROS_DEBUG("    Sample Spline Deriv [target frame]: %s", to_string(sample_spline_deriv_robot_frame).c_str());

        // compute the normal to the grasp spline that most points "up" in the
        // robot frame
        Eigen::Vector3d up_bias(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d up_grasp_dir =
                up_bias -
                        up_bias.dot(sample_spline_deriv_robot_frame) *
                        sample_spline_deriv_robot_frame;
        up_grasp_dir.normalize();
        up_grasp_dir *= -1.0;

        // compute the normal to the grasp spline that most points "down" in the
        // robot frame
        Eigen::Vector3d down_bias(-Eigen::Vector3d::UnitZ());
        Eigen::Vector3d down_grasp_dir =
                down_bias -
                        down_bias.dot(sample_spline_deriv_robot_frame) *
                        sample_spline_deriv_robot_frame;
        down_grasp_dir.normalize();
        down_grasp_dir *= -1.0;

        Eigen::Vector3d grasp_dir;
        Eigen::Vector3d object_to_sample =
                sample_spline_point_robot_frame - object_pos_robot_frame;
        if (up_grasp_dir.dot(object_to_sample) < 0) {
            grasp_dir = up_grasp_dir;
        } else {
            ROS_DEBUG("Skipping grasp_spline(%0.3f) [derivative goes backwards along the spline]", u);
            continue;
//            grasp_dir = down_grasp_dir;
        }

        ROS_DEBUG("    Grasp Direction [robot frame]: %s", to_string(grasp_dir).c_str());

        Eigen::Vector3d grasp_candidate_dir_x = grasp_dir;
        Eigen::Vector3d grasp_candidate_dir_y = sample_spline_deriv_robot_frame;
        Eigen::Vector3d grasp_candidate_dir_z =
                grasp_dir.cross(sample_spline_deriv_robot_frame);

        Eigen::Matrix3d grasp_rotation_matrix;
        grasp_rotation_matrix(0, 0) = grasp_candidate_dir_x.x();
        grasp_rotation_matrix(1, 0) = grasp_candidate_dir_x.y();
        grasp_rotation_matrix(2, 0) = grasp_candidate_dir_x.z();
        grasp_rotation_matrix(0, 1) = grasp_candidate_dir_y.x();
        grasp_rotation_matrix(1, 1) = grasp_candidate_dir_y.y();
        grasp_rotation_matrix(2, 1) = grasp_candidate_dir_y.z();
        grasp_rotation_matrix(0, 2) = grasp_candidate_dir_z.x();
        grasp_rotation_matrix(1, 2) = grasp_candidate_dir_z.y();
        grasp_rotation_matrix(2, 2) = grasp_candidate_dir_z.z();

        // robot_frame -> candidate tool frame pose
        Eigen::Affine3d grasp_candidate_rotation =
                Eigen::Translation3d(sample_spline_point_robot_frame) *
                grasp_rotation_matrix;

        // robot -> grasp candidate (desired tool) *
        // tool -> wrist *
        // wrist (grasp) -> pregrasp =
        // robot -> wrist
        Eigen::Affine3d candidate_wrist_transform =
                grasp_candidate_rotation *
                m_T_wrist_tool.inverse() *
                m_T_grasp_pregrasp;

        ROS_DEBUG("    Pregrasp Pose [robot frame]: %s", to_string(candidate_wrist_transform).c_str());

        candidates.emplace_back(
                candidate_wrist_transform,
                T_grasp_object.inverse() * candidate_wrist_transform,
                u);

        const GraspCandidate& added = candidates.back();
        Eigen::Affine3d flipped_candidate_transform =
                added.pose *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        candidates.emplace_back(
                flipped_candidate_transform,
                T_grasp_object.inverse() * flipped_candidate_transform,
                added.u);
    }

    return true;
}

} // namespace rcta
