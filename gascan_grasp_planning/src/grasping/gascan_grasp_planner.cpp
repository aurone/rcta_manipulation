#include <gascan_grasp_planning/gascan_grasp_planner.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <spellbook/msg_utils/msg_utils.h>
#include <spellbook/stringifier/stringifier.h>
#include <spellbook/geometry_msgs/geometry_msgs.h>

namespace rcta {

/// \param cp sequence of control points representing the grasp spline
/// \param degree The degree of the grasp spline
GascanGraspPlanner::GascanGraspPlanner() :
    m_initialized(false),
    m_grasp_spline(),
    m_gascan_scale(1.0),
    m_T_wrist_tool(Eigen::Affine3d::Identity()),
    m_T_pregrasp_grasp(Eigen::Affine3d::Identity())
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
    for (auto& cp : spline().control_points()) {
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
            Eigen::Translation3d(-pregrasp_to_grasp_offset_x_m, 0.0, 0.0));
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
    const Eigen::Affine3d& object_pose,
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
        ROS_DEBUG_NAMED("grasping", "Candidate Pregrasp %3d", i);
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
            ROS_DEBUG_NAMED("grasping", "Skipping grasp_spline(%0.3f) [point governed by same knot more than once]", u);
            continue;
        }


        Eigen::Vector3d sample_point_in_obj =
                Eigen::Scaling(m_gascan_scale) * m_grasp_spline(u);
        Eigen::Vector3d sample_deriv_in_obj = m_grasp_spline.deriv(u);

        ROS_DEBUG_NAMED("grasping", "    Sample Spline Point [object frame]: %s", to_string(sample_point_in_obj).c_str());
        ROS_DEBUG_NAMED("grasping", "    Sample Spline Deriv [object frame]: %s", to_string(sample_deriv_in_obj).c_str());

        Eigen::Vector3d sample_spline_point = object_pose * sample_point_in_obj;

        Eigen::Vector3d sample_spline_deriv = object_pose.rotation() * sample_deriv_in_obj.normalized();

        ROS_DEBUG_NAMED("grasping", "    Sample Spline Point [target frame]: %s", to_string(sample_spline_point).c_str());
        ROS_DEBUG_NAMED("grasping", "    Sample Spline Deriv [target frame]: %s", to_string(sample_spline_deriv).c_str());

        // compute the normal to the grasp spline that most points "up"
        Eigen::Vector3d up_bias(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d up_grasp_dir =
                up_bias - up_bias.dot(sample_spline_deriv) * sample_spline_deriv;
        up_grasp_dir.normalize();
        up_grasp_dir *= -1.0;

        // compute the normal to the grasp spline that most points "down"
        Eigen::Vector3d down_bias(-Eigen::Vector3d::UnitZ());
        Eigen::Vector3d down_grasp_dir =
                down_bias - down_bias.dot(sample_spline_deriv) * sample_spline_deriv;
        down_grasp_dir.normalize();
        down_grasp_dir *= -1.0;

        Eigen::Vector3d object_pos(object_pose.translation());
        Eigen::Vector3d grasp_dir;
        Eigen::Vector3d object_to_sample = sample_spline_point - object_pos;
        if (up_grasp_dir.dot(object_to_sample) < 0) {
            grasp_dir = up_grasp_dir;
        } else {
            ROS_DEBUG_NAMED("grasping", "Skipping grasp_spline(%0.3f) [derivative goes backwards along the spline]", u);
            continue;
//            grasp_dir = down_grasp_dir;
        }

        ROS_DEBUG_NAMED("grasping", "    Grasp Direction [robot frame]: %s", to_string(grasp_dir).c_str());

        Eigen::Vector3d grasp_candidate_dir_x = grasp_dir;
        Eigen::Vector3d grasp_candidate_dir_y = sample_spline_deriv;
        Eigen::Vector3d grasp_candidate_dir_z = grasp_dir.cross(sample_spline_deriv);

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

        // model frame -> candidate tool frame pose
        Eigen::Affine3d grasp_candidate_rotation =
                Eigen::Translation3d(sample_spline_point) *
                grasp_rotation_matrix;

        // model -> grasp candidate (desired tool) *
        // tool -> wrist *
        // wrist (grasp) -> wrist (pregrasp) =
        // model -> wrist (pregrasp)
        Eigen::Affine3d candidate_wrist_transform =
                grasp_candidate_rotation * m_T_wrist_tool.inverse();

        ROS_DEBUG_NAMED("grasping", "    Pregrasp Pose [robot frame]: %s", to_string(candidate_wrist_transform).c_str());

        candidates.emplace_back(
                candidate_wrist_transform,
                object_pose.inverse() * candidate_wrist_transform,
                u);

        auto& added = candidates.back();
        Eigen::Affine3d flipped_candidate_transform =
                added.pose *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        candidates.emplace_back(
                flipped_candidate_transform,
                object_pose.inverse() * flipped_candidate_transform,
                added.u);
    }

    double mid_u = 0.5 * (min_u + max_u);
    double u_span = max_u - min_u;
    for (auto& grasp : candidates) {
        // 1 in the middle, 0 at the endpoints
        grasp.u = 1.0 - std::fabs(mid_u - grasp.u) / (0.5 * u_span);
    }

    return true;
}

} // namespace rcta
