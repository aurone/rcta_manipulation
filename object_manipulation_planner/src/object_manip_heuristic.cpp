#include "object_manip_heuristic.h"

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/colors.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/planning_params.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>

#include "assert.h"
#include "variables.h"

static const double FixedPointRatio = 1000.0;

#define HV_LOG H_LOG ".verbose"

#define ENABLE_PREPHI 1

static
void GetEquivalentStates(
    ObjectManipHeuristic* heur,
    int state_id,
    std::vector<int>& ids)
{
    auto* graph = static_cast<const RomanObjectManipLattice*>(
            heur->planningSpace());

    auto* state = graph->getState(state_id);

    SMPL_ASSERT(state->state.size() == VARIABLE_COUNT);
    SMPL_ASSERT(state->coord.size() == VARIABLE_COUNT);

    auto phi = graph->getPhiCoord(state->coord);

    // TODO: connectable might be different than the snap condition

    // return the state ids of all e-graph nodes whose end effector position
    // lie near the input state's end effector position
    auto* egraph = heur->eg->getExperienceGraph();
    auto nodes = egraph->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto egraph_state_id = heur->eg->getStateID(node);

        auto* egraph_state = graph->getState(egraph_state_id);

        SMPL_ASSERT(egraph_state->state.size() == VARIABLE_COUNT);

        if (state->state[HINGE] != egraph_state->state[HINGE]) continue;

//        auto egraph_phi = graph->egraph_pre_phi_coords[node];
        auto egraph_phi = graph->getPhiCoord(egraph_state->coord);

        if (egraph_phi[0] == phi[0] &
            egraph_phi[1] == phi[1] &
            egraph_phi[2] == phi[2])
        {
            ids.push_back(egraph_state_id);
        }
    }
    SMPL_DEBUG_NAMED(H_LOG, "Found %zu potential snap states!", ids.size());
}

auto deadband(double val, double bot) -> double
{
    if (val < bot) {
        return 0.0;
    } else {
        return val;
    }
}

void GetShortcutSuccs(
    ObjectManipHeuristic* heur,
    int state_id,
    std::vector<int>& ids)
{
    SMPL_ASSERT(heur->eg != NULL);

    auto* egraph = heur->eg->getExperienceGraph();
    SMPL_ASSERT(egraph != NULL);

    std::vector<smpl::ExperienceGraph::node_id> egraph_nodes;
    heur->eg->getExperienceGraphNodes(state_id, egraph_nodes);

    for (auto node : egraph_nodes) {
        // Return the final state on the demonstration
        // TODO: check for same component
        smpl::ExperienceGraph::node_id min_node;
        auto min_g = std::numeric_limits<int>::max();
        for (auto i = smpl::ExperienceGraph::node_id(0); i < heur->egraph_goal_heuristics.size(); ++i) {
            if (heur->egraph_goal_heuristics[i] < min_g) {
                min_node = i;
                min_g = heur->egraph_goal_heuristics[i];
            }
        }
        if (min_g != std::numeric_limits<int>::max()) {
            ids.push_back(heur->eg->getStateID(min_node));
        }

#if 0
        auto adj = egraph->adjacent_nodes(node);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            ids.push_back(heur->eg->getStateID(*ait));
        }
#endif
    }
}

auto MakeManipulateHeuristicVisualization(
    const RomanObjectManipLattice* graph,
    const smpl::hash_map<PhiCoord, int, PhiCoordHash>& h)
    -> smpl::visual::Marker
{
    auto min_h_manipulate = std::numeric_limits<int>::max();
    auto max_h_manipulate = 0;
    for (auto& e : h) {
        min_h_manipulate = std::min(min_h_manipulate, e.second);
        max_h_manipulate = std::max(max_h_manipulate, e.second);
    }

    std::vector<Eigen::Vector3d> points;
    std::vector<smpl::visual::Color> colors;
    points.reserve(h.size());
    colors.reserve(h.size());
    for (auto& e : h) {
        Eigen::Vector3d pos;
        graph->posCoordToWorkspace(e.first.data(), pos.data());
        points.push_back(pos);

        auto hue = double(e.second - min_h_manipulate) /
                double(max_h_manipulate - min_h_manipulate);
        auto color = smpl::visual::MakeColorHSV((1.0 - hue) * 270.0);
        colors.push_back(color);
    }

    smpl::visual::Marker m;
    m.pose = Eigen::Affine3d::Identity();
    m.shape = smpl::visual::CubeList{ std::move(points), graph->resolution()[0] };
    m.color = std::move(colors);
    m.frame_id = "map";
    m.ns = "h_manipulate";
    return m;
}

void UpdateUserGoal(
    ObjectManipHeuristic* heur,
    const smpl::GoalConstraint& goal)
{
    heur->egraph_goal_heuristics.clear();
    heur->z_to_phi.clear();
    heur->z_to_pre_phi.clear();
    heur->z_to_egraph_node.clear();
    heur->phi_heuristic.clear();
    heur->pre_phi_heuristic.clear();

    auto* graph = static_cast<const RomanObjectManipLattice*>(
            heur->planningSpace());

    auto goal_z = goal.angles.back();
    auto goal_thresh = goal.angle_tolerances.back();

    SMPL_INFO_NAMED(H_LOG, "Goal Z: %f @ %f", goal_z, goal_thresh);

    auto* egraph = heur->eg->getExperienceGraph();

    SMPL_DEBUG_NAMED(H_LOG, "Precompute manipulation heuristic for %zu e-graph states", egraph->num_nodes());

    heur->egraph_goal_heuristics.resize(egraph->num_nodes(), std::numeric_limits<int>::max());

    struct EGraphSearchNode : smpl::heap_element
    {
        int                g        = std::numeric_limits<int>::max();
        bool               closed   = false;
        EGraphSearchNode*  bp       = NULL;
    };

    // search data for e-graph nodes
//    std::vector<EGraphSearchNode> search_nodes(2 * egraph->num_nodes());
    std::vector<EGraphSearchNode> search_nodes(egraph->num_nodes());

    struct NodeCompare
    {
        bool operator()(const EGraphSearchNode& a, const EGraphSearchNode& b) const
        {
            return a.g < b.g;
        }
    };

    using OpenList = smpl::intrusive_heap<EGraphSearchNode, NodeCompare>;
    OpenList open;

    auto nodes = egraph->nodes();

    // 1. Map z-value -> phi coords of V_demo with that z-value.
    // 2. Map z-value -> pre-phi coords of V_demo with that z-value.
    // 3. Initialize heuristic value for each phi coord to inf.
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& egraph_state = egraph->state(node);
        auto state_id = heur->eg->getStateID(node);
        auto* state = graph->getState(state_id);

        auto phi = graph->getPhiCoord(state->coord);
        heur->z_to_phi[state->coord[OB_P]].push_back(phi);

        auto pre_phi = graph->m_egraph_pre_phi_coords[node];
        heur->z_to_pre_phi[state->coord[OB_P]].push_back(pre_phi);

        heur->z_to_egraph_node[state->coord[OB_P]].push_back(node);
        heur->phi_heuristic[phi] = std::numeric_limits<int>::max();
        heur->pre_phi_heuristic[pre_phi] = std::numeric_limits<int>::max();
    }

    // Remove duplicate phi coordinates.
#if 0 // don't remove for now if we want to maintain e-graph states alongside phi nodes
    for (auto& entry : heur->z_to_phi) {
        std::sort(begin(entry.second), end(entry.second));
        auto uit = std::unique(begin(entry.second), end(entry.second));
        entry.second.erase(uit, end(entry.second));
    }
#endif

    // Add all goal states in the experience graph to the open list.
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& egraph_state = egraph->state(node);
        auto state_id = heur->eg->getStateID(node);
        auto* state = graph->getState(state_id);

        // TODO: maybe make the goal threshold be the resolution of the z
        // variable?
        if (std::fabs(egraph_state[HINGE] - goal_z) <= goal_thresh) {
            search_nodes[node].g = 0;
            open.push(&search_nodes[node]);

            auto phi = graph->getPhiCoord(state->coord);
            heur->phi_heuristic[phi] = 0;
        }
    }

    SMPL_DEBUG_NAMED(H_LOG, "Experience graph contains %zu goal states", open.size());
    if (open.empty()) {
        SMPL_WARN_NAMED(H_LOG, "Experience graph contains no goal states. Reaching the goal is impossible");
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            auto node = *nit;
            auto& egraph_state = egraph->state(node);
            auto egraph_state_z = egraph_state[HINGE];
            SMPL_WARN_NAMED(H_LOG, "  z(%zu) = %f", node, egraph_state_z);
        }
    }

    auto exp_count = 0;
    while (!open.empty()) {
        ++exp_count;
        auto* min = open.min();
        open.pop();
        min->closed = true;

        // Get the e-graph state id from the position of the heuristic
        // state within the array. Get the discrete state by egraph id ->
        // state id -> state.
        auto egraph_node_id = std::distance(search_nodes.data(), min);
        auto state_id = heur->eg->getStateID(egraph_node_id);
        auto* state = graph->getState(state_id);

        auto phi = graph->m_egraph_phi_coords[egraph_node_id]; //graph->getPhiCoord(state->coord);
        auto pre_phi = graph->m_egraph_pre_phi_coords[egraph_node_id];

        // Store the heuristic value of the phi state.
        heur->phi_heuristic[phi] = min->g;
        heur->pre_phi_heuristic[pre_phi] = min->g + 1;

        // store the heuristic value of each e-graph state
        heur->egraph_goal_heuristics[egraph_node_id] = min->g;

        auto adj = egraph->adjacent_nodes(egraph_node_id);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            auto succ_egraph_id = *ait;
            auto& succ = search_nodes[succ_egraph_id];
            if (succ.closed) {
                continue;
            }

            auto succ_state_id = heur->eg->getStateID(succ_egraph_id);
            auto* succ_state = graph->getState(succ_state_id);

            auto succ_phi = graph->getPhiCoord(succ_state->coord);

            auto dx = graph->resolution()[0] * double(succ_phi[0] - phi[0]);
            auto dy = graph->resolution()[1] * double(succ_phi[1] - phi[1]);
            auto dz = graph->resolution()[2] * double(succ_phi[2] - phi[2]);

            auto cost = (int)std::round(
                    FixedPointRatio *
                    std::sqrt(dx * dx + dy * dy + dz * dz));

            int new_cost = min->g + cost;
            if (new_cost < succ.g) {
                succ.g = new_cost;
                succ.bp = min;
                if (open.contains(&succ)) {
                    open.decrease(&succ);
                } else {
                    open.push(&succ);
                }
            }
        }
    }


    SV_SHOW_INFO_NAMED("h_manipulate", MakeManipulateHeuristicVisualization(graph, heur->phi_heuristic));

    SMPL_INFO_NAMED(H_LOG, "Expanded %d nodes computing H_manipulate", exp_count);
}

void UpdateGoal(ObjectManipHeuristic* heur, const smpl::GoalConstraint& goal)
{
    // find the shortest path from any state in the demonstration to the final
    // demonstration state (basically get the length of the 3d arc motion
    // remaining on the demonnstration

    // map from all z values to x,y,z  states in the demonstration (more
    // generally all robot states (all all, including those states suggested by
    // the E_z edges)
    switch (goal.type) {
    case smpl::GoalType::USER_GOAL_CONSTRAINT_FN:
        return UpdateUserGoal(heur, goal);
    case smpl::GoalType::XYZ_GOAL:
    case smpl::GoalType::XYZ_RPY_GOAL:
    case smpl::GoalType::MULTIPLE_POSE_GOAL:
    case smpl::GoalType::JOINT_STATE_GOAL:
    default:
        SMPL_WARN_NAMED(H_LOG, "Unsupported goal type %d", (int)goal.type);
        break;
    }
}

auto normalize_disc_theta(int theta, int num_angles) -> int
{
    if (theta >= 0) {
        return theta % num_angles;
    } else {
        return (theta % num_angles + num_angles) % num_angles;
    }
}

auto shortest_angle_dist(int t1, int t2, int num_angles) -> int
{
    if (t2 > t1) {
        auto diff = (t2 - t1) % num_angles;
        return std::min(diff, num_angles - diff);
    } else {
        auto diff = (t1 - t2) % num_angles;
        return std::min(diff, num_angles - diff);
    }
}

auto discretize_angle(double angle, double res, int num_angles) -> int
{
    angle = smpl::normalize_angle_positive(angle);

    auto coord = (int)((angle + res * 0.5) / res);

    if (coord == num_angles) {
        coord = 0;
    }

    return coord;
}

auto MakePoseTransform(double x, double y, double theta) -> Eigen::Affine3d
{
    return Eigen::Translation3d(x, y, 0.0) * Eigen::AngleAxisd(theta, Eigen::Vector3d::Zero());
}

auto ComputeRotationHeuristic(
    int disc_src_theta,     // discrete source orientation
    int disc_dst_theta,     // discrete destination orientation
    double src_theta,       // continuous source orientation
    double dst_theta,       // continuous destination orientation
    double dx,              // continuous base delta
    double dy,
    double dtheta,
    int disc_dx,            // discrete base delta
    int disc_dy,
    int disc_dtheta,
    int heading_condition,  // type of condition to determine if heading is considered
    double heading_thresh,
    int num_angles,         // number of discrete angles
    double angle_res,       // resolution of each discrete angle
    bool discretize)        // whether to discrete the result
    -> double
{
    // determine whether to include a heading term
    auto add_heading = false;
    switch (heading_condition) {
    case ObjectManipHeuristic::HeadingCondition::Discrete:
        add_heading = (disc_dx != 0 || disc_dy != 0);
        break;
    case ObjectManipHeuristic::HeadingCondition::Continuous:
        add_heading = (dx * dx + dy * dy) > heading_thresh * heading_thresh;
        break;
    case ObjectManipHeuristic::HeadingCondition::None:
        add_heading = false;
        break;
    }

    // compute heading term if required
    auto heading = 0.0;
    if (add_heading) {
        heading = atan2(dy, dx);
    }

    if (add_heading) {
        if (discretize) {
            auto disc_heading = discretize_angle(heading, angle_res, num_angles);
            // heading pointing in the opposite direction
            auto disc_heading_back = disc_heading + (num_angles >> 1);
            if (shortest_angle_dist(disc_src_theta, disc_heading, num_angles) <
                shortest_angle_dist(disc_src_theta, disc_heading_back, num_angles))
            {
                auto disc_angle_dist =
                        shortest_angle_dist(
                                disc_src_theta, disc_heading, num_angles) +
                        shortest_angle_dist(
                                disc_heading, disc_dst_theta, num_angles);
                return angle_res * (double)disc_angle_dist;
            } else {
                auto disc_angle_dist =
                        shortest_angle_dist(
                                disc_heading_back, disc_src_theta, num_angles) +
                        shortest_angle_dist(
                                disc_heading_back, disc_dst_theta, num_angles);
                return angle_res * (double)disc_angle_dist;
            }
        } else {
            auto heading_back = heading + M_PI;
            // rotate to face the target base position forwards or backwards
            if (smpl::shortest_angle_dist(src_theta, heading) <
                smpl::shortest_angle_dist(src_theta, heading_back))
            {
                // face towards the target base position
                return smpl::shortest_angle_dist(src_theta, heading) +
                        smpl::shortest_angle_dist(heading, dst_theta);
            } else {
                // face away from the target base position
                return smpl::shortest_angle_dist(src_theta, heading_back) +
                        smpl::shortest_angle_dist(heading_back, dst_theta);
            }
        }
    } else {
        if (discretize) {
            return angle_res * double(disc_dtheta);
        } else {
            // return nominal rotation term
            return dtheta;
        }
    }

    assert(0);
    return 0.0;
}

auto ComputePositionHeuristic(
    double dx, double dy,
    double disc_dx, double disc_dy,
    double x_res, double y_res,
    bool discretize)
    -> double
{
    if (discretize) {
        auto cdx = x_res * disc_dx;
        auto cdy = y_res * disc_dy;
        return std::sqrt(cdx * cdx + cdy * cdy);
    } else {
        return std::sqrt(dx * dx + dy * dy);
    }
}

int GetGoalHeuristic(ObjectManipHeuristic* heur, int state_id)
{
    SMPL_DEBUG_NAMED(H_LOG, "GetGoalHeuristic(%d)", state_id);

    auto* graph = static_cast<RomanObjectManipLattice*>(heur->planningSpace());

    if (state_id == graph->getGoalStateID()) {
        SMPL_DEBUG_NAMED(H_LOG, "  h(goal) = 0");
        return 0;
    }

    auto* state = graph->getState(state_id);
    SMPL_ASSERT(state->coord.size() == VARIABLE_COUNT);
    SMPL_ASSERT(state->state.size() == VARIABLE_COUNT);

    // visualize the base pose
    SV_SHOW_DEBUG_NAMED("state_base", smpl::visual::MakeFrameMarkers(
            MakePoseTransform(
                    state->state[WORLD_JOINT_X],
                    state->state[WORLD_JOINT_Y],
                    state->state[WORLD_JOINT_THETA]),
            "map",
            "state_base"));

//    SMPL_DEBUG_STREAM_NAMED(H_LOG, "  coord(state) = " << state->state);

    auto phi = graph->getPhiCoord(state->coord);

    // Test whether the task-space projection of this state is coincident with
    // any state on the demonstration
    auto on_demo = heur->phi_heuristic.find(phi) != end(heur->phi_heuristic);

    SMPL_DEBUG_NAMED(H_LOG, "  on-demo(state) = %s", on_demo ? "true" : "false");

    SMPL_DEBUG_NAMED(H_LOG, "  phi(state) = (%d, %d, %d)", phi[0], phi[1], phi[2]);

    auto* egraph = heur->eg->getExperienceGraph();
    SMPL_ASSERT(egraph != NULL);

    auto h_min = std::numeric_limits<int>::max();
    int h_base_min;
    int h_contact_min;
    int h_manipulate_min;

    std::vector<smpl::ExperienceGraph::node_id> the_nodes;
    heur->eg->getExperienceGraphNodes(state_id, the_nodes);
    auto is_egraph = !the_nodes.empty();

    // If our projected state is coincident with any state on the projected
    // e-graph, we'll consider the state on the projected e-graph, otherwise
    // we'll consider offsets from the projected e-graph, to bias the search
    // towards pre-grasps.
    auto& phis = on_demo ?
            heur->z_to_phi[state->coord[OB_P]] :
            heur->z_to_pre_phi[state->coord[OB_P]];

    auto& egraph_nodes = heur->z_to_egraph_node[state->coord[OB_P]];

    SMPL_DEBUG_NAMED(H_LOG, "  Inspect %zu phi nodes", phis.size());
    for (auto i = 0; i < phis.size(); ++i) {
        ///////////////////////
        // Compute h_contact //
        ///////////////////////

        auto& egraph_phi = phis[i];
        SMPL_DEBUG_NAMED(HV_LOG, "    phi(v) = (%d, %d, %d)", egraph_phi[0], egraph_phi[1], egraph_phi[2]);

        auto h_contact = 0;
        {
            auto dx = graph->resolution()[0] * double(egraph_phi[0] - phi[0]);
            auto dy = graph->resolution()[1] * double(egraph_phi[1] - phi[1]);
            auto dz = graph->resolution()[2] * double(egraph_phi[2] - phi[2]);
            h_contact = (int)(heur->w_egraph * FixedPointRatio * std::sqrt(dx * dx + dy * dy + dz * dz));
        }

        SMPL_DEBUG_NAMED(HV_LOG, "    h_contact(s,v) = %d", h_contact);

        /////////////////////////
        // Lookup h_manipulate //
        /////////////////////////

        auto manipit = on_demo ?
                heur->phi_heuristic.find(egraph_phi) :
                heur->pre_phi_heuristic.find(egraph_phi);
//        SMPL_ASSERT(manipit != end(heur->phi_heuristic));
        auto h_manipulate = manipit->second;
        SMPL_DEBUG_NAMED(HV_LOG, "    h_manip(v) = %d", h_manipulate);

        ///////////////////////////////////////////////////////////////
        // h_base - now we care about the actual demonstration state //
        ///////////////////////////////////////////////////////////////

        auto node = egraph_nodes[i];

        // get the state for this experience graph node
        // or just use the provided robot state directly
        auto& egraph_state = egraph->state(node);
        SMPL_ASSERT(egraph_state.size() == VARIABLE_COUNT);

        auto egraph_state_id = heur->eg->getStateID(node);
        auto* egraph_graph_state = graph->getState(egraph_state_id);
        auto* graph_state = graph->getState(state_id);

        // visualize the base pose of the egraph state
        SV_SHOW_DEBUG_NAMED("egraph_base", smpl::visual::MakeFrameMarkers(
                MakePoseTransform(
                        egraph_state[WORLD_JOINT_X],
                        egraph_state[WORLD_JOINT_Y],
                        egraph_state[WORLD_JOINT_THETA]),
                "map",
                "egraph_base"));

        // pose delta in discrete space
        auto disc_dx = graph_state->coord[BD_PX] - egraph_graph_state->coord[BD_PX];
        auto disc_dy = graph_state->coord[BD_PY] - egraph_graph_state->coord[BD_PY];
        auto disc_dtheta = shortest_angle_dist(graph_state->coord[BD_TH], egraph_graph_state->coord[BD_TH], graph->m_val_count[BD_TH]);

        SMPL_DEBUG_NAMED(HV_LOG, "    disc delta base = (%d, %d, %d)", disc_dx, disc_dy, disc_dtheta);

        // pose delta in continuous space
        auto dx = egraph_state[WORLD_JOINT_X] - state->state[WORLD_JOINT_X];
        auto dy = egraph_state[WORLD_JOINT_Y] - state->state[WORLD_JOINT_Y];
        auto dtheta = smpl::shortest_angle_dist(
                egraph_state[WORLD_JOINT_THETA], state->state[WORLD_JOINT_THETA]);

        auto rot_dist = 0.0;
        if (heur->use_rotation) {
            rot_dist = ComputeRotationHeuristic(
                    graph_state->coord[BD_TH], egraph_graph_state->coord[BD_TH],
                    graph_state->state[BD_TH], egraph_graph_state->coord[BD_TH],
                    dx, dy, dtheta,
                    disc_dx, disc_dy, disc_dtheta,
                    heur->heading_condition,
                    heur->heading_thresh,
                    graph->m_val_count[BD_TH],
                    graph->resolution()[BD_TH],
                    heur->disc_rotation_heuristic);
            if (!heur->disc_rotation_heuristic) {
                rot_dist = deadband(rot_dist, heur->theta_db);
            }
        }

        auto pos_dist = ComputePositionHeuristic(
                dx, dy,
                disc_dx, disc_dy,
                graph->resolution()[BD_PX],
                graph->resolution()[BD_PY],
                heur->disc_position_heuristic);
        if (!heur->disc_position_heuristic) {
            pos_dist = deadband(pos_dist, heur->pos_db);
        }

        SMPL_DEBUG_NAMED(HV_LOG, "    pos dist = %f", pos_dist);
        SMPL_DEBUG_NAMED(HV_LOG, "    rot dist(degs) = %f", smpl::to_degrees(rot_dist));

        auto h_base_pos = pos_dist;
        auto h_base_rot = heur->theta_normalizer * rot_dist;
        auto h_base = (int)(heur->w_egraph * FixedPointRatio * (h_base_rot + h_base_pos));

        SMPL_DEBUG_NAMED(HV_LOG, "    h_base_pos = %f, h_base_rot = %f", h_base_pos, h_base_rot);
        h_base *= heur->h_base_weight;

        SMPL_DEBUG_NAMED(HV_LOG, "    h_base(s,v) = %d", h_base);

        auto cost = 0;
        switch (heur->combination) {
        case ObjectManipHeuristic::CombinationMethod::Max:
            cost = std::max(h_base, h_contact) + h_manipulate;
            break;
        case ObjectManipHeuristic::CombinationMethod::Sum:
            cost = h_base + h_contact + h_manipulate;
            break;
        }

        if (cost < h_min) {
            h_min = cost;
            h_base_min = h_base;
            h_contact_min = h_contact;
            h_manipulate_min = h_manipulate;
        }
    }

    if (h_min == std::numeric_limits<int>::max()) {
        SMPL_WARN_ONCE_NAMED(H_LOG, "no e-graph state with z = %0.12f", state->state[HINGE]);
    }

    if (is_egraph) {
        SMPL_DEBUG_NAMED(H_LOG, "  h(%d) = %d", state_id, h_manipulate_min);
        return h_manipulate_min;
    } else {
        switch (heur->combination) {
        case ObjectManipHeuristic::CombinationMethod::Max:
            SMPL_DEBUG_NAMED(H_LOG, "  h(%d) = max(base, contact) + manip = max(%d, %d) + %d = %d", state_id, h_base_min, h_contact_min, h_manipulate_min, h_min);
            break;
        case ObjectManipHeuristic::CombinationMethod::Sum:
            SMPL_DEBUG_NAMED(H_LOG, "  h(%d) = base + contact + manip = %d + %d + %d = %d", state_id, h_base_min, h_contact_min, h_manipulate_min, h_min);
            break;
        }
        if (h_min == std::numeric_limits<int>::max()) {
            return std::numeric_limits<int16_t>::max();
        }
        return h_min;
    }
}

////////////////////////////////////
// ObjectManipHeuristic Interface //
////////////////////////////////////

bool Init(ObjectManipHeuristic* heur, smpl::RobotPlanningSpace* space)
{
    heur->eg = space->getExtension<smpl::ExperienceGraphExtension>();
    if (heur->eg == NULL) {
        SMPL_WARN_NAMED(H_LOG, "ObjectManipHeuristic requires Experience Graph Extension");
        return false;
    }

    heur->extract_state = space->getExtension<smpl::ExtractRobotStateExtension>();
    if (heur->extract_state == NULL) {
        SMPL_WARN_NAMED(H_LOG, "ObjectManipHeuristic requires Extract Robot State Extension");
        return false;
    }

    heur->project_to_point = space->getExtension<smpl::PointProjectionExtension>();
    if (heur->project_to_point == NULL) {
        SMPL_WARN_NAMED(H_LOG, "ObjectManipHeuristic requires Point Projection Extension");
        return false;
    }

    if (!heur->RobotHeuristic::init(space)) {
        SMPL_WARN_NAMED(H_LOG, "Failed to initialize Robot Heuristic");
        return false;
    }

    return true;
}

void ObjectManipHeuristic::getEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    return GetEquivalentStates(this, state_id, ids);
}

void ObjectManipHeuristic::getShortcutSuccs(
    int state_id,
    std::vector<int>& ids)
{
    return GetShortcutSuccs(this, state_id, ids);
}

double ObjectManipHeuristic::getMetricStartDistance(double x, double y, double z)
{
    return 0.0;
}

double ObjectManipHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

void ObjectManipHeuristic::updateGoal(const smpl::GoalConstraint& goal)
{
    return UpdateGoal(this, goal);
}

int ObjectManipHeuristic::GetGoalHeuristic(int state_id)
{
    return ::GetGoalHeuristic(this, state_id);
}

int ObjectManipHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int ObjectManipHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

auto ObjectManipHeuristic::getExtension(size_t class_code) -> Extension*
{
    if (class_code == smpl::GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return NULL;
}
