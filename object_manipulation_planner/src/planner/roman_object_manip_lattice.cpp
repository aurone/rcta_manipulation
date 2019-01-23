#include "roman_object_manip_lattice.h"

#include <smpl/angles.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/workspace_lattice_action_space.h>
#include <smpl/spatial.h>

#include "smpl_assert.h"
#include "object_manip_heuristic.h"
#include "variables.h"

#define G_SNAP_LOG G_SUCCESSORS_LOG ".snap"

namespace TransitionType
{
auto to_cstring(Type t) -> const char*
{
    switch (t) {
    case OrigStateOrigSucc:         return "OrigStateOrigSucc";
    case OrigStateBridgeSucc:       return "OrigStateBridgeSucc";
    case OrigStateZSucc:            return "OrigStateZSucc";
    case EGraphStateAdjSucc:        return "EGraphStateAdjSucc";
    case EGraphStateBridgeSucc:     return "EGraphStateBridgeSucc";
    case EGraphStateZSucc:          return "EGraphStateZSucc";
    case PreGraspAmpSucc:           return "PreGraspAmpSucc";
    case GraspSucc:                 return "GraspSucc";
    case PreGraspSucc:              return "PreGraspSucc";
    case SnapSucc:                  return "SnapSucc";
    case ShortcutSucc:              return "ShortcutSucc";
    default:                        return "<UNKNOWN>";
    }
}
}

// Color a set of markers.
static
auto color(
    std::vector<smpl::visual::Marker>&& markers,
    smpl::visual::Color color)
    -> std::vector<smpl::visual::Marker>&&
{
    for (auto& m : markers) {
        m.color = color;
    }
    return std::move(markers);
}

// Get successors of actions from E_z where u is an E-Graph state.
void RomanObjectManipLattice::getEGraphStateZSuccs(
    smpl::WorkspaceLatticeState* state,
    smpl::ExperienceGraph::node_id egraph_node,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    // TODO: E_z? probably not worthwhile to generate z-edges here...we could
    // run IK to end effector poses of adjacent states to generate alternatives
    // but that seems covered by a bridge + orig-z action sequence
}

// Get successors of actions from E_z where u is not an E-Graph state. The
// successors are determined by applying the original action space and checking
// whether the endpoints of the action align with any edge in the
// demonstration.
void RomanObjectManipLattice::getOrigStateZSuccs(
    smpl::WorkspaceLatticeState* state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Apply Z actions");
    auto orig_succ_count = succs->size();

    // s_demo states where phi(s) ~ phi(s_demo) and z(s) = z(s_demo)
    auto parent_nearby_nodes = std::vector<smpl::ExperienceGraph::node_id>();
    {
        auto pose_coord = getPhiCoord(state->coord);
        SMPL_DEBUG_STREAM("parent pose coord = " << pose_coord);
        auto it = m_phi_to_egraph_nodes.find(pose_coord);
        if (it != end(m_phi_to_egraph_nodes)) {
            for (auto node : it->second) {
                if (m_egraph.state(node)[HINGE] == state->state[HINGE]) {
                    parent_nearby_nodes.push_back(node);
                }
            }
        }
    }

    if (parent_nearby_nodes.empty()) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  -> off demonstration");
        return;
    }

    auto actions = std::vector<smpl::WorkspaceAction>();
    m_actions->apply(*state, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  base actions: %zu", actions.size());

    auto invalid_count = 0;
    auto off_demo_count = 0;
    auto disconnected_count = 0;

    auto marker_id = 0;

    // iterate through successors of source state
    for (auto i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        auto final_robot_state = smpl::RobotState();
        if (!checkAction(state->state, action, &final_robot_state)) {
            ++invalid_count;
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      -> invalid");
            continue;
        }

        auto& final_state = action.back();
        auto succ_coord = smpl::WorkspaceCoord();
        stateWorkspaceToCoord(final_state, succ_coord);

        // E_z from V_orig
        auto pose_coord = getPhiCoord(succ_coord);
        auto it = m_phi_to_egraph_nodes.find(pose_coord);
        if (it == end(m_phi_to_egraph_nodes)) {
            ++off_demo_count;
            SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      -> off demonstration");
            continue;
        }

        // for each demonstration state s_demo' where phi(s_demo') = phi(s')
        for (auto node : it->second) {
            // is there any edge between S_demo and S_demo'?
            // for (auto nn : parent_nearby_nodes)
            {
                // if (!m_egraph.edge(nn, node)) continue;

                SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      -> found successor");

                // overwrite the z value of the destination state
                auto& egraph_state = m_egraph.state(node);
                SMPL_ASSERT(egraph_state.size() == robot()->jointVariableCount());
                auto z = egraph_state[HINGE];
                auto this_final_state = final_state;
                auto this_final_robot_state = final_robot_state;
                this_final_state[OB_P] = z;
                this_final_robot_state[HINGE] = z;

                auto succ_coord = smpl::WorkspaceCoord();
                stateWorkspaceToCoord(this_final_state, succ_coord);
                auto succ_id = createState(succ_coord);
                auto* succ_state = getState(succ_id);
                succ_state->state = this_final_robot_state;
                SMPL_DEBUG_NAMED(G_LOG, "Return Z-EDGE z = %f", z);
                succs->push_back(succ_id);

                auto edge_cost = computeCost(*state, *succ_state);
                costs->push_back(edge_cost);
            }
        }

        ++disconnected_count;

        auto color = [](std::vector<smpl::visual::Marker> markers) {
            for (auto& marker : markers) {
                marker.color = smpl::visual::Color{ 1.0f, 0.0f, 0.5f, 0.9f };
            }
            return markers;
        };

        auto label = [&](std::vector<smpl::visual::Marker> markers) {
            for (auto& marker : markers) {
                marker.id = marker_id++;
            }
            return markers;
        };

        auto vis_name = "candidate_z_successor";
        SV_SHOW_DEBUG_NAMED(vis_name, label(color(getStateVisualization(final_robot_state, vis_name))));
    }

    auto succ_count = succs->size();
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Found %zu/%zu z successors (%d invalid, %d off demo, %d disconnected)", succ_count - orig_succ_count, actions.size(), invalid_count, off_demo_count, disconnected_count);
}

// Get successors of actions from E_z where u is not an E-Graph state. The
// successors are determined by attempting an adaptive IK motion to the
// states adjacent to any demonstration states that are near this state.
void RomanObjectManipLattice::getOrigStateZSuccs2(
    smpl::WorkspaceLatticeState* state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Apply Z actions");

    // s_demo states where phi(s) ~ phi(s_demo) and z(s) = z(s_demo)
    auto parent_nearby_nodes = std::vector<smpl::ExperienceGraph::node_id>();
    {
        auto pose_coord = getPhiCoord(state->coord);
        SMPL_DEBUG_STREAM("  parent pose coord = " << pose_coord);
        auto it = m_phi_to_egraph_nodes.find(pose_coord);
        if (it != end(m_phi_to_egraph_nodes)) {
            SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  check z-values of " << it->second.size() << " e-graph states at " << pose_coord);
            for (auto node : it->second) {
                // The reason for this tolerance check...
                //
                // The z-value of a state is unfortunately discretized (even
                // though there are already a finite number of known values
                // taken directly from the demonstration). This means if
                // there are multiple states within the same bin, but with
                // slightly different z-values, we'll end up only retaining
                // one of them and either discarding the new continuous z-value
                // or overwriting it with the new continuous z-value.
                //
                // Ideally, for each z-value we would create new special states
                // with the same phi coordinates and only discretize the free
                // angles.
                if (std::fabs(m_egraph.state(node)[HINGE] - state->state[HINGE] < 1e-3)) {
                    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    matching z == %f!", m_egraph.state(node)[HINGE]);
                    parent_nearby_nodes.push_back(node);
                } else {
                    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    %f != %f", m_egraph.state(node)[HINGE], state->state[HINGE]);
                }
            }
        }
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  %zu equivalent e-graph states", parent_nearby_nodes.size());

    auto old_size = succs->size();

    auto these_succs = std::vector<int>();

    auto marker_id = 0;

    auto candidate_succ_count = 0;
    auto ik_failure_count = 0;
    auto collision_count = 0;

    // for all adjacent nodes in the experience graph
    for (auto neighbor_id : parent_nearby_nodes) {
        auto adj_nodes = m_egraph.adjacent_nodes(neighbor_id);
        for (auto ait = adj_nodes.first; ait != adj_nodes.second; ++ait) {
            ++candidate_succ_count;

            // generate successors that have the same z-value and the same
            // phi coordinate as the adjacent node

            auto adj_id = *ait;

            auto& pose = m_egraph_node_grasps[adj_id];

            // TODO: permissive or restrictive ik?
            // probably want restrictive but with a mini-search over free angles
            // for the arm/torso
#define RESTRICTIVE_Z_EDGES 0
#if RESTRICTIVE_Z_EDGES
            auto solution = smpl::RobotState();
            if (!m_rm_iface->computeFastIK(pose, state->state, solution)) {
                ++ik_failure_count;
                SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, " -> IK Failure");
                continue;
            }
#else
            auto solution = smpl::RobotState();
            if (!m_ik_iface->computeIK(pose, state->state, solution)) {
                ++ik_failure_count;
                SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, " -> IK Failure");
                continue;
            }
#endif

            if (!collisionChecker()->isStateToStateValid(state->state, solution)) {
                ++collision_count;
                SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, " -> In collision");
                continue;
            }

            solution[HINGE] = m_egraph.state(adj_id)[HINGE];

            // 1. add this to our state table?
            // 2. check if there is an e-graph state with the same coordinates?
            // 3. create a new state for this state?

            auto succ_coord = smpl::WorkspaceCoord();
            stateRobotToCoord(solution, succ_coord);

            auto succ_id = createState(succ_coord);
            auto* succ_state = getState(succ_id);
            succ_state->state = solution;
            these_succs.push_back(succ_id);

            succs->push_back(succ_id);

            auto edge_cost = computeCost(*state, *succ_state);
            costs->push_back(edge_cost);

            auto color = [](std::vector<smpl::visual::Marker> markers) {
                for (auto& marker : markers) {
                    marker.color = smpl::visual::Color{ 1.0f, 0.0f, 0.5f, 0.9f };
                }
                return markers;
            };

            auto label = [&](std::vector<smpl::visual::Marker> markers) {
                for (auto& marker : markers) {
                    marker.id = marker_id++;
                }
                return markers;
            };

            auto vis_name = "candidate_z_successor";
            SV_SHOW_DEBUG_NAMED(vis_name, label(color(getStateVisualization(solution, vis_name))));
        }
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Generated %zu/%d z successors (%d ik failures, %d invalid states)",
            succs->size() - old_size, candidate_succ_count, ik_failure_count, collision_count);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  successors: " << these_succs);
}

// Apply an adaptive motion to a state that moves the end effector to the
// nearest pre-grasp pose of any state in the demonstration with the same
// z-value.
void RomanObjectManipLattice::getPreGraspAmpSucc(
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    // find the closest e-graph node, wrt to phi coordinate positions, that
    // has the same z-value
    auto closest = smpl::ExperienceGraph::node_id(-1);
    auto best = std::numeric_limits<int>::max();
    auto nodes = m_egraph.nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        if (m_egraph.state(node)[HINGE] != state->state[HINGE]) continue;

        auto& egraph_phi = m_egraph_pre_phi_coords[node];
        auto dx = egraph_phi[0] - phi_coord[0];
        auto dy = egraph_phi[1] - phi_coord[1];
        auto dz = egraph_phi[2] - phi_coord[2];
        auto dist = dx * dx + dy * dy + dz * dz;
        if (dist < best) {
            best = dist;
            closest = node;
        }
    }

    if (closest != -1) {
        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "Attempt adaptive motion to pre-grasp");

        auto& pregrasp = m_egraph_node_pregrasps[closest];

        SV_SHOW_DEBUG_NAMED("pregrasp_target", smpl::visual::MakeFrameMarkers(pregrasp, "map", "pregrasp_target"));

#if 0
        auto& closest_egraph_state = m_egraph.state(closest);
        smpl::RobotState target_state;
        if (!m_ik_iface->computeIK(pregrasp, closest_egraph_state, target_state)) return;
        if (!collisionChecker()->isStateToStateValid(state->state, target_state)) return;

        smpl::WorkspaceState succ_workspace_state;
        stateRobotToWorkspace(target_state, succ_workspace_state);

        smpl::WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(succ_workspace_state, succ_coord);
        auto succ_id = createState(succ_coord);
        auto* succ_state = getState(succ_id);
        succ_state->state = target_state;

        succs->push_back(succ_id);
        costs->push_back(1); // edge cost of one for grasp/pregrasp actions
#else
        auto seed = state->state;
        smpl::RobotState final_robot_state;
        if (!m_ik_iface->computeIK(pregrasp, seed, final_robot_state)) {
            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "  Failed to move to pre-grasp");
            return;
        }

        SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "  Adaptive motion to pre-grasp succeeded");

        if (!collisionChecker()->isStateToStateValid(
                state->state, final_robot_state))
        {
            return;
        }

        smpl::WorkspaceState succ_workspace_state;
        stateRobotToWorkspace(final_robot_state, succ_workspace_state);

        smpl::WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(succ_workspace_state, succ_coord);
        auto succ_id = createState(succ_coord);
        auto* succ_state = getState(succ_id);
        succ_state->state = final_robot_state;

        succs->push_back(succ_id);
        costs->push_back(1); // edge cost of one for grasp/pregrasp actions
#endif
    }
}

// For a state $s$, for all states $s_demo$ on the demonstration where
// $z(s_demo) = z(s)$ and $phi(state) = phi(s_demo)$, apply an adaptive
// motion that uses IK to move from $s$ to a state $s'$ where $phi(s') =
// pre-phi(s_demo)$. These actions effectively "release" the object and move
// the end effector away to the pre-grasp pose.
void RomanObjectManipLattice::getPreGraspSuccs(
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto it = m_phi_to_egraph_nodes.find(phi_coord);
    if (it != end(m_phi_to_egraph_nodes)) {
        for (auto node : it->second) {
            if (m_egraph.state(node)[HINGE] != state->state[HINGE]) continue;

            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "Attempt pre-grasp motion");
            auto& pregrasp = m_egraph_node_pregrasps[node];

            // add an action that "releases" the object and moves to the post-grasp

            // allowed to move the torso and right arm, but not the base...so the
            // ik group
            auto seed = state->state;
            smpl::RobotState final_robot_state;
            if (m_ik_iface->computeIK(pregrasp, seed, final_robot_state)) {
                // TODO: no collision checking here. We explicitly don't want to
                // run nominal collision checking, because it is expected that
                // there will be collisions between the robot and the object when
                // the object is grasped. It might be worthwhile to at least check
                // the post-grasp endpoint for collisions.

                smpl::WorkspaceState succ_workspace_state;
                stateRobotToWorkspace(final_robot_state, succ_workspace_state);

                smpl::WorkspaceCoord succ_coord;
                stateWorkspaceToCoord(succ_workspace_state, succ_coord);
                auto succ_id = createState(succ_coord);
                auto* succ_state = getState(succ_id);
                succ_state->state = final_robot_state;

                SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "Pre-grasp motion succeeded");
                succs->push_back(succ_id);
                costs->push_back(1); // edge cost of one for grasp/pregrasp actions
            }
        }
    }
}

// For a state $s$, for all states $s_demo$ on the demonstration where
// $z(s_demo) = z(s)$ and $phi(state) = pre-phi(s_demo)$, apply an adaptive
// motion that uses IK to move from $s$ to a state $s'$ where $phi(s') =
// phi(s_demo)$.
void RomanObjectManipLattice::getGraspSuccs(
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    // If the contact coordinates are the same as the pregrasp coordinates of
    // some state in the demonstration, then an action is available to "grasp"
    // the object by moving the end effector to the grasp pose.
    auto pre_it = m_pregrasp_phi_to_egraph_node.find(phi_coord);
    if (pre_it != end(m_pregrasp_phi_to_egraph_node)) {
        for (auto node : pre_it->second) {
            if (m_egraph.state(node)[HINGE] != state->state[HINGE]) continue;

            SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "Attempt grasp motion");
            auto& grasp = m_egraph_node_grasps[node];
            auto seed = state->state;
            smpl::RobotState final_robot_state;
            if (m_ik_iface->computeIK(grasp, seed, final_robot_state)) {
                smpl::WorkspaceState succ_workspace_state;
                stateRobotToWorkspace(final_robot_state, succ_workspace_state);

                smpl::WorkspaceCoord succ_coord;
                stateWorkspaceToCoord(succ_workspace_state, succ_coord);

                auto succ_id = createState(succ_coord);
                auto* succ_state = getState(succ_id);
                succ_state->state = final_robot_state;

                SMPL_DEBUG_NAMED(G_SUCCESSORS_LOG, "Grasp motion succeeded");
                succs->push_back(succ_id);
                costs->push_back(1);
            }
        }
    }
}

// TODO:
void GetEGraphStateAdjacentSuccs(
    RomanObjectManipLattice* graph,
    smpl::WorkspaceLatticeState* state,
    smpl::ExperienceGraph::node_id egraph_node,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto& egraph_state = graph->m_egraph.state(egraph_node);

#if 0
    // original logic from WorkspaceLatticeEGraph
    // E_demo from V_demo
    auto adj = m_egraph.adjacent_nodes(egraph_node);
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  %td adjacent experience graph edges", std::distance(adj.first, adj.second));
    for (auto ait = adj.first; ait != adj.second; ++ait) {
        auto& adj_egraph_state = m_egraph.state(*ait);
        WorkspaceState workspace_state;
        stateRobotToWorkspace(adj_egraph_state, workspace_state);
        succs->push_back(m_egraph_node_to_state[*ait]);
        costs->push_back(10);
    }
#else
    auto edges = graph->m_egraph.edges(egraph_node);
    for (auto eit = edges.first; eit != edges.second; ++eit) {
        auto edge = *eit;
        if (graph->m_egraph_edge_validity[edge]) {
            // can't remember if edges are shared or directional in
            // ExperienceGraph...test both here
            auto n1 = graph->m_egraph.source(edge);
            auto n2 = graph->m_egraph.target(edge);
            if (n1 != egraph_node) {
                auto& adj_egraph_state = graph->m_egraph.state(n1);
                auto workspace_state = smpl::WorkspaceState();
                graph->stateRobotToWorkspace(adj_egraph_state, workspace_state);
                succs->push_back(graph->m_egraph_node_to_state[n1]);
                costs->push_back(10);
            }

            if (n2 != egraph_node) {
                auto& adj_egraph_state = graph->m_egraph.state(n2);
                auto workspace_state = smpl::WorkspaceState();
                graph->stateRobotToWorkspace(adj_egraph_state, workspace_state);
                succs->push_back(graph->m_egraph_node_to_state[n2]);
                costs->push_back(10);
            }
        }
    }
#endif
}


void RomanObjectManipLattice::getUniqueSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);
    auto* state = getState(state_id);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  workspace coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "      robot state: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(state->state, vis_name));

    auto is_egraph_node = false;
    smpl::ExperienceGraph::node_id egraph_node;
    {
        auto it = m_state_to_egraph_node.find(state_id);
        if (it != end(m_state_to_egraph_node)) {
            is_egraph_node = true;
            egraph_node = it->second;
        }
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  egraph state: %s", is_egraph_node ? "true" : "false");

//    auto enable_z_edges = false;
    auto enable_z_edges = true;

    auto enable_egraph_edges = true;
//    auto enable_egraph_edges = false;

    if (is_egraph_node) { // expanding an egraph node
        GetEGraphStateAdjacentSuccs(this, state, egraph_node, succs, costs);

        // TODO: it should be ok to use WorkspaceLatticeEGraph's variant, but
        // it should check for collisions, which it doesn't currently
        getEGraphStateBridgeSuccs(state, egraph_node, succs, costs);

        if (enable_z_edges) {
            getEGraphStateZSuccs(state, egraph_node, succs, costs);
        }
    } else {
        getOrigStateOrigSuccs(state, succs, costs);
        if (enable_egraph_edges) {
            getOrigStateBridgeSuccs(state, succs, costs);
        }
        if (enable_z_edges) {
            // getOrigStateZSuccs(state, succs, costs);
            getOrigStateZSuccs2(state, succs, costs);
        }
    }

    auto phi_coord = getPhiCoord(state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "phi(s) = " << phi_coord);
    getGraspSuccs(state, phi_coord, succs, costs);
    getPreGraspSuccs(state, phi_coord, succs, costs);
    getPreGraspAmpSucc(state, phi_coord, succs, costs);

    auto enable_heuristic_edges = true;
    if (enable_heuristic_edges && m_heuristic != NULL) {
        std::vector<int> snap_ids;
        m_heuristic->getEquivalentStates(state_id, snap_ids);
        for (auto snap_id : snap_ids) {
            int cost;
            if (snap(state_id, snap_id, cost)) {
                succs->push_back(snap_id);
                costs->push_back(cost);
            }
        }

        std::vector<int> shortcut_ids;
        m_heuristic->getShortcutSuccs(state_id, shortcut_ids);
        for (auto shortcut_id : shortcut_ids) {
            int cost;
            if (shortcut(state_id, shortcut_id, cost)) {
                succs->push_back(shortcut_id);
                costs->push_back(cost);
            }
        }
    }
}

bool RomanObjectManipLattice::extractTransition(
    int src_id,
    int dst_id,
    std::vector<smpl::RobotState>& path)
{
    auto* state = getState(src_id);

    auto is_egraph_node = false;
    smpl::ExperienceGraph::node_id egraph_node;
    {
        auto it = m_state_to_egraph_node.find(src_id);
        if (it != end(m_state_to_egraph_node)) {
            is_egraph_node = true;
            egraph_node = it->second;
        }
    }

    auto best_cost = std::numeric_limits<int>::max();
    std::vector<smpl::RobotState> best_path;

    if (is_egraph_node) { // expanding an egraph node
        updateBestTransitionEGraphBridge(state, egraph_node, dst_id, best_cost, best_path);
        updateBestTransitionEGraphAdjacent(state, egraph_node, dst_id, best_cost, best_path);
        updateBestTransitionEGraphZ(state, egraph_node, dst_id, best_cost, best_path);
    } else {
        updateBestTransitionOrig(state, dst_id, best_cost, best_path);
        updateBestTransitionOrigBridge(state, dst_id, best_cost, best_path);
        updateBestTransitionOrigZ2(state, dst_id, best_cost, best_path);
    }

    auto phi_coord = getPhiCoord(state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "phi(s) = " << phi_coord);
    updateBestTransitionGrasp(state, phi_coord, dst_id, best_cost, best_path);
    updateBestTransitionPreGrasp(state, phi_coord, dst_id, best_cost, best_path);
    updateBestTransitionPreGraspAmp(state, phi_coord, dst_id, best_cost, best_path);

    if (m_heuristic != NULL) {
        updateBestTransitionSnap(src_id, dst_id, best_cost, best_path);
        updateBestTransitionShortcut(src_id, dst_id, best_cost, best_path);
    }

    // finding a suitable transition should produce a non-trivial path
    if (!best_path.empty()) {
        path = std::move(best_path);
        return true;
    }
    return false;
}

bool RomanObjectManipLattice::isGoal(
    const smpl::WorkspaceLatticeState* state) const
{
    smpl::WorkspaceState workspace_state;
    stateCoordToWorkspace(state->coord, workspace_state);
    return isGoal(workspace_state, state->state);
}

// Loop through a set of successor transitions and, if a transition is found
// that is cheaper than the best_cost so far, update the best_cost and the
// associated path segment. The path segment consists only of the destination
// waypoint. Return true if a cheaper transition was found. The transition type
// is added as an additional variable to end of the appended waypoint.
bool RomanObjectManipLattice::updateBestTransitionSimple(
    const std::vector<int>& succs,
    const std::vector<int>& costs,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path,
    TransitionType::Type type)
{
    for (auto i = 0; i < succs.size(); ++i) {
        auto* succ_state = getState(succs[i]);
        if (costs[i] < best_cost) {
            if ((dst_id == getGoalStateID() && isGoal(succ_state)) ||
                dst_id == succs[i])
            {
                ROS_INFO("Found transition to state %d", dst_id);
                auto wp = getState(succs[i])->state;
                wp.push_back(double(type));
                best_path = { std::move(wp) };
                best_cost = costs[i];
                return true;
            }
        }
    }
    return false;
}

void RomanObjectManipLattice::updateBestTransitionOrig(
    smpl::WorkspaceLatticeState* state,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getOrigStateOrigSuccs(state, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::OrigStateOrigSucc);
}

void RomanObjectManipLattice::updateBestTransitionOrigBridge(
    smpl::WorkspaceLatticeState* state,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getOrigStateBridgeSuccs(state, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::OrigStateBridgeSucc);
}

void RomanObjectManipLattice::updateBestTransitionOrigZ(
    smpl::WorkspaceLatticeState* state,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getOrigStateZSuccs(state, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::OrigStateZSucc);
}

void RomanObjectManipLattice::updateBestTransitionOrigZ2(
    smpl::WorkspaceLatticeState* state,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getOrigStateZSuccs2(state, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::OrigStateZSucc);
}

void RomanObjectManipLattice::updateBestTransitionEGraphBridge(
    smpl::WorkspaceLatticeState* state,
    smpl::ExperienceGraph::node_id egraph_node,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getEGraphStateBridgeSuccs(state, egraph_node, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::EGraphStateBridgeSucc);
}

void RomanObjectManipLattice::updateBestTransitionEGraphAdjacent(
    smpl::WorkspaceLatticeState* state,
    smpl::ExperienceGraph::node_id egraph_node,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getEGraphStateBridgeSuccs(state, egraph_node, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::EGraphStateAdjSucc);
}

void RomanObjectManipLattice::updateBestTransitionEGraphZ(
    smpl::WorkspaceLatticeState* state,
    smpl::ExperienceGraph::node_id egraph_node,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> succs, costs;
    getEGraphStateZSuccs(state, egraph_node, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::EGraphStateZSucc);
}

void RomanObjectManipLattice::updateBestTransitionGrasp(
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    // TODO: finer resolution
    std::vector<int> succs, costs;
    getGraspSuccs(state, phi_coord, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::GraspSucc);
}

void RomanObjectManipLattice::updateBestTransitionPreGrasp(
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    // TODO: finer resolution
    std::vector<int> succs, costs;
    getPreGraspSuccs(state, phi_coord, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::PreGraspSucc);
}

void RomanObjectManipLattice::updateBestTransitionPreGraspAmp(
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    // TODO: finer resolution
    std::vector<int> succs, costs;
    getPreGraspAmpSucc(state, phi_coord, &succs, &costs);
    updateBestTransitionSimple(succs, costs, dst_id, best_cost, best_path, TransitionType::PreGraspAmpSucc);
}

void RomanObjectManipLattice::updateBestTransitionSnap(
    int state_id,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    std::vector<int> snap_ids;
    m_heuristic->getEquivalentStates(state_id, snap_ids);

    for (auto& snap_id : snap_ids) {
        auto* snap_state = getState(snap_id);
        if ((dst_id == getGoalStateID() && isGoal(snap_state)) ||
            dst_id == snap_id)
        {
            std::vector<smpl::RobotState> snap_path;
            auto cost = getSnapMotion(state_id, snap_id, &snap_path);
            if (cost > 0 && cost < best_cost) {
                best_cost = cost;
                for (auto& wp : snap_path) {
                    wp.push_back(TransitionType::SnapSucc);
                }
                best_path = std::move(snap_path);
            }
        }
    }
}

void RomanObjectManipLattice::updateBestTransitionShortcut(
    int state_id,
    int dst_id,
    int& best_cost,
    std::vector<smpl::RobotState>& best_path)
{
    // fixed cost used for shortcut successors in WorkspaceLatticeEGraph
    auto fixed_cost = 10;
    if (fixed_cost >= best_cost) {
        return;
    }

    auto src_it = std::find(begin(m_egraph_node_to_state), end(m_egraph_node_to_state), state_id);
    if (src_it == end(m_egraph_node_to_state)) {
        return;
    }

    auto src_node = std::distance(begin(m_egraph_node_to_state), src_it);

    if (dst_id == getGoalStateID()) {
        auto nodes = m_egraph.nodes();
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            auto node = *nit;
            auto id = m_egraph_node_to_state[node];
            auto* state = getState(id);
            if (isGoal(state)) {
                std::vector<smpl::ExperienceGraph::node_id> node_path;
                if (!smpl::FindShortestExperienceGraphPath(m_egraph, src_node, node, node_path)) {
                    return;
                }

                std::vector<smpl::RobotState> shortcut_path;
                for (auto node : node_path) {
                    auto id = m_egraph_node_to_state[node];
                    auto* entry = getState(id);
                    shortcut_path.push_back(entry->state);
                }

                for (auto& wp : shortcut_path) {
                    wp.push_back(TransitionType::ShortcutSucc);
                }

                best_cost = fixed_cost;
                best_path = std::move(shortcut_path);
                return;
            }
        }
    } else {
        auto dst_it = std::find(begin(m_egraph_node_to_state), end(m_egraph_node_to_state), dst_id);
        if (dst_it == end(m_egraph_node_to_state)) {
            return;
        }

        auto dst_node = std::distance(begin(m_egraph_node_to_state), dst_it);

        std::vector<smpl::ExperienceGraph::node_id> node_path;
        if (!smpl::FindShortestExperienceGraphPath(m_egraph, src_node, dst_node, node_path)) {
            return;
        }

        std::vector<smpl::RobotState> shortcut_path;
        for (auto node : node_path) {
            auto id = m_egraph_node_to_state[node];
            auto* entry = getState(id);
            shortcut_path.push_back(entry->state);
        }
        for (auto& wp : shortcut_path) {
            wp.push_back(TransitionType::ShortcutSucc);
        }
        best_cost = fixed_cost;
        best_path = std::move(shortcut_path);
    }
}

void RomanObjectManipLattice::insertExperienceGraphPath(
    const std::vector<smpl::RobotState>& path)
{
    WorkspaceLatticeEGraph::insertExperienceGraphPath(path);

    // we're only adding one path...but going to clear and recompute all
    // auxiliary data for the entire e-graph :/
    m_phi_to_egraph_nodes.clear();
    m_pregrasp_phi_to_egraph_node.clear();
    m_grasp_phi_to_egraph_node.clear();
    m_egraph_phi_coords.clear();
    m_egraph_pre_phi_coords.clear();
    m_egraph_node_pregrasps.clear();
    m_egraph_node_grasps.clear();

    // discrete 3d positions of the end effector throughout the demonstration
    auto phi_points = std::vector<Eigen::Vector3i>();

    // discrete 3d positions of potential pre-grasp poses for the end effector
    auto pg_phi_points = std::vector<Eigen::Vector3i>();

    m_egraph_node_pregrasps.resize(m_egraph.num_nodes());
    m_egraph_node_grasps.resize(m_egraph.num_nodes());
    m_egraph_phi_coords.resize(m_egraph.num_nodes());
    m_egraph_pre_phi_coords.resize(m_egraph.num_nodes());

    m_egraph_node_validity.resize(m_egraph.num_nodes(), true);
    m_egraph_edge_validity.resize(m_egraph.num_edges(), true);

    auto nodes = m_egraph.nodes();

    auto num_invalid_nodes = 0;
    auto num_invalid_edges = 0;

    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto* checker = this->collisionChecker();
        auto state = m_egraph.state(node);
        if (!checker->isStateValid(state)) {
            m_egraph_node_validity[node] = false;
            ++num_invalid_nodes;
        }
    }

    auto edges = m_egraph.edges();

    for (auto eit = edges.first; eit != edges.second; ++eit) {
        auto e = *eit;
        auto src = m_egraph.source(e);
        auto dst = m_egraph.target(e);
        auto src_state = m_egraph.state(src);
        auto dst_state = m_egraph.state(dst);
        auto* checker = this->collisionChecker();
        if (!checker->isStateToStateValid(src_state, dst_state)) {
            m_egraph_edge_validity[e] = false;
            ++num_invalid_edges;
        }
    }

    ROS_INFO("Num valid egraph nodes: %d/%zu, Num valid egraph edges: %d/%zu",
            m_egraph.num_nodes() - num_invalid_nodes,
            m_egraph.num_nodes(),
            m_egraph.num_edges() - num_invalid_edges,
            m_egraph.num_edges());

    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& egraph_robot_state = m_egraph.state(node);

        auto tmp = smpl::WorkspaceState();
        stateRobotToWorkspace(egraph_robot_state, tmp);

        auto pregrasp_offset = smpl::MakeAffine(this->pregrasp_offset_x, 0.0, 0.0);

        auto grasp_pose = smpl::MakeAffine(tmp[0], tmp[1], tmp[2], tmp[5], tmp[4], tmp[3]);

        auto pregrasp_pose = smpl::Affine3(grasp_pose * pregrasp_offset);

        // TODO: these are stored in the state now, get them from there
        auto disc_egraph_state = smpl::WorkspaceCoord(dofCount());
        stateWorkspaceToCoord(tmp, disc_egraph_state);

        auto adj = m_egraph.adjacent_nodes(node);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            auto anode = *ait;
            auto& adj_state = m_egraph.state(anode);
            if (egraph_robot_state[RobotVariableIndex::HINGE] !=
                adj_state[RobotVariableIndex::HINGE])
            {
                // map phi(discrete egraph state) -> egraph node
                auto phi_coord = getPhiCoord(disc_egraph_state);
                m_phi_to_egraph_nodes[phi_coord].push_back(node);
                phi_points.emplace_back(phi_coord[0], phi_coord[1], phi_coord[2]);

                // map phi'(discrete egraph state) -> egraph node
                auto pre_phi_coord = getPhiCoord(pregrasp_pose);
                m_pregrasp_phi_to_egraph_node[pre_phi_coord].push_back(node);
                pg_phi_points.emplace_back(pre_phi_coord[0], pre_phi_coord[1], pre_phi_coord[2]);
                break;
            }
        }

        m_egraph_node_grasps[node] = grasp_pose;
        m_egraph_node_pregrasps[node] = pregrasp_pose;
        m_egraph_phi_coords[node] = getPhiCoord(grasp_pose);
        m_egraph_pre_phi_coords[node] = getPhiCoord(pregrasp_pose);
    }

    auto phi_markers = std::vector<smpl::visual::Marker>();
    for (auto& grasp_pose : m_egraph_node_grasps) {
        auto vis_name = "phi";
        auto more_markers = smpl::visual::MakeFrameMarkers(grasp_pose, "map", vis_name, phi_markers.size());
        phi_markers.insert(end(phi_markers), begin(more_markers), end(more_markers));
    }
    SV_SHOW_INFO_NAMED("phi", std::move(phi_markers));

    auto phi_points_cont = std::vector<smpl::Vector3>();
    phi_points_cont.reserve(phi_points.size());
    for (auto& point : phi_points) {
        double pcont[3];
        posCoordToWorkspace(point.data(), pcont);
        phi_points_cont.emplace_back(pcont[0], pcont[1], pcont[2]);
    }

    auto pre_phi_points_cont = std::vector<smpl::Vector3>();
    pre_phi_points_cont.reserve(pg_phi_points.size());
    for (auto& point : pg_phi_points) {
        double pcont[3];
        posCoordToWorkspace(point.data(), pcont);
        pre_phi_points_cont.emplace_back(pcont[0], pcont[1], pcont[2]);
    }

    auto vis_name = "phi_points";
    SV_SHOW_INFO_NAMED(
            vis_name,
            MakeCubesMarker(
                    std::move(phi_points_cont),
                    resolution()[0],
                    smpl::visual::Color{ 0.5, 0.5, 0.5, 1.0f },
                    "map",
                    vis_name));
    SV_SHOW_INFO_NAMED(
            "pre_phi_points",
            MakeCubesMarker(
                std::move(pre_phi_points_cont),
                resolution()[0],
                smpl::visual::Color{ 1.0f, 0.5f, 0.5f, 1.0f },
                "map",
                "pre_phi_points"));
}

void RomanObjectManipLattice::clearExperienceGraph()
{
    m_phi_to_egraph_nodes.clear();
    m_pregrasp_phi_to_egraph_node.clear();
    m_grasp_phi_to_egraph_node.clear();
    m_egraph_phi_coords.clear();
    m_egraph_pre_phi_coords.clear();
    m_egraph_node_pregrasps.clear();
    m_egraph_node_grasps.clear();
    m_egraph_node_validity.clear();
    m_egraph_edge_validity.clear();
    WorkspaceLatticeEGraph::clearExperienceGraph();
}

// Attempt to generate a snap motion between two states. The source and
// destination states are assumed to have the same phi-coordinates, which are
// the same phi-coordinates as one of the demonstration states. (These are the
// snap destinations generated by ObjectManipHeuristic). The snap motion itself
// is described as:
//
// if end effector is not on the handle:
// ....drive to base pose
// ....interpolate torso/arm motion only to line up with the demonstration
// else:
// ....interpolate torso/arm motion only to line up with the demonstration
//
// If $path is non-null, the intermediate waypoints of the snap motion are
// extracted.
//
// A negative value is returned if the snap motion is infeasible, otherwise
// the cost of the motion is returned.
int RomanObjectManipLattice::getSnapMotion(
    int src_id,
    int dst_id,
    std::vector<smpl::RobotState>* path)
{
    SMPL_DEBUG_NAMED(G_SNAP_LOG, "snap(%d, %d)", src_id, dst_id);

    auto* src_state = getState(src_id);
    auto* dst_state = getState(dst_id);

    SMPL_ASSERT(src_state != NULL && dst_state != NULL);
    SMPL_ASSERT(src_state->state.size() == VARIABLE_COUNT);
    SMPL_ASSERT(dst_state->state.size() == VARIABLE_COUNT);
    SMPL_ASSERT(src_state->coord.size() == VARIABLE_COUNT);
    SMPL_ASSERT(dst_state->coord.size() == VARIABLE_COUNT);

    // log source/destination states
    SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, "  snap " << src_state->coord << " -> " << dst_state->coord);

    // visualize source/destination states
    auto* vis_name = "snap";
    SV_SHOW_DEBUG_NAMED(vis_name, color(getStateVisualization(src_state->state, "snap_from"), smpl::visual::Color{ 1.0f, 0.5f, 0.0f, 0.8f }));
    SV_SHOW_DEBUG_NAMED(vis_name, color(getStateVisualization(dst_state->state, "snap_to"), smpl::visual::Color{ 0.0f, 0.5f, 1.0f, 0.8f }));

    auto dx = dst_state->state[WORLD_JOINT_X] - src_state->state[WORLD_JOINT_X];
    auto dy = dst_state->state[WORLD_JOINT_Y] - src_state->state[WORLD_JOINT_Y];
    auto thresh = 1e-4;
    if (std::fabs(dx) > thresh | std::fabs(dy) > thresh) {
        auto heading = atan2(dy, dx);
        auto alt_heading = heading + M_PI;
        auto thresh = smpl::to_radians(10.0);
        if (smpl::shortest_angle_dist(heading, src_state->state[WORLD_JOINT_THETA]) > thresh &&
            smpl::shortest_angle_dist(alt_heading, src_state->state[WORLD_JOINT_THETA]) > thresh)
        {
            SMPL_DEBUG_NAMED(G_SNAP_LOG, "  Skip! (requires base motion)");
            return -1; //false;
        }
    }

    smpl::WorkspaceState start_state;
    smpl::WorkspaceState finish_state;
    stateCoordToWorkspace(src_state->coord, start_state);
    stateCoordToWorkspace(dst_state->coord, finish_state);

    SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, "  Interpolate between states " << start_state << " and " << finish_state);

    auto prev_fa = start_state[AR_FA];

    smpl::RobotState prev_robot_state;

    auto num_waypoints = 20;
    for (auto i = 0; i < num_waypoints; ++i) {
        // Construct a seed state by interpolating between the start and final
        // snap state. Run IK to determine the free angles that it is allowed
        // to change
        smpl::WorkspaceState interm_workspace_state;
        interm_workspace_state.resize(dofCount());

        // interpolate x, y, z, r, p, y too?

        // x, y, z, R, P, Y, FA1 same as start
        interm_workspace_state[EE_PX] = finish_state[EE_PX];
        interm_workspace_state[EE_PY] = finish_state[EE_PY];
        interm_workspace_state[EE_PZ] = finish_state[EE_PZ];
        interm_workspace_state[EE_QX] = finish_state[EE_QX];
        interm_workspace_state[EE_QY] = finish_state[EE_QY];
        interm_workspace_state[EE_QZ] = finish_state[EE_QZ];
        interm_workspace_state[AR_FA] = prev_fa; //finish_state[AR_FA];

        auto interp = [](double a, double b, double t) {
            return (1.0 - t) * a + t * b;
        };

        auto t = (double)i / (double)(num_waypoints - 1);
        // interpolate torso, theta, x, y
        interm_workspace_state[TR_JP] = interp(start_state[TR_JP], finish_state[TR_JP], t);
        interm_workspace_state[BD_PZ] = interp(start_state[BD_PZ], finish_state[BD_PZ], t);
        interm_workspace_state[BD_TH] = interp(start_state[BD_TH], finish_state[BD_TH], t);
        interm_workspace_state[BD_PX] = interp(start_state[BD_PX], finish_state[BD_PX], t);
        interm_workspace_state[BD_PY] = interp(start_state[BD_PY], finish_state[BD_PY], t);

        // hinge kept the same
        interm_workspace_state[OB_P] = finish_state[OB_P];

        // TODO: this should be permissive and allow moving the redundant angles
        auto stateWorkspaceToRobotPermissive = [&](
            const smpl::WorkspaceState& state,
            smpl::RobotState& ostate)
        {
            auto seed = src_state->state;
            for (size_t fai = 0; fai < freeAngleCount(); ++fai) {
                seed[m_fangle_indices[fai]] = state[6 + fai];
            }

            Eigen::Affine3d pose =
                    Eigen::Translation3d(state[0], state[1], state[2]) *
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX());

            return m_ik_iface->computeIK(pose, seed, ostate);
        };

        smpl::RobotState robot_state;
        if (!stateWorkspaceToRobotPermissive(interm_workspace_state, robot_state)) {
//        if (!stateWorkspaceToRobot(interm_workspace_state, robot_state)) {
            SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, " -> Failed to find ik solution for interpolated state " << interm_workspace_state);
            return -1;
        } else {
            SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, " -> Found ik solution for interpolated state " << interm_workspace_state);
        }
        prev_fa = robot_state[LIMB_JOINT3];

        if (i > 0 && !collisionChecker()->isStateToStateValid(prev_robot_state, robot_state)) {
            SMPL_DEBUG_NAMED(G_SNAP_LOG, " -> Failed snap!");
            return -1;
        }

        prev_robot_state = robot_state;

        if (path != NULL) {
            path->push_back(std::move(robot_state));
        }
    }

    SMPL_DEBUG_NAMED(G_SNAP_LOG, "  Snap %d -> %d!", src_id, dst_id);
    return 10;
}

bool RomanObjectManipLattice::trySnap(int src_id, int dst_id, int& cost)
{
    auto c = getSnapMotion(src_id, dst_id, NULL);

    if (c >= 0) {
        cost = c;
        return true;
    }

    return false;
}

#define PHI_INCLUDE_RP 1

auto RomanObjectManipLattice::getPhiCoord(const Eigen::Affine3d& pose) const
    -> PhiCoord
{
#if PHI_INCLUDE_RP
    PhiCoord coord(6);
#else
    PhiCoord coord(4);
#endif

    posWorkspaceToCoord(pose.translation().data(), coord.data());

    double ea[3];
    smpl::get_euler_zyx(Eigen::Matrix3d(pose.rotation()), ea[2], ea[1], ea[0]);

    int ea_disc[3];
    rotWorkspaceToCoord(ea, ea_disc);

#if PHI_INCLUDE_RP
    coord[3] = ea_disc[0];
    coord[4] = ea_disc[1];
    coord[5] = ea_disc[2];
#else
    coord[3] = ea_disc[2];
#endif

    return coord;
}

auto RomanObjectManipLattice::getPhiState(const Eigen::Affine3d& pose) const
    -> PhiState
{
#if PHI_INCLUDE_RP
    PhiState state(6);
#else
    PhiState state(4);
#endif

    state[0] = pose.translation().x();
    state[1] = pose.translation().y();
    state[2] = pose.translation().z();

    double y, p, r;
    smpl::get_euler_zyx(Eigen::Matrix3d(pose.rotation()), y, p, r);

#if PHI_INCLUDE_RP
    state[3] = r;
    state[4] = p;
    state[5] = y;
#else
    state[3] = y;
#endif

    return state;
}

auto RomanObjectManipLattice::getPhiCoord(const smpl::WorkspaceCoord& coord) const
    -> PhiCoord
{
#if PHI_INCLUDE_RP
    return PhiCoord{ coord[0], coord[1], coord[2], coord[3], coord[4], coord[5] };
#else
    return PhiCoord{ coord[0], coord[1], coord[2], coord[5] };
#endif
}

auto GetPhiState(
    const RomanObjectManipLattice* graph,
    const smpl::WorkspaceState& state)
    -> PhiState
{
#if PHI_INCLUDE_RP
    return PhiState{ state[0], state[1], state[2], state[3], state[4], state[5] };
#else
    return PhiState{ state[0], state[1], state[2], state[5] };
#endif
};


bool RomanObjectManipLattice::snap(int src_id, int dst_id, int& cost)
{
    return trySnap(src_id, dst_id, cost);
}

bool RomanObjectManipLattice::loadExperienceGraph(const std::string& path)
{
    if (!WorkspaceLatticeEGraph::loadExperienceGraph(path)) {
        return false;
    }

    // discrete 3d positions of the end effector throughout the demonstration
    auto phi_points = std::vector<Eigen::Vector3i>();

    // discrete 3d positions of potential pre-grasp poses for the end effector
    auto pg_phi_points = std::vector<Eigen::Vector3i>();

    m_egraph_node_pregrasps.resize(m_egraph.num_nodes());
    m_egraph_node_grasps.resize(m_egraph.num_nodes());
    m_egraph_phi_coords.resize(m_egraph.num_nodes());
    m_egraph_pre_phi_coords.resize(m_egraph.num_nodes());

    auto nodes = m_egraph.nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& egraph_robot_state = m_egraph.state(node);

        auto tmp = smpl::WorkspaceState();
        stateRobotToWorkspace(egraph_robot_state, tmp);

        auto pregrasp_offset = smpl::MakeAffine(this->pregrasp_offset_x, 0.0, 0.0);

        auto grasp_pose = smpl::MakeAffine(
                tmp[0], tmp[1], tmp[2], tmp[5], tmp[4], tmp[3]);

        auto pregrasp_pose = smpl::Affine3(grasp_pose * pregrasp_offset);

        // TODO: these are stored in the state now, get them from there
        auto disc_egraph_state = smpl::WorkspaceCoord(dofCount());
        stateWorkspaceToCoord(tmp, disc_egraph_state);

        auto adj = m_egraph.adjacent_nodes(node);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            auto anode = *ait;
            auto& adj_state = m_egraph.state(anode);
            if (egraph_robot_state[RobotVariableIndex::HINGE] !=
                adj_state[RobotVariableIndex::HINGE])
            {
                // map phi(discrete egraph state) -> egraph node
                auto phi_coord = getPhiCoord(disc_egraph_state);
                m_phi_to_egraph_nodes[phi_coord].push_back(node);
                phi_points.emplace_back(phi_coord[0], phi_coord[1], phi_coord[2]);

                // map phi'(discrete egraph state) -> egraph node
                auto pre_phi_coord = getPhiCoord(pregrasp_pose);
                m_pregrasp_phi_to_egraph_node[pre_phi_coord].push_back(node);
                pg_phi_points.emplace_back(pre_phi_coord[0], pre_phi_coord[1], pre_phi_coord[2]);
                break;
            }
        }

        m_egraph_node_grasps[node] = grasp_pose;
        m_egraph_node_pregrasps[node] = pregrasp_pose;
        m_egraph_phi_coords[node] = getPhiCoord(grasp_pose);
        m_egraph_pre_phi_coords[node] = getPhiCoord(pregrasp_pose);
    }

    auto phi_markers = std::vector<smpl::visual::Marker>();
    for (auto& grasp_pose : m_egraph_node_grasps) {
        auto vis_name = "phi";
        auto more_markers = smpl::visual::MakeFrameMarkers(grasp_pose, "map", vis_name, phi_markers.size());
        phi_markers.insert(end(phi_markers), begin(more_markers), end(more_markers));
    }
    SV_SHOW_INFO_NAMED("phi", std::move(phi_markers));

    auto phi_points_cont = std::vector<smpl::Vector3>();
    phi_points_cont.reserve(phi_points.size());
    for (auto& point : phi_points) {
        double pcont[3];
        posCoordToWorkspace(point.data(), pcont);
        phi_points_cont.emplace_back(pcont[0], pcont[1], pcont[2]);
    }

    auto pre_phi_points_cont = std::vector<smpl::Vector3>();
    pre_phi_points_cont.reserve(pg_phi_points.size());
    for (auto& point : pg_phi_points) {
        double pcont[3];
        posCoordToWorkspace(point.data(), pcont);
        pre_phi_points_cont.emplace_back(pcont[0], pcont[1], pcont[2]);
    }

    auto vis_name = "phi_points";
    SV_SHOW_INFO_NAMED(
            vis_name,
            MakeCubesMarker(
                    std::move(phi_points_cont),
                    resolution()[0],
                    smpl::visual::Color{ 0.5, 0.5, 0.5, 1.0f },
                    "map",
                    vis_name));
    SV_SHOW_INFO_NAMED(
            "pre_phi_points",
            MakeCubesMarker(
                std::move(pre_phi_points_cont),
                resolution()[0],
                smpl::visual::Color{ 1.0f, 0.5f, 0.5f, 1.0f },
                "map",
                "pre_phi_points"));

    return true;
}

bool RomanObjectManipLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<smpl::RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << ids);

    if (ids.empty()) return true;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (ids.size() == 1) {
        auto state_id = ids[0];

        if (state_id == getGoalStateID()) {
            auto* entry = getState(getStartStateID());
            SMPL_ASSERT(entry != NULL);
            path.push_back(entry->state);
        } else {
            auto* entry = getState(state_id);
            SMPL_ASSERT(entry != NULL);
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
        return true;
    }

    if (ids[0] == getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<smpl::RobotState> opath;

    // grab the first point
    {
        auto* entry = getState(ids[0]);
        SMPL_ASSERT(entry != NULL);
        auto first_state = entry->state;
        first_state.push_back(double(TransitionType::OrigStateOrigSucc));
        opath.push_back(first_state);
    }

    // grab the rest of the points
    for (auto i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        std::vector<smpl::RobotState> motion;
        if (!extractTransition(prev_id, curr_id, motion)) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to find valid action to successor during path extraction");
        }

        auto type = !motion.empty() ? (TransitionType::Type)(motion.back().back()) : TransitionType::Unknown;
        ROS_INFO("Found transition %d -> %d of type %s with %zu points", prev_id, curr_id, to_cstring(type), motion.size());
        ROS_INFO_STREAM("  motion: " << motion);

        for (auto& point : motion) {
            opath.push_back(std::move(point));
        }
    }

    // we made it!
    path = std::move(opath);

    SMPL_DEBUG_NAMED(G_LOG, "Final path:");
    for (auto& point : path) {
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  " << point);
    }

    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    return true;
}

void RomanObjectManipLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    auto prev_count = succs->size();
    getUniqueSuccs(state_id, succs, costs);
    auto curr_count = succs->size();
    for (auto i = prev_count; i != curr_count; ++i) {
        auto* state = getState((*succs)[i]);
        smpl::WorkspaceState workspace_state;
        stateCoordToWorkspace(state->coord, workspace_state);
        if (isGoal(workspace_state, state->state)) {
            (*succs)[i] = getGoalStateID();
        }
    }
}
