#include "roman_workspace_lattice_egraph.h"

#include <smpl/angles.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/workspace_lattice_action_space.h>

#include "assert.h"
#include "variables.h"

#define G_SNAP_LOG G_SUCCESSORS_LOG ".snap"

// Color a set of markers.
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
static
void GetEGraphStateZSuccs(
    RomanWorkspaceLatticeEGraph* graph,
    smpl::WorkspaceLatticeState* state,
    smpl::ExperienceGraph::node_id egraph_node,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
    // TODO: E_z?
}

// Get successors of actions from E_z where u is not an E-Graph state.
static
void GetOrigStateZSuccs(
    RomanWorkspaceLatticeEGraph* graph,
    smpl::WorkspaceLatticeState* state,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
    // e-graph nodes within phi tolerance, for E_z
    std::vector<smpl::ExperienceGraph::node_id> parent_nearby_nodes;
    {
        auto pose_coord = GetPhiCoord(graph, state->coord);
        SMPL_DEBUG_STREAM("parent pose coord = " << pose_coord);
        auto it = graph->phi_to_egraph_nodes.find(pose_coord);
        if (it != end(graph->phi_to_egraph_nodes)) {
            for (auto node : it->second) {
                auto z = graph->egraph.state(node)[HINGE];
                if (z == state->state[HINGE]) {
                    parent_nearby_nodes.push_back(node);
                }
            }
        }
    }

    std::vector<smpl::WorkspaceAction> actions;
    graph->m_actions->apply(*state, actions);

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        smpl::RobotState final_robot_state;
        if (!graph->checkAction(state->state, action, &final_robot_state)) {
            continue;
        }

        auto& final_state = action.back();
        smpl::WorkspaceCoord succ_coord;
        graph->stateWorkspaceToCoord(final_state, succ_coord);

        // E_z from V_orig
        if (false && !parent_nearby_nodes.empty()) {
            auto pose_coord = GetPhiCoord(graph, succ_coord);
            auto it = graph->phi_to_egraph_nodes.find(pose_coord);
            if (it != end(graph->phi_to_egraph_nodes)) {
                // for each experience graph node within error...
                for (auto node : it->second) {
                    // if node in the adjacent list of any nearby node...
                    for (auto nn : parent_nearby_nodes) {
                        if (graph->egraph.edge(nn, node)) {
                            auto& egraph_state = graph->egraph.state(node);
                            SMPL_ASSERT(egraph_state.size() == graph->robot()->jointVariableCount());
                            auto z = egraph_state[HINGE];
                            auto this_final_state = final_state;
                            auto this_final_robot_state = final_robot_state;
                            this_final_state[OB_P] = z;
                            this_final_robot_state[HINGE] = z;
                            smpl::WorkspaceCoord succ_coord;
                            graph->stateWorkspaceToCoord(this_final_state, succ_coord);
                            auto succ_id = graph->createState(succ_coord);
                            auto* succ_state = graph->getState(succ_id);
                            succ_state->state = this_final_robot_state;
                            SMPL_DEBUG_NAMED(G_LOG, "Return Z-EDGE z = %f", z);
                            if (!unique && graph->isGoal(this_final_state, this_final_robot_state)) {
                                succs->push_back(graph->getGoalStateID());
                            } else {
                                succs->push_back(succ_id);
                            }
                            auto edge_cost = graph->computeCost(*state, *succ_state);
                            costs->push_back(edge_cost);
                        }
                    }
                }
            }
        }
    }
}

// Apply an adaptive motion to a state that moves the end effector to the
// nearest pre-grasp pose of any state in the demonstration with the same
// z-value.
static
void GetPreGraspAmpSucc(
    RomanWorkspaceLatticeEGraph* graph,
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
    auto closest = smpl::ExperienceGraph::node_id(-1);
    auto best = std::numeric_limits<int>::max();
    auto nodes = graph->egraph.nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        if (graph->egraph.state(node)[HINGE] != state->state[HINGE]) continue;

        auto& egraph_phi = graph->egraph_pre_phi_coords[node];
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
        SMPL_INFO_NAMED(G_SUCCESSORS_LOG, "Attempt adaptive motion to pre-grasp");
        auto& pregrasp = graph->egraph_node_pregrasps[closest];

        SV_SHOW_INFO_NAMED("pregrasp_target", smpl::visual::MakeFrameMarkers(pregrasp, "map", "pregrasp_target"));

        auto seed = state->state;
        smpl::RobotState final_robot_state;
        if (graph->m_ik_iface->computeIK(pregrasp, seed, final_robot_state)) {
            SMPL_INFO_NAMED(G_SUCCESSORS_LOG, "  Adaptive motion to pre-grasp succeeded");

            // TODO: should do collision checking here
            if (!graph->collisionChecker()->isStateToStateValid(state->state, final_robot_state)) {
                return;
            }

            smpl::WorkspaceState succ_workspace_state;
            graph->stateRobotToWorkspace(final_robot_state, succ_workspace_state);

            smpl::WorkspaceCoord succ_coord;
            graph->stateWorkspaceToCoord(succ_workspace_state, succ_coord);
            auto succ_id = graph->createState(succ_coord);
            auto* succ_state = graph->getState(succ_id);
            succ_state->state = final_robot_state;

            // TODO: We could check whether this state is a goal state
            // or not, but we should have found the goal state already if we
            // are at a state with the goal z-value. This might be an issue if
            // we run the planner with start = goal?

            succs->push_back(succ_id);
            costs->push_back(1); // edge cost of one for grasp/pregrasp actions
        }
    }
}

static
void GetPreGraspSuccs(
    RomanWorkspaceLatticeEGraph* graph,
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
    // If the contact coordinates are the same as some node on the
    // demonstration, then an action is available to "release" the object
    // and move the end effector to the pre-grasp pose.
    auto it = graph->phi_to_egraph_nodes.find(phi_coord);
    if (it != end(graph->phi_to_egraph_nodes)) {
        for (auto node : it->second) {
            if (graph->egraph.state(node)[HINGE] != state->state[HINGE]) continue;

            SMPL_INFO_NAMED(G_SUCCESSORS_LOG, "Attempt pre-grasp motion");
            auto& pregrasp = graph->egraph_node_pregrasps[node];

            // add an action that "releases" the object and moves to the post-grasp

            // allowed to move the torso and right arm, but not the base...so the
            // ik group
            auto seed = state->state;
            smpl::RobotState final_robot_state;
            if (graph->m_ik_iface->computeIK(pregrasp, seed, final_robot_state)) {
                // TODO: no collision checking here. We explicitly don't want to
                // run nominal collision checking, because it is expected that
                // there will be collisions between the robot and the object when
                // the object is grasped. It might be worthwhile to at least check
                // the post-grasp endpoint for collisions.

                smpl::WorkspaceState succ_workspace_state;
                graph->stateRobotToWorkspace(final_robot_state, succ_workspace_state);

                smpl::WorkspaceCoord succ_coord;
                graph->stateWorkspaceToCoord(succ_workspace_state, succ_coord);
                auto succ_id = graph->createState(succ_coord);
                auto* succ_state = graph->getState(succ_id);
                succ_state->state = final_robot_state;

                // TODO: We could check whether this state is a goal state
                // or not, but we should have found the goal state already if we
                // are at a state with the goal z-value. This might be an issue if
                // we run the planner with start = goal?

                SMPL_INFO_NAMED(G_SUCCESSORS_LOG, "Pre-grasp motion succeeded");
                succs->push_back(succ_id);
                costs->push_back(1); // edge cost of one for grasp/pregrasp actions
            }
        }
    }
}

// For all states v in the demonstration where z(v) = z(s), if pre-phi(v) =
// phi(s), apply an adaptive motion to move the end effector from phi(s) to
// phi(v).
static
void GetGraspSuccs(
    RomanWorkspaceLatticeEGraph* graph,
    smpl::WorkspaceLatticeState* state,
    const PhiCoord& phi_coord,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
    // If the contact coordinates are the same as the pregrasp coordinates of
    // some state in the demonstration, then an action is available to "grasp"
    // the object by moving the end effector to the grasp pose.
    auto pre_it = graph->pre_phi_to_egraph_nodes.find(phi_coord);
    if (pre_it != end(graph->pre_phi_to_egraph_nodes)) {
        for (auto node : pre_it->second) {
            if (graph->egraph.state(node)[HINGE] != state->state[HINGE]) continue;

            SMPL_INFO_NAMED(G_SUCCESSORS_LOG, "Attempt grasp motion");
            auto& grasp = graph->egraph_node_grasps[node];
            auto seed = state->state;
            // NOTE: the final robot state has the same
            smpl::RobotState final_robot_state;
            if (graph->m_ik_iface->computeIK(grasp, seed, final_robot_state)) {
                smpl::WorkspaceState succ_workspace_state;
                graph->stateRobotToWorkspace(final_robot_state, succ_workspace_state);

                smpl::WorkspaceCoord succ_coord;
                graph->stateWorkspaceToCoord(succ_workspace_state, succ_coord);

                auto succ_id = graph->createState(succ_coord);
                auto* succ_state = graph->getState(succ_id);
                succ_state->state = final_robot_state;

                SMPL_INFO_NAMED(G_SUCCESSORS_LOG, "Grasp motion succeeded");
                succs->push_back(succ_id);
                costs->push_back(1);
            }
        }
    }
}

static
void GetSuccs(
    RomanWorkspaceLatticeEGraph* graph,
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    bool unique)
{
#if 0
    // Get successors of actions from E_orig, E_demo, and E_bridge.
    GetSuccs((smpl::WorkspaceLatticeEGraph*)graph, state_id, succs, costs, unique);
#else
    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Expand state %d", state_id);
    auto* state = graph->getState(state_id);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  workspace coord: " << state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "      robot state: " << state->state);

    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, graph->getStateVisualization(state->state, vis_name));

    auto is_egraph_node = false;
    smpl::ExperienceGraph::node_id egraph_node;
    {
        auto it = graph->state_to_egraph_node.find(state_id);
        if (it != end(graph->state_to_egraph_node)) {
            is_egraph_node = true;
            egraph_node = it->second;
        }
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  egraph state: %s", is_egraph_node ? "true" : "false");

    if (is_egraph_node) { // expanding an egraph node
        GetEGraphStateAdjacentSuccs(graph, state, egraph_node, succs, costs, unique);
        GetEGraphStateBridgeSuccs(graph, state, egraph_node, succs, costs, unique);
//        GetEGraphStateZSuccs(graph, state, egraph_node, succs, costs, unique);
    } else {
        GetOrigStateOrigSuccs(graph, state, succs, costs, unique);
        GetOrigStateBridgeSuccs(graph, state, succs, costs, unique);
//        GetOrigStateZSuccs(graph, state, succs, costs, unique);
    }

    auto phi_coord = GetPhiCoord(graph, state->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "phi(s) = " << phi_coord);
    GetGraspSuccs(graph, state, phi_coord, succs, costs, unique);
    GetPreGraspSuccs(graph, state, phi_coord, succs, costs, unique);
    GetPreGraspAmpSucc(graph, state, phi_coord, succs, costs, unique);
#endif
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
static
int GetSnapMotion(
    RomanWorkspaceLatticeEGraph* graph,
    int src_id,
    int dst_id,
    std::vector<smpl::RobotState>* path)
{
    SMPL_DEBUG_NAMED(G_SNAP_LOG, "snap(%d, %d)", src_id, dst_id);

    auto* src_state = graph->getState(src_id);
    auto* dst_state = graph->getState(dst_id);

    SMPL_ASSERT(src_state != NULL && dst_state != NULL);
    SMPL_ASSERT(src_state->state.size() == VARIABLE_COUNT);
    SMPL_ASSERT(dst_state->state.size() == VARIABLE_COUNT);
    SMPL_ASSERT(src_state->coord.size() == VARIABLE_COUNT);
    SMPL_ASSERT(dst_state->coord.size() == VARIABLE_COUNT);

    // log source/destination states
    SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, "  snap " << src_state->coord << " -> " << dst_state->coord);

    // visualize source/destination states
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, color(graph->getStateVisualization(src_state->state, "snap_from"), smpl::visual::Color{ 1.0f, 0.5f, 0.0f, 0.8f }));
    SV_SHOW_INFO_NAMED(vis_name, color(graph->getStateVisualization(dst_state->state, "snap_to"), smpl::visual::Color{ 0.0f, 0.5f, 1.0f, 0.8f }));

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
    graph->stateCoordToWorkspace(src_state->coord, start_state);
    graph->stateCoordToWorkspace(dst_state->coord, finish_state);

    SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, "  Interpolate between states " << start_state << " and " << finish_state);

    auto prev_fa = start_state[AR_FA];

    smpl::RobotState prev_robot_state;

    auto num_waypoints = 20;
    for (auto i = 0; i < num_waypoints; ++i) {
        // Construct a seed state by interpolating between the start and final
        // snap state. Run IK to determine the free angles that it is allowed
        // to change
        smpl::WorkspaceState interm_workspace_state;
        interm_workspace_state.resize(graph->dofCount());

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
            for (size_t fai = 0; fai < graph->freeAngleCount(); ++fai) {
                seed[graph->m_fangle_indices[fai]] = state[6 + fai];
            }

            Eigen::Affine3d pose =
                    Eigen::Translation3d(state[0], state[1], state[2]) *
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX());

            return graph->m_ik_iface->computeIK(pose, seed, ostate);
        };

        smpl::RobotState robot_state;
        if (!stateWorkspaceToRobotPermissive(interm_workspace_state, robot_state)) {
//        if (!graph->stateWorkspaceToRobot(interm_workspace_state, robot_state)) {
            SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, " -> Failed to find ik solution for interpolated state " << interm_workspace_state);
            return -1;
        } else {
            SMPL_DEBUG_STREAM_NAMED(G_SNAP_LOG, " -> Found ik solution for interpolated state " << interm_workspace_state);
        }
        prev_fa = robot_state[LIMB_JOINT3];

        if (i > 0 && !graph->collisionChecker()->isStateToStateValid(prev_robot_state, robot_state)) {
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

static
bool TrySnap(RomanWorkspaceLatticeEGraph* graph, int src_id, int dst_id, int& cost)
{
    auto c = GetSnapMotion(graph, src_id, dst_id, NULL);

    if (c >= 0) {
        cost = c;
        return true;
    }

    return false;
}

static
bool ExtractPath(
    RomanWorkspaceLatticeEGraph* graph,
    const std::vector<int>& ids,
    std::vector<smpl::RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << ids);

    if (ids.empty()) return true;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (ids.size() == 1) {
        auto state_id = ids[0];

        if (state_id == graph->getGoalStateID()) {
            auto* entry = graph->getState(graph->getStartStateID());
            SMPL_ASSERT(entry != NULL);
            path.push_back(entry->state);
        } else {
            auto* entry = graph->getState(state_id);
            SMPL_ASSERT(entry != NULL);
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, graph->getStateVisualization(path.back(), vis_name));
        return true;
    }

    if (ids[0] == graph->getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<smpl::RobotState> opath;

    // grab the first point
    {
        auto* entry = graph->getState(ids[0]);
        SMPL_ASSERT(entry != NULL);
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == graph->getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        // find the successor state corresponding to the cheapest valid action

        // TODO: return an iterator here to avoid collision checking all
        // successors
        std::vector<int> succs, costs;
        GetSuccs(graph, prev_id, &succs, &costs, true);

        smpl::WorkspaceLatticeState* best_state = NULL;
        auto best_cost = std::numeric_limits<int>::max();
        for (size_t ii = 0; ii < succs.size(); ++ii) {
            if (curr_id == graph->getGoalStateID()) {
                auto* state = graph->getState(succs[ii]);
                SMPL_ASSERT(state != NULL);
                smpl::WorkspaceState workspace_state;
                graph->stateRobotToWorkspace(state->state, workspace_state);
                if (costs[ii] < best_cost && graph->isGoal(workspace_state, state->state)) {
                    best_state = state;
                    best_cost = costs[ii];
                }
            } else {
                if (succs[ii] == curr_id && costs[ii] < best_cost) {
                    best_state = graph->getState(succs[ii]);
                    best_cost = costs[ii];
                }
            }
        }

        if (best_state != NULL) {
            SMPL_DEBUG_STREAM_NAMED(G_LOG, "Found normal action to successor (coord = " << best_state->coord << ", state = " << best_state->state << ")");
            opath.push_back(best_state->state);
            continue;
        }

        SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut successor");

        auto found = false;
        // check for shortcut transition
        auto pnit = std::find(begin(graph->egraph_node_to_state), end(graph->egraph_node_to_state), prev_id);
        auto cnit = std::find(begin(graph->egraph_node_to_state), end(graph->egraph_node_to_state), curr_id);
        if (pnit != end(graph->egraph_node_to_state) &&
            cnit != end(graph->egraph_node_to_state))
        {
            // position in node array is synonymous with e-graph node id
            auto prev_node = std::distance(begin(graph->egraph_node_to_state), pnit);
            auto curr_node = std::distance(begin(graph->egraph_node_to_state), cnit);

            SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, prev_node, curr_node);

            std::vector<smpl::ExperienceGraph::node_id> node_path;
            found = FindShortestExperienceGraphPath(graph->egraph, prev_node, curr_node, node_path);
            if (found) {
                SMPL_DEBUG_NAMED(G_LOG, "Found shortcut action with %zu waypoints to successor", node_path.size());
                for (auto n : node_path) {
                    auto state_id = graph->egraph_node_to_state[n];
                    auto* entry = graph->getState(state_id);
                    SMPL_ASSERT(entry != NULL);
                    opath.push_back(entry->state);
                }
            }
        }

        if (found) continue;

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        {
            auto prev_size = opath.size();
            auto cost = GetSnapMotion(graph, prev_id, curr_id, &opath);
            if (cost >= 0) {
                auto curr_size = opath.size();
                SMPL_DEBUG_NAMED(G_LOG, "Found snap action with %zu waypoints to successor", curr_size - prev_size);
                continue;
            }
        }

        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid action to successor during path extraction");
#if 1
        return false;
#else
        auto* giveup = graph->getState(curr_id);
        opath.push_back(giveup->state);
#endif
    }

    // we made it!
    path = std::move(opath);

    SMPL_DEBUG_NAMED(G_LOG, "Final path:");
    for (auto& point : path) {
        SMPL_DEBUG_STREAM_NAMED(G_LOG, "  " << point);
    }

    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, graph->getStateVisualization(path.back(), vis_name));
    return true;
}

#define PHI_INCLUDE_RP 1

static
auto GetPhiCoord(
    const RomanWorkspaceLatticeEGraph* graph,
    const Eigen::Affine3d& pose)
    -> PhiCoord
{
#if PHI_INCLUDE_RP
    PhiCoord coord(6);
#else
    PhiCoord coord(4);
#endif

    graph->posWorkspaceToCoord(pose.translation().data(), coord.data());

    double ea[3];
    smpl::get_euler_zyx(Eigen::Matrix3d(pose.rotation()), ea[2], ea[1], ea[0]);

    int ea_disc[3];
    graph->rotWorkspaceToCoord(ea, ea_disc);

#if PHI_INCLUDE_RP
    coord[3] = ea_disc[0];
    coord[4] = ea_disc[1];
    coord[5] = ea_disc[2];
#else
    coord[3] = ea_disc[2];
#endif

    return coord;
}

static
auto GetPhiState(
    const RomanWorkspaceLatticeEGraph* graph,
    const Eigen::Affine3d& pose)
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

bool LoadExperienceGraph(
    RomanWorkspaceLatticeEGraph* graph,
    const std::string& path)
{
    if (!graph->WorkspaceLatticeEGraph::loadExperienceGraph(path)) {
        return false;
    }

    std::vector<Eigen::Vector3i> phi_points;
    std::vector<Eigen::Vector3i> pg_phi_points;

    graph->egraph_node_pregrasps.resize(graph->egraph.num_nodes());
    graph->egraph_node_grasps.resize(graph->egraph.num_nodes());
    graph->egraph_phi_coords.resize(graph->egraph.num_nodes());
    graph->egraph_pre_phi_coords.resize(graph->egraph.num_nodes());

    auto nodes = graph->egraph.nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& egraph_robot_state = graph->egraph.state(node);

        smpl::WorkspaceState tmp;
        graph->stateRobotToWorkspace(egraph_robot_state, tmp);

        Eigen::Affine3d pregrasp_offset(
                Eigen::Translation3d(graph->pregrasp_offset_x, 0.0, 0.0));

        Eigen::Affine3d grasp_pose =
                Eigen::Translation3d(tmp[0], tmp[1], tmp[2]) *
                Eigen::AngleAxisd(tmp[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(tmp[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(tmp[3], Eigen::Vector3d::UnitX());

        Eigen::Affine3d pregrasp_pose = grasp_pose * pregrasp_offset;

        // TODO: these are stored in the state now, get them from there
        smpl::WorkspaceCoord disc_egraph_state(graph->dofCount());
        graph->stateWorkspaceToCoord(tmp, disc_egraph_state);

        // map phi(discrete egraph state) -> egraph node
        auto phi_coord = GetPhiCoord(graph, disc_egraph_state);
        graph->phi_to_egraph_nodes[phi_coord].push_back(node);
        phi_points.emplace_back(phi_coord[0], phi_coord[1], phi_coord[2]);

        // map phi'(discrete egraph state) -> egraph node
        auto pre_phi_coord = GetPhiCoord(graph, pregrasp_pose);
        graph->pre_phi_to_egraph_nodes[pre_phi_coord].push_back(node);
        pg_phi_points.emplace_back(pre_phi_coord[0], pre_phi_coord[1], pre_phi_coord[2]);

        graph->egraph_node_grasps[node] = grasp_pose;
        graph->egraph_node_pregrasps[node] = pregrasp_pose;
        graph->egraph_phi_coords[node] = GetPhiCoord(graph, grasp_pose);
        graph->egraph_pre_phi_coords[node] = GetPhiCoord(graph, pregrasp_pose);
    }

    std::vector<Eigen::Vector3d> phi_points_cont;
    phi_points_cont.reserve(phi_points.size());
    for (auto& point : phi_points) {
        double pcont[3];
        graph->posCoordToWorkspace(point.data(), pcont);
        phi_points_cont.emplace_back(pcont[0], pcont[1], pcont[2]);
    }

    std::vector<Eigen::Vector3d> pre_phi_points_cont;
    pre_phi_points_cont.reserve(pg_phi_points.size());
    for (auto& point : pg_phi_points) {
        double pcont[3];
        graph->posCoordToWorkspace(point.data(), pcont);
        pre_phi_points_cont.emplace_back(pcont[0], pcont[1], pcont[2]);
    }

    auto vis_name = "phi";
    SV_SHOW_INFO_NAMED(
            vis_name,
            MakeCubesMarker(
                    std::move(phi_points_cont),
                    graph->resolution()[0],
                    smpl::visual::Color{ 0.5, 0.5, 0.5, 1.0f },
                    "map",
                    vis_name));
    SV_SHOW_INFO_NAMED(
            "pre_phi",
            MakeCubesMarker(
                std::move(pre_phi_points_cont),
                graph->resolution()[0],
                smpl::visual::Color{ 1.0f, 0.5f, 0.5f, 1.0f },
                "map",
                "pre_phi"));

    return true;
}

auto GetPhiCoord(
    const RomanWorkspaceLatticeEGraph* graph,
    const smpl::WorkspaceCoord& coord)
    -> PhiCoord
{
#if PHI_INCLUDE_RP
    return PhiCoord{ coord[0], coord[1], coord[2], coord[3], coord[4], coord[5] };
#else
    return PhiCoord{ coord[0], coord[1], coord[2], coord[5] };
#endif
}

auto GetPhiState(
    const RomanWorkspaceLatticeEGraph* graph,
    const smpl::WorkspaceState& state)
    -> PhiState
{
#if PHI_INCLUDE_RP
    return PhiState{ state[0], state[1], state[2], state[3], state[4], state[5] };
#else
    return PhiState{ state[0], state[1], state[2], state[5] };
#endif
};


bool RomanWorkspaceLatticeEGraph::snap(int src_id, int dst_id, int& cost)
{
    return TrySnap(this, src_id, dst_id, cost);
}

bool RomanWorkspaceLatticeEGraph::loadExperienceGraph(const std::string& path)
{
    return LoadExperienceGraph(this, path);
}

bool RomanWorkspaceLatticeEGraph::extractPath(
    const std::vector<int>& ids,
    std::vector<smpl::RobotState>& path)
{
    return ExtractPath(this, ids, path);
}

void RomanWorkspaceLatticeEGraph::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    return ::GetSuccs(this, state_id, succs, costs, false);
}
