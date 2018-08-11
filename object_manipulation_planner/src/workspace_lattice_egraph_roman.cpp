#include "workspace_lattice_egraph_roman.h"

#include <smpl/angles.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>

#include "assert.h"
#include "variables.h"

static
bool GetSnapMotion(
    RomanWorkspaceLatticeEGraph* graph,
    int src_id,
    int dst_id,
    std::vector<smpl::RobotState>& path)
{
    auto* src_state = graph->getState(src_id);
    auto* dst_state = graph->getState(dst_id);
    SMPL_ASSERT(src_state != NULL && dst_state != NULL, "snap motion endpoints are null");

    smpl::WorkspaceState start_state;
    smpl::WorkspaceState finish_state;
    graph->stateCoordToWorkspace(src_state->coord, start_state);
    graph->stateCoordToWorkspace(dst_state->coord, finish_state);
    int num_waypoints = 10;
    for (int i = 0; i < num_waypoints; ++i) {
        smpl::WorkspaceState interm_workspace_state;
        interm_workspace_state.resize(graph->dofCount());

        // x, y, z, R, P, Y, FA1 same as start
        interm_workspace_state[0] = finish_state[0];
        interm_workspace_state[1] = finish_state[1];
        interm_workspace_state[2] = finish_state[2];
        interm_workspace_state[3] = finish_state[3];
        interm_workspace_state[4] = finish_state[4];
        interm_workspace_state[5] = finish_state[5];
        interm_workspace_state[6] = finish_state[6];

        auto interp = [](double a, double b, double t) {
            return (1.0 - t) * a + t * b;
        };

        auto t = (double)i / (double)(num_waypoints - 1);
        // interpolate torso, theta, x, y
        interm_workspace_state[7] = interp(start_state[7], finish_state[7], t);
        interm_workspace_state[8] = interp(start_state[8], finish_state[8], t);
        interm_workspace_state[9] = interp(start_state[9], finish_state[9], t);
        interm_workspace_state[10] = interp(start_state[10], finish_state[10], t);

        // hinge kept the same
        interm_workspace_state[11] = finish_state[11];

        auto stateWorkspaceToRobotPermissive = [&](
            const smpl::WorkspaceState& state,
            smpl::RobotState& ostate)
        {
            smpl::RobotState seed = src_state->state; //(graph->robot()->jointVariableCount(), 0);
            for (size_t fai = 0; fai < graph->freeAngleCount(); ++fai) {
                seed[graph->m_fangle_indices[fai]] = state[6 + fai];
            }

            Eigen::Affine3d pose =
                    Eigen::Translation3d(state[0], state[1], state[2]) *
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX());

//            ostate = seed;
//            return graph->m_ik_iface->computeIK(pose, dst_state->state, ostate);
            return graph->m_ik_iface->computeIK(pose, seed, ostate);
        };

        smpl::RobotState robot_state;
        // TODO: this should be permissive and allow moving the redundant angles
        if (!stateWorkspaceToRobotPermissive(interm_workspace_state, robot_state)) {
            SMPL_WARN("Failed to find ik solution for interpolated state");
            return false;
        }

        path.push_back(std::move(robot_state));
    }

    return true;
}

bool RomanWorkspaceLatticeEGraph::snap(int src_id, int dst_id, int& cost)
{
    SMPL_WARN_NAMED(G_LOG, "Try snap(%d, %d)", src_id, dst_id);

    auto* src_state = this->getState(src_id);
    auto* dst_state = this->getState(dst_id);
    SMPL_ASSERT(src_state != NULL && dst_state != NULL, "snap motion endpoints are null");
    SMPL_ASSERT(src_state->state.size() == VARIABLE_COUNT, "not enough variables");
    SMPL_ASSERT(dst_state->state.size() == VARIABLE_COUNT, "not enough variables");
    SMPL_ASSERT(src_state->coord.size() == VARIABLE_COUNT, "not enough variables");
    SMPL_ASSERT(dst_state->coord.size() == VARIABLE_COUNT, "not enough variables");

    SMPL_DEBUG_STREAM("Snap " << src_state->coord << " -> " << dst_state->coord);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(src_state->state, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(dst_state->state, "snap_to"));

    // if end effector is not on the handle:
    // ....drive to base pose
    // ....interpolate torso/arm motion only to line up with the demonstration
    // else:
    // ....interpolate torso/arm motion only to line up with the demonstration

    // TODO: allow base motion while the end effector is kept steady on the
    // handle?

    auto dx = dst_state->state[WORLD_JOINT_X] - src_state->state[WORLD_JOINT_X];
    auto dy = dst_state->state[WORLD_JOINT_Y] - src_state->state[WORLD_JOINT_Y];
    if (std::fabs(dx) > 1e-6 || std::fabs(dy) > 1e-6) {
        auto heading = atan2(dy, dx);
        auto alt_heading = heading + M_PI;
        auto thresh = smpl::to_radians(10.0);
        if (smpl::shortest_angle_dist(heading, src_state->state[WORLD_JOINT_THETA]) > thresh &&
            smpl::shortest_angle_dist(alt_heading, src_state->state[WORLD_JOINT_THETA]) > thresh)
        {
            SMPL_DEBUG_NAMED(G_LOG, "SKIP SNAP MOTION");
//            return false;
        }
    }

    smpl::WorkspaceState start_state;
    smpl::WorkspaceState finish_state;
    this->stateCoordToWorkspace(src_state->coord, start_state);
    this->stateCoordToWorkspace(dst_state->coord, finish_state);
    int num_waypoints = 10;
    for (int i = 0; i < num_waypoints; ++i) {
        smpl::WorkspaceState interm_workspace_state;
        interm_workspace_state.resize(this->dofCount());

        // x, y, z, R, P, Y, FA1 same as start
        interm_workspace_state[EE_PX] = finish_state[EE_PX];
        interm_workspace_state[EE_PY] = finish_state[EE_PY];
        interm_workspace_state[EE_PZ] = finish_state[EE_PZ];
        interm_workspace_state[EE_QX] = finish_state[EE_QX];
        interm_workspace_state[EE_QY] = finish_state[EE_QY];
        interm_workspace_state[EE_QZ] = finish_state[EE_QZ];
        interm_workspace_state[AR_FA] = finish_state[AR_FA];

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

//            return m_ik_iface->computeIK(pose, dst_state->state, ostate);
            return m_ik_iface->computeIK(pose, seed, ostate);
        };

        smpl::RobotState robot_state;
        // TODO: this should be permissive and allow moving the redundant angles
        if (!stateWorkspaceToRobotPermissive(interm_workspace_state, robot_state)) {
            SMPL_WARN("Failed to find ik solution for interpolated state");
            return false;
        }
    }

    if (!this->collisionChecker()->isStateToStateValid(
            src_state->state, dst_state->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_DEBUG_NAMED(G_LOG, "  Snap %d -> %d!", src_id, dst_id);
    cost = 10;
    return true;
}

bool RomanWorkspaceLatticeEGraph::extractPath(
    const std::vector<int>& ids,
    std::vector<smpl::RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << ids);

    if (ids.empty()) return true;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (ids.size() == 1) {
        auto state_id = ids[0];

        if (state_id == this->getGoalStateID()) {
            auto* entry = this->getState(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", this->getStartStateID());
                return false;
            }
            path.push_back(entry->state);
        } else {
            auto* entry = this->getState(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, this->getStateVisualization(path.back(), vis_name));
        return true;
    }

    if (ids[0] == this->getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<smpl::RobotState> opath;

    // grab the first point
    {
        auto* entry = this->getState(ids[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", ids[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < ids.size(); ++i) {
        auto prev_id = ids[i - 1];
        auto curr_id = ids[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == this->getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        // find the successor state corresponding to the cheapest valid action

        // TODO: return an iterator here to avoid collision checking all
        // successors
        std::vector<int> succs, costs;
        GetSuccs(prev_id, &succs, &costs, true);

        smpl::WorkspaceLatticeState* best_state = NULL;
        auto best_cost = std::numeric_limits<int>::max();
        for (size_t i = 0; i < succs.size(); ++i) {
            if (curr_id == this->getGoalStateID()) {
                auto* state = this->getState(succs[i]);
                smpl::WorkspaceState workspace_state;
                this->stateRobotToWorkspace(state->state, workspace_state);
                if (costs[i] < best_cost && this->isGoal(workspace_state, state->state)) {
                    best_state = state;
                    best_cost = costs[i];
                }
            } else {
                if (succs[i] == curr_id && costs[i] < best_cost) {
                    best_state = this->getState(succs[i]);
                    best_cost = costs[i];
                }
            }
        }

        if (best_state != NULL) {
            SMPL_DEBUG_STREAM_NAMED(G_LOG, "Extract successor state " << best_state->state);
            opath.push_back(best_state->state);
            continue;
        }

        SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut successor");

        auto found = false;
        // check for shortcut transition
        auto pnit = std::find(begin(this->egraph_node_to_state), end(this->egraph_node_to_state), prev_id);
        auto cnit = std::find(begin(this->egraph_node_to_state), end(this->egraph_node_to_state), curr_id);
        if (pnit != end(this->egraph_node_to_state) &&
            cnit != end(this->egraph_node_to_state))
        {
            // position in node array is synonymous with e-graph node id
            auto prev_node = std::distance(begin(this->egraph_node_to_state), pnit);
            auto curr_node = std::distance(begin(this->egraph_node_to_state), cnit);

            SMPL_DEBUG_NAMED(G_LOG, "Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, prev_node, curr_node);

            std::vector<smpl::ExperienceGraph::node_id> node_path;
            found = FindShortestExperienceGraphPath(this->egraph, prev_node, curr_node, node_path);
            if (found) {
                for (auto n : node_path) {
                    auto state_id = this->egraph_node_to_state[n];
                    auto* entry = this->getState(state_id);
                    assert(entry);
                    opath.push_back(entry->state);
                }
            }
        }

        if (found) continue;

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        int cost;
        if (snap(prev_id, curr_id, cost)) {
            SMPL_DEBUG_NAMED(G_LOG, "Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            if (!GetSnapMotion(this, prev_id, curr_id, opath)) {

            }

//            auto* entry = this->getState(curr_id);
//            assert(entry);
//            opath.push_back(entry->state);
            continue;
        }



        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid successor during path extraction");
//        return false;
        {
            auto* giveup = this->getState(curr_id);
            opath.push_back(giveup->state);
        }
    }

    // we made it!
    path = std::move(opath);

    SMPL_DEBUG_NAMED(G_LOG, "Final path:");
    for (auto& point : path) {
        SMPL_INFO_STREAM("  " << point);
    }

    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    return true;
}

