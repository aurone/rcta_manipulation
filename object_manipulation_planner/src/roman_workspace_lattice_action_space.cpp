#include "roman_workspace_lattice_action_space.h"

// standard includes
#include <utility>

// system includes
#include <smpl/angles.h>
#include <smpl/robot_model.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>

enum VariableIndex {
    EE_PX,
    EE_PY,
    EE_PZ,
    EE_QX,
    EE_QY,
    EE_QZ,
    AR_FA,
    TR_JP,
    BD_TH,
    BD_PX,
    BD_PY,
    OB_P,
};

enum RobotVariableIndex {
    WORLD_JOINT_X,
    WORLD_JOINT_Y,
    WORLD_JOINT_THETA,
    TORSO_JOINT1,
    LIMB_JOINT1,
    LIMB_JOINT2,
    LIMB_JOINT3,
    LIMB_JOINT4,
    LIMB_JOINT5,
    LIMB_JOINT6,
    LIMB_JOINT7,
    HINGE,
};

bool InitRomanWorkspaceLatticeActions(
    smpl::WorkspaceLattice* space,
    RomanWorkspaceLatticeActionSpace* actions)
{
    actions->space = space;

    actions->m_prims.clear();

    smpl::MotionPrimitive prim;

    auto add_xyz_prim = [&](int dx, int dy, int dz)
    {
        std::vector<double> d(space->dofCount(), 0.0);
        d[EE_PX] = space->resolution()[EE_PX] * dx;
        d[EE_PY] = space->resolution()[EE_PY] * dy;
        d[EE_PZ] = space->resolution()[EE_PZ] * dz;
        prim.type = smpl::MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(std::move(d));

        actions->m_prims.push_back(prim);
    };

#if 0
    // create 26-connected position motions
    for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
    for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }
        add_xyz_prim(dx, dy, dz);
    }
    }
    }
#else
    // 2-connected motions for x, y, and z of the end effector
    add_xyz_prim(-1, 0, 0);
    add_xyz_prim(1, 0, 0);
    add_xyz_prim(0, 1, 0);
    add_xyz_prim(0, -1, 0);
    add_xyz_prim(0, 0, 1);
    add_xyz_prim(0, 0, -1);
#endif

    // create 2-connected motions for rotation and free angle motions
    for (int a = 3; a < space->dofCount(); ++a) {
        std::vector<double> d(space->dofCount(), 0.0);

        // skip roll and pitch primitives
        if (a == EE_QX || a == EE_QY) continue;

        // do base motions later
        if (a == BD_PX || a == BD_PY) continue;

        d[a] = space->resolution()[a] * -1;
        prim.type = smpl::MotionPrimitive::Type::LONG_DISTANCE;

        if (a == BD_PX) { // base x
            d[EE_PX] = -space->resolution()[EE_PX]; // also move the end effector by -res in x
        }
        if (a == BD_PY) { // base y
            d[EE_PY] = -space->resolution()[EE_PY]; // also move the end effector by -res in y
        }
        if (a == TR_JP || a == BD_TH) {
            d[EE_QZ] = -space->resolution()[EE_QZ]; // also move the end effector yaw by -res in theta
        }

        prim.action.clear();
        prim.action.push_back(d);
        actions->m_prims.push_back(prim);

        d[a] = space->resolution()[a] * 1;
        prim.type = smpl::MotionPrimitive::Type::LONG_DISTANCE;

        if (a == BD_PX) {
            d[EE_PX] = space->resolution()[EE_PX]; // also move the end effector by +res in x
        }
        if (a == BD_PY) {
            d[EE_PY] = space->resolution()[EE_PY]; // also move the end effector by +res in y
        }
        if (a == TR_JP || a == BD_TH) {
            d[EE_QZ] = space->resolution()[EE_QZ];
        }

        prim.action.clear();
        prim.action.push_back(d);
        actions->m_prims.push_back(prim);
    }

    // Add motions to move the base forward/backward, left/right, and
    // diagonally. Move the end effector along with the base. Note that these
    // actions will be pruned later to enforce non-holonomic constraints.
    //
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            // skip no motion
            if (dx == 0 && dy == 0) continue;

            // skip long diagonals
            if (dx == -2 && dy == -2) continue;
            if (dx == -2 && dy == 2) continue;
            if (dx == 2 && dy == -2) continue;
            if (dx == 2 && dy == 2) continue;

            if (dx == -2 && dy == -2) continue;
            if (dx == 2 && dy == -2) continue;
            if (dx == -2 && dy == 2) continue;
            if (dx == 2 && dy == 2) continue;

            std::vector<double> d(space->dofCount(), 0.0);
            d[BD_PX] = dx * space->resolution()[BD_PX];
            d[BD_PY] = dy * space->resolution()[BD_PY];

            d[EE_PX] = dx * space->resolution()[EE_PX];
            d[EE_PY] = dy * space->resolution()[EE_PY];
            prim.action.clear();
            prim.action.push_back(d);
            actions->m_prims.push_back(prim);
        }
    }

    SMPL_INFO("%zu total motion primitives", actions->m_prims.size());
    for (auto& prim : actions->m_prims) {
        SMPL_INFO_STREAM("primitive:  " << prim.action);
    }

    return true;
}

void RomanWorkspaceLatticeActionSpace::apply(
    const smpl::WorkspaceLatticeState& state,
    std::vector<smpl::WorkspaceAction>& actions)
{
    actions.reserve(actions.size() + m_prims.size());

    smpl::WorkspaceState cont_state;
    space->stateCoordToWorkspace(state.coord, cont_state);

    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  create actions for workspace state: " << cont_state);

    for (auto& prim : m_prims) {
        {   // disallow sideways motion
            auto dx = prim.action.back()[BD_PX]; //action.back()[BD_PX] - state.state[0];
            auto dy = prim.action.back()[BD_PY]; //action.back()[BD_PY] - state.state[1];
            if (std::fabs(dx) > 1e-6 | std::fabs(dy) > 1e-6) {
                auto heading = atan2(dy, dx);
                auto alt_heading = heading + M_PI;
                if (smpl::angles::shortest_angle_dist(heading, state.state[2]) >
                        smpl::angles::to_radians(10.0) &&
                    smpl::angles::shortest_angle_dist(alt_heading, state.state[2]) >
                        smpl::angles::to_radians(10.0))
                {
                    continue;
                }
            }
        }

#define CORRECT_EE 0

        // action moves the torso or the base theta
        if (
#if CORRECT_EE
            prim.action.back()[BD_PX] != 0.0 ||
            prim.action.back()[BD_PY] != 0.0 ||
#endif
            prim.action.back()[TR_JP] != 0.0 ||
            prim.action.back()[BD_TH] != 0.0)
        {
            auto delta_state = state.state;
            delta_state[WORLD_JOINT_X] += prim.action.back()[BD_PX];
            delta_state[WORLD_JOINT_Y] += prim.action.back()[BD_PY];
            delta_state[TORSO_JOINT1] += prim.action.back()[TR_JP];
            delta_state[WORLD_JOINT_THETA] += prim.action.back()[BD_TH];
            auto pose = space->m_fk_iface->computeFK(delta_state);

            smpl::WorkspaceState tmp(space->dofCount(), 0.0);
            tmp[EE_PX] = pose.translation().x();
            tmp[EE_PY] = pose.translation().y();
            tmp[EE_PZ] = pose.translation().z();
            smpl::angles::get_euler_zyx(pose.rotation(), tmp[EE_QZ], tmp[EE_QY], tmp[EE_QX]);
            tmp[AR_FA] = cont_state[AR_FA];// + prim.action.back()[AR_FA];
            tmp[TR_JP] = cont_state[TR_JP] + prim.action.back()[TR_JP];
            tmp[BD_TH] = cont_state[BD_TH] + prim.action.back()[BD_TH];
            tmp[BD_PX] = cont_state[BD_PX] + prim.action.back()[BD_PX];
            tmp[BD_PY] = cont_state[BD_PY] + prim.action.back()[BD_PY];
            tmp[OB_P] = cont_state[OB_P];// + prim.action.back()[OB_P];

            smpl::WorkspaceCoord ctmp;
            space->stateWorkspaceToCoord(tmp, ctmp);
            space->stateCoordToWorkspace(ctmp, tmp);

            smpl::WorkspaceAction action;
            action.push_back(std::move(tmp));
            actions.push_back(std::move(action));
            continue;
        }

        smpl::WorkspaceAction action;
        action.reserve(prim.action.size());

        auto final_state = cont_state;
        for (auto& delta_state : prim.action) {
            // increment the state
            for (size_t d = 0; d < space->dofCount(); ++d) {
                final_state[d] += delta_state[d];
            }

            smpl::angles::normalize_euler_zyx(&final_state[3]);

            action.push_back(final_state);
        }

        actions.push_back(std::move(action));
    }

    if (m_ik_amp_enabled && space->numHeuristics() > 0) {
        auto* h = space->heuristic(0);
        auto goal_dist = h->getMetricGoalDistance(
                cont_state[EE_PX], cont_state[EE_PY], cont_state[EE_PZ]);
        if (goal_dist < m_ik_amp_thresh) {
            smpl::RobotState ik_sol;
            if (space->m_ik_iface->computeIK(space->goal().pose, state.state, ik_sol)) {
                smpl::WorkspaceState final_state;
                space->stateRobotToWorkspace(ik_sol, final_state);
                smpl::WorkspaceAction action(1);
                action[0] = final_state;
                actions.push_back(std::move(action));
            }
        }
    }
}

