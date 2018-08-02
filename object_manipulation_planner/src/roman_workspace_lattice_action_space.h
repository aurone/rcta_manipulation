#ifndef SMPL_ROMAN_WORKSPACE_LATTICE_ACTION_SPACE_H
#define SMPL_ROMAN_WORKSPACE_LATTICE_ACTION_SPACE_H

// standard includes
#include <vector>

// system includes
#include <smpl/graph/workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice_types.h>
#include <smpl/graph/motion_primitive.h>

namespace smpl {
struct WorkspaceLattice;
}

class RomanWorkspaceLatticeActionSpace : public smpl::WorkspaceLatticeActionSpace
{
public:

    smpl::WorkspaceLattice* space = NULL;
    std::vector<smpl::MotionPrimitive> m_prims;
    bool m_ik_amp_enabled = true;
    double m_ik_amp_thresh = 0.2;

    void apply(
        const smpl::WorkspaceLatticeState& state,
        std::vector<smpl::WorkspaceAction>& actions) override;
};

bool InitRomanWorkspaceLatticeActions(
    smpl::WorkspaceLattice* space,
    RomanWorkspaceLatticeActionSpace* actions);

#endif

