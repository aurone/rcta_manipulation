#ifndef SMPL_ROMAN_WORKSPACE_LATTICE_ACTION_SPACE_H
#define SMPL_ROMAN_WORKSPACE_LATTICE_ACTION_SPACE_H

// standard includes
#include <vector>

// project includes
#include <smpl/graph/workspace_lattice_action_space.h>
#include <smpl/graph/workspace_lattice_types.h>
#include <smpl/graph/motion_primitive.h>

namespace sbpl {
namespace motion {

struct WorkspaceLattice;

class RomanWorkspaceLatticeActionSpace : public WorkspaceLatticeActionSpace
{
public:

    WorkspaceLattice* space = NULL;
    std::vector<MotionPrimitive> m_prims;
    bool m_ik_amp_enabled = true;
    double m_ik_amp_thresh = 0.2;

    void apply(
        const WorkspaceLatticeState& state,
        std::vector<WorkspaceAction>& actions) override;
};

bool InitRomanWorkspaceLatticeActions(
    WorkspaceLattice* space,
    RomanWorkspaceLatticeActionSpace* actions);

} // namespace motion
} // namespace sbpl

#endif

