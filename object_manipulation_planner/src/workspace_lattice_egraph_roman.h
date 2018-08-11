#ifndef OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H
#define OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H

// system includes
#include <smpl/graph/workspace_lattice_egraph.h>

struct RomanWorkspaceLatticeEGraph : public smpl::WorkspaceLatticeEGraph
{
    bool snap(int src_id, int dst_id, int& cost) override;

    bool extractPath(
        const std::vector<int>& ids,
        std::vector<smpl::RobotState>& path) override;
};

#endif

