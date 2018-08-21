#ifndef OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H
#define OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H

// system includes
#include <smpl/graph/workspace_lattice_egraph.h>

using PsiState = std::vector<double>;
using PsiCoord = std::vector<int>;

/// * Provides a mapping from psi coordinates to experience graph states
/// * Implements Roman-specific snap actions
/// * Implements object-manipulation-specific "z-edges"
/// * Overrides path extraction, primarily to return interpolated snap actions.
struct RomanWorkspaceLatticeEGraph : public smpl::WorkspaceLatticeEGraph
{
    // map: [x, y, z, yaw] -> [n1, ..., nn]
    using PsiCoordToEGraphNodesMap = smpl::hash_map<
            PsiCoord,
            std::vector<smpl::ExperienceGraph::node_id>,
            smpl::VectorHash<int>>;

    // map discrete (x, y, z, yaw) poses to e-graph states whose discrete state
    // is within some tolerance. The tolerance is defined as lying within the
    // same discrete bin, i.e. discrete(psi(s)) = (x, y, z, yaw). This is
    // queried to determine the set of edges E_z during planning.
    PsiCoordToEGraphNodesMap psi_to_egraph_nodes;

    /// \name ExperienceGraphExtension Interface
    ///@{
    bool snap(int src_id, int dst_id, int& cost) override;
    bool loadExperienceGraph(const std::string& path) override;
    ///@}

    /// \name RobotPlanningSpace Interface
    ///@{
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<smpl::RobotState>& path) override;
    ///@}

    /// \name DiscreteSpaceInformation Interface
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    ///@}
};

auto GetPsiState(
    const RomanWorkspaceLatticeEGraph* graph,
    const smpl::WorkspaceState& state)
    -> PsiState;

auto GetPsiCoord(
    const RomanWorkspaceLatticeEGraph* graph,
    const smpl::WorkspaceCoord& coord)
    -> PsiCoord;

#endif

