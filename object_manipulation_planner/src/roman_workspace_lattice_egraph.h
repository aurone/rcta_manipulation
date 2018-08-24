#ifndef OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H
#define OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <smpl/graph/workspace_lattice_egraph.h>

using PhiState = std::vector<double>;
using PhiCoord = std::vector<int>;

/// * Provides a mapping from phi coordinates to experience graph states
/// * Implements Roman-specific snap actions
/// * Implements object-manipulation-specific "z-edges"
/// * Overrides path extraction, primarily to return interpolated snap actions.
struct RomanWorkspaceLatticeEGraph : public smpl::WorkspaceLatticeEGraph
{
    // map: [x, y, z, yaw] -> [n1, ..., nn]
    using PhiCoordToEGraphNodesMap = smpl::hash_map<
            PhiCoord,
            std::vector<smpl::ExperienceGraph::node_id>,
            smpl::VectorHash<int>>;

    // map discrete (x, y, z, yaw) poses to e-graph states whose discrete state
    // is within some tolerance. The tolerance is defined as lying within the
    // same discrete bin, i.e. discrete(phi(s)) = (x, y, z, yaw). This is
    // queried to determine the set of edges E_z during planning.
    PhiCoordToEGraphNodesMap phi_to_egraph_nodes;
    PhiCoordToEGraphNodesMap pre_phi_to_egraph_nodes;
    double pregrasp_offset_x = -0.10;

    std::vector<PhiCoord> egraph_phi_coords;
    std::vector<PhiCoord> egraph_pre_phi_coords;

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
        egraph_node_pregrasps;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
        egraph_node_grasps;

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

auto GetPhiState(
    const RomanWorkspaceLatticeEGraph* graph,
    const smpl::WorkspaceState& state)
    -> PhiState;

auto GetPhiCoord(
    const RomanWorkspaceLatticeEGraph* graph,
    const smpl::WorkspaceCoord& coord)
    -> PhiCoord;

#endif

