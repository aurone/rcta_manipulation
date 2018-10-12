#ifndef OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H
#define OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <smpl/graph/workspace_lattice_egraph.h>

using PhiState = std::vector<double>;
using PhiCoord = std::vector<int>;

class ObjectManipHeuristic;

namespace TransitionType
{
enum Type
{
    OrigStateOrigSucc       = 0,
    OrigStateBridgeSucc     = 1,
    OrigStateZSucc          = 2,
    EGraphStateAdjSucc      = 3,
    EGraphStateBridgeSucc   = 4,
    EGraphStateZSucc        = 5,
    PreGraspAmpSucc         = 6,
    GraspSucc               = 7,
    PreGraspSucc            = 8,
    SnapSucc                = 9,
    ShortcutSucc            = 10
};

auto to_cstring(Type) -> const char*;

} // namespace TransitionType

/// * Provides a mapping from phi coordinates to experience graph states
/// * Implements Roman-specific snap actions
/// * Implements object-manipulation-specific "z-edges"
/// * Overrides path extraction, primarily to return interpolated snap actions.
class RomanObjectManipLattice : public smpl::WorkspaceLatticeEGraph
{
public:

    using WorkspaceLattice::isGoal;

    auto getPhiState(const smpl::WorkspaceState& state) const -> PhiState;
    auto getPhiCoord(const smpl::WorkspaceCoord& coord) const -> PhiCoord;
    auto getPhiCoord(const Eigen::Affine3d& pose) const -> PhiCoord;
    auto getPhiState(const Eigen::Affine3d& pose) const -> PhiState;

    void getUniqueSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getOrigStateZSuccs(
        smpl::WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getOrigStateZSuccs2(
        smpl::WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getEGraphStateZSuccs(
        smpl::WorkspaceLatticeState* state,
        smpl::ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getPreGraspAmpSucc(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getGraspSuccs(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getPreGraspSuccs(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        std::vector<int>* succs,
        std::vector<int>* costs);

    int getSnapMotion(int src_id, int dst_id, std::vector<smpl::RobotState>* path);

    bool trySnap(int src_id, int dst_id, int& cost);

    bool isGoal(const smpl::WorkspaceLatticeState* state) const;

    bool extractTransition(int src_id, int dst_id, std::vector<smpl::RobotState>& path);

    bool updateBestTransitionSimple(
        const std::vector<int>& succs,
        const std::vector<int>& costs,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path,
        TransitionType::Type);

    void updateBestTransitionOrig(
        smpl::WorkspaceLatticeState* state,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionOrigBridge(
        smpl::WorkspaceLatticeState* state,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionOrigZ(
        smpl::WorkspaceLatticeState* state,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionEGraphBridge(
        smpl::WorkspaceLatticeState* state,
        smpl::ExperienceGraph::node_id egraph_node,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionEGraphAdjacent(
        smpl::WorkspaceLatticeState* state,
        smpl::ExperienceGraph::node_id egraph_node,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionEGraphZ(
        smpl::WorkspaceLatticeState* state,
        smpl::ExperienceGraph::node_id egraph_node,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionGrasp(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionPreGrasp(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionPreGraspAmp(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionSnap(
        int state_id,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionShortcut(
        int state_id,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void insertExperienceGraphPath(const std::vector<smpl::RobotState>& path);
    void clearExperienceGraph();

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

    // map: [x, y, z, yaw] -> [n1, ..., nn]
    using PhiCoordToEGraphNodesMap = smpl::hash_map<
            PhiCoord,
            std::vector<smpl::ExperienceGraph::node_id>,
            smpl::VectorHash<int>>;

    // map discrete (x, y, z, yaw) poses to e-graph states whose discrete state
    // is within some tolerance. The tolerance is defined as lying within the
    // same discrete bin, i.e. discrete(phi(s)) = (x, y, z, yaw). This is
    // queried to determine the set of edges E_z during planning.
    PhiCoordToEGraphNodesMap m_phi_to_egraph_nodes;

    double pregrasp_offset_x = -0.10;
    PhiCoordToEGraphNodesMap m_pregrasp_phi_to_egraph_node;
    PhiCoordToEGraphNodesMap m_grasp_phi_to_egraph_node;

    std::vector<PhiCoord> m_egraph_phi_coords;
    std::vector<PhiCoord> m_egraph_pre_phi_coords;

    template <class T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

    AlignedVector<Eigen::Affine3d> m_egraph_node_pregrasps;
    AlignedVector<Eigen::Affine3d> m_egraph_node_grasps;

    // heurisic required to determine destination states for snap and shortcut
    // actions
    ObjectManipHeuristic* m_heuristic = NULL;
};

#endif

