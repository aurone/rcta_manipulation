#ifndef OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H
#define OBJECT_MANIPULATION_PLANNER_WORKSPACE_LATTICE_EGRAPH_ROMAN_H

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <smpl/graph/workspace_lattice_egraph.h>

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
    ShortcutSucc            = 10,
    Unknown
};

auto to_cstring(Type) -> const char*;

} // namespace TransitionType

struct CachedActionData
{
    int                 succ_id;
    int                 cost;
};

/// * Provides a mapping from phi coordinates to experience graph states
/// * Implements Roman-specific snap actions
/// * Implements object-manipulation-specific "z-edges"
/// * Overrides path extraction, primarily to return interpolated snap actions.
class RomanObjectManipLattice : public smpl::WorkspaceLatticeEGraph
{
public:

    using WorkspaceLattice::isGoal;

    void setObjectPose(const smpl::Affine3& pose);

    auto getPhiCoord(const smpl::WorkspaceLatticeState* coord) const -> PhiCoord;
    auto getPhiCoord(const Eigen::Affine3d& pose) const -> PhiCoord;

    void getUniqueSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs);

#if 0
    void getOrigStateZSuccs(
        smpl::WorkspaceLatticeState* state,
        std::vector<int>* succs,
        std::vector<int>* costs);
#endif

    void getOrigStateZSuccs2(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getEGraphStateZSuccs(
        smpl::WorkspaceLatticeState* state,
        smpl::ExperienceGraph::node_id egraph_node,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getPreGraspAmpSucc(
        int state_id,
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getGraspSuccs(
        int state_id,
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        std::vector<int>* succs,
        std::vector<int>* costs);

    void getPreGraspSuccs(
        int state_id,
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

    void updateBestTransitionOrigZ2(
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
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
        int state_id,
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionPreGrasp(
        int state_id,
        smpl::WorkspaceLatticeState* state,
        const PhiCoord& phi_coord,
        int dst_id,
        int& best_cost,
        std::vector<smpl::RobotState>& best_path);

    void updateBestTransitionPreGraspAmp(
        int state_id,
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

    template <class T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

    // map: [x, y, z, yaw] -> [n1, ..., nn]
    using PhiCoordToEGraphNodesMap = smpl::hash_map<
            PhiCoord,
            std::vector<smpl::ExperienceGraph::node_id>,
            smpl::VectorHash<int>>;

    double pregrasp_offset_x = -0.10;

    smpl::Affine3 m_object_pose;

    // Additional properties of demonstration nodes
    std::vector<bool>               m_egraph_node_validity;
    std::vector<PhiCoord>           m_egraph_phi_coords;
    std::vector<PhiCoord>           m_egraph_pre_phi_coords;
    std::vector<double>             m_demo_z_values;
    AlignedVector<Eigen::Affine3d>  m_egraph_node_pregrasps;
    AlignedVector<Eigen::Affine3d>  m_egraph_node_grasps;

    // map discrete (x, y, z, yaw) poses to e-graph states whose discrete state
    // is within some tolerance. The tolerance is defined as lying within the
    // same discrete bin, i.e. discrete(phi(s)) = (x, y, z, yaw). This is
    // queried to determine the set of edges E_z during planning.
    PhiCoordToEGraphNodesMap m_phi_to_egraph_nodes;

    PhiCoordToEGraphNodesMap m_pregrasp_phi_to_egraph_node;

    // Additional properties of demonstration edges
    std::vector<bool> m_egraph_edge_validity;

    // these tables store the previously generated actions of different types.
    // This is especially useful for actions which have some probability of
    // being generated (because the underlying kinematics functions are non-
    // deterministic)
    using ActionCache = std::unordered_map<int, std::vector<CachedActionData>>;
//    ActionCache m_grasp_action_cache;
//    ActionCache m_pregrasp_action_cache;
    ActionCache m_pregrasp_amp_action_cache;
    ActionCache m_pregrasp_action_cache;
    ActionCache m_grasp_action_cache;

    // heurisic required to determine destination states for snap and shortcut
    // actions
    ObjectManipHeuristic* m_heuristic = NULL;
};

void ClearActionCache(RomanObjectManipLattice* graph);

#endif

