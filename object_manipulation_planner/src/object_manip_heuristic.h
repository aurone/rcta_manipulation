#ifndef SMPL_OBJECT_MANIP_HEURISTIC_H
#define SMPL_OBJECT_MANIP_HEURISTIC_H

#include <smpl/angles.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

namespace smpl {

class ExperienceGraphExtension;
class ExtractRobotStateExtension;

class ObjectManipulationHeuristic :
    public RobotHeuristic,
    public ExperienceGraphHeuristicExtension
{
public:

    ExperienceGraphExtension* eg = NULL;
    ExtractRobotStateExtension* extract_state = NULL;
    PointProjectionExtension* project_to_point = NULL;

    double goal_z;

    std::vector<int> egraph_goal_heuristics;

    double heading_thresh = 0.1;
    double theta_db = angles::to_radians(2.0);
    double pos_db = 0.1;

    bool init(RobotPlanningSpace* space);

    /// \name Required ExperienceGraphHeuristicExtension Interface
    ///@{
    void getEquivalentStates(
        int state_id,
        std::vector<int>& ids) override;

    void getShortcutSuccs(
        int state_id,
        std::vector<int>& ids) override;
    ///@}

    /// \name Required RobotHeuristic Interface
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    void updateGoal(const GoalConstraint& goal) override;

    /// \name Required Heuristic Interface
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}

    auto getExtension(size_t class_code) -> Extension* override;
};

} // namespace smpl

#endif

