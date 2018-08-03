#include "object_manip_heuristic.h"

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/heap/intrusive_heap.h>

namespace smpl {

static const double FixedPointRatio = 1000.0;

void ObjectManipulationHeuristic::getEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    // this can probably be about the same as DijkstraEgraphHeuristic3D
    Eigen::Vector3d p;
    this->project_to_point->projectToPoint(state_id, p);

    auto& state = this->extract_state->extractState(state_id);

    auto* egraph = this->eg->getExperienceGraph();
    auto nodes = egraph->nodes();

    // TODO: connectable might be different than the snap condition

    // return the state ids of all e-graph nodes whose end effector position
    // lie near the input state's end effector position
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& state = egraph->state(node);

        if (state.back() != state.back()) continue;

        auto node_state_id = this->eg->getStateID(node);
        Eigen::Vector3d egraph_pos;
        this->project_to_point->projectToPoint(node_state_id, egraph_pos);

        // TODO: THIS SHOULD BE RELATED TO THE SEARCH RESOLUTION
        double thresh = 0.08; //0.06;
        if ((egraph_pos - p).squaredNorm() < thresh * thresh) {
            SMPL_WARN("FOUND IT!");
            ids.push_back(node_state_id);
        }
    }
}

bool ObjectManipulationHeuristic::init(RobotPlanningSpace* space)
{
    this->eg = space->getExtension<ExperienceGraphExtension>();
    if (this->eg == NULL) {
        SMPL_WARN("ObjectManipulationHeuristic requires Experience Graph Extension");
        return false;
    }

    this->extract_state = space->getExtension<ExtractRobotStateExtension>();
    if (this->extract_state == NULL) {
        SMPL_WARN("ObjectManipulationHeuristic requires Extract Robot State Extension");
        return false;
    }

    this->project_to_point = space->getExtension<PointProjectionExtension>();
    if (this->project_to_point == NULL) {
        SMPL_WARN("ObjectManipulationHeuristic requires Point Projection Extension");
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        SMPL_WARN("Failed to initialize Robot Heuristic");
        return false;
    }

    return true;
}

void ObjectManipulationHeuristic::getShortcutSuccs(
    int state_id,
    std::vector<int>& ids)
{
    auto* egraph = this->eg->getExperienceGraph();

    std::vector<ExperienceGraph::node_id> egraph_nodes;
    this->eg->getExperienceGraphNodes(state_id, egraph_nodes);

    for (auto node : egraph_nodes) {
        // Return the final state on the demonstration
        // TODO: check for same component
        ExperienceGraph::node_id min_node;
        auto min_g = std::numeric_limits<int>::max();
        for (ExperienceGraph::node_id i = 0; i < egraph_goal_heuristics.size(); ++i) {
            if (this->egraph_goal_heuristics[i] < min_g) {
                min_node = i;
                min_g = this->egraph_goal_heuristics[i];
            }
        }
        if (min_g != std::numeric_limits<int>::max()) {
            ids.push_back(this->eg->getStateID(min_node));
        }

#if 0
        auto adj = egraph->adjacent_nodes(node);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            ids.push_back(this->eg->getStateID(*ait));
        }
#endif
    }
}

double ObjectManipulationHeuristic::getMetricStartDistance(double x, double y, double z)
{
    return 0.0;
}

double ObjectManipulationHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

void ObjectManipulationHeuristic::updateGoal(const GoalConstraint& goal)
{
    // find the shortest path from any state in the demonstration to the final
    // demonstration state (basically get the length of the 3d arc motion
    // remaining on the demonnstration

    // map from all z values to x,y,z  states in the demonstration (more
    // generally all robot states (all all, including those states suggested by
    // the E_z edges)
    switch (goal.type) {
    case GoalType::JOINT_STATE_GOAL:
    {
        this->goal_z = goal.angles.back();
        SMPL_INFO("Goal Z: %f", this->goal_z);

        auto* egraph = this->eg->getExperienceGraph();

        this->egraph_goal_heuristics.resize(egraph->num_nodes(), -1);

        struct ExperienceGraphSearchNode : heap_element
        {
            int                         g = std::numeric_limits<int>::max();
            bool                        closed = false;
            ExperienceGraphSearchNode*  bp = NULL;
        };

        struct NodeCompare
        {
            bool operator()(
                const ExperienceGraphSearchNode& a,
                const ExperienceGraphSearchNode& b)
            {
                return a.g < b.g;
            }
        };

        typedef intrusive_heap<ExperienceGraphSearchNode, NodeCompare> heap_type;

        std::vector<ExperienceGraphSearchNode> search_nodes(egraph->num_nodes());

        heap_type open;

        // add all goal states in the experience graph to the open list
        auto nodes = egraph->nodes();
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            auto node = *nit;
            auto& egraph_state = egraph->state(node);
            auto egraph_state_z = egraph_state.back();

            if (egraph_state_z == goal_z) {
                search_nodes[node].g = 0;
                open.push(&search_nodes[node]);
            }
        }

        int exp_count = 0;
        while (!open.empty()) {
            ++exp_count;
            auto* min = open.min();
            open.pop();
            min->closed = true;

            // get this state
            auto node_id = min - &search_nodes.front();
            auto& min_state = egraph->state(node_id);
            auto min_state_id = this->eg->getStateID(node_id);
            Eigen::Vector3d min_pos;
            this->project_to_point->projectToPoint(min_state_id, min_pos);

            auto n = std::distance(search_nodes.data(), min);
            auto adj = egraph->adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                auto succ_id = *ait;
                auto& succ = search_nodes[succ_id];
                if (succ.closed) {
                    continue;
                }

                auto succ_state_id = this->eg->getStateID(succ_id);
                Eigen::Vector3d succ_pos;
                this->project_to_point->projectToPoint(succ_state_id, succ_pos);

                auto cost = (int)std::round((succ_pos - min_pos).norm() * FixedPointRatio);

                int new_cost = min->g + cost;
                if (new_cost < succ.g) {
                    succ.g = new_cost;
                    succ.bp = min;
                    if (open.contains(&succ)) {
                        open.decrease(&succ);
                    } else {
                        open.push(&succ);
                    }
                }
            }
        }

        SMPL_INFO("Expanded %d nodes looking for shortcut", exp_count);
        this->egraph_goal_heuristics.resize(egraph->num_nodes());
        for (int i = 0; i < egraph->num_nodes(); ++i) {
            this->egraph_goal_heuristics[i] = search_nodes[i].g;
        }

        break;
    }
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL:
    case GoalType::MULTIPLE_POSE_GOAL:
    case GoalType::USER_GOAL_CONSTRAINT_FN:
    default:
        SMPL_WARN("Unsupported goal type %d", (int)goal.type);
        break;
    }
}

int ObjectManipulationHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == this->planningSpace()->getGoalStateID()) {
        return 0;
    }
    auto& state = this->extract_state->extractState(state_id);

    assert(!state.empty());

    auto state_z = state.back();

    Eigen::Vector3d point;
    this->project_to_point->projectToPoint(state_id, point);

    auto* egraph = this->eg->getExperienceGraph();

    auto h_min = std::numeric_limits<int>::max();
    int h_base_min;
    int h_contact_min;
    int h_manipulate_min;

    std::vector<ExperienceGraph::node_id> the_nodes;
    this->eg->getExperienceGraphNodes(state_id, the_nodes);
    auto is_egraph = !the_nodes.empty();

    auto nodes = egraph->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = (*nit);
        // get the state for this experience graph node
        // or just use the provided robot state directly
        auto& egraph_state = egraph->state(node);
        auto egraph_state_z = egraph_state.back();

        if (std::fabs(egraph_state_z - state_z) <= 1e-4) {
            // h_contact
            auto egraph_state_id = this->eg->getStateID(node);
            Eigen::Vector3d egraph_pos;
            this->project_to_point->projectToPoint(egraph_state_id, egraph_pos);
            auto h_contact = (int)(FixedPointRatio * (egraph_pos - point).norm());

            // h_manipulate
            assert(this->egraph_goal_heuristics.size() > node);
            auto h_manipulate = this->egraph_goal_heuristics[node];

            // h_base
            auto dbx = egraph_state[0] - state[0];
            auto dby = egraph_state[1] - state[1];
            auto dbtheta = angles::shortest_angle_dist(egraph_state[2], state[2]);

            auto heading_weight = 0.0;
            if ((dbx * dbx + dby * dby) > (this->heading_thresh * this->heading_thresh)) {
                auto heading = atan2(dby, dbx);
                double heading_diff;
                if (smpl::angles::shortest_angle_dist(heading, state[2]) <
                    smpl::angles::shortest_angle_dist(heading + M_PI, state[2]))
                {
                    heading_weight = smpl::angles::shortest_angle_dist(heading, state[2]) +
                            smpl::angles::shortest_angle_dist(heading, egraph_state[2]);
                } else {
                    heading_weight = smpl::angles::shortest_angle_dist(heading + M_PI, state[2]) +
                            smpl::angles::shortest_angle_dist(heading + M_PI, egraph_state[2]);
                }
            } else {
                heading_weight = dbtheta;
            }

            auto theta_normalizer = 0.05 / angles::to_radians(45.0);

            auto discretize = [](double val, double disc) {
                return disc * std::round(val / disc);
            };

            auto deadband = [](double val, double disc) {
                if (val < disc) {
                    return 0.0;
                } else {
                    return val;
                }
            };

            // heading heuristic (1 or 10 weight)
            auto h_base_pos = deadband(sqrt(dbx * dbx + dby * dby), this->pos_db);
            auto h_base_rot = theta_normalizer * deadband(heading_weight, this->theta_db);
            auto h_base = (int)(
                    FixedPointRatio *
                    (
                        h_base_rot +
//                        std::fabs(dbx) + std::fabs(dby) +
                        h_base_pos
//                        theta_normalizer * heading_weight +
//                        + dbtheta * theta_normalizer
                    ));

            h_base *= 10;
            auto cost = std::max(h_base, h_contact) + h_manipulate;
//            auto cost = h_base + h_contact + h_manipulate;
            if (cost < h_min) {
                h_min = cost;
                h_base_min = h_base;
                h_contact_min = h_contact;
                h_manipulate_min = h_manipulate;
            }
        }
    }

    if (h_min == std::numeric_limits<int>::max()) {
        SMPL_INFO("state z = %0.12f", state_z);
    }

    if (is_egraph) {
        return h_manipulate_min;
    }

    SMPL_DEBUG_NAMED("heuristic", "h(%d) = %d + %d + %d = %d",
            state_id, h_base_min, h_contact_min, h_manipulate_min, h_min);
    return h_min;
}

int ObjectManipulationHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int ObjectManipulationHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

Extension* ObjectManipulationHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return nullptr;
}

} // namespace smpl
