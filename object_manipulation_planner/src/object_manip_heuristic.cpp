#include "object_manip_heuristic.h"

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/heap/intrusive_heap.h>
#include <smpl/planning_params.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/visualize.h>

#include "assert.h"
#include "variables.h"
#include "workspace_lattice_egraph_roman.h"

static const double FixedPointRatio = 1000.0;

void ObjectManipulationHeuristic::getEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    // this can probably be about the same as DijkstraEgraphHeuristic3D
    Eigen::Vector3d p;
    this->project_to_point->projectToPoint(state_id, p);

    auto* graph = static_cast<RomanWorkspaceLatticeEGraph*>(planningSpace());

    int state_coord_xyz[3];
    graph->posWorkspaceToCoord(p.data(), state_coord_xyz);

    auto& state = this->extract_state->extractState(state_id);
    SMPL_ASSERT(state.size() == VARIABLE_COUNT, "state has insufficient variables");

    auto* egraph = this->eg->getExperienceGraph();
    auto nodes = egraph->nodes();

    // TODO: connectable might be different than the snap condition

    // return the state ids of all e-graph nodes whose end effector position
    // lie near the input state's end effector position
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = *nit;
        auto& egraph_state = egraph->state(node);
        SMPL_ASSERT(egraph_state.size() == VARIABLE_COUNT, "egraph state has insufficient variables");

        if (state.back() != egraph_state.back()) continue;

        auto node_state_id = this->eg->getStateID(node);
        Eigen::Vector3d egraph_pos;
        this->project_to_point->projectToPoint(node_state_id, egraph_pos);

        int egraph_coord_xyz[3];
        graph->posWorkspaceToCoord(egraph_pos.data(), egraph_coord_xyz);

        // TODO: THIS SHOULD BE RELATED TO THE SEARCH RESOLUTION
#if 0
        double thresh = 0.08; //0.06;
        if ((egraph_pos - p).squaredNorm() < thresh * thresh) {
#else
        if (egraph_coord_xyz[0] == state_coord_xyz[0] &
            egraph_coord_xyz[1] == state_coord_xyz[1] &
            egraph_coord_xyz[2] == state_coord_xyz[2])
        {
#endif
            SMPL_WARN_NAMED(H_LOG, "FOUND IT!");
            ids.push_back(node_state_id);
        }
    }
}

auto deadband(double val, double bot) -> double
{
    if (val < bot) {
        return 0.0;
    } else {
        return val;
    }
}

bool ObjectManipulationHeuristic::init(smpl::RobotPlanningSpace* space)
{
    this->eg = space->getExtension<smpl::ExperienceGraphExtension>();
    if (this->eg == NULL) {
        SMPL_WARN_NAMED(H_LOG, "ObjectManipulationHeuristic requires Experience Graph Extension");
        return false;
    }

    this->extract_state = space->getExtension<smpl::ExtractRobotStateExtension>();
    if (this->extract_state == NULL) {
        SMPL_WARN_NAMED(H_LOG, "ObjectManipulationHeuristic requires Extract Robot State Extension");
        return false;
    }

    this->project_to_point = space->getExtension<smpl::PointProjectionExtension>();
    if (this->project_to_point == NULL) {
        SMPL_WARN_NAMED(H_LOG, "ObjectManipulationHeuristic requires Point Projection Extension");
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        SMPL_WARN_NAMED(H_LOG, "Failed to initialize Robot Heuristic");
        return false;
    }

    return true;
}

void ObjectManipulationHeuristic::getShortcutSuccs(
    int state_id,
    std::vector<int>& ids)
{
    SMPL_ASSERT(this->eg != NULL, "Need experience graph extension");

    auto* egraph = this->eg->getExperienceGraph();
    SMPL_ASSERT(egraph != NULL, "Need an e-graph");

    std::vector<smpl::ExperienceGraph::node_id> egraph_nodes;
    this->eg->getExperienceGraphNodes(state_id, egraph_nodes);

    for (auto node : egraph_nodes) {
        // Return the final state on the demonstration
        // TODO: check for same component
        smpl::ExperienceGraph::node_id min_node;
        auto min_g = std::numeric_limits<int>::max();
        for (auto i = smpl::ExperienceGraph::node_id(0); i < egraph_goal_heuristics.size(); ++i) {
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

void ObjectManipulationHeuristic::updateGoal(const smpl::GoalConstraint& goal)
{
    // find the shortest path from any state in the demonstration to the final
    // demonstration state (basically get the length of the 3d arc motion
    // remaining on the demonnstration

    // map from all z values to x,y,z  states in the demonstration (more
    // generally all robot states (all all, including those states suggested by
    // the E_z edges)
    switch (goal.type) {
    case smpl::GoalType::USER_GOAL_CONSTRAINT_FN:
    {
        auto goal_z = goal.angles.back();
        auto goal_thresh = goal.angle_tolerances.back();

        SMPL_INFO_NAMED(H_LOG, "Goal Z: %f", goal_z);

        auto* egraph = this->eg->getExperienceGraph();

        SMPL_DEBUG_NAMED(H_LOG, "Precompute manipulation heuristic for %zu e-graph states", egraph->num_nodes());
        this->egraph_goal_heuristics.resize(egraph->num_nodes(), -1);

        struct ExperienceGraphSearchNode : smpl::heap_element
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

        using heap_type = smpl::intrusive_heap<ExperienceGraphSearchNode, NodeCompare>;

        std::vector<ExperienceGraphSearchNode> search_nodes(egraph->num_nodes());

        heap_type open;

        // add all goal states in the experience graph to the open list
        auto nodes = egraph->nodes();
        for (auto nit = nodes.first; nit != nodes.second; ++nit) {
            auto node = *nit;
            auto& egraph_state = egraph->state(node);
            auto egraph_state_z = egraph_state.back();

            if (std::fabs(egraph_state_z - goal_z) <= goal_thresh) {
                search_nodes[node].g = 0;
                open.push(&search_nodes[node]);
            }
        }

        SMPL_DEBUG_NAMED(H_LOG, "Experience graph contains %zu goal states", open.size());
        if (open.empty()) {
            SMPL_WARN_NAMED(H_LOG, "Experience graph contains no goal states. Reaching the goal is impossible");
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                auto node = *nit;
                auto& egraph_state = egraph->state(node);
                auto egraph_state_z = egraph_state.back();
                SMPL_WARN_NAMED(H_LOG, "  z(%zu) = %f", node, egraph_state_z);
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

        SMPL_INFO_NAMED(H_LOG, "Expanded %d nodes looking for shortcut", exp_count);
        this->egraph_goal_heuristics.resize(egraph->num_nodes());
        for (int i = 0; i < egraph->num_nodes(); ++i) {
            this->egraph_goal_heuristics[i] = search_nodes[i].g;
        }

        break;
    }
    case smpl::GoalType::XYZ_GOAL:
    case smpl::GoalType::XYZ_RPY_GOAL:
    case smpl::GoalType::MULTIPLE_POSE_GOAL:
    case smpl::GoalType::JOINT_STATE_GOAL:
    default:
        SMPL_WARN_NAMED(H_LOG, "Unsupported goal type %d", (int)goal.type);
        break;
    }
}

auto normalize_disc_theta(int theta, int num_angles) -> int
{
    if (theta >= 0) {
        return theta % num_angles;
    } else {
        return (theta % num_angles + num_angles) % num_angles;
    }
}

auto shortest_angle_dist(int t1, int t2, int num_angles) -> int
{
    if (t2 > t1) {
        auto diff = (t2 - t1) % num_angles;
        return std::min(diff, num_angles - diff);
    } else {
        auto diff = (t1 - t2) % num_angles;
        return std::min(diff, num_angles - diff);
    }
}

auto discretize_angle(double angle, double res, int num_angles) -> int
{
    angle = smpl::normalize_angle_positive(angle);

    auto coord = (int)((angle + res * 0.5) / res);

    if (coord == num_angles) {
        coord = 0;
    }

    return coord;
}

auto MakePoseTransform(double x, double y, double theta) -> Eigen::Affine3d
{
    return Eigen::Translation3d(x, y, 0.0) * Eigen::AngleAxisd(theta, Eigen::Vector3d::Zero());
}

int ObjectManipulationHeuristic::GetGoalHeuristic(int state_id)
{
    SMPL_DEBUG_NAMED(H_LOG, "GetGoalHeuristic(%d)", state_id);
    if (state_id == this->planningSpace()->getGoalStateID()) {
        SMPL_DEBUG_NAMED(H_LOG, "h(goal) = 0");
        return 0;
    }

    auto& state = this->extract_state->extractState(state_id);
    SMPL_ASSERT(state.size() == VARIABLE_COUNT, "state has incorrect variables");

    // visualize the base pose
    SV_SHOW_INFO_NAMED("state_base", smpl::visual::MakeFrameMarkers(
            MakePoseTransform(
                    state[WORLD_JOINT_X],
                    state[WORLD_JOINT_Y],
                    state[WORLD_JOINT_THETA]),
            "map",
            "state_base"));

    SMPL_DEBUG_STREAM_NAMED(H_LOG, "  coord(state) = " << state);

    auto state_z = state.back();
//    SMPL_DEBUG_NAMED(H_LOG, "  z(state) = %f", state_z);

    Eigen::Vector3d point;
    this->project_to_point->projectToPoint(state_id, point);

    SMPL_DEBUG_NAMED(H_LOG, "  psi(state) = (%f, %f, %f)", point.x(), point.y(), point.z());

    auto* egraph = this->eg->getExperienceGraph();
    SMPL_ASSERT(egraph != NULL, "egraph is null");

    auto h_min = std::numeric_limits<int>::max();
    int h_base_min;
    int h_contact_min;
    int h_manipulate_min;

    std::vector<smpl::ExperienceGraph::node_id> the_nodes;
    this->eg->getExperienceGraphNodes(state_id, the_nodes);
    auto is_egraph = !the_nodes.empty();

    // For all nodes v in the E-Graph where z(v) == z(s)
    auto nodes = egraph->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        auto node = (*nit);

        // get the state for this experience graph node
        // or just use the provided robot state directly
        auto& egraph_state = egraph->state(node);
        SMPL_ASSERT(egraph_state.size() == VARIABLE_COUNT, "egraph state is empty");
        auto egraph_state_z = egraph_state[HINGE];

        // z-value should be exact, but we'll test a small threshold here anyway
        auto z_eps = 1e-4;
        if (std::fabs(egraph_state_z - state_z) > z_eps) continue;

        SMPL_DEBUG_NAMED(H_LOG, "    z(v) = %f", egraph_state_z);

        ///////////////////////
        // Compute h_contact //
        ///////////////////////

        auto egraph_state_id = this->eg->getStateID(node);
        Eigen::Vector3d egraph_pos;
        this->project_to_point->projectToPoint(egraph_state_id, egraph_pos);
        SMPL_DEBUG_NAMED(H_LOG, "    psi(v) = (%f, %f, %f)", egraph_pos.x(), egraph_pos.y(), egraph_pos.z());

        auto h_contact = (int)(FixedPointRatio * (egraph_pos - point).norm());

        SMPL_DEBUG_NAMED(H_LOG, "    h_contact(s,v) = %d", h_contact);

        /////////////////////////
        // Lookup h_manipulate //
        /////////////////////////

        SMPL_ASSERT(node < this->egraph_goal_heuristics.size(), "node is out of bounds");
        auto h_manipulate = this->egraph_goal_heuristics[node];
        SMPL_DEBUG_NAMED(H_LOG, "    h_manip(v) = %d", h_manipulate);

        ////////////
        // h_base //
        ////////////

        auto* graph = static_cast<RomanWorkspaceLatticeEGraph*>(planningSpace());

        auto* egraph_graph_state = graph->getState(egraph_state_id);
        auto* graph_state = graph->getState(state_id);

        // visualize the base pose of the egraph state
        SV_SHOW_INFO_NAMED("egraph_base", smpl::visual::MakeFrameMarkers(
                MakePoseTransform(
                        egraph_state[WORLD_JOINT_X],
                        egraph_state[WORLD_JOINT_Y],
                        egraph_state[WORLD_JOINT_THETA]),
                "map",
                "egraph_base"));

        // pose delta in discrete space
        auto ddx = graph_state->coord[BD_PX] - egraph_graph_state->coord[BD_PX];
        auto ddy = graph_state->coord[BD_PY] - egraph_graph_state->coord[BD_PY];
        auto ddtheta = shortest_angle_dist(graph_state->coord[BD_TH], egraph_graph_state->coord[BD_TH], graph->m_val_count[BD_TH]);

        SMPL_DEBUG_NAMED(H_LOG, "    disc delta base = (%d, %d, %d)", ddx, ddy, ddtheta);

        // pose delta in continuous space
        auto dbx = egraph_state[WORLD_JOINT_X] - state[WORLD_JOINT_X];
        auto dby = egraph_state[WORLD_JOINT_Y] - state[WORLD_JOINT_Y];
        auto dbtheta = smpl::shortest_angle_dist(
                egraph_state[WORLD_JOINT_THETA], state[WORLD_JOINT_THETA]);

        auto rot_dist = 0.0;
        if (this->use_rotation) {
            auto heading = atan2(dby, dbx);

            // determine whether to add heading information
            auto add_heading = false;
            switch (this->heading_condition) {
            case 0:
                add_heading = (ddx != 0 || ddy != 0);
                break;
            case 1:
                add_heading = dbx * dbx + dby * dby > this->heading_thresh * this->heading_thresh;
                break;
            case 2:
                break;
            }

            auto num_angles = graph->m_val_count[BD_TH];

            if (add_heading) {
                if (this->disc_rotation_heuristic) {
                    auto disc_heading = discretize_angle(
                            heading,
                            graph->resolution()[BD_TH],
                            graph->m_val_count[BD_TH]);
                    auto disc_theta = graph_state->coord[BD_TH];
                    auto egraph_theta = egraph_graph_state->coord[BD_TH];
                    if (shortest_angle_dist(disc_heading, disc_theta, num_angles) <
                        shortest_angle_dist(disc_heading + (num_angles >> 1), disc_theta, num_angles))
                    {
                        rot_dist =
                                2.0 * M_PI / num_angles *
                                double(shortest_angle_dist(disc_heading, disc_theta, num_angles) +
                                shortest_angle_dist(disc_heading, egraph_theta, num_angles));
                    } else {
                        rot_dist =
                                2.0 * M_PI / num_angles *
                                double(shortest_angle_dist(disc_heading + (num_angles >> 1), disc_theta, num_angles) +
                                shortest_angle_dist(disc_heading + (num_angles >> 1), egraph_theta, num_angles));
                    }
                } else {
                    // closer to face the e-graph state's base position
                    if (smpl::shortest_angle_dist(heading, state[WORLD_JOINT_THETA]) <
                        smpl::shortest_angle_dist(heading + M_PI, state[WORLD_JOINT_THETA]))
                    {
                        rot_dist =
                                smpl::shortest_angle_dist(heading, state[WORLD_JOINT_THETA]) +
                                smpl::shortest_angle_dist(heading, egraph_state[WORLD_JOINT_THETA]);
                    } else { // close to face away from the e-graph state's base position
                        rot_dist =
                                smpl::shortest_angle_dist(heading + M_PI, state[WORLD_JOINT_THETA]) +
                                smpl::shortest_angle_dist(heading + M_PI, egraph_state[WORLD_JOINT_THETA]);
                    }
                }
            } else {
                // return nominal rotation term
                if (this->disc_rotation_heuristic) {
                    rot_dist = 2.0 * M_PI / num_angles * double(ddtheta);
                } else {
                    rot_dist = deadband(dbtheta, this->theta_db);
                }
            }
        }

        auto pos_dist = 0.0;
        if (disc_position_heuristic) {
            pos_dist = graph->resolution()[BD_PX] * std::sqrt((double)(ddx * ddx + ddy * ddy));
        } else {
            pos_dist = deadband(std::sqrt(dbx * dbx + dby * dby), this->pos_db);
        }

        SMPL_DEBUG_NAMED(H_LOG, "    pos dist = %f", pos_dist);
        SMPL_DEBUG_NAMED(H_LOG, "    rot dist(degs) = %f", smpl::to_degrees(rot_dist));

        auto h_base_pos = pos_dist;
        auto h_base_rot = this->theta_normalizer * rot_dist;
        auto h_base = (int)(FixedPointRatio * (h_base_rot + h_base_pos));

        SMPL_DEBUG_NAMED(H_LOG, "  h_base_pos = %f, h_base_rot = %f", h_base_pos, h_base_rot);
        h_base *= this->h_base_weight;

        SMPL_DEBUG_NAMED(H_LOG, "    h_base(s,v) = %d", h_base);

        auto cost = 0;
        switch (this->combination) {
        case CombinationMethod::Max:
            cost = std::max(h_base, h_contact) + h_manipulate;
            break;
        case CombinationMethod::Sum:
            cost = h_base + h_contact + h_manipulate;
            break;
        }

        if (cost < h_min) {
            h_min = cost;
            h_base_min = h_base;
            h_contact_min = h_contact;
            h_manipulate_min = h_manipulate;
        }
    }

    if (h_min == std::numeric_limits<int>::max()) {
        SMPL_INFO_NAMED(H_LOG, "state z = %0.12f", state_z);
    }

    if (is_egraph) {
        SMPL_DEBUG_NAMED(H_LOG, "  h(%d) = %d", state_id, h_manipulate_min);
        return h_manipulate_min;
    } else {
        switch (this->combination) {
        case CombinationMethod::Max:
            SMPL_DEBUG_NAMED(H_LOG, "h(%d) = max(%d, %d) + %d = %d", state_id, h_base_min, h_contact_min, h_manipulate_min, h_min);
            break;
        case CombinationMethod::Sum:
            SMPL_DEBUG_NAMED(H_LOG, "h(%d) = %d + %d + %d = %d", state_id, h_base_min, h_contact_min, h_manipulate_min, h_min);
            break;
        }
        return h_min;
    }
}

int ObjectManipulationHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int ObjectManipulationHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

auto ObjectManipulationHeuristic::getExtension(size_t class_code) -> Extension*
{
    if (class_code == smpl::GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return nullptr;
}
