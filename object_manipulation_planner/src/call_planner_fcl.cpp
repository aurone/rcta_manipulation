// standard includes
#include <chrono>
#include <unordered_map>
#include <utility>
#include <vector>

// system includes
#include <fcl/collision.h>
#include <fcl/collision_object.h>
#include <fcl/collision_data.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <smpl/collision_checker.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl_urdf_robot_model/robot_model.h>
#include <smpl_urdf_robot_model/robot_state.h>
#include <smpl_urdf_robot_model/robot_state_visualization.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/ros.h>
#include <smpl/spatial.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>
#include <geometric_shapes/mesh_operations.h>

#define TEST_BROADPHASE 1

// Turns out this isn't really pessimistic. If an object changes position, its
// global AABB *does* need to be recomputed, and the broadphase manager *does*
// need to be updated to account for it.
#define BROADPHASE_PESSIMISTIC 1

// Many optimizations to be made...
// (1) Prune out pairs that cannot possibly lead to self collision within a group
// (2) Prune out pairs whose collision state cannot possibly be modified by modifying variables within the group
// (3) Remove objects from (2) from broadphase detection

namespace fcl {
class CollisionGeometry;
} // namespace fcl

// Generally, we have a set of objects, with defined geometry, with poses
// expressed in a common frame. The collision detection problem is to identify
// pairs of objects whose boundaries or volumes intersect (discrete collision
// detection)
//
// We may want to identify collisions between pairs of time-parameterized paths
// of two objects over some time horizon (continuous collision detection)
//
// We may wish to know, for objects which are not collision, what is the minimum
// separating distance between them (discrete and continuous proximity
// detection)
//
// We may wish to know, for objects moving relative to one another, what is the
// time-to-collision, if they continue moving along their predicted
// trajectories.
//
// For an articulated model, there are often a large number of links which can
// simply never collide due to the construction of articulated model. These
// pairs of collision objects can be safely ignored.
//
// For a robotic model, collisions are typically always disabled between
// adjacent links in the model. This is because the links are expected to
// collide near the joint between them. Sometimes, we still want to detect
// these collisions; for example, two links may be able to collide at a contact
// far away from the joint axis. In this case, it usually suffices to constrain
// the position limits of the joint so that the collision is avoided. Many
// collision detection algorithms for articulated models are also based around
// the assumption that this is the case.
//
// In the case of robot motion planning, we typically only care about collisions
// between the robot's links or between the robot's links and the objects in the
// world. Collisions between objects in the world may be ignored.
//
// Also, in robot motion planning, we often are not planning for the entire
// state of the robot, but rather a subset (e.g. an arm, the base, an end
// effector, etc.). Collisions with links that cannot move may be safely
// ignored. They may be in collision in the initial state, but there is nothing
// the planning algorithm could do to cause or prevent those collisions.
//
// Robot links and object geometries can often be approximated for the sake of
// planning efficiency and to model a required minimum separation distance to
// ensure safety.
//
// Also, in robot motion planning, the model of the robot is typically an
// articulated model, while the the models of the objects in the world are
// usually non-articulated or have a fixed articulation throughout the motion
// planning query.
//

std::pair<const char*, const char*> RomanAllowedCollisionPairs[] = {
    { "base_link", "chest" },
    { "chest", "torso_link0" },
    { "base_link", "torso_link0" },
    { "base_link", "limb_right_link0" },
    { "base_link", "limb_right_link1" },
    { "base_link", "limb_right_link2" },
    { "base_link", "limb_right_link3" },
    { "base_link", "limb_right_link4" },
    { "base_link", "limb_right_link5" },
    { "chest", "limb_right_link0" },
    { "chest", "limb_right_link1" },
    { "limb_right_link0", "torso_link0" },
    { "limb_right_link1", "torso_link0" },
    { "limb_right_link2", "torso_link0" },
    { "limb_right_link3", "torso_link0" },
    { "limb_right_link4", "torso_link0" },
    { "limb_right_link5", "torso_link0" },
    { "limb_right_link0", "limb_right_link1" },
    { "limb_right_link0", "limb_right_link2" },
    { "limb_right_link0", "limb_right_link3" },
    { "limb_right_link1", "limb_right_link2" },
    { "limb_right_link1", "limb_right_link3" },
    { "limb_right_link2", "limb_right_link3" },
    { "limb_right_link2", "limb_right_link4" },
    { "limb_right_link3", "limb_right_link4" },
    { "limb_right_link3", "limb_right_link5" },
    { "limb_right_link4", "limb_right_link5" },
    { "limb_right_link4", "limb_right_link6" },
    { "limb_right_link5", "limb_right_link6" },
    { "limb_right_link5", "limb_right_link7" },
    { "limb_right_link6", "limb_right_link7" },
    { "limb_right_link7", "limb_right_palm" },
    { "limb_right_palm", "limb_right_finger_1_link_0" },
    { "limb_right_palm", "limb_right_finger_1_link_1" },
    { "limb_right_palm", "limb_right_finger_2_link_0" },
    { "limb_right_palm", "limb_right_finger_2_link_1" },
    { "limb_right_palm", "limb_right_finger_middle_link_0" },
    { "limb_right_palm", "limb_right_finger_middle_link_1" },
    { "limb_right_palm", "limb_right_tool0" },
    { "limb_right_finger_1_link_0", "limb_right_finger_1_link_1" },
    { "limb_right_finger_1_link_1", "limb_right_finger_1_link_2" },
    { "limb_right_finger_1_link_2", "limb_right_finger_1_link_3" },
    { "limb_right_finger_2_link_0", "limb_right_finger_2_link_1" },
    { "limb_right_finger_2_link_1", "limb_right_finger_2_link_2" },
    { "limb_right_finger_2_link_2", "limb_right_finger_2_link_3" },
    { "limb_right_finger_middle_link_0", "limb_right_finger_middle_link_1" },
    { "limb_right_finger_middle_link_1", "limb_right_finger_middle_link_2" },
    { "limb_right_finger_middle_link_2", "limb_right_finger_middle_link_3" },
    { "base_link", "limb_left_link0" },
    { "base_link", "limb_left_link1" },
    { "base_link", "limb_left_link2" },
    { "base_link", "limb_left_link3" },
    { "base_link", "limb_left_link4" },
    { "base_link", "limb_left_link5" },
    { "chest", "limb_left_link0" },
    { "chest", "limb_left_link1" },
    { "limb_left_link0", "torso_link0" },
    { "limb_left_link1", "torso_link0" },
    { "limb_left_link2", "torso_link0" },
    { "limb_left_link3", "torso_link0" },
    { "limb_left_link4", "torso_link0" },
    { "limb_left_link5", "torso_link0" },
    { "limb_left_link0", "limb_left_link1" },
    { "limb_left_link0", "limb_left_link2" },
    { "limb_left_link0", "limb_left_link3" },
    { "limb_left_link1", "limb_left_link2" },
    { "limb_left_link1", "limb_left_link3" },
    { "limb_left_link2", "limb_left_link3" },
    { "limb_left_link2", "limb_left_link4" },
    { "limb_left_link3", "limb_left_link4" },
    { "limb_left_link3", "limb_left_link5" },
    { "limb_left_link4", "limb_left_link5" },
    { "limb_left_link4", "limb_left_link6" },
    { "limb_left_link5", "limb_left_link6" },
    { "limb_left_link5", "limb_left_link7" },
    { "limb_left_link6", "limb_left_link7" },
    { "limb_left_link7", "limb_left_palm" },
    { "limb_left_palm", "limb_left_finger_1_link_0" },
    { "limb_left_palm", "limb_left_finger_1_link_1" },
    { "limb_left_palm", "limb_left_finger_2_link_0" },
    { "limb_left_palm", "limb_left_finger_2_link_1" },
    { "limb_left_palm", "limb_left_finger_middle_link_0" },
    { "limb_left_palm", "limb_left_finger_middle_link_1" },
    { "limb_left_palm", "limb_left_tool0" },
    { "limb_left_finger_1_link_0", "limb_left_finger_1_link_1" },
    { "limb_left_finger_1_link_1", "limb_left_finger_1_link_2" },
    { "limb_left_finger_1_link_2", "limb_left_finger_1_link_3" },
    { "limb_left_finger_2_link_0", "limb_left_finger_2_link_1" },
    { "limb_left_finger_2_link_1", "limb_left_finger_2_link_2" },
    { "limb_left_finger_2_link_2", "limb_left_finger_2_link_3" },
    { "limb_left_finger_middle_link_0", "limb_left_finger_middle_link_1" },
    { "limb_left_finger_middle_link_1", "limb_left_finger_middle_link_2" },
    { "limb_left_finger_middle_link_2", "limb_left_finger_middle_link_3" },
    { "limb_left_link0", "limb_right_link0" },
    { "limb_left_link0", "limb_right_link1" },
    { "limb_left_link0", "limb_right_link2" },
    { "limb_left_link0", "limb_right_link3" },
    { "limb_left_link1", "limb_right_link0" },
    { "limb_left_link1", "limb_right_link1" },
    { "limb_left_link1", "limb_right_link2" },
    { "limb_left_link1", "limb_right_link3" },
    { "limb_left_link2", "limb_right_link0" },
    { "limb_left_link2", "limb_right_link1" },
    { "limb_left_link2", "limb_right_link2" },
    { "limb_left_link2", "limb_right_link3" },
    { "limb_left_link3", "limb_right_link0" },
    { "limb_left_link3", "limb_right_link1" },
    { "limb_left_link3", "limb_right_link2" },
    { "limb_left_link3", "limb_right_link3" },

    { "torso_link0", "base_laser_frame" },
    { "base_link", "front_left_wheel_link" },
    { "base_link", "front_right_wheel_link" },
    { "base_link", "rear_left_wheel_link" },
    { "base_link", "rear_right_wheel_link" },
};

struct WorldCollisionState
{
    std::deque<fcl::Sphere> spheres;
    std::deque<fcl::Box> boxes;
    std::deque<fcl::Cylinder> cylinders;
    std::deque<fcl::BVHModel<fcl::OBBRSS>> meshes;
    std::vector<fcl::CollisionObject> objects;
    fcl::DynamicAABBTreeCollisionManager broadphase;
};

struct AllowedCollisionMatrix
{
    std::vector<bool> data;
};

//    const smpl::urdf::RobotModel* model,
//    smpl::urdf::RobotState* state,

struct BroadphaseResult
{
    std::vector<std::pair<const fcl::CollisionObject*, const fcl::CollisionObject*>> collision_pairs;
} broadphase_result;

bool BroadphaseCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data)
{
    static_cast<BroadphaseResult*>(data)->collision_pairs.emplace_back(o1, o2);
    return false;
}

// Return true if the state is collision free.
//
// for each pair of links in the robot
//     test for collision
// for each link of the robot
//     for every object in the world
//         test for collision
bool CheckCollisions(
    WorldCollisionState* world_state, // non-const to update broadphase
    const AllowedCollisionMatrix* acm,
    const smpl::urdf::RobotModel* robot_model)
{
#if TEST_BROADPHASE
    broadphase_result.collision_pairs.clear();
#if BROADPHASE_PESSIMISTIC
    world_state->broadphase.update(); // what is this update method for? pose updates? geometry updates?
#endif
    world_state->broadphase.collide(&broadphase_result, BroadphaseCallback);

    for (auto& pair : broadphase_result.collision_pairs) { {
        auto i = pair.first - &world_state->objects[0];
        auto j = pair.second - &world_state->objects[0];
#else
    for (auto i = 0; i < (int)world_state->objects.size(); ++i) {
    for (auto j = i + 1; j < (int)world_state->objects.size(); ++j) {
#endif
        auto& o1 = world_state->objects[i];
        auto& o2 = world_state->objects[j];

        if (acm->data[i * world_state->objects.size() + j]) {
            continue;
        }

        fcl::CollisionRequest req;
        fcl::CollisionResult res;

        req.num_max_contacts = 1;
        req.enable_contact = false;
        req.num_max_cost_sources = 1;
        req.enable_cost = false;
        req.use_approximate_cost = true;
        req.gjk_solver_type = fcl::GST_LIBCCD;
        req.enable_cached_gjk_guess = false;
        req.cached_gjk_guess = fcl::Vec3f(1.0, 0.0, 0.0);

        auto num_contacts = fcl::collide(&o1, &o2, req, res);
        if (num_contacts != 0) {
            auto* body_1 = GetCollisionBody(robot_model, i);
            auto* body_2 = GetCollisionBody(robot_model, j);
            auto* link_1 = body_1->link;
            auto* link_2 = body_2->link;
            auto link_index_1 = GetLinkIndex(robot_model, link_1);
            auto link_index_2 = GetLinkIndex(robot_model, link_2);

            auto* link_name_1 = GetLinkName(robot_model, link_index_1);
            auto* link_name_2 = GetLinkName(robot_model, link_index_2);
            SMPL_DEBUG("Objects '%s' and '%s' are in collision", link_name_1->c_str(), link_name_2->c_str());
            return false;
        }
    }
    }

    return true;
}

bool GetCollisionDistance(
    const smpl::urdf::RobotModel* model,
    smpl::urdf::RobotState* state,
    const WorldCollisionState* world_state)
{
    return false;
}

bool GetCollisionDetails(
    const smpl::urdf::RobotModel* model,
    smpl::urdf::RobotState* state,
    const WorldCollisionState* world_state)
{
    return false;
}

class CollisionCheckerFCL : public smpl::CollisionChecker
{
public:

    const WorldCollisionState* collision_model;
    std::vector<std::string> planning_joints;

    bool isStateValid(
        const smpl::RobotState& state,
        bool verbose = false) override;

    bool isStateToStateValid(
        const smpl::RobotState& src,
        const smpl::RobotState& dst,
        bool verbose) override;

    bool interpolatePath(
        const smpl::RobotState& src,
        const smpl::RobotState& dst,
        std::vector<smpl::RobotState>& path) override;

    auto getExtension(size_t class_code) -> smpl::Extension* override;
};

bool CollisionCheckerFCL::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    return false;
}

bool CollisionCheckerFCL::isStateToStateValid(
    const smpl::RobotState& src,
    const smpl::RobotState& dst,
    bool verbose)
{
    return isStateValid(src) && isStateValid(dst);
}

bool CollisionCheckerFCL::interpolatePath(
    const smpl::RobotState& src,
    const smpl::RobotState& dst,
    std::vector<smpl::RobotState>& path)
{
    path = { src, dst };
    return true;
}

auto CollisionCheckerFCL::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::CollisionChecker>()) {
        return this;
    }
    return 0;
}

template <class T>
struct EmptyDeleter
{
    void operator()(T* ptr) noexcept { }
};

template <class RNG>
static
void SampleJointPositions(
    const smpl::urdf::Joint* joint,
    double* positions,
    RNG* rng)
{
    auto* variables = GetFirstVariable(joint);
    switch (GetJointType(joint)) {
    case smpl::urdf::JointType::Fixed:
        break;
    case smpl::urdf::JointType::Prismatic:
    {
        std::uniform_real_distribution<double> dist(
                variables->limits.min_position,
                variables->limits.max_position);
        auto value = dist(*rng);
        SMPL_DEBUG("Sampled %f from (%f, %f)", value, variables->limits.min_position, variables->limits.max_position);
        positions[0] = value;
        break;
    }
    case smpl::urdf::JointType::Revolute:
    {
        if (variables->limits.has_position_limits) {
            std::uniform_real_distribution<double> dist(
                    variables->limits.min_position,
                    variables->limits.max_position);
            auto value = dist(*rng);
            SMPL_DEBUG("Sampled %f from (%f, %f)", value, variables->limits.min_position, variables->limits.max_position);
            positions[0] = value;
        } else {
            std::uniform_real_distribution<double> dist(-M_PI, M_PI);
            auto value = dist(*rng);
            SMPL_DEBUG("Sampled %f from (%f, %f)", value, variables->limits.min_position, variables->limits.max_position);
            positions[0] = value;
        }
        break;
    }
    case smpl::urdf::JointType::Planar:
        break;
    case smpl::urdf::JointType::Floating:
        break;
    }
};

#define TEST_PR2 0

// Benchmark several planning scenarios which perform collision detection using
// the FCL library.
int main(int argc, char* argv[])
{
    if (argc < 2) {
        SMPL_ERROR("Usage: fcl_test <path/to/urdf>");
        return -1;
    }

    SMPL_INFO("Load the URDF model");

    ros::init(argc, argv, "call_planner_fcl");
    ros::NodeHandle nh;

    smpl::VisualizerROS visualizer;
    smpl::visual::set_visualizer(&visualizer);
    ros::Duration(1.0).sleep();

    // 1. Load the Robot Model
    auto urdf_model = urdf::parseURDFFile(argv[1]);
    if (urdf_model == 0) {
        SMPL_ERROR("Failed to load/parse URDF");
        return -1;
    }

    SMPL_INFO("Initialize the Robot Model from URDF");

    smpl::urdf::RobotModel robot_model;
    smpl::urdf::JointSpec world_joint;
    world_joint.origin = smpl::Affine3::Identity();
    world_joint.axis = smpl::Vector3::Zero();
    world_joint.name = "world_joint";
    world_joint.type = smpl::urdf::JointType::Floating;
    if (!InitRobotModel(&robot_model, urdf_model.get(), &world_joint)) {
        SMPL_ERROR("Failed to initialize Robot Model");
        return -1;
    }

    SMPL_INFO("Initialize the Collision Model from the Robot Model");

    // 1. Create a collision representation
    auto collision_model = WorldCollisionState();

    // map from link to collision objects
    auto link_collision_object_offsets = std::vector<int>();
    auto link_collision_object_counts = std::vector<int>();

    // Create collision objects for each link
    for (auto i = 0; i < GetLinkCount(&robot_model); ++i) {
        auto* link = GetLink(&robot_model, i);

        auto geometry = std::shared_ptr<fcl::CollisionGeometry>();

        auto offset = (int)collision_model.objects.size();

        for (auto& collision : CollisionBodies(link)) {
            switch (collision.shape->type) {
            case smpl::urdf::ShapeType::Sphere:
            {
                auto* sphere = static_cast<smpl::urdf::Sphere*>(collision.shape);
                collision_model.spheres.push_back(fcl::Sphere{ sphere->radius });
                auto* s = &collision_model.spheres.back();
                geometry = std::shared_ptr<fcl::Sphere>(s, EmptyDeleter<fcl::Sphere>());
                break;
            }
            case smpl::urdf::ShapeType::Box:
            {
                auto* box = static_cast<smpl::urdf::Box*>(collision.shape);
                collision_model.boxes.push_back(fcl::Box{ box->size.x(), box->size.y(), box->size.z() });
                auto* s = &collision_model.boxes.back();
                geometry = std::shared_ptr<fcl::Box>(s, EmptyDeleter<fcl::Box>());
                break;
            }
            case smpl::urdf::ShapeType::Cylinder:
            {
                auto* cylinder = static_cast<smpl::urdf::Cylinder*>(collision.shape);
                collision_model.cylinders.push_back(fcl::Cylinder{ cylinder->radius, cylinder->height });
                auto* s = &collision_model.cylinders.back();
                geometry = std::shared_ptr<fcl::Cylinder>(s, EmptyDeleter<fcl::Cylinder>());
                break;
            }
            case smpl::urdf::ShapeType::Mesh:
            {
                auto* umesh = static_cast<smpl::urdf::Mesh*>(collision.shape);

                auto scale = smpl::Vector3(umesh->scale.x(), umesh->scale.y(), umesh->scale.z());

                auto mesh = std::unique_ptr<shapes::Mesh>();
                mesh.reset(shapes::createMeshFromResource(umesh->filename, scale));
                if (mesh == NULL) {
                    SMPL_WARN("Failed to load mesh '%s'", umesh->filename.c_str());
                    return false;
                }

                auto vertices = std::vector<fcl::Vec3f>();
                auto triangles = std::vector<fcl::Triangle>();

                // ...copy vertices
                vertices.resize(mesh->vertex_count);
                for (unsigned int i = 0; i < mesh->vertex_count; ++i) {
                    double x = mesh->vertices[3 * i    ];
                    double y = mesh->vertices[3 * i + 1];
                    double z = mesh->vertices[3 * i + 2];
                    vertices[i] = fcl::Vec3f(x, y, z);
                }

                // ...copy triangles
                triangles.resize(mesh->triangle_count);
                for (unsigned int i = 0; i < mesh->triangle_count; ++i) {
                    triangles[i] = fcl::Triangle(
                            mesh->triangles[3 * i],
                            mesh->triangles[3 * i + 1],
                            mesh->triangles[3 * i + 2]);
                }

                collision_model.meshes.push_back(fcl::BVHModel<fcl::OBBRSS>{ });
                auto* s = &collision_model.meshes.back();
                s->beginModel();
                s->addSubModel(std::move(vertices), std::move(triangles));
                s->endModel();
                geometry = std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>>(
                        s,
                        EmptyDeleter<fcl::BVHModel<fcl::OBBRSS>>());
                break;
            }
            }

            if (geometry != NULL) {
                auto collision_object = fcl::CollisionObject(geometry);
                collision_model.objects.push_back(collision_object);
            }
        }

        auto offset_end = (int)collision_model.objects.size();
        link_collision_object_offsets.push_back(offset);
        link_collision_object_counts.push_back(offset_end - offset);
    }

    SMPL_INFO("Setup broadphase");

    for (auto& object : collision_model.objects) {
        collision_model.broadphase.registerObject(&object);
    }
    collision_model.broadphase.setup();

    SMPL_INFO("Created %zu collision objects for the robot", collision_model.objects.size());

    SMPL_INFO("Initialize allowed collision matrix");

    auto acm = AllowedCollisionMatrix();
    acm.data = std::vector<bool>(
            collision_model.objects.size() * collision_model.objects.size(),
            false);
#if TEST_PR2
    for (auto& entry : PR2AllowedCollisionPairs) {
#else
    for (auto& entry : RomanAllowedCollisionPairs) {
#endif
        auto* link_a = GetLink(&robot_model, entry.first);
        auto* link_b = GetLink(&robot_model, entry.second);
        if (link_a == 0 | link_b == 0) {
            continue;
        }

        for (auto& collision_a : CollisionBodies(link_a)) {
            auto index_a = GetCollisionBodyIndex(&robot_model, &collision_a);
            for (auto& collision_b : CollisionBodies(link_b)) {
                auto index_b = GetCollisionBodyIndex(&robot_model, &collision_b);
                acm.data[index_a * collision_model.objects.size() + index_b] = true;
                acm.data[index_b * collision_model.objects.size() + index_a] = true;
            }
        }
    }

    for (auto i = 0; i < GetLinkCount(&robot_model); ++i) {
        auto* link = GetLink(&robot_model, i);
        for (auto& b1 : CollisionBodies(link)) {
            auto b1i = GetCollisionBodyIndex(&robot_model, &b1);
            for (auto& b2 : CollisionBodies(link)) {
                auto b2i = GetCollisionBodyIndex(&robot_model, &b2);
                acm.data[b1i * collision_model.objects.size() + b2i] = true;
            }
        }
    }

    for (auto i = 0; i < (int)collision_model.objects.size(); ++i) {
        acm.data[i * collision_model.objects.size() + i] = true;
    }

    auto sample_joint_names = std::vector<std::string>();
#if TEST_PR2
    sample_joint_names = {
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_forearm_roll_joint",
        "r_elbow_flex_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint",
    };
#else
    sample_joint_names = {
        "limb_right_joint1",
        "limb_right_joint2",
        "limb_right_joint3",
        "limb_right_joint4",
        "limb_right_joint5",
        "limb_right_joint6",
        "limb_right_joint7",
    };
#endif

    CollisionCheckerFCL checker;
    checker.collision_model = &collision_model;
    checker.planning_joints = sample_joint_names;

    SMPL_INFO("Begin testing");

    smpl::urdf::RobotState robot_state;
    if (!InitRobotState(&robot_state, &robot_model)) {
        SMPL_ERROR("Failed to initialize Robot State");
        return -1;
    }

    SetToDefaultValues(&robot_state);
    UpdateTransforms(&robot_state);
    SV_SHOW_INFO(MakeCollisionVisualization(&robot_state, smpl::visual::Color{ 1.0f, 0.0f, 0.0f, 1.0f }, "map", "default"));

    std::default_random_engine rng;

    auto avg_check_time = 0.0;
    auto check_samples = 0;

    auto GetStateVector = [](
        const smpl::urdf::RobotModel* model,
        const smpl::urdf::RobotState* state,
        const std::vector<std::string>* joint_names)
    {
        auto v = std::vector<double>();
        for (auto i = 0; i < joint_names->size(); ++i) {
            auto& name = (*joint_names)[i];
            auto* joint = GetJoint(model, name.c_str());
            auto* positions = GetJointPositions(state, joint);
            for (auto j = 0; j < GetVariableCount(joint); ++j) {
                v.push_back(positions[j]);
            }
        }
        return v;
    };

    auto samples = 10000;
    for (auto i = 0; i < samples; ++i) {
        if (!ros::ok()) break;

        for (auto& joint_name : sample_joint_names) {
            // maximum number of positions for any joint type
            double positions[7];
            auto* joint = GetJoint(&robot_model, joint_name.c_str());
            SampleJointPositions(joint, positions, &rng);
            SetJointPositions(&robot_state, joint, positions);
        }
        auto v = GetStateVector(&robot_model, &robot_state, &sample_joint_names);
        SMPL_DEBUG_STREAM("Sampled state: " << v);

        UpdateTransforms(&robot_state);

        for (auto ii = 0; ii < GetCollisionBodyCount(&robot_model); ++ii) {
            auto* transform = GetCollisionBodyTransform(&robot_state, ii);
            auto q = smpl::Quaternion(transform->rotation());
            auto rot = fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z());

            auto pos = fcl::Vec3f(
                    transform->translation().x(),
                    transform->translation().y(),
                    transform->translation().z());
            collision_model.objects[ii].setTransform(rot, pos);
#if BROADPHASE_PESSIMISTIC
            collision_model.objects[ii].computeAABB();
#endif
        }

        auto color = smpl::visual::Color { };
        auto valid_then = std::chrono::high_resolution_clock::now();
        auto valid = CheckCollisions(&collision_model, &acm, &robot_model);
        auto valid_now = std::chrono::high_resolution_clock::now();
        auto valid_time = std::chrono::duration<double>(valid_now - valid_then).count();
        ++check_samples;
        auto alpha = (1.0 / (double)check_samples);
        avg_check_time = (1.0 - alpha) * avg_check_time + alpha * valid_time;

        if (valid) {
            color = smpl::visual::Color{ 0.0f, 1.0f, 0.0f, 1.0f };
        } else {
            color = smpl::visual::Color{ 1.0f, 0.0f, 0.0f, 1.0f };
        }

//        SV_SHOW_INFO(MakeCollisionVisualization(&robot_state, color, "map", "sample"));
//        getchar();
//        ros::Duration(0.1).sleep();
    }

    SMPL_INFO("Average Collision Check Time: %f", avg_check_time);

    return 0;
}
