#include "object_manip_checker.h"

// system includes
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/stl/memory.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>

#include "variables.h"

struct ObjectManipCheckerImpl
{
    // object collision geometry
    std::shared_ptr<fcl::Box> obj_geom;

    // origin of collision geometry in the object's frame
    smpl::Affine3 obj_geom_origin;

    std::unique_ptr<fcl::CollisionObject> obj_collision;

    // gripper collision geometry
    std::vector<std::shared_ptr<fcl::Box>> gripper_geoms;

    // origin of gripper geometry components in their respective frames
    std::vector<smpl::Affine3> gripper_geom_origins;
    std::vector<std::unique_ptr<fcl::CollisionObject>> gripper_collisions;

    fcl::DynamicAABBTreeCollisionManager broadphase;
};

static
auto MakeBoxMarker(
    const Eigen::Affine3d& pose,
    double sx, double sy, double sz,
    const std::string& ns,
    const std::string& frame_id = "map",
    const smpl::visual::Color& color = smpl::visual::Color{ 0.5f, 0.5f, 0.5f, 1.0f })
    -> smpl::visual::Marker
{
    smpl::visual::Marker box_marker;
    box_marker.pose = pose;
    box_marker.shape = smpl::visual::Cube{ sx, sy, sz };
    box_marker.color = color;
    box_marker.frame_id = frame_id;
    box_marker.ns = ns;
    return box_marker;
}

static
auto ExtractState(const smpl::RobotState& state) -> smpl::RobotState
{
    auto s = state;
    s.pop_back();
    return s;
}

constexpr auto obj_length = 0.27;
constexpr auto obj_width = 0.5;
constexpr auto obj_height = 0.25;

constexpr auto palm_length = 0.10;
constexpr auto palm_width = 0.05;
constexpr auto palm_height = 0.10;

constexpr auto finger_length = 0.10;
constexpr auto finger_width = 0.05;
constexpr auto finger_height = 0.05;

bool Init(ObjectManipChecker* checker, smpl::collision::CollisionSpace* parent)
{
    checker->parent = parent;
    checker->object_pose = smpl::Affine3::Identity();

    auto* impl = new ObjectManipCheckerImpl;
    impl->obj_geom = std::make_shared<fcl::Box>(obj_length, obj_width, obj_height);
    impl->obj_geom_origin = smpl::Affine3::Identity();
    impl->obj_collision = smpl::make_unique<fcl::CollisionObject>(impl->obj_geom);

    impl->gripper_geoms =
    {
        std::make_shared<fcl::Box>(palm_length, palm_width, palm_height),
        std::make_shared<fcl::Box>(finger_length, finger_width, finger_height),
        std::make_shared<fcl::Box>(finger_length, finger_width, finger_height),
        std::make_shared<fcl::Box>(finger_length, finger_width, finger_height),
    };

    // "limb_right_palm"
    // "limb_right_finger_1_link_0"
    // "limb_right_finger_2_link_0"
    // "limb_right_finger_middle_link_0"
    impl->gripper_geom_origins = std::vector<smpl::Affine3>(4, smpl::Affine3::Identity());
    impl->gripper_geom_origins[1] = smpl::Translation3(0.08, 0.0, 0.0);
    impl->gripper_geom_origins[2] = smpl::Translation3(0.08, 0.0, 0.0);
    impl->gripper_geom_origins[3] = smpl::Translation3(0.08, 0.0, 0.0);

    impl->gripper_collisions = std::vector<std::unique_ptr<fcl::CollisionObject>>(4);
    impl->gripper_collisions[0] = smpl::make_unique<fcl::CollisionObject>(impl->gripper_geoms[0]);
    impl->gripper_collisions[1] = smpl::make_unique<fcl::CollisionObject>(impl->gripper_geoms[1]);
    impl->gripper_collisions[2] = smpl::make_unique<fcl::CollisionObject>(impl->gripper_geoms[2]);
    impl->gripper_collisions[3] = smpl::make_unique<fcl::CollisionObject>(impl->gripper_geoms[3]);

    impl->broadphase.registerObject(impl->obj_collision.get());
    impl->broadphase.registerObject(impl->gripper_collisions[0].get());
    impl->broadphase.registerObject(impl->gripper_collisions[1].get());
    impl->broadphase.registerObject(impl->gripper_collisions[2].get());
    impl->broadphase.registerObject(impl->gripper_collisions[3].get());
    impl->broadphase.setup();

    checker->impl = impl;

    return true;
}

void UpdateTransform(fcl::CollisionObject* obj, const smpl::Affine3& pose)
{
    auto q = smpl::Quaternion(pose.rotation());
    auto rot = fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z());
    auto pos = fcl::Vec3f(pose.translation().x(), pose.translation().y(), pose.translation().z());
    obj->setTransform(rot, pos);
    obj->computeAABB();
}

struct BroadphaseResult
{
    std::vector<std::pair<const fcl::CollisionObject*, const fcl::CollisionObject*>> collision_pairs;
} broadphase_result;

bool BroadphaseCallback(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* data)
{
    static_cast<BroadphaseResult*>(data)->collision_pairs.emplace_back(o1, o2);
    return false;
}

// NOTE: THIS FUNCTION RELIES ON THE UNDERLYING ROBOT COLLISION STATE
// HAVING BEEN UPDATED BY A CALL TO IS_STATE_VALID
bool CheckObjectGripperCollisions(ObjectManipChecker* checker)
{
    checker->parent->m_rcs->updateLinkTransform("limb_right_palm");
    checker->parent->m_rcs->updateLinkTransform("limb_right_finger_1_link_0");
    checker->parent->m_rcs->updateLinkTransform("limb_right_finger_2_link_0");
    checker->parent->m_rcs->updateLinkTransform("limb_right_finger_middle_link_0");

    auto& T_model_palm = checker->parent->m_rcs->linkTransform("limb_right_palm");
    auto& T_model_finger_1 = checker->parent->m_rcs->linkTransform("limb_right_finger_1_link_0");
    auto& T_model_finger_2 = checker->parent->m_rcs->linkTransform("limb_right_finger_2_link_0");
    auto& T_model_finger_m = checker->parent->m_rcs->linkTransform("limb_right_finger_middle_link_0");

    auto T_model_palm_geom = smpl::Affine3(T_model_palm * checker->impl->gripper_geom_origins[0]);
    auto T_model_finger_1_geom = smpl::Affine3(T_model_finger_1 * checker->impl->gripper_geom_origins[1]);
    auto T_model_finger_2_geom = smpl::Affine3(T_model_finger_2 * checker->impl->gripper_geom_origins[2]);
    auto T_model_finger_m_geom = smpl::Affine3(T_model_finger_m * checker->impl->gripper_geom_origins[3]);

    auto T_model_object_geom = checker->object_pose * checker->impl->obj_geom_origin;

    UpdateTransform(checker->impl->obj_collision.get(), T_model_object_geom);
    UpdateTransform(checker->impl->gripper_collisions[0].get(), T_model_palm_geom);
    UpdateTransform(checker->impl->gripper_collisions[1].get(), T_model_finger_1_geom);
    UpdateTransform(checker->impl->gripper_collisions[2].get(), T_model_finger_2_geom);
    UpdateTransform(checker->impl->gripper_collisions[3].get(), T_model_finger_m_geom);

    BroadphaseResult result;

    checker->impl->broadphase.update();
    checker->impl->broadphase.collide(&result, BroadphaseCallback);

    for (auto& pair : result.collision_pairs) {
        auto first_is_obj = (pair.first == checker->impl->obj_collision.get());
        auto second_is_obj = (pair.second == checker->impl->obj_collision.get());

        // gripper/gripper. skip...we should filter these out from the
        // broadphase
        // if there is some collision with the object, we did a bad thing
        if (first_is_obj | second_is_obj) {
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

            auto num_contacts = fcl::collide(pair.first, pair.second, req, res);
            if (num_contacts != 0) {
                SV_SHOW_INFO(MakeBoxMarker(T_model_object_geom, obj_length, obj_width, obj_height, "object_collision", "map"));
                SV_SHOW_INFO(MakeBoxMarker(T_model_palm_geom, palm_length, palm_width, palm_height, "gripper_collision_palm", "map"));
                SV_SHOW_INFO(MakeBoxMarker(T_model_finger_1_geom, finger_length, finger_width, finger_height, "gripper_collision_finger_1", "map"));
                SV_SHOW_INFO(MakeBoxMarker(T_model_finger_2_geom, finger_length, finger_width, finger_height, "gripper_collision_finger_2", "map"));
                SV_SHOW_INFO(MakeBoxMarker(T_model_finger_m_geom, finger_length, finger_width, finger_height, "gripper_collision_finger_m", "map"));
                return false;
            }
        }
    }

    return true;
}

void SetObjectPose(ObjectManipChecker* checker, const smpl::Affine3& pose)
{
    checker->object_pose = pose;
}

// NOTE: this might be overly generous. Also, what we really care about is how
// far the height of the robot deviates from the demonstration.
constexpr auto z_thresh = 0.03;

bool ObjectManipChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    if (state[WORLD_JOINT_Z] * state[WORLD_JOINT_Z] > z_thresh * z_thresh) {
        return false;
    }
    if (!parent->isStateValid(ExtractState(state), verbose)) {
        return false;
    }

    return CheckObjectGripperCollisions(this);
}

bool ObjectManipChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    if (start[WORLD_JOINT_Z] * start[WORLD_JOINT_Z] > z_thresh * z_thresh) {
        return false;
    }
    if (finish[WORLD_JOINT_Z] * finish[WORLD_JOINT_Z] > z_thresh * z_thresh) {
        return false;
    }

#if 0
    if (!parent->isStateToStateValid(ExtractState(start), ExtractState(finish), verbose)) {
        return false;
    }
#endif

    auto interp_path = std::vector<smpl::RobotState>();
    interp_path.reserve(4);
    interpolatePath(start, finish, interp_path);
    for (auto& point : interp_path) {
        auto s = ExtractState(point);
        if (!parent->isStateValid(s, false)) return false;

        if (!CheckObjectGripperCollisions(this)) return false;
    }

    return true;
}

bool ObjectManipChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    if (start.back() != finish.back()) {
        path.push_back(start);
        path.push_back(finish);
        return true;
    }

    auto old_size = path.size();
    if (!parent->interpolatePath(ExtractState(start), ExtractState(finish), path)) {
        return false;
    }
    auto new_size = path.size();

    for (auto i = old_size; i != new_size; ++i) {
        path[i].push_back(start.back());
    }

    return true;
}

auto ObjectManipChecker::getCollisionModelVisualization(const smpl::RobotState& state)
    -> std::vector<smpl::visual::Marker>
{
    return parent->getCollisionModelVisualization(ExtractState(state));
}

auto ObjectManipChecker::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<CollisionChecker>()) {
        return this;
    }
    return NULL;
}

