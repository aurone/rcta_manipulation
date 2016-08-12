#include "WorldModel.h"

WorldModel::WorldModel(const std::string& world_frame) :
    world_frame_(world_frame),
    collision_objects_()
{
}

WorldModel::~WorldModel()
{
}

bool WorldModel::add_collision_object(const moveit_msgs::CollisionObject& collision_object)
{
    return false;
}

bool WorldModel::remove_collision_object(const std::string& object_name)
{
    return false;
}

bool WorldModel::get_collision_object(
    const std::string& object_name,
    moveit_msgs::CollisionObject& obj_out)
{
    return false;
}

WorldModel::CollisionObjectsConstIterator
WorldModel::collision_objects_begin() const
{
    return CollisionObjectsConstIterator();
}

WorldModel::CollisionObjectsConstIterator
WorldModel::collision_objects_end() const
{
    return CollisionObjectsConstIterator();
}
