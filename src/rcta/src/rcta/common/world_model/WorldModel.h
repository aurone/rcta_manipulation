#ifndef WorldModel_h
#define WorldModel_h

#include <string>
#include <unordered_map>
#include <moveit_msgs/CollisionObject.h>

class WorldModel
{
    typedef std::unordered_map<std::string, moveit_msgs::CollisionObject> CollisionObjectMap;

public:

    typedef CollisionObjectMap::const_iterator CollisionObjectsConstIterator;

    WorldModel(const std::string& world_frame);
    ~WorldModel();

    bool add_collision_object(const moveit_msgs::CollisionObject& collision_object);
    bool remove_collision_object(const std::string& object_name);
    bool get_collision_object(
            const std::string& object_name,
            moveit_msgs::CollisionObject& obj_out);

    CollisionObjectsConstIterator collision_objects_begin() const;
    CollisionObjectsConstIterator collision_objects_end() const;

private:

    std::string world_frame_;
    CollisionObjectMap collision_objects_;
};

#endif
