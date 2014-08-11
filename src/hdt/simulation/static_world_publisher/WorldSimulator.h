#ifndef WorldSimulator_h
#define WorldSimulator_h

#include <string>
#include <moveit_msgs/CollisionObject.h>

class WorldSimulator
{
    typedef std::unordered_map<std::string, moveit_msgs::CollisionObject> CollisionObjectMap;
    typedef CollisionObjectMap::const_iterator CollisionObjectsConstIterator;

    CollisionObjectMap collision_objects_;

public:

    WorldSimulator(const std::string& world_frame);
    ~WorldSimulator();

    void add_collision_object(const moveit_msgs::CollisionObject& collision_object);
    void remove_collision_object(const std::string& object_name);

    bool get_collision_object(const std::string& object_name);

    CollisionObjectsConstIterator collision_objects_begin() const;
    CollisionObjectsConstIterator collision_objects_end() const;
};

#endif
