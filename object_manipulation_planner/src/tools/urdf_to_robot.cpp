#include <urdf/model.h>
#include <stdio.h>

//#define TRY(func) do { if (!(func)) return false; } while(0)

#define TRY(func) do { if (!(func)) std::abort(); } while (0)

template <class T>
bool serialize_pod(const T& obj, FILE* f)
{
    auto read = fwrite(&obj, sizeof(obj), 1, f);
    return read == 1;
}

template <class T>
bool serialize(const std::vector<T>& v, FILE* f);

bool serialize(const std::string& s, FILE* f);

bool serialize(size_t s, FILE* f);

bool serialize(bool s, FILE* f);

//////////////////////////////
// Basic type serialization //
//////////////////////////////

template <class T>
bool serialize(const std::vector<T>& v, FILE* f)
{
    serialize(v.size(), f);
    for (auto& e : v) {
        serialize(e, f);
    }
    return true;
}

bool deserialize(std::string* s, FILE* f)
{
    auto size = size_t(0);
    fread(&size, sizeof(size), 1, f);
    auto buff = std::vector<char>(size);
    fread(buff.data(), sizeof(buff[0]), 1, f);

    *s = std::string{ buff.data(), size };
    return true;
}

bool serialize(const std::string& s, FILE* f)
{
    auto wrote = size_t(0);
    auto size = s.size();
    TRY(serialize(s.size(), f));
    wrote = fwrite(s.c_str(), sizeof(s[0]), s.size(), f);
    return true;
}

///////////////////
// Integer types //
///////////////////

bool serialize(signed char c, FILE* f)
{
    return serialize_pod(c, f);
}

bool serialize(unsigned char c, FILE* f)
{
    return serialize_pod(c, f);
}

bool serialize(signed short int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(unsigned short int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(signed int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(unsigned int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(signed long int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(unsigned long int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(signed long long int i, FILE* f)
{
    return serialize_pod(i, f);
}

bool serialize(unsigned long long int i, FILE* f)
{
    return serialize_pod(i, f);
}

//////////////////
// Boolean Type //
//////////////////

bool serialize(bool b, FILE* f)
{
    return serialize_pod(b, f);
}

//////////////////////////
// Floating-point Types //
//////////////////////////

bool serialize(float d, FILE* f)
{
    return serialize_pod(d, f);
}

bool serialize(double d, FILE* f)
{
    return serialize_pod(d, f);
}

bool serialize(long double d, FILE* f)
{
    return serialize_pod(d, f);
}

////////////////
// URDF Types //
////////////////

bool serialize(const urdf::Vector3& v, FILE* f)
{
    return serialize_pod(v, f);
}

bool serialize(const urdf::Rotation& r, FILE* f)
{
    return serialize_pod(r, f);
}

bool serialize(const urdf::Pose& p, FILE* f)
{
    TRY(serialize(p.position, f));
    TRY(serialize(p.rotation, f));
}

bool serialize(const urdf::Inertial& inertial, FILE* f)
{
    TRY(serialize(inertial.origin, f));
    TRY(serialize(inertial.mass, f));
    TRY(serialize(inertial.ixx, f));
    TRY(serialize(inertial.ixy, f));
    TRY(serialize(inertial.ixz, f));
    TRY(serialize(inertial.iyy, f));
    TRY(serialize(inertial.iyz, f));
    TRY(serialize(inertial.izz, f));
    return true;
}

bool serialize(const urdf::Geometry& geometry, FILE* f)
{
    TRY(serialize(geometry.type, f));
    switch (geometry.type) {
    case urdf::Geometry::SPHERE:
    {
        auto* s = (const urdf::Sphere*)(&geometry);
        TRY(serialize(s->radius, f));
        break;
    }
    case urdf::Geometry::BOX:
    {
        auto* b = (const urdf::Box*)(&geometry);
        TRY(serialize(b->dim, f));
        break;
    }
    case urdf::Geometry::CYLINDER:
    {
        auto* c = (const urdf::Cylinder*)(&geometry);
        TRY(serialize(c->length, f));
        TRY(serialize(c->radius, f));
        break;
    }
    case urdf::Geometry::MESH:
    {
        auto* m = (const urdf::Mesh*)(&geometry);
        TRY(serialize(m->filename, f));
        TRY(serialize(m->scale, f));
        break;
    }
    default:
        return false;
    }
    return true;
}

bool serialize(const urdf::Collision& collision, FILE* f)
{
    TRY(serialize(collision.origin, f));
    assert(collision.geometry != NULL);
    TRY(serialize(*collision.geometry, f));
    return true;
}

bool serialize(const urdf::Visual& visual, FILE* f)
{
    TRY(serialize(visual.origin, f));
    assert(visual.geometry != NULL);
    TRY(serialize(*visual.geometry, f));
    TRY(serialize(visual.material_name, f));
    return true;
}

bool serialize(const urdf::Link& link, FILE* f)
{
    TRY(serialize(link.name, f));

    TRY(serialize(link.inertial != NULL, f));
    if (link.inertial != NULL) {
        TRY(serialize(*link.inertial, f));
    }

    auto collisions = std::vector<boost::shared_ptr<urdf::Collision>>();
    if (link.collision != NULL) {
        collisions.push_back(link.collision);
    } else {
        collisions.insert(end(collisions), begin(link.collision_array), end(link.collision_array));
    }

    TRY(serialize(collisions.size(), f));
    for (auto& collision : collisions) {
        TRY(serialize(*collision, f));
    }

    auto visuals = std::vector<boost::shared_ptr<urdf::Visual>>();
    if (link.visual != NULL) {
        visuals.push_back(link.visual);
    } else {
        visuals.insert(end(visuals), begin(link.visual_array), end(link.visual_array));
    }
    TRY(serialize(visuals.size(), f));
    for (auto& visual : visuals) {
        TRY(serialize(*visual, f));
    }

    return true;
}

bool serialize(const urdf::JointDynamics& d, FILE* f)
{
    TRY(serialize(d.damping, f));
    TRY(serialize(d.friction, f));
    return true;
}

bool serialize(const urdf::JointLimits& l, FILE* f)
{
    TRY(serialize(l.lower, f));
    TRY(serialize(l.upper, f));
    TRY(serialize(l.effort, f));
    TRY(serialize(l.velocity, f));
    return true;
}

bool serialize(const urdf::JointMimic& m, FILE* f)
{
    TRY(serialize(m.offset, f));
    TRY(serialize(m.multiplier, f));
    TRY(serialize(m.joint_name, f));
    return true;
}

bool serialize(const urdf::Joint& joint, FILE* f)
{
    TRY(serialize(joint.type, f));
    TRY(serialize(joint.name, f));
    TRY(serialize(joint.axis, f));
    TRY(serialize(joint.child_link_name, f));
    TRY(serialize(joint.parent_link_name, f));
    TRY(serialize(joint.parent_to_joint_origin_transform, f));
    TRY(serialize(joint.dynamics != NULL, f));
    if (joint.dynamics != NULL) {
        TRY(serialize(*joint.dynamics, f));
    }
    TRY(serialize(joint.limits != NULL, f));
    if (joint.limits != NULL) {
        TRY(serialize(*joint.limits, f));
    }
    TRY(serialize(joint.mimic != NULL, f));
    if (joint.mimic != NULL) {
        TRY(serialize(*joint.mimic, f));
    }
    return true;
}

bool serialize(const urdf::Color& color, FILE* f)
{
    fwrite(&color.r, sizeof(color.r), 1, f);
    fwrite(&color.g, sizeof(color.g), 1, f);
    fwrite(&color.b, sizeof(color.b), 1, f);
    fwrite(&color.a, sizeof(color.a), 1, f);
    return true;
}

bool serialize(const urdf::Material& material, FILE* f)
{
    TRY(serialize(material.name, f));
    TRY(serialize(material.texture_filename, f));
    TRY(serialize(material.color, f));
    return true;
}

bool serialize(const urdf::ModelInterface& urdf, FILE* f)
{
    TRY(serialize(urdf.name_, f));
    TRY(serialize(urdf.root_link_->name, f));

    TRY(serialize(urdf.links_.size(), f));
    for (auto& e : urdf.links_) {
        TRY(serialize(*e.second, f));
    }

    TRY(serialize(urdf.joints_.size(), f));
    for (auto& e : urdf.joints_) {
        TRY(serialize(*e.second, f));
    }

    TRY(serialize(urdf.materials_.size(), f));
    for (auto& e : urdf.materials_) {
        TRY(serialize(*e.second, f));
    }

    return true;
}

// Load a URDF from XML file and export to a serialized binary format.
// TODO: Mesh and material data are stored externally to the URDF. Should
// we include those files within the binary archive or fixup the paths to
// not use ROS paths.
int main(int argc, char* argv[])
{
    if (argc < 3) {
        fprintf(stderr, "usage: urdf_to_robot <path/to/urdf> <path/to/output>\n");
        return 1;
    }

    auto filepath = argv[1];
    auto ofp = argv[2];

    auto urdf = urdf::Model{ };
    if (!urdf.initFile(filepath)) {
        fprintf(stderr, "Failed to parse URDF\n");
        return 1;
    }

    auto* f = fopen(ofp, "w");
    if (f == NULL) {
        fprintf(stderr, "Failed to open archive file for writing\n");
        return 2;
    }

    if (!serialize(urdf, f)) {
        fprintf(stderr, "Failed to serialize URDF\n");
        return 3;
    }

    return 0;
}

