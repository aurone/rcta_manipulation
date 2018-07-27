#ifndef CABINET_MODEL_H
#define CABINET_MODEL_H

#include <Eigen/Dense>

// cabinet is composed of:
//
// (1) a body whose shape is a hollow box, missing one face, with walls of a
//     certain thickness
// (2) a door with a rectangular shape, with a given thickness
// (3) a hinge joint defining the transform between the main body and the door,
//     located on one edge
// (4) a cylindrical handle rigidly attached to the door
//
// The origin of the cabinet is the geometric center of the hollow volume. The
// x axis is pointed along the depth of the cabinet, the y axis along its width,
// and the z axis along its height.
// The origin of the hinge joint is...
// The origin of the door is...
struct CabinetModel
{
    // true -> hinge on the right side (local -y axis) and handle on the left side (local +y axis)
    bool right;

    double depth;       // depth of the cabinet (space between the inside face of the back wall and and any of the front edges)
    double width;       // width of the cabinet (space between the inside faces of the left and right walls)
    double height;      // height of the cabinet (space between the inside faces of the top and bottom walls)
    double thickness;   // uniform thickness of each wall

    double handle_offset_y; // how far from the center of the door is the handle positioned
    double handle_offset_x; // how much space between the door and the handle
    double handle_height; // how tall is the handle
    double handle_radius; // how thick is the handle (and its supports)
};

auto GetCabinetTopGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;
auto GetCabinetBottomGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;
auto GetCabinetLeftGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;
auto GetCabinetRightGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;
auto GetCabinetBackGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;

auto GetCabinetTopGeometrySize(CabinetModel* model) -> Eigen::Vector3d;
auto GetCabinetBottomGeometrySize(CabinetModel* model) -> Eigen::Vector3d;
auto GetCabinetLeftGeometrySize(CabinetModel* model) -> Eigen::Vector3d;
auto GetCabinetRightGeometrySize(CabinetModel* model) -> Eigen::Vector3d;
auto GetCabinetBackGeometrySize(CabinetModel* model) -> Eigen::Vector3d;

auto GetCabinetTopGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;
auto GetCabinetBottomGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;
auto GetCabinetLeftGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;
auto GetCabinetRightGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;
auto GetCabinetBackGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;

// hinge joint origin (local)
auto GetHingeOrigin(CabinetModel* model) -> Eigen::Affine3d;

auto GetHingeAxis(CabinetModel* model) -> Eigen::Vector3d;
auto GetHingeLowerLimit(CabinetModel* model) -> double;
auto GetHingeUpperLimit(CabinetModel* model) -> double;
auto GetHingeSpan(CabinetModel* model) -> double;

auto GetHingeDefaultPosition(CabinetModel* model) -> double;

// hinge joint transform (local)
auto GetHingeTransform(CabinetModel* model, double pos) -> Eigen::Affine3d;

// door link transform (global)
auto GetDoorPose(CabinetModel* model, double pos) -> Eigen::Affine3d;

// door geometry transform (local)
auto GetDoorGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;
auto GetDoorGeometrySize(CabinetModel* model) -> Eigen::Vector3d;

// door geometry pose (global)
auto GetDoorGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;

// handle joint origin (local)
auto GetHandleOrigin(CabinetModel* model) -> Eigen::Affine3d;

// handle joint transform (local)
auto GetHandleTransform(CabinetModel* model, double pos) -> Eigen::Affine3d;

// handle transform (global)
auto GetHandlePose(CabinetModel* model, double pos) -> Eigen::Affine3d;

// handle geometry transform (local)
auto GetHandleGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;

// handle geometry pose (global)
auto GetHandleGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;

// handle geometry transforms (local)
auto GetHandleLowerGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;
auto GetHandleUpperGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d;

// handle geometry transforms (global)
auto GetHandleLowerGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;
auto GetHandleUpperGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d;

// handle kinematics
auto GetHandleRotationRadius(CabinetModel* model) -> double;
auto GetHandleRotationOffset(CabinetModel* model) -> double;

/////////
// OLD //
/////////

// hinge origin in cabinet frame
auto GetCabinetToHingeTransform(CabinetModel* model) -> Eigen::Affine3d;

auto GetHingeFrame(CabinetModel* model, const Eigen::Affine3d* pose) -> Eigen::Affine3d;

// handle origin in door/hinge frame
auto GetHingeToHandleTransform(CabinetModel* model) -> Eigen::Affine3d;
auto GetHingeTransformNormalized(CabinetModel* model, double pos) -> Eigen::Affine3d;

#endif

