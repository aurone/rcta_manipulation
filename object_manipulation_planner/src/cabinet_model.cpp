#include "cabinet_model.h"

auto GetCabinetTopGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(Eigen::Translation3d(0.0, 0.0, 0.5 * model->height - 0.5 * model->thickness));
}

auto GetCabinetBottomGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(Eigen::Translation3d(0.0, 0.0, -0.5 * model->height + 0.5 * model->thickness));
}

auto GetCabinetLeftGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(Eigen::Translation3d(0.0, -0.5 * model->width + 0.5 * model->thickness, 0.0));
}

auto GetCabinetRightGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(Eigen::Translation3d(0.0, 0.5 * model->width - 0.5 * model->thickness, 0.0));
}

auto GetCabinetBackGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(Eigen::Translation3d(-0.5 * model->depth - 0.5 * model->thickness, 0.0, 0.0));
}

auto GetCabinetTopGeometrySize(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d(model->depth, model->width, model->thickness);
}

auto GetCabinetBottomGeometrySize(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d(model->depth, model->width, model->thickness);
}

auto GetCabinetLeftGeometrySize(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d(model->depth, model->thickness, model->height - 2.0 * model->thickness);
}

auto GetCabinetRightGeometrySize(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d(model->depth, model->thickness, model->height - 2.0 * model->thickness);
}

auto GetCabinetBackGeometrySize(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d(model->thickness, model->width, model->height);
}

auto GetCabinetTopGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetCabinetTopGeometryOrigin(model);
}

auto GetCabinetBottomGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetCabinetBottomGeometryOrigin(model);
}

auto GetCabinetLeftGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetCabinetLeftGeometryOrigin(model);
}

auto GetCabinetRightGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetCabinetRightGeometryOrigin(model);
}

auto GetCabinetBackGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetCabinetBackGeometryOrigin(model);
}

auto GetHingeOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    double y_offset;
    if (model->right) {
        y_offset = 0.5 * model->width - 0.5 * model->thickness;
    } else {
        y_offset = -0.5 * model->width + 0.5 * model->thickness;
    }

    return Eigen::Affine3d(Eigen::Translation3d(
                0.5 * model->depth + 0.5 * model->thickness,
                y_offset,
                0.0));
}

auto GetHingeAxis(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d::UnitZ();
}

auto GetHingeLowerLimit(CabinetModel* model) -> double
{
    if (model->right) {
        return 0.0;
    } else {
        return -0.75 * M_PI;
    }
}

auto GetHingeUpperLimit(CabinetModel* model) -> double
{
    if (model->right) {
        return 0.75 * M_PI;
    } else {
        return 0.0;
    }
}

auto GetHingeSpan(CabinetModel* model) -> double
{
    return GetHingeUpperLimit(model) - GetHingeLowerLimit(model);
}

auto GetHingeDefaultPosition(CabinetModel* model) -> double
{
    if (GetHingeLowerLimit(model) <= 0.0 && 0.0 <= GetHingeUpperLimit(model)) {
        return 0.0;
    } else {
        return 0.5 * (GetHingeLowerLimit(model) + GetHingeUpperLimit(model));
    }
}

auto GetHingeTransform(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return Eigen::Affine3d(Eigen::AngleAxisd(pos, GetHingeAxis(model)));
}

auto GetDoorPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetHingeOrigin(model) * GetHingeTransform(model, pos);
}

auto GetDoorGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    double y_offset;
    if (model->right) {
        y_offset = -0.5 * model->width + 0.5 * model->thickness;
    } else {
        y_offset = 0.5 * model->width - 0.5 * model->thickness;
    }
    return Eigen::Affine3d(Eigen::Translation3d(0.0, y_offset, 0.0));
}

auto GetDoorGeometrySize(CabinetModel* model) -> Eigen::Vector3d
{
    return Eigen::Vector3d(model->thickness, model->width, model->height);
}

auto GetDoorGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetDoorPose(model, pos) * GetDoorGeometryOrigin(model);
}

auto GetHandleOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    double y_offset;
    if (model->right) {
        y_offset = 0.5 * model->thickness - 0.5 * model->width - model->handle_offset_y;
    } else {
        y_offset = -0.5 * model->thickness + 0.5 * model->width + model->handle_offset_y;
    }

    return Eigen::Affine3d(Eigen::Translation3d(
                0.5 * model->thickness + model->handle_offset_x + model->handle_radius,
                y_offset,
                0.0));
}

auto GetHandleTransform(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return Eigen::Affine3d::Identity();
}

auto GetHandlePose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetDoorPose(model, pos) * GetHandleOrigin(model) * GetHandleTransform(model, pos);
}

auto GetHandleGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d::Identity();
}

auto GetHandleGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetHandlePose(model, pos) * GetHandleGeometryOrigin(model);
}

auto GetHandleLowerGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(
            Eigen::Translation3d(
                0.5 * (-model->handle_radius - model->handle_offset_x - 0.5 * model->thickness),
                0.0,
                -0.5 * model->handle_height + model->handle_radius) *
            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()));
}

auto GetHandleUpperGeometryOrigin(CabinetModel* model) -> Eigen::Affine3d
{
    return Eigen::Affine3d(
            Eigen::Translation3d(
                0.5 * (-model->handle_radius - model->handle_offset_x - 0.5 * model->thickness),
                0.0,
                0.5 * model->handle_height + model->handle_radius) *
            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()));
}

auto GetHandleLowerGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetHandlePose(model, pos) * GetHandleLowerGeometryOrigin(model);
}

auto GetHandleUpperGeometryPose(CabinetModel* model, double pos) -> Eigen::Affine3d
{
    return GetHandlePose(model, pos) * GetHandleUpperGeometryOrigin(model);
}

auto GetHandleRotationRadius(CabinetModel* model) -> double
{
    // TODO: + handle radius?
    auto dx = 0.5 * model->thickness + model->handle_offset_x;
    auto dy = 0.5 * model->width - 0.5 * model->thickness + model->handle_offset_y;
    return std::sqrt(dx * dx + dy * dy);
}

auto GetHandleRotationOffset(CabinetModel* model) -> double
{
    // TODO: + handle radius?
    auto dx = 0.5 * model->thickness + model->handle_offset_x;
    if (model->right) {
        auto dy = 0.5 * model->width - 0.5 * model->thickness - model->handle_offset_y;
        return atan2(fabs(dx), fabs(dy));
    } else {
        auto dy = 0.5 * model->width - 0.5 * model->thickness + model->handle_offset_y;
        return atan2(fabs(dx), fabs(dy));
    }
}

/////////
// OLD //
/////////

auto GetCabinetToHingeTransform(CabinetModel* model)
    -> Eigen::Affine3d
{
    return GetHingeOrigin(model);
}

auto GetHingeFrame(CabinetModel* model, const Eigen::Affine3d* pose)
    -> Eigen::Affine3d
{
    Eigen::Affine3d T_cabinet_hinge = GetCabinetToHingeTransform(model);
    return (*pose) * T_cabinet_hinge;
}

auto GetHandlePose(CabinetModel* model, const Eigen::Affine3d* pose, double door_pos)
    -> Eigen::Affine3d
{
    Eigen::Affine3d T_cabinet_hinge = GetCabinetToHingeTransform(model);

    Eigen::Affine3d door_pose;
    door_pose = (*pose)
            * T_cabinet_hinge
            * GetHingeTransform(model, door_pos)
            * GetDoorGeometryOrigin(model);

    Eigen::Affine3d handle_pose;
    handle_pose =
            door_pose *
            Eigen::Translation3d(
                    0.5 * model->thickness + model->handle_offset_x + model->handle_radius,
                    model->handle_offset_y,
                    0.0);

    return handle_pose;
}

// frame at the center of the handle shaft
auto GetHingeToHandleTransform(CabinetModel* model) -> Eigen::Affine3d
{
    double y_offset;
    if (model->right) {
        y_offset = 0.5 * model->thickness - 0.5 * model->width - model->handle_offset_y;
    } else {
        y_offset = - 0.5 * model->thickness + 0.5 * model->width + model->handle_offset_y;
    }
    return Eigen::Affine3d(Eigen::Translation3d(
                0.5 * model->thickness + 0.5 * model->handle_offset_x + 0.5 * model->handle_radius,
                y_offset,
                0.0));
}

// hinge_pos in [0, 1], 0 = completely closed, 1 = completely open
auto GetHingeTransformNormalized(CabinetModel* model, double hinge_pos) -> Eigen::Affine3d
{
    if (!model->right) {
        hinge_pos = 1.0 - hinge_pos;
    }
    auto jpos = (1.0 - hinge_pos) * GetHingeLowerLimit(model) + hinge_pos * GetHingeUpperLimit(model);
    return GetHingeTransform(model, jpos);
}

