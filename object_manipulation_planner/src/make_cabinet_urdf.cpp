#include <stdio.h>

#include "cabinet_model.h"

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: make_cabinet_urdf <filename>\n");
        return 1;
    }

    CabinetModel cabinet;
    cabinet.right = true;
    cabinet.width = 0.50;
    cabinet.height = 0.80;
    cabinet.depth = 0.50;
    cabinet.thickness = 0.02;
    cabinet.handle_offset_y = 0.4 * cabinet.width;
    cabinet.handle_offset_x = 0.08;
    cabinet.handle_height = 0.20;
    cabinet.handle_radius = 0.01;

    int depth = 0;

    FILE* f = fopen(argv[1], "w");
    fprintf(f, "<?xml version=\"1.0\"?>\n");
    fprintf(f, "<robot name=\"cabinet\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n");

    ++depth;
    auto print_depth = [&]() { fprintf(f, "%*s", 4 * depth, ""); };

    auto print_box_geometry = [&](const Eigen::Affine3d& origin, const Eigen::Vector3d& size)
    {
        Eigen::Vector3d rpy = origin.rotation().eulerAngles(0, 1, 2);
        ++depth;
        print_depth(); fprintf(f, "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>\n", origin.translation().x(), origin.translation().y(), origin.translation().z(), rpy.x(), rpy.y(), rpy.z());
        print_depth(); fprintf(f, "<geometry>\n");
        ++depth;
        print_depth(); fprintf(f, "<box size=\"%f %f %f\"/>\n", size.x(), size.y(), size.z());
        --depth;
        print_depth(); fprintf(f, "</geometry>\n");
        --depth;
    };

    auto print_cylinder_geometry = [&](const Eigen::Affine3d& origin, double radius, double height)
    {
        Eigen::Vector3d rpy = origin.rotation().eulerAngles(0, 1, 2);
        ++depth;
        print_depth(); fprintf(f, "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>\n", origin.translation().x(), origin.translation().y(), origin.translation().z(), rpy.x(), rpy.y(), rpy.z());
        print_depth(); fprintf(f, "<geometry>\n");
        ++depth;
        print_depth(); fprintf(f, "<cylinder radius=\"%f\" length=\"%f\"/>\n", radius, height);
        --depth;
        print_depth(); fprintf(f, "</geometry>\n");
        --depth;
    };

    auto print_box_visual = [&](const Eigen::Affine3d& origin, const Eigen::Vector3d& size) {
        print_depth(); fprintf(f, "<visual>\n");
        print_box_geometry(origin, size);
        print_depth(); fprintf(f, "</visual>\n");
    };

    auto print_cylinder_visual = [&](const Eigen::Affine3d& origin, double radius, double height)
    {
        print_depth(); fprintf(f, "<visual>\n");
        print_cylinder_geometry(origin, radius, height);
        print_depth(); fprintf(f, "</visual>\n");
    };

    auto print_box_collision = [&](const Eigen::Affine3d& origin, const Eigen::Vector3d& size) {
        print_depth(); fprintf(f, "<collision>\n");
        print_box_geometry(origin, size);
        print_depth(); fprintf(f, "</collision>\n");
    };

    auto print_cylinder_collision = [&](const Eigen::Affine3d& origin, double radius, double height)
    {
        print_depth(); fprintf(f, "<collision>\n");
        print_cylinder_geometry(origin, radius, height);
        print_depth(); fprintf(f, "</collision>\n");
    };

    print_depth(); fprintf(f, "<link name=\"base_link\">\n");
    ++depth;
    print_box_visual(GetCabinetBottomGeometryOrigin(&cabinet), GetCabinetBottomGeometrySize(&cabinet));
    print_box_visual(GetCabinetTopGeometryOrigin(&cabinet), GetCabinetTopGeometrySize(&cabinet));
    print_box_visual(GetCabinetLeftGeometryOrigin(&cabinet), GetCabinetLeftGeometrySize(&cabinet));
    print_box_visual(GetCabinetRightGeometryOrigin(&cabinet), GetCabinetRightGeometrySize(&cabinet));
    print_box_visual(GetCabinetBackGeometryOrigin(&cabinet), GetCabinetBackGeometrySize(&cabinet));
    print_box_collision(GetCabinetBottomGeometryOrigin(&cabinet), GetCabinetBottomGeometrySize(&cabinet));
    print_box_collision(GetCabinetTopGeometryOrigin(&cabinet), GetCabinetTopGeometrySize(&cabinet));
    print_box_collision(GetCabinetLeftGeometryOrigin(&cabinet), GetCabinetLeftGeometrySize(&cabinet));
    print_box_collision(GetCabinetRightGeometryOrigin(&cabinet), GetCabinetRightGeometrySize(&cabinet));
    print_box_collision(GetCabinetBackGeometryOrigin(&cabinet), GetCabinetBackGeometrySize(&cabinet));
    --depth;
    print_depth(); fprintf(f, "</link>\n");

    auto J0 = GetHingeOrigin(&cabinet);
    auto J0_rpy = J0.rotation().eulerAngles(0, 1, 2);
    auto J0_axis = GetHingeAxis(&cabinet);

    print_depth(); fprintf(f, "<joint name=\"door_joint\" type=\"revolute\">\n");
    ++depth;
    print_depth(); fprintf(f, "<parent link=\"base_link\"/>\n");
    print_depth(); fprintf(f, "<child link=\"door\"/>\n");
    print_depth(); fprintf(f, "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>\n", J0.translation().x(), J0.translation().y(), J0.translation().z(), J0_rpy.x(), J0_rpy.y(), J0_rpy.z());
    print_depth(); fprintf(f, "<axis xyz=\"%f %f %f\"/>\n", J0_axis.x(), J0_axis.y(), J0_axis.z());
    print_depth(); fprintf(f, "<limit lower=\"%f\" upper=\"%f\" velocity=\"0\" effort=\"0\"/>\n", GetHingeLowerLimit(&cabinet), GetHingeUpperLimit(&cabinet));
    --depth;
    print_depth(); fprintf(f, "</joint>\n");

    print_depth(); fprintf(f, "<link name=\"door\">\n");
    ++depth;
    print_box_visual(GetDoorGeometryOrigin(&cabinet), GetDoorGeometrySize(&cabinet));
    print_box_collision(GetDoorGeometryOrigin(&cabinet), GetDoorGeometrySize(&cabinet));
    --depth;
    print_depth(); fprintf(f, "</link>\n");

    auto T_J1 = GetHandleOrigin(&cabinet);
    auto T_J1_rpy = T_J1.rotation().eulerAngles(0, 1, 2);

    print_depth(); fprintf(f, "<joint name=\"handle_joint\" type=\"fixed\">\n");
    ++depth;
    print_depth(); fprintf(f, "<parent link=\"door\"/>\n");
    print_depth(); fprintf(f, "<child link=\"handle\"/>\n");
    print_depth(); fprintf(f, "<origin xyz=\"%f %f %f\" rpy=\"%f %f %f\"/>\n", T_J1.translation().x(), T_J1.translation().y(), T_J1.translation().z(), T_J1_rpy.x(), T_J1_rpy.y(), T_J1_rpy.z());
    --depth;
    print_depth(); fprintf(f, "</joint>\n");

    print_depth(); fprintf(f, "<link name=\"handle\">\n");
    ++depth;
    print_cylinder_visual(GetHandleGeometryOrigin(&cabinet), GetHandleGeometryRadius(&cabinet), GetHandleGeometryHeight(&cabinet));
    print_cylinder_visual(GetHandleLowerGeometryOrigin(&cabinet), GetHandleLowerGeometryRadius(&cabinet), GetHandleLowerGeometryHeight(&cabinet));
    print_cylinder_visual(GetHandleUpperGeometryOrigin(&cabinet), GetHandleUpperGeometryRadius(&cabinet), GetHandleUpperGeometryHeight(&cabinet));
    print_cylinder_collision(GetHandleGeometryOrigin(&cabinet), GetHandleGeometryRadius(&cabinet), GetHandleGeometryHeight(&cabinet));
    print_cylinder_collision(GetHandleLowerGeometryOrigin(&cabinet), GetHandleLowerGeometryRadius(&cabinet), GetHandleLowerGeometryHeight(&cabinet));
    print_cylinder_collision(GetHandleUpperGeometryOrigin(&cabinet), GetHandleUpperGeometryRadius(&cabinet), GetHandleUpperGeometryHeight(&cabinet));
    --depth;
    print_depth(); fprintf(f, "</link>\n");

    --depth;
    fprintf(f, "</robot>\n");

    fclose(f);
}
