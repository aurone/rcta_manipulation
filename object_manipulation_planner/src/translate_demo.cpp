#include <fstream>
#include <Eigen/Dense>
#include <smpl/csv_parser.h>
#include <smpl/angles.h>

#define RIGHT_ARM 1

struct CabinetModel
{
    double depth;
    double width;
    double height;
    double thickness;

    double handle_offset_y; // how far from the center of the door is the handle positioned
    double handle_offset_x; // how much space between the door and the handle
    double handle_height; // how tall is the handle
    double handle_radius; // how thick is the handle (and its supports)
};

int main(int argc, char* argv[])
{
    if (argc != 5) {
        fprintf(stderr, "translate_demo <demo file> <x> <y> <theta>\n");
        return 1;
    }

    auto demo_filename = argv[1];
    auto new_x = std::stod(argv[2]);
    auto new_y = std::stod(argv[3]);
    auto new_theta = std::stod(argv[4]);

    CabinetModel cabinet;
    cabinet.width = 0.50;
    cabinet.height = 0.80;
    cabinet.depth = 0.50;
    cabinet.thickness = 0.02;
#if RIGHT_ARM
    cabinet.handle_offset_y = -0.4 * cabinet.width;
#else
    cabinet.handle_offset_y = 0.4 * cabinet.width;
#endif
    cabinet.handle_offset_x = 0.08;
    cabinet.handle_height = 0.20;
    cabinet.handle_radius = 0.01;

    // cabinet pose from the original demonstration
    Eigen::Affine3d cabinet_pose =
            Eigen::Translation3d(2.0, 0.0, 0.5 * cabinet.height) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    Eigen::Affine3d new_cabinet_pose =
            Eigen::Translation3d(new_x, new_y, 0.5 * cabinet.height) *
            Eigen::AngleAxisd(new_theta, Eigen::Vector3d::UnitZ());

    // cabinet -> new cabinet = cabinet -> world * world -> new cabinet
    Eigen::Affine3d cabinet_transform = cabinet_pose.inverse() * new_cabinet_pose;

    std::ifstream ifs(demo_filename);
    if (!ifs.is_open()) {
        fprintf(stderr, "Failed to open %s\n", demo_filename);
        return 2;
    }
    smpl::CSVParser parser;
    if (!parser.parseStream(ifs, true)) {
        fprintf(stderr, "Failed to parse %s\n", demo_filename);
        return 3;
    }

    if (parser.fieldCount() != 12) {
        fprintf(stderr, "Expected 12 fields\n");
        return 4;
    }

    FILE* f = fopen("translated.csv", "w");
    if (f == NULL) {
        fprintf(stderr, "Failed to open translated.csv for writing\n");
        return 5;
    }

    for (int i = 0; i < parser.fieldCount(); ++i) {
        fprintf(f, "%s", parser.nameAt(i).c_str());
        if (i == parser.fieldCount() - 1) {
            fprintf(f, "\n");
        } else {
            fprintf(f, ",");
        }
    }

    for (int i = 0; i < parser.recordCount(); ++i) {
        std::vector<double> values;
        for (int j = 0; j < 12; ++j) {
            values.push_back(std::stod(parser.fieldAt(i,j)));
        }

        Eigen::Affine3d T_robot_pose(
                Eigen::Translation3d(values[0], values[1], 0.0) *
                Eigen::AngleAxisd(values[2], Eigen::Vector3d::UnitZ()));

        // robot -> cabinet = robot -> world * world -> cabinet
        Eigen::Affine3d T_robot_cabinet = T_robot_pose.inverse() * cabinet_pose;

        // new robot = new cabinet * cabinet -> robot
        // robot -> new cabinet
//        Eigen::Affine3d T_robot_new_cabinet = T_robot_cabinet * cabinet_transform;
        Eigen::Affine3d T_robot_new_cabinet = new_cabinet_pose * T_robot_cabinet.inverse();

        for (int j = 0; j < 12; ++j) {
            switch (j) {
            case 0:
                fprintf(f, "%f", T_robot_new_cabinet.translation().x());
                break;
            case 1:
                fprintf(f, "%f", T_robot_new_cabinet.translation().y());
                break;
            case 2:
            {
                Eigen::Quaterniond q(T_robot_new_cabinet.rotation());
                fprintf(f, "%f", smpl::angles::get_nearest_planar_rotation(q));
                break;
            }
            default:
                fprintf(f, "%f", values[j]);
                break;
            }

            if (j == 11) {
                fprintf(f, "\n");
            } else {
                fprintf(f, ",");
            }
        }
    }

    return 0;
}
