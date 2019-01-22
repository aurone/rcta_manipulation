#include <stdio.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <smpl/csv_parser.h>

// modify the demonstration so that the manipulation manifold is transformed
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "transform_demo");
    ros::NodeHandle nh;

    ROSCONSOLE_AUTOINIT;
//    ros::console::set_logger_level("log4j.logger.ros", ros::console::levels::Error);
//    ros::console::set_logger_level("logger.ros", ros::console::levels::Error);
    ros::console::set_logger_level("ros", ros::console::levels::Error);
//    ros::console::set_logger_level("", ros::console::levels::Error);

    if (argc < 5) {
        fprintf(stderr, "Usage: transform_demo <demo file> <x> <y> <z> [Y] [P] [R]\n");
        return 1;
    }

    auto demo_filename = argv[1];
    auto dx = atof(argv[2]);
    auto dy = atof(argv[3]);
    auto dz = atof(argv[4]);
    auto dY = argc > 5 ? atof(argv[5]) : 0.0;
    auto dP = argc > 6 ? atof(argv[6]) : 0.0;
    auto dR = argc > 7 ? atof(argv[7]) : 0.0;

    auto delta = Eigen::Affine3d(
            Eigen::Translation3d(dx, dy, dz) *
            Eigen::AngleAxisd(dY, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(dP, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(dR, Eigen::Vector3d::UnitX()));

    robot_model_loader::RobotModelLoader loader;
    auto model = loader.getModel();

    if (model == NULL) {
        fprintf(stderr, "Failed to load model\n");
        return -1;
    }

    moveit::core::RobotState robot_state(model);

    std::ifstream ifs(demo_filename);
    if (!ifs.is_open()) {
        fprintf(stderr, "Failed to open %s\n", demo_filename);
        return -2;
    }

    smpl::CSVParser parser;
    auto has_header = true;
    if (!parser.parseStream(ifs, has_header)) {
        fprintf(stderr, "Failed to parse stream\n");
        return -3;
    }

    // NOTE: hardcoding the joints from the group we're using
    if (parser.fieldCount() != 12) {
        fprintf(stderr, "csv file has incorrect number of fields. expected 12\n");
        return -4;
    }

    // seed the robot with the first state on the demonstration
    for (auto i = 0; i < 11; ++i) {
        auto pos = std::stod(parser.fieldAt(0, i));
        robot_state.setVariablePosition(parser.nameAt(i), pos);
    }

    auto* group = model->getJointModelGroup("right_arm_and_torso");
    if (group == NULL) {
        ROS_ERROR("Group 'right_arm_and_torso' not in robot model");
        return -5;
    }

    std::vector<double> new_positions;
    for (auto i = 0; i < parser.recordCount(); ++i) {
        moveit::core::RobotState demo_state(model);
        // read in the demo state
        for (auto j = 0; j < 11; ++j) {
            auto pos = std::stod(parser.fieldAt(i, j));
            demo_state.setVariablePosition(parser.nameAt(j), pos);
        }

        auto& T_map_ee = demo_state.getGlobalLinkTransform("limb_right_link7");
        auto T_map_goal = Eigen::Affine3d(delta * T_map_ee);

        if (!demo_state.setFromIK(group, T_map_goal)) {
            ROS_ERROR("Failed to compute IK");
            return -6;
        }

        for (auto j = 0; j < 11; ++j) {
            new_positions.push_back(demo_state.getVariablePosition(parser.nameAt(j)));
        }
        new_positions.push_back(std::stod(parser.fieldAt(i, 11)));
    }

    for (auto i = 0; i < 12; ++i) {
        if (i != 0) printf(",");
        printf("%s", parser.nameAt(i).c_str());
    }
    for (auto i = 0; i < new_positions.size(); ++i) {
        if (i % 12 == 0) printf("\n");
        else printf(",");
        printf("%f", new_positions[i]);
    }
    printf("\n");

    return 0;
}
