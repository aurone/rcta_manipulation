#include <fstream>
#include <string>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <smpl/csv_parser.h>

// modify the demonstration so that the manipulation manifold is transformed
int main(int argc, char* argv[])
{
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
    if (!parser.parseStream(ifs, true)) {
        fprintf(stderr, "Failed to parse stream\n");
        return -3;
    }

    if (parser.fieldCount() != 12) {
        fprintf(stderr, "csv file has incorrect number of fields. expected 12\n");
        return -4;
    }

    for (auto& point : demonstration) {
    }

    // extract the non-object variables
    for (auto i = 0; i < 11; ++i) {

    }

    return 0;
}
