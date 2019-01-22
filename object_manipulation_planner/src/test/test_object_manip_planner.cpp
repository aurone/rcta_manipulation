#include <sstream>

#include <boost/filesystem.hpp>
#include <smpl/console/ansi.h>

// we have:
// (1) a planning library
// (2) a runtime service that configures the planning library with a planning
//     and collision model for the roman.
// (3) a test executable that sends a planning [and execution] request to the
//     runtime service
//
// For the test harness, we could either:
// (1) Configure the planning library similarly to the runtime service and
//     perform multiple planning queries
// (2) Send planning requests to the runtime service.
// (3) Automatically generate configuration and launch files for the test
//     executable and run the test several times in succession
//
// Option (1) is the most flexible, allowing us to query the planner multiple
// times at once (or instantiate multiple planners while sharing thread-safe
// data structures between them). However, we may duplicate a lot of planner
// configuration or begin to deviate from the runtime service.
//
// Option (2) limits us to testing one query at a time, but reuses the same code
// and configuration that will run on the robot.
//
// Option (3) stresses the test executable and avoids writing common
// configuration parsing code for a scenario. However, we'll end up working
// around roslaunch to automatically generate test configurations.

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: test_object_manip_planner <path/to/tests>\n");
        return 0;
    }

    namespace bfs = boost::filesystem;

    auto path = bfs::path(argv[1]);
    if (!is_directory(path)) {
        fprintf(stderr, "path argument must be a directory\n");
        return -1;
    }

    auto use_rosrun = true;

    for (auto it = bfs::directory_iterator(path);
        it != bfs::directory_iterator();
        ++it)
    {
        if (use_rosrun) {
            auto err = 0;
            std::stringstream ss;
            auto cmd = std::string();

            ss << "roslaunch --disable-title object_manipulation_planner manipulate_object_params.launch scenario:=" << it->path().generic_string() << " > /dev/null";
            cmd = ss.str();
            printf("%sexecute '%s'%s\n", smpl::console::codes::cyan, cmd.c_str(), smpl::console::codes::reset);
            err = system(cmd.c_str());
            if (err != 0) {
                fprintf(stderr, "%s -> command returned exit status (%d)%s\n", smpl::console::codes::red, err, smpl::console::codes::reset);
            } else {
                printf("%s -> command returned exit status (%d)%s\n", smpl::console::codes::green, err, smpl::console::codes::reset);
            }

            ss.str("");
            ss << "rosrun object_manipulation_planner manipulate_object > /dev/null";
            cmd = ss.str();
            printf("%sexecute '%s'%s\n", smpl::console::codes::cyan, cmd.c_str(), smpl::console::codes::reset);
            err = system(cmd.c_str());

            if (err != 0) {
                fprintf(stderr, "%s -> command returned exit status (%d)%s\n", smpl::console::codes::red, err, smpl::console::codes::reset);
            } else {
                printf("%s -> command returned exit status (%d)%s\n", smpl::console::codes::green, err, smpl::console::codes::reset);
            }
        } else {
            auto cmd = std::string("roslaunch");
            cmd += " --disable-title";
//        cmd += " --screen";
            cmd += " object_manipulation_planner manipulate_object.launch";
            cmd += std::string(" scenario:=") + it->path().generic_string();
            cmd += " > /dev/null";
            printf("%sexecute '%s'%s\n", smpl::console::codes::cyan, cmd.c_str(), smpl::console::codes::reset);
            auto err = system(cmd.c_str());
            if (err != 0) {
                fprintf(stderr, "%s -> command returned exit status (%d)%s\n", smpl::console::codes::red, err, smpl::console::codes::reset);
            } else {
                printf("%s -> command returned exit status (%d)%s\n", smpl::console::codes::green, err, smpl::console::codes::reset);
            }
        }
    }

    return 0;
}
