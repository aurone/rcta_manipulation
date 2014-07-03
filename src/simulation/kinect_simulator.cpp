#include <cmath>
#include <cstdlib>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

std::string basename(const std::string& path)
{
    // search for the last directory separator
    std::string::size_type pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return path;
    }
    else {
        return path.substr(pos + 1);
    }
}

void print_usage(int argc, char* argv[])
{
    fprintf(stderr, "Usage: %s --mode=(tilting|static [angle])\n", basename(std::string(argv[0])).c_str());
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        print_usage(argc, argv);
        exit(1);
    }

    std::string mode_op(argv[1]);
    if (mode_op.size() < 7 || mode_op.substr(0, 7) != std::string("--mode=")) {
        print_usage(argc, argv);
        exit(1);
    }

    std::string mode = mode_op.substr(7);
    if (mode != "tilting" && mode != "static") {
        print_usage(argc, argv);
        exit(1);
    }

    double angle = 0.0;
    if (mode == "static" && argc > 2) {
        try {
            angle = stod(std::string(argv[2]));
        }
        catch (const std::invalid_argument& ex) {
            fprintf(stderr, "Invalid static angle\n");
            exit(1);
        }
        catch (const std::out_of_range& ex) {
            fprintf(stderr, "Value for static angle too large\n");
            exit(1);
        }
    }

    if (mode == "static") {
        printf("Simulating Static Kinect at angle %0.3f\n", angle);
    }
    if (mode == "tilting") {
        printf("Simulating Tilting Kinect starting at angle %0.3f\n", angle);
    }

    ros::init(argc, argv, "kinect_simulator");
    ros::NodeHandle nh;

    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    auto to_rads = [](double degs) { return degs * M_PI / 180.0; };

    const double max_angle_deg = 27.0;
    const double min_angle_deg = -27.0;
    double speed_dps = 40.0;

    int direction = mode == "static" ? 0 : 1;

    // simulate tilting or static kinect
    ros::Rate loop_rate(10.0);
    while (ros::ok()) {
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        static int seqno = 0;
        js.header.seq = seqno++;
        js.name.push_back("kinect_tilt");

        if (direction > 0) {
            angle += speed_dps * (1.0 / 10.0);
            if (angle > max_angle_deg) {
                angle = max_angle_deg;
                direction = -1;
            }
        }
        else if (direction > 0) {
            angle -= speed_dps * (1.0 / 10.0);
            if (angle < min_angle_deg) {
                angle = min_angle_deg;
                direction = 1;
            }
        }
        else {
            // kinect is not tilting; do nothing
        }

        js.position.push_back(to_rads(angle));
        joint_states_pub.publish(js);

        loop_rate.sleep();
    }

    return 0;
}