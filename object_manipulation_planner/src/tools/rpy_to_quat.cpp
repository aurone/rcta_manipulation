#include <Eigen/Geometry>
int main(int argc, char* argv[])
{
    if (argc < 4) {
        printf("Usage: rpy_to_quat <r> <p> <y>\n");
        return 0;
    }

    auto r = std::stoi(argv[1]);
    auto p = std::stoi(argv[2]);
    auto y = std::stoi(argv[3]);

    auto q = Eigen::Quaterniond(
        Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()));
    printf("%f %f %f %f\n", q.w(), q.x(), q.y(), q.z());
    return 0;
}
