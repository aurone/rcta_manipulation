#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <ros/ros.h>
#include <spellbook/grid/grid.h>
#include <smpl/angles.h>
#include <smpl/intrusive_heap.h>

const double MIN_YAW = -M_PI;
const double MIN_PITCH = -M_PI / 2.0;
const double MIN_ROLL = -M_PI;
const double MAX_YAW = M_PI;
const double MAX_PITCH = M_PI / 2.0;
const double MAX_ROLL = M_PI;

// cell centered on 0
int discretize(double x, double min, double res)
{
    return (int)floor((x - min) / res + 0.5);
};

double realize(int x, double min, double res)
{
    return min + x * res;
};

// generate a mapping from points in 3d space to the number of orientations that
// can be achieved via ik
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "analyze_workspace");
    ros::NodeHandle nh;

    robot_model_loader::RobotModelLoader rml;
    auto robot_model = rml.getModel();
    moveit::core::RobotState robot_state(robot_model);

    const std::string group_name = "right_arm";
    const moveit::core::JointModelGroup* joint_group = robot_model->getJointModelGroup(group_name);
    if (!joint_group) {
        ROS_ERROR("Failed to find joint group '%s' in the robot model", joint_group->getName().c_str());
        return 1;
    }
    const moveit::core::LinkModel* tip_link = joint_group->getOnlyOneEndEffectorTip();
    if (!tip_link) {
        ROS_ERROR("Joint group has no tip link!");
        return 1;
    }

    /////////////////////
    /// CONFIGURATION ///
    /////////////////////

    moveit_msgs::WorkspaceParameters workspace;
    workspace.header.frame_id = robot_model->getModelFrame();
    workspace.min_corner.x = -0.5;
    workspace.min_corner.y = -1.5;
    workspace.min_corner.z = 0.0;
    workspace.max_corner.x = 1.5;
    workspace.max_corner.y = 0.5;
    workspace.max_corner.z = 1.9;

    const double workspace_res = 0.02;
    const double workspace_radius = 1.5;

    const int Y_samples = 4;
    const int P_samples = 3;
    const int R_samples = 1;

    const double FP_IRES = 1000.0;

    // round to the nearest desired samples
    const int x_samples = std::round((workspace.max_corner.x - workspace.min_corner.y) / workspace_res) + 1;
    const int y_samples = std::round((workspace.max_corner.y - workspace.min_corner.y) / workspace_res) + 1;
    const int z_samples = std::round((workspace.max_corner.z - workspace.min_corner.z) / workspace_res) + 1;

    // adjust res for each dimension to be able to hit the min and max boundaries
    const double x_res = (workspace.max_corner.x - workspace.min_corner.y) / x_samples;
    const double y_res = (workspace.max_corner.y - workspace.min_corner.y) / y_samples;
    const double z_res = (workspace.max_corner.z - workspace.min_corner.z) / z_samples;

    // derive angular resolutions from configured sample counts
    const double Y_res = (MAX_YAW -  MIN_YAW)    / (Y_samples - 1);
    const double P_res = (MAX_PITCH - MIN_PITCH) / (P_samples - 1);
    const double R_res = (MAX_ROLL - MIN_ROLL)   / (R_samples - 1);

    struct GridCell : public sbpl::heap_element
    {
        std::vector<double> state;
        int x, y, z, R, P, Y;
        int g;
    };

    au::grid<6, GridCell> cells(
            x_samples, y_samples, z_samples, Y_samples, P_samples, R_samples);

    // initialize grid cells to know their own coordinates and initialize g values
    for (int x = 0; x < x_samples; ++x) { for (int y = 0; y < y_samples; ++y) { for (int z = 0; z < z_samples; ++z) {
    for (int Y = 0; Y < Y_samples; ++Y) { for (int P = 0; P < P_samples; ++P) { for (int R = 0; R < R_samples; ++R) {
        GridCell& cell = cells(x, y, z, Y, P, R);
        cell.x = x;
        cell.y = y;
        cell.z = z;
        cell.Y = Y;
        cell.P = P;
        cell.R = R;
        cell.g = std::numeric_limits<int>::max() >> 1;
    } } }
    } } }

    // goal to maximize the success of hitting ik solutions at each point
    // start with the cell of the initial robot state
    // dijkstra outwards to find the shortest path to each cell
    // distance function = hamming distance
    // on transitions, update the corresponding ik solution

    auto cell_comp = [](const GridCell& c1, const GridCell& c2) {
        return c1.g < c2.g;
    };
    sbpl::intrusive_heap<GridCell, decltype(cell_comp)> open(cell_comp);

    robot_state.setToDefaultValues();

    auto discretize_pose = [&](
            const Eigen::Affine3d& pose,
            int& dx, int& dy, int& dz, int& dY, int& dP, int& dR)
    {
        int start_x = discretize(pose.translation()[0], workspace.min_corner.x, x_res);
        int start_y = discretize(pose.translation()[1], workspace.min_corner.y, y_res);
        int start_z = discretize(pose.translation()[2], workspace.min_corner.z, z_res);
        double yaw, pitch, roll;
        sbpl::angles::get_euler_zyx(pose.rotation(), yaw, pitch, roll);
        int start_yaw = discretize(yaw, MIN_YAW, Y_res);
        int start_pitch = discretize(pitch, MIN_PITCH, P_res);
        int start_roll = discretize(roll, MIN_ROLL, R_res);
    };

    auto realize_pose = [&](
            int dx, int dy, int dz, int dY, int dP, int dR,
            Eigen::Affine3d& out)
    {
        double x, y, z, Y, P, R;
        x = realize(dx, workspace.min_corner.x, x_res);
        y = realize(dy, workspace.min_corner.y, y_res);
        z = realize(dz, workspace.min_corner.z, z_res);
        Y = realize(dY, MIN_YAW, Y_res);
        P = realize(dP, MIN_YAW, P_res);
        R = realize(dR, MIN_YAW, R_res);

        out = Eigen::Translation3d(x, y, z) *
                Eigen::AngleAxisd(Y, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(P, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(R, Eigen::Vector3d::UnitX());
    };

    robot_state.updateLinkTransforms();
    const Eigen::Affine3d& start_pose = robot_state.getGlobalLinkTransform(tip_link);

    int start_x, start_y, start_z, start_Y, start_P, start_R;
    discretize_pose(start_pose, start_x, start_y, start_z, start_Y, start_P, start_R);

    GridCell& start_cell = cells(start_x, start_y, start_z, start_Y, start_P, start_R);
    start_cell.state.resize(joint_group->getVariableCount());
    robot_state.copyJointGroupPositions(joint_group, start_cell.state);

    open.push(&start_cell);
    while (!open.empty()) {
        GridCell* g = open.min();
        open.pop();

        robot_state.setJointGroupPositions(joint_group, g->state);

        for (int dx = -1; dx <= 1; ++dx) { for (int dy = -1; dy <= 1; ++dy) { for (int dz = -1; dz <= 1; ++dz) {
        for (int dY = -1; dY <= 1; ++dY) { for (int dP = -1; dP <= 1; ++dP) { for (int dR = -1; dR <= 1; ++dR) {
            int nx = g->x + dx;
            int ny = g->y + dy;
            int nz = g->z + dz;
            int nY = g->Y + dY;
            int nP = g->P + dP;
            int nR = g->R + dR;

            if (nx < 0 || ny < 0 || nz < 0 || nY < 0 || nP < 0 || nR < 0 ||
                nx >= cells.size(0) || ny >= cells.size(1) || nz >= cells.size(2) ||
                nY >= cells.size(3) || nP >= cells.size(4) || nR >= cells.size(5))
            {
                continue;
            }

            GridCell& gn = cells(nx, ny, nz, nY, nP, nR);
            const int cost = (int)(1.0 * FP_IRES);
            const int new_cost = g->g + cost;
            if (new_cost < gn.g) {
                // TODO: update the ik solution to this pose
                robot_state.setJointGroupPositions(joint_group, g->state);
                Eigen::Affine3d spose;
                realize_pose(nx, ny, nz, nY, nP, nR, spose);
                if (robot_state.setFromIK(joint_group, spose)) {
                    continue;
                }

                // update cost
                gn.g = new_cost;
                // insert/update in heap
                if (open.contains(&gn)) {
                    open.push(&gn);
                } else {
                    open.decrease(&gn);
                }
            }
        } } } } } }
    }

    return 0;
}
