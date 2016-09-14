#include "xytheta_collision_checker.h"

#include <Eigen/Dense>
#include <spellbook/geometry_msgs/geometry_msgs.h>
#include <spellbook/std_msgs/std_msgs.h>

XYThetaCollisionChecker::XYThetaCollisionChecker(
    const std::vector<sbpl_2Dpt_t>& footprint_polygon,
    int obs_thresh,
    int num_heading_disc)
:
    m_grid(),
    m_footprint_polygon(footprint_polygon),
    m_num_heading_disc(num_heading_disc),
    m_obs_thresh(obs_thresh),
    m_footprint_res(0.0),
    m_precomputed_footprint_cells()
{
}

XYThetaCollisionChecker::~XYThetaCollisionChecker()
{
}

bool XYThetaCollisionChecker::updateOccupancyGrid(
    const nav_msgs::OccupancyGrid& new_occ_grid)
{
    if (new_occ_grid.info.width <= 0 || new_occ_grid.info.height <= 0) {
        ROS_ERROR("Grid dimensions invalid");
        return false;
    }
    if (new_occ_grid.info.resolution <= 0.0) {
        ROS_ERROR("Grid resolution is invalid");
        return false;
    }
    const size_t total_size = new_occ_grid.info.width * new_occ_grid.info.height;
    if (new_occ_grid.data.size() < total_size) {
        ROS_ERROR("Grid data invalid");
        return false;
    }

    if (!reinitFootprintCells(new_occ_grid.info.resolution)) {
        ROS_ERROR("Failed to precompute footprint cells");
        return false;
    }

    if (!updateDistanceMap(new_occ_grid)) {
        return false;
    }

    m_grid = new_occ_grid;

    return true;
}

double XYThetaCollisionChecker::getCollisionProbability(
    double x,
    double y,
    double theta)
{
    if (!initialized()) {
        ROS_ERROR("No valid map data received");
        return 0.0;
    }
    return getCollisionProbability(
            CONTXY2DISC(x, m_grid.info.resolution),
            CONTXY2DISC(y, m_grid.info.resolution),
            ContTheta2Disc(theta, m_num_heading_disc));
}

double XYThetaCollisionChecker::getCollisionProbability(int x, int y, int theta)
{
    if (!initialized()) {
        ROS_ERROR("No valid map data received");
        return 0.0;
    }

    sbpl_2Dcell_t cell;
    EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
    int i;
    double max_cell_prob_ = getCellColllisionProbability(x, y);

    // check collisions that for the particular footprint orientation along the action
    if (m_footprint_polygon.size() > 1) {
        for (i = 0; i < (int)m_precomputed_footprint_cells[theta].size(); i++) {
            //get the cell in the map
            cell = m_precomputed_footprint_cells[theta].at(i);
            cell.x = cell.x + x;
            cell.y = cell.y + y;

            //check validity
            max_cell_prob_ = std::max(max_cell_prob_, getCellColllisionProbability(cell.x, cell.y));
        }
    }
    return max_cell_prob_;
}

visualization_msgs::MarkerArray
XYThetaCollisionChecker::getFootprintVisualization(
    double x, double y, double theta) const
{
    if (!initialized()) {
        ROS_WARN("No valid map data received");
        return visualization_msgs::MarkerArray();
    }

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = m_grid.header.frame_id;
    m.ns = "footprint_polygon";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.01;
    m.color = std_msgs::MagentaColorRGBA(1.0f);
    Eigen::Affine3d T_world_footprint =
            Eigen::Translation3d(x, y, 0.0) *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
    for (const auto& vertex : m_footprint_polygon) {
        Eigen::Vector3d v(vertex.x, vertex.y, 0.0);
        v = T_world_footprint * v;
        geometry_msgs::Point p(geometry_msgs::CreatePoint(v.x(), v.y(), v.z()));
        m.points.push_back(p);
    }
    if (!m.points.empty()) {
        m.points.push_back(m.points.front());
    }
    ma.markers.push_back(std::move(m));
    return ma;
}

visualization_msgs::MarkerArray
XYThetaCollisionChecker::getDiscreteFootprintVisualization(
    double x, double y, double theta) const
{
    if (!initialized()) {
        ROS_WARN("No valid map data received");
        return visualization_msgs::MarkerArray();
    }

    visualization_msgs::MarkerArray ma;
    return ma;
}

bool XYThetaCollisionChecker::isValidState(double x, double y, double theta)
{
    if (!initialized()) {
        ROS_ERROR("No valid map data received!");
        return false;
    }
    const double res = m_grid.info.resolution;
    int cx = (int)((x - m_grid.info.origin.position.x) / res);
    int cy = (int)((y - m_grid.info.origin.position.y) / res);
    int cth = ContTheta2Disc(theta, m_num_heading_disc);
    return isValidState(cx, cy, cth);
}

double XYThetaCollisionChecker::getCellObstacleDistance(int x, int y)
{
    if (!initialized()) {
        ROS_ERROR("No valid map data received");
        return 0.0;
    }

    if (!isValidCell(x, y)) {
        return 0.0;
    }
    return std::sqrt(m_distance_map(x, y) * m_grid.info.resolution);
}

double XYThetaCollisionChecker::getCellObstacleDistance(double x, double y)
{
    const double res = m_grid.info.resolution;
    int cx = (int)((x - m_grid.info.origin.position.x) / res);
    int cy = (int)((y - m_grid.info.origin.position.y) / res);
    return getCellObstacleDistance(cx, cy);
}

bool XYThetaCollisionChecker::isValidCell(int x, int y)
{
    if (x < 0 || x >= m_grid.info.width) {
        return false;
    }
    if (y < 0 || y >= m_grid.info.height) {
        return false;
    }

    assert(m_grid.data.size() < y * m_grid.info.width + x);

    if (m_grid.data[y * m_grid.info.width + x] >= m_obs_thresh) {
        return false;
    }

    return true;
}

double XYThetaCollisionChecker::getCellColllisionProbability(int x, int y)
{
    return std::max(0.0, (double)(m_grid.data[y * m_grid.info.width + x]) / 100.0);
}

bool XYThetaCollisionChecker::isValidState(int x, int y, int theta)
{
    if (!initialized()) {
        ROS_ERROR("No valid map data received");
        return false;
    }

    if (!isValidCell(x, y)) {
        return false;
    }

    // check collisions that for the particular footprint orientation along the
    // action
    if (m_footprint_polygon.size() > 1) {
        for (size_t i = 0; i < m_precomputed_footprint_cells[theta].size(); i++) {
            //get the cell in the map
            sbpl_2Dcell_t cell = m_precomputed_footprint_cells[theta][i];
            cell.x = cell.x + x;
            cell.y = cell.y + y;

            // check validity
            if (!isValidCell(cell.x, cell.y)) {
                return false;
            }
        }
    }
    return true;
}

bool XYThetaCollisionChecker::updateDistanceMap(
    const nav_msgs::OccupancyGrid& grid)
{
    m_distance_map.resize(grid.info.width, grid.info.height);
    au::grid<2, int> G(grid.info.width, grid.info.height);

    const int inf = grid.info.width + grid.info.height;

    auto sqrd = [](int i) { return i * i; };
    auto occupied = [&](int x, int y) -> std::int8_t {
        return grid.data[y * grid.info.width + x] >= m_obs_thresh;
    };

    // scan the first row
    for (size_t x = 0; x < grid.info.width; ++x) {
        if (occupied(x, 0)) {
            G(x, 0) = 0;
        } else {
            G(x, 0) = inf;
        }

        for (size_t y = 1; y < grid.info.height; ++y) {
            if (occupied(x, y)) {
                G(x, y) = 0;
            } else {
                G(x, y) = G(x, y - 1) + 1;
            }
        }

        for (int y = (int)grid.info.height - 2; y >= 0; --y) {
            if (G(x, y + 1) < G(x, y)) {
                G(x, y) = 1 + G(x, y + 1);
            }
        }
    }

    for (size_t y = 0; y < grid.info.height; ++y) {
        auto g = [&](int i) { return G(i, y); };
        auto f = [&](int x, int i) {
            return sqrd(x - i) + sqrd(g(i));
        };

        au::grid<1, int> s(grid.info.width), t(grid.info.width);
        int q = 0;
        s(0) = 0;
        t(0) = 0;
        for (size_t u = 1; u < grid.info.width; ++u) {
            while (q >= 0 && f(t(q), s(q)) > f(t(q), u)) {
                --q;
            }
            if (q < 0) {
                q = 0;
                s(0) = u;
            } else {
                auto sep = [&](int i, int u) {
                    return (u*u - i*i + g(u)*g(u) - g(i)*g(i)) / (2 * (u - i));
                };
                int w = 1 + sep(s(q), u);
                if (w < grid.info.width) {
                    ++q;
                    s(q) = u;
                    t(q) = w;
                }
            }
        }
        for (int u = (int)grid.info.width - 1; u >= 0; --u) {
            m_distance_map(u, y) = f(u, s(q));
            if (u == t(q)) {
                --q;
            }
        }
    }

    return true;
}

bool XYThetaCollisionChecker::reinitFootprintCells(double new_res)
{
    assert(new_res > 0.0);

    if (new_res == m_footprint_res) {
        ROS_DEBUG("Footprint cells already precomputed for resolution %0.3f", new_res);
        return true;
    }

    if (!precomputeFootprints(new_res)) {
        return false;
    }

    return true;
}

bool XYThetaCollisionChecker::precomputeFootprints(double res)
{
    EnvNAVXYTHETALAT3Dpt_t pose;
    pose.x = 0;
    pose.y = 0;
    pose.theta = 0;

    std::vector<std::vector<sbpl_2Dcell_t>> footprint_cells;
    footprint_cells.resize(m_num_heading_disc);
    for (int h = 0; h < m_num_heading_disc; h++) {
        pose.theta = DiscTheta2Cont(h, m_num_heading_disc);
        ROS_DEBUG("Pre-computing footprint for heading %d/%d", h, m_num_heading_disc);
        if (!calculateFootprintForPose(pose, footprint_cells[h], m_footprint_polygon, res)) {
            ROS_WARN("Failed to pre-compute footprint polygon for heading %d", h);
            return false;
        }
    }

    m_precomputed_footprint_cells = std::move(footprint_cells);
    m_footprint_res = res;
    return true;
}

bool XYThetaCollisionChecker::calculateFootprintForPose(
    EnvNAVXYTHETALAT3Dpt_t pose,
    std::vector<sbpl_2Dcell_t> &footprint_cells,
    const std::vector<sbpl_2Dpt_t>& FootprintPolygon,
    double res)
{
    // handle special case where footprint is just a point
    if (FootprintPolygon.size() <= 1) {
        ROS_WARN("Robot footprint not defined! Using point-robot!");
        sbpl_2Dcell_t cell;
        cell.x = CONTXY2DISC(pose.x, res);
        cell.y = CONTXY2DISC(pose.y, res);

        int pind;
        for (pind = 0; pind < (int)footprint_cells.size(); pind++) {
            if (cell.x == footprint_cells.at(pind).x && cell.y == footprint_cells.at(pind).y) {
                break;
            }
        }
        if (pind == (int)footprint_cells.size()) {
            footprint_cells.push_back(cell);
        }
        return true;
    }

    // bounding polygon in the pose frame
    std::vector<sbpl_2Dpt_t> bounding_polygon;
    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    for (size_t vidx = 0; vidx < FootprintPolygon.size(); vidx++) {
        const sbpl_2Dpt_t& pt = FootprintPolygon[vidx];

        // transform into the world frame
        Eigen::Affine2d T_world_footprint =
                Eigen::Translation2d(pose.x, pose.y) *
                Eigen::Rotation2Dd(pose.theta);

        Eigen::Vector2d v_footprint(pt.x, pt.y);
        Eigen::Vector2d v_world = T_world_footprint * v_footprint;

        sbpl_2Dpt_t corner(v_world.x(), v_world.y());
        bounding_polygon.push_back(corner);

        if (corner.x < min_x) {
            min_x = corner.x;
        }
        if (corner.x > max_x) {
            max_x = corner.x;
        }
        if (corner.y < min_y) {
            min_y = corner.y;
        }
        if (corner.y > max_y) {
            max_y = corner.y;
        }
    }

    int min_disc_x = CONTXY2DISC(min_x, res);
    int min_disc_y = CONTXY2DISC(min_y, res);
    int max_disc_x = CONTXY2DISC(max_x, res);
    int max_disc_y = CONTXY2DISC(max_y, res);

    int prev_inside = 0;
    int discrete_x;
    int discrete_y;

    for (int x = min_disc_x; x <= max_disc_x; ++x) {
        for (int y = min_disc_y; y <= max_disc_y; ++y) {
            sbpl_2Dpt_t pt;
            pt.x = DISCXY2CONT(x, res);
            pt.y = DISCXY2CONT(y, res);

            // the first iteration checks the four corners of the cell for
            // inclusion in the footprint; successive iterations check
            // subsampled points for inclusion in the footprint. the sampling
            // returns when either a point is found to lie within the footprint
            // or all subsample levels have been checked and no intersection
            // has been found

            // maximum level of subsamples to check
            int ss_level = 0;
            bool intersects = false;
            for (int l = 0; l <= ss_level && !intersects; ++l) {
                int sample_count = ipow(2, ss_level) + 1;
                for (int si = 0; si < sample_count && !intersects; ++si) {
                    for (int sj = 0; sj < sample_count && !intersects; ++sj) {
                        // for successive levels, both even indices even -> already checked
                        if (l > 0 && si % 2 == 0 && sj % 2 == 0) {
                            continue;
                        }
                        sbpl_2Dpt_t ss;
                        ss.x = pt.x - 0.5 * res + si * res / (sample_count - 1);
                        ss.y = pt.y - 0.5 * res + sj * res / (sample_count - 1);
                        if (IsInsideFootprint(ss, &bounding_polygon)) {
                            footprint_cells.emplace_back(x, y);
                            intersects = true;
                        }
                    }
                }
            }
        }
    }

    ROS_DEBUG("Footprint size: %zd cells", footprint_cells.size());
    return true;
}

int XYThetaCollisionChecker::ipow(int b, int e)
{
    if (e == 0) {
        return 1;
    } else if (e % 2 == 0) {
        int r = ipow(b, e >> 1);
        return r * r;
    } else {
        return b * ipow(b, e - 1);
    }
}

bool XYThetaCollisionChecker::initialized() const
{
    return m_footprint_res > 0.0;
}
