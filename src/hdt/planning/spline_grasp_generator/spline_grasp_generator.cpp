#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <Eigen/Dense>
#include <GL/freeglut.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/matplotpp/matplotpp.h>

using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// NURB API
////////////////////////////////////////////////////////////////////////////////

template <typename PointType> // Where PoinType is meant to be an Eigen::Vector
struct Nurb
{
    std::vector<PointType> control_points_;
    int degree_;
    std::vector<int> knots_;

    Nurb(const std::vector<PointType>& control_points, int degree) :
        control_points_(control_points), degree_(degree)
    {
        knots_.push_back(0);
        for (int i = 0; i < control_points_.size(); ++i) {
            knots_.push_back(i);
        }
        for (int i = 0; i < degree_; ++i) {
            knots_.push_back((int)control_points_.size() - 1);
        }

        printf("Constructed NURBS with knots %s\n", to_string(knots_).c_str());

        assert(knots_.size() == control_points_.size() + degree_ + 1);
    }

    const std::vector<PointType>& control_points() const { return control_points_; }
    int degree() const { return degree_; }
    const std::vector<int> knots() const { return knots_; }

    PointType operator()(double u)
    {
        std::vector<double> weights(control_points_.size(), 1.0); // TODO: all weights are 1 currently for simplicity

        PointType agg = PointType::Zero();

        double normalizer = 0.0;
        for (int n = 0; n < control_points_.size(); ++n) {
            double tmp = basis_func(n, degree_, u) * weights[n];
            agg += tmp * control_points_[n];
            normalizer += tmp;
        }

        if (normalizer == 0) {
//            fprintf(stderr, "Normalizer is 0\n");
            return agg;
        }

        return (agg / normalizer);
    }

    PointType deriv(double u)
    {
        std::vector<double> weights(control_points_.size(), 1.0);

        PointType agg = PointType::Zero();

        double normalizer = 0.0;
        for (int n = 0; n < control_points_.size(); ++n) {
            double tmp = dbasis(n, degree_, u);
            agg += tmp * control_points_[n];
            normalizer += tmp;
        }

//        if (normalizer == 0) {
//            fprintf(stderr, "Normalizer is 0\n");
            return agg;
//        }

//        return (agg / normalizer);
    }

public:

    double dbasis(int i, int n, double u)
    {
        return (n / (knot(i + n) - knot(i)))         * basis_func(i, n - 1, u) -
               (n / (knot(i + n + 1) - knot(i + 1))) * basis_func(i + 1, n - 1, u);
//        if (n == 0) {
//            return 0;
//        }
//        else {
//            double fin = f(i, n, u);
//            double dfin = df(i, n, u);
//            double gin = g(i + 1, n, u);
//            double dgin = dg(i + 1, n, u);
//            double nin1 = basis_func(i, n - 1, u);
//            double dnin1 = dbasis(i, n - 1, u);
//            double nin2 = basis_func(i + 1, n - 1, u);
//            double dnin2 = dbasis(i + 1, n - 1, u);
//            double res = nin1 * dfin + dnin1 * fin + nin2 * dgin + dnin2 * gin;
//            return res;
//        }
    }

    double basis_func(int i, int n, double u)
    {
        if (u < 0 || u > 1) {
            fprintf(stderr, "Invalid parameter given to basis function\n");
            return std::numeric_limits<double>::quiet_NaN();
        }

        if (n == 0) {
            if (u >= knot(i) && u < knot(i + 1)) {
                return 1.0;
            }
            else {
                return 0.0;
            }
        }
        else {
            double fin = f(i, n, u);
            double gin = g(i + 1, n, u);
            double nin1 = basis_func(i, n - 1, u);
            double nin2 = basis_func(i + 1, n - 1, u);
            double res = fin * nin1 + gin * nin2;
//            printf("f_i,n * N_i,n-1 + g_i+1,n * N_i+1,n-1 = %0.3f * %0.3f + %0.3f * %0.3f = %0.3f\n", fin, gin, nin1, nin2, res);
            return res;
        }
    }

    double f(int i, int n, double u) const
    {
        return (u - knot(i)) / (knot(i + n) - knot(i));
    }

    double g(int i, int n, double u) const
    {
        return (knot(i + n) - u) / (knot(i + n) - knot(i));
    }

    double df(int i, int n, double u) const
    {
        return 1.0 / (knot(i + n) - knot(i));
    }

    double dg(int i, int n, double u) const
    {
        return -1.0 / (knot(i + n) - knot(i));
    }

    double knot(int i) const
    {
        return (double)i / (double)(knots_.size() - 1);
    }
};

////////////////////////////////////////////////////////////////////////////////
// VISUALIZER API
////////////////////////////////////////////////////////////////////////////////

class SplineVisualizer
{
public:

    virtual ~SplineVisualizer() { } // eclipse needs to quit bitching

    static SplineVisualizer& get()
    {
        static SplineVisualizer viz;
        return viz;
    }

    void display()
    {
        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        printf("Displaying...\n");
        const int num_samples = 100;
        const double min_u = 0.0;
        const double max_u = 1.0;

        std::vector<double> xs(num_samples), ys(num_samples), zs(num_samples);
        std::vector<double> dxs(num_samples), dys(num_samples), dzs(num_samples);
        for (int i = 0; i < num_samples; ++i) {
            double u = (max_u - min_u) / (num_samples - 1) * i;

            Vector3d p = nurb_(u);
            xs[i] = p.x();
            ys[i] = p.y();
            zs[i] = p.z();

            p = nurb_.deriv(u);
            dxs[i] = p.x();
            dys[i] = p.y();
            dzs[i] = p.z();
        }

        plot(xs, ys, zs);
        plot_arrows(xs, ys, zs, dxs, dys, dzs);

        const Vector3d bias(0, 1, 0);
        for (std::size_t i = 0; i < xs.size(); ++i) {
            Vector3d x(xs[i], ys[i], zs[i]);
            Vector3d d(dxs[i], dys[i], dzs[i]);

            d.normalize();
            Vector3d grasp_dir = bias - bias.dot(d) * d;
            grasp_dir.normalize();
            grasp_dir *= 0.3;

            Vector3d grasp_pos = x + grasp_dir;
            grasp_dir = -grasp_dir;

            xs[i] = grasp_pos.x();
            ys[i] = grasp_pos.y();
            zs[i] = grasp_pos.z();

            dxs[i] = grasp_dir.x();
            dys[i] = grasp_dir.y();
            dzs[i] = grasp_dir.z();

            printf("Grasp dir: %s\n", to_string(grasp_dir).c_str());
        }

        plot_arrows(xs, ys, zs, dxs, dys, dzs, "blue");

        glutSwapBuffers();
    }

    void reshape(int w, int h)
    {
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
//        gluOrtho2D(-1.05, 1.05, -1.05, 1.05); // for basis functions
        gluOrtho2D(-1.0, 7.0, -3.0, 3.0); // for the points
//        gluOrtho2D(-10.0, 16.0, -12.0, 12.0); // for the points
        glMatrixMode(GL_MODELVIEW);
    }

private:

    void plot(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z)
    {
        glColor3d(0.0, 0.0, 0.0);
        glBegin(GL_POINTS);
        for (std::size_t i = 0; i < x.size(); ++i) {
            glVertex3d(x[i], y[i], z[i]);
        }
        glEnd();
    }

    void plot_arrows(
        const std::vector<double>& x,
        const std::vector<double>& y,
        const std::vector<double>& z,
        const std::vector<double>& dx,
        const std::vector<double>& dy,
        const std::vector<double>& dz,
        const std::string& color = "red")
    {
        if (color == "red") {
            glColor3d(1.0, 0.0, 0.0);
        }
        else if (color == "blue") {
            glColor3f(0.0, 0.0, 1.0);
        }
        glBegin(GL_LINES);
        for (std::size_t i = 0; i < x.size(); ++i) {
            Vector3d d(dx[i], dy[i], dz[i]);
            d.normalize();
            glVertex3d(x[i], y[i], z[i]);
            glVertex3d(x[i] + 0.3 * d.x(), y[i] + 0.3 * d.y(), z[i] + 0.3 * d.z());
        }
        glEnd();
    }

    SplineVisualizer() :
        nurb_({
            Vector3d(0.0, 0.0, 0),
            Vector3d(1, 2, 0),
            Vector3d(2, 1.75, 0),
            Vector3d(3, 1.5, 0),
            Vector3d(5, -1, 0),
            Vector3d(6, 1, 0) },
            2)
    {
        nurb_.knots_ = { 0, 0, 1, 2, 3, 4, 5, 5, 5 };

        printf("Control Points:\n");
        for (const Vector3d& v : nurb_.control_points()) {
            printf("    %s\n", to_string(v).c_str());
        }

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
    }

    Nurb<Vector3d> nurb_;
};

////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////

void display()
{
    SplineVisualizer::get().display();
}

void reshape(int w, int h)
{
    SplineVisualizer::get().reshape(w, h);
}

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutCreateWindow("Spline Grasp Generator");
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(500, 500);
    glutInitWindowPosition(0, 0);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMainLoop();

    return 0;
}
