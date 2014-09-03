#ifndef NURB_h
#define NURB_h

#include <cassert>
#include <cstdio>
#include <vector>

template <typename PointType> // Where PoinType is meant to be an Eigen::Vector
struct Nurb
{
    std::vector<PointType> control_points_;
    int degree_;
    std::vector<int> knots_;

    Nurb();
    Nurb(const std::vector<PointType>& control_points, int degree);

    bool initialize(const std::vector<PointType>& control_points, int degree);

    const std::vector<PointType>& control_points() const { return control_points_; }
    int degree() const { return degree_; }
    const std::vector<int> knots() const { return knots_; }

    PointType operator()(double u);
    PointType deriv(double u);

public:

    double dbasis(int i, int n, double u);
    double dbasis2(int i, int n, double u);

    double basis_func(int i, int n, double u);

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
//        if (i < degree) {
//            return 0.0;
//        }
//        else if (i >= control_points_.size() - degree) {
//            return 1.0;
//        }
//        return (double)i / (double)(knots_.size() - 1 - 2 * degree);
        return (double)i / (double)(knots_.size() - 1);
    }
};

template <typename PointType>
Nurb<PointType>::Nurb() : control_points_(), knots_(), degree_(0) { }

template <typename PointType>
Nurb<PointType>::Nurb(const std::vector<PointType>& control_points, int degree) :
    control_points_(),
    knots_(),
    degree_()
{
    initialize(control_points, degree);
}

template <typename PointType>
bool Nurb<PointType>::initialize(const std::vector<PointType>& control_points, int degree)
{
    if (degree < 1 || control_points.size() < degree) {
        return false;
    }

    control_points_ = control_points;
    degree_ = degree;

    const int order = degree + 1;
    int num_knots = (int)control_points.size() + order;

    // append knot assigned to the first cv with order multiplicity
    for (int i = 0; i < order; ++i) {
        knots_.push_back(0);
    }

    int num_remaining = num_knots - 2 * order;

    for (int i = 0; i < num_remaining; ++i) {
        knots_.push_back(i + 1); // assign knots linearly to the cvs, starting with the second cv
    }

    // append knot assigned to the last cv with order multiplicity
    for (int i = 0; i < order; ++i) {
        knots_.push_back((int)control_points.size() - 1);
    }

    assert(knots_.size() == control_points_.size() + order);
    return true;
}

template <typename PointType>
PointType Nurb<PointType>::operator()(double u)
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
        if (u == 0) {
            return control_points_.front();
        }
        else if (u == 1) {
            return control_points_.back();
        }
        else {
            fprintf(stderr, "Normalizer is 0\n");
            return agg;
        }
    }

    return (agg / normalizer);
}

template <typename PointType>
PointType Nurb<PointType>::deriv(double u)
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

template <typename PointType>
double Nurb<PointType>::dbasis(int i, int n, double u)
{
    return (n / (knot(i + n) - knot(i)))         * basis_func(i, n - 1, u) -
           (n / (knot(i + n + 1) - knot(i + 1))) * basis_func(i + 1, n - 1, u);
}

template <typename PointType>
double Nurb<PointType>::dbasis2(int i, int n, double u)
{
    if (n == 0) {
        return 0;
    }
    else {
        double fin = f(i, n, u);
        double dfin = df(i, n, u);
        double gin = g(i + 1, n, u);
        double dgin = dg(i + 1, n, u);
        double nin1 = basis_func(i, n - 1, u);
        double dnin1 = dbasis(i, n - 1, u);
        double nin2 = basis_func(i + 1, n - 1, u);
        double dnin2 = dbasis(i + 1, n - 1, u);
        double res = nin1 * dfin + dnin1 * fin + nin2 * dgin + dnin2 * gin;
        return res;
    }
}

template <typename PointType>
double Nurb<PointType>::basis_func(int i, int n, double u)
{
    if (u < 0 || u > 1) {
        fprintf(stderr, "Invalid parameter given to basis function\n");
        return std::numeric_limits<double>::quiet_NaN();
    }

    if (n == 0) {
        if (u >= knot(i) && u < knot(i + 1)) {
//            printf("N(%d, %d) = 1.0\n", i, n);
//            printf("k_i <= u < k_i+1 => 1 (%0.3f <= %0.3f < %0.3f)\n", knot(i), u, knot(i + 1));
            return 1.0;
        }
        else {
//            printf("N(%d, %d) = 0.0\n", i, n);
            return 0.0;
        }
    }
    else {
        double fin = f(i, n, u);
        double gin = g(i + 1, n, u);
        double nin1 = basis_func(i, n - 1, u);
        double nin2 = basis_func(i + 1, n - 1, u);
        double res = fin * nin1 + gin * nin2;
//        printf("N(%d, %d) = f * nin1 + g * nin2 = %0.3f * %0.3f + %0.3f * %0.3f = %0.3f\n", i, n, fin, nin1, gin, nin2, res);
        return res;
    }
}

#endif
