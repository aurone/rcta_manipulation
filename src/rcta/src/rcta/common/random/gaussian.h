#ifndef Gaussian_h
#define Gaussian_h

#include <cmath>
#include <Eigen/Dense>
#include <rcta/common/utils/utils.h>

constexpr double SqrtTwoPi() { return sqrt(2.0 * M_PI); }

double gaussian(double x, double mean, double sigma)
{
    return ((1.0 / sigma) * SqrtTwoPi()) * exp(-0.5 * sqrd((1.0 / sigma) * (x - mean)));
}

class Gaussian1
{
public:

    Gaussian1(double mean, double sigma) :
        mean_(mean),
        sigma_(sigma)
    {
    }

    double operator()(double x)
    {
        return gaussian(x, mean_, sigma_);
    }

private:

    double mean_;
    double sigma_;
};

class Gaussian2
{
public:

    Gaussian2(const Eigen::Vector2d& mean, const Eigen::Matrix2d& covariance) :
        mean_(mean), covariance_(covariance) { }

    // todo: support any version of operator() that can construct a Vector2d
//    template <typename... Args>
//    const double operator(Args... args) const { return this->operator()(Eigen::Vector2d(args)); }

    const double operator()(const Eigen::Vector2d& x) const
    {
        const double k = 2.0; // dimensionality
        return (1.0 / sqrt(pow((2.0 * M_PI), k)) * sqrt(covariance_.determinant())) *
                exp(-0.5 * (((x - mean_).transpose() * covariance_.inverse() * (x - mean_))(0, 0)));

//        Eigen::Vector2d diff = x - mean_;
//        return (1.0 / ((2.0 * M_PI) * sqrt(covariance_.determinant()))) *
//                exp(-0.5 *
//                    ((diff.transpose() * covariance_.inverse() * diff)(0, 0))
//                   );
    }

private:

    Eigen::Vector2d mean_;
    Eigen::Matrix2d covariance_;
};

#endif
