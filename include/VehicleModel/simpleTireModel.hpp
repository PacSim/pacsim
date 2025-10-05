#ifndef SIMPLE_TIRE_MODEL_HPP
#define SIMPLE_TIRE_MODEL_HPP

#include <Eigen/Core>

struct tireParams {
    double Blat = 10.0;
    double Clat = -1.9;
    double Dlat = 1.6;
    double Elat = 0.98;
    // TODO get this automatically
    double alpha_peak = 0.1832;

    double Blon = 11.5;
    double Clon = 2.196;
    double Dlon = 1.6;
    double Elon = 1.0;
    // TODO get this automatically
    double kappa_peak = 0.1028;
};

double processSlipAngleLat(double alpha, const tireParams& params);

double processSlipRatioLon(double kappa, const tireParams& params);

Eigen::Vector3d getTireForcesFromModel(double slip_ratio, double slip_angle, double fz, double friction_coefficient, const tireParams& params);

#endif /* SIMPLE_TIRE_MODEL_HPP */