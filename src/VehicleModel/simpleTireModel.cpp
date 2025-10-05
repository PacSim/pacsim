#include "VehicleModel/simpleTireModel.hpp"

double processSlipAngleLat(double alpha, const tireParams& params)
{
    return std::sin(params.Clat * std::atan(params.Blat * alpha - params.Elat * (params.Blat * alpha - std::atan(params.Blat * alpha))));
}

double processSlipRatioLon(double kappa, const tireParams& params)
{
    return std::sin(params.Clon * std::atan(params.Blon * kappa - params.Elon * (params.Blon * kappa - std::atan(params.Blon * kappa))));
}


Eigen::Vector3d getTireForcesFromModel(double slip_ratio, double slip_angle, double fz, double friction_coefficient, const tireParams& params) {
        double fx_stat_rel = processSlipRatioLon(slip_ratio, params);

        double fy_stat_rel = processSlipAngleLat(slip_angle, params);

        // https://chatgpt.com/share/68e158f0-b148-8010-9f1a-d7c586e99ae3
        double n = std::sqrt(fx_stat_rel*fx_stat_rel + fy_stat_rel*fy_stat_rel);

        // small beta -> more sharp grip loss
        double beta = 0.2;

        double S = 1.0/std::sqrt(1+std::pow((std::max(0.0,n-1.0)/beta),2.0));

        double Dlon = params.Dlon * friction_coefficient * fz;
        
        double Dlat = params.Dlat * friction_coefficient * fz;

        double fx_stat = S * Dlon * fx_stat_rel;

        double fy_stat = S * Dlat * fy_stat_rel;

        return Eigen::Vector3d(fx_stat, fy_stat, 0.0);
}