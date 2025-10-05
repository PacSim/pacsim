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
        
        double tan_alpha = std::tan(slip_angle);
        double kappa_norm = slip_ratio / params.kappa_peak;
        double tan_alpha_norm = tan_alpha / std::tan(params.alpha_peak);
        // get "normalized" slip, scale based on that
        double sigma_norm = std::sqrt(kappa_norm*kappa_norm + tan_alpha_norm*tan_alpha_norm);

        double epsilon = 1e-6;

        double fx_stat = 0.0;
        double fy_stat = 0.0;

        if(sigma_norm > epsilon) {
            // use longitudinal model as scaling reference
            double fx_norm = processSlipRatioLon(sigma_norm * params.kappa_peak, params);
            double fy_norm = processSlipAngleLat(sigma_norm * params.alpha_peak, params);
            
            double Dlon = params.Dlon * friction_coefficient * fz;
            
            double Dlat = params.Dlat * friction_coefficient * fz;
            
            double fx_combined = Dlon * (kappa_norm / sigma_norm) * fx_norm;
            
            double fy_combined = Dlat * (tan_alpha_norm / sigma_norm) * fy_norm;
            
            fx_stat = fx_combined;
            fy_stat = fy_combined;
        }

        return Eigen::Vector3d(fx_stat, fy_stat, 0.0);
}