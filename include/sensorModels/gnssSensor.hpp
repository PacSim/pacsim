#ifndef PACSIMGNSSSENSOR_HPP
#define PACSIMGNSSSENSOR_HPP

#include "configParser.hpp"
#include "sensorBase.hpp"
#include "transform.hpp"
#include "types.hpp"
#include <queue>
#include <random>

class GnssSensor : public SensorBase<GnssData>
{
public:
    GnssSensor(double rate, double deadTime);

    void readConfig(ConfigElement& config);

    bool RunTick(Eigen::Vector3d& gnssOrigin, Eigen::Vector3d& enuToTrackRotation, Eigen::Vector3d& trans,
        Eigen::Vector3d& rot, double time, Eigen::Vector3d velocity);

private:
    double error_mean;
    double error_sigma;
};

#endif /* PACSIMGNSSSENSOR_HPP */