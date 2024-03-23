#ifndef PACSIMWHEELSSENSOR_HPP
#define PACSIMWHEELSSENSOR_HPP

#include "types.hpp"
#include "configParser.hpp"
#include <queue>
#include "sensorBase.hpp"
#include <random>

class WheelsSensor : public SensorBase<Wheels>{
  public:
    WheelsSensor(double rate, double deadTime);

    void readConfig(ConfigElement& config);

    bool RunTick(Wheels& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time);

    Wheels applyError(Wheels input);

private:
    double error_mean;
    double error_sigma;
};

#endif /* PACSIMWHEELSSENSOR_HPP */