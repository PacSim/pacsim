#ifndef SCALARVALUESENSOR_HPP
#define SCALARVALUESENSOR_HPP

#include "types.hpp"
#include "configParser.hpp"
#include <queue>
#include "sensorBase.hpp"
#include <random>

class ScalarValueSensor : public SensorBase<StampedScalar>{
  public:
    ScalarValueSensor(double rate, double deadTime);

    void readConfig(ConfigElement& config);

    bool RunTick(StampedScalar& in, double time);

    StampedScalar applyError(StampedScalar input);

private:
    double error_mean;
    double error_sigma;
};

#endif /* SCALARVALUESENSOR_HPP */