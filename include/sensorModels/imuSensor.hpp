#ifndef PACSIMIMUSENSOR_HPP
#define PACSIMIMUSENSOR_HPP

#include "configParser.hpp"
#include "sensorBase.hpp"
#include "transform.hpp"
#include "types.hpp"
#include <queue>
#include <random>

class ImuSensor : public SensorBase<ImuData>
{
public:
    ImuSensor(double rate, double deadTime);

    void readConfig(ConfigElement& config);

    ImuData process(ImuData& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time);

    bool RunTick(ImuData& in, Eigen::Vector3d& alpha, double time);

    ImuData applyError(ImuData input);

    std::string getName();

private:
    double error_mean_acc;
    double error_sigma_acc;
    double error_mean_rot;
    double error_sigma_rot;
    std::string name;
    std::string frame;
};

#endif /* PACSIMIMUSENSOR_HPP */