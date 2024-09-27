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
    GnssSensor();

    void readConfig(ConfigElement& config);

    std::string getName();

    std::string getFrameId();

    bool RunTick(Eigen::Vector3d& gnssOrigin, Eigen::Vector3d& enuToTrackRotation, Eigen::Vector3d& trans,
        Eigen::Vector3d& rot, double time, Eigen::Vector3d velocity, Eigen::Vector3d omega,
        Eigen::Vector3d start_position, Eigen::Vector3d start_orientation, bool trackPreTransformed);

private:
    std::string name;
    std::string frame_id;

    int noiseSeed;

    Eigen::Vector3d errorSigmaPosition;
    Eigen::Vector3d errorMeanPosition;

    Eigen::Vector3d errorSigmaVelocity;
    Eigen::Vector3d errorMeanVelocity;

    Eigen::Vector3d errorSigmaOrientation;
    Eigen::Vector3d errorMeanOrientation;

    bool outputVelocity;
    int orientationMode;
};

#endif /* PACSIMGNSSSENSOR_HPP */