#ifndef PACSIMPERCEPTIONSENSOR_HPP
#define PACSIMPERCEPTIONSENSOR_HPP

#include "configParser.hpp"
#include "sensorBase.hpp"
#include "transform.hpp"
#include "types.hpp"
#include <queue>
#include <random>

class PerceptionSensor : public SensorBase<LandmarkList>
{
public:
    PerceptionSensor(std::string name, double minRange, double maxRange, double minAngle, double maxAngle, double rate,
        double deadTime, std::string frame_id);

    PerceptionSensor();

    void readConfig(ConfigElement& config);

    void setSensorPose(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation);

    void setSensorNoiseXYZ(const Eigen::Vector3d& mean, const Eigen::Vector3d& sigma);

    void setSensorNoiseRange(double mean, double sigma);

    void setSensorNoiseRangeRelative(double mean, double sigma);

    void setSensorNoiseAngle(const Eigen::Vector2d& mean, const Eigen::Vector2d& sigma);

    LandmarkList process(LandmarkList& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time);

    std::vector<Landmark> filterFoV(std::vector<Landmark>& in);

    std::vector<Landmark> addNoise(std::vector<Landmark>& in);

    std::string getName();

    std::string getFrameId();

    bool RunTick(LandmarkList& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time);

private:
    std::string name;
    std::string frame_id;
    double minRange;
    double maxRange;
    double minAngleHorizontal;
    double maxAngleHorizontal;
    double minAngleVertical;
    double maxAngleVertical;
    Eigen::Vector3d errorSigmaXYZ;
    Eigen::Vector3d errorMeanXYZ;

    double errorSigmaRange;
    double errorMeanRange;
    double errorSigmaRangeRelative;
    double errorMeanRangeRelative;
    Eigen::Vector2d errorSigmaAngle;
    Eigen::Vector2d errorMeanAngle;
};

#endif /* PACSIMPERCEPTIONSENSOR_HPP */