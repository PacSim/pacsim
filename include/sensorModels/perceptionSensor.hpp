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
    PerceptionSensor();

    void readConfig(ConfigElement& config);

    LandmarkList process(LandmarkList& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time);

    std::vector<Landmark> filterFoV(std::vector<Landmark>& in);

    std::vector<Landmark> filterTypeAndDOO(std::vector<Landmark>& in);

    std::vector<Landmark> addClassProbailities(std::vector<Landmark>& in);

    std::vector<Landmark> handleFalsePositivesAndNegatives(std::vector<Landmark>& in);

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

    double detection_prob_min_dist;
    double detection_prob_decrease_dist_linear;
    double detection_prob_decrease_dist_quadratic;
    double min_detection_prob;

    double classification_max_distance;
    double classification_prob_min_dist;
    double classification_prob_decrease_dist_linear;
    double classification_prob_decrease_dist_quadratic;
    double min_classification_prob;
};

#endif /* PACSIMPERCEPTIONSENSOR_HPP */