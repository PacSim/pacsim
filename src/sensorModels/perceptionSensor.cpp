#include "sensorModels/perceptionSensor.hpp"

PerceptionSensor::PerceptionSensor()
{
    this->name = "";
    this->frame_id = "";
    this->maxRange = 0.0;
    this->minRange = 0.0;
    this->minAngleHorizontal = 0.0;
    this->maxAngleHorizontal = 0.0;
    this->rate = rate;
    this->lastSampleTime = 0.0;
    this->deadTime = deadTime;
    this->position = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->orientation = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->errorMeanXYZ = Eigen::Vector3d::Zero();
    this->errorSigmaXYZ = Eigen::Vector3d::Zero();
    this->errorMeanRange = 0.0;
    this->errorSigmaRange = 0.0;
    this->errorMeanRangeRelative = 0.0;
    this->errorSigmaRangeRelative = 0.0;
    this->errorMeanAngle = Eigen::Vector2d::Zero();
    this->errorSigmaAngle = Eigen::Vector2d::Zero();
    this->numFrames = 0;
    this->detection_prob_min_dist = 0.99;
    this->detection_prob_decrease_dist_linear = 0.01;
    this->detection_prob_decrease_dist_quadratic = 0.00012;
    this->min_detection_prob = 0.1;
}

void PerceptionSensor::readConfig(ConfigElement& config)
{
    config.getElement<std::string>(&this->name, "name");
    config.getElement<std::string>(&this->frame_id, "name");
    config.getElement<double>(&this->rate, "rate");
    config["delay"].getElement<double>(&this->deadTime, "mean");
    std::vector<double> position;
    config["pose"].getElement<std::vector<double>>(&position, "position");
    this->position = Eigen::Vector3d(position[0], position[1], position[2]);
    std::vector<double> orientation;
    config["pose"].getElement<std::vector<double>>(&orientation, "orientation");
    this->orientation = Eigen::Vector3d(orientation[0], orientation[1], orientation[2]);
    config["observation"].getElement<double>(&this->minRange, "min_range");
    config["observation"].getElement<double>(&this->maxRange, "max_range");

    config["observation"].getElement<double>(&this->minAngleHorizontal, "min_angle_horizontal");
    config["observation"].getElement<double>(&this->maxAngleHorizontal, "max_angle_horizontal");
    config["observation"].getElement<double>(&this->minAngleVertical, "min_angle_vertical");
    config["observation"].getElement<double>(&this->maxAngleVertical, "max_angle_vertical");

    std::vector<double> meanErrorXYZ;
    config["noise"]["cartesian"].getElement<std::vector<double>>(&meanErrorXYZ, "mean");
    this->errorMeanXYZ = Eigen::Vector3d(meanErrorXYZ[0], meanErrorXYZ[1], meanErrorXYZ[2]);

    std::vector<double> sigmaErrorXYZ;
    config["noise"]["cartesian"].getElement<std::vector<double>>(&sigmaErrorXYZ, "sigma");
    this->errorSigmaXYZ = Eigen::Vector3d(sigmaErrorXYZ[0], sigmaErrorXYZ[1], sigmaErrorXYZ[2]);

    std::vector<double> meanErrorAngle;
    config["noise"]["angle"].getElement<std::vector<double>>(&meanErrorAngle, "mean");
    this->errorMeanAngle = Eigen::Vector2d(meanErrorAngle[0], meanErrorAngle[1]);

    std::vector<double> sigmaErrorAngle;
    config["noise"]["angle"].getElement<std::vector<double>>(&sigmaErrorAngle, "sigma");
    this->errorSigmaAngle = Eigen::Vector2d(sigmaErrorAngle[0], sigmaErrorAngle[1]);

    config["noise"]["range"].getElement<double>(&this->errorMeanRange, "mean");
    config["noise"]["range"].getElement<double>(&this->errorSigmaRange, "sigma");
    config["noise"]["range_relative"].getElement<double>(&this->errorMeanRangeRelative, "mean");
    config["noise"]["range_relative"].getElement<double>(&this->errorSigmaRangeRelative, "sigma");

    config["detection"].getElement<double>(&this->detection_prob_min_dist, "prob_min_dist");
    config["detection"].getElement<double>(&this->detection_prob_decrease_dist_linear, "prob_decrease_dist_linear");
    config["detection"].getElement<double>(
        &this->detection_prob_decrease_dist_quadratic, "prob_decrease_dist_quadratic");
    config["detection"].getElement<double>(&this->min_detection_prob, "min_prob");
}

LandmarkList PerceptionSensor::process(LandmarkList& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time)
{
    LandmarkList transformedLmsVehicle = transformLmList(in, trans, rot);
    LandmarkList transformedLms = transformLmList(transformedLmsVehicle, this->position, this->orientation);
    std::vector<Landmark> listNew = this->filterFoV(transformedLms.list);
    listNew = this->filterTypeAndDOO(listNew);
    listNew = this->handleFalsePositivesAndNegatives(listNew);
    listNew = this->addClassProbailities(listNew);
    listNew = this->addNoise(listNew);
    LandmarkList ret;
    ret.list = listNew;
    ret.timestamp = time;
    ret.frame_id = frame_id;
    numFrames += 1;
    return ret;
}

std::vector<Landmark> PerceptionSensor::filterFoV(std::vector<Landmark>& in)
{
    std::vector<Landmark> listNew;
    for (Landmark& lm : in)
    {
        double dist = lm.position.norm();
        double angleHorizontal = std::atan2(lm.position.y(), lm.position.x());
        // double angleVertical = std::asin(lm.position.z() / dist);
        // if (dist >= minRange && dist <= maxRange && angleHorizontal >= minAngleHorizontal
        //         && angleHorizontal <= maxAngleHorizontal && angleVertical >= minAngleVertical,
        //     angleVertical <= maxAngleVertical)
        if (dist >= minRange && dist <= maxRange && angleHorizontal >= minAngleHorizontal
            && angleHorizontal <= maxAngleHorizontal)
        {
            listNew.push_back(lm);
        }
    }
    return listNew;
}

std::vector<Landmark> PerceptionSensor::filterTypeAndDOO(std::vector<Landmark>& in)
{
    std::vector<Landmark> listNew;
    for (Landmark& lm : in)
    {
        if (lm.type != LandmarkType::INVISIBLE && (!lm.beenHit))
        {
            listNew.push_back(lm);
        }
    }
    return listNew;
}

std::vector<Landmark> PerceptionSensor::addNoise(std::vector<Landmark>& in)
{
    std::vector<Landmark> listNew;
    // provide numFrames as seed because the pseudo random generator would otherwise output the same values in
    // standstill and the noise would be constant
    std::default_random_engine generator(numFrames);
    std::normal_distribution<double> distX(errorMeanXYZ.x(), errorSigmaXYZ.x());
    std::normal_distribution<double> distY(errorMeanXYZ.y(), errorSigmaXYZ.y());
    std::normal_distribution<double> distZ(errorMeanXYZ.z(), errorSigmaXYZ.z());

    std::normal_distribution<double> distRange(errorMeanRange, errorSigmaRange);
    std::normal_distribution<double> distRangeRelative(errorMeanRangeRelative, errorSigmaRangeRelative);

    std::normal_distribution<double> distTheta(errorMeanAngle.x(), errorSigmaAngle.x());
    std::normal_distribution<double> distPhi(errorMeanAngle.y(), errorSigmaAngle.y());

    Eigen::Matrix3d rotationJacobian;

    for (Landmark& lm : in)
    {
        double dist = lm.position.norm();
        double theta = std::acos(lm.position.z() / dist);
        double phi = std::atan2(lm.position.y(), lm.position.x());

        Eigen::Matrix3d covSpherical = Eigen::Matrix3d::Zero();
        covSpherical(0, 0) = errorSigmaAngle.x() * errorSigmaAngle.x();
        covSpherical(1, 1) = errorSigmaAngle.y() * errorSigmaAngle.y();
        covSpherical(2, 2)
            = errorSigmaRange * errorSigmaRange + dist * dist * errorSigmaRangeRelative * errorSigmaRangeRelative;

        rotationJacobian(0, 0) = dist * std::cos(theta) * std::cos(phi);
        rotationJacobian(0, 1) = dist * std::sin(theta) * -std::sin(phi);
        rotationJacobian(0, 2) = std::sin(theta) * std::cos(phi);

        rotationJacobian(1, 0) = dist * std::cos(theta) * std::sin(phi);
        rotationJacobian(1, 1) = dist * std::sin(theta) * std::cos(phi);
        rotationJacobian(1, 2) = std::sin(theta) * std::sin(phi);

        rotationJacobian(2, 0) = -dist * std::sin(theta);
        rotationJacobian(2, 1) = 0.0;
        rotationJacobian(2, 2) = std::cos(theta);

        theta += distTheta(generator);
        phi += distPhi(generator);

        dist += dist * distRangeRelative(generator);
        dist += distRange(generator);

        lm.position.x() = dist * std::sin(theta) * std::cos(phi);
        lm.position.y() = dist * std::sin(theta) * std::sin(phi);
        lm.position.z() = dist * std::cos(theta);

        lm.position.x() += distX(generator);
        lm.position.y() += distY(generator);
        lm.position.z() += distZ(generator);

        lm.cov = rotationJacobian * covSpherical * rotationJacobian.transpose();
        lm.cov(0, 0) += errorSigmaXYZ.x() * errorSigmaXYZ.x();
        lm.cov(1, 1) += errorSigmaXYZ.y() * errorSigmaXYZ.y();
        lm.cov(2, 2) += errorSigmaXYZ.z() * errorSigmaXYZ.z();

        listNew.push_back(lm);
    }
    return listNew;
}

std::vector<Landmark> PerceptionSensor::addClassProbailities(std::vector<Landmark>& in)
{
    std::vector<Landmark> listNew;

    for (Landmark& lm : in)
    {
        std::fill(lm.typeWeights, lm.typeWeights + LandmarkType::UNKNOWN, 0);
        lm.typeWeights[lm.type] = 1.0;
        listNew.push_back(lm);
    }
    return listNew;
}

std::vector<Landmark> PerceptionSensor::handleFalsePositivesAndNegatives(std::vector<Landmark>& in)
{
    std::vector<Landmark> listNew;

    std::default_random_engine random_generator(this->numFrames);
    std::uniform_real_distribution<double> unif(0, 1.0);

    for (Landmark& lm : in)
    {
        double dist = lm.position.norm();
        double detection_prob = this->detection_prob_min_dist
            - (dist - this->minRange) * this->detection_prob_decrease_dist_linear
            - std::pow((dist - this->minRange), 2) * this->detection_prob_decrease_dist_quadratic;
        detection_prob = std::max(this->min_detection_prob, detection_prob);
        double random = unif(random_generator);
        if (random < detection_prob)
        {
            lm.detection_probability = detection_prob;
            listNew.push_back(lm);
        }
    }
    return listNew;
}

std::string PerceptionSensor::getName() { return this->name; }

std::string PerceptionSensor::getFrameId() { return this->frame_id; }

bool PerceptionSensor::RunTick(LandmarkList& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time)
{
    if (this->sampleReady(time))
    {
        LandmarkList value = this->process(in, trans, rot, time);
        this->deadTimeQueue.push(value);
        this->registerSampling();
    }
    return availableDeadTime(time);
}
