#include "sensorModels/imuSensor.hpp"

ImuSensor::ImuSensor(double rate, double deadTime)
{
    this->rate = rate;
    this->lastSampleTime = 0.0;
    this->deadTime = deadTime;
}

void ImuSensor::readConfig(ConfigElement& config)
{
    config.getElement<double>(&this->rate, "rate");
    config.getElement<double>(&this->deadTime, "dead_time");
    config["error_acc"].getElement<double>(&this->error_mean_acc, "mean");
    config["error_acc"].getElement<double>(&this->error_sigma_acc, "sigma");
    config["error_rot"].getElement<double>(&this->error_mean_rot, "mean");
    config["error_rot"].getElement<double>(&this->error_sigma_rot, "sigma");
    config.getElement<std::string>(&this->name, "name");
    config.getElement<std::string>(&this->frame, "frame");

    std::vector<double> position;
    config["pose"].getElement<std::vector<double>>(&position, "position");
    this->position = Eigen::Vector3d(position[0], position[1], position[2]);
    std::vector<double> orientation;
    config["pose"].getElement<std::vector<double>>(&orientation, "orientation");
    this->orientation = Eigen::Vector3d(orientation[0], orientation[1], orientation[2]);
}

bool ImuSensor::RunTick(ImuData& in, Eigen::Vector3d& alpha, double time)
{
    // TODO rigid body tranform of accleration
    if (this->sampleReady(time))
    {
        ImuData value = this->applyError(in);
        value.frame = "cog_static";
        this->deadTimeQueue.push(value);
        this->registerSampling();
    }
    return availableDeadTime(time);
}

ImuData ImuSensor::applyError(ImuData input)
{
    std::default_random_engine generator(numFrames);
    std::normal_distribution<double> distAccError(error_mean_acc, error_sigma_acc);
    std::normal_distribution<double> distRotError(error_mean_rot, error_sigma_rot);

    input.acc.x() += distAccError(generator);
    input.acc.y() += distAccError(generator);
    input.acc.z() += distAccError(generator);

    input.acc_cov(0, 0) = error_sigma_acc * error_sigma_acc;
    input.acc_cov(1, 1) = error_sigma_acc * error_sigma_acc;
    input.acc_cov(2, 2) = error_sigma_acc * error_sigma_acc;

    input.rot.x() += distRotError(generator);
    input.rot.y() += distRotError(generator);
    input.rot.z() += distRotError(generator);

    input.rot_cov(0, 0) = error_sigma_rot * error_sigma_rot;
    input.rot_cov(1, 1) = error_sigma_rot * error_sigma_rot;
    input.rot_cov(2, 2) = error_sigma_rot * error_sigma_rot;

    numFrames += 1;
    return input;
}

std::string ImuSensor::getName() { return this->name; }