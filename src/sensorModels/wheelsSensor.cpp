#include "sensorModels/wheelsSensor.hpp"

WheelsSensor::WheelsSensor(double rate, double deadTime) {
  this->rate = rate;
  this->lastSampleTime = 0.0;
  this->deadTime = deadTime;
}

void WheelsSensor::readConfig(ConfigElement& config)
{
    config.getElement<double>(&this->rate, "rate");
    config.getElement<double>(&this->deadTime, "dead_time");
    config["error"].getElement<double>(&this->error_mean, "mean");
    config["error"].getElement<double>(&this->error_sigma, "sigma");
}

bool WheelsSensor::RunTick(Wheels& in, Eigen::Vector3d& trans, Eigen::Vector3d& rot, double time) {
  if(this->sampleReady(time)) {
      Wheels value = this->applyError(in);
      this->deadTimeQueue.push(value);
      this->registerSampling();
  }
  return availableDeadTime(time);
}

Wheels WheelsSensor::applyError(Wheels input) {
  std::default_random_engine generator(numFrames);
  std::normal_distribution<double> distError(error_mean, error_sigma);

  input.FL += distError(generator);
  input.FR += distError(generator);
  input.RL += distError(generator);
  input.RR += distError(generator);
  numFrames += 1;
  return input;
}