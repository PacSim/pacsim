#include "sensorModels/scalarValueSensor.hpp"

ScalarValueSensor::ScalarValueSensor(double rate, double deadTime) {
  this->rate = rate;
  this->lastSampleTime = 0.0;
  this->deadTime = deadTime;
  this->error_mean = 0.0;
  this->error_sigma = 0.0;
}

void ScalarValueSensor::readConfig(ConfigElement& config)
{
    config.getElement<double>(&this->rate, "rate");
    config.getElement<double>(&this->deadTime, "dead_time");
    config["error"].getElement<double>(&this->error_mean, "mean");
    config["error"].getElement<double>(&this->error_sigma, "sigma");
}

bool ScalarValueSensor::RunTick(StampedScalar& in, double time) {
  if(this->sampleReady(time)) {
      StampedScalar value = applyError(in);
      this->deadTimeQueue.push(value);
      this->registerSampling();
  }
  return availableDeadTime(time);
}

StampedScalar ScalarValueSensor::applyError(StampedScalar input) {
  std::default_random_engine generator(numFrames);
  std::normal_distribution<double> distError(error_mean, error_sigma);

  input.data += distError(generator);
  numFrames += 1;
  return input;
}