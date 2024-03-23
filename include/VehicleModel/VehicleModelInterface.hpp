#ifndef PACSIMVEHICLEMODELINTERFACE_HPP
#define PACSIMVEHICLEMODELINTERFACE_HPP

#include "configParser.hpp"
#include "types.hpp"
#include <Eigen/Core>

class IVehicleModel
{
public:
    // pure virtual function
    virtual Eigen::Vector3d getPosition() = 0;
    virtual Eigen::Vector3d getOrientation() = 0;
    virtual Eigen::Vector3d getAngularVelocity() = 0;
    virtual Eigen::Vector3d getAngularAcceleration() = 0;
    virtual Eigen::Vector3d getVelocity() = 0;
    virtual Eigen::Vector3d getAcceleration() = 0;
    virtual Wheels getSteeringAngles() = 0;
    virtual Wheels getWheelspeeds() = 0;
    virtual Wheels getTorques() = 0;
    virtual Wheels getWheelOrientations() = 0;
    virtual double getSteeringWheelAngle() = 0;
    virtual bool readConfig(ConfigElement& config) = 0;

    virtual void setTorques(Wheels in) = 0;
    virtual void setRpmSetpoints(Wheels in) = 0;
    virtual void setMinTorques(Wheels in) = 0;
    virtual void setMaxTorques(Wheels in) = 0;
    virtual void setSteeringSetpointFront(double in) = 0;
    virtual void setSteeringSetpointRear(double in) = 0;
    virtual void setPosition(Eigen::Vector3d position) = 0;
    virtual void setOrientation(Eigen::Vector3d orientation) = 0;

    virtual void forwardIntegrate(double dt) = 0;

protected:
    // states
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;

    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d angularAcceleration;

    Wheels wheelOrientations;
    Wheels wheelspeeds;
    // inputs
    Wheels torques;
    Wheels steeringAngles;

    double time;
};

#endif /* PACSIMVEHICLEMODELINTERFACE_HPP */