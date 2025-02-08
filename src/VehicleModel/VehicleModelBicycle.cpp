#include "VehicleModel/VehicleModelInterface.hpp"

#include "transform.hpp"
class VehicleModelBicycle : public IVehicleModel
{

public:
    VehicleModelBicycle()
    {
        this->position = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->orientation = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

        this->torques = { 0.0, 0.0, 0.0, 0.0 };
        this->steeringAngles = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelOrientations = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelspeeds = { 0.0, 0.0, 0.0, 0.0 };
    }

    bool readConfig(ConfigElement& config)
    {
        auto configModel = config["simple_bicycle_model"];
        configModel["kinematics"].getElement<double>(&this->lf, "lf");
        configModel["kinematics"].getElement<double>(&this->lr, "lr");
        configModel["kinematics"].getElement<double>(&this->sf, "sf");
        configModel["kinematics"].getElement<double>(&this->sr, "sr");

        configModel["tire"].getElement<double>(&this->Blat, "Blat");
        configModel["tire"].getElement<double>(&this->Clat, "Clat");
        configModel["tire"].getElement<double>(&this->Dlat, "Dlat");
        configModel["tire"].getElement<double>(&this->Elat, "Elat");

        configModel["aero"].getElement<double>(&this->cla, "cla");
        configModel["aero"].getElement<double>(&this->cda, "cda");
        configModel["aero"].getElement<double>(&this->aeroArea, "aeroArea");

        configModel.getElement<double>(&this->m, "m");
        configModel.getElement<double>(&this->Izz, "Izz");
        configModel.getElement<double>(&this->wheelRadius, "wheelRadius");
        configModel.getElement<double>(&this->gearRatio, "gearRatio");
        configModel.getElement<double>(&this->innerSteeringRatio, "innerSteeringRatio");
        configModel.getElement<double>(&this->outerSteeringRatio, "outerSteeringRatio");
        configModel.getElement<double>(&this->nominalVoltageTS, "nominalVoltageTS");
        configModel.getElement<double>(&this->powerGroundForce, "powerGroundForce");
        configModel.getElement<double>(&this->powertrainEfficiency, "powertrainEfficiency");

        return true;
    }

    Eigen::Vector3d getPosition()
    {
        Eigen::Vector3d ret = this->position;
        return ret;
    }

    Eigen::Vector3d getOrientation() { return this->orientation; }

    Eigen::Vector3d getVelocity() { return this->velocity; }

    Eigen::Vector3d getAcceleration() { return this->acceleration; }

    Eigen::Vector3d getAngularVelocity() { return this->angularVelocity; }

    Eigen::Vector3d getAngularAcceleration() { return this->angularAcceleration; }

    Wheels getSteeringAngles() { return this->steeringAngles; }

    double getSteeringWheelAngle()
    {
        return (this->steeringAngles.FL > 0) ? this->steeringAngles.FL / this->innerSteeringRatio
                                             : this->steeringAngles.FL / this->outerSteeringRatio;
    }

    double getVoltageTS() { return this->nominalVoltageTS; }

    double getCurrentTS()
    {
        double powerCoeff = 1.0 / 9.55;
        double powerFL = this->torques.FL * this->wheelspeeds.FL * powerCoeff;
        double powerFR = this->torques.FR * this->wheelspeeds.FR * powerCoeff;
        double powerRL = this->torques.RL * this->wheelspeeds.RL * powerCoeff;
        double powerRR = this->torques.RR * this->wheelspeeds.RR * powerCoeff;
        double totalPower = (powerFL + powerFR + powerRL + powerRR) / powertrainEfficiency;

        return (totalPower / this->nominalVoltageTS);
    }

    Wheels getWheelspeeds() { return this->wheelspeeds; }

    Wheels getWheelOrientations() { return this->wheelOrientations; }

    Wheels getTorques() { return this->torques; }

    std::array<Eigen::Vector3d, 4> getWheelPositions()
    {
        auto rotMat = eulerAnglesToRotMat(this->orientation).transpose();
        Eigen::Vector3d FL = rotMat * Eigen::Vector3d(this->lf, this->sf * 0.5, 0.0) + this->position;
        Eigen::Vector3d FR = rotMat * Eigen::Vector3d(this->lf, -this->sf * 0.5, 0.0) + this->position;
        Eigen::Vector3d RL = rotMat * Eigen::Vector3d(-this->lr, this->sr * 0.5, 0.0) + this->position;
        Eigen::Vector3d RR = rotMat * Eigen::Vector3d(-this->lr, -this->sr * 0.5, 0.0) + this->position;
        std::array<Eigen::Vector3d, 4> ret { FL, FR, RL, RR };
        return ret;
    }

    void setTorques(Wheels in) { this->torques = in; }

    void setRpmSetpoints(Wheels in) { this->rpmSetpoints = in; }

    void setMaxTorques(Wheels in) { this->maxTorques = in; }

    void setMinTorques(Wheels in) { this->minTorques = in; }

    void setSteeringSetpointFront(double in) { setSteeringFront(in); }

    void setSteeringSetpointRear(double in) { return; }

    void setPowerGroundSetpoint(double in) { this->powerGroundSetpoint = std::min(std::max(in, 0.0), 1.0); }

    void setSteeringFront(double in)
    {
        double avgRatio = 0.5 * (this->innerSteeringRatio + this->outerSteeringRatio);
        if (in > 0)
        {
            this->steeringAngles.FL = this->innerSteeringRatio * in;
            this->steeringAngles.FR = this->outerSteeringRatio * in;
        }
        else
        {
            this->steeringAngles.FL = this->outerSteeringRatio * in;
            this->steeringAngles.FR = this->innerSteeringRatio * in;
        }
        return;
    }

    void setPosition(Eigen::Vector3d position) { this->position = position; }
    void setOrientation(Eigen::Vector3d orientation) { this->orientation = orientation; }

    double processSlipAngleLat(double alpha)
    {
        return std::sin(Clat * std::atan(Blat * alpha - Elat * (Blat * alpha - std::atan(Blat * alpha))));
    }

    // ax, ay, rdot
    Eigen::Vector3d getDynamicStates(double dt, Wheels frictionCoefficients)
    {
        double l = this->lr + this->lf;
        double vx = this->velocity.x();
        double vy = this->velocity.y();
        double ax = this->acceleration.x();
        double ay = this->acceleration.y();
        double r = this->angularVelocity.z();
        // Downforce
        double F_aero_downforce
            = 0.5 * 1.29 * this->aeroArea * this->cla * (vx * vx) + this->powerGroundSetpoint * this->powerGroundForce;
        double F_aero_drag = 0.5 * 1.29 * this->aeroArea * this->cda * (vx * vx);
        double g = 9.81;
        double steeringFront = 0.5 * (this->steeringAngles.FL + this->steeringAngles.FR);

        // max because lifted tire makes no forces
        double Fz_Front = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lr / l));
        double Fz_Rear = std::max(0.0, ((m * g + F_aero_downforce) * 0.5 * this->lf / l));

        Eigen::Vector3d vCog = this->velocity;
        Eigen::Vector3d omega = this->angularVelocity;

        Eigen::Vector3d rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
        Eigen::Vector3d rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
        Eigen::Vector3d rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
        Eigen::Vector3d rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);
        Eigen::Vector3d rFront = Eigen::Vector3d(lf, 0.0, 0.0);
        Eigen::Vector3d rRear = Eigen::Vector3d(-lf, 0.0, 0.0);

        Eigen::Vector3d vFL = vCog + omega.cross(rFL);
        Eigen::Vector3d vFR = vCog + omega.cross(rFR);
        Eigen::Vector3d vRL = vCog + omega.cross(rRL);
        Eigen::Vector3d vRR = vCog + omega.cross(rRR);
        Eigen::Vector3d vFront = vCog + omega.cross(rFront);
        Eigen::Vector3d vRear = vCog + omega.cross(rRear);

        double rpm2ms = this->wheelRadius * 2.0 * M_PI / (this->gearRatio * 60.0);

        bool stillstand = (vCog.norm() < 0.1) && (std::abs(this->angularVelocity.z()) < 0.001);

        // tire side slip angles
        double eps = 0.00001;

        double kappaFront = std::atan2(vFront.y(), std::max(std::abs(vFront.x()), eps)) - steeringFront;
        double kappaRear = std::atan2(vRear.y(), std::max(std::abs(vRear.x()), eps));

        // don't steer when vehicle doesn't move
        if (stillstand)
        {
            kappaFront = 0.0;
            kappaRear = 0.0;
        }

        // old calculation as backup for now
        double Fx_FL = this->gearRatio * this->torques.FL / this->wheelRadius;
        double Fx_FR = this->gearRatio * this->torques.FR / this->wheelRadius;
        double Fx_RL = this->gearRatio * this->torques.RL / this->wheelRadius;
        double Fx_RR = this->gearRatio * this->torques.RR / this->wheelRadius;

        Fx_FL *= (((this->torques.FL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_FR *= (((this->torques.FR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RL *= (((this->torques.RL) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;
        Fx_RR *= (((this->torques.RR) > 0.5) || (vCog.x() > 0.3)) ? 1.0 : 0.0;

        double Dlat_Front = 0.5 * (frictionCoefficients.FL + frictionCoefficients.FR) * this->Dlat * Fz_Front;
        double Dlat_Rear = 0.5 * (frictionCoefficients.RL + frictionCoefficients.RR) * this->Dlat * Fz_Rear;

        double Fy_Front = Dlat_Front * processSlipAngleLat(kappaFront);
        double Fy_Rear = Dlat_Rear * processSlipAngleLat(kappaRear);

        // todo convert speed
        this->wheelspeeds.FL = vFL.x() / rpm2ms;
        this->wheelspeeds.FR = vFR.x() / rpm2ms;
        this->wheelspeeds.RL = vRL.x() / rpm2ms;
        this->wheelspeeds.RR = vRR.x() / rpm2ms;

        double axTires = (std::cos(this->steeringAngles.FL) * Fx_FL + std::cos(this->steeringAngles.FR) * Fx_FR + Fx_RL
                             + Fx_RR - std::sin(steeringFront) * Fy_Front)
            / m;
        double axModel = axTires - F_aero_drag / m;

        double ayTires = (std::sin(this->steeringAngles.FL) * Fx_FL + std::sin(this->steeringAngles.FR) * Fx_FR
                             + std::cos(steeringFront) * Fy_Front + Fy_Rear)
            / m;
        double ayModel = (ayTires);

        double rdotFx
            = 0.5 * this->sf * (-Fx_FL * std::cos(this->steeringAngles.FL) + Fx_FR * std::cos(this->steeringAngles.FR))
            + this->lf * (Fx_FL * std::sin(this->steeringAngles.FL) + Fx_FR * std::sin(this->steeringAngles.FR))
            + 0.5 * this->sr * (Fx_RR * std::cos(this->steeringAngles.RR) - Fx_RL * std::cos(this->steeringAngles.RL))
            - this->lr * (Fx_RL * std::sin(this->steeringAngles.RL) + Fx_RR * std::sin(this->steeringAngles.RR));
        double rdotFy = this->lf * (Fy_Front * std::cos(steeringFront)) - this->lr * (Fy_Rear);
        double rdot = (1 / Izz * (rdotFx + rdotFy));

        Eigen::Vector3d ret(axModel, ayModel, rdot);
        return ret;
    }

    void forwardIntegrate(double dt, Wheels frictionCoefficients)
    {
        Eigen::Vector3d friction(std::min(200.0, 2000.0 * std::abs(this->velocity.x())),
            std::min(200.0, 2000.0 * std::abs(this->velocity.y())),
            std::min(200.0, 2000.0 * std::abs(this->velocity.z())));
        friction[0] = (this->velocity.x() > 0) ? friction.x() : -friction.x();
        friction[1] = (this->velocity.y() > 0) ? friction.y() : -friction.y();
        friction[2] = (this->velocity.z() > 0) ? friction.z() : -friction.z();

        Eigen::AngleAxisd yawAngle(this->orientation.z(), Eigen::Vector3d::UnitZ());
        this->position += (yawAngle.matrix() * this->velocity) * dt;

        this->torques = this->maxTorques;

        Eigen::Vector3d xdotdyn = getDynamicStates(dt, frictionCoefficients);

        this->orientation += Eigen::Vector3d(0.0, 0.0, dt * angularVelocity.z());

        this->acceleration = Eigen::Vector3d(xdotdyn[0] - friction.x() / m, xdotdyn[1], 0.0);

        this->angularVelocity = (this->angularVelocity + Eigen::Vector3d(0.0, 0.0, xdotdyn[2] * dt));

        this->angularAcceleration = Eigen::Vector3d(0.0, 0.0, xdotdyn[2]);

        this->velocity += dt * (this->acceleration - this->angularVelocity.cross(this->velocity));
        this->wheelOrientations.FL = std::fmod(
            this->wheelOrientations.FL + (this->wheelspeeds.FL / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.FR = std::fmod(
            this->wheelOrientations.FR + (this->wheelspeeds.FR / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.RL = std::fmod(
            this->wheelOrientations.RL + (this->wheelspeeds.RL / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
        this->wheelOrientations.RR = std::fmod(
            this->wheelOrientations.RR + (this->wheelspeeds.RR / (60.0 * this->gearRatio)) * dt * 2.0 * M_PI,
            2.0 * M_PI);
    }

private:
    double lr = 0.72;
    double lf = 0.78;
    double sf = 1.15; // track width front
    double sr = 1.15; // track width rear

    double Blat = 9.63;
    double Clat = -1.39;
    double Dlat = 1.6;
    double Elat = 1.0;

    double cla = 3.7;
    double cda = 1.1;
    double aeroArea = 1.1;
    double m = 178.0;
    double Izz = 111.0;
    double wheelRadius = 0.206;
    double gearRatio = 12.23;
    double innerSteeringRatio = 0.255625;
    double outerSteeringRatio = 0.20375;
    double nominalVoltageTS = 550.0;
    double powerGroundSetpoint = 0.0;
    double powerGroundForce = 700.0;
    double powertrainEfficiency = 1.0;

    Wheels minTorques = { -0.0, -0.0, -0.0, -0.0 };
    Wheels maxTorques = { 0.0, 0.0, 0.0, 0.0 };
    Wheels rpmSetpoints = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFx = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFy = { 0.0, 0.0, 0.0, 0.0 };
};
