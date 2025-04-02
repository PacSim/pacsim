#include "VehicleModel/VehicleModelInterface.hpp"

#include "transform.hpp"
class VehicleModel4Wheel : public IVehicleModel
{

public:
    VehicleModel4Wheel()
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

    double getSlip(double wheelspeed, double vx)
    {
        double eps = 0.0001;
        double ret = std::abs(wheelspeed - vx) / std::max(std::abs(vx), eps);
        if (wheelspeed < vx)
        {
            ret = -ret;
        }
        return std::max(std::min(ret, 1.0), -1.0);
    }

    double processSlipRatioLon(double kappa)
    {
        return std::sin(Clon * std::atan(Blon * kappa - Elon * (Blon * kappa - std::atan(Blon * kappa))));
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
        double F_aero_downforce = 0.5 * 1.29 * this->aeroArea * this->cla * (vx * vx);
        double F_aero_drag = 0.5 * 1.29 * this->aeroArea * this->cda * (vx * vx);
        // Fz load shift
        double g = 9.81;

        // max because lifted tire makes no forces
        // 50/50 left/right weight distribution
        double Fz_FL = std::max(
            0.0, ((m * g + F_aero_downforce) * 0.5 * this->lr / l + 0.5 * (-ax / l - ay / sf) * m * this->hc));
        double Fz_FR = std::max(
            0.0, ((m * g + F_aero_downforce) * 0.5 * this->lr / l + 0.5 * (-ax / l + ay / sf) * m * this->hc));
        double Fz_RL = std::max(
            0.0, ((m * g + F_aero_downforce) * 0.5 * this->lf / l + 0.5 * (ax / l - ay / sr) * m * this->hc));
        double Fz_RR = std::max(
            0.0, ((m * g + F_aero_downforce) * 0.5 * this->lf / l + 0.5 * (ax / l + ay / sr) * m * this->hc));

        Eigen::Vector3d vCog = this->velocity;
        Eigen::Vector3d omega = this->angularVelocity;

        Eigen::Vector3d rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
        Eigen::Vector3d rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
        Eigen::Vector3d rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
        Eigen::Vector3d rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);

        Eigen::Vector3d vFL = vCog + omega.cross(rFL);
        Eigen::Vector3d vFR = vCog + omega.cross(rFR);
        Eigen::Vector3d vRL = vCog + omega.cross(rRL);
        Eigen::Vector3d vRR = vCog + omega.cross(rRR);

        double rpm2ms = this->wheelRadius * 2.0 * M_PI / (this->gearRatio * 60.0);
        double slipFL = getSlip(this->wheelspeeds.FL * rpm2ms, vFL.x());
        double slipFR = getSlip(this->wheelspeeds.FR * rpm2ms, vFR.x());
        double slipRL = getSlip(this->wheelspeeds.RL * rpm2ms, vRL.x());
        double slipRR = getSlip(this->wheelspeeds.RR * rpm2ms, vRR.x());

        bool stillstand = (vCog.norm() < 0.1) && (std::abs(this->angularVelocity.z()) < 0.001);

        slipFL *= ((std::abs(this->torques.FL) > 0.1) || (!stillstand)) ? 1.0 : 0.0;
        slipFR *= ((std::abs(this->torques.FR) > 0.1) || (!stillstand)) ? 1.0 : 0.0;
        slipRL *= ((std::abs(this->torques.RL) > 0.1) || (!stillstand)) ? 1.0 : 0.0;
        slipRR *= ((std::abs(this->torques.RR) > 0.1) || (!stillstand)) ? 1.0 : 0.0;

        // tire side slip angles
        double eps = 0.0001;
        double kappaFL
            = std::atan2(vFL.y(), std::max(std::abs(this->wheelspeeds.FL * rpm2ms), eps)) - this->steeringAngles.FL;
        double kappaFR
            = std::atan2(vFR.y(), std::max(std::abs(this->wheelspeeds.FR * rpm2ms), eps)) - this->steeringAngles.FR;
        double kappaRL
            = std::atan2(vRL.y(), std::max(std::abs(this->wheelspeeds.RL * rpm2ms), eps)) - this->steeringAngles.RL;
        double kappaRR
            = std::atan2(vRR.y(), std::max(std::abs(this->wheelspeeds.FR * rpm2ms), eps)) - this->steeringAngles.RR;

        // don't steer when vehicle doesn't move
        if (stillstand)
        {
            kappaFL = 0.0;
            kappaFR = 0.0;
            kappaRL = 0.0;
            kappaRR = 0.0;
        }

        // see https://research.chalmers.se/publication/523390/file/523390_Fulltext.pdf for combined slip calculations
        double syfl = std::max(std::min(vFL.y() / std::max(std::abs(this->wheelspeeds.FL * rpm2ms), eps), 1.0), -1.0);
        double syfr = std::max(std::min(vFR.y() / std::max(std::abs(this->wheelspeeds.FR * rpm2ms), eps), 1.0), -1.0);
        double syrl = std::max(std::min(vRL.y() / std::max(std::abs(this->wheelspeeds.RL * rpm2ms), eps), 1.0), -1.0);
        double syrr = std::max(std::min(vRR.y() / std::max(std::abs(this->wheelspeeds.RR * rpm2ms), eps), 1.0), -1.0);

        double absSlipFL = std::max(std::hypot(slipFL, syfl), 0.00001);
        double absSlipFR = std::max(std::hypot(slipFR, syfr), 0.00001);
        double absSlipRL = std::max(std::hypot(slipRL, syrl), 0.00001);
        double absSlipRR = std::max(std::hypot(slipRR, syrl), 0.00001);

        double M_FL = this->gearRatio * this->torques.FL;
        double M_FR = this->gearRatio * this->torques.FR;
        double M_RL = this->gearRatio * this->torques.RL;
        double M_RR = this->gearRatio * this->torques.RR;

        double minCombinedSlipFactor = 0.1;

        double Fx_FL = Fz_FL * Dlon * processSlipRatioLon(slipFL);
        double Fx_FR = Fz_FR * Dlon * processSlipRatioLon(slipFR);
        double Fx_RL = Fz_RL * Dlon * processSlipRatioLon(slipRL);
        double Fx_RR = Fz_RR * Dlon * processSlipRatioLon(slipRR);

        // double Fx_FL = std::max(std::min((std::abs(slipFL)/absSlipFL),1.0), minCombinedSlipFactor) * Fz_FL * Dlon *
        // processSlipRatioLon(slipFL); double Fx_FR = std::max(std::min((std::abs(slipFR)/absSlipFR),1.0),
        // minCombinedSlipFactor) * Fz_FR * Dlon * processSlipRatioLon(slipFR); double Fx_RL =
        // std::max(std::min((std::abs(slipRL)/absSlipRL),1.0), minCombinedSlipFactor) * Fz_RL * Dlon *
        // processSlipRatioLon(slipRL); double Fx_RR = std::max(std::min((std::abs(slipRR)/absSlipRR),1.0),
        // minCombinedSlipFactor) * Fz_RR * Dlon * processSlipRatioLon(slipRR);

        // TODO check whether it's better to use slip and not force as transient variable??

        if (vCog.norm() > 0.5)
        {
            this->currentFx.FL += dt * (Fx_FL - currentFx.FL) * this->wheelspeeds.FL * rpm2ms / relaxationLengthLon;
            this->currentFx.FR += dt * (Fx_FR - currentFx.FR) * this->wheelspeeds.FR * rpm2ms / relaxationLengthLon;
            this->currentFx.RL += dt * (Fx_RL - currentFx.RL) * this->wheelspeeds.RL * rpm2ms / relaxationLengthLon;
            this->currentFx.RR += dt * (Fx_RR - currentFx.RR) * this->wheelspeeds.RR * rpm2ms / relaxationLengthLon;
        }
        else
        {
            this->currentFx.FL = Fx_FL;
            this->currentFx.FR = Fx_FR;
            this->currentFx.RL = Fx_RL;
            this->currentFx.RR = Fx_RR;
        }

        Fx_FL = this->currentFx.FL;
        Fx_FR = this->currentFx.FR;
        Fx_RL = this->currentFx.RL;
        Fx_RR = this->currentFx.RR;

        double M_total_FL = M_FL - Fx_FL * this->wheelRadius;
        double M_total_FR = M_FR - Fx_FR * this->wheelRadius;
        double M_total_RL = M_RL - Fx_RL * this->wheelRadius;
        double M_total_RR = M_RR - Fx_RR * this->wheelRadius;

        // old calculation as backup for now
        // Fx_FL = this->gearRatio*this->torques.FL / this->wheelRadius;
        // Fx_FR = this->gearRatio*this->torques.FR / this->wheelRadius;
        // Fx_RL = this->gearRatio*this->torques.RL / this->wheelRadius;
        // Fx_RR = this->gearRatio*this->torques.RR / this->wheelRadius;

        double Dlat_FL = this->Dlat * Fz_FL;
        double Dlat_FR = this->Dlat * Fz_FR;
        double Dlat_RL = this->Dlat * Fz_RL;
        double Dlat_RR = this->Dlat * Fz_RR;

        Dlat_FL *= (vCog.norm() > 0.5)
            ? std::sqrt(std::max(1 - std::pow(Fx_FL / (Fz_FL * Dlon), 2.0), std::pow(0.1, 2.0)))
            : 1.0;
        Dlat_FR *= (vCog.norm() > 0.5)
            ? std::sqrt(std::max(1 - std::pow(Fx_FR / (Fz_FR * Dlon), 2.0), std::pow(0.1, 2.0)))
            : 1.0;
        Dlat_RL *= (vCog.norm() > 0.5)
            ? std::sqrt(std::max(1 - std::pow(Fx_RL / (Fz_RL * Dlon), 2.0), std::pow(0.1, 2.0)))
            : 1.0;
        Dlat_RR *= (vCog.norm() > 0.5)
            ? std::sqrt(std::max(1 - std::pow(Fx_FR / (Fz_RR * Dlon), 2.0), std::pow(0.1, 2.0)))
            : 1.0;

        double Fy_FL = Dlat_FL * processSlipAngleLat(kappaFL);
        double Fy_FR = Dlat_FR * processSlipAngleLat(kappaFR);
        double Fy_RL = Dlat_RL * processSlipAngleLat(kappaRL);
        double Fy_RR = Dlat_RR * processSlipAngleLat(kappaRR);

        // double Fy_FL = std::max(std::min((std::abs(syfl)/absSlipFL),1.0), minCombinedSlipFactor) * (this->Dlat *
        // Fz_FL) * processSlipAngleLat(kappaFL); double Fy_FR = std::max(std::min((std::abs(syfr)/absSlipFR),1.0),
        // minCombinedSlipFactor) * (this->Dlat * Fz_FR) * processSlipAngleLat(kappaFR); double Fy_RL =
        // std::max(std::min((std::abs(syrl)/absSlipRL),1.0), minCombinedSlipFactor) * (this->Dlat * Fz_RL) *
        // processSlipAngleLat(kappaRL); double Fy_RR = std::max(std::min((std::abs(syrr)/absSlipRR),1.0),
        // minCombinedSlipFactor) * (this->Dlat * Fz_RR) * processSlipAngleLat(kappaRR);

        // double relaxationTime = relaxationLengthLat / vFL.x()
        if (vCog.norm() > 0.5)
        {
            this->currentFy.FL += dt * (Fy_FL - currentFy.FL) * this->wheelspeeds.FL * rpm2ms / relaxationLengthLat;
            this->currentFy.FR += dt * (Fy_FR - currentFy.FR) * this->wheelspeeds.FR * rpm2ms / relaxationLengthLat;
            this->currentFy.RL += dt * (Fy_RL - currentFy.RL) * this->wheelspeeds.RL * rpm2ms / relaxationLengthLat;
            this->currentFy.RR += dt * (Fy_RR - currentFy.RR) * this->wheelspeeds.RR * rpm2ms / relaxationLengthLat;
        }
        else
        {
            this->currentFy.FL = Fy_FL;
            this->currentFy.FR = Fy_FR;
            this->currentFy.RL = Fy_RL;
            this->currentFy.RR = Fy_RR;
        }

        Fy_FL = this->currentFy.FL;
        Fy_FR = this->currentFy.FR;
        Fy_RL = this->currentFy.RL;
        Fy_RR = this->currentFy.RR;

        // random value for wheel inertia
        double J = 0.2;

        this->wheelspeeds.FL += 60 * (M_total_FL * dt) / (J * 2 * M_PI);
        this->wheelspeeds.FR += 60 * (M_total_FR * dt) / (J * 2 * M_PI);
        this->wheelspeeds.RL += 60 * (M_total_RL * dt) / (J * 2 * M_PI);
        this->wheelspeeds.RR += 60 * (M_total_RR * dt) / (J * 2 * M_PI);

        double axTires = (std::cos(this->steeringAngles.FL) * Fx_FL - std::sin(this->steeringAngles.FL) * Fy_FL
                             + std::cos(this->steeringAngles.FR) * Fx_FR - std::sin(this->steeringAngles.FR) * Fy_FR
                             + std::cos(this->steeringAngles.RL) * Fx_RL - std::sin(this->steeringAngles.RL) * Fy_RL
                             + std::cos(this->steeringAngles.RR) * Fx_RR - std::sin(this->steeringAngles.RR) * Fy_RR)
            / m;
        double axModel = axTires - F_aero_drag / m;

        double ayTires = (std::sin(this->steeringAngles.FL) * Fx_FL + std::cos(this->steeringAngles.FL) * Fy_FL
                             + std::sin(this->steeringAngles.FR) * Fx_FR + std::cos(this->steeringAngles.FR) * Fy_FR
                             + std::sin(this->steeringAngles.RL) * Fx_RL + std::cos(this->steeringAngles.RL) * Fy_RL
                             + std::sin(this->steeringAngles.RR) * Fx_RR + std::cos(this->steeringAngles.RR) * Fy_RR)
            / m;
        double ayModel = (ayTires);

        double rdotFx
            = 0.5 * this->sf * (-Fx_FL * std::cos(this->steeringAngles.FL) + Fx_FR * std::cos(this->steeringAngles.FR))
            + this->lf * (Fx_FL * std::sin(this->steeringAngles.FL) + Fx_FR * std::sin(this->steeringAngles.FR))
            + 0.5 * this->sr * (Fx_RR * std::cos(this->steeringAngles.RR) - Fx_RL * std::cos(this->steeringAngles.RL))
            - this->lr * (Fx_RL * std::sin(this->steeringAngles.RL) + Fx_RR * std::sin(this->steeringAngles.RR));
        double rdotFy
            = 0.5 * this->sf * (Fy_FL * std::sin(this->steeringAngles.FL) - Fy_FR * std::sin(this->steeringAngles.FR))
            + this->lf * (Fy_FL * std::cos(this->steeringAngles.FL) + Fy_FR * std::cos(this->steeringAngles.FR))
            + 0.5 * this->sr * (-Fy_RR * std::sin(this->steeringAngles.RR) + Fy_RL * std::sin(this->steeringAngles.RL))
            - this->lr * (Fy_RL * std::cos(this->steeringAngles.RL) + Fy_RR * std::cos(this->steeringAngles.RR));
        double rdot = (1 / Izz * (rdotFx + rdotFy));

        Eigen::Vector3d ret(axModel, ayModel, rdot);
        return ret;
    }

    Wheels inverterWheelspeedControl(double dt)
    {
        Wheels error = { this->rpmSetpoints.FL - this->wheelspeeds.FL, this->rpmSetpoints.FR - this->wheelspeeds.FR,
            this->rpmSetpoints.RL - this->wheelspeeds.RL, this->rpmSetpoints.RR - this->wheelspeeds.RR };
        double kp = this->wspdControlKp;
        double ki = this->wspdControlKi;
        double FLt = error.FL * kp + wspdControlErrorIntegrator.FL * ki;
        FLt = std::max(std::min(FLt, maxTorques.FL), minTorques.FL);
        double FRt = error.FR * kp + wspdControlErrorIntegrator.FR * ki;
        FRt = std::max(std::min(FRt, maxTorques.FR), minTorques.FR);
        double RLt = error.RL * kp + wspdControlErrorIntegrator.RL * ki;
        RLt = std::max(std::min(RLt, maxTorques.RL), minTorques.RL);
        double RRt = error.RR * kp + wspdControlErrorIntegrator.RR * ki;
        RRt = std::max(std::min(RRt, maxTorques.RR), minTorques.RR);

        Wheels ret = { FLt, FRt, RLt, RRt };
        Wheels integrator = { this->wspdControlErrorIntegrator.FL + dt * error.FL,
            this->wspdControlErrorIntegrator.FR + dt * error.FR, this->wspdControlErrorIntegrator.RL + dt * error.RL,
            this->wspdControlErrorIntegrator.RR + dt * error.RR };
        wspdControlErrorIntegrator.FL = std::max(std::min(integrator.FL, std::max(maxTorques.FL - error.FL * kp, 0.0)),
            std::min(minTorques.FL - error.FL * kp, 0.0));
        wspdControlErrorIntegrator.FR = std::max(std::min(integrator.FR, std::max(maxTorques.FR - error.FR * kp, 0.0)),
            std::min(minTorques.FR - error.FR * kp, 0.0));
        wspdControlErrorIntegrator.RL = std::max(std::min(integrator.RL, std::max(maxTorques.RL - error.RL * kp, 0.0)),
            std::min(minTorques.RL - error.RL * kp, 0.0));
        wspdControlErrorIntegrator.RR = std::max(std::min(integrator.RR, std::max(maxTorques.RR - error.RR * kp, 0.0)),
            std::min(minTorques.RR - error.RR * kp, 0.0));
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

        this->torques = inverterWheelspeedControl(dt);
        
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
    double relaxationLengthLat = 0.2;
    
    double Blon = 11.5;
    double Clon = 2.196;
    double Dlon = 1.6;
    double Elon = 1.0;
    double relaxationLengthLon = 0.1;

    double cla = 3.7;
    double cda = 1.1;
    double aeroArea = 1.1;
    double m = 178.0;
    double hc = 0.32;
    double Izz = 111.0;
    double wheelRadius = 0.206;
    double gearRatio = 12.23;
    double innerSteeringRatio = 0.255625;
    double outerSteeringRatio = 0.20375;
    double nominalVoltageTS = 550.0;
    double powerGroundSetpoint = 0.0;
    double powerGroundForce = 700.0;
    double powertrainEfficiency = 1.0;

    double wspdControlKp = 40.0 / 1000.0;
    double wspdControlKi = 10000.0 / 1000.0;
    Wheels wspdControlErrorIntegrator = { 0.0, 0.0, 0.0, 0.0 };

    Wheels minTorques = { -0.0, -0.0, -0.0, -0.0 };
    Wheels maxTorques = { 0.0, 0.0, 0.0, 0.0 };
    Wheels rpmSetpoints = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFx = { 0.0, 0.0, 0.0, 0.0 };
    Wheels currentFy = { 0.0, 0.0, 0.0, 0.0 };
};
