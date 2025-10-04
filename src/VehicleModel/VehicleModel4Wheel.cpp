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
        this->acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->angularVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->angularAcceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

        this->torques = { 0.0, 0.0, 0.0, 0.0 };
        this->steeringAngles = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelOrientations = { 0.0, 0.0, 0.0, 0.0 };
        this->wheelspeeds = { 0.0, 0.0, 0.0, 0.0 };
        this->stateVectorBck.setZero();
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
        Eigen::Matrix3d rotMat = eulerAnglesToRotMat(this->orientation).transpose();
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

    void setSteeringSetpointFront(double in) { this->steeringFrontSetpoint = in; }

    void setSteeringSetpointRear(double in) { return; }

    void setPowerGroundSetpoint(double in) { this->powerGroundSetpoint = std::min(std::max(in, 0.0), 1.0); }

    void steeringKinematic(double in, double& leftAngle, double& rightAngle)
    {
        if (in > 0)
        {
            leftAngle = this->innerSteeringRatio * in;
            rightAngle = this->outerSteeringRatio * in;
        }
        else
        {
            leftAngle = this->outerSteeringRatio * in;
            rightAngle = this->innerSteeringRatio * in;
        }
        return;
    }

    void setSteeringFront(double in)
    {
        double left, right;
        steeringKinematic(in, left, right);
        this->steeringAngles.FL = left;
        this->steeringAngles.FR = right;
        return;
    }

    void setPosition(Eigen::Vector3d position) { 
        this->position = position;
        this->stateVectorBck(0, 0) = position[0];
        this->stateVectorBck(1, 0) = position[1];
    }
    void setOrientation(Eigen::Vector3d orientation) { 
        this->orientation = orientation;
        this->stateVectorBck(2, 0) = orientation[2];
    }

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

    Eigen::Matrix<double, 8, 1> getTireForcesStationary(
        Eigen::Matrix<double, 24, 1> stateVector, Eigen::Matrix<double, 4, 1> uVector, Wheels frictionCoefficients)
    {
        double fx_stat_fl;
        double fx_stat_fr;
        double fx_stat_rl;
        double fx_stat_rr;

        double fy_stat_fl;
        double fy_stat_fr;
        double fy_stat_rl;
        double fy_stat_rr;

        Wheels wheelspeeds;
        wheelspeeds.FL = stateVector(14, 0);
        wheelspeeds.FR = stateVector(15, 0);
        wheelspeeds.RL = stateVector(16, 0);
        wheelspeeds.RR = stateVector(17, 0);

        Wheels torques;
        torques.FL = uVector(0, 0);
        torques.FR = uVector(1, 0);
        torques.RL = uVector(2, 0);
        torques.RR = uVector(3, 0);

        Eigen::Vector3d velocity(stateVector(3, 0), stateVector(4, 0), 0.0);
        Eigen::Vector3d angularVelocity(0.0, 0.0, stateVector(5, 0));

        Eigen::Vector3d friction(std::min(200.0, 2000.0 * std::abs(velocity.x())),
        std::min(200.0, 2000.0 * std::abs(velocity.y())), std::min(200.0, 2000.0 * std::abs(velocity.z())));
        friction[0] = (velocity.x() > 0) ? friction.x() : -friction.x();
        friction[1] = (velocity.y() > 0) ? friction.y() : -friction.y();
        friction[2] = (velocity.z() > 0) ? friction.z() : -friction.z();

        double l = this->lr + this->lf;
        double vx = velocity.x();
        double vy = velocity.y();

        double F_aero_downforce = 0.5 * 1.29 * this->aeroArea * this->cla * (vx * vx) + this->powerGroundSetpoint * this->powerGroundForce;
        double F_aero_drag = 0.5 * 1.29 * this->aeroArea * this->cda * (vx * vx);

        double Fx_FL = stateVector(6, 0);
        double Fx_FR = stateVector(7, 0);
        double Fx_RL = stateVector(8, 0);
        double Fx_RR = stateVector(9, 0);

        double Fy_FL = stateVector(10, 0);
        double Fy_FR = stateVector(11, 0);
        double Fy_RL = stateVector(12, 0);
        double Fy_RR = stateVector(13, 0);

        double leftSteering, rightSteering;
        steeringKinematic(stateVector(22, 0), leftSteering, rightSteering);

        double axTires = (std::cos(leftSteering) * Fx_FL - std::sin(leftSteering) * Fy_FL
                             + std::cos(rightSteering) * Fx_FR - std::sin(rightSteering) * Fy_FR
                             + std::cos(this->steeringAngles.RL) * Fx_RL - std::sin(this->steeringAngles.RL) * Fy_RL
                             + std::cos(this->steeringAngles.RR) * Fx_RR - std::sin(this->steeringAngles.RR) * Fy_RR)
            / m;
        double ax = axTires - (F_aero_drag + friction.x()) / m;

        double ayTires = (std::sin(leftSteering) * Fx_FL + std::cos(leftSteering) * Fy_FL
                             + std::sin(rightSteering) * Fx_FR + std::cos(rightSteering) * Fy_FR
                             + std::sin(this->steeringAngles.RL) * Fx_RL + std::cos(this->steeringAngles.RL) * Fy_RL
                             + std::sin(this->steeringAngles.RR) * Fx_RR + std::cos(this->steeringAngles.RR) * Fy_RR)
            / m;
        double ay = (ayTires);

        double r = angularVelocity.z();
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

        Eigen::Vector3d vCog = velocity;
        Eigen::Vector3d omega = angularVelocity;

        Eigen::Vector3d rFL = Eigen::Vector3d(lf, 0.5 * sf, 0.0);
        Eigen::Vector3d rFR = Eigen::Vector3d(lf, -0.5 * sf, 0.0);
        Eigen::Vector3d rRL = Eigen::Vector3d(-lr, 0.5 * sr, 0.0);
        Eigen::Vector3d rRR = Eigen::Vector3d(-lr, -0.5 * sr, 0.0);

        Eigen::Vector3d vFL = vCog + omega.cross(rFL);
        Eigen::Vector3d vFR = vCog + omega.cross(rFR);
        Eigen::Vector3d vRL = vCog + omega.cross(rRL);
        Eigen::Vector3d vRR = vCog + omega.cross(rRR);

        double rpm2ms = this->wheelRadius * 2.0 * M_PI / (this->gearRatio * 60.0);
        double slipFL = getSlip(wheelspeeds.FL * rpm2ms, vFL.x());
        double slipFR = getSlip(wheelspeeds.FR * rpm2ms, vFR.x());
        double slipRL = getSlip(wheelspeeds.RL * rpm2ms, vRL.x());
        double slipRR = getSlip(wheelspeeds.RR * rpm2ms, vRR.x());

        bool stillstand = (vCog.norm() < 0.1) && (std::abs(angularVelocity.z()) < 0.001);

        slipFL *= ((std::abs(torques.FL) > 0.1) || (!stillstand)) ? 1.0 : 0.0;
        slipFR *= ((std::abs(torques.FR) > 0.1) || (!stillstand)) ? 1.0 : 0.0;
        slipRL *= ((std::abs(torques.RL) > 0.1) || (!stillstand)) ? 1.0 : 0.0;
        slipRR *= ((std::abs(torques.RR) > 0.1) || (!stillstand)) ? 1.0 : 0.0;

        // tire side slip angles
        double eps = 0.0001;
        double kappaFL
            = std::atan2(vFL.y(), std::max(std::abs(wheelspeeds.FL * rpm2ms), eps)) - leftSteering;
        double kappaFR
            = std::atan2(vFR.y(), std::max(std::abs(wheelspeeds.FR * rpm2ms), eps)) - rightSteering;
        double kappaRL
            = std::atan2(vRL.y(), std::max(std::abs(wheelspeeds.RL * rpm2ms), eps)) - this->steeringAngles.RL;
        double kappaRR
            = std::atan2(vRR.y(), std::max(std::abs(wheelspeeds.RR * rpm2ms), eps)) - this->steeringAngles.RR;

        // don't steer when vehicle doesn't move
        if (stillstand)
        {
            kappaFL = 0.0;
            kappaFR = 0.0;
            kappaRL = 0.0;
            kappaRR = 0.0;
        }

        // see https://research.chalmers.se/publication/523390/file/523390_Fulltext.pdf for combined slip calculations
        double syfl = std::max(std::min(vFL.y() / std::max(std::abs(wheelspeeds.FL * rpm2ms), eps), 1.0), -1.0);
        double syfr = std::max(std::min(vFR.y() / std::max(std::abs(wheelspeeds.FR * rpm2ms), eps), 1.0), -1.0);
        double syrl = std::max(std::min(vRL.y() / std::max(std::abs(wheelspeeds.RL * rpm2ms), eps), 1.0), -1.0);
        double syrr = std::max(std::min(vRR.y() / std::max(std::abs(wheelspeeds.RR * rpm2ms), eps), 1.0), -1.0);

        double absSlipFL = std::max(std::hypot(slipFL, syfl), 0.00001);
        double absSlipFR = std::max(std::hypot(slipFR, syfr), 0.00001);
        double absSlipRL = std::max(std::hypot(slipRL, syrl), 0.00001);
        double absSlipRR = std::max(std::hypot(slipRR, syrr), 0.00001);

        double M_FL = this->gearRatio * torques.FL;
        double M_FR = this->gearRatio * torques.FR;
        double M_RL = this->gearRatio * torques.RL;
        double M_RR = this->gearRatio * torques.RR;

        double minCombinedSlipFactor = 0.1;

        double Dlon_FL = this->Dlon * frictionCoefficients.FL * Fz_FL;
        double Dlon_FR = this->Dlon * frictionCoefficients.FR * Fz_FR;
        double Dlon_RL = this->Dlon * frictionCoefficients.RL * Fz_RL;
        double Dlon_RR = this->Dlon * frictionCoefficients.RR * Fz_RR;
        
        double Dlat_FL = this->Dlat * frictionCoefficients.FL * Fz_FL;
        double Dlat_FR = this->Dlat * frictionCoefficients.FR * Fz_FR;
        double Dlat_RL = this->Dlat * frictionCoefficients.RL * Fz_RL;
        double Dlat_RR = this->Dlat * frictionCoefficients.RR * Fz_RR;


        double fx_stat_fl_rel = processSlipRatioLon(slipFL);
        double fx_stat_fr_rel = processSlipRatioLon(slipFR);
        double fx_stat_rl_rel = processSlipRatioLon(slipRL);
        double fx_stat_rr_rel = processSlipRatioLon(slipRR);

        double fy_stat_fl_rel = processSlipAngleLat(kappaFL);
        double fy_stat_fr_rel = processSlipAngleLat(kappaFR);
        double fy_stat_rl_rel = processSlipAngleLat(kappaRL);
        double fy_stat_rr_rel = processSlipAngleLat(kappaRR);

        // https://chatgpt.com/share/68e158f0-b148-8010-9f1a-d7c586e99ae3
        double n_fl = std::sqrt(fx_stat_fl_rel*fx_stat_fl_rel + fy_stat_fl_rel*fy_stat_fl_rel);
        double n_fr = std::sqrt(fx_stat_fr_rel*fx_stat_fr_rel + fy_stat_fr_rel*fy_stat_fr_rel);
        double n_rl = std::sqrt(fx_stat_rl_rel*fx_stat_rl_rel + fy_stat_rl_rel*fy_stat_rl_rel);
        double n_rr = std::sqrt(fx_stat_rr_rel*fx_stat_rr_rel + fy_stat_rr_rel*fy_stat_rr_rel);

        // small beta -> more sharp grip loss
        double beta = 0.2;

        double S_fl = 1.0/std::sqrt(1+std::pow((std::max(0.0,n_fl-1.0)/beta),2.0));
        double S_fr = 1.0/std::sqrt(1+std::pow((std::max(0.0,n_fr-1.0)/beta),2.0));
        double S_rl = 1.0/std::sqrt(1+std::pow((std::max(0.0,n_rl-1.0)/beta),2.0));
        double S_rr = 1.0/std::sqrt(1+std::pow((std::max(0.0,n_rr-1.0)/beta),2.0));
        

        fx_stat_fl = S_fl * Dlon_FL * fx_stat_fl_rel;
        fx_stat_fr = S_fr * Dlon_FR * fx_stat_fr_rel;
        fx_stat_rl = S_rl * Dlon_RL * fx_stat_rl_rel;
        fx_stat_rr = S_rr * Dlon_RR * fx_stat_rr_rel;

        fy_stat_fl = S_fl * Dlat_FL * fy_stat_fl_rel;
        fy_stat_fr = S_fr * Dlat_FR * fy_stat_fr_rel;
        fy_stat_rl = S_rl * Dlat_RL * fy_stat_rl_rel;
        fy_stat_rr = S_rr * Dlat_RR * fy_stat_rr_rel;

        Eigen::Matrix<double, 8, 1> retVal;
        retVal(0, 0) = fx_stat_fl;
        retVal(1, 0) = fx_stat_fr;
        retVal(2, 0) = fx_stat_rl;
        retVal(3, 0) = fx_stat_rr;

        retVal(4, 0) = fy_stat_fl;
        retVal(5, 0) = fy_stat_fr;
        retVal(6, 0) = fy_stat_rl;
        retVal(7, 0) = fy_stat_rr;

        return retVal;
    }

    Eigen::Matrix<double, 8, 1> inverterWheelspeedControl(
        Eigen::Matrix<double, 24, 1> stateVector, Eigen::Matrix<double, 13, 1> uVector)
    {
        Wheels rpmSetpoints { uVector(1, 0), uVector(2, 0), uVector(3, 0), uVector(4, 0) };
        Wheels maxTorques { uVector(5, 0), uVector(6, 0), uVector(7, 0), uVector(8, 0) };
        Wheels minTorques { uVector(9, 0), uVector(10, 0), uVector(11, 0), uVector(12, 0) };
        Wheels wheelspeeds { stateVector(14, 0), stateVector(15, 0), stateVector(16, 0), stateVector(17, 0) };

        Wheels error = { rpmSetpoints.FL - wheelspeeds.FL, rpmSetpoints.FR - wheelspeeds.FR,
            rpmSetpoints.RL - wheelspeeds.RL, rpmSetpoints.RR - wheelspeeds.RR };
        Wheels wspdControlErrorIntegrator;
        wspdControlErrorIntegrator.FL = stateVector(18, 0);
        wspdControlErrorIntegrator.FR = stateVector(19, 0);
        wspdControlErrorIntegrator.RL = stateVector(20, 0);
        wspdControlErrorIntegrator.RR = stateVector(21, 0);

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

        Eigen::Matrix<double, 8, 1> ret;
        ret(0, 0) = FLt;
        ret(1, 0) = FRt;
        ret(2, 0) = RLt;
        ret(3, 0) = RRt;

        ret(4, 0) = error.FL;
        ret(5, 0) = error.FR;
        ret(6, 0) = error.RL;
        ret(7, 0) = error.RR;
        return ret;
    }

    Eigen::Matrix<double, 24, 1> getX_dot(
        Eigen::Matrix<double, 24, 1> state, Eigen::Matrix<double, 13, 1> uVector, Wheels frictionCoefficients, Eigen::Matrix<double, 2,1>& helperOutputs)
    {

        Eigen::Matrix<double, 8, 1> inverterStuff = inverterWheelspeedControl(state, uVector);

        Wheels wheelTorques;
        wheelTorques.FL = inverterStuff(0, 0);
        wheelTorques.FR = inverterStuff(1, 0);
        wheelTorques.RL = inverterStuff(2, 0);
        wheelTorques.RR = inverterStuff(3, 0);

        Eigen::Matrix<double, 4, 1> uVectorSimple;
        uVectorSimple(0, 0) = wheelTorques.FL;
        uVectorSimple(1, 0) = wheelTorques.FR;
        uVectorSimple(2, 0) = wheelTorques.RL;
        uVectorSimple(3, 0) = wheelTorques.RR;

        Eigen::Matrix<double, 8, 1> tireForces = getTireForcesStationary(state, uVectorSimple, frictionCoefficients);

        Eigen::Vector3d velocity(state(3, 0), state(4, 0), 0.0);
        Eigen::Vector3d angularVelocity(0.0, 0.0, state(5, 0));

        double l = this->lr + this->lf;
        double vx = velocity.x();
        double vy = velocity.y();

        // random value for wheel inertia
        double J = 0.2;

        double Fx_FL = state(6, 0);
        double Fx_FR = state(7, 0);
        double Fx_RL = state(8, 0);
        double Fx_RR = state(9, 0);

        double Fy_FL = state(10, 0);
        double Fy_FR = state(11, 0);
        double Fy_RL = state(12, 0);
        double Fy_RR = state(13, 0);

        double F_aero_downforce = 0.5 * 1.29 * this->aeroArea * this->cla * (vx * vx) + this->powerGroundSetpoint * this->powerGroundForce;
        double F_aero_drag = 0.5 * 1.29 * this->aeroArea * this->cda * (vx * vx);

        Eigen::Vector3d friction(std::min(200.0, 2000.0 * std::abs(velocity.x())),
            std::min(200.0, 2000.0 * std::abs(velocity.y())), std::min(200.0, 2000.0 * std::abs(velocity.z())));
        friction[0] = (velocity.x() > 0) ? friction.x() : -friction.x();
        friction[1] = (velocity.y() > 0) ? friction.y() : -friction.y();
        friction[2] = (velocity.z() > 0) ? friction.z() : -friction.z();

        double leftSteering, rightSteering;
        steeringKinematic(state(22, 0), leftSteering, rightSteering);

        double axTires = (std::cos(leftSteering) * Fx_FL - std::sin(leftSteering) * Fy_FL
                             + std::cos(rightSteering) * Fx_FR - std::sin(rightSteering) * Fy_FR
                             + std::cos(this->steeringAngles.RL) * Fx_RL - std::sin(this->steeringAngles.RL) * Fy_RL
                             + std::cos(this->steeringAngles.RR) * Fx_RR - std::sin(this->steeringAngles.RR) * Fy_RR)
            / m;
        double axModel = axTires - (F_aero_drag + friction.x()) / m;

        double ayTires = (std::sin(leftSteering) * Fx_FL + std::cos(leftSteering) * Fy_FL
                             + std::sin(rightSteering) * Fx_FR + std::cos(rightSteering) * Fy_FR
                             + std::sin(this->steeringAngles.RL) * Fx_RL + std::cos(this->steeringAngles.RL) * Fy_RL
                             + std::sin(this->steeringAngles.RR) * Fx_RR + std::cos(this->steeringAngles.RR) * Fy_RR)
            / m;
        double ayModel = (ayTires - friction.y());

        double rdotFx = 0.5 * this->sf * (-Fx_FL * std::cos(leftSteering) + Fx_FR * std::cos(rightSteering))
            + this->lf * (Fx_FL * std::sin(leftSteering) + Fx_FR * std::sin(rightSteering))
            + 0.5 * this->sr * (Fx_RR * std::cos(this->steeringAngles.RR) - Fx_RL * std::cos(this->steeringAngles.RL))
            - this->lr * (Fx_RL * std::sin(this->steeringAngles.RL) + Fx_RR * std::sin(this->steeringAngles.RR));
        double rdotFy = 0.5 * this->sf * (Fy_FL * std::sin(leftSteering) - Fy_FR * std::sin(rightSteering))
            + this->lf * (Fy_FL * std::cos(leftSteering) + Fy_FR * std::cos(rightSteering))
            + 0.5 * this->sr * (-Fy_RR * std::sin(this->steeringAngles.RR) + Fy_RL * std::sin(this->steeringAngles.RL))
            - this->lr * (Fy_RL * std::cos(this->steeringAngles.RL) + Fy_RR * std::cos(this->steeringAngles.RR));

        double M_FL = this->gearRatio * wheelTorques.FL;
        double M_FR = this->gearRatio * wheelTorques.FR;
        double M_RL = this->gearRatio * wheelTorques.RL;
        double M_RR = this->gearRatio * wheelTorques.RR;

        double M_total_FL = M_FL - Fx_FL * this->wheelRadius;
        double M_total_FR = M_FR - Fx_FR * this->wheelRadius;
        double M_total_RL = M_RL - Fx_RL * this->wheelRadius;
        double M_total_RR = M_RR - Fx_RR * this->wheelRadius;

        Eigen::Vector3d aModel(axModel, ayModel, 0.0);
        Eigen::Vector3d vdot = aModel - angularVelocity.cross(velocity);

        double xdot = vx * std::cos(state(2, 0)) - vy * std::sin(state(2, 0));
        double ydot = vx * std::sin(state(2, 0)) + vy * std::cos(state(2, 0));

        double phidot = angularVelocity.z();
        double vxdot = vdot.x();
        double vydot = vdot.y();
        double rdot = (1 / Izz * (rdotFx + rdotFy));

        double fx_fl_dot
            = (tireForces(0, 0) - state(6, 0)) / (this->relaxationLengthLon / std::max(velocity.x(), 5.0));
        double fx_fr_dot
            = (tireForces(1, 0) - state(7, 0)) / (this->relaxationLengthLon / std::max(velocity.x(), 5.0));
        double fx_rl_dot
            = (tireForces(2, 0) - state(8, 0)) / (this->relaxationLengthLon / std::max(velocity.x(), 5.0));
        double fx_rr_dot
            = (tireForces(3, 0) - state(9, 0)) / (this->relaxationLengthLon / std::max(velocity.x(), 5.0));

        double fy_fl_dot
            = (tireForces(4, 0) - state(10, 0)) / (this->relaxationLengthLat / std::max(velocity.x(), 5.0));
        double fy_fr_dot
            = (tireForces(5, 0) - state(11, 0)) / (this->relaxationLengthLat / std::max(velocity.x(), 5.0));
        double fy_rl_dot
            = (tireForces(6, 0) - state(12, 0)) / (this->relaxationLengthLat / std::max(velocity.x(), 5.0));
        double fy_rr_dot
            = (tireForces(7, 0) - state(13, 0)) / (this->relaxationLengthLat / std::max(velocity.x(), 5.0));

        double omega_fl_dot = 60 * (M_total_FL) / (J * 2 * M_PI);
        double omega_fr_dot = 60 * (M_total_FR) / (J * 2 * M_PI);
        double omega_rl_dot = 60 * (M_total_RL) / (J * 2 * M_PI);
        double omega_rr_dot = 60 * (M_total_RR) / (J * 2 * M_PI);

        double integrator_fl_dot = inverterStuff(4, 0);
        double integrator_fr_dot = inverterStuff(5, 0);
        double integrator_rl_dot = inverterStuff(6, 0);
        double integrator_rr_dot = inverterStuff(7, 0);

        double steering_a2 = 1 / (this->steering_w0 * this->steering_w0);
        double steering_a1 = 2 * this->steering_d / this->steering_w0;
        double steering_a0 = 1.0;
        double steering_acceleration
            = (steeringFrontSetpoint - steering_a0 * state(22, 0) - steering_a1 * state(23, 0)) / steering_a2;
        double steering_speed = std::min(std::max(state(23, 0), -this->steering_max_rate), this->steering_max_rate);

        // x,y,phi,vx,vy,r  fx(4), fx(4), omega(4), int_int(4), steering_pos, steering_vel
        Eigen::Matrix<double, 24, 1> ret;
        ret(0, 0) = xdot;
        ret(1, 0) = ydot;
        ret(2, 0) = phidot;
        ret(3, 0) = vxdot;
        ret(4, 0) = vydot;
        ret(5, 0) = rdot;

        ret(6, 0) = fx_fl_dot;
        ret(7, 0) = fx_fr_dot;
        ret(8, 0) = fx_rl_dot;
        ret(9, 0) = fx_rr_dot;

        ret(10, 0) = fy_fl_dot;
        ret(11, 0) = fy_fr_dot;
        ret(12, 0) = fy_rl_dot;
        ret(13, 0) = fy_rr_dot;

        ret(14, 0) = omega_fl_dot;
        ret(15, 0) = omega_fr_dot;
        ret(16, 0) = omega_rl_dot;
        ret(17, 0) = omega_rr_dot;

        ret(18, 0) = integrator_fl_dot;
        ret(19, 0) = integrator_fr_dot;
        ret(20, 0) = integrator_rl_dot;
        ret(21, 0) = integrator_rr_dot;

        ret(22, 0) = steering_speed;
        ret(23, 0) = steering_acceleration;

        helperOutputs(0, 0) = axModel;
        helperOutputs(1, 0) = ayModel;
        return ret;
    }

    Eigen::Matrix<double, 24, 1> capStates(Eigen::Matrix<double, 24, 1> state, Eigen::Matrix<double, 13, 1> uVector,
        Wheels frictionCoefficients, Eigen::Matrix<double, 24, 1> xDot)
    {
        Eigen::Matrix<double, 24, 1> ret = state;

        double kp = this->wspdControlKp;
        double ki = this->wspdControlKi;

        ret(18, 0) = std::max(std::min(ret(18, 0), std::max(maxTorques.FL - xDot(18, 0) * kp, 0.0)),
            std::min(minTorques.FL - xDot(18, 0) * kp, 0.0));
        ret(19, 0) = std::max(std::min(ret(19, 0), std::max(maxTorques.FR - xDot(19, 0) * kp, 0.0)),
            std::min(minTorques.FR - xDot(19, 0) * kp, 0.0));
        ret(20, 0) = std::max(std::min(ret(20, 0), std::max(maxTorques.RL - xDot(20, 0) * kp, 0.0)),
            std::min(minTorques.RL - xDot(20, 0) * kp, 0.0));
        ret(21, 0) = std::max(std::min(ret(21, 0), std::max(maxTorques.RR - xDot(21, 0) * kp, 0.0)),
            std::min(minTorques.RR - xDot(21, 0) * kp, 0.0));

        ret(22, 0) = std::min(std::max(ret(22, 0), -this->steering_max), this->steering_max);
        ret(23, 0) = std::min(std::max(ret(23, 0), -this->steering_max_rate), this->steering_max_rate);

        return ret;
    }

    Eigen::Matrix<double, 24, 1> eulerIntegration(double dt, Eigen::Matrix<double, 24, 1> state,
        Eigen::Matrix<double, 13, 1> uVector, Wheels frictionCoefficients, Eigen::Matrix<double, 2,1>& helperOutputs)
    {
        Eigen::Matrix<double, 24, 1> xdot = getX_dot(state, uVector, frictionCoefficients, helperOutputs);
        state = state + dt * xdot;
        Eigen::Matrix<double, 24, 1> cappedState = capStates(state, uVector, frictionCoefficients, xdot);
        return cappedState;
    }

    Eigen::Matrix<double, 24, 1> rk4Integration(double dt, Eigen::Matrix<double, 24, 1> state,
      Eigen::Matrix<double, 13, 1> uVector, Wheels frictionCoefficients, Eigen::Matrix<double, 2,1>& helperOutputs)
  {
    Eigen::Matrix<double, 2, 1> helpers1;
      Eigen::Matrix<double, 24, 1> k1 = getX_dot(state, uVector, frictionCoefficients, helpers1);
      Eigen::Matrix<double, 24, 1> tempState1 = state + 0.5 * dt * k1;
      Eigen::Matrix<double, 2, 1> helpers2;
      Eigen::Matrix<double, 24, 1> k2 = getX_dot(tempState1, uVector, frictionCoefficients, helpers2);
      Eigen::Matrix<double, 24, 1> tempState2 = state + 0.5 * dt * k2;
      Eigen::Matrix<double, 2, 1> helpers3;
      Eigen::Matrix<double, 24, 1> k3 = getX_dot(tempState2, uVector, frictionCoefficients, helpers3);
      Eigen::Matrix<double, 24, 1> tempState3 = state + 1.0 * dt * k3;
      Eigen::Matrix<double, 2, 1> helpers4;
      Eigen::Matrix<double, 24, 1> k4 = getX_dot(tempState3, uVector, frictionCoefficients, helpers4);
      state = state + dt * (1.0/6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
      Eigen::Matrix<double, 24, 1> xdot = (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
      Eigen::Matrix<double, 2, 1> helpers = (1.0 / 6.0) * (helpers1 + 2.0 * helpers2 + 2.0 * helpers3 + helpers4);
      Eigen::Matrix<double, 24, 1> cappedState = capStates(state, uVector, frictionCoefficients, xdot);
      helperOutputs = helpers;
      return cappedState;
  }

    void forwardIntegrate(double dt, Wheels frictionCoefficients)
    {
        Eigen::Matrix<double, 13, 1> uvector;

        uvector(0, 0) = this->steeringFrontSetpoint;
        uvector(1, 0) = this->rpmSetpoints.FL;
        uvector(2, 0) = this->rpmSetpoints.FR;
        uvector(3, 0) = this->rpmSetpoints.RL;
        uvector(4, 0) = this->rpmSetpoints.RR;

        uvector(5, 0) = this->maxTorques.FL;
        uvector(6, 0) = this->maxTorques.FR;
        uvector(7, 0) = this->maxTorques.RL;
        uvector(8, 0) = this->maxTorques.RR;

        uvector(9, 0) = this->minTorques.FL;
        uvector(10, 0) = this->minTorques.FR;
        uvector(11, 0) = this->minTorques.RL;
        uvector(12, 0) = this->minTorques.RR;

        Eigen::Matrix<double, 24, 1> stateVector = this->stateVectorBck;
        Eigen::Matrix<double, 2, 1> helperVars;

        this->stateVectorBck = rk4Integration(dt, stateVector, uvector, frictionCoefficients, helperVars);
        this->position[0] = this->stateVectorBck(0, 0);
        this->position[1] = this->stateVectorBck(1, 0);

        this->orientation[2] = this->stateVectorBck(2, 0);

        this->velocity[0] = this->stateVectorBck(3, 0);
        this->velocity[1] = this->stateVectorBck(4, 0);

        this->angularVelocity[2] = this->stateVectorBck(5, 0);

        this->acceleration[0] = helperVars(0,0);
        this->acceleration[1] = helperVars(1,0);
        this->acceleration[2] = 0.0;

        this->wheelspeeds.FL = this->stateVectorBck(14, 0);
        this->wheelspeeds.FR = this->stateVectorBck(15, 0);
        this->wheelspeeds.RL = this->stateVectorBck(16, 0);
        this->wheelspeeds.RR = this->stateVectorBck(17, 0);

        Eigen::Matrix<double, 8, 1> inverterTorques = inverterWheelspeedControl(this->stateVectorBck, uvector);
        this->torques.FL = inverterTorques(0, 0);
        this->torques.FR = inverterTorques(1, 0);
        this->torques.RL = inverterTorques(2, 0);
        this->torques.RR = inverterTorques(3, 0);

        setSteeringFront(this->stateVectorBck(22, 0));

        // really no need to do anything complex for this since it's only cosmetic
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
    double relaxationLengthLat = 0.1;

    double Blon = 11.5;
    double Clon = 2.196;
    double Dlon = 1.6;
    double Elon = 1.0;
    double relaxationLengthLon = 0.05;

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

    double steeringFrontSetpoint = 0.0;

    double steering_w0 = 35.0;
    double steering_d = 0.6;
    double steering_max = 1.834;
    double steering_max_rate = 5.0;

    // x,y,phi,vx,vy,r, fx(4), fy(4), omega(4), int_int(4), steering_position, steering_speed
    Eigen::Matrix<double, 24, 1> stateVectorBck;
};
