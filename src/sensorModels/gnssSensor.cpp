#include "sensorModels/gnssSensor.hpp"

#include "quaternion.hpp"

GnssSensor::GnssSensor()
{
    this->rate = rate;
    this->lastSampleTime = 0.0;
    this->deadTime = deadTime;
    this->noiseSeed = 0;
}

void GnssSensor::readConfig(ConfigElement& config)
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

    std::vector<double> sigmaPosition;
    config["noise"]["position"].getElement<std::vector<double>>(&sigmaPosition, "sigma");
    this->errorSigmaPosition = Eigen::Vector3d(sigmaPosition[0], sigmaPosition[1], sigmaPosition[2]);
    std::vector<double> meanPosition;
    config["noise"]["position"].getElement<std::vector<double>>(&meanPosition, "mean");
    this->errorMeanPosition = Eigen::Vector3d(meanPosition[0], meanPosition[1], meanPosition[2]);

    std::vector<double> sigmaVelocity;
    config["noise"]["velocity"].getElement<std::vector<double>>(&sigmaVelocity, "sigma");
    this->errorSigmaVelocity = Eigen::Vector3d(sigmaVelocity[0], sigmaVelocity[1], sigmaVelocity[2]);
    std::vector<double> meanVelocity;
    config["noise"]["velocity"].getElement<std::vector<double>>(&meanVelocity, "mean");
    this->errorMeanVelocity = Eigen::Vector3d(meanVelocity[0], meanVelocity[1], meanVelocity[2]);

    std::vector<double> sigmaOrientation;
    config["noise"]["orientation"].getElement<std::vector<double>>(&sigmaOrientation, "sigma");
    this->errorSigmaOrientation = Eigen::Vector3d(sigmaOrientation[0], sigmaOrientation[1], sigmaOrientation[2]);
    std::vector<double> meanOrientation;
    config["noise"]["orientation"].getElement<std::vector<double>>(&meanOrientation, "mean");
    this->errorMeanOrientation = Eigen::Vector3d(meanOrientation[0], meanOrientation[1], meanOrientation[2]);

    config["features"].getElement<bool>(&this->outputVelocity, "velocity");
    config["features"].getElement<int>(&this->orientationMode, "orientation");

    return;
}

Eigen::Vector3d wgs84toEcef(double lat, double lon, double alt)
{
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    double deg2rad = M_PI / 180.0;
    double a = 6378137.0;
    double b = 6356752.314;
    double h = alt;
    double esqr = 1 - std::pow(b / a, 2);
    double N = a / (sqrt(1 - esqr * std::pow(std::sin(lat * deg2rad), 2)));
    double X = (N + h) * std::cos(lat * deg2rad) * std::cos(lon * deg2rad);
    double Y = (N + h) * std::cos(lat * deg2rad) * std::sin(lon * deg2rad);
    double Z = std::sin(lat * deg2rad) * ((1 - esqr) * N + h);
    return Eigen::Vector3d(X, Y, Z);
}

double getRn(double latitude)
{
    double a = 6378137.0;
    double b = 6356752.314;
    double esqr = 1 - std::pow(b / a, 2);

    double N = a / (sqrt(1 - esqr * std::pow(std::sin(latitude), 2)));
    return N;
}

Eigen::Vector3d ecefToWgs84(double x, double y, double z)
{
    // https://www.oc.nps.edu/oc2902w/coord/coordcvt.pdf
    double lon = std::atan2(y, x);
    double p = std::hypot(x, y);
    double deg2rad = M_PI / 180.0;
    double a = 6378137.0;
    double b = 6356752.314;

    double esqr = 1 - std::pow(b / a, 2);
    // there exists no closed form solution
    // iterative method to get height and latitude
    double h = 0.0;
    double lat = std::atan2(p, z);
    // 7 iterarations gave me error which rounds to 0 on doubles
    for (int i = 0; i < 7; ++i)
    {
        double rn = getRn(lat);
        h = (p / std::cos(lat)) - rn;
        lat = std::atan((z / p) / (1 - esqr * rn / (rn + h)));
    }
    lon = lon / deg2rad;
    lat = lat / deg2rad;
    return Eigen::Vector3d(lat, lon, h);
}

Eigen::Matrix3d getEnuToEcefRotMat(double lat, double lon)
{
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF
    double deg2rad = M_PI / 180.0;
    lat = lat * deg2rad;
    lon = lon * deg2rad;
    Eigen::Matrix3d ret;
    ret << -sin(lon), -sin(lat) * cos(lon), cos(lat) * cos(lon), cos(lon), -sin(lat) * sin(lon), cos(lat) * sin(lon),
        0.0, cos(lat), sin(lat);
    return ret;
}

std::string GnssSensor::getName() { return this->name; }

std::string GnssSensor::getFrameId() { return this->frame_id; }

bool GnssSensor::RunTick(Eigen::Vector3d& gnssOrigin, Eigen::Vector3d& enuToTrackRotation, Eigen::Vector3d& trans,
    Eigen::Vector3d& rot, double time, Eigen::Vector3d velocity, Eigen::Vector3d omega, Eigen::Vector3d start_position,
    Eigen::Vector3d start_orientation, bool trackPreTransformed)
{

    if (this->sampleReady(time))
    {
        double originLat = gnssOrigin.x();
        double originLon = gnssOrigin.y();
        double originAlt = gnssOrigin.z();
        auto originEcef = wgs84toEcef(originLat, originLon, originAlt);
        Eigen::Vector3d ecefRef = Eigen::Vector3d(originEcef.x(), originEcef.y(), originEcef.z());
        auto rotEnuToEcef = getEnuToEcefRotMat(originLat, originLon);
        Eigen::Vector3d rotTrackToENU(enuToTrackRotation.x(), enuToTrackRotation.y(), enuToTrackRotation.z());
        Eigen::Matrix3d rotMatTrackToEnu = eulerAnglesToRotMat(rotTrackToENU).transpose();
        Eigen::Vector3d actualTrackCar = start_position;
        // if the track was transformed, we need to account for the changed track origin
        if (trackPreTransformed)
        {
            actualTrackCar = eulerAnglesToRotMat(start_orientation).transpose() * trans + start_position;
        }

        Eigen::Matrix3d R_body_to_world = eulerAnglesToRotMat(rot).transpose();
        if (trackPreTransformed)
        {
            R_body_to_world = eulerAnglesToRotMat(start_orientation).transpose() * R_body_to_world;
        }
        Eigen::Vector3d gpsLeverArmWorld = R_body_to_world * this->position;
        actualTrackCar += gpsLeverArmWorld;

        Eigen::Vector3d enuCar = rotMatTrackToEnu * actualTrackCar;

        std::default_random_engine generator(noiseSeed);

        std::normal_distribution<double> distX(errorMeanPosition.x(), errorSigmaPosition.x());
        std::normal_distribution<double> distY(errorMeanPosition.y(), errorSigmaPosition.y());
        std::normal_distribution<double> distZ(errorMeanPosition.z(), errorSigmaPosition.z());

        std::normal_distribution<double> distXOr(errorMeanOrientation.x(), errorSigmaOrientation.x());
        std::normal_distribution<double> distYOr(errorMeanOrientation.y(), errorSigmaOrientation.y());
        std::normal_distribution<double> distZOr(errorMeanOrientation.z(), errorSigmaOrientation.z());

        std::normal_distribution<double> distXVel(errorMeanVelocity.x(), errorSigmaVelocity.x());
        std::normal_distribution<double> distYVel(errorMeanVelocity.y(), errorSigmaVelocity.y());
        std::normal_distribution<double> distZVel(errorMeanVelocity.z(), errorSigmaVelocity.z());

        enuCar.x() += distX(generator);
        enuCar.y() += distY(generator);
        enuCar.z() += distZ(generator);
        Eigen::Vector3d ecefCar = rotEnuToEcef * enuCar + ecefRef;

        Eigen::Vector3d positionWgs = ecefToWgs84(ecefCar.x(), ecefCar.y(), ecefCar.z());

        Eigen::Vector3d velENU
            = rotMatTrackToEnu * eulerAnglesToRotMat(rot) * rigidBodyVelocity(velocity, this->position, omega);
        if (trackPreTransformed)
        {
            velENU = rotMatTrackToEnu * eulerAnglesToRotMat(start_orientation) * eulerAnglesToRotMat(rot)
                * rigidBodyVelocity(velocity, this->position, omega);
        }

        // sensor->car->track->enu
        quaternion q0 = quatFromEulerAngles(this->orientation);
        quaternion q1 = quatFromEulerAngles(rot);
        quaternion q11 = quatFromEulerAngles(start_orientation);
        quaternion qCar = q1;
        // if the track was transformed, we need to account for the changed track origin
        if (trackPreTransformed)
        {
            qCar = quatMult(q11, q1);
        }
        quaternion q2 = quatFromEulerAngles(rotTrackToENU);
        quaternion q3 = quatMult(q2, quatMult(qCar, q0));
        // apply noise as additional rotation
        quaternion q4
            = quatFromEulerAngles(Eigen::Vector3d(distXOr(generator), distYOr(generator), distZOr(generator)));
        quaternion q5 = quatMult(q4, q3);

        quaternion outQuat = quatFromEulerAngles(Eigen::Vector3d(0.0, 0.0, 0.0));
        if (this->orientationMode == 1)
        {
            Eigen::Matrix3d testMat = rotationMatrixFromQuaternion(q5);
            Eigen::Vector3d testVec = testMat * Eigen::Vector3d(1, 0, 0);
            double angleN = std::atan2(testVec.y(), testVec.x());
            outQuat = quatFromEulerAngles(Eigen::Vector3d(0.0, 0.0, angleN));
        }
        else if (this->orientationMode == 2)
        {
            outQuat = q5;
        }

        GnssData value;
        value.latitude = positionWgs.x();
        value.longitude = positionWgs.y();
        value.altitude = positionWgs.z();
        value.position_covariance = this->errorSigmaPosition.asDiagonal();
        value.position_covariance = value.position_covariance.array().square();

        if (this->outputVelocity)
        {
            value.vel_east = velENU.x() + distXVel(generator);
            value.vel_north = velENU.y() + distYVel(generator);
            value.vel_up = velENU.z() + distZVel(generator);
            value.velocity_covariance = this->errorSigmaVelocity.asDiagonal();
            value.velocity_covariance = value.velocity_covariance.array().square();
        }
        else
        {
            value.vel_east = 0.0;
            value.vel_north = 0.0;
            value.vel_up = 0.0;
            value.velocity_covariance(0, 0) = -1.0;
            value.velocity_covariance(1, 1) = -1.0;
            value.velocity_covariance(2, 2) = -1.0;
        }

        value.orientation = outQuat;
        value.orientation_covariance = this->errorSigmaOrientation.asDiagonal();
        value.orientation_covariance = value.orientation_covariance.array().square();

        value.fix_status = GnssData::FixStatus::FIX;

        value.timestamp = time;
        value.frame = this->frame_id;

        noiseSeed += 1;

        this->deadTimeQueue.push(value);
        this->registerSampling();
    }
    return availableDeadTime(time);
}