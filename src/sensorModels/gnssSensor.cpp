#include "sensorModels/gnssSensor.hpp"

GnssSensor::GnssSensor(double rate, double deadTime)
{
    this->rate = rate;
    this->lastSampleTime = 0.0;
    this->deadTime = deadTime;
}

void GnssSensor::readConfig(ConfigElement& config) { return; }

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

bool GnssSensor::RunTick(Eigen::Vector3d& gnssOrigin, Eigen::Vector3d& enuToTrackRotation, Eigen::Vector3d& trans,
    Eigen::Vector3d& rot, double time, Eigen::Vector3d velocity)
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
        Eigen::Vector3d ecefCar = rotEnuToEcef * (rotMatTrackToEnu * trans) + ecefRef;
        Eigen::Vector3d positionWgs = ecefToWgs84(ecefCar.x(), ecefCar.y(), ecefCar.z());

        Eigen::Vector3d velENU = rotMatTrackToEnu * eulerAnglesToRotMat(rot) * velocity;

        GnssData value;
        value.latitude = positionWgs.x();
        value.longitude = positionWgs.y();
        value.altitude = positionWgs.z();

        value.vel_east = velENU.x();
        value.vel_north = velENU.y();
        value.vel_up = velENU.z();

        this->deadTimeQueue.push(value);
        this->registerSampling();
    }
    return availableDeadTime(time);
}