#include "sensorModels/perceptionSensor.hpp"
#include "configParser.hpp"
#include "VehicleModel/VehicleModelBicycle.cpp"
#include "VehicleModel/VehicleModel4Wheel.cpp"
#include "transform.hpp"
#include "sensorModels/imuSensor.hpp"
#include "track/trackLoader.hpp"

#include "logger.hpp"
#include "competitionLogic.hpp"

#include "VehicleModel/deadTime.hpp"


#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

int add(int i, int j) {
    return i + j;
}

int add2(int i, int j) {
    return i + j;
}

class CubicSpline {
private:
    std::vector<double> x, y;       // data points
    std::vector<double> a, b, c, d; // spline coefficients

public:
    // Construct spline with given data
    CubicSpline(const std::vector<double>& X, const std::vector<double>& Y) : x(X), y(Y) {
        int n = x.size() - 1;
        a = y;
        b.resize(n);
        c.resize(n + 1);
        d.resize(n);

        std::vector<double> h(n), alpha(n), l(n + 1), mu(n + 1), z(n + 1);

        for (int i = 0; i < n; i++)
            h[i] = x[i + 1] - x[i];

        for (int i = 1; i < n; i++)
            alpha[i] = (3.0 / h[i]) * (a[i + 1] - a[i]) - (3.0 / h[i - 1]) * (a[i] - a[i - 1]);

        l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;
        for (int i = 1; i < n; i++) {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        l[n] = 1.0; z[n] = 0.0; c[n] = 0.0;

        for (int j = n - 1; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
    }

    // Evaluate spline at point X
    double operator()(double X) const {
        int i = findInterval(X);
        double dx = X - x[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

    // Evaluate derivative at point X
    double derivative(double X) const {
        int i = findInterval(X);
        double dx = X - x[i];
        return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;
    }

private:
    // Find interval [x[i], x[i+1]] containing X
    int findInterval(double X) const {
        int n = x.size() - 1;
        int i = n - 1;
        for (int j = 0; j < n; j++) {
            if (X >= x[j] && X <= x[j + 1]) {
                i = j;
                break;
            }
        }
        return i;
    }
};

// like np.linspace
std::vector<double> linspace(double start, double end, int steps)
{
    std::vector<double> out;
    double last = start;
    double stepSize = ((end - start) / static_cast<double>(steps)) * 0.99999999;
    for (int i = 0; i < steps; ++i)
    {
        out.push_back(last + stepSize * i);
        // last = out.back();
    }
    out[out.size() - 1] = end - 0.00001;
    return out;
}

std::pair<double, double> findCurvlinearCoords(
    const CubicSpline& spline_x, const CubicSpline& spline_y, double arc_length, double x, double y)
{

    // sample at equal step size to get rough guess
    double stepsize = 0.2;
    int steps = (int)(arc_length / stepsize);
    auto ss = linspace(0, arc_length, steps);

    int bestInd = 0;
    double bestDist = 999.9;
    double bestS = 999.9;

    for (int i = 0; i < ss.size(); ++i)
    {
        double dist = std::sqrt(std::pow((spline_x(ss[i]) - x), 2) + std::pow((spline_y(ss[i]) - y), 2));

        if (dist < bestDist)
        {
            bestDist = dist;
            bestInd = i;
            bestS = ss[bestInd];
        }
    }

    // do binary search to refine
    for (int i = 1; i < 10; ++i)
    {
        double arc = bestS;
        for (int j = -1; j <= 1; ++j)
        {
            double dist = std::sqrt(std::pow((spline_x(arc + j * stepsize * std::pow(0.5, i)) - x), 2)
                + std::pow((spline_y(arc + j * stepsize * std::pow(0.5, i)) - y), 2));
            if (dist < bestDist)
            {
                bestDist = dist;
                bestS = arc + j * stepsize * std::pow(0.5, i);
            }
        }
    }

    double s = bestS;
    double n = bestDist;

    std::pair<double, double> ret(s,n);
    
    return ret;
}


std::vector<double> evalRay(
const std::pair<Eigen::Vector2d, Eigen::Vector2d>& ray,
const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& segsLeft,
const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& segsRight    
) {
    std::vector<double> ret;
    for(auto& i: segsLeft) {
        // auto o = ray.first;
        // auto a = i.first;
        // auto b = i.second;
        // auto d = ray.second;
        // auto v1 = o-a;
        // auto v2 = b-a;
        // Eigen::Vector2d v3(-d[1], d[0]);
        // double cross21 = v2.x() * v1.y() - v2.y() * v1.x();
        // double t1 = cross21 / v2.dot(v3);
        // double t2 = v1.dot(v3) / v2.dot(v3);
        auto d = ray.second;
        auto v1 = ray.first-i.first;
        auto v2 = i.second-i.first;
        Eigen::Vector2d v3(-d[1], d[0]);
        double cross21 = v2.x() * v1.y() - v2.y() * v1.x();
        double t1 = cross21 / v2.dot(v3);
        double t2 = v1.dot(v3) / v2.dot(v3);
        if((t1 >= 0) && ((t2 >= 0) && (t2 <= 1))) {
            ret.push_back(t1);
        }
    }
    for(auto& i: segsRight) {
        auto d = ray.second;
        auto v1 = ray.first-i.first;
        auto v2 = i.second-i.first;
        Eigen::Vector2d v3(-d[1], d[0]);
        double cross21 = v2.x() * v1.y() - v2.y() * v1.x();
        double t1 = cross21 / v2.dot(v3);
        double t2 = v1.dot(v3) / v2.dot(v3);
        if((t1 >= 0) && ((t2 >= 0) && (t2 <= 1))) {
            ret.push_back(t1);
        }
    }
    if(ret.size() == 0) {
        ret.push_back(0.0);
    }
    return ret;
}

std::pair<std::vector<double>, std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, double>>> runRangefinder(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation, const std::vector<double>& angles, 
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& segsLeft,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& segsRight) {
    
    // std::tuple<std::vector<double>, std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> ret;
    std::pair<std::vector<double>, std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, double>>> ret;
    std::vector<double> ret_distances;
    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, double>> ret_rays;
        // std::vector<Eigen::Vector2d> rayOrigins;
        // std::vector<Eigen::Vector2d> rayDirections;


    // auto angles = linspace(-M_PI/2.0, M_PI/2.0, 2*M+1);
    // angles = np.power(np.abs(angles),1) * np.sign(angles)
    // angles = angles * np.pi/2.0

    // Create a 2D rotation object
    Eigen::Rotation2D<double> rot(orientation[2]);

    // Convert it to a 2x2 matrix
    Eigen::Matrix2d R = rot.toRotationMatrix();

    for(auto& i: angles) {
        Eigen::Vector2d rayDir(std::cos(i), std::sin(i));
        auto vec = R*rayDir;
        std::pair<Eigen::Vector2d, Eigen::Vector2d> ray(position.head<2>(), vec);
        auto distances = evalRay(ray, segsLeft, segsRight);
        auto it = std::min_element(distances.begin(), distances.end());
        // std::tuple<double, Eigen::Vector2d, Eigen::Vector2d> val(*it, )
        ret_distances.push_back(*it);
        auto rayTuple = std::make_tuple(ray.first, ray.second, *it);
        ret_rays.push_back(rayTuple);
    }
    ret.first = ret_distances;
    ret.second = ret_rays;
    return ret;
}


PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function that adds two numbers");
    m.def("add2", &add2, "A function that adds two numbers");

    m.def("evalRayBinded", &evalRay, "Takes a ray (pair of start position and direction) and gives distances to all intersections with track");

    pybind11::class_<CubicSpline>(m, "CubicSpline")
    // .def(py::init())
    .def(pybind11::init<const std::vector<double>, const std::vector<double>>())
    .def("__call__", &CubicSpline::operator())  // bind operator()
    .def("derivative", &CubicSpline::derivative);
    // .def("getOldest", &ImuSensor::getOldest);

    m.def("findCurvlinearCoords", &findCurvlinearCoords, "Blabla");

    m.def("runRangefinder", &runRangefinder, "Blabla");


    pybind11::class_<Config>(m, "Config")
    // .def(py::init())
    .def(pybind11::init<std::string>())
    .def("getElement", &ConfigElement::getConfigElement, "Set the pet's age");
    // .def("getElement", &Config::getElement);
    // .def("getElement", pybind11::overload_cast<std::string>(&ConfigElement::getElement), "Set the pet's age");
    // .def("getElement", pybind11::overload_cast<std::string>(&Config::getElement), "Set the pet's age");
    // .def_readwrite("coords", &Landmark::coords)
    // .def_readwrite("id", &Landmark::id)
    // .def_readwrite("colors", &Landmark::colors);

    pybind11::class_<ConfigElement>(m, "ConfigElement")
    .def("getElement", &ConfigElement::getConfigElement, "Set the pet's age")
    .def("getElements", pybind11::overload_cast<>(&ConfigElement::getElements), "Set the pet's age");
    // // .def(py::init())
    // .def(pybind11::init<std::string>())
    // .def("getElement", &Config::getElement);
    // .def("getElement", pybind11::overload_cast<std::string>(&Config::getElement), "Set the pet's age");

    pybind11::class_<Wheels>(m, "Wheels")
    .def(pybind11::init())
    .def_readwrite("FL", &Wheels::FL)
    .def_readwrite("FR", &Wheels::FR)
    .def_readwrite("RL", &Wheels::RL)
    .def_readwrite("RR", &Wheels::RR)
    .def_readwrite("timestamp", &Wheels::timestamp);

    pybind11::class_<ImuData>(m, "ImuData")
    .def(pybind11::init())
    .def_readwrite("acceleration", &ImuData::acc)
    .def_readwrite("rot", &ImuData::rot)
    .def_readwrite("acc_cov", &ImuData::acc_cov)
    .def_readwrite("rot_cov", &ImuData::rot_cov)
    .def_readwrite("timestamp", &ImuData::timestamp);
    // Eigen::Vector3d acc;
    // Eigen::Vector3d rot;

    // Eigen::Matrix3d acc_cov;
    // Eigen::Matrix3d rot_cov;

    // double timestamp;
    // std::string frame;

    pybind11::class_<Landmark>(m, "Landmark")
    .def(pybind11::init())
    .def_readwrite("id", &Landmark::id)
    .def_readwrite("position", &Landmark::position)
    .def_readwrite("cov", &Landmark::cov)
    .def_readwrite("beenHit", &Landmark::beenHit);


    pybind11::class_<Track>(m, "Track")
    .def(pybind11::init())
    .def_readwrite("left_lane", &Track::left_lane)
    .def_readwrite("right_lane", &Track::right_lane)
    .def_readwrite("unknown", &Track::unknown)
    .def_readwrite("path_left_point_indices", &Track::path_left_point_indices)
    .def_readwrite("path_right_point_indices", &Track::path_right_point_indices)
    .def_readwrite("time_keeping_gates", &Track::time_keeping_gates);

    pybind11::class_<MainConfig>(m, "MainConfig")
    .def(pybind11::init());

    // .def_readwrite("timestamp", &Track::timestamp);


    pybind11::class_<VehicleModelBicycle>(m, "VehicleModel")
    // .def(py::init())
    .def(pybind11::init())
    .def("readConfig", &VehicleModelBicycle::readConfig)
    // .def("readConfigString", &VehicleModelBicycle::readConfigString)
    .def("forwardIntegrate", &VehicleModelBicycle::forwardIntegrate)
    .def("setSteeringSetpointFront", &VehicleModelBicycle::setSteeringSetpointFront)
    .def("setRpmSetpoints", &VehicleModelBicycle::setRpmSetpoints)
    .def("setMinTorques", &VehicleModelBicycle::setMinTorques)
    .def("setMaxTorques", &VehicleModelBicycle::setMaxTorques)
    .def("getPosition", &VehicleModelBicycle::getPosition)
    .def("getOrientation", &VehicleModelBicycle::getOrientation)
    .def("getVelocity", &VehicleModelBicycle::getVelocity)
    .def("getAcceleration", &VehicleModelBicycle::getAcceleration)
    .def("getAngularVelocity", &VehicleModelBicycle::getAngularVelocity)
    .def("getWheelspeeds", &VehicleModelBicycle::getWheelspeeds)
    .def("getWheelOrientations", &VehicleModelBicycle::getWheelOrientations)
    .def("getSteeringWheelAngle", &VehicleModelBicycle::getSteeringWheelAngle)
    .def("getTorques", &VehicleModelBicycle::getTorques)
    .def("setPosition", &VehicleModelBicycle::setPosition)
    .def("setOrientation", &VehicleModelBicycle::setOrientation);


    pybind11::class_<VehicleModel4Wheel>(m, "VehicleModel4Wheel")
    // .def(py::init())
    .def(pybind11::init())
    .def("readConfig", &VehicleModel4Wheel::readConfig)
    // .def("readConfigString", &VehicleModelBicycle::readConfigString)
    .def("forwardIntegrate", &VehicleModel4Wheel::forwardIntegrate)
    .def("setSteeringSetpointFront", &VehicleModel4Wheel::setSteeringSetpointFront)
    .def("setRpmSetpoints", &VehicleModel4Wheel::setRpmSetpoints)
    .def("setMinTorques", &VehicleModel4Wheel::setMinTorques)
    .def("setMaxTorques", &VehicleModel4Wheel::setMaxTorques)
    .def("getPosition", &VehicleModel4Wheel::getPosition)
    .def("getOrientation", &VehicleModel4Wheel::getOrientation)
    .def("getVelocity", &VehicleModel4Wheel::getVelocity)
    .def("getAcceleration", &VehicleModel4Wheel::getAcceleration)
    .def("getAngularVelocity", &VehicleModel4Wheel::getAngularVelocity)
    .def("getWheelspeeds", &VehicleModel4Wheel::getWheelspeeds)
    .def("getWheelOrientations", &VehicleModel4Wheel::getWheelOrientations)
    .def("getSteeringWheelAngle", &VehicleModel4Wheel::getSteeringWheelAngle)
    .def("getTorques", &VehicleModel4Wheel::getTorques)
    .def("setPosition", &VehicleModel4Wheel::setPosition)
    .def("setOrientation", &VehicleModel4Wheel::setOrientation);


    pybind11::class_<ImuSensor>(m, "ImuSensor")
    // .def(py::init())
    .def(pybind11::init<double, double>())
    .def("readConfig", &ImuSensor::readConfig)
    .def("RunTick", &ImuSensor::RunTick)
    .def("getOldest", &ImuSensor::getOldest);


    m.def("loadMap", &loadMap);


    pybind11::class_<Logger, std::shared_ptr<Logger>>(m, "Logger")
    .def(pybind11::init());


    pybind11::class_<CompetitionLogic>(m, "CompetitionLogic")
    .def(pybind11::init<std::shared_ptr<Logger>, Track&, MainConfig>())
    .def("performAllChecks", &CompetitionLogic::performAllChecks)
    .def("pointsInTrackConnected", &CompetitionLogic::pointsInTrackConnected)
    .def("fillReport", &CompetitionLogic::fillReport);


    typedef DeadTime<Wheels> wheelsDeadtime;
    pybind11::class_<wheelsDeadtime>(m, "WheelsDeadtime")
    .def(pybind11::init<double>())
    .def("getOldest", &wheelsDeadtime::getOldest)
    .def("availableDeadTime", &wheelsDeadtime::availableDeadTime)
    .def("addVal", &wheelsDeadtime::addVal);

    typedef DeadTime<double> scalarDeadtime;
    pybind11::class_<scalarDeadtime>(m, "ScalarDeadtime")
    .def(pybind11::init<double>())
    .def("getOldest", &scalarDeadtime::getOldest)
    .def("availableDeadTime", &scalarDeadtime::availableDeadTime)
    .def("addVal", &scalarDeadtime::addVal);

    // .def_readwrite("coords", &Landmark::coords)
    // .def_readwrite("id", &Landmark::id)
    // .def_readwrite("colors", &Landmark::colors);

}
 

// export CPATH=/usr/include/python3.10:$CPATH
// export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH