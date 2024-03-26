#include "rclcpp/rclcpp.hpp"
#include <sstream>

#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosgraph_msgs/msg/clock.h"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "VehicleModel/VehicleModelBicycle.cpp"
#include "configParser.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "track/trackLoader.hpp"

#include "sensorModels/imuSensor.hpp"
#include "sensorModels/perceptionSensor.hpp"
#include "sensorModels/scalarValueSensor.hpp"
#include "sensorModels/wheelsSensor.hpp"

#include "VehicleModel/deadTime.hpp"

#include <rosgraph_msgs/msg/clock.hpp>

#include "ros2Helpers.hpp"

#include "pacsim/msg/perception_detections.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"

#include "types.hpp"

#include "competitionLogic.hpp"

#include "std_srvs/srv/empty.hpp"

#include "reportWriter.hpp"

// DynamicDoubleTrackModel7Dof model;
std::shared_ptr<IVehicleModel> model;
std::string mapFilePrefix;
double simTime = 0.0;
rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub;
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mapVizPub;
rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steeringFrontPub;
rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steeringRearPub;
rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr wheelspeedPub;
rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torquesPub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perceptionVizPub;
std::vector<rclcpp::Publisher<pacsim::msg::PerceptionDetections>::SharedPtr> lms_pubs;
std::map<std::shared_ptr<PerceptionSensor>, rclcpp::Publisher<pacsim::msg::PerceptionDetections>::SharedPtr>
    perceptionSensorPublisherMap;
std::map<std::shared_ptr<PerceptionSensor>, std::shared_ptr<LandmarksMarkerWrapper>> perceptionSensorMarkersWrappersMap;
std::map<std::shared_ptr<PerceptionSensor>, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>
    perceptionSensorVizPublisherMap;
std::map<std::shared_ptr<ImuSensor>, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imuPublisherMap;

DeadTime<double> deadTimeSteeringFront(0.0);
DeadTime<double> deadTimeSteeringRear(0.0);
DeadTime<Wheels> deadTimeTorques(0.0);
DeadTime<Wheels> deadTimeWspdSetpoints(0.0);
DeadTime<Wheels> deadTimeMaxTorques(0.0);
DeadTime<Wheels> deadTimeMinTorques(0.0);

std::mutex mutexSimTime;

std::string trackName;
std::string trackFrame;
std::string report_file_dir;
std::string main_config_path;
std::string perception_config_path;
std::string sensors_config_path;
std::string vehicle_model_config_path;
std::string discipline;
std::vector<std::string> jointNames
    = { "FL_steer", "FL_rotate", "FR_steer", "FR_rotate", "RR_rotate", "RL_rotate", "steering" };
double realtimeRatio = 1.0;
MainConfig mainConfig;
std::vector<std::shared_ptr<PerceptionSensor>> perceptionSensors;
std::vector<std::shared_ptr<ImuSensor>> imus;
std::shared_ptr<CompetitionLogic> cl;

std::shared_ptr<ImuSensor> imuSensor;
std::shared_ptr<ScalarValueSensor> steeringSensorFront;
std::shared_ptr<ScalarValueSensor> steeringSensorRear;
std::shared_ptr<WheelsSensor> wheelspeedSensor;
std::shared_ptr<WheelsSensor> torquesSensor;

int threadMainLoopFunc(std::shared_ptr<rclcpp::Node> node)
{

    std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    /*
    auto static_broadcaster = std::make_shared<rclcpp::Node>("static_broadcaster");
    std::unique_ptr<tf2_ros::TransformBroadcaster> brstatic =
    std::make_unique<tf2_ros::TransformBroadcaster>(static_broadcaster);*/
    double timestep = 1.0 / 1000.0;
    double egoMotionSensorRate = 200.0;
    double lastEgoMotionSensorSampleTime = 0.0;
    std::string framePerception = "perception";
    std::string frameStateEstimation = "cog_static";

    Eigen::Vector3d start_position;
    Eigen::Vector3d start_orientation;
    Track lms = loadMap(trackName, start_position, start_orientation);
    LandmarkList trackAsLMList = trackToLMList(lms);
    model->setPosition(start_position);
    model->setOrientation(start_orientation);

    LandmarksMarkerWrapper mapMarkersWrapper(1.0, 0.0, 0.0, 0.3, 0.3, 0.5, 0.8, 0.15, "pacsim");

    visualization_msgs::msg::MarkerArray mapMarkerMsg = mapMarkersWrapper.markerFromLMs(lms, trackFrame, 0.0);
    mapVizPub->publish(mapMarkerMsg);

    deadTimeSteeringFront = DeadTime<double>(0.05);
    deadTimeSteeringRear = DeadTime<double>(0.05);
    deadTimeTorques = DeadTime<Wheels>(0.02);
    deadTimeWspdSetpoints = DeadTime<Wheels>(0.02);
    deadTimeMaxTorques = DeadTime<Wheels>(0.02);
    deadTimeMinTorques = DeadTime<Wheels>(0.02);
    double current_wheel_speed_angle = 0.0;

    auto nextLoopTime = std::chrono::steady_clock::now();

    cl = std::make_shared<CompetitionLogic>(lms, mainConfig);
    bool finish = false;
    while (rclcpp::ok() && !(finish))
    {
        rosgraph_msgs::msg::Clock clockMsg;
        clockMsg.clock = rclcpp::Time(static_cast<uint64_t>(simTime * 1e9));
        clockPub->publish(clockMsg);
        model->forwardIntegrate(timestep);
        auto t = model->getPosition();
        auto rEulerAngles = model->getOrientation();
        auto alpha = model->getAngularAcceleration();
        finish = cl->performAllChecks(lms, simTime, t, rEulerAngles);
        // geometry_msgs::msg::TransformStamped static_transform = createStaticTransform("map", "center", simTime);
        geometry_msgs::msg::TransformStamped transformStamped
            = createRosTransformMsg(t, rEulerAngles, trackFrame, "car", simTime);

        br->sendTransform(transformStamped);
        // brstatic->sendTransform(static_transform);

        if (deadTimeSteeringFront.availableDeadTime(simTime))
        {
            double val = deadTimeSteeringFront.getOldest();
            model->setSteeringSetpointFront(val);
        }
        if (deadTimeSteeringRear.availableDeadTime(simTime))
        {
            double val = deadTimeSteeringRear.getOldest();
            model->setSteeringSetpointRear(val);
        }
        if (deadTimeTorques.availableDeadTime(simTime))
        {
            Wheels val = deadTimeTorques.getOldest();
            model->setTorques(val);
        }
        if (deadTimeWspdSetpoints.availableDeadTime(simTime))
        {
            Wheels val = deadTimeWspdSetpoints.getOldest();
            model->setRpmSetpoints(val);
        }
        if (deadTimeMaxTorques.availableDeadTime(simTime))
        {
            Wheels val = deadTimeMaxTorques.getOldest();
            model->setMaxTorques(val);
        }
        if (deadTimeMinTorques.availableDeadTime(simTime))
        {
            Wheels val = deadTimeMinTorques.getOldest();
            model->setMinTorques(val);
        }

        if (simTime >= (lastEgoMotionSensorSampleTime + 1 / egoMotionSensorRate))
        {
            lastEgoMotionSensorSampleTime += 1 / egoMotionSensorRate;
            Eigen::Vector3d vel = model->getVelocity();
            Eigen::Vector3d rot = model->getAngularVelocity();
            geometry_msgs::msg::TwistWithCovarianceStamped velMsg = createRosTwistMsg(vel, rot, "car", simTime);
            velocity_pub->publish(velMsg);
        }

        ImuData imuDataCog { model->getAcceleration(), model->getAngularVelocity(), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), simTime, "" };
        for (auto& imuSensor : imus)
        {
            if (imuSensor->RunTick(imuDataCog, alpha, simTime))
            {
                ImuData imuData = imuSensor->getOldest();
                sensor_msgs::msg::Imu imuMsg = createRosImuMsg(imuData);
                imuPublisherMap[imuSensor]->publish(imuMsg);
            }
        }
        Wheels steeringCurr = model->getSteeringAngles();
        double steeringWheelCurr = model->getSteeringWheelAngle();

        StampedScalar steeringDataFront { steeringWheelCurr, simTime };
        // tire_positions for model

        //
        if (steeringSensorFront->RunTick(steeringDataFront, simTime))
        {
            StampedScalar steeringData = steeringSensorFront->getOldest();
            pacsim::msg::StampedScalar msg;
            msg.value = steeringData.data;
            msg.stamp = rclcpp::Time(static_cast<uint64_t>(steeringData.timestamp * 1e9));
            steeringFrontPub->publish(msg);
        }
        StampedScalar steeringDataRear { (steeringCurr.RL + steeringCurr.RR) * 0.5, simTime };
        if (steeringSensorRear->RunTick(steeringDataRear, simTime))
        {
            StampedScalar steeringData = steeringSensorRear->getOldest();
            pacsim::msg::StampedScalar msg;
            msg.value = steeringData.data;
            msg.stamp = rclcpp::Time(static_cast<uint64_t>(steeringData.timestamp * 1e9));
            steeringRearPub->publish(msg);
        }

        for (auto& perceptionSensor : perceptionSensors)
        {
            if (perceptionSensor->RunTick(trackAsLMList, t, rEulerAngles, simTime))
            {
                LandmarkList sensorLms = perceptionSensor->getOldest();
                // TODO fix conversion hack
                Track trackMsg = lmListToTrack(sensorLms);
                visualization_msgs::msg::MarkerArray lmsMarkerMsg
                    = perceptionSensorMarkersWrappersMap[perceptionSensor]->markerFromLMs(
                        trackMsg, sensorLms.frame_id, sensorLms.timestamp);
                perceptionSensorVizPublisherMap[perceptionSensor]->publish(lmsMarkerMsg);

                mapVizPub->publish(mapMarkerMsg);
                pacsim::msg::PerceptionDetections lmsMsg
                    = LandmarkListToRosMessage(sensorLms, sensorLms.frame_id, sensorLms.timestamp);
                perceptionSensorPublisherMap[perceptionSensor]->publish(lmsMsg);
            }
        }

        Wheels wspd = model->getWheelspeeds();
        Wheels torques = model->getTorques();
        wspd.timestamp = simTime;
        torques.timestamp = simTime;
        Wheels orientation = model->getWheelOrientations();
        if (wheelspeedSensor->RunTick(wspd, t, rEulerAngles, simTime))
        {
            wspd = wheelspeedSensor->getOldest();
            pacsim::msg::Wheels wspdMsg;
            wspdMsg.fl = wspd.FL;
            wspdMsg.fr = wspd.FR;
            wspdMsg.rl = wspd.RL;
            wspdMsg.rr = wspd.RR;
            wspdMsg.stamp = rclcpp::Time(static_cast<uint64_t>(wspd.timestamp * 1e9));
            wheelspeedPub->publish(wspdMsg);
        }
        if (torquesSensor->RunTick(torques, t, rEulerAngles, simTime))
        {
            torques = torquesSensor->getOldest();
            pacsim::msg::Wheels torquesMsg;
            torquesMsg.fl = torques.FL;
            torquesMsg.fr = torques.FR;
            torquesMsg.rl = torques.RL;
            torquesMsg.rr = torques.RR;
            torquesMsg.stamp = rclcpp::Time(static_cast<uint64_t>(torques.timestamp * 1e9));
            torquesPub->publish(torquesMsg);
        }

        std::vector<double> jointMsg = { steeringCurr.FL, orientation.FL, steeringCurr.FR, orientation.FR,
            orientation.RR, orientation.RL, -steeringWheelCurr };
        sensor_msgs::msg::JointState jointStamped = createRosJointMsg(jointNames, jointMsg, simTime);
        jointStatePublisher->publish(jointStamped);
        mutexSimTime.lock();
        simTime += timestep;
        mutexSimTime.unlock();
        nextLoopTime += std::chrono::microseconds((int)((timestep / realtimeRatio) * 1000000.0));
        std::this_thread::sleep_until(nextLoopTime);
    }
    Report report;
    cl->fillReport(report, simTime);
    reportToFile(report, report_file_dir);
    return 0;
}

void cbFuncLat(const pacsim::msg::StampedScalar& msg)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    deadTimeSteeringFront.addVal(msg.value, simTime);
    deadTimeSteeringRear.addVal(0.0, simTime);
}

void cbFuncTorquesInverterMin(const pacsim::msg::Wheels& msg)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    Wheels min;
    min.FL = msg.fl;
    min.FR = msg.fr;
    min.RL = msg.rl;
    min.RR = msg.rr;
    deadTimeMinTorques.addVal(min, simTime);
}

void cbFuncTorquesInverterMax(const pacsim::msg::Wheels& msg)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    Wheels max;
    max.FL = msg.fl;
    max.FR = msg.fr;
    max.RL = msg.rl;
    max.RR = msg.rr;
    deadTimeMaxTorques.addVal(max, simTime);
}

void cbWheelspeeds(const pacsim::msg::Wheels& msg)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    Wheels w { msg.fl, msg.fr, msg.rl, msg.rr };
    deadTimeWspdSetpoints.addVal(w, simTime);
}

void cbFinishSignal(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    cl->setFinish(true);
}

void getRos2Params(rclcpp::Node::SharedPtr& node)
{
    std::vector<std::pair<std::string, std::string*>> params;
    params.push_back({ "track_name", &trackName });
    params.push_back({ "track_frame", &trackFrame });
    params.push_back({ "report_file_dir", &report_file_dir });
    params.push_back({ "main_config_path", &main_config_path });
    params.push_back({ "perception_config_path", &perception_config_path });
    params.push_back({ "sensors_config_path", &sensors_config_path });
    params.push_back({ "vehicle_model_config_path", &vehicle_model_config_path });
    params.push_back({ "discipline", &discipline });

    for (auto p : params)
    {
        node->declare_parameter(std::string(p.first), rclcpp::PARAMETER_STRING);
        (*p.second) = node->get_parameter(std::string(p.first)).as_string();
    }

    node->declare_parameter("realtime_ratio", rclcpp::PARAMETER_DOUBLE);
    node->get_parameter("realtime_ratio", realtimeRatio);
}

void initPerceptionSensors()
{
    Config cfg(perception_config_path);
    auto perceptionSensorsConfig = cfg.getElement("perception_sensors");
    std::vector<ConfigElement> sensors;
    perceptionSensorsConfig.getElements(&sensors);
    for (auto& sensor : sensors)
    {
        std::shared_ptr<PerceptionSensor> perceptionSensor = std::make_shared<PerceptionSensor>();
        perceptionSensor->readConfig(sensor);
        perceptionSensors.push_back(perceptionSensor);
    }
    return;
}

void initSensors()
{
    Config cfg(sensors_config_path);
    auto sensorsConfig = cfg.getElement("sensors");

    auto imuConfigs = sensorsConfig.getElement("imus");
    std::vector<ConfigElement> sensors;
    imuConfigs.getElements(&sensors);
    for (auto& sensor : sensors)
    {
        std::shared_ptr<ImuSensor> imuSensor = std::make_shared<ImuSensor>(200.0, 0.002);
        imuSensor->readConfig(sensor);
        imus.push_back(imuSensor);
    }
    steeringSensorFront = std::make_shared<ScalarValueSensor>(200.0, 0.005);
    auto frontSteeringConfig = sensorsConfig.getElement("steering_front");
    steeringSensorFront->readConfig(frontSteeringConfig);
    steeringSensorRear = std::make_shared<ScalarValueSensor>(200.0, 0.005);
    auto rearSteeringConfig = sensorsConfig.getElement("steering_front");
    steeringSensorRear->readConfig(rearSteeringConfig);
    auto wheelSpeedConfig = sensorsConfig.getElement("wheelspeeds");
    wheelspeedSensor = std::make_shared<WheelsSensor>(200.0, 0.005);
    wheelspeedSensor->readConfig(wheelSpeedConfig);

    auto torquesConfig = sensorsConfig.getElement("wheelspeeds");
    torquesSensor = std::make_shared<WheelsSensor>(200.0, 0.005);
    torquesSensor->readConfig(torquesConfig);
}

MainConfig fillMainConfig(std::string path)
{
    MainConfig ret;
    Config cfg(path);
    ConfigElement config = cfg.getElement("pacsim");
    config["timeouts"].getElement<double>(&ret.timeout_start, "start");
    config["timeouts"].getElement<double>(&ret.timeout_acceleration, "acceleration");
    config["timeouts"].getElement<double>(&ret.timeout_autocross, "autocross");
    config["timeouts"].getElement<double>(&ret.timeout_skidpad, "skidpad");
    config["timeouts"].getElement<double>(&ret.timeout_trackdrive_first, "trackdrive_first");
    config["timeouts"].getElement<double>(&ret.timeout_trackdrive_total, "trackdrive_total");

    config.getElement<bool>(&ret.oc_detect, "oc_detect");
    config.getElement<bool>(&ret.doo_detect, "doo_detect");
    config.getElement<bool>(&ret.uss_detect, "uss_detect");
    config.getElement<bool>(&ret.finish_validate, "finish_validate");

    ret.discipline = stringToDiscipline(discipline);
    return ret;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pacsim_node");

    auto latsub = node->create_subscription<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 1, cbFuncLat);
    auto torquessubmin
        = node->create_subscription<pacsim::msg::Wheels>("/pacsim/torques_min", 1, cbFuncTorquesInverterMin);
    auto torquessubmax
        = node->create_subscription<pacsim::msg::Wheels>("/pacsim/torques_max", 1, cbFuncTorquesInverterMax);

    auto wspdSetpointSub
        = node->create_subscription<pacsim::msg::Wheels>("/pacsim/wheelspeed_setpoints", 1, cbWheelspeeds);

    velocity_pub = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/pacsim/velocity", 3);

    clockPub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    mapVizPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/pacsim/map", 1);

    auto finishSignalServer = node->create_service<std_srvs::srv::Empty>("/pacsim/finish_signal", cbFinishSignal);

    getRos2Params(node);
    mainConfig = fillMainConfig(main_config_path);
    initPerceptionSensors();
    initSensors();
    for (auto& i : perceptionSensors)
    {
        auto detectionsMarkersWrapper = std::make_shared<LandmarksMarkerWrapper>(
            0.0, 1.0, 0.0, 0.5, 0.5, 0.35, 0.8, 0.1, "pacsim/" + i->getName());
        auto pub = node->create_publisher<pacsim::msg::PerceptionDetections>(
            "/pacsim/perception/" + i->getName() + "/landmarks", 1);
        auto pubViz = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pacsim/perception/" + i->getName() + "/visualization", 1);
        lms_pubs.push_back(pub);
        perceptionSensorPublisherMap[i] = pub;
        perceptionSensorMarkersWrappersMap[i] = detectionsMarkersWrapper;
        perceptionSensorVizPublisherMap[i] = pubViz;
        pubViz->publish(detectionsMarkersWrapper->deleteAllMsg(i->getFrameId()));
    }
    for (auto& i : imus)
    {
        auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/pacsim/imu/" + i->getName(), 3);
        imuPublisherMap[i] = pub;
    }
    steeringFrontPub = node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steeringFront", 1);
    steeringRearPub = node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steeringRear", 1);
    wheelspeedPub = node->create_publisher<pacsim::msg::Wheels>("/pacsim/wheelspeeds", 1);
    torquesPub = node->create_publisher<pacsim::msg::Wheels>("/pacsim/torques", 1);

    jointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 3);

    model = std::make_shared<VehicleModelBicycle>();
    Config modelConfig(vehicle_model_config_path);
    auto configVehicleModel = modelConfig.getElement("vehicle_model");
    model->readConfig(configVehicleModel);

    std::thread mainLoopThread(threadMainLoopFunc, std::ref(node));
    RCLCPP_INFO_STREAM(node->get_logger(), "Started pacsim");
    rclcpp::spin(node);

    mainLoopThread.join();

    rclcpp::shutdown();

    return 0;
}
