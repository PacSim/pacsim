#include "rclcpp/rclcpp.hpp"
#include <sstream>

#include <chrono>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosgraph_msgs/msg/clock.h"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "VehicleModel/VehicleModelBicycle.cpp"
#include "configParser.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "track/trackLoader.hpp"

#include "logger.hpp"
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
#include "pacsim/srv/clock_trigger_absolute.hpp"
#include "pacsim/srv/clock_trigger_relative.hpp"
#include "types.hpp"

#include "competitionLogic.hpp"

#include "std_srvs/srv/empty.hpp"

#include "reportWriter.hpp"

#include "sensorModels/gnssSensor.hpp"

#include "track/gripMap.hpp"

// DynamicDoubleTrackModel7Dof model;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
std::shared_ptr<IVehicleModel> model;
std::string mapFilePrefix;
double simTime = 0.0;
rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub;
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mapVizPub;
rclcpp::Publisher<pacsim::msg::Track>::SharedPtr trackPub;
rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steeringFrontPub;
rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steeringRearPub;
rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr wheelspeedPub;
rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torquesPub;
rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr voltageSensorTSPub;
rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr currentSensorTSPub;

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher;

std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr perceptionVizPub;
std::vector<rclcpp::Publisher<pacsim::msg::PerceptionDetections>::SharedPtr> lms_pubs;
std::map<std::shared_ptr<PerceptionSensor>, rclcpp::Publisher<pacsim::msg::PerceptionDetections>::SharedPtr>
    perceptionSensorPublisherMap;
std::map<std::shared_ptr<PerceptionSensor>, std::shared_ptr<LandmarksMarkerWrapper>> perceptionSensorMarkersWrappersMap;
std::map<std::shared_ptr<PerceptionSensor>, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>
    perceptionSensorVizPublisherMap;
std::map<std::shared_ptr<ImuSensor>, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imuPublisherMap;
std::map<std::shared_ptr<GnssSensor>, rclcpp::Publisher<pacsim::msg::GNSS>::SharedPtr> gnssPublisherMap;

DeadTime<double> deadTimeSteeringFront(0.0);
DeadTime<double> deadTimeSteeringRear(0.0);
DeadTime<Wheels> deadTimeTorques(0.0);
DeadTime<Wheels> deadTimeWspdSetpoints(0.0);
DeadTime<Wheels> deadTimeMaxTorques(0.0);
DeadTime<Wheels> deadTimeMinTorques(0.0);
DeadTime<double> deadTimePowerGroundSetpoint(0.0);

std::mutex mutexSimTime;

std::string trackName;
std::string grip_map_path;
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
std::vector<std::shared_ptr<GnssSensor>> gnssSensors;
std::shared_ptr<CompetitionLogic> cl;

std::shared_ptr<ImuSensor> imuSensor;
std::shared_ptr<ScalarValueSensor> steeringSensorFront;
std::shared_ptr<ScalarValueSensor> steeringSensorRear;
std::shared_ptr<WheelsSensor> wheelspeedSensor;
std::shared_ptr<WheelsSensor> torquesSensor;
std::shared_ptr<ScalarValueSensor> currentSensorTS;
std::shared_ptr<ScalarValueSensor> voltageSensorTS;

std::shared_ptr<Logger> logger;

std::condition_variable cvClockTrigger;
double clockStopTime = std::numeric_limits<double>::max();

int threadMainLoopFunc(std::shared_ptr<rclcpp::Node> node)
{

    std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(node);

    double timestep = 1.0 / 1000.0;
    double egoMotionSensorRate = 200.0;
    double lastEgoMotionSensorSampleTime = 0.0;
    std::string framePerception = "perception";
    std::string frameStateEstimation = "cog_static";

    Eigen::Vector3d start_position;
    Eigen::Vector3d start_orientation;
    Track lms = loadMap(trackName, start_position, start_orientation);

    model->setPosition(start_position);
    model->setOrientation(start_orientation);

    gripMap gm(logger);
    gm.loadConfig(grip_map_path);

    if (mainConfig.pre_transform_track)
    {
        lms = transformTrack(lms, start_position, start_orientation);
        model->setPosition(Eigen::Vector3d::Zero());
        model->setOrientation(Eigen::Vector3d::Zero());
        gm.transformPoints(start_position, start_orientation);
        start_position = Eigen::Vector3d::Zero();
        start_orientation = Eigen::Vector3d::Zero();
    }

    LandmarkList trackAsLMList = trackToLMList(lms);

    LandmarksMarkerWrapper mapMarkersWrapper(0.8, "pacsim");

    visualization_msgs::msg::MarkerArray mapMarkerMsg = mapMarkersWrapper.markerFromLMs(lms, trackFrame, 0.0);
    mapVizPub->publish(mapMarkerMsg);
    trackPub->publish(createRosTrackMessage(lms, "map", 0.0));

    deadTimeSteeringFront = DeadTime<double>(0.05);
    deadTimeSteeringRear = DeadTime<double>(0.05);
    deadTimeTorques = DeadTime<Wheels>(0.02);
    deadTimeWspdSetpoints = DeadTime<Wheels>(0.02);
    deadTimeMaxTorques = DeadTime<Wheels>(0.02);
    deadTimeMinTorques = DeadTime<Wheels>(0.02);
    deadTimePowerGroundSetpoint = DeadTime<double>(0.05);

    double current_wheel_speed_angle = 0.0;

    auto nextLoopTime = std::chrono::steady_clock::now();

    cl = std::make_shared<CompetitionLogic>(logger, lms, mainConfig);
    bool finish = false;
    std::mutex mtxClockTrigger;
    std::unique_lock<std::mutex> lockClockTrigger(mtxClockTrigger);

    while (rclcpp::ok() && !(finish))
    {
        rosgraph_msgs::msg::Clock clockMsg;
        clockMsg.clock = rclcpp::Time(static_cast<uint64_t>(simTime * 1e9));
        clockPub->publish(clockMsg);
        auto wheelPositions = model->getWheelPositions();
        Wheels gripValues = gm.getGripValues(wheelPositions);
        model->forwardIntegrate(timestep, gripValues);
        auto t = model->getPosition();
        auto rEulerAngles = model->getOrientation();
        auto alpha = model->getAngularAcceleration();
        finish = cl->performAllChecks(lms, simTime, t, rEulerAngles);
        // geometry_msgs::msg::TransformStamped static_transform = createStaticTransform("map", "center", simTime);
        geometry_msgs::msg::TransformStamped transformStamped
            = createRosTransformMsg(t, rEulerAngles, trackFrame, "car", simTime);

        br->sendTransform(transformStamped);

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
        if (deadTimePowerGroundSetpoint.availableDeadTime(simTime))
        {
            double val = deadTimePowerGroundSetpoint.getOldest();
            model->setPowerGroundSetpoint(val);
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

        double voltageTsCurr = model->getVoltageTS();
        StampedScalar voltageTSData { voltageTsCurr, simTime };
        if (voltageSensorTS->RunTick(voltageTSData, simTime))
        {
            StampedScalar voltageData = voltageSensorTS->getOldest();
            pacsim::msg::StampedScalar msg;
            msg.value = voltageData.data;
            msg.stamp = rclcpp::Time(static_cast<uint64_t>(voltageData.timestamp * 1e9));
            voltageSensorTSPub->publish(msg);
        }

        double currentTsCurr = model->getCurrentTS();
        StampedScalar currentTSData { currentTsCurr, simTime };
        if (currentSensorTS->RunTick(currentTSData, simTime))
        {
            StampedScalar currentData = currentSensorTS->getOldest();
            pacsim::msg::StampedScalar msg;
            msg.value = currentData.data;
            msg.stamp = rclcpp::Time(static_cast<uint64_t>(currentData.timestamp * 1e9));
            currentSensorTSPub->publish(msg);
        }

        for (auto& gnss : gnssSensors)
        {

            if (gnss->RunTick(lms.gnssOrigin, lms.enuToTrackRotation, t, rEulerAngles, simTime, model->getVelocity(),
                    model->getAngularVelocity(), start_position, start_orientation, mainConfig.pre_transform_track))
            {
                auto gnssData = gnss->getOldest();
                auto gnssMsg = createRosGnssMessage(gnssData);
                gnssPublisherMap[gnss]->publish(gnssMsg);
            }
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
                trackPub->publish(createRosTrackMessage(lms, "map", simTime));
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
        if (simTime >= clockStopTime)
        {
            cvClockTrigger.wait(lockClockTrigger);
            nextLoopTime = std::chrono::steady_clock::now();
        }
        nextLoopTime += std::chrono::microseconds((int)((timestep / realtimeRatio) * 1000000.0));
        std::this_thread::sleep_until(nextLoopTime);
    }
    Report report;
    cl->fillReport(report, simTime);
    report.track_name = trackName;
    reportToFile(report, report_file_dir);
    executor->cancel();
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

void cbPowerGround(const pacsim::msg::StampedScalar& msg)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    deadTimePowerGroundSetpoint.addVal(msg.value, simTime);
}

void cbFinishSignal(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    cl->setFinish(true);
}

void cbClockTriggerAbsolute(const std::shared_ptr<pacsim::srv::ClockTriggerAbsolute::Request> request,
    std::shared_ptr<pacsim::srv::ClockTriggerAbsolute::Response> response)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    clockStopTime = rclcpp::Time(request->stop_time).seconds();
    response->already_past = (clockStopTime < simTime);
    cvClockTrigger.notify_all();
}

void cbClockTriggerRelative(const std::shared_ptr<pacsim::srv::ClockTriggerRelative::Request> request,
    std::shared_ptr<pacsim::srv::ClockTriggerRelative::Response> response)
{
    std::lock_guard<std::mutex> l(mutexSimTime);
    clockStopTime = simTime + rclcpp::Duration(request->runtime).seconds();
    response->stop_time = rclcpp::Time(static_cast<uint64_t>(clockStopTime * 1e9));
    cvClockTrigger.notify_all();
}
void getRos2Params(rclcpp::Node::SharedPtr& node)
{
    std::vector<std::pair<std::string, std::string*>> params;
    params.push_back({ "track_name", &trackName });
    params.push_back({ "grip_map_path", &grip_map_path });
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

    auto gnssConfigs = sensorsConfig.getElement("gnssSensors");
    std::vector<ConfigElement> gnssSensorConfigs;
    gnssConfigs.getElements(&gnssSensorConfigs);
    for (auto& sensor : gnssSensorConfigs)
    {
        std::shared_ptr<GnssSensor> gnssSensor = std::make_shared<GnssSensor>();
        gnssSensor->readConfig(sensor);
        gnssSensors.push_back(gnssSensor);
    }

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
    auto rearSteeringConfig = sensorsConfig.getElement("steering_rear");
    steeringSensorRear->readConfig(rearSteeringConfig);
    auto wheelSpeedConfig = sensorsConfig.getElement("wheelspeeds");
    wheelspeedSensor = std::make_shared<WheelsSensor>(200.0, 0.005);
    wheelspeedSensor->readConfig(wheelSpeedConfig);

    voltageSensorTS = std::make_shared<ScalarValueSensor>(200.0, 0.005);
    auto voltageTSConfig = sensorsConfig.getElement("voltage_ts");
    voltageSensorTS->readConfig(voltageTSConfig);
    currentSensorTS = std::make_shared<ScalarValueSensor>(200.0, 0.005);
    auto currentTSConfig = sensorsConfig.getElement("current_ts");
    currentSensorTS->readConfig(currentTSConfig);

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

    config.getElement<std::string>(&ret.cog_frame_id_pipeline, "cog_frame_id_pipeline");
    config.getElement<bool>(&ret.broadcast_sensors_tf2, "broadcast_sensors_tf2");

    config.getElement<bool>(&ret.oc_detect, "oc_detect");
    config.getElement<bool>(&ret.doo_detect, "doo_detect");
    config.getElement<bool>(&ret.uss_detect, "uss_detect");
    config.getElement<bool>(&ret.finish_validate, "finish_validate");

    config.getElement<bool>(&ret.pre_transform_track, "pre_transform_track");

    ret.discipline = stringToDiscipline(discipline);
    return ret;
}

void handleTf2StaticTransforms()
{
    if (mainConfig.broadcast_sensors_tf2)
    {

        for (auto sensor : perceptionSensors)
        {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = rclcpp::Time(0, 0);
            t.header.frame_id = mainConfig.cog_frame_id_pipeline;
            t.child_frame_id = sensor->getFrameId();

            auto translation = sensor->getPosition();
            auto orientation = sensor->getOrientation();
            t.transform.translation.x = translation.x();
            t.transform.translation.y = translation.y();
            t.transform.translation.z = translation.z();
            tf2::Quaternion q;
            q.setRPY(orientation.x(), orientation.y(), orientation.z());
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_static_broadcaster_->sendTransform(t);
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pacsim_node");
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    logger = std::make_shared<Logger>();

    auto latsub = node->create_subscription<pacsim::msg::StampedScalar>("/pacsim/steering_setpoint", 1, cbFuncLat);
    auto torquessubmin
        = node->create_subscription<pacsim::msg::Wheels>("/pacsim/torques_min", 1, cbFuncTorquesInverterMin);
    auto torquessubmax
        = node->create_subscription<pacsim::msg::Wheels>("/pacsim/torques_max", 1, cbFuncTorquesInverterMax);

    auto wspdSetpointSub
        = node->create_subscription<pacsim::msg::Wheels>("/pacsim/wheelspeed_setpoints", 1, cbWheelspeeds);

    auto powerGroundSetpointSub
        = node->create_subscription<pacsim::msg::StampedScalar>("/pacsim/powerground_setpoint", 1, cbPowerGround);

    velocity_pub = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/pacsim/velocity", 3);

    clockPub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    mapVizPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/pacsim/track/visualization", 1);

    trackPub = node->create_publisher<pacsim::msg::Track>("/pacsim/track/landmarks", 1);

    auto finishSignalServer = node->create_service<std_srvs::srv::Empty>("/pacsim/finish_signal", cbFinishSignal);
    auto clockTriggerAbsoluteServer = node->create_service<pacsim::srv::ClockTriggerAbsolute>(
        "/pacsim/clock_trigger/absolute", cbClockTriggerAbsolute);
    auto clockTriggerRelativeServer = node->create_service<pacsim::srv::ClockTriggerRelative>(
        "/pacsim/clock_trigger/relative", cbClockTriggerRelative);

    getRos2Params(node);
    mainConfig = fillMainConfig(main_config_path);
    initPerceptionSensors();
    initSensors();
    handleTf2StaticTransforms();
    for (auto& i : perceptionSensors)
    {
        auto detectionsMarkersWrapper = std::make_shared<LandmarksMarkerWrapper>(0.8, "pacsim/" + i->getName());
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
    for (auto& i : gnssSensors)
    {
        auto pub = node->create_publisher<pacsim::msg::GNSS>("/pacsim/gnss/" + i->getName(), 3);
        gnssPublisherMap[i] = pub;
    }

    steeringFrontPub = node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steeringFront", 1);
    steeringRearPub = node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steeringRear", 1);
    voltageSensorTSPub = node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/ts/voltage", 1);
    currentSensorTSPub = node->create_publisher<pacsim::msg::StampedScalar>("/pacsim/ts/current", 1);

    wheelspeedPub = node->create_publisher<pacsim::msg::Wheels>("/pacsim/wheelspeeds", 1);
    torquesPub = node->create_publisher<pacsim::msg::Wheels>("/pacsim/torques", 1);

    jointStatePublisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 3);

    model = std::make_shared<VehicleModelBicycle>();
    Config modelConfig(vehicle_model_config_path);
    auto configVehicleModel = modelConfig.getElement("vehicle_model");
    model->readConfig(configVehicleModel);

    std::thread mainLoopThread(threadMainLoopFunc, std::ref(node));
    logger->logInfo("Started pacsim");
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    executor->spin();

    mainLoopThread.join();

    rclcpp::shutdown();

    return 0;
}
