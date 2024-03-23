#ifndef PACSIMROS2HELPERS_HPP
#define PACSIMROS2HELPERS_HPP

#include "rclcpp/rclcpp.hpp"

#include "pacsim/msg/perception_detections.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "types.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <string>
#include <tf2/LinearMath/Quaternion.h>

class LandmarksMarkerWrapper
{
public:
    LandmarksMarkerWrapper(double r, double g, double b, double sx, double sy, double sz, double alpha, double offsetZ,
        std::string nameSpace);

    visualization_msgs::msg::MarkerArray markerFromLMs(Track& in, std::string frame, double time);

    visualization_msgs::msg::MarkerArray deleteAllMsg(std::string frame);

private:
    double r = 0.0;
    double g = 1.0;
    double b = 0.0;
    double sx = 0.3;
    double sy = 0.3;
    double sz = 0.5;
    double alpha = 0.3;
    double offsetZ = 0.1;
    std::string nameSpace = "pacsim";
    int lastMaxId;
};

pacsim::msg::PerceptionDetections LandmarkListToRosMessage(
    const LandmarkList& sensorLms, std::string frameId, double time);

sensor_msgs::msg::Imu createRosImuMsg(const ImuData& data);

geometry_msgs::msg::TwistWithCovarianceStamped createRosTwistMsg(
    const Eigen::Vector3d& vel, const Eigen::Vector3d& rot, const std::string& frame, double time);

geometry_msgs::msg::TransformStamped createRosTransformMsg(const Eigen::Vector3d& trans, const Eigen::Vector3d& rot,
    const std::string& frame, const std::string& child_frame, double time);

geometry_msgs::msg::TransformStamped createStaticTransform(
    const std::string& frame, const std::string& child_frame, double time);

sensor_msgs::msg::JointState createRosJointMsg(
    const std::vector<std::string>& joint_names, const std::vector<double>& joint_vals, double time);

#endif /* PACSIMROS2HELPERS_HPP */