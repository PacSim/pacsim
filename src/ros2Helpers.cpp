#include "ros2Helpers.hpp"

LandmarksMarkerWrapper::LandmarksMarkerWrapper(
    double r, double g, double b, double sx, double sy, double sz, double alpha, double offsetZ, std::string nameSpace)
{
    this->r = r;
    this->g = g;
    this->b = b;
    this->sx = sx;
    this->sy = sy;
    this->sz = sz;
    this->alpha = alpha;
    this->offsetZ = offsetZ;
    this->nameSpace = nameSpace;
    this->lastMaxId = 0;
}
visualization_msgs::msg::MarkerArray LandmarksMarkerWrapper::markerFromLMs(Track& in, std::string frame, double time)
{
    visualization_msgs::msg::MarkerArray out;
    int id = 0;
    rclcpp::Time stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    // TODO: no copy for lists
    std::vector<std::vector<Landmark>> lists { in.left_lane, in.right_lane, in.unknown };
    for (auto& list : lists)
    {
        for (Landmark lm : list)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame;
            marker.header.stamp = stamp;
            marker.ns = this->nameSpace;
            marker.id = id;
            marker.frame_locked = false;
            id += 1;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::MODIFY;
            marker.pose.position.x = lm.position.x();
            marker.pose.position.y = lm.position.y();
            marker.pose.position.z = lm.position.z() + this->offsetZ;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = this->sx;
            marker.scale.y = this->sy;
            marker.scale.z = this->sz;
            marker.color.a = this->alpha;
            marker.color.r = this->r;
            marker.color.g = this->g;
            marker.color.b = this->b;
            out.markers.push_back(marker);
        }
    }

    for (int i = id; i < lastMaxId; ++i)
    {
        // visualization_msgs::MarkerArray out;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = stamp;
        marker.ns = this->nameSpace;
        marker.id = i;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        out.markers.push_back(marker);
    }
    lastMaxId = id;
    return out;
}

visualization_msgs::msg::MarkerArray LandmarksMarkerWrapper::deleteAllMsg(std::string frame)
{
    visualization_msgs::msg::MarkerArray out;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = rclcpp::Time(static_cast<uint64_t>(0));
    marker.ns = this->nameSpace;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(marker);
    return out;
}

efr_pacsim::msg::PerceptionDetections LandmarkListToRosMessage(
    const LandmarkList& sensorLms, std::string frameId, double time)
{
    efr_pacsim::msg::PerceptionDetections lmsMsg;
    lmsMsg.header.frame_id = frameId;
    lmsMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    ;
    for (Landmark lm : sensorLms.list)
    {
        efr_pacsim::msg::PerceptionDetection lmMsg;
        lmMsg.header = lmsMsg.header;
        lmMsg.pose.pose.position.x = lm.position.x();
        lmMsg.pose.pose.position.y = lm.position.y();
        lmMsg.pose.pose.position.z = lm.position.z();
        lmMsg.pose.covariance[0] = lm.cov(0, 0);
        lmMsg.pose.covariance[1] = lm.cov(0, 1);
        lmMsg.pose.covariance[2] = lm.cov(0, 2);
        lmMsg.pose.covariance[3 + 3] = lm.cov(1, 0);
        lmMsg.pose.covariance[4 + 3] = lm.cov(1, 1);
        lmMsg.pose.covariance[5 + 3] = lm.cov(1, 2);
        lmMsg.pose.covariance[6 + 6] = lm.cov(2, 0);
        lmMsg.pose.covariance[7 + 6] = lm.cov(2, 1);
        lmMsg.pose.covariance[8 + 6] = lm.cov(2, 2);
        // orientation unknown
        lmMsg.pose.covariance[3 + 3 * 6] = -1;
        lmMsg.pose.covariance[4 + 4 * 6] = -1;
        lmMsg.pose.covariance[4 + 5 * 6] = -1;
        lmsMsg.detections.push_back(lmMsg);
    }
    return lmsMsg;
}

sensor_msgs::msg::Imu createRosImuMsg(
    const ImuData& data)
{
    sensor_msgs::msg::Imu imuMsg;
    imuMsg.linear_acceleration.x = data.acc.x();
    imuMsg.linear_acceleration.y = data.acc.y();
    imuMsg.linear_acceleration.z = data.acc.z();

    imuMsg.angular_velocity.x = data.rot.x();
    imuMsg.angular_velocity.y = data.rot.y();
    imuMsg.angular_velocity.z = data.rot.z();

    imuMsg.orientation_covariance[0] = -1;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j) {
          imuMsg.angular_velocity_covariance[j+i*3] = data.rot_cov(i,j);
          imuMsg.linear_acceleration_covariance[j+i*3] = data.acc_cov(i,j);
      }
    }

    imuMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(data.timestamp * 1e9));
    imuMsg.header.frame_id = data.frame;
    return imuMsg;
}

geometry_msgs::msg::TwistWithCovarianceStamped createRosTwistMsg(
    const Eigen::Vector3d& vel, const Eigen::Vector3d& rot, const std::string& frame, double time)
{
    geometry_msgs::msg::TwistWithCovarianceStamped velMsg;
    velMsg.twist.twist.linear.x = vel.x();
    velMsg.twist.twist.linear.y = vel.y();
    velMsg.twist.twist.linear.z = vel.z();

    velMsg.twist.twist.angular.x = rot.x();
    velMsg.twist.twist.angular.y = rot.y();
    velMsg.twist.twist.angular.z = rot.z();

    velMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    ;
    velMsg.header.frame_id = frame;
    return velMsg;
}

geometry_msgs::msg::TransformStamped createRosTransformMsg(const Eigen::Vector3d& trans, const Eigen::Vector3d& rot,
    const std::string& frame, const std::string& child_frame, double time)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    ;
    transformStamped.header.frame_id = frame;
    transformStamped.child_frame_id = child_frame;

    transformStamped.transform.translation.x = trans.x();
    transformStamped.transform.translation.y = trans.y();
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, rot.z());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    return transformStamped;
}
geometry_msgs::msg::TransformStamped createStaticTransform(
    const std::string& frame, const std::string& child_frame, double time)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    ;
    transformStamped.header.frame_id = frame;
    transformStamped.child_frame_id = child_frame;

    return transformStamped;
}
sensor_msgs::msg::JointState createRosJointMsg(
    const std::vector<std::string>& joint_names, const std::vector<double>& joint_vals, double time)
{
    sensor_msgs::msg::JointState jointStamped;
    jointStamped.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    jointStamped.name = joint_names;

    jointStamped.position = joint_vals;

    return jointStamped;
}