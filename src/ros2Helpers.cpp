#include "ros2Helpers.hpp"

LandmarksMarkerWrapper::LandmarksMarkerWrapper(double alpha, std::string nameSpace)
{
    this->alpha = alpha;
    this->nameSpace = nameSpace;
    this->lastMaxId = 0;
}
visualization_msgs::msg::MarkerArray LandmarksMarkerWrapper::markerFromLMs(Track& in, std::string frame, double time)
{
    visualization_msgs::msg::MarkerArray out;
    int id = 0;
    rclcpp::Time stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    // TODO: no copy for lists
    std::vector<Landmark> timekeepingLms;
    for (auto& pair : in.time_keeping_gates)
    {
        timekeepingLms.push_back(pair.first);
        timekeepingLms.push_back(pair.second);
    }
    std::vector<std::vector<Landmark>> lists { in.left_lane, in.right_lane, in.unknown, timekeepingLms };
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
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.228;
            marker.scale.y = 0.228;
            marker.scale.z = 0.325;
            marker.color.a = this->alpha;
            marker.color.r = 0.0 / 255.0;
            marker.color.g = 153.0 / 255.0;
            marker.color.b = 51.0 / 255.0;
            switch (lm.type)
            {
            case LandmarkType::BLUE:
                marker.color.r = 51.0 / 255.0;
                marker.color.g = 102.0 / 255.0;
                marker.color.b = 255.0 / 255.0;
                break;
            case LandmarkType::YELLOW:
                marker.color.r = 255.0 / 255.0;
                marker.color.g = 255.0 / 255.0;
                marker.color.b = 0.0 / 255.0;
                break;
            case LandmarkType::ORANGE:
                marker.color.r = 255.0 / 255.0;
                marker.color.g = 153.0 / 255.0;
                marker.color.b = 0.0 / 255.0;
                break;
            case LandmarkType::BIG_ORANGE:
                marker.color.r = 255.0 / 255.0;
                marker.color.g = 153.0 / 255.0;
                marker.color.b = 0.0 / 255.0;
                marker.scale.x = 0.285;
                marker.scale.y = 0.285;
                marker.scale.z = 0.5;
                break;
            case LandmarkType::TIMEKEEPING:
                marker.color.r = 204.0 / 255.0;
                marker.color.g = 51.0 / 255.0;
                marker.color.b = 153.0 / 255.0;
                marker.scale.x = 0.285;
                marker.scale.y = 0.285;
                marker.scale.z = 0.5;
                break;
            case LandmarkType::INVISIBLE:
                marker.color.r = 99.0 / 255.0;
                marker.color.g = 100.0 / 255.0;
                marker.color.b = 102.0 / 255.0;
                break;
            case LandmarkType::UNKNOWN:
                marker.color.r = 0.0 / 255.0;
                marker.color.g = 153.0 / 255.0;
                marker.color.b = 51.0 / 255.0;
                break;
            }

            marker.pose.position.z = lm.position.z() + marker.scale.z * 0.5;
            out.markers.push_back(marker);
        }
    }
    // draw lines timekeeping
    for (int i = 0; i < in.time_keeping_gates.size(); ++i)
    {
        auto line = in.time_keeping_gates[i];
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = stamp;
        marker.ns = this->nameSpace;
        marker.id = id;
        marker.frame_locked = false;
        id += 1;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::MODIFY;
        geometry_msgs::msg::Point p1;
        p1.x = line.first.position.x();
        p1.y = line.first.position.y();
        p1.z = line.first.position.z();
        marker.points.push_back(p1);
        geometry_msgs::msg::Point p2;
        p2.x = line.second.position.x();
        p2.y = line.second.position.y();
        p2.z = line.second.position.z();
        marker.points.push_back(p2);
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
        marker.color.a = this->alpha;
        marker.color.r = 153.0 / 255.0;
        marker.color.g = 0.0 / 255.0;
        marker.color.b = 153.0 / 255.0;
        if (i == 0)
        {
            // green
            marker.color.r = 0.0 / 255.0;
            marker.color.g = 102.0 / 255.0;
            marker.color.b = 0.0 / 255.0;
        }
        else if (i == 1 && (!in.lanesFirstWithLastConnected))
        {
            // red
            marker.color.r = 128.0 / 255.0;
            marker.color.g = 0.0 / 255.0;
            marker.color.b = 0.0 / 255.0;
        }
        out.markers.push_back(marker);
    }

    // draw lines left lane
    visualization_msgs::msg::Marker markerLeftLine;
    markerLeftLine.header.frame_id = frame;
    markerLeftLine.header.stamp = stamp;
    markerLeftLine.ns = this->nameSpace;
    markerLeftLine.id = id;
    markerLeftLine.frame_locked = false;
    id += 1;
    markerLeftLine.type = visualization_msgs::msg::Marker::LINE_STRIP;
    markerLeftLine.action = visualization_msgs::msg::Marker::MODIFY;
    markerLeftLine.pose.position.x = 0.0;
    markerLeftLine.pose.position.y = 0.0;
    markerLeftLine.pose.position.z = 0.0;
    markerLeftLine.pose.orientation.x = 0.0;
    markerLeftLine.pose.orientation.y = 0.0;
    markerLeftLine.pose.orientation.z = 0.0;
    markerLeftLine.pose.orientation.w = 1.0;
    markerLeftLine.scale.x = 0.05;
    markerLeftLine.scale.y = 0.0;
    markerLeftLine.scale.z = 0.0;
    markerLeftLine.color.a = this->alpha;
    markerLeftLine.color.r = 51.0 / 255.0;
    markerLeftLine.color.g = 102.0 / 255.0;
    markerLeftLine.color.b = 255.0 / 255.0;
    for (auto& p : in.left_lane)
    {
        geometry_msgs::msg::Point point;
        point.x = p.position.x();
        point.y = p.position.y();
        point.z = p.position.z() + 0.5 * markerLeftLine.scale.z;
        markerLeftLine.points.push_back(point);
    }
    if (in.lanesFirstWithLastConnected && (in.left_lane.size() >= 1))
    {
        geometry_msgs::msg::Point point;
        auto p = in.left_lane[0];
        point.x = p.position.x();
        point.y = p.position.y();
        point.z = p.position.z() + 0.5 * markerLeftLine.scale.z;
        markerLeftLine.points.push_back(point);
    }
    out.markers.push_back(markerLeftLine);

    // draw lines right lane
    visualization_msgs::msg::Marker markerRightLane;
    markerRightLane.header.frame_id = frame;
    markerRightLane.header.stamp = stamp;
    markerRightLane.ns = this->nameSpace;
    markerRightLane.id = id;
    markerRightLane.frame_locked = false;
    id += 1;
    markerRightLane.type = visualization_msgs::msg::Marker::LINE_STRIP;
    markerRightLane.action = visualization_msgs::msg::Marker::MODIFY;
    markerRightLane.pose.position.x = 0.0;
    markerRightLane.pose.position.y = 0.0;
    markerRightLane.pose.position.z = 0.0;
    markerRightLane.pose.orientation.x = 0.0;
    markerRightLane.pose.orientation.y = 0.0;
    markerRightLane.pose.orientation.z = 0.0;
    markerRightLane.pose.orientation.w = 1.0;
    markerRightLane.scale.x = 0.05;
    markerRightLane.scale.y = 0.0;
    markerRightLane.scale.z = 0.0;
    markerRightLane.color.a = this->alpha;
    markerRightLane.color.r = 255.0 / 255.0;
    markerRightLane.color.g = 255.0 / 255.0;
    markerRightLane.color.b = 0.0 / 255.0;
    for (auto& p : in.right_lane)
    {
        geometry_msgs::msg::Point point;
        point.x = p.position.x();
        point.y = p.position.y();
        point.z = p.position.z() + 0.5 * markerRightLane.scale.z;
        markerRightLane.points.push_back(point);
    }
    if (in.lanesFirstWithLastConnected && (in.right_lane.size() >= 1))
    {
        geometry_msgs::msg::Point point;
        auto p = in.right_lane[0];
        point.x = p.position.x();
        point.y = p.position.y();
        point.z = p.position.z() + 0.5 * markerRightLane.scale.z;
        markerRightLane.points.push_back(point);
    }
    out.markers.push_back(markerRightLane);

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

pacsim::msg::PerceptionDetections LandmarkListToRosMessage(
    const LandmarkList& sensorLms, std::string frameId, double time)
{
    pacsim::msg::PerceptionDetections lmsMsg;
    lmsMsg.header.frame_id = frameId;
    lmsMsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(time * 1e9));
    ;
    for (Landmark lm : sensorLms.list)
    {
        pacsim::msg::PerceptionDetection lmMsg;
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

        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_UNKNOWN]
            = lm.typeWeights[LandmarkType::UNKNOWN];
        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_BLUE] = lm.typeWeights[LandmarkType::BLUE];
        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_YELLOW]
            = lm.typeWeights[LandmarkType::YELLOW];
        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_ORANGE]
            = lm.typeWeights[LandmarkType::ORANGE];
        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_BIGORANGE]
            = lm.typeWeights[LandmarkType::BIG_ORANGE];
        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_TIMEKEEPING]
            = lm.typeWeights[LandmarkType::TIMEKEEPING];
        lmMsg.class_probabilities[pacsim::msg::PerceptionDetection::CLASS_INVISIBLE]
            = lm.typeWeights[LandmarkType::INVISIBLE];

        lmMsg.detection_probability = lm.detection_probability;
        lmMsg.id = lm.id;
        lmsMsg.detections.push_back(lmMsg);
    }
    return lmsMsg;
}

sensor_msgs::msg::Imu createRosImuMsg(const ImuData& data)
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
        for (int j = 0; j < 3; ++j)
        {
            imuMsg.angular_velocity_covariance[j + i * 3] = data.rot_cov(i, j);
            imuMsg.linear_acceleration_covariance[j + i * 3] = data.acc_cov(i, j);
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

pacsim::msg::GNSS createRosGnssMessage(const GnssData& data)
{
    pacsim::msg::GNSS msg;
    msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(data.timestamp * 1e9));
    msg.status = data.fix_status;
    msg.latitude = data.latitude;
    msg.longitude = data.longitude;
    msg.altitude = data.altitude;

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            msg.position_covariance[3 * i + j] = data.position_covariance(i, j);
            msg.velocity_covariance[3 * i + j] = data.velocity_covariance(i, j);
            msg.orientation_covariance[3 * i + j] = data.orientation_covariance(i, j);
        }
    }

    msg.velocity_east = data.vel_east;
    msg.velocity_north = data.vel_north;
    msg.velocity_up = data.vel_up;

    msg.orientation.w = data.orientation.w;
    msg.orientation.x = data.orientation.x;
    msg.orientation.y = data.orientation.y;
    msg.orientation.z = data.orientation.z;

    return msg;
}