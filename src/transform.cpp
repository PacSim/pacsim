#include "transform.hpp"

Eigen::Vector3d rigidBodyVelocity(Eigen::Vector3d v, Eigen::Vector3d r, Eigen::Vector3d omega)
{
    return (v + omega.cross(r));
}

Eigen::Vector3d rigidBodyAcceleration(
    Eigen::Vector3d a, Eigen::Vector3d r, Eigen::Vector3d omega, Eigen::Vector3d alpha)
{
    return (a + omega.cross(omega.cross(r)) + alpha.cross(r));
}

Eigen::Matrix3d eulerAnglesToRotMat(Eigen::Vector3d& angles)
{
    Eigen::AngleAxisd rollAngle(-angles.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(-angles.z(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(-angles.y(), Eigen::Vector3d::UnitY());

    // rotation order matches with tf2
    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    return rotationMatrix;
}

Eigen::Vector3d inverseTranslation(Eigen::Vector3d trans, Eigen::Vector3d rot)
{
    Eigen::Matrix3d rotationMatrix = eulerAnglesToRotMat(rot);

    return (-rotationMatrix * trans);
}

LandmarkList transformLmList(LandmarkList& in, Eigen::Vector3d trans, Eigen::Vector3d rot)
{
    Eigen::Matrix3d rotationMatrix = eulerAnglesToRotMat(rot);
    Eigen::Vector3d transInverse = inverseTranslation(trans, rot);

    LandmarkList out;
    for (auto& lm : in.list)
    {
        Landmark temp = lm;
        temp.position = rotationMatrix * lm.position;
        temp.position += transInverse;
        temp.id = lm.id;
        out.list.push_back(temp);
    }

    return out;
}

Track transformTrack(Track& in, Eigen::Vector3d trans, Eigen::Vector3d rot)
{
    Eigen::Matrix3d rotationMatrix = eulerAnglesToRotMat(rot);
    Eigen::Vector3d transInverse = inverseTranslation(trans, rot);

    Track out = in;
    out.lanesFirstWithLastConnected = in.lanesFirstWithLastConnected;
    out.left_lane.clear();
    for (auto& lm : in.left_lane)
    {
        Landmark temp = lm;
        temp.position = rotationMatrix * lm.position;
        temp.position += transInverse;
        temp.id = lm.id;
        out.left_lane.push_back(temp);
    }
    out.right_lane.clear();
    for (auto& lm : in.right_lane)
    {
        Landmark temp = lm;
        temp.position = rotationMatrix * lm.position;
        temp.position += transInverse;
        temp.id = lm.id;
        out.right_lane.push_back(temp);
    }
    out.unknown.clear();
    for (auto& lm : in.unknown)
    {
        Landmark temp = lm;
        temp.position = rotationMatrix * lm.position;
        temp.position += transInverse;
        temp.id = lm.id;
        out.unknown.push_back(temp);
    }

    out.time_keeping_gates.clear();
    for (auto& gate : in.time_keeping_gates)
    {
        Landmark temp1 = gate.first;
        temp1.position = rotationMatrix * gate.first.position;
        temp1.position += transInverse;
        temp1.id = gate.first.id;
        Landmark temp2 = gate.second;
        temp2.position = rotationMatrix * gate.second.position;
        temp2.position += transInverse;
        temp2.id = gate.second.id;

        out.time_keeping_gates.push_back(std::make_pair(temp1, temp2));
    }

    return out;
}