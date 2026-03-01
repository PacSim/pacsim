#include "quaternion.hpp"

double quatNorm(quaternion a)
{
    double ret;
    ret = std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    return ret;
}

quaternion quatMult(quaternion a, quaternion b)
{
    quaternion ret;
    ret.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    ret.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    ret.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    ret.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return ret;
}

quaternion quatInversion(quaternion a)
{
    quaternion ret;
    ret.w = a.w;
    ret.x = -a.x;
    ret.y = -a.y;
    ret.z = -a.z;
    return ret;
}

quaternion quatNormalize(quaternion a)
{
    double norm = quatNorm(a);
    quaternion ret;
    ret.w = a.w / norm;
    ret.x = a.x / norm;
    ret.y = a.y / norm;
    ret.z = a.z / norm;
    return ret;
}

quaternion getRelativeQuat(quaternion a, quaternion b)
{
    quaternion ret;
    ret = quatMult(a, quatInversion(b));
    return ret;
}

quaternion quatFromEulerAngles(Eigen::Vector3d a)
{
    double x = a.x();
    double y = a.y();
    double z = a.z();

    quaternion quatX;
    quatX.w = std::cos(x / 2);
    quatX.x = std::sin(x / 2);
    quatX.y = 0.0;
    quatX.z = 0.0;

    quaternion quatY;
    quatY.w = std::cos(y / 2);
    quatY.x = 0.0;
    quatY.y = std::sin(y / 2);
    quatY.z = 0.0;

    quaternion quatZ;
    quatZ.w = std::cos(z / 2);
    quatZ.x = 0.0;
    quatZ.y = 0.0;
    quatZ.z = std::sin(z / 2);

    quaternion ret;
    ret.w = 1.0;
    ret.x = 0.0;
    ret.y = 0.0;
    ret.z = 0.0;

    ret = quatMult(quatX, ret);
    ret = quatMult(quatY, ret);
    ret = quatMult(quatZ, ret);
    return ret;
}

quaternion quatFromRotationMatrix(Eigen::Matrix3d mat)
{
    Eigen::Quaterniond q(mat);
    quaternion ret;
    ret.w = q.w();
    ret.x = q.x();
    ret.y = q.y();
    ret.z = q.z();
    return ret;
}

Eigen::Matrix3d rotationMatrixFromQuaternion(quaternion a)
{
    Eigen::Quaterniond q;
    q.w() = a.w;
    q.x() = a.x;
    q.y() = a.y;
    q.z() = a.z;
    Eigen::Matrix3d ret = q.toRotationMatrix();
    return ret;
}

Eigen::Vector3d eulerAnglesFromQuat(quaternion a)
{
    double sinr_cosp = 2 * (a.w * a.x + a.y * a.z);
    double cosr_cosp = 1 - 2 * (a.x * a.x + a.y * a.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (a.w * a.y - a.z * a.x);
    double pitch = std::asin(sinp);

    double siny_cosp = 2 * (a.w * a.z + a.x * a.y);
    double cosy_cosp = 1 - 2 * (a.y * a.y + a.z * a.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    Eigen::Vector3d ret(roll, pitch, yaw);
    return ret;
}