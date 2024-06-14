#ifndef PACSIMQUATERNION_HPP
#define PACSIMQUATERNION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

struct quaternion
{
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

double quatNorm(quaternion a);

quaternion quatMult(quaternion a, quaternion b);

quaternion quatInversion(quaternion a);

quaternion quatNormalize(quaternion a);

quaternion getRelativeQuat(quaternion a, quaternion b);

quaternion quatFromEulerAngles(Eigen::Vector3d a);

quaternion quatFromRotationMatrix(Eigen::Matrix3d mat);

Eigen::Matrix3d rotationMatrixFromQuaternion(quaternion a);

Eigen::Vector3d eulerAnglesFromQuat(quaternion a);

#endif /* PACSIMQUATERNION_HPP */