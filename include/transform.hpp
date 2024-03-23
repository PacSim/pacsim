#ifndef PACSIMTRANSFORM_HPP
#define PACSIMTRANSFORM_HPP

#include "types.hpp"
#include <Eigen/Dense>

Eigen::Vector3d rigidBodyVelocity(Eigen::Vector3d v, Eigen::Vector3d r, Eigen::Vector3d omega);

Eigen::Vector3d rigidBodyAcceleration(Eigen::Vector3d a, Eigen::Vector3d r, Eigen::Vector3d omega, Eigen::Vector3d alpha);

Eigen::Vector3d inverseTranslation(Eigen::Vector3d trans, Eigen::Vector3d rot);

LandmarkList transformLmList(LandmarkList& in, Eigen::Vector3d trans, Eigen::Vector3d rot);

#endif /* PACSIMTRANSFORM_HPP */