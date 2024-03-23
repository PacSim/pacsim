#ifndef TRACKLOADER_HPP
#define TRACKLOADER_HPP

#include "types.hpp"
#include <vector>
#include <string>

Track loadMap(std::string mapPath, Eigen::Vector3d& start_position, Eigen::Vector3d& start_orientation);

#endif /* TRACKLOADER_HPP */