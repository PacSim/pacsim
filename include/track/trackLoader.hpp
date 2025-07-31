#ifndef TRACKLOADER_HPP
#define TRACKLOADER_HPP

#include "types.hpp"
#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"
#include "util/centerLine.hpp"

Track loadMap(std::string mapPath, Eigen::Vector3d& start_position, Eigen::Vector3d& start_orientation, bool flip_y);

#endif /* TRACKLOADER_HPP */