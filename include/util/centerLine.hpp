#ifndef CENTERLINE_HPP
#define CENTERLINE_HPP

// #include <Eigen/Core>
// #include <vector>
// #include <memory>
// #include <limits>
#include <iostream>

#include "util/frechet.hpp"
#include "types.hpp"

std::vector<std::pair<size_t, size_t>> getMiddleLine(Track& track);

#endif /* CENTERLINE_HPP */