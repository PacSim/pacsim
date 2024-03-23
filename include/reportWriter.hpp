#ifndef REPORTWRITER_HPP
#define REPORTWRITER_HPP

#include "types.hpp"
#include "yaml-cpp/yaml.h"
#include <ctime>
#include <fstream>
#include <iostream>

bool reportToFile(Report report, std::string dir);

#endif /* REPORTWRITER_HPP */