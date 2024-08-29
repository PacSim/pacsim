#include "configParser.hpp"
#include "logger.hpp"
#include "types.hpp"
#include <csignal>

class gripMap
{
public:
    gripMap(std::shared_ptr<Logger> logger);
    void loadConfig(std::string path);
    Wheels getGripValues(std::array<Eigen::Vector3d, 4> in);

private:
    std::shared_ptr<Logger> logger;
    double total_scale = 1.0;
    std::vector<std::pair<Eigen::Vector3d, double>> mapPoints;
};
