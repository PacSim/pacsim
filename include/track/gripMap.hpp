#include "configParser.hpp"
#include "logger.hpp"
#include "transform.hpp"
#include "types.hpp"
#include <csignal>

class gripMap
{
public:
    gripMap(std::shared_ptr<Logger> logger);
    void loadConfig(std::string path);
    Wheels getGripValues(std::array<Eigen::Vector3d, 4> in);
    void transformPoints(Eigen::Vector3d trans, Eigen::Vector3d rot);

private:
    std::shared_ptr<Logger> logger;
    double total_scale = 1.0;
    std::vector<std::pair<Eigen::Vector3d, double>> mapPoints;
};
