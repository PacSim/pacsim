#include "track/gripMap.hpp"

gripMap::gripMap(std::shared_ptr<Logger> logger) { this->logger = logger; }

void gripMap::loadConfig(std::string path)
{
    Config cfg(path);
    ConfigElement config = cfg.getElement("grip_map");
    config.getElement<double>(&this->total_scale, "total_scale");

    std::vector<ConfigElement> gripPoints;
    config["points"].getElements(&gripPoints);

    if (gripPoints.size() == 0)
    {
        this->logger->logFatal("No gripMap points provided!");
        std::abort();
    }

    for (auto& point : gripPoints)
    {
        double gripValue;
        std::vector<double> position;
        point.getElement<std::vector<double>>(&position, "position");
        Eigen::Vector3d positionEigen(position.at(0), position.at(1), position.at(2));
        point.getElement<double>(&gripValue, "value");

        this->mapPoints.push_back(std::make_pair(positionEigen, gripValue));
    }
}

Wheels gripMap::getGripValues(std::array<Eigen::Vector3d, 4> in)
{
    std::array<double, 4> temp;
    for (int i = 0; i < 4; ++i)
    {
        auto el = in.at(i);
        auto bestPoint = std::min_element(std::begin(this->mapPoints), std::end(this->mapPoints),
            [&el](const std::pair<Eigen::Vector3d, double>& a, const std::pair<Eigen::Vector3d, double>& b)
            { return (a.first - el).norm() < (b.first - el).norm(); });
        temp[i] = ((*bestPoint).second);
    }

    Wheels ret;
    ret.FL = this->total_scale * temp[0];
    ret.FR = this->total_scale * temp[1];
    ret.RL = this->total_scale * temp[2];
    ret.RR = this->total_scale * temp[3];

    return ret;
}

void gripMap::transformPoints(Eigen::Vector3d trans, Eigen::Vector3d rot)
{
    Eigen::Matrix3d rotationMatrix = eulerAnglesToRotMat(rot);
    Eigen::Vector3d transInverse = inverseTranslation(trans, rot);

    for (int i = 0; i < this->mapPoints.size(); ++i)
    {
        this->mapPoints[i].first = rotationMatrix * this->mapPoints[i].first;
        this->mapPoints[i].first += transInverse;
    }
}
