#include "track/trackLoader.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>

using namespace std;
using namespace YAML;

Node fillChildNode(Node parentNode, string tag)
{
    if (parentNode[tag])
    {
        Node childNode = parentNode[tag];
        return childNode;
    }
    else
        throw runtime_error(string("Failed loading the map, no ") + tag);
}

void addLandmarks(std::vector<Landmark>* _ret, Node* list, int* _coneCounter)
{
    Landmark lm;
    for (const_iterator it = list->begin(); it != list->end(); ++it)
    {
        const Node& position = *it;
        vector<double> vi = position["position"].as<vector<double>>();
        lm.id = *_coneCounter;
        *_coneCounter++;
        lm.position = Eigen::Vector3d(vi[0], vi[1], vi[2]);
        lm.type = stringToLandmarkType(position["class"].as<std::string>());
        lm.typeWeights[lm.type] = 1.0;
        _ret->push_back(lm);
    }
}

void addTimeKeepings(std::vector<std::pair<Landmark, Landmark>>* _ret, Node* list, int* _coneCounter)
{
    Landmark lm;
    std::vector<Landmark> lms;
    for (const_iterator it = list->begin(); it != list->end(); ++it)
    {
        const Node& position = *it;
        vector<double> vi = position["position"].as<vector<double>>();
        lm.id = *_coneCounter;
        *_coneCounter++;
        lm.position = Eigen::Vector3d(vi[0], vi[1], vi[2]);
        lm.type = stringToLandmarkType(position["class"].as<std::string>());
        lm.typeWeights[lm.type] = 1.0;
        lms.push_back(lm);
    }
    for (int i = 0; i < (lms.size() / 2); ++i)
    {
        _ret->push_back(std::make_pair(lms[i * 2], lms[i * 2 + 1]));
    }
}

Track loadMap(string mapPath, Eigen::Vector3d& start_position, Eigen::Vector3d& start_orientation)
{
    Node map = LoadFile(mapPath);
    Node track = fillChildNode(map, "track");
    Node left = fillChildNode(track, "left");
    Node right = fillChildNode(track, "right");
    Node time_keeping = fillChildNode(track, "time_keeping");
    Node unknown = fillChildNode(track, "unknown");
    // TODO: Catch pos or orientation size != 3
    std::vector<double> start_pos = track["start"]["position"].as<vector<double>>();
    std::vector<double> start_or = track["start"]["orientation"].as<vector<double>>();
    start_position = Eigen::Vector3d(start_pos.data());
    start_orientation = Eigen::Vector3d(start_or.data());

    int coneCounter = 0;
    Track ret;
    if (track["lanesFirstWithLastConnected"])
    {
        ret.lanesFirstWithLastConnected = track["lanesFirstWithLastConnected"].as<bool>();
    }
    else
    {
        ret.lanesFirstWithLastConnected = true;
    }

    addLandmarks(&ret.left_lane, &left, &coneCounter);
    addLandmarks(&ret.right_lane, &right, &coneCounter);
    addTimeKeepings(&ret.time_keeping_gates, &time_keeping, &coneCounter);
    addLandmarks(&ret.unknown, &unknown, &coneCounter);

    return ret;
}
