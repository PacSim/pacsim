#include "util/centerLine.hpp"

std::vector<std::pair<size_t, size_t>> getMiddleLine(Track& track) {
    std::vector<std::pair<size_t, size_t>> ret;
    bool isProper = (track.left_lane.size() > 3) && (track.right_lane.size() >= 3);
    if(isProper) {
        std::vector<Eigen::Vector2d> left;
        for (auto &lm : track.left_lane)
        {
            left.push_back(Eigen::Vector2d(lm.position.x(), lm.position.y()));
        }
        std::vector<Eigen::Vector2d> right;
        for (auto &lm : track.right_lane)
        {
            right.push_back(Eigen::Vector2d(lm.position.x(), lm.position.y()));
        }
        
        auto nodes = compute_discrete_LCFM(left, right);
        
        
        for (int i = 0; i < (nodes.size() - 1); ++i)
        {
            auto n1 = nodes[i];
            auto pair = std::make_pair(n1->i, n1->j);
            ret.push_back(pair);
        }
    }
        
    return ret;
}