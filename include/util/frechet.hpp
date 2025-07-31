#ifndef FRECHET_HPP
#define FRECHET_HPP

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <limits>

// Shoutout to McDonaldsHappyMeal for providing an initial python implementation of the algorithm
struct GridNode
{
    /*
    A class representing a matching from the ith vertex of path P with jth vertex of path Q.
    */
    size_t i;
    size_t j;
    double val;
    std::shared_ptr<GridNode> parent;
};

std::vector<std::vector<std::shared_ptr<GridNode>>> build_discrete_grid(std::vector<Eigen::Vector2d> P, std::vector<Eigen::Vector2d> Q);

double get_path_max(std::shared_ptr<GridNode> node);

void add_to_tree(std::vector<std::vector<std::shared_ptr<GridNode>>> &nodes, size_t i, size_t j);

std::vector<std::shared_ptr<GridNode>> compute_discrete_LCFM(std::vector<Eigen::Vector2d> P, std::vector<Eigen::Vector2d> Q);

#endif /* FRECHET_HPP */