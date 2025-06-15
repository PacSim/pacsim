#include "util/frechet.hpp"

std::vector<std::vector<std::shared_ptr<GridNode>>> build_discrete_grid(std::vector<Eigen::Vector2d> P, std::vector<Eigen::Vector2d> Q)
{
    /*
    Create (m+1) by (n+1) grid of distances.
    grid[i,j] = distance(P[i], Q[j])
    FYI polygonal chain P has m+1 vertices, polygonal chain Q has n+1 vertices.
    */
    size_t m = P.size();
    size_t n = Q.size();

    std::vector<std::vector<std::shared_ptr<GridNode>>> nodes(m, std::vector<std::shared_ptr<GridNode>>(n));
    for (size_t i = 0; i < m; ++i)
    {
        for (size_t j = 0; j < n; ++j)
        {
            std::shared_ptr<GridNode> node = std::make_shared<GridNode>();
            node->i = i;
            node->j = j;
            node->val = (P[i] - Q[j]).norm();
            nodes[i][j] = node;
        }
    }
    return nodes;
}

double get_path_max(std::shared_ptr<GridNode> node)
{
    /*
    Very naive check to get the maximum distance from the root node to node.
    */
    double mx = node->val;
    std::shared_ptr<GridNode> cur = node;
    while (cur != nullptr)
    {
        mx = std::max(mx, cur->val);
        cur = cur->parent;
    }
    return mx;
}

void add_to_tree(std::vector<std::vector<std::shared_ptr<GridNode>>> &nodes, size_t i, size_t j)
{
    /*
    Links node (i,j) to tree of locally correct Frechet matchings.
    Note that if there are tiebreaks, we put in order: (i-1,j), (i-1,j-1), (i,j-1). Page 14 of paper.
    */
    if ((i == 0) && (j == 0))
    {
        return; // root node, no parent
    }
    std::vector<std::shared_ptr<GridNode>> candidates;
    if (i > 0)
    {
        candidates.push_back(nodes[i - 1][j]);
    }
    if ((i > 0) && (j > 0))
    {
        candidates.push_back(nodes[i - 1][j - 1]);
    }
    if (j > 0)
    {
        candidates.push_back(nodes[i][j - 1]);
    }

    std::shared_ptr<GridNode> best_parent;
    double best_path_max = std::numeric_limits<float>::infinity();

    for (auto &candidate : candidates)
    {
        double candidate_path_max = get_path_max(candidate);
        if (candidate_path_max < best_path_max)
        {
            best_parent = candidate;
            best_path_max = candidate_path_max;
        }
        else if (std::abs(candidate_path_max - best_path_max) < 1e-12)
        {
            // TODO
            // pass
        }
    }

    nodes[i][j]->parent = best_parent;
}

std::vector<std::shared_ptr<GridNode>> compute_discrete_LCFM(std::vector<Eigen::Vector2d> P, std::vector<Eigen::Vector2d> Q)
{
    /*
    Builds grid, links up all nodes so that it enforces local correctness. Returns the path from (0,0) to (m,n).
    */
    // Build the grid of distances:
    auto nodes = build_discrete_grid(P, Q);
    size_t m = P.size();
    size_t n = Q.size();

    // Fill out the entire first row and first column, joining each node to its immediate predecessor, so we get a monotone path up each row or column.
    for (size_t i = 0; i < m; ++i)
    {
        if (i == 0)
        {
            continue;
        }
        nodes[i][0]->parent = nodes[i - 1][0];
    }
    for (size_t j = 0; j < n; ++j)
    {
        if (j == 0)
        {
            continue;
        }
        nodes[0][j]->parent = nodes[0][j - 1];
    }

    // Now fill inside:
    for (size_t i = 1; i < m; ++i)
    {
        for (size_t j = 1; j < n; ++j)
        {
            add_to_tree(nodes, i, j);
        }
    }

    // Reconstruct path from(m, n) back to(0, 0)
    std::vector<std::shared_ptr<GridNode>> path;
    auto cur = nodes[m - 1][n - 1];
    while (cur != nullptr)
    {
        path.push_back(cur);
        cur = cur->parent;
        std::reverse(path.begin(), path.end()); // Now it is (0,0) to (m,n)
    }
    return path;
}