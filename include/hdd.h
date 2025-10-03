#pragma once
#include <vector>
#include <utility>
#include "roads.h"

struct HDDGraph
{
    std::vector<Pt> nodes;
    std::vector<std::pair<int, int>> edges;
    std::vector<int> trench_to_hdd;
};

struct HDDParams
{
    double cross_min = 3.0;
    double cross_max = 50.0;
    double cross_angle_tol_deg = 20.0;
};

HDDGraph build_hdd_from_trench(const Roads &roads,
                               const std::vector<Pt> &trench_nodes,
                               const std::vector<std::pair<int, int>> &trench_edges,
                               const HDDParams &prm);
