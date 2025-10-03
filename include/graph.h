#pragma once
#include <vector>
#include <utility>
#include "roads.h"

struct TrenchGraph
{
    std::vector<Pt> nodes;
    std::vector<std::pair<int, int>> edges;
};

std::vector<Pt> sample_ring(const std::vector<Pt> &ring, double h);

TrenchGraph build_trench_strict(const Roads &roads, double boundary_step);
