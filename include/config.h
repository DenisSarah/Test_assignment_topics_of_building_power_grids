#pragma once
#include <string>

struct Config
{
    double trench_per_m = 1.0;
    double hdd_per_m = 2.0;
    double transition_per_edge = 5.0;

    double hdd_min_length = 30.0;
    double hdd_max_length = 150.0;
    double hdd_alpha_deg = 10.0;

    double grid_step = 25.0;
    double boundary_step = 20.0;

    std::string output_basename = "graph";
};
