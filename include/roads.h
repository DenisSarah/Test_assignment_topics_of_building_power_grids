#pragma once
#include <vector>
#include "geometry.h"

struct Polygon
{
    std::vector<Pt> ring;
};

struct Roads
{
    std::vector<Polygon> polygons;
    std::vector<std::vector<Pt>> lines;
};
