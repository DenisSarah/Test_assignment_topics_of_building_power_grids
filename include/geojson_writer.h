#pragma once
#include <string>
#include <map>
#include <vector>
#include "geometry.h"

namespace gj
{

    struct Writer
    {
        std::vector<std::string> features;

        std::string crs_name = "urn:ogc:def:crs:EPSG::3857";

        void add_point(double x, double y, const std::map<std::string, std::string> &props);
        void add_line(const std::vector<Pt> &pts, const std::map<std::string, std::string> &props);

        std::string finish(const std::string &layer_name) const;
    };

}
