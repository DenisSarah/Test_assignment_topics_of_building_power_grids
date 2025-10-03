#pragma once
#include <string>
#include "roads.h"
#include "config.h"

namespace io
{

    std::string read_file(const std::string &path);

    bool load_config(const std::string &path, Config &cfg);

    bool load_roads_geojson(const std::string &path, Roads &roads);

}
