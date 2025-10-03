#include "io.h"

#include <fstream>
#include <sstream>
#include <regex>
#include <cctype>

using namespace std;

namespace io
{

    string read_file(const string &path)
    {
        ifstream f(path, ios::binary);
        if (!f)
            return {};
        ostringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    // CONFIG
    static bool extract_double(const string &s, const string &key, double &dst)
    {
        regex re("\\\"" + key + "\\\"\\s*:\\s*([-+]?([0-9]*\\.)?[0-9]+([eE][-+]?[0-9]+)?)");
        smatch m;
        if (regex_search(s, m, re))
        {
            dst = stod(m[1].str());
            return true;
        }
        return false;
    }
    static bool extract_string(const string &s, const string &key, string &dst)
    {
        regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
        smatch m;
        if (regex_search(s, m, re))
        {
            dst = m[1].str();
            return true;
        }
        return false;
    }

    bool load_config(const string &path, Config &cfg)
    {
        string s = read_file(path);
        if (s.empty())
            return false;

        extract_double(s, "trench_per_m", cfg.trench_per_m);
        extract_double(s, "hdd_per_m", cfg.hdd_per_m);
        extract_double(s, "transition_per_edge", cfg.transition_per_edge);

        extract_double(s, "min_length", cfg.hdd_min_length);
        extract_double(s, "max_length", cfg.hdd_max_length);
        extract_double(s, "alpha_deg", cfg.hdd_alpha_deg);

        extract_double(s, "grid_step", cfg.grid_step);
        extract_double(s, "boundary_sample_step", cfg.boundary_step);

        extract_string(s, "basename", cfg.output_basename);
        return true;
    }

    //  GEOJSON
    static vector<Pt> parse_coords_array(const string &s, size_t start_idx)
    {
        vector<Pt> out;
        int depth = 0;
        bool inNum = false;
        string num;
        double last = 0;
        bool expectY = false;
        for (size_t i = start_idx; i < s.size(); ++i)
        {
            char c = s[i];
            if (c == '[')
            {
                depth++;
                continue;
            }
            if (c == ']')
            {
                if (inNum)
                {
                    double v = stod(num);
                    if (!expectY)
                    {
                        last = v;
                        expectY = true;
                    }
                    else
                    {
                        out.push_back({last, v});
                        expectY = false;
                    }
                    inNum = false;
                    num.clear();
                }
                depth--;
                if (depth == 0)
                    break;
                else
                    continue;
            }
            if ((c == '-' || c == '+' || isdigit((unsigned char)c) || c == '.' || c == 'e' || c == 'E') && depth >= 2)
            {
                inNum = true;
                num.push_back(c);
            }
            else if (inNum)
            {
                double v = stod(num);
                if (!expectY)
                {
                    last = v;
                    expectY = true;
                }
                else
                {
                    out.push_back({last, v});
                    expectY = false;
                }
                inNum = false;
                num.clear();
            }
        }
        if (out.size() > 1)
        {
            const auto &a = out.front();
            const auto &b = out.back();
            if (fabs(a.x - b.x) > 1e-9 || fabs(a.y - b.y) > 1e-9)
                out.push_back(a);
        }
        return out;
    }

    static void parse_polygons(const string &s, Roads &roads)
    {
        size_t pos = 0;
        while ((pos = s.find("\"Polygon\"", pos)) != string::npos)
        {
            size_t coords = s.find("\"coordinates\"", pos);
            if (coords == string::npos)
                break;
            coords = s.find('[', coords);
            if (coords == string::npos)
                break;
            auto ring = parse_coords_array(s, coords);
            if (!ring.empty())
                roads.polygons.push_back({ring});
            pos += 8;
        }
    }

    static void parse_multipolygons(const string &s, Roads &roads)
    {
        size_t pos = 0;
        while ((pos = s.find("\"MultiPolygon\"", pos)) != string::npos)
        {
            size_t coords = s.find("\"coordinates\"", pos);
            if (coords == string::npos)
                break;
            coords = s.find('[', coords);
            if (coords == string::npos)
                break;
            int depth = 0;
            size_t firstRingPos = string::npos;
            for (size_t i = coords; i < s.size(); ++i)
            {
                char c = s[i];
                if (c == '[')
                {
                    depth++;
                    if (depth == 3)
                    {
                        firstRingPos = i;
                        break;
                    }
                }
                else if (c == ']')
                {
                    depth--;
                    if (depth == 0)
                        break;
                }
            }
            if (firstRingPos != string::npos)
            {
                auto ring = parse_coords_array(s, firstRingPos);
                if (!ring.empty())
                    roads.polygons.push_back({ring});
            }
            pos += 12;
        }
    }

    static void parse_linestrings(const string &s, Roads &roads)
    {
        size_t pos = 0;
        while ((pos = s.find("\"LineString\"", pos)) != string::npos)
        {
            size_t coords = s.find("\"coordinates\"", pos);
            if (coords == string::npos)
                break;
            coords = s.find('[', coords);
            if (coords == string::npos)
                break;
            vector<Pt> line;
            int depth = 0;
            bool inNum = false;
            string num;
            double last = 0;
            bool expectY = false;
            for (size_t i = coords; i < s.size(); ++i)
            {
                char c = s[i];
                if (c == '[')
                {
                    depth++;
                    continue;
                }
                if (c == ']')
                {
                    if (inNum)
                    {
                        double v = stod(num);
                        if (!expectY)
                        {
                            last = v;
                            expectY = true;
                        }
                        else
                        {
                            line.push_back({last, v});
                            expectY = false;
                        }
                        inNum = false;
                        num.clear();
                    }
                    depth--;
                    if (depth == 1)
                    {
                        break;
                    }
                    else
                        continue;
                }
                if ((c == '-' || c == '+' || isdigit((unsigned char)c) || c == '.' || c == 'e' || c == 'E') && depth >= 3)
                {
                    inNum = true;
                    num.push_back(c);
                }
                else if (inNum)
                {
                    double v = stod(num);
                    if (!expectY)
                    {
                        last = v;
                        expectY = true;
                    }
                    else
                    {
                        line.push_back({last, v});
                        expectY = false;
                    }
                    inNum = false;
                    num.clear();
                }
            }
            if (line.size() >= 2)
                roads.lines.push_back(line);
            pos += 10;
        }
    }

    bool load_roads_geojson(const string &path, Roads &roads)
    {
        string s = read_file(path);
        if (s.empty())
            return false;
        parse_polygons(s, roads);
        parse_multipolygons(s, roads);
        parse_linestrings(s, roads);
        return !(roads.polygons.empty() && roads.lines.empty());
    }

}
