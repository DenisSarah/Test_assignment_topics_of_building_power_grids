#include "geojson_writer.h"
#include <sstream>
#include <iomanip>

using namespace std;

namespace gj
{

    static string esc(const string &s)
    {
        string o;
        o.reserve(s.size() + 16);
        for (char c : s)
        {
            if (c == '"' || c == '\\')
            {
                o.push_back('\\');
                o.push_back(c);
            }
            else if (c == '\n')
                o += "\\n";
            else
                o.push_back(c);
        }
        return o;
    }

    void Writer::add_point(double x, double y, const map<string, string> &props)
    {
        ostringstream o;
        o.setf(std::ios::fixed);
        o << setprecision(6);
        o << "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":["
          << x << "," << y << "]},\"properties\":{";
        bool first = true;
        for (const auto &kv : props)
        {
            if (!first)
                o << ",";
            first = false;
            o << "\"" << esc(kv.first) << "\":\"" << esc(kv.second) << "\"";
        }
        o << "}}";
        features.push_back(o.str());
    }

    void Writer::add_line(const vector<Pt> &pts, const map<string, string> &props)
    {
        if (pts.size() < 2)
            return;
        ostringstream o;
        o.setf(std::ios::fixed);
        o << setprecision(6);
        o << "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";
        for (size_t i = 0; i < pts.size(); ++i)
        {
            if (i)
                o << ",";
            o << "[" << pts[i].x << "," << pts[i].y << "]";
        }
        o << "]},\"properties\":{";
        bool first = true;
        for (const auto &kv : props)
        {
            if (!first)
                o << ",";
            first = false;
            o << "\"" << esc(kv.first) << "\":\"" << esc(kv.second) << "\"";
        }
        o << "}}";
        features.push_back(o.str());
    }

    std::string Writer::finish(const std::string &layer_name) const
    {
        ostringstream o;
        o << "{\n"
          << "  \"type\": \"FeatureCollection\",\n"
          << "  \"name\": \"" << esc(layer_name) << "\",\n"
          << "  \"crs\": {\"type\":\"name\",\"properties\":{\"name\":\""
          << esc(crs_name) << "\"}},\n"
          << "  \"features\": [\n";
        for (size_t i = 0; i < features.size(); ++i)
        {
            if (i)
                o << ",\n";
            o << "    " << features[i];
        }
        o << "\n  ]\n}";
        return o.str();
    }

}
