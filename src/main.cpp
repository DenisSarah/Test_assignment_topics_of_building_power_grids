#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "io.h"
#include "graph.h"
#include "hdd.h"
#include "geojson_writer.h"
#include "geometry.h"

static void save_text(const std::string &path, const std::string &data)
{
    std::ofstream f(path);
    f << data;
}

int main(int argc, char **argv)
{
    std::string roads_path;
    std::string config_path;
    std::string out_base = "graph";

    // аргументы
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        if (a == "--roads" && i + 1 < argc)
            roads_path = argv[++i];
        else if (a == "--config" && i + 1 < argc)
            config_path = argv[++i];
        else if (a == "--out" && i + 1 < argc)
            out_base = argv[++i];
    }

    if (roads_path.empty())
    {
        std::cerr << "Usage: reader --roads roads.geojson [--config config.json] [--out graph]\n";
        return 1;
    }

    // чтение
    Roads roads;
    if (!io::load_roads_geojson(roads_path, roads))
    {
        std::cerr << "Failed to read GeoJSON roads from: " << roads_path << "\n";
        return 2;
    }
    Config cfg;
    if (!config_path.empty())
    {
        if (!io::load_config(config_path, cfg))
        {
            std::cerr << "Warning: can't read config: " << config_path << " (using defaults)\n";
        }
    }

    std::cout << "OK: loaded roads\n";
    std::cout << " polygons: " << roads.polygons.size() << "\n";
    std::cout << " lines:    " << roads.lines.size() << "\n";

    auto trench = build_trench_strict(roads, cfg.boundary_step);
    std::cout << "Trench: nodes=" << trench.nodes.size() << ", edges=" << trench.edges.size() << "\n";

    {
        gj::Writer w;
        for (size_t i = 0; i < trench.nodes.size(); ++i)
        {
            w.add_point(
                trench.nodes[i].x, trench.nodes[i].y,
                {{"type", "trench"}, {"id", std::to_string(i)}});
        }
        save_text(out_base + "_nodes_trench.geojson", w.finish("nodes_trench"));
    }

    {
        gj::Writer w;
        for (auto [u, v] : trench.edges)
        {
            std::vector<Pt> line{trench.nodes[u], trench.nodes[v]};
            double L = norm(trench.nodes[v] - trench.nodes[u]);
            double C = L * cfg.trench_per_m;
            w.add_line(line, {{"type", "trench"},
                              {"length", std::to_string(L)},
                              {"cost", std::to_string(C)}});
        }
        save_text(out_base + "_edges_trench.geojson", w.finish("edges_trench"));
    }
    std::cout << "Written: " << out_base << "_nodes_trench.geojson, " << out_base << "_edges_trench.geojson\n";

    HDDParams prm;

    prm.cross_min = cfg.hdd_min_length;
    prm.cross_max = cfg.hdd_max_length;
    prm.cross_angle_tol_deg = cfg.hdd_alpha_deg;

    auto hdd = build_hdd_from_trench(roads, trench.nodes, trench.edges, prm);
    std::cout << "HDD: nodes=" << hdd.nodes.size() << ", edges=" << hdd.edges.size() << "\n";

    {
        gj::Writer w;
        for (size_t i = 0; i < hdd.nodes.size(); ++i)
        {
            w.add_point(
                hdd.nodes[i].x, hdd.nodes[i].y,
                {{"type", "hdd"}, {"id", std::to_string(i)}});
        }
        save_text(out_base + "_nodes_hdd.geojson", w.finish("nodes_hdd"));
    }

    {
        gj::Writer w;
        for (auto [u, v] : hdd.edges)
        {
            std::vector<Pt> line{hdd.nodes[u], hdd.nodes[v]};
            double L = norm(hdd.nodes[v] - hdd.nodes[u]);
            double C = L * cfg.hdd_per_m;
            w.add_line(line, {{"type", "hdd"},
                              {"length", std::to_string(L)},
                              {"cost", std::to_string(C)}});
        }
        save_text(out_base + "_edges_hdd.geojson", w.finish("edges_hdd"));
    }
    std::cout << "Written: " << out_base << "_nodes_hdd.geojson, "
              << out_base << "_edges_hdd.geojson\n";

    {
        gj::Writer w;
        int transitions = 0;
        for (size_t i = 0; i < trench.nodes.size(); ++i)
        {
            const Pt &p = trench.nodes[i];
            // std::vector<Pt> zero{p, p}; // нулевая длина и ненулевая стоимость
            // w.add_line(zero, {{"type", "transition"},
            //                   {"length", "0"},
            //                   {"cost", std::to_string(cfg.transition_per_edge)}});
            w.add_point( // так их видно
                p.x, p.y,
                {{"type", "transition"},
                 {"length", "0"},
                 {"cost", std::to_string(cfg.transition_per_edge)}});
            ++transitions;
        }
        save_text(out_base + "_edges_transition.geojson", w.finish("edges_transition"));
        std::cout << "Transitions: " << transitions << "\n";
        std::cout << "Written: " << out_base << "_edges_transition.geojson\n";
    }

    return 0;
}
