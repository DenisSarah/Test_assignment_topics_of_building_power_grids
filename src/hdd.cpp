#include "hdd.h"
#include "geometry.h"
#include <unordered_map>
#include <cmath>
#include <algorithm>

using std::pair;
using std::vector;

static bool inside_any_polygon(Pt p, const std::vector<Polygon> &polys)
{
    for (const auto &poly : polys)
        if (point_in_polygon(p, poly.ring))
            return true;
    return false;
}

static bool segment_is_cross_across_polygon(const Seg &s, const Roads &roads, int polyIdx) // TODO
{
    const auto &poly = roads.polygons[polyIdx];
    Pt mid{(s.a.x + s.b.x) / 2.0, (s.a.y + s.b.y) / 2.0};

    if (!point_in_polygon(mid, poly.ring))
        return false; // середина должна быть внутри этой дороги

    //  имеется как минимум два пересечения с его границей (вход и выход) - пока отмена ибо не работает
    // int hits = 0;
    // for (size_t k = 1; k < poly.ring.size(); ++k)
    // {
    //     if (seg_intersect(s, {poly.ring[k - 1], poly.ring[k]}))
    //         hits++;
    //     if (hits >= 2)
    //         break;
    // }
    // if (hits < 2)
    //     return false;

    // не должен проходить через внутренность другой дороги - пока отмена ибо не уверен, что оно надо
    // for (int i = 0; i < (int)roads.polygons.size(); ++i)
    // {
    //     if (i == polyIdx)
    //         continue;
    //     if (point_in_polygon(mid, roads.polygons[i].ring))
    //         return false;
    // }
    return true;
}

// Угол между направлением ребра s и сегментом границы дороги e - в градусах (0..180)
static double line_angle_deg(const Pt &p1, const Pt &p2, const Pt &a, const Pt &b)
{
    // return 79.0;
    Pt u = p2 - p1;
    Pt v = b - a;
    double cu = norm(u), cv = norm(v);
    if (cu < 1e-12 || cv < 1e-12)
        return 0.0;
    double c = std::max(-1.0, std::min(1.0, dot(u, v) / (cu * cv)));
    double ang = std::acos(c) * 180.0 / M_PI;
    return ang;
}


// для каждой точки пересечения угол из [90-alpha, 90+alpha].
static bool all_intersections_within_perp_band(const Roads &roads, int polyIdx,
                                               const Seg &s, double alphaDeg)
{
    // return true;
    const auto &R = roads.polygons[polyIdx].ring;
    bool any = false;
    const double lo = 90.0 - alphaDeg;
    const double hi = 90.0 + alphaDeg;

    for (size_t k = 1; k < R.size(); ++k)
    {
        Pt ip;
        if (seg_intersect(s, {R[k - 1], R[k]}, &ip))
        {
            any = true;
            double ang = line_angle_deg(s.a, s.b, R[k - 1], R[k]);
            double ang1 = line_angle_deg(s.b, s.a, R[k - 1], R[k]);
            if (!(ang + 1 >= lo && ang - 1 <= hi))
            {
                return false;
            }
        }
    }
    return any;
}

HDDGraph build_hdd_from_trench(const Roads &roads,
                               const vector<Pt> &trench_nodes,
                               const vector<pair<int, int>> &trench_edges,
                               const HDDParams &prm)
{
    HDDGraph g;


    g.nodes = trench_nodes;
    g.trench_to_hdd.resize(trench_nodes.size());
    for (size_t i = 0; i < trench_nodes.size(); ++i)
        g.trench_to_hdd[i] = (int)i;


    g.edges = trench_edges;

    // Рёбра поперёк дорог — добавляем все пары (i,j), удовлетворяющие длине и углу
    double cell = std::max(1e-6, prm.cross_max);
    std::unordered_map<long long, vector<int>> grid;
    auto cellKey = [&](const Pt &p)
    {
        long long ix = (long long)std::floor(p.x / cell), iy = (long long)std::floor(p.y / cell);
        return (ix << 32) ^ (iy & 0xffffffff);
    };
    for (int i = 0; i < (int)g.nodes.size(); ++i)
        grid[cellKey(g.nodes[i])].push_back(i);

    auto nearby = [&](const Pt &p)
    {
        vector<int> out;
        out.reserve(64);
        long long ix = (long long)std::floor(p.x / cell), iy = (long long)std::floor(p.y / cell);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
            {
                auto it = grid.find(((ix + dx) << 32) ^ ((iy + dy) & 0xffffffff));
                if (it != grid.end())
                    out.insert(out.end(), it->second.begin(), it->second.end());
            }
        return out;
    };

    auto L2 = [](const Pt &a, const Pt &b)
    { Pt d=b-a; return d.x*d.x + d.y*d.y; };

    for (int i = 0; i < (int)g.nodes.size(); ++i)
    {
        auto cand = nearby(g.nodes[i]);
        for (int j : cand)
        {
            if (j <= i)
                continue; // избежать дублей и петель

            double L = std::sqrt(L2(g.nodes[i], g.nodes[j]));
            if (L + 1e-9 < prm.cross_min || L - 1e-9 > prm.cross_max)
                continue;

            Seg s{g.nodes[i], g.nodes[j]};

            // Должен пересекать ВНУТРЕННОСТЬ хотя бы одной дороги и удовлетворять углу 90°±α
            bool ok = false;
            for (int pi = 0; pi < (int)roads.polygons.size(); ++pi)
            {
                if (segment_is_cross_across_polygon(s, roads, pi) && all_intersections_within_perp_band(roads, pi, s, prm.cross_angle_tol_deg))
                {
                    ok = true;
                    break;
                }
            }
            if (!ok)
                continue;

            g.edges.emplace_back(i, j);
        }
    }

    return g;
}
