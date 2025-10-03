#include "graph.h"
#include "geometry.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

static bool inside_any_other(Pt p, const Roads &roads, int selfIdx)
{
    for (int i = 0; i < (int)roads.polygons.size(); ++i)
    {
        if (i == selfIdx)
            continue;
        if (point_in_polygon(p, roads.polygons[i].ring))
            return true;
    }
    return false;
}

static bool seg_crosses_other_roads(const Seg &s, const Roads &roads, int selfIdx)
{
    const double EPS_END = 1e-7;
    Pt d{s.b.x - s.a.x, s.b.y - s.a.y};

    for (int i = 0; i < (int)roads.polygons.size(); ++i)
    {
        if (i == selfIdx)
            continue;
        const auto &R = roads.polygons[i].ring;

        int in_cnt = 0;
        for (double t : {0.2, 0.4, 0.6, 0.8})
        {
            Pt q{s.a.x + d.x * t, s.a.y + d.y * t};
            if (point_in_polygon(q, R))
                ++in_cnt;
        }
        if (in_cnt >= 2)
            return true;

        for (size_t k = 1; k < R.size(); ++k)
        {
            Pt ip;
            if (seg_intersect(s, {R[k - 1], R[k]}, &ip))
            {
                if (norm(ip - s.a) < EPS_END || norm(ip - s.b) < EPS_END)
                    continue;
                return true;
            }
        }
    }
    return false;
}

std::vector<Pt> sample_ring(const std::vector<Pt> &ring, double h)
{
    std::vector<Pt> out;
    if (ring.size() < 2)
        return out;
    out.push_back(ring.front());

    auto seglen = [](Pt a, Pt b)
    {
        double dx = a.x - b.x, dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    };

    for (size_t i = 1; i < ring.size(); ++i)
    {
        Pt a = ring[i - 1], b = ring[i];
        double L = seglen(a, b);
        if (L < 1e-9)
            continue;

        int m = std::max(1, (int)std::floor(L / std::max(1e-9, h)));
        double step = L / m;

        for (int k = 1; k <= m; k++)
        {
            double t = (k * step) / L;
            Pt p{a.x * (1.0 - t) + b.x * t, a.y * (1.0 - t) + b.y * t};
            if (i < ring.size() - 1 || k < m)
                out.push_back(p);
        }
    }
    return out;
}

static inline long long node_key_mm(Pt p)
{
    long long X = llround(p.x * 1000.0);
    long long Y = llround(p.y * 1000.0);
    return (X << 21) ^ Y;
}

static int add_node_dedup(TrenchGraph &g, std::unordered_map<long long, int> &index, Pt p)
{
    long long k = node_key_mm(p);
    auto it = index.find(k);
    if (it != index.end())
        return it->second;
    int id = (int)g.nodes.size();
    g.nodes.push_back(p);
    index.emplace(k, id);
    return id;
}

static double param_on_seg(Pt a, Pt b, Pt p)
{
    Pt ab = b - a, ap = p - a;
    double den = std::max(1e-12, norm2(ab));
    double t = dot(ap, ab) / den;
    if (t < 0.0)
        t = 0.0;
    else if (t > 1.0)
        t = 1.0;
    return t;
}

struct Hit
{
    double t;
    Pt p;
};

static std::vector<std::vector<std::vector<Hit>>>
collect_cross_hits(const std::vector<std::vector<Pt>> &sampled)
{
    const double EPS_END = 1e-7;
    int N = (int)sampled.size();
    std::vector<std::vector<std::vector<Hit>>> hits(N);

    for (int a = 0; a < N; ++a)
    {
        int na = (int)sampled[a].size();
        if (na < 2)
            continue;
        hits[a].assign(na, {});
    }

    for (int a = 0; a < N; ++a)
    {
        const auto &A = sampled[a];
        int na = (int)A.size();
        if (na < 2)
            continue;

        for (int b = a + 1; b < N; ++b)
        {
            const auto &B = sampled[b];
            int nb = (int)B.size();
            if (nb < 2)
                continue;

            for (int i = 0; i < na; ++i)
            {
                Seg sa{A[i], A[(i + 1) % na]};

                for (int j = 0; j < nb; ++j)
                {
                    Seg sb{B[j], B[(j + 1) % nb]};
                    Pt ip;
                    if (!seg_intersect(sa, sb, &ip))
                        continue;

                    double ta = param_on_seg(sa.a, sa.b, ip);
                    double tb = param_on_seg(sb.a, sb.b, ip);
                    bool at_end_a = (ta < EPS_END || ta > 1.0 - EPS_END);
                    bool at_end_b = (tb < EPS_END || tb > 1.0 - EPS_END);
                    if (at_end_a && at_end_b)
                        continue;

                    if (!at_end_a)
                        hits[a][i].push_back({ta, ip});
                    if (!at_end_b)
                        hits[b][j].push_back({tb, ip});
                }
            }
        }
    }

    const double EPS_MERGE = 1e-6;
    for (auto &poly_hits : hits)
    {
        for (auto &seg_hits : poly_hits)
        {
            std::sort(seg_hits.begin(), seg_hits.end(),
                      [](const Hit &u, const Hit &v)
                      { return u.t < v.t; });
            std::vector<Hit> u;
            u.reserve(seg_hits.size());
            for (const auto &h : seg_hits)
            {
                if (u.empty() || norm(h.p - u.back().p) > EPS_MERGE)
                    u.push_back(h);
            }
            seg_hits.swap(u);
        }
    }
    return hits;
}

TrenchGraph build_trench_strict(const Roads &roads, double boundary_step)
{
    TrenchGraph g;

    std::vector<std::vector<Pt>> sampled;
    sampled.reserve(roads.polygons.size());
    std::vector<std::vector<char>> keep;
    keep.reserve(roads.polygons.size());

    for (int i = 0; i < (int)roads.polygons.size(); ++i)
    {
        auto s = sample_ring(roads.polygons[i].ring, boundary_step);
        sampled.push_back(s);

        std::vector<char> k(s.size(), 1);
        for (int j = 0; j < (int)s.size(); ++j)
        {
            if (inside_any_other(s[j], roads, i))
                k[j] = 0;
        }
        keep.push_back(std::move(k));
    }

    auto hits = collect_cross_hits(sampled);

    std::unordered_map<long long, int> node_index;
    node_index.reserve(200000);

    for (int selfIdx = 0; selfIdx < (int)roads.polygons.size(); ++selfIdx)
    {
        const auto &s = sampled[selfIdx];
        const auto &k = keep[selfIdx];
        if (s.size() < 2)
            continue;
        int n = (int)s.size();

        auto id_of = [&](Pt p) -> int
        { return add_node_dedup(g, node_index, p); };

        for (int i = 0; i < n; ++i)
        {
            Pt A = s[i], B = s[(i + 1) % n];

            std::vector<int> chain_ids;
            chain_ids.reserve(4 + hits[selfIdx][i].size());

            if (k[i])
                chain_ids.push_back(id_of(A));

            for (const auto &h : hits[selfIdx][i])
            {
                chain_ids.push_back(id_of(h.p));
            }

            if (k[(i + 1) % n])
                chain_ids.push_back(id_of(B));

            for (int t = 1; t < (int)chain_ids.size(); ++t)
            {
                int u = chain_ids[t - 1], v = chain_ids[t];
                if (u == v)
                    continue;
                Seg seg{g.nodes[u], g.nodes[v]};
                if (!seg_crosses_other_roads(seg, roads, selfIdx))
                {
                    g.edges.emplace_back(u, v);
                }
            }
        }
    }

    return g;
}
