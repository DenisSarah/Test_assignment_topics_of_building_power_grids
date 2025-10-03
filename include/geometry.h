#pragma once
#include <cmath>
#include <algorithm>

struct Pt
{
    double x{0}, y{0};
};

inline double dot(const Pt &a, const Pt &b) { return a.x * b.x + a.y * b.y; }
inline double cross(const Pt &a, const Pt &b) { return a.x * b.y - a.y * b.x; }
inline Pt operator+(Pt a, Pt b) { return {a.x + b.x, a.y + b.y}; }
inline Pt operator-(Pt a, Pt b) { return {a.x - b.x, a.y - b.y}; }
inline Pt operator*(Pt a, double k) { return {a.x * k, a.y * k}; }
inline double norm2(Pt a) { return dot(a, a); }
inline double norm(Pt a) { return std::sqrt(norm2(a)); }


struct Seg
{
    Pt a, b;
};

inline double seg_len(const Seg &s) { return norm(s.b - s.a); }

inline int sgn(double v, double eps = 1e-9) { return (v > eps) - (v < -eps); }

inline bool on_segment(Pt p, Seg s)
{
    if (sgn(cross(s.b - s.a, p - s.a)) != 0)
        return false;
    return (std::min(s.a.x, s.b.x) - 1e-9 <= p.x && p.x <= std::max(s.a.x, s.b.x) + 1e-9 &&
            std::min(s.a.y, s.b.y) - 1e-9 <= p.y && p.y <= std::max(s.a.y, s.b.y) + 1e-9);
}

inline bool seg_intersect(Seg s1, Seg s2, Pt *ip = nullptr)
{
    Pt r = s1.b - s1.a;
    Pt s = s2.b - s2.a;
    double rxs = cross(r, s);
    double qpxr = cross(s2.a - s1.a, r);
    if (std::fabs(rxs) < 1e-12)
    {
        if (std::fabs(qpxr) < 1e-12)
        {
            auto between = [&](double a, double b, double c)
            { return (std::min(a, b) - 1e-12 <= c && c <= std::max(a, b) + 1e-12); };
            if (between(s1.a.x, s1.b.x, s2.a.x) && between(s1.a.y, s1.b.y, s2.a.y))
            {
                if (ip)
                    *ip = s2.a;
                return true;
            }
            if (between(s1.a.x, s1.b.x, s2.b.x) && between(s1.a.y, s1.b.y, s2.b.y))
            {
                if (ip)
                    *ip = s2.b;
                return true;
            }
            if (between(s2.a.x, s2.b.x, s1.a.x) && between(s2.a.y, s2.b.y, s1.a.y))
            {
                if (ip)
                    *ip = s1.a;
                return true;
            }
            if (between(s2.a.x, s2.b.x, s1.b.x) && between(s2.a.y, s2.b.y, s1.b.y))
            {
                if (ip)
                    *ip = s1.b;
                return true;
            }
            return false;
        }
        return false;
    }
    double t = cross(s2.a - s1.a, s) / rxs;
    double u = cross(s2.a - s1.a, r) / rxs;
    if (t >= -1e-12 && t <= 1 + 1e-12 && u >= -1e-12 && u <= 1 + 1e-12)
    {
        if (ip)
            *ip = s1.a + r * t;
        return true;
    }
    return false;
}

inline bool point_in_polygon(const Pt &p, const std::vector<Pt> &ring)
{
    bool c = false;
    int n = (int)ring.size();
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        Pt a = ring[i], b = ring[j];
        if (on_segment(p, {a, b}))
            return false;
        bool cond = ((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-18) + a.x);
        if (cond)
            c = !c;
    }
    return c;
}
