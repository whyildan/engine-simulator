#include "ui/Mesh3D.h"
#include <algorithm>
#include <cmath>

namespace es::ui::mesh {

using namespace m3;
static constexpr float kTau = 6.28318530718f;

Data lathe(const Prof& prof, int seg) {
    Data d;
    for (size_t k = 0; k + 1 < prof.size(); ++k) {
        const float r0 = prof[k].first, y0 = prof[k].second, r1 = prof[k + 1].first, y1 = prof[k + 1].second;
        const float dr = r1 - r0, dy = y1 - y0;
        if (std::fabs(dr) < 1e-7f && std::fabs(dy) < 1e-7f) continue;
        // outward normal in the (r, y) plane: rotate the segment by -90 deg
        float nr = dy, ny = -dr;
        const float nl = std::sqrt(nr * nr + ny * ny); nr /= nl; ny /= nl;
        const uint32_t base = (uint32_t)d.v.size();
        for (int s = 0; s <= seg; ++s) {
            const float a = kTau * s / seg, c = std::cos(a), sn = std::sin(a);
            d.add(v3(r0 * c, y0, r0 * sn), v3(nr * c, ny, nr * sn));
            d.add(v3(r1 * c, y1, r1 * sn), v3(nr * c, ny, nr * sn));
        }
        for (int s = 0; s < seg; ++s) {
            const uint32_t a = base + 2 * s, b = a + 1, c = a + 2, e = a + 3;
            d.tri(a, c, b); d.tri(b, c, e);
        }
    }
    return d;
}

// ear clipping (the outline is made counter-clockwise first)
static std::vector<int> triangulate(const Poly& p) {
    const int n = (int)p.size();
    std::vector<int> idx(n), out;
    double area = 0;
    for (int k = 0; k < n; ++k) { const auto& a = p[k]; const auto& b = p[(k + 1) % n]; area += a.first * b.second - b.first * a.second; }
    for (int k = 0; k < n; ++k) idx[k] = area >= 0 ? k : n - 1 - k;
    auto crossv = [&](int a, int b, int c) {
        return (p[b].first - p[a].first) * (p[c].second - p[a].second) - (p[b].second - p[a].second) * (p[c].first - p[a].first);
    };
    auto inside = [&](int a, int b, int c, int q) {
        const float c1 = (p[b].first - p[a].first) * (p[q].second - p[a].second) - (p[b].second - p[a].second) * (p[q].first - p[a].first);
        const float c2 = (p[c].first - p[b].first) * (p[q].second - p[b].second) - (p[c].second - p[b].second) * (p[q].first - p[b].first);
        const float c3 = (p[a].first - p[c].first) * (p[q].second - p[c].second) - (p[a].second - p[c].second) * (p[q].first - p[c].first);
        return c1 >= 0 && c2 >= 0 && c3 >= 0;
    };
    int guard = 0;
    while (idx.size() > 3 && guard++ < 10000) {
        bool cut = false;
        const int m = (int)idx.size();
        for (int k = 0; k < m; ++k) {
            const int a = idx[(k + m - 1) % m], b = idx[k], c = idx[(k + 1) % m];
            if (crossv(a, b, c) <= 1e-9f) continue;   // reflex
            bool any = false;
            for (int q : idx) if (q != a && q != b && q != c && inside(a, b, c, q)) { any = true; break; }
            if (any) continue;
            out.push_back(a); out.push_back(b); out.push_back(c);
            idx.erase(idx.begin() + k);
            cut = true;
            break;
        }
        if (!cut) break;   // degenerate: fan the rest
    }
    for (size_t k = 1; k + 1 < idx.size(); ++k) { out.push_back(idx[0]); out.push_back(idx[k]); out.push_back(idx[k + 1]); }
    return out;
}

Data extrude(const Poly& poly, float z0, float z1) {
    Data d;
    const int n = (int)poly.size();
    if (n < 3) return d;
    const std::vector<int> t = triangulate(poly);
    for (int cap = 0; cap < 2; ++cap) {
        const float z = cap ? z1 : z0;
        const V3 nrm = v3(0, 0, cap ? 1.0f : -1.0f);
        const uint32_t base = (uint32_t)d.v.size();
        for (const auto& q : poly) d.add(v3(q.first, q.second, z), nrm);
        for (size_t k = 0; k + 2 < t.size(); k += 3) {
            if (cap) d.tri(base + t[k], base + t[k + 1], base + t[k + 2]);
            else d.tri(base + t[k], base + t[k + 2], base + t[k + 1]);
        }
    }
    double area = 0;
    for (int k = 0; k < n; ++k) { const auto& a = poly[k]; const auto& b = poly[(k + 1) % n]; area += a.first * b.second - b.first * a.second; }
    const float sgn = area >= 0 ? 1.0f : -1.0f;
    for (int k = 0; k < n; ++k) {
        const auto& a = poly[k]; const auto& b = poly[(k + 1) % n];
        const V3 e = v3(b.first - a.first, b.second - a.second, 0);
        const V3 nrm = norm(v3(e.y, -e.x, 0) * sgn);
        const uint32_t base = (uint32_t)d.v.size();
        d.add(v3(a.first, a.second, z0), nrm); d.add(v3(b.first, b.second, z0), nrm);
        d.add(v3(b.first, b.second, z1), nrm); d.add(v3(a.first, a.second, z1), nrm);
        d.tri(base, base + 1, base + 2); d.tri(base, base + 2, base + 3);
    }
    return d;
}

Data tube(const std::vector<V3>& path, const std::vector<float>& radius, int sides, bool closed, bool caps) {
    Data d;
    const int n = (int)path.size();
    if (n < 2) return d;
    // tangents and a parallel-transported frame
    std::vector<V3> T(n), X(n), Y(n);
    for (int k = 0; k < n; ++k) {
        const V3 a = path[closed ? (k - 1 + n) % n : std::max(0, k - 1)], b = path[closed ? (k + 1) % n : std::min(n - 1, k + 1)];
        T[k] = norm(b - a);
    }
    const V3 ref = std::fabs(T[0].y) < 0.9f ? v3(0, 1, 0) : v3(1, 0, 0);
    X[0] = norm(cross(T[0], ref)); Y[0] = cross(T[0], X[0]);
    for (int k = 1; k < n; ++k) {
        V3 x = X[k - 1] - T[k] * dot(X[k - 1], T[k]);
        X[k] = norm(x); Y[k] = cross(T[k], X[k]);
    }
    const int rings = closed ? n + 1 : n;
    const uint32_t base = (uint32_t)d.v.size();
    for (int kk = 0; kk < rings; ++kk) {
        const int k = kk % n;
        const float r = radius[std::min<size_t>(k, radius.size() - 1)];
        for (int s = 0; s <= sides; ++s) {
            const float a = kTau * s / sides;
            const V3 nn = X[k] * std::cos(a) + Y[k] * std::sin(a);
            d.add(path[k] + nn * r, nn);
        }
    }
    for (int k = 0; k + 1 < rings; ++k)
        for (int s = 0; s < sides; ++s) {
            const uint32_t a = base + k * (sides + 1) + s, b = a + 1, c = a + (sides + 1), e = c + 1;
            d.tri(a, b, c); d.tri(b, e, c);
        }
    if (caps && !closed)
        for (int end = 0; end < 2; ++end) {
            const int k = end ? n - 1 : 0;
            const V3 nrm = end ? T[k] : -T[k];
            const float r = radius[std::min<size_t>(k, radius.size() - 1)];
            const uint32_t c0 = d.add(path[k], nrm);
            const uint32_t b0 = (uint32_t)d.v.size();
            for (int s = 0; s <= sides; ++s) {
                const float a = kTau * s / sides;
                d.add(path[k] + (X[k] * std::cos(a) + Y[k] * std::sin(a)) * r, nrm);
            }
            for (int s = 0; s < sides; ++s) {
                if (end) d.tri(c0, b0 + s, b0 + s + 1); else d.tri(c0, b0 + s + 1, b0 + s);
            }
        }
    return d;
}

Data box(float hx, float hy, float hz) {
    Poly p = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
    return extrude(p, -hz, hz);
}

Data twisted(const Poly& poly, float z0, float z1, float twist, int slices) {
    Data d;
    const int n = (int)poly.size();
    const uint32_t base = (uint32_t)d.v.size();
    for (int s = 0; s <= slices; ++s) {
        const float t = (float)s / slices, z = z0 + (z1 - z0) * t, a = twist * t, c = std::cos(a), sn = std::sin(a);
        for (int k = 0; k < n; ++k) {
            const auto& q = poly[k];
            const auto& qa = poly[(k + n - 1) % n];
            const auto& qb = poly[(k + 1) % n];
            const float ex = qb.first - qa.first, ey = qb.second - qa.second;
            const float nx = ey, ny = -ex, nl = std::sqrt(nx * nx + ny * ny) + 1e-9f;
            d.add(v3(q.first * c - q.second * sn, q.first * sn + q.second * c, z), v3((nx * c - ny * sn) / nl, (nx * sn + ny * c) / nl, 0));
        }
    }
    for (int s = 0; s < slices; ++s)
        for (int k = 0; k < n; ++k) {
            const uint32_t a = base + s * n + k, b = base + s * n + (k + 1) % n, c = a + n, e = b + n;
            d.tri(a, b, e); d.tri(a, e, c);
        }
    // end caps (flat)
    Data e0 = extrude(poly, z0, z0 + 1e-3f);
    M4 rot = rotZ(twist);
    Data e1 = extrude(poly, z1 - 1e-3f, z1);
    append(d, e0, M4{});
    append(d, e1, rot);
    return d;
}

void append(Data& dst, const Data& src, const M4& xf) {
    const uint32_t base = (uint32_t)dst.v.size();
    const M4 nm = normalMatrix(xf);
    for (const Vtx& q : src.v) {
        const V3 p = xform(xf, v3(q.px, q.py, q.pz)), n = norm(xdir(nm, v3(q.nx, q.ny, q.nz)));
        dst.v.push_back({p.x, p.y, p.z, n.x, n.y, n.z});
    }
    for (uint32_t k : src.i) dst.i.push_back(base + k);
}

Poly circleOutline(float r, int n, float cx, float cy) {
    Poly p;
    for (int k = 0; k < n; ++k) { const float a = kTau * k / n; p.push_back({cx + r * std::cos(a), cy + r * std::sin(a)}); }
    return p;
}

Poly gearOutline(float rRoot, float rTip, int teeth) {
    Poly p;
    for (int t = 0; t < teeth; ++t)
        for (int k = 0; k < 4; ++k) {
            static const float frac[4] = {0.0f, 0.18f, 0.5f, 0.68f};
            const bool tip = k == 1 || k == 2;
            const float a = kTau * (t + frac[k]) / teeth;
            const float r = tip ? rTip : rRoot;
            p.push_back({r * std::cos(a), r * std::sin(a)});
        }
    return p;
}

Poly roundedRect(float hx, float hy, float r, int perCorner) {
    Poly p;
    r = std::min(r, std::min(hx, hy));
    const float cx[4] = {hx - r, -hx + r, -hx + r, hx - r}, cy[4] = {hy - r, hy - r, -hy + r, -hy + r};
    for (int c = 0; c < 4; ++c)
        for (int k = 0; k <= perCorner; ++k) {
            const float a = kTau * 0.25f * (c + (float)k / perCorner);
            p.push_back({cx[c] + r * std::cos(a), cy[c] + r * std::sin(a)});
        }
    return p;
}

} // namespace es::ui::mesh
