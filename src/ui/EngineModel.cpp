// Parametric 3D engine model (GPU engine view). Units: mm. World: crank axis along z, y up, x across.
// Palette: the app's own (dark background, neutral steel / silver, the orange accent for heat and combustion, blue for
// the fresh charge) - see the engine view plan in the project notes.
#include "ui/EngineModel.h"
#include "ui/EngineModelParts.h"
#include "ui/EngineView3D.h"
#include "sim/Exhaust.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <unordered_map>

namespace es::ui {

using namespace m3;
using gpu::Material;
using mesh::Data;
using mesh::Poly;
using mesh::Prof;

namespace {

constexpr float kPiF = 3.14159265358979f;
constexpr float kDegF = kPiF / 180.0f;

} // namespace

// ------------------------------------------------------------------ shared pieces (EngineModelParts.h)
namespace model {

namespace { std::unordered_map<std::string, Cached> g_cache; }

const Cached& meshOf(const std::string& key, const std::function<Data()>& make) {
    auto it = g_cache.find(key);
    if (it != g_cache.end()) return it->second;
    const Data d = make();
    Cached c;
    c.id = gpu::meshCreate(d);
    c.lo = v3(1e9f, 1e9f, 1e9f); c.hi = v3(-1e9f, -1e9f, -1e9f);
    for (const auto& q : d.v) {
        c.lo = v3(std::min(c.lo.x, q.px), std::min(c.lo.y, q.py), std::min(c.lo.z, q.pz));
        c.hi = v3(std::max(c.hi.x, q.px), std::max(c.hi.y, q.py), std::max(c.hi.z, q.pz));
    }
    return g_cache.emplace(key, c).first->second;
}
void cacheTrim() { if (g_cache.size() > 600 || gpu::meshCount() > 800) { gpu::meshClear(); g_cache.clear(); } }

std::string key(const char* name, std::initializer_list<float> v) {
    std::string s = name;
    char b[24];
    for (float x : v) { std::snprintf(b, sizeof(b), "|%.2f", x); s += b; }
    return s;
}

void Scene::put(const Cached& c, const M4& w, const Material& m) {
    if (c.id < 0) return;
    out->items.push_back({c.id, w, m});
    for (int k = 0; k < 8; ++k) {
        const V3 q = xform(w, v3(k & 1 ? c.hi.x : c.lo.x, k & 2 ? c.hi.y : c.lo.y, k & 4 ? c.hi.z : c.lo.z));
        if (fit) out->hull.push_back(q);
        out->lo = v3(std::min(out->lo.x, q.x), std::min(out->lo.y, q.y), std::min(out->lo.z, q.z));
        out->hi = v3(std::max(out->hi.x, q.x), std::max(out->hi.y, q.y), std::max(out->hi.z, q.z));
    }
}

Material mat(float r, float g, float b, float metal, float rough) { Material m; m.r = r; m.g = g; m.b = b; m.metal = metal; m.rough = rough; return m; }
const Material kSteel = mat(0.60f, 0.61f, 0.64f, 0.90f, 0.32f);
const Material kSilver = mat(0.80f, 0.81f, 0.84f, 0.85f, 0.24f);
const Material kSteelDark = mat(0.30f, 0.31f, 0.34f, 0.70f, 0.45f);
const Material kFrame = mat(0.22f, 0.23f, 0.25f, 0.55f, 0.55f);
const Material kGold = mat(0.85f, 0.70f, 0.30f, 0.90f, 0.30f);
const Material kRubber = mat(0.10f, 0.10f, 0.11f, 0.0f, 0.75f);
const Material kCeramic = mat(0.88f, 0.88f, 0.86f, 0.0f, 0.35f);
const Material kAlu = mat(0.74f, 0.75f, 0.78f, 0.80f, 0.38f);
const Material kHotPipe = mat(0.44f, 0.45f, 0.47f, 0.80f, 0.38f);
Material glassMat(float a, float r, float g, float b) { Material m = mat(r, g, b, 0.0f, 0.08f); m.a = a; m.glass = true; return m; }
Material emissive(Material m, float r, float g, float b) { m.er = r; m.eg = g; m.eb = b; return m; }
const float kAccent[3] = {1.0f, 0.62f, 0.25f};     // the app's orange
const float kCharge[3] = {0.30f, 0.55f, 0.95f};    // fresh charge blue (the existing gas colours)

namespace { uint32_t hash32(uint32_t x) { x ^= x >> 16; x *= 0x7feb352d; x ^= x >> 15; x *= 0x846ca68b; x ^= x >> 16; return x; } }
float rnd(uint32_t a, uint32_t b) { return (hash32(a * 9781u + b * 6271u + 17u) & 0xffffff) / 16777216.0f; }

std::vector<V3> spline(const std::vector<V3>& c, int per) {
    std::vector<V3> out;
    const int n = (int)c.size();
    if (n < 2) return c;
    for (int k = 0; k + 1 < n; ++k) {
        const V3 p0 = c[std::max(0, k - 1)], p1 = c[k], p2 = c[k + 1], p3 = c[std::min(n - 1, k + 2)];
        for (int j = 0; j < per; ++j) {
            const float t = (float)j / per, t2 = t * t, t3 = t2 * t;
            out.push_back((p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) * 0.5f);
        }
    }
    out.push_back(c.back());
    return out;
}

} // namespace model

namespace {
using namespace model;

Poly hull2(Poly p) {   // convex hull (monotone chain), counter-clockwise
    std::sort(p.begin(), p.end());
    if (p.size() < 3) return p;
    Poly h(2 * p.size());
    size_t k = 0;
    auto cr = [](const std::pair<float, float>& o, const std::pair<float, float>& a, const std::pair<float, float>& b) {
        return (a.first - o.first) * (b.second - o.second) - (a.second - o.second) * (b.first - o.first);
    };
    for (size_t i = 0; i < p.size(); ++i) { while (k >= 2 && cr(h[k - 2], h[k - 1], p[i]) <= 0) --k; h[k++] = p[i]; }
    for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) { while (k >= t && cr(h[k - 2], h[k - 1], p[i - 1]) <= 0) --k; h[k++] = p[i - 1]; }
    h.resize(k - 1);
    return h;
}
Poly circlesHull(std::initializer_list<std::tuple<float, float, float>> cs, int n = 28) {
    Poly all;
    for (const auto& c : cs) for (const auto& q : mesh::circleOutline(std::get<2>(c), n, std::get<0>(c), std::get<1>(c))) all.push_back(q);
    return hull2(all);
}


double liftAt(double phi, double center, double dur, double lift) {
    const double d = wrapSym(phi - center, 720.0), h = dur * 0.5;
    if (std::fabs(d) >= h) return 0;
    return lift * std::pow(0.5 * (1.0 + std::cos(kPi * d / h)), 0.75);
}

// gas colour by stroke (phi: cycle angle, 0 = firing TDC): rgb + alpha + how much it glows
struct Gas { float r, g, b, a, burn; };
Gas gasAt(double phi, double adv, double cycle) {
    const double since = wrapPos(phi + adv, cycle);
    const double burn = cycle > 800 ? 130.0 : 90.0;
    if (since < burn) { const float t = (float)(1 - since / burn); return {kAccent[0], 0.45f + 0.30f * t, 0.12f, 0.35f + 0.45f * t, t}; }
    const double q = cycle / 4;
    if (phi < q) return {0.85f, 0.45f, 0.15f, 0.22f, 0.15f};          // late expansion
    if (phi < 2 * q) return {0.45f, 0.38f, 0.33f, 0.28f, 0.0f};       // exhaust
    if (phi < 3 * q) return {kCharge[0], kCharge[1], kCharge[2], 0.22f, 0.0f};   // intake
    return {kCharge[0] * 1.05f, kCharge[1] * 1.05f, kCharge[2], 0.30f, 0.0f};    // compression
}

// two-stroke: burn, blowdown once the exhaust port opens, fresh charge from the transfers, compression
Gas gasTwoStroke(double phi, double adv, double epo, double tpo) {
    const double since = wrapPos(phi + adv, 360.0);
    if (since < 75.0) { const float t = (float)(1 - since / 75.0); return {kAccent[0], 0.45f + 0.30f * t, 0.12f, 0.35f + 0.45f * t, t}; }
    if (phi < epo) return {0.85f, 0.45f, 0.15f, 0.22f, 0.15f};
    if (phi < tpo) return {0.45f, 0.38f, 0.33f, 0.28f, 0.0f};
    if (phi < 360.0 - tpo) return {kCharge[0], kCharge[1], kCharge[2], 0.24f, 0.0f};
    return {kCharge[0] * 1.05f, kCharge[1] * 1.05f, kCharge[2], 0.30f, 0.0f};
}

// ------------------------------------------------------------------ meshes
Data pistonMesh(float B, bool bowl) {
    const float R = 0.497f * B, pinOff = 0.30f * B, skirt = -0.34f * B;
    Prof p = {{0, skirt + 0.06f * B}, {R * 0.80f, skirt + 0.04f * B}, {R * 0.97f, skirt}, {R * 0.985f, pinOff - 0.22f * B}};
    // three ring grooves near the crown
    for (int g = 2; g >= 0; --g) {
        const float y = pinOff - 0.055f * B - g * 0.05f * B;
        p.push_back({R, y - 0.025f * B}); p.push_back({R * 0.955f, y - 0.022f * B}); p.push_back({R * 0.955f, y}); p.push_back({R, y + 0.003f * B});
    }
    p.push_back({R, pinOff - 0.01f * B});
    if (bowl) {   // diesel: flat crown with a re-entrant combustion bowl
        p.push_back({R * 0.98f, pinOff}); p.push_back({0.34f * B, pinOff}); p.push_back({0.31f * B, pinOff - 0.05f * B});
        p.push_back({0.30f * B, pinOff - 0.13f * B}); p.push_back({0.20f * B, pinOff - 0.19f * B}); p.push_back({0.07f * B, pinOff - 0.12f * B});
        p.push_back({0, pinOff - 0.10f * B});
    } else {      // spark ignition: a shallow dome
        p.push_back({R * 0.96f, pinOff}); p.push_back({0.32f * B, pinOff + 0.025f * B}); p.push_back({0, pinOff + 0.035f * B});
    }
    return mesh::lathe(p, 48);
}

Data linerMesh(float B, float y0, float y1, float wall) {
    const float ri = 0.5f * B, ro = ri + wall;
    return mesh::lathe({{ri, y0}, {ro, y0}, {ro, y1}, {ri, y1}, {ri, y0}}, 48);
}

Data rodMesh(float B, float l) {
    // I-section look: a slimmer web between two flanged ends
    Data d = mesh::extrude(circlesHull({{0, 0, 0.29f * B}, {0, l, 0.155f * B}}), -0.055f * B, 0.055f * B);
    mesh::append(d, mesh::extrude(circlesHull({{0, 0, 0.27f * B}, {0, l, 0.14f * B}}), -0.10f * B, 0.10f * B), M4{});
    // big end cap bolts
    for (int s = -1; s <= 1; s += 2) mesh::append(d, mesh::box(0.035f * B, 0.11f * B, 0.035f * B), translate(v3(s * 0.23f * B, -0.02f * B, 0.06f * B)));
    return d;
}

Data throwMesh(float B, float r, float web) {
    const float pinR = 0.20f * B;
    Poly fan;
    for (int k = 0; k <= 16; ++k) { const float a = kPiF * (1.5f + (-0.40f + 0.80f * k / 16.0f)); fan.push_back({1.30f * r * std::cos(a), 1.30f * r * std::sin(a)}); }
    for (const auto& q : mesh::circleOutline(0.36f * B, 24, 0, r)) fan.push_back(q);
    for (const auto& q : mesh::circleOutline(0.30f * B, 24)) fan.push_back(q);
    const Poly webOut = hull2(fan);
    const float zi = 0.115f * B;
    Data d = mesh::extrude(webOut, zi, zi + web);
    mesh::append(d, mesh::extrude(webOut, -zi - web, -zi), M4{});
    // crank pin between the webs
    mesh::append(d, mesh::extrude(mesh::circleOutline(pinR, 28, 0, r), -zi, zi), M4{});
    return d;
}

Data valveMesh(float B, float dia) {
    const float R = std::min(0.5f * dia, 0.23f * B), stem = 0.032f * B, L = 0.66f * B;
    return mesh::lathe({{0, 0}, {R, 0}, {R, 0.018f * B}, {R * 0.55f, 0.05f * B}, {stem, 0.16f * B}, {stem, L}, {0, L}}, 32);
}

// one turn of a valve spring: a wire ring (the spring is drawn as turns stacked over its height, so the wire keeps
// its thickness however far the spring is compressed)
Data springTurnMesh(float B) {
    std::vector<V3> path;
    const float rc = 0.085f * B;
    for (int k = 0; k < 24; ++k) { const float a = 2 * kPiF * k / 24; path.push_back(v3(rc * std::cos(a), 0, rc * std::sin(a))); }
    return mesh::tube(path, std::vector<float>(path.size(), 0.013f * B), 8, true, false);
}

Data lobeMesh(float Rb, float rn, float nd, float w) { return mesh::extrude(circlesHull({{0, 0, Rb}, {nd, 0, rn}}, 32), -0.5f * w, 0.5f * w); }

Data rodZ(float r, float z0, float z1, int seg = 24) { return mesh::extrude(mesh::circleOutline(r, seg), z0, z1); }

Data gearMesh(float r, float teethH, int teeth, float w, float hubR) {
    Data d = mesh::extrude(mesh::gearOutline(r - teethH, r, teeth), -0.5f * w, 0.5f * w);
    mesh::append(d, mesh::extrude(mesh::circleOutline(hubR, 24), -0.9f * w, 0.9f * w), M4{});
    return d;
}

Data sparkPlugMesh(float B) {
    const float s = B / 86.0f;   // a real plug is ~14 mm thread: scale gently with the bore
    return mesh::lathe({{0, -2 * s}, {1.2f * s, -2 * s}, {1.2f * s, 0}, {7 * s, 0}, {7 * s, 19 * s}, {10.4f * s, 19 * s}, {10.4f * s, 27 * s},
                        {5.5f * s, 30 * s}, {5.0f * s, 52 * s}, {2.5f * s, 56 * s}, {0, 56 * s}}, 24);
}

Data injectorMesh(float B) {
    const float s = B / 80.0f;
    return mesh::lathe({{0, -1 * s}, {2.5f * s, -1 * s}, {3 * s, 6 * s}, {6 * s, 8 * s}, {6 * s, 34 * s}, {9 * s, 38 * s}, {9 * s, 60 * s}, {4 * s, 64 * s}, {0, 64 * s}}, 24);
}

// rocker arm in its pivot frame: cam / pushrod end vC, valve end vT, a hub on the pivot
Data rockerMesh(V3 vC, V3 vT, float B) {
    Data d = mesh::tube({vC, v3(0, 0.02f * B, 0), vT}, {0.026f * B, 0.036f * B, 0.026f * B}, 10);
    mesh::append(d, rodZ(0.042f * B, -0.045f * B, 0.045f * B, 16), M4{});
    mesh::append(d, mesh::box(0.03f * B, 0.015f * B, 0.03f * B), translate(vT - v3(0, 0.01f * B, 0)));
    return d;
}

// cooling fins: thin rings about +y from y0 to y1, `pitchY` apart
Data finsMesh(float ri, float ro, float y0, float y1, float pitchY, float t) {
    Data d;
    for (float y = y0; y <= y1; y += pitchY) mesh::append(d, mesh::lathe({{ri, y}, {ro, y}, {ro, y + t}, {ri, y + t}, {ri, y}}, 40), M4{});
    return d;
}

// closed path around a set of pulleys (circle centre x, y, radius): the belt / chain line
// A serpentine belt / chain round pulleys (x, y, r) in the plane z. The pulleys on the outside (their circle reaches the
// convex hull) are wrapped on their outside, in hull order; a pulley inside the hull (a boxer's crank sprocket between
// the cams) is put into the loop on the stretch it is nearest to and wrapped on its back - the belt bends round it
// (the plain hull used to pass it by without touching). Tangents between circles with the centres on the chosen
// sides, arcs round each pulley.
std::vector<V3> beltPath(const std::vector<std::tuple<float, float, float>>& pulleys, float z, const std::vector<int>* force = nullptr,
                         bool preferLow = false) {
    struct C { float x, y, r; int s; };   // s: +1 outside (centre left of the travel), -1 back side
    const int n = (int)pulleys.size();
    auto forced = [&](int k) { return force && k < (int)force->size() ? (*force)[k] : 0; };
    std::vector<V3> out;
    if (n < 2) return out;
    Poly all;
    std::vector<int> owner;
    for (int k = 0; k < n; ++k) {
        if (forced(k) < 0) continue;   // (idlers / tensioners pressing on the back of the belt: not part of the outline)
        for (const auto& q : mesh::circleOutline(std::get<2>(pulleys[k]), 40, std::get<0>(pulleys[k]), std::get<1>(pulleys[k]))) { all.push_back(q); owner.push_back(k); }
    }
    const Poly h = hull2(all);
    // hull order (counter-clockwise) of the outside pulleys
    std::vector<int> ring;
    for (const auto& q : h) {
        int k = -1;
        for (size_t j = 0; j < all.size(); ++j) if (all[j] == q) { k = owner[j]; break; }
        if (k >= 0 && (ring.empty() || ring.back() != k) && (ring.size() < 2 || ring.front() != k)) ring.push_back(k);
    }
    std::vector<C> loop;
    for (int k : ring) loop.push_back({std::get<0>(pulleys[k]), std::get<1>(pulleys[k]), std::get<2>(pulleys[k]), 1});
    // inside pulleys: into the stretch whose line passes nearest
    for (int k = 0; k < n; ++k) {
        if (std::find(ring.begin(), ring.end(), k) != ring.end()) continue;
        const float px = std::get<0>(pulleys[k]), py = std::get<1>(pulleys[k]);
        int best = 0; float bd = 1e18f;
        for (size_t a = 0; a < loop.size(); ++a) {
            const C& A = loop[a]; const C& Bc = loop[(a + 1) % loop.size()];
            const float dx = Bc.x - A.x, dy = Bc.y - A.y, L2 = std::max(1e-6f, dx * dx + dy * dy);
            const float t = clampv(((px - A.x) * dx + (py - A.y) * dy) / L2, 0.0f, 1.0f);
            const float ex = A.x + t * dx - px, ey = A.y + t * dy - py;
            // (preferLow: a stretch below the pulley wins a near tie - a boxer's crank is wrapped from underneath)
            const float d = (ex * ex + ey * ey) * ((preferLow && 0.5f * (A.y + Bc.y) > py) ? 2.0f : 1.0f);
            if (d < bd) { bd = d; best = (int)a; }
        }
        loop.insert(loop.begin() + best + 1, {px, py, std::get<2>(pulleys[k]), forced(k) > 0 ? 1 : -1});
    }
    const int m = (int)loop.size();
    if (m < 2) return out;
    // tangent from circle A to circle B (centre of each on its side s): points pa, pb
    auto tangent = [](const C& A, const C& Bc, float& pax, float& pay, float& pbx, float& pby) {
        const float Dx = Bc.x - A.x, Dy = Bc.y - A.y, D2 = Dx * Dx + Dy * Dy;
        const float k = Bc.s * Bc.r - A.s * A.r;
        const float t = std::sqrt(std::max(1e-6f, D2 - k * k));
        // d = D (t - i k) / (t^2 + k^2), L = i d (the left normal)
        const float dx = (Dx * t + Dy * k) / D2, dy = (Dy * t - Dx * k) / D2;
        const float lx = -dy, ly = dx;
        pax = A.x - A.s * A.r * lx; pay = A.y - A.s * A.r * ly;
        pbx = Bc.x - Bc.s * Bc.r * lx; pby = Bc.y - Bc.s * Bc.r * ly;
    };
    std::vector<float> inX(m), inY(m), outX(m), outY(m);
    for (int a = 0; a < m; ++a) {
        const int b = (a + 1) % m;
        tangent(loop[a], loop[b], outX[a], outY[a], inX[b], inY[b]);
    }
    for (int a = 0; a < m; ++a) {
        const C& c = loop[a];
        // arc round the pulley from where the belt arrives to where it leaves: counter-clockwise on an outside
        // pulley, clockwise on the back of an inside one
        float a0 = std::atan2(inY[a] - c.y, inX[a] - c.x), a1 = std::atan2(outY[a] - c.y, outX[a] - c.x);
        float sweep = a1 - a0;
        if (c.s > 0) { while (sweep < 0) sweep += 2 * kPiF; }
        else { while (sweep > 0) sweep -= 2 * kPiF; }
        const int steps = std::max(2, (int)(std::fabs(sweep) / (2 * kPiF) * 40));
        for (int j = 0; j <= steps; ++j) {
            const float an = a0 + sweep * j / steps;
            out.push_back(v3(c.x + c.r * std::cos(an), c.y + c.r * std::sin(an), z));
        }
    }
    return out;
}

// ------------------------------------------------------------------ piston engines
struct CylGeo { float z, axDeg; int bank; };

void buildPistonEngine(Scene& sc, const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live, float glassA,
                       EngineBits& eb) {
    const int n = clampv(p.cylinders, 1, kMaxCyl);
    const bool boxer = p.layout == Layout::Boxer, vee = p.layout == Layout::V;
    const bool diesel = isDiesel(p), two = isTwoStroke(p);
    const double cycle = two ? 360.0 : 720.0;
    const float r = p.stroke_mm * 0.5f, l = std::max(p.rod_mm, r * 1.2f), B = p.bore_mm;
    const float pinOff = 0.30f * B;
    const float deck = r + l + pinOff + 0.03f * B;
    const float wall = 0.07f * B;
    // the liner reaches down past the skirt at BDC (the crank webs swing ~1.3 r: they stay clear of it)
    const float linerBottom = std::max(1.32f * r + 0.05f * B, l - r - 0.36f * B);
    int bankCount[2] = {0, 0}, column[kMaxCyl];
    for (int i = 0; i < n; ++i) { const int b = p.layout == Layout::Inline ? 0 : clampv(p.bank[i], 0, 1); column[i] = bankCount[b]++; }
    const int cols = std::max(1, std::max(bankCount[0], bankCount[1]));
    const float pitch = B * 1.32f, bankOff = (vee || boxer) ? B * 0.46f : 0.0f;
    const float zOf0 = -(cols - 1) * pitch * 0.5f;
    const float zFront = zOf0 - pitch * 0.72f - bankOff * 0.5f, zRear = -zOf0 + pitch * 0.72f + bankOff * 0.5f;
    const float vtec = p.vtec ? clampv(live.vtecBlend, 0.0f, 1.0f) : 0.0f;
    const double inC = 360.0 + p.intakeCenterline - p.intakeCamAdvance - (p.vtc ? live.vtcAdvance : 0.0f);
    const double exC = 360.0 - p.exhaustCenterline - p.exhaustCamAdvance;
    const float crankRad = -crankDeg * kDegF;
    eb.zFront = zFront; eb.zRear = zRear; eb.B = B; eb.r = r; eb.vee = vee; eb.boxer = boxer;
    eb.crankFront = v3(0, 0, zFront - 0.25f * B);

    std::vector<CylGeo> cyl(n);
    for (int i = 0; i < n; ++i) {
        const int b = p.layout == Layout::Inline ? 0 : clampv(p.bank[i], 0, 1);
        float ax = 0;
        if (vee) ax = (b == 0 ? -1.0f : 1.0f) * p.bankAngle * 0.5f;
        if (boxer) ax = b == 0 ? -90.0f : 90.0f;
        cyl[i] = {zOf0 + column[i] * pitch + (b == 0 ? -0.5f : 0.5f) * bankOff, ax, b};
    }

    // ---- shared meshes
    const Cached& mPiston = meshOf(key("piston", {B, diesel ? 1.0f : 0.0f}), [&] { return pistonMesh(B, diesel); });
    const Cached& mRod = meshOf(key("rod", {B, l}), [&] { return rodMesh(B, l); });
    const Cached& mThrow = meshOf(key("throw", {B, r}), [&] { return throwMesh(B, r, 0.15f * B); });
    const Cached& mLiner = meshOf(key("liner", {B, linerBottom, deck, wall}), [&] { return linerMesh(B, linerBottom, deck, wall); });
    const Cached& mGas = meshOf(key("gas", {B}), [&] { return mesh::lathe({{0, 0}, {0.492f * B, 0}, {0.492f * B, 1}, {0, 1}}, 40); });
    const Cached& mPin = meshOf(key("pin", {B}), [&] { return rodZ(0.11f * B, -0.21f * B, 0.21f * B); });
    const Cached& mBigEnd = meshOf(key("bigend", {B}), [&] { return mesh::lathe({{0.205f * B, -0.1f * B}, {0.215f * B, -0.1f * B}, {0.215f * B, 0.1f * B}, {0.205f * B, 0.1f * B}}, 28); });

    // ---- crankshaft: main journals between the throws, gold main bearings, front sprocket, rear flywheel
    {
        // main journals from the first main bearing back; in front of it the snout (thinner: the timing sprocket and
        // the pulley sit on it - the full-size journal used to swallow the crank sprocket)
        const float zMain0 = zOf0 - 0.5f * pitch - 0.06f * B;
        const Cached& mMain = meshOf(key("mainjournal", {B, zMain0, zRear}), [&] { return rodZ(0.25f * B, zMain0, zRear + 0.2f * B, 32); });
        sc.put(mMain, rotZ(crankRad), kSteel);
        if (!two) {
            const Cached& mSnout = meshOf(key("cranksnout", {B, zFront, zMain0}), [&] { return rodZ(0.11f * B, zFront - 0.34f * B, zMain0 + 0.01f * B, 24); });
            sc.put(mSnout, rotZ(crankRad), kSteel);
        }
        const Cached& mBear = meshOf(key("mainbear", {B}), [&] { return mesh::extrude(mesh::circleOutline(0.30f * B, 28), -0.06f * B, 0.06f * B); });
        for (int c = 0; c <= cols; ++c) {
            const float z = zOf0 + (c - 0.5f) * pitch;
            sc.put(mBear, translate(v3(0, 0, z)), kGold);
        }
    }
    // ---- cradle frame under the crank (the app's dark steel, no block in the way)
    {
        const float half = (vee || boxer) ? 1.15f * B : 0.75f * B, yb = -1.45f * r - 0.25f * B;
        const float len = zRear - zFront + 0.6f * B;
        const Cached& mRail = meshOf(key("rail", {len, B}), [&] { return mesh::box(0.06f * B, 0.06f * B, 0.5f * len); });
        const Cached& mCross = meshOf(key("cross", {half, B}), [&] { return mesh::box(half, 0.05f * B, 0.05f * B); });
        const Cached& mPost = meshOf(key("post", {r, B}), [&] { return mesh::box(0.05f * B, 0.5f * (-yb), 0.05f * B); });
        for (int s = -1; s <= 1; s += 2) sc.put(mRail, translate(v3(s * half, yb, 0.5f * (zFront + zRear))), kFrame);
        for (int c = 0; c <= cols; ++c) {
            const float z = zOf0 + (c - 0.5f) * pitch;
            sc.put(mCross, translate(v3(0, yb, z)), kFrame);
            sc.put(mPost, translate(v3(0, 0.5f * yb, z)), kFrame);
        }
    }

    // ---- per cylinder
    struct CamLine { float x, y, axDeg, sgn; int bank; bool shaft; };   // camshaft (or rocker shaft) lines in the bank plane
    const bool ohv = p.camLayout == CamLayout::OHV && !two, sohc = p.camLayout == CamLayout::SOHC && !two, dohc = !ohv && !sohc;
    const bool air = p.cooling == Cooling::Air;
    const Material finM = glassMat(std::min(0.85f, 0.22f + glassA * 1.6f), 0.74f, 0.76f, 0.80f);
    // OHV: one cam in the block - in the V's valley above the crank, under the crank of a boxer, beside an inline's
    // OHV pushrods run parallel to the bore, just outside it on the intake (valley) side, at kPushX from the bore axis;
    // a V's one cam sits where that line meets for both banks: kPushX / sin(half the V angle) above the crank
    const float kPushX = 0.5f * B + wall + 0.06f * B;
    const float halfV = std::max(5.0f, 0.5f * std::fabs(p.bankAngle)) * kDegF;
    V3 ohvCam = v3(0, std::min(3.0f * B, kPushX / std::sin(halfV)), 0), ohvCamXY = v3(0, 0, 0);
    if (boxer) ohvCam.y = -(1.30f * r + 0.30f * B);
    bool ohvCamSet = false;
    std::vector<CamLine> camLines;
    float valveTop = deck;
    for (int i = 0; i < n; ++i) {
        const CylGeo& cg = cyl[i];
        const M4 cf = translate(v3(0, 0, cg.z)) * rotZ(-cg.axDeg * kDegF);   // cylinder frame: +y up the bore
        const double phi = wrapPos(crankDeg - phases[i], cycle);
        const float th = (float)(wrapPos(phi, 360.0) * kDeg);
        const float sn = std::sin(th), cs = std::cos(th);
        const float dist = r * cs + std::sqrt(std::max(1.0f, l * l - r * r * sn * sn));
        const V3 pin = v3(r * sn, r * cs, 0);
        // crank throw (rotates with the crank)
        sc.put(mThrow, cf * rotZ(-th), kSteel);
        // rod: big end on the pin, small end on the piston pin
        const V3 sm = v3(0, dist, 0), dv = sm - pin;
        const M4 rodW = cf * translate(pin) * rotZ(std::atan2(-dv.x, dv.y));
        sc.put(mRod, rodW, kSteel);
        sc.put(mBigEnd, cf * translate(pin) * rotX(kPiF * 0.5f), kGold);
        // piston and pin
        sc.put(mPiston, cf * translate(sm), kSilver);
        sc.put(mPin, cf * translate(sm), kSteelDark);
        // liner (glass) and the gas column above the crown
        sc.put(mLiner, cf, glassMat(glassA * 0.9f));
        if (air) {   // air cooling: fins round the barrel (and the two-stroke's head dome)
            const float fb = std::max(linerBottom + 0.15f * B, deck - 1.05f * B);
            const Cached& mFins = meshOf(key("barrelfins", {B, fb, deck}), [&] { return finsMesh(0.5f * B + wall, 0.5f * B + wall + 0.27f * B, fb, deck - 0.05f * B, 0.085f * B, 0.022f * B); });
            sc.put(mFins, cf, finM);
            if (two) {
                const Cached& mHf = meshOf(key("2theadfins", {B, deck}), [&] { return finsMesh(0.25f * B, 0.5f * B + wall + 0.30f * B, deck + 0.02f * B, deck + 0.26f * B, 0.06f * B, 0.02f * B); });
                sc.put(mHf, cf, finM);
            }
        }
        const float crownTop = dist + pinOff + (diesel ? 0.0f : 0.03f * B);
        const Gas gas = two ? gasTwoStroke(phi, live.advance, p.tsExhaustOpen, p.tsTransferOpen) : gasAt(phi, live.advance, 720.0);
        if (deck - crownTop > 0.5f) {
            Material gm = mat(gas.r, gas.g, gas.b, 0.0f, 0.6f);
            gm.a = gas.a; gm.er = gas.r * gas.burn * 0.9f; gm.eg = gas.g * gas.burn * 0.7f; gm.eb = gas.b * gas.burn * 0.4f;
            sc.put(mGas, cf * translate(v3(0, crownTop, 0)) * scale(v3(1, deck - crownTop, 1)), gm);
        }
        // combustion: sparkles in the burning charge (the accent), a few charge dots on the intake stroke
        const double since = wrapPos(phi + live.advance, cycle);
        if (since < 110.0) {
            const float k = 1.0f - (float)(since / 110.0);
            const int nDots = 14;
            for (int d = 0; d < nDots; ++d) {
                const float a = rnd(i, d) * 2 * kPiF, rr = std::sqrt(rnd(i + 31, d)) * 0.45f * B;
                const float y = crownTop + rnd(i + 57, d) * std::max(1.0f, deck - crownTop);
                const V3 pw = xform(cf, v3(rr * std::cos(a), y, rr * std::sin(a)));
                sc.glow(pw, 0.05f * B * (0.6f + 0.6f * rnd(i + 3, d)), kAccent[0], 0.55f, 0.18f, 0.55f * k);
            }
            sc.glow(xform(cf, v3(0, 0.5f * (crownTop + deck), 0)), 0.55f * B, kAccent[0], 0.45f, 0.12f, 0.35f * k);
        } else if (two ? (phi >= p.tsTransferOpen && phi < 360.0 - p.tsTransferOpen) : (phi >= 360.0 && phi < 540.0)) {
            for (int d = 0; d < 6; ++d) {
                const float a = rnd(i + 91, d) * 2 * kPiF, rr = std::sqrt(rnd(i + 7, d)) * 0.42f * B;
                const float y = crownTop + rnd(i + 13, d) * std::max(1.0f, deck - crownTop);
                sc.glow(xform(cf, v3(rr * std::cos(a), y, rr * std::sin(a))), 0.035f * B, kCharge[0], kCharge[1], kCharge[2], 0.35f);
            }
        }

        if (two) {
            buildTwoStrokeCyl(sc, p, cf, phi, crankDeg, live, glassA, deck, crownTop, i);
            sc.label(xform(cf, v3(0, deck + 0.95f * B, 0)), std::to_string(i + 1));
            continue;
        }
        // ---- valvetrain: valves (intake one side, exhaust the other), springs, buckets, cam lobes. V / boxer: the
        //      exhaust side faces out on both banks (bank 0 mirrored)
        const float flip = ((vee || boxer) && cg.bank == 0) ? -1.0f : 1.0f;
        struct Side { float sgn; int count; float dia; double lift, maxLift, dur, center; bool intake; };
        double li = liftAt(phi, inC, p.intakeDuration, p.intakeLift_mm), le = liftAt(phi, exC, p.exhaustDuration, p.exhaustLift_mm);
        if (vtec > 0) {
            const double liHi = liftAt(phi, inC, p.vtecInDuration, p.vtecInLift_mm), leHi = liftAt(phi, exC, p.vtecExDuration, p.vtecExLift_mm);
            li += vtec * (std::max(liHi, li) - li); le += vtec * (std::max(leHi, le) - le);
        }
        const Side sides[2] = {
            {-flip, clampv((int)p.intakeValves, 1, 3), p.intakeValveDia_mm, li, p.intakeLift_mm + vtec * (p.vtecInLift_mm - p.intakeLift_mm),
             p.intakeDuration + vtec * (p.vtecInDuration - p.intakeDuration), inC, true},
            {flip, clampv((int)p.exhaustValves, 1, 3), p.exhaustValveDia_mm, le, p.exhaustLift_mm + vtec * (p.vtecExLift_mm - p.exhaustLift_mm),
             p.exhaustDuration + vtec * (p.vtecExDuration - p.exhaustDuration), exC, false}};
        const float stemL = 0.66f * B, bucketH = 0.075f * B, Rb = 0.12f * B, lifterH = 0.13f * B;
        const float sIn = -flip;   // the intake side (OHV: the pushrods come up on it)
        float cylTop = deck;
        for (const Side& s : sides) {
            // DOHC: splayed valves under their own cams; SOHC: one cam between them, rockers; OHV: nearly upright
            // valves in a row, rockers worked by pushrods from the cam in the block
            // (OHV: the valves stand upright in one row along the crank - intake then exhaust - as in a small-block V8 or
            //  a 6BT: every rocker then reaches across to its pushrod on the valley side, none crossing another)
            const int nIn = sides[0].count, nAll = sides[0].count + sides[1].count;
            const float rowSp = nAll > 1 ? std::min(0.44f * B, 0.88f * B / (float)nAll) : 0.0f;
            const float tilt = ohv ? 0.0f : s.sgn * (sohc ? 10.0f : 13.0f) * kDegF;
            const V3 ax = v3(std::sin(tilt), std::cos(tilt), 0);   // up the valve, towards its cam / rocker
            const float seatX = ohv ? (s.intake ? sIn : -sIn) * 0.05f * B : s.sgn * 0.23f * B;
            const float diaDraw = ohv && nAll > 1 ? std::min(s.dia, rowSp - 0.03f * B) : s.dia;
            const Cached& mValve = meshOf(key("valve", {B, diaDraw}), [&] { return valveMesh(B, diaDraw); });
            const Cached& mTurn = meshOf(key("springturn", {B}), [&] { return springTurnMesh(B); });
            const Cached& mRet = meshOf(key("retainer", {B}), [&] { return mesh::lathe({{0.03f * B, 0}, {0.10f * B, 0.01f * B}, {0.10f * B, 0.025f * B}, {0.03f * B, 0.03f * B}}, 24); });
            const float rn = Rb * (0.36f + 0.0011f * (float)s.dur);
            const float lift = (float)s.lift;
            const bool open = s.lift > 0.3;
            const double rot = wrapSym(phi - s.center, 720.0) * 0.5 * kDeg;
            const Material lobeM = open ? emissive(kSteel, 0.05f, 0.05f, 0.05f) : kSteel;
            for (int k = 0; k < s.count; ++k) {
                const float zoff = ohv ? ((s.intake ? k : nIn + k) - 0.5f * (nAll - 1)) * rowSp
                                       : s.count == 1 ? 0.0f : (s.count == 2 ? (k ? 0.21f : -0.21f) * B : (k - 1) * 0.27f * B);
                const V3 seat = v3(seatX, deck, zoff);
                const V3 face = seat - ax * lift;
                const M4 vf = cf * translate(face) * rotZ(-tilt);   // valve frame: +y up the stem
                Material vm = kSteel;
                if (open) vm = s.intake ? emissive(kSteel, kCharge[0] * 0.10f, kCharge[1] * 0.12f, kCharge[2] * 0.18f)
                                        : emissive(kSteel, kAccent[0] * 0.18f, kAccent[1] * 0.10f, kAccent[2] * 0.04f);
                sc.put(mValve, vf, vm);
                // spring from its seat in the head to the retainer (the retainer moves with the valve, the turns close up)
                const float sBot = 0.30f * B, sTop = stemL - 0.06f * B - lift;
                const int turns = 6;
                for (int j = 0; j < turns; ++j) {
                    const float y = sBot + (sTop - sBot) * (j + 0.5f) / turns;
                    sc.put(mTurn, cf * translate(seat + ax * y) * rotZ(-tilt) * rotX(0.06f), kSteelDark);
                }
                sc.put(mRet, vf * translate(v3(0, stemL - 0.06f * B, 0)), kSteel);
                const V3 tip0 = seat + ax * stemL;   // the stem's tip with the valve shut
                if (dohc) {
                    // bucket on the stem, the lobe right above it: base circle on the bucket when shut, the nose
                    // pushes it down to full lift
                    const Cached& mBucket = meshOf(key("bucketcup", {B}), [&] { return mesh::lathe({{0, bucketH}, {0.115f * B, bucketH}, {0.118f * B, bucketH - 0.01f * B},
                                                                                                   {0.118f * B, -0.11f * B}, {0.104f * B, -0.11f * B},
                                                                                                   {0.104f * B, bucketH - 0.025f * B}, {0, bucketH - 0.025f * B}}, 32); });
                    sc.put(mBucket, vf * translate(v3(0, stemL, 0)), kSilver);
                    const float nd = Rb + (float)s.maxLift - rn;
                    const Cached& mLobe = meshOf(key("lobe", {Rb, rn, nd, B}), [&] { return lobeMesh(Rb, rn, nd, 0.11f * B); });
                    const V3 cc = seat + ax * (stemL + bucketH + Rb);
                    sc.put(mLobe, cf * translate(cc) * rotZ(std::atan2(-ax.y, -ax.x) - (float)rot), lobeM);
                    cylTop = std::max(cylTop, cc.y + Rb);
                } else if (sohc) {
                    // rocker: pivot on a shaft beside the cam; the cam lifts its inner end, the outer end pushes the valve
                    const V3 P = v3(s.sgn * 0.17f * B, tip0.y + 0.10f * B, zoff);
                    const V3 vT = v3(tip0.x, tip0.y, zoff) - P, vC = v3(s.sgn * 0.03f * B, tip0.y + 0.07f * B, zoff) - P;
                    const Cached& mRock = meshOf(key("rockerS", {B, s.sgn, tilt}), [&] { return rockerMesh(vC, vT, B); });
                    sc.put(mRock, cf * translate(P) * rotZ(-lift / vT.x), kSilver);
                    const V3 contact = P + vC, cc = v3(0, contact.y - Rb - 0.025f * B, zoff);
                    const float nd = Rb + (float)s.maxLift * std::fabs(vC.x / vT.x) - rn;
                    const Cached& mLobe = meshOf(key("lobe", {Rb, rn, nd, B}), [&] { return lobeMesh(Rb, rn, nd, 0.09f * B); });
                    const V3 dn = norm(contact - cc);
                    sc.put(mLobe, cf * translate(cc) * rotZ(std::atan2(dn.y, dn.x) - (float)rot), lobeM);
                    cylTop = std::max(cylTop, P.y + 0.08f * B);
                    bool have = false;
                    for (const CamLine& c : camLines) if (c.bank == cg.bank && c.sgn == 0) have = true;
                    if (!have) camLines.push_back({0, cc.y, cg.axDeg, 0, cg.bank, false});
                    have = false;
                    for (const CamLine& c : camLines) if (c.bank == cg.bank && c.sgn == s.sgn && c.shaft) have = true;
                    if (!have) camLines.push_back({P.x, P.y, cg.axDeg, s.sgn, cg.bank, true});
                } else {
                    // OHV: rocker on a stud between the valve and its pushrod (1.5:1, the valve side longer); the pushrod
                    // stands parallel to the bore just outside it, on the valley side, and rises lift / 1.5 as the lifter
                    // rides the block cam's lobe
                    const V3 T0 = v3(tip0.x, tip0.y, zoff);
                    const float xc = sIn * kPushX;
                    const V3 P = v3(T0.x + 0.6f * (xc - T0.x), T0.y + 0.06f * B, zoff);
                    const V3 vT = T0 - P, vC = v3(xc - P.x, -0.06f * B, 0);
                    const float ratio = std::fabs(vC.x / vT.x);
                    const Cached& mRock = meshOf(key("rockerO", {B, vC.x, vT.x}), [&] { return rockerMesh(vC, vT, B); });
                    sc.put(mRock, cf * translate(P) * rotZ(-lift / vT.x), kSilver);
                    const V3 topW = xform(cf, P + vC - v3(0, 0.02f * B, 0));
                    V3 camW = (vee && !boxer) ? ohvCam : xform(cf, v3(xc, 1.30f * r + 0.30f * B, 0));
                    camW.z = topW.z;
                    const V3 dir = norm(topW - camW);
                    const float pl = lift * ratio;   // pushrod / lifter lift
                    const float Lp = std::max(0.2f * B, len(topW - camW) - Rb - lifterH);
                    const Cached& mLifter = meshOf(key("lifter", {B}), [&] { return mesh::lathe({{0, 0}, {0.065f * B, 0}, {0.065f * B, lifterH}, {0, lifterH}}, 20); });
                    sc.put(mLifter, alongY(camW + dir * (Rb + pl), dir), kSteelDark);
                    const Cached& mPush = meshOf(key("pushrod", {B, Lp}), [&] { return mesh::tube({v3(0, 0, 0), v3(0, Lp, 0)}, {0.022f * B, 0.022f * B}, 10); });
                    sc.put(mPush, alongY(camW + dir * (Rb + lifterH + pl), dir), kSteel);
                    const float nd = Rb + (float)s.maxLift * ratio - rn;
                    const Cached& mLobe = meshOf(key("lobe", {Rb, rn, nd, B}), [&] { return lobeMesh(Rb, rn, nd, 0.09f * B); });
                    sc.put(mLobe, translate(camW) * rotZ(std::atan2(dir.y, dir.x) - (float)rot), lobeM);
                    cylTop = std::max(cylTop, P.y + 0.08f * B);
                    if (!ohvCamSet) { ohvCamSet = true; ohvCamXY = camW; }
                }
            }
            if (dohc) {   // camshaft line for this side (one per bank, from the first cylinder of the bank)
                const V3 cc = v3(seatX, deck, 0) + ax * (stemL + bucketH + Rb);
                bool have = false;
                for (const CamLine& c : camLines) if (c.bank == cg.bank && c.sgn == s.sgn) have = true;
                if (!have) camLines.push_back({cc.x, cc.y, cg.axDeg, s.sgn, cg.bank, false});
            }
        }
        valveTop = std::max(valveTop, cylTop);
        if (air) {   // finned head round each cylinder's valvetrain, a glass rocker box on top
            const float ht = cylTop + 0.06f * B;
            const Cached& mHf = meshOf(key("headfins", {B, deck, ht}), [&] { return finsMesh(0.56f * B, 0.86f * B, deck + 0.04f * B, ht - 0.10f * B, 0.075f * B, 0.022f * B); });
            sc.put(mHf, cf, finM);
            const Cached& mBox = meshOf(key("rockerbox", {B, deck, ht}), [&] { return mesh::lathe({{0, deck}, {0.55f * B, deck}, {0.55f * B, ht}, {0.45f * B, ht + 0.06f * B}, {0, ht + 0.06f * B}}, 36); });
            sc.put(mBox, cf, glassMat(glassA * 0.8f, 0.62f, 0.64f, 0.68f));
        }
        // where the manifolds meet the head (exhaust on the exhaust valves' side)
        {
            const V3 pe = xform(cf, v3(flip * 0.70f * B, deck + 0.22f * B, 0)), de = norm(xdir(cf, v3(flip, 0.15f, 0)));
            const V3 pi = xform(cf, v3(-flip * 0.70f * B, deck + 0.26f * B, 0)), di = norm(xdir(cf, v3(-flip, 0.35f, 0)));
            eb.ex.push_back({pe, de, i, cg.bank});
            eb.in.push_back({pi, di, i, cg.bank});
            // port stubs through the head wall
            const Cached& mStubE = meshOf(key("stubE", {B}), [&] { return mesh::tube({v3(0, 0, 0), v3(0, 0.30f * B, 0)}, {0.13f * B, 0.15f * B}, 16); });
            sc.put(mStubE, cf * translate(v3(flip * 0.42f * B, deck + 0.19f * B, 0)) * rotZ(-flip * 1.42f), kHotPipe);
            const Cached& mStubI = meshOf(key("stubI", {B}), [&] { return mesh::tube({v3(0, 0, 0), v3(0, 0.30f * B, 0)}, {0.14f * B, 0.15f * B}, 16); });
            sc.put(mStubI, cf * translate(v3(-flip * 0.42f * B, deck + 0.22f * B, 0)) * rotZ(flip * 1.25f), kAlu);
        }
        // spark plug (or injector + glow plug)
        if (diesel) {
            const Cached& mInj = meshOf(key("injector", {B}), [&] { return injectorMesh(B); });
            sc.put(mInj, cf * translate(v3(0, deck - 0.02f * B, 0)), kSteelDark);
            eb.injTop.push_back(xform(cf, v3(0, deck + 0.77f * B, 0)));
            const Cached& mGlowPlug = meshOf(key("glowplug", {B}), [&] { return mesh::lathe({{0, 0}, {0.02f * B, 0}, {0.03f * B, 0.12f * B}, {0.06f * B, 0.14f * B}, {0.06f * B, 0.55f * B}, {0, 0.55f * B}}, 20); });
            const float gt = live.glowPlugOn ? 1.0f : 0.0f;
            sc.put(mGlowPlug, cf * translate(v3(0.12f * B, deck - 0.05f * B, 0.12f * B)) * rotZ(-0.35f), gt > 0 ? emissive(kSteelDark, 0.9f, 0.35f, 0.08f) : kSteelDark);
            eb.glowTop.push_back(xform(cf * translate(v3(0.12f * B, deck - 0.05f * B, 0.12f * B)) * rotZ(-0.35f), v3(0, 0.55f * B, 0)));
            if (gt > 0) sc.glow(xform(cf, v3(0.12f * B, deck - 0.05f * B, 0.12f * B)), 0.06f * B, kAccent[0], 0.5f, 0.15f, 0.9f);
            // injection spray: a cone of droplets from the nozzle while the fuel goes in (start of injection
            // ~ the advance, ~20 deg long)
            const double sinceInj = wrapPos(phi + live.advance, 720.0);
            if (sinceInj < 22.0) {
                for (int j = 0; j < 6; ++j) {   // six nozzle holes
                    const float a = j * kPiF / 3.0f;
                    const V3 dir = norm(v3(std::cos(a) * 0.85f, -0.45f, std::sin(a) * 0.85f));
                    for (int d = 1; d <= 6; ++d) {
                        const float t = d / 6.0f * 0.33f * B * (float)(0.4 + sinceInj / 22.0);
                        sc.glow(xform(cf, v3(0, deck - 0.03f * B, 0) + dir * t), 0.025f * B, 0.85f, 0.80f, 0.55f, 0.55f);
                    }
                }
            }
        } else {
            const Cached& mPlug = meshOf(key("plug", {B}), [&] { return sparkPlugMesh(B); });
            const M4 pf = cf * translate(v3(0, deck - 0.01f * B, 0));
            sc.put(mPlug, pf, kCeramic);
            const double sinceSpark = wrapPos(phi + live.advance, 720.0);
            if (sinceSpark < 12.0) sc.glow(xform(cf, v3(0, deck - 0.04f * B, 0)), 0.16f * B, 1.0f, 0.92f, 0.60f, 1.0f - (float)(sinceSpark / 12.0));
        }
        // cylinder number over the head
        sc.label(xform(cf, v3(0, valveTop + 0.35f * B, 0)), std::to_string(i + 1));
    }

    eb.headTop = valveTop + 0.08f * B;
    // ---- per bank: glass head (a liquid-cooled engine's: one box over the bank), camshafts / rocker shafts with gold
    //      bearing caps, cam sprockets; the timing drive (belt, chain or gears) at the front
    std::vector<std::tuple<float, float, float>> pulleys;   // (x, y, r) in the front plane
    if (two) camLines.clear();
    std::vector<std::pair<V3, float>> sprocketsAt;            // world centre, radius (drawn below)
    // cam sprockets (and a diesel's pump pulley, 0.85 of them) twice the crank sprocket (2:1: the cams turn at half
    // crank speed) - 0.48 bore, or smaller where two cams sit closer than that (DOHC: ~0.85 bore apart): they keep a
    // gap for the belt / chain between them (gears: just clear), and the crank sprocket shrinks with them
    std::vector<std::pair<V3, float>> sprCentres;            // centre (front plane), size relative to a cam sprocket
    std::vector<int> sprBank;                                // its bank (-1: the block cam, a pump)
    for (int b = 0; b < ((vee || boxer) ? 2 : 1) && !two; ++b) {
        float ax = 0;
        bool any = false;
        for (int i = 0; i < n; ++i) if (cyl[i].bank == b) { ax = cyl[i].axDeg; any = true; break; }
        if (!any) continue;
        const M4 bf = rotZ(-ax * kDegF);   // bank plane rotation (about the crank axis)
        float zb0 = 1e9f, zb1 = -1e9f;
        for (int i = 0; i < n; ++i) if (cyl[i].bank == b) { zb0 = std::min(zb0, cyl[i].z); zb1 = std::max(zb1, cyl[i].z); }
        if (!air) {   // head: a glass box over the valvetrain
            const float hy0 = deck, hy1 = valveTop + 0.08f * B, hw = 0.68f * B;
            const Cached& mHead = meshOf(key("head", {hw, hy1 - hy0, zb1 - zb0, B}), [&] {
                Data d = mesh::extrude(mesh::roundedRect(hw, 0.5f * (hy1 - hy0), 0.06f * B), -(0.5f * (zb1 - zb0) + 0.62f * B), 0.5f * (zb1 - zb0) + 0.62f * B);
                return d; });
            sc.put(mHead, bf * translate(v3(0, 0.5f * (hy0 + hy1), 0.5f * (zb0 + zb1))), glassMat(glassA * 0.75f, 0.62f, 0.64f, 0.68f));
        }
        for (const CamLine& c : camLines) {
            if (c.bank != b) continue;
            const float z0 = zb0 - 0.75f * B, z1 = zb1 + 0.55f * B;
            const M4 at = bf * translate(v3(c.x, c.y, 0));
            if (c.shaft) {   // SOHC rocker shaft: stationary
                const Cached& mRs = meshOf(key("rockershaft", {B, z0 + 0.3f * B, z1}), [&] { return rodZ(0.03f * B, z0 + 0.3f * B, z1, 12); });
                sc.put(mRs, at, kSteelDark);
                continue;
            }
            const Cached& mShaft = meshOf(key("camshaft", {B, z0, z1}), [&] { return rodZ(0.045f * B, z0, z1, 16); });
            sc.put(mShaft, at * rotZ(crankRad * 0.5f), kSteel);
            const Cached& mCap = meshOf(key("camcap", {B}), [&] { return mesh::box(0.07f * B, 0.06f * B, 0.05f * B); });
            for (int i = 0; i < n; ++i) if (cyl[i].bank == b) sc.put(mCap, at * translate(v3(0, 0.02f * B, cyl[i].z + 0.5f * pitch * 0.85f)), kGold);
            const V3 w = xform(bf, v3(c.x, c.y, 0));
            sprCentres.push_back({v3(w.x, w.y, zFront), 1.0f});
            sprBank.push_back(b);
        }
    }
    if (ohv && ohvCamSet) {   // the block cam: one shaft along the engine
        const Cached& mShaft = meshOf(key("blockcam", {B, zFront, zRear}), [&] { return rodZ(0.05f * B, zFront, zRear - 0.2f * B, 16); });
        sc.put(mShaft, translate(v3(ohvCamXY.x, ohvCamXY.y, 0)) * rotZ(crankRad * 0.5f), kSteel);
        sprCentres.push_back({v3(ohvCamXY.x, ohvCamXY.y, zFront), 1.0f});
        sprBank.push_back(-1);
    }
    // diesel: the distributor injection pump on the intake side, driven at cam speed by the same belt / chain
    if (diesel && !two) {
        const float sideX = (vee || boxer) ? 0.0f : -1.0f;   // inline: the intake side (-x)
        const V3 pc = v3(sideX * 1.05f * B, (vee || boxer) ? valveTop + 0.4f * B : 0.52f * deck, zFront);
        sprCentres.push_back({pc, 0.85f});
        sprBank.push_back(-1);
        eb.pump = pc; eb.hasPump = true;
    }
    float camSprR = 0.48f * B;
    {
        const float gap = (!two && p.camDrive == CamDrive::Gears) ? 0.01f * B : 0.07f * B;
        for (size_t a = 0; a < sprCentres.size(); ++a) {
            const V3 ca = sprCentres[a].first;
            // against the crank sprocket (half size) and every other sprocket
            const float dc = len(v3(ca.x, ca.y, 0));
            camSprR = std::min(camSprR, (dc - gap) / (sprCentres[a].second + 0.5f));
            for (size_t b2 = a + 1; b2 < sprCentres.size(); ++b2) {
                const V3 cb = sprCentres[b2].first;
                camSprR = std::min(camSprR, (len(v3(ca.x - cb.x, ca.y - cb.y, 0)) - gap) / (sprCentres[a].second + sprCentres[b2].second));
            }
        }
        camSprR = std::max(camSprR, 0.30f * B);   // (the crank sprocket, half of it, stays well over the 0.11 bore snout)
    }
    const float crankSprR = 0.5f * camSprR;
    // A chain-driven V or boxer has a chain per bank (Ford 4.6 Modular: two primary chains off one wide crank sprocket;
    // Subaru FA20: bank 1 and bank 2 chains, each with its tensioner and guides; Nissan VQ35DE: a primary chain to both
    // banks plus a short chain per bank) - here one chain per bank, in two planes just apart, from a two-track crank
    // sprocket. A belt-driven V / boxer keeps its one belt round both banks with idlers (Toyota 1UZ-FE, Honda J35,
    // Subaru EJ).
    const bool chainPerBank = !two && (vee || boxer) && p.camDrive == CamDrive::Chain &&
                              std::count(sprBank.begin(), sprBank.end(), 0) > 0 && std::count(sprBank.begin(), sprBank.end(), 1) > 0;
    const float zBank[2] = {zFront - 0.065f * B, zFront + 0.065f * B};
    pulleys.push_back({0, 0, crankSprR});
    for (size_t k = 0; k < sprCentres.size(); ++k) {
        V3 c = sprCentres[k].first;
        if (chainPerBank) c.z = zBank[sprBank[k] == 1 ? 1 : 0];
        pulleys.push_back({c.x, c.y, camSprR * sprCentres[k].second});
        sprocketsAt.push_back({c, camSprR * sprCentres[k].second});
    }
    {
        const CamDrive drive = two ? CamDrive::Belt : p.camDrive;
        const bool gears = drive == CamDrive::Gears;
        const float crankSprW = chainPerBank ? 0.26f * B : 0.12f * B;   // (two tracks for two chains)
        const Cached& mCrankSpr = meshOf(key("crankspr", {crankSprR, crankSprW, B}), [&] { return gearMesh(crankSprR, 0.04f * B, 20, crankSprW, 0.12f * B); });
        if (!two) sc.put(mCrankSpr, translate(v3(0, 0, zFront)) * rotZ(crankRad), kSteel);
        if (!two && p.induction != Induction::Supercharger) {
            // the accessory belt pulley / harmonic balancer: a ribbed rim, a rubber ring, the hub
            const Cached& mPul = meshOf(key("frontpulley", {B}), [&] {
                Data d = mesh::extrude(mesh::gearOutline(0.33f * B, 0.345f * B, 48), -0.07f * B, 0.07f * B);
                mesh::append(d, mesh::extrude(mesh::circleOutline(0.15f * B, 24), -0.10f * B, 0.10f * B), M4{});
                return d; });
            sc.put(mPul, translate(v3(0, 0, zFront - 0.25f * B)) * rotZ(crankRad), kSteelDark);
            const Cached& mDamp = meshOf(key("damperring", {B}), [&] { return mesh::extrude(mesh::circleOutline(0.27f * B, 36), -0.075f * B, 0.075f * B); });
            sc.put(mDamp, translate(v3(0, 0, zFront - 0.25f * B)), kRubber);
        }
        for (const auto& sp : sprocketsAt) {
            const Cached& mCamSpr = meshOf(key("camspr", {sp.second, B}), [&] { return gearMesh(sp.second, 0.04f * B, (int)(sp.second / (0.012f * B)), 0.12f * B, 0.10f * B); });
            sc.put(mCamSpr, translate(sp.first) * rotZ(crankRad * 0.5f * (gears ? -1.0f : 1.0f)), kSteel);
        }
        if (gears && !two) {
            // gear train: every cam (and pump) gear is driven from the nearest gear already in the train - the crank
            // gear, or a gear it drives (a 6BT's pump gear runs off the cam gear) - through idlers on the line between
            // them; a route whose idlers would hit another gear is not used (the next nearest source is tried)
            struct G { V3 c; float r, sgn; };                    // centre (front plane), radius, turning sense
            std::vector<G> train = {{v3(0, 0, 0), crankSprR, 1.0f}};
            std::vector<G> all = train;
            for (const auto& sp : sprocketsAt) all.push_back({v3(sp.first.x, sp.first.y, 0), sp.second, -1.0f});
            std::vector<int> order(sprocketsAt.size());
            for (size_t k = 0; k < order.size(); ++k) order[k] = (int)k;
            std::sort(order.begin(), order.end(), [&](int a, int b2) { return len(all[a + 1].c) < len(all[b2 + 1].c); });
            auto clearOf = [&](V3 c, float r, int skipA, int skipB) {
                for (size_t g = 0; g < all.size(); ++g) {
                    if ((int)g == skipA || (int)g == skipB) continue;
                    if (len(c - all[g].c) < r + all[g].r + 0.01f * B) return false;
                }
                return true;
            };
            for (int t : order) {
                const G tgt = all[t + 1];
                // sources: the crank (index 0) and the sprockets already connected, nearest first
                std::vector<int> src = {0};
                for (size_t g = 1; g < all.size(); ++g) if ((int)g != t + 1 && all[g].sgn > 1.5f) src.push_back((int)g);
                std::sort(src.begin(), src.end(), [&](int a, int b2) { return len(all[a].c - tgt.c) < len(all[b2].c - tgt.c); });
                for (int s0 : src) {
                    const G from = all[s0];
                    const float d = len(tgt.c - from.c), gap = d - from.r - tgt.r;
                    std::vector<G> idl;
                    if (gap >= 0.03f * B) {
                        const int k = std::max(1, (int)std::lround(gap / (0.7f * B)));
                        const float ri = 0.5f * gap / k;
                        const V3 u = (tgt.c - from.c) * (1.0f / std::max(1e-3f, d));
                        for (int j = 0; j < k; ++j) idl.push_back({from.c + u * (from.r + ri + 2.0f * ri * j), ri, (j % 2 == 0 ? -1.0f : 1.0f) * (s0 == 0 ? 1.0f : -1.0f)});
                    }
                    bool ok = true;
                    for (const G& g : idl) ok = ok && clearOf(g.c, g.r, s0, t + 1);
                    if (!ok) continue;
                    for (const G& g : idl) {
                        all.push_back(g);
                        const Cached& mIdl = meshOf(key("idler", {g.r, B}), [&] { return gearMesh(g.r, 0.04f * B, std::max(10, (int)(g.r / (0.012f * B))), 0.12f * B, 0.06f * B); });
                        sc.put(mIdl, translate(v3(g.c.x, g.c.y, zFront)) * rotZ(crankRad * (crankSprR / g.r) * -g.sgn), kSteel);
                    }
                    all[t + 1].sgn = 2.0f;   // connected: it can drive the next one
                    break;
                }
            }
        }
        // idlers and the tensioner, after the engines these layouts come from:
        //  - V: an idler between the banks under the intake's throttle body - the run across the top dips under it
        //    instead of cutting through the throttle (Toyota 1UZ-FE: crank, No.1 and No.2 idlers, cam pulleys, tensioner)
        //  - boxer: idlers either side of the crank, the belt round the crank's underside (Subaru EJ DOHC: an automatic
        //    tensioner and three idlers)
        //  - a belt: a tensioner pulley on an arm pressing on its longest run; a chain: a pivoting tensioner arm and a
        //    fixed guide on its two long runs (Honda K20: tensioner arm, upper / lower guides)
        const bool chainDrive = drive == CamDrive::Chain;
        struct Run { V3 a, b; };
        auto runsOf = [&](const std::vector<V3>& pth) {   // the long straight runs (between the arcs), longest first
            std::vector<Run> rr;
            for (size_t k = 0; k < pth.size(); ++k) {
                const V3 a0 = pth[k], bq = pth[(k + 1) % pth.size()];
                if (len(bq - a0) > 0.35f * B) rr.push_back({a0, bq});
            }
            std::sort(rr.begin(), rr.end(), [](const Run& x, const Run& y) { return len(x.b - x.a) > len(y.b - y.a); });
            return rr;
        };
        auto centroidOf = [&](const std::vector<V3>& pth) {
            V3 c = v3(0, 0, 0);
            for (const V3& q : pth) c = c + q * (1.0f / std::max<size_t>(1, pth.size()));
            return c;
        };
        auto outwardOf = [&](const Run& rn, V3 cen) {   // unit normal of a run, pointing out of the loop
            const V3 d = norm(rn.b - rn.a), nrm = v3(-d.y, d.x, 0), mid = (rn.a + rn.b) * 0.5f;
            return (nrm.x * (mid.x - cen.x) + nrm.y * (mid.y - cen.y)) > 0 ? nrm : nrm * -1.0f;
        };
        // the belt / chain itself, with its teeth or rollers moving at the crank sprocket's surface speed
        auto drawLoop = [&](const std::vector<V3>& path, bool chain, int salt) {
            std::vector<float> rad(path.size(), chain ? 0.012f * B : 0.022f * B);
            const Cached& mBelt = meshOf(key(chain ? "chain" : "belt", {B, (float)path.size(), path[0].x, path[0].y, path[1].x, path[0].z, (float)salt}), [&] {
                if (!chain) return mesh::tube(path, rad, 10, true, false);
                Data d;   // a chain: two side plates' lines
                for (int sd = -1; sd <= 1; sd += 2) {
                    std::vector<V3> q = path;
                    for (V3& v : q) v.z += sd * 0.03f * B;
                    mesh::append(d, mesh::tube(q, rad, 6, true, false), M4{});
                }
                return d; });
            sc.put(mBelt, M4{}, chain ? kSteelDark : kRubber);
            std::vector<float> acc(path.size() + 1, 0.0f);
            for (size_t k = 0; k < path.size(); ++k) acc[k + 1] = acc[k] + len(path[(k + 1) % path.size()] - path[k]);
            const float total = acc.back(), step = chain ? 0.07f * B : 0.10f * B;
            const float shift = std::fmod(crankDeg * kDegF * crankSprR, step);
            const Cached& mTooth = meshOf(key(chain ? "roller" : "tooth", {B}), [&] {
                return chain ? rodZ(0.016f * B, -0.035f * B, 0.035f * B, 8) : mesh::box(0.018f * B, 0.018f * B, 0.05f * B); });
            for (float sd = shift; sd < total; sd += step) {
                size_t k = 0;
                while (k + 1 < acc.size() && acc[k + 1] < sd) ++k;
                const V3 a0 = path[k % path.size()], bpt = path[(k + 1) % path.size()];
                const float t = (sd - acc[k]) / std::max(1e-6f, acc[k + 1] - acc[k]);
                sc.put(mTooth, translate(lerp(a0, bpt, t)), chain ? kSteel : kRubber);
            }
        };
        // a chain's tensioner arm (a shoe with its hydraulic plunger) on its longest run, a fixed guide on the next
        auto drawShoes = [&](const std::vector<V3>& path) {
            const std::vector<Run> rr = runsOf(path);
            const V3 cen = centroidOf(path);
            for (size_t k = 0; k < rr.size() && k < 2; ++k) {
                const Run& rn = rr[k];
                const float L = len(rn.b - rn.a) * 0.72f;
                const V3 mid = (rn.a + rn.b) * 0.5f, nrm = outwardOf(rn, cen);
                const float ang = std::atan2(rn.b.y - rn.a.y, rn.b.x - rn.a.x);
                const Cached& mShoe = meshOf(key("chainshoe", {L, B}), [&] { return mesh::box(0.5f * L, 0.025f * B, 0.06f * B); });
                sc.put(mShoe, translate(mid + nrm * (0.045f * B)) * rotZ(ang), kSteelDark);
                if (k == 0) {
                    const V3 endp = lerp(rn.a, rn.b, 0.82f) + nrm * (0.07f * B);
                    const Cached& mPl = meshOf(key("tensplunger", {B}), [&] { return mesh::tube({v3(0, 0, 0), v3(0, 0.22f * B, 0)}, {0.045f * B, 0.045f * B}, 14); });
                    sc.put(mPl, alongY(endp, nrm), kAlu);
                }
            }
        };
        if (chainPerBank && !gears) {
            for (int bk = 0; bk < 2; ++bk) {
                std::vector<std::tuple<float, float, float>> pb = {{0.0f, 0.0f, crankSprR}};
                for (size_t k = 0; k < sprocketsAt.size(); ++k)
                    if ((sprBank[k] == 1 ? 1 : 0) == bk) pb.push_back({sprocketsAt[k].first.x, sprocketsAt[k].first.y, sprocketsAt[k].second});
                const std::vector<V3> pth = beltPath(pb, zBank[bk]);
                if (pth.size() < 4) continue;
                drawShoes(pth);
                drawLoop(pth, true, 10 + bk);
            }
        } else if (!two && !gears) {
            std::vector<int> force(pulleys.size(), 0);
            struct Extra { V3 c; float r; int kind; V3 runDir; };   // kind 1 idler, 2 tensioner pulley
            std::vector<Extra> extras;
            auto addPulley = [&](float x, float y, float r, int s2, int kind) {
                pulleys.push_back({x, y, r}); force.push_back(s2); extras.push_back({v3(x, y, zFront), r, kind, v3(0, 1, 0)});
            };
            if (vee && !eb.in.empty()) {
                float py = 0;
                for (const PortOut& q : eb.in) py += q.p.y;
                py /= (float)eb.in.size();
                const float thrY = py + 0.55f * B, rt = clampv(p.throttleBore_mm * 0.5f, 0.15f * B, 0.45f * B);
                float top = -1e9f;
                for (const auto& sp : sprocketsAt) top = std::max(top, sp.first.y + sp.second);
                const float ri = 0.13f * B, yI = thrY - rt - 0.10f * B - ri;
                if (top > thrY - rt - 0.05f * B && yI > 0.9f * B) addPulley(0, yI, ri, -1, 1);
            }
            if (boxer) {   // the belt dips up round the crank sprocket's underside between two idlers below it
                force[0] = 1;
                const float ri = 0.12f * B, yI = -crankSprR - ri + 0.10f * B;
                addPulley(-0.60f * B, yI, ri, -1, 1);
                addPulley(0.60f * B, yI, ri, -1, 2);
                extras.back().runDir = v3(1, 0, 0);
            }
            std::vector<V3> path = beltPath(pulleys, zFront, &force, boxer);
            V3 cen = centroidOf(path);
            if (!chainDrive && !boxer && path.size() > 3) {   // the belt's tensioner on its longest run
                const std::vector<Run> rr = runsOf(path);
                if (!rr.empty()) {
                    const Run& rn = rr[0];
                    const V3 mid = (rn.a + rn.b) * 0.5f, nrm = outwardOf(rn, cen);
                    const float rT = 0.12f * B;
                    const V3 c = mid + nrm * (rT - 0.05f * B);
                    addPulley(c.x, c.y, rT, -1, 2);
                    extras.back().runDir = norm(rn.b - rn.a);
                    path = beltPath(pulleys, zFront, &force, boxer);
                    cen = centroidOf(path);
                }
            }
            // idler / tensioner pulleys: a flanged pulley (a sprocket on a chain), the tensioner on an arm with its damper
            for (const Extra& x : extras) {
                if (chainDrive) {
                    const Cached& mIs = meshOf(key("idlerspr", {x.r, B}), [&] { return gearMesh(x.r, 0.03f * B, std::max(10, (int)(x.r / (0.012f * B))), 0.10f * B, 0.05f * B); });
                    sc.put(mIs, translate(x.c) * rotZ(crankRad * crankSprR / x.r), kSteel);
                } else {
                    const Cached& mIp = meshOf(key("idlerpul", {x.r, B}), [&] { return mesh::lathe({{0.04f * B, -0.07f * B}, {x.r + 0.02f * B, -0.07f * B}, {x.r + 0.02f * B, -0.055f * B},
                                                                                                     {x.r, -0.05f * B}, {x.r, 0.05f * B}, {x.r + 0.02f * B, 0.055f * B},
                                                                                                     {x.r + 0.02f * B, 0.07f * B}, {0.04f * B, 0.07f * B}}, 28); });
                    sc.put(mIp, translate(x.c) * rotX(0.5f * kPiF), kSteel);
                }
                const Cached& mHub = meshOf(key("idlerhub", {B}), [&] { return rodZ(0.05f * B, -0.10f * B, 0.10f * B, 16); });
                sc.put(mHub, translate(x.c), kSteelDark);
                if (x.kind == 2) {   // the tensioner: an arm from the pulley's hub to its pivot, a damper pushing on it
                    const V3 along = x.runDir, piv = x.c + along * (0.38f * B);
                    const float ang = std::atan2(piv.y - x.c.y, piv.x - x.c.x);
                    const Cached& mArm = meshOf(key("tensarm", {B}), [&] { return mesh::box(0.19f * B, 0.035f * B, 0.025f * B); });
                    sc.put(mArm, translate((x.c + piv) * 0.5f + v3(0, 0, -0.08f * B)) * rotZ(ang), kSteelDark);
                    const Cached& mDamp = meshOf(key("tensdamp", {B}), [&] { return mesh::tube({v3(0, 0, 0), v3(0, 0.26f * B, 0)}, {0.04f * B, 0.04f * B}, 14); });
                    const V3 nrm = v3(-along.y, along.x, 0);
                    const V3 dOut = (nrm.x * (x.c.x - cen.x) + nrm.y * (x.c.y - cen.y)) > 0 ? nrm : nrm * -1.0f;
                    sc.put(mDamp, alongY(piv + dOut * (0.04f * B) + v3(0, 0, -0.08f * B), dOut), kAlu);
                }
            }
            if (path.size() > 3) {
                if (chainDrive) drawShoes(path);
                drawLoop(path, chainDrive, (int)pulleys.size());
            }
        }
        // flywheel with its ring gear at the rear
        const float fwR = std::max(1.45f * B, 2.2f * r);
        const Cached& mFly = meshOf(key("flywheel", {fwR, B}), [&] {
            Data d = mesh::extrude(mesh::gearOutline(fwR - 0.045f * B, fwR, 96), -0.07f * B, 0.07f * B);
            mesh::append(d, mesh::extrude(mesh::circleOutline(0.30f * B, 24), -0.20f * B, 0.20f * B), M4{});
            return d; });
        sc.put(mFly, translate(v3(0, 0, zRear + 0.25f * B)) * rotZ(crankRad), kSteelDark);
    }
}

} // namespace

void buildEngineModel(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live, float glassAlpha,
                      EngineModel& out) {
    using namespace model;
    out.clear();
    // shapes are cached per dimension: dragging a slider makes new ones - start over now and then (between frames,
    // never while a frame holds references into the cache)
    model::cacheTrim();
    Scene sc{&out};
    if (isRotary(p)) {
        EngineBits eb;
        V3 turbineIn = v3(0, 0, 0), throttleIn = v3(0, 0, 0), throttleDir = v3(0, 0, -1);
        buildRotaryEngine(sc, p, phases, crankDeg, live, glassAlpha, p.induction == Induction::Turbo, eb, turbineIn, throttleIn, throttleDir);
        if (isBoosted(p)) buildBoost(sc, p, eb, live, crankDeg, turbineIn, throttleIn, throttleDir);
        return;
    }
    EngineBits eb;
    buildPistonEngine(sc, p, phases, crankDeg, live, glassAlpha, eb);
    if (isTwoStroke(p)) return;   // (its expansion chamber / intake are part of each cylinder)
    V3 inletDir = v3(0, 0, -1);
    const V3 throttleIn = buildIntake(sc, p, eb, inletDir);
    V3 turbineIn = v3(0, 0, 0);
    const bool turbo = p.induction == Induction::Turbo;
    buildExhaust(sc, p, eb, live, turbo, turbineIn);
    if (isDiesel(p)) buildDiesel(sc, p, eb, live);
    if (isBoosted(p)) buildBoost(sc, p, eb, live, crankDeg, turbineIn, throttleIn, inletDir);
}

namespace model {
// (the serpentine belt router for the parts file: blower drives with their idler / tensioner)
std::vector<V3> serpentine(const std::vector<std::tuple<float, float, float>>& pulleys, float z, const std::vector<int>& force) {
    return beltPath(pulleys, z, &force);
}
} // namespace model

} // namespace es::ui
