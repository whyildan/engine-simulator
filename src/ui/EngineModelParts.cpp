// Engine model parts: intake, exhaust headers (glowing with the exhaust temperature), turbo / supercharger /
// intercooler, the two-stroke's ports / transfers / crankcase / reed / expansion chamber, and the rotary engine.
#include "ui/EngineModelParts.h"
#include "sim/Exhaust.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>

namespace es::ui::model {

using namespace m3;
using gpu::Material;
using mesh::Data;
using mesh::Poly;

namespace {

constexpr float kPiF = 3.14159265358979f;
constexpr float kDegF = kPiF / 180.0f;

float smooth01(float a, float b, float x) { const float t = clampv((x - a) / (b - a), 0.0f, 1.0f); return t * t * (3 - 2 * t); }

// exhaust pipe material: plain steel that glows the app's orange once the gas is really hot (dull from ~550 C)
Material hot(float egtC, float k = 1.0f) {
    const float t = smooth01(550.0f, 950.0f, egtC) * k;
    Material m = kHotPipe;
    m.r *= 1.0f - 0.55f * t; m.g *= 1.0f - 0.60f * t; m.b *= 1.0f - 0.65f * t;   // the glow shows best on darkened metal
    return emissive(m, kAccent[0] * 0.45f * t, kAccent[1] * 0.40f * t, kAccent[2] * 0.30f * t);
}

// a tube along a path, cached by a key made from the path itself
const Cached& tubeOf(const char* name, const std::vector<V3>& path, float r, int sides = 16) {
    float h = 0;
    for (size_t k = 0; k < path.size(); ++k) h += (path[k].x * 1.3f + path[k].y * 2.1f + path[k].z * 0.7f) * (float)(k + 1);
    return meshOf(key(name, {h, r, (float)path.size()}), [&] { return mesh::tube(path, std::vector<float>(path.size(), r), sides); });
}

// a tube whose radius changes along it (radius per path point)
const Cached& tubeVar(const char* name, const std::vector<V3>& path, const std::vector<float>& rad, float salt) {
    float h = salt;
    for (size_t k = 0; k < path.size(); k += 3) h += (path[k].x * 1.3f + path[k].y * 2.1f + path[k].z * 0.7f + rad[k] * 3.3f) * (float)(k + 1);
    return meshOf(key(name, {h, (float)path.size()}), [&] { return mesh::tube(path, rad, 20); });
}

Data plugMesh(float B) {
    const float s = B / 86.0f;
    return mesh::lathe({{0, -2 * s}, {1.2f * s, -2 * s}, {1.2f * s, 0}, {7 * s, 0}, {7 * s, 19 * s}, {10.4f * s, 19 * s}, {10.4f * s, 27 * s},
                        {5.5f * s, 30 * s}, {5.0f * s, 52 * s}, {2.5f * s, 56 * s}, {0, 56 * s}}, 24);
}

// spiral housing (volute) around +y, opening growing over the turn - turbo / centrifugal compressor shells
Data voluteMesh(float r0, float r1, float t0, float t1) {
    std::vector<V3> path;
    std::vector<float> rad;
    const int n = 48;
    for (int k = 0; k <= n; ++k) {
        const float u = (float)k / n, a = u * 1.85f * kPiF;
        const float R = r0 + (r1 - r0) * u;
        path.push_back(v3(R * std::cos(a), 0, R * std::sin(a)));
        rad.push_back(t0 + (t1 - t0) * u);
    }
    // the outlet leaves tangentially
    const V3 e = path.back(), dir = norm(path.back() - path[path.size() - 2]);
    for (int k = 1; k <= 6; ++k) { path.push_back(e + dir * (t1 * 0.55f * k)); rad.push_back(t1); }
    Data d = mesh::tube(path, rad, 18);
    mesh::append(d, mesh::lathe({{0, -0.5f * t1}, {r0 * 0.95f, -0.5f * t1}, {r0 * 0.95f, 0.5f * t1}, {0, 0.5f * t1}}, 36), M4{});
    return d;
}

// compressor / turbine wheel about +y: hub and swept blades
Data wheelMesh(float R, float h, int blades) {
    Data d = mesh::lathe({{0, 0}, {R * 0.95f, 0}, {R * 0.55f, h * 0.5f}, {R * 0.22f, h}, {0, h}}, 28);
    for (int b = 0; b < blades; ++b) {
        const float a = 2 * kPiF * b / blades;
        mesh::append(d, mesh::box(0.5f * R * 0.8f, 0.5f * h * 0.9f, 0.012f * R * 3.0f), rotY(a) * translate(v3(R * 0.5f, h * 0.45f, 0)) * rotX(0.5f));
    }
    return d;
}

// Roots / screw rotor outline: lobes on a circle
Poly lobedOutline(float R, int lobes, float depth) {
    Poly p;
    const int n = lobes * 18;
    for (int k = 0; k < n; ++k) {
        const float a = 2 * kPiF * k / n;
        const float c = std::fabs(std::cos(0.5f * lobes * a));
        p.push_back({R * (1.0f - depth + depth * std::pow(c, 0.7f)) * std::cos(a), R * (1.0f - depth + depth * std::pow(c, 0.7f)) * std::sin(a)});
    }
    return p;
}

// the belt line around two pulleys (x, y, r) in the plane z
std::vector<V3> beltAround(V3 a, float ra, V3 b, float rb) {
    Poly all;
    for (const auto& q : mesh::circleOutline(ra, 40, a.x, a.y)) all.push_back(q);
    for (const auto& q : mesh::circleOutline(rb, 40, b.x, b.y)) all.push_back(q);
    std::sort(all.begin(), all.end());
    Poly h(2 * all.size());
    size_t k = 0;
    auto cr = [](const std::pair<float, float>& o, const std::pair<float, float>& p1, const std::pair<float, float>& p2) {
        return (p1.first - o.first) * (p2.second - o.second) - (p1.second - o.second) * (p2.first - o.first);
    };
    for (size_t i = 0; i < all.size(); ++i) { while (k >= 2 && cr(h[k - 2], h[k - 1], all[i]) <= 0) --k; h[k++] = all[i]; }
    for (size_t i = all.size() - 1, t = k + 1; i > 0; --i) { while (k >= t && cr(h[k - 2], h[k - 1], all[i - 1]) <= 0) --k; h[k++] = all[i - 1]; }
    h.resize(k - 1);
    std::vector<V3> out;
    for (const auto& q : h) out.push_back(v3(q.first, q.second, a.z));
    return out;
}

// A pipe along a polyline with rounded corners (bend radius up to `bend`, less where the legs are short): straight runs
// and clean bends, as boost and exhaust tubing is made - a spline through the same points wanders and overshoots.
std::vector<V3> routePipe(const std::vector<V3>& pts, float bend) {
    if (pts.size() < 3) return pts;
    std::vector<V3> out = {pts[0]};
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        const V3 a = pts[i - 1], c = pts[i], b = pts[i + 1];
        const float la = len(c - a), lb = len(b - c);
        if (la < 1e-4f || lb < 1e-4f) continue;
        const float r = std::min(bend, 0.45f * std::min(la, lb));
        const V3 p0 = c + (a - c) * (r / la), p1 = c + (b - c) * (r / lb);
        for (int k = 0; k <= 8; ++k) {
            const float t = k / 8.0f;
            out.push_back(p0 * ((1 - t) * (1 - t)) + c * (2 * (1 - t) * t) + p1 * (t * t));
        }
    }
    out.push_back(pts.back());
    return out;
}

// A turbo / centrifugal compressor housing ("snail"), local frame: the shaft along +y (the outer face), the scroll in the
// xz plane round a hub disc: the scroll tube grows from t0 to t1 over 0.9 of a turn and leaves tangentially for `ext` -
// the inlet of a turbine housing, the outlet of a compressor housing (port E, pointing out along D).
struct Snail { float R, w, t0, t1, ext; };
static void snailCurve(const Snail& s, float u, V3& pt) {
    const float t = s.t0 + (s.t1 - s.t0) * u, a = u * 1.8f * kPiF, rc = s.R + 0.55f * t;
    pt = v3(rc * std::cos(a), 0, rc * std::sin(a));
}
static void snailPort(const Snail& s, V3& E, V3& D) {
    V3 p1, p0;
    snailCurve(s, 1.0f, p1); snailCurve(s, 0.99f, p0);
    D = norm(p1 - p0);
    E = p1 + D * s.ext;
}
static Data snailMesh(const Snail& s) {
    std::vector<V3> path;
    std::vector<float> rad;
    for (int k = 0; k <= 40; ++k) { V3 q; snailCurve(s, k / 40.0f, q); path.push_back(q); rad.push_back(s.t0 + (s.t1 - s.t0) * k / 40.0f); }
    V3 E, D;
    snailPort(s, E, D);
    for (int k = 1; k <= 4; ++k) { path.push_back(path.back() + D * (s.ext / 4.0f)); rad.push_back(s.t1); }
    Data d = mesh::tube(path, rad, 18);
    mesh::append(d, mesh::lathe({{0, -s.w}, {s.R * 0.85f, -s.w}, {s.R + 0.5f * s.t0, -0.55f * s.w}, {s.R + 0.6f * s.t0, 0},
                                 {s.R + 0.5f * s.t0, 0.55f * s.w}, {s.R * 0.85f, s.w}, {0, s.w}}, 36), M4{});
    return d;
}
// the frame for a snail whose shaft points along `axis`, turned about the shaft so its port points along `portDir`
static M4 snailFrame(const Snail& s, V3 centre, V3 axis, V3 portDir) {
    V3 E, D;
    snailPort(s, E, D);
    float best = -2, bestA = 0;
    for (int k = 0; k < 360; ++k) {
        const float a = k * kPiF / 180.0f;
        const float d = dot(xdir(alongY(centre, axis) * rotY(a), D), portDir);
        if (d > best) { best = d; bestA = a; }
    }
    return alongY(centre, axis) * rotY(bestA);
}

// a supercharger drive belt: crank pulley, blower pulley, and an idler (Roots 6-71 drive kits: upper and lower pulleys,
// an idler pulley on its bracket) or a spring tensioner (ProCharger brackets) pushing into the run on `side` (+1 / -1
// along the line's normal) - the belt is routed round all three
void blowerBelt(Scene& sc, V3 cp, float rc, V3 pul, float rp, float side, bool tensioner, float B, float spin) {
    const V3 d = norm(v3(pul.x - cp.x, pul.y - cp.y, 0)), nrm = v3(-d.y, d.x, 0) * side;
    const float ri = 0.13f * B;
    const V3 mid = (cp + pul) * 0.5f + nrm * (0.5f * (rc + rp) + ri - 0.06f * B);
    const std::vector<V3> path = serpentine({{cp.x, cp.y, rc}, {pul.x, pul.y, rp}, {mid.x, mid.y, ri}}, cp.z, {1, 1, -1});
    if (path.size() < 4) return;
    sc.put(tubeOf("scbelt", path, 0.025f * B, 8), M4{}, kRubber);
    const Cached& mIp = meshOf(key("scidler", {ri, B}), [&] { return mesh::lathe({{0.04f * B, -0.06f * B}, {ri + 0.02f * B, -0.06f * B}, {ri, -0.04f * B}, {ri, 0.04f * B},
                                                                                {ri + 0.02f * B, 0.06f * B}, {0.04f * B, 0.06f * B}}, 28); });
    sc.put(mIp, translate(v3(mid.x, mid.y, cp.z)) * rotZ(spin) * rotX(0.5f * kPiF), kSteel);
    // its bracket back to the engine (an arm to a pivot for the tensioner, with the spring can)
    const V3 piv = v3(mid.x, mid.y, cp.z) + nrm * (0.30f * B) + d * (tensioner ? 0.25f * B : 0.0f) + v3(0, 0, 0.10f * B);
    sc.put(tubeOf("scbracket", {v3(mid.x, mid.y, cp.z + 0.08f * B), piv}, 0.03f * B, 10), M4{}, kSteelDark);
    if (tensioner) {
        const Cached& mCan = meshOf(key("sctens", {B}), [&] { return mesh::lathe({{0, 0}, {0.07f * B, 0}, {0.07f * B, 0.12f * B}, {0, 0.12f * B}}, 20); });
        sc.put(mCan, translate(piv) * rotX(0.5f * kPiF), kAlu);
    }
}

void tailWithFlame(Scene& sc, const std::vector<V3>& ctrl, float rt, const EngineViewLive& live, float egt, int salt, float canLen = 0) {
    const std::vector<V3> path = spline(ctrl, 8);
    sc.put(tubeOf("tail", path, rt), M4{}, hot(egt, 0.35f));
    // silencer can on the way (canLen: its length; 0 = the old short can)
    const V3 a = path[path.size() / 2], b = path[path.size() / 2 + 2 < path.size() ? path.size() / 2 + 2 : path.size() - 1];
    const float hl = canLen > 0 ? 0.5f * canLen : 2.6f * rt, rc = canLen > 0 ? 2.0f * rt : 2.3f * rt;
    const Cached& can = meshOf(key("muffcan", {rt, hl, rc}), [&] { return mesh::lathe({{0, -hl}, {rc * 0.9f, -hl}, {rc, -hl + 0.4f * rt}, {rc, hl - 0.4f * rt},
                                                                                      {rc * 0.9f, hl}, {0, hl}}, 28); });
    sc.put(can, alongY((a + b) * 0.5f, b - a), kSteelDark);
    // afterfire pop: a flame out of the tip (the app's orange, a hot yellow core)
    if (live.popFlash > 0.02f) {
        const V3 tip = path.back(), d = norm(path.back() - path[path.size() - 2]);
        const float k = live.popFlash;
        for (int j = 0; j < 7; ++j) {
            const float t = (j + 0.5f) / 7.0f;
            const float jit = 0.25f * rt * (rnd(salt, j + (int)(live.time * 37)) - 0.5f);
            sc.glow(tip + d * (t * 9.0f * rt * k) + v3(jit, jit * 0.7f, 0), rt * (1.6f - t) * (0.8f + k), kAccent[0], 0.45f + 0.25f * (1 - t), 0.12f, 0.9f * k * (1 - t * 0.6f));
        }
        sc.glow(tip + d * rt, rt * 1.2f, 1.0f, 0.92f, 0.6f, 0.9f * k);
    }
}

} // namespace

// ------------------------------------------------------------------ intake
V3 buildIntake(Scene& sc, const EngineParams& p, EngineBits& eb, V3& inletDir) {
    if (eb.in.empty()) return v3(0, 0, 0);
    const float B = eb.B;
    const float rr = clampv(p.runnerDia_mm * 0.5f, 0.12f * B, 0.24f * B);
    V3 avgP = v3(0, 0, 0), avgD = v3(0, 0, 0);
    float z0 = 1e9f, z1 = -1e9f;
    for (const PortOut& q : eb.in) { avgP = avgP + q.p; avgD = avgD + q.d; z0 = std::min(z0, q.p.z); z1 = std::max(z1, q.p.z); }
    avgP = avgP * (1.0f / eb.in.size());
    avgD = norm(avgD);
    // V: the ports face into the valley - one plenum in the middle, on top; inline / boxer: beside the head
    V3 C = eb.vee ? v3(0, avgP.y + 0.55f * B, 0) : avgP + avgD * (0.95f * B) + v3(0, 0.20f * B, 0);
    C.z = 0.5f * (z0 + z1);
    const float L = (z1 - z0) + 1.0f * B;
    eb.plenum = C; eb.plenumLen = L;
    if (p.itb) {
        // individual throttle bodies: each runner ends in a velocity stack
        const Cached& stack = meshOf(key("stack", {rr}), [&] { return mesh::lathe({{rr * 0.92f, 0}, {rr, 0}, {rr * 1.05f, 0.9f * rr}, {rr * 1.55f, 1.7f * rr}, {rr * 1.45f, 1.75f * rr},
                                                                                   {rr * 0.95f, 0.95f * rr}, {rr * 0.92f, 0}}, 28); });
        for (const PortOut& q : eb.in) {
            const std::vector<V3> path = spline({q.p, q.p + q.d * (0.35f * B), q.p + q.d * (0.80f * B) + v3(0, 0.15f * B, 0)}, 8);
            sc.put(tubeOf("runner", path, rr), M4{}, kAlu);
            sc.put(stack, alongY(path.back(), path.back() - path[path.size() - 2]), kAlu);
        }
        inletDir = v3(0, 0, -1);
        return C;
    }
    for (const PortOut& q : eb.in) {
        const V3 tgt = v3(C.x, C.y, q.p.z);
        const std::vector<V3> path = spline({q.p, q.p + q.d * (0.35f * B), lerp(q.p + q.d * (0.6f * B), tgt, 0.55f), tgt}, 8);
        sc.put(tubeOf("runner", path, rr), M4{}, kAlu);
    }
    const float hx = 0.42f * B, hy = 0.34f * B;
    const Cached& plen = meshOf(key("plenum", {hx, hy, L}), [&] { return mesh::extrude(mesh::roundedRect(hx, hy, 0.16f * B), -0.5f * L, 0.5f * L); });
    sc.put(plen, translate(C), kAlu);

    // throttle body at the front, or a carburettor on top
    const float rt = clampv(p.throttleBore_mm * 0.5f, 0.15f * B, 0.45f * B);
    const V3 tb0 = v3(C.x, C.y, C.z - 0.5f * L), tb1 = tb0 + v3(0, 0, -0.55f * B);
    if (p.fuelSystem == FuelSystem::Carburetor) {
        const Cached& carb = meshOf(key("carb", {B}), [&] { return mesh::box(0.32f * B, 0.22f * B, 0.30f * B); });
        sc.put(carb, translate(C + v3(0, hy + 0.22f * B, -0.15f * L)), kAlu);
        const Cached& horn = meshOf(key("airhorn", {B}), [&] { return mesh::lathe({{0.20f * B, 0}, {0.23f * B, 0}, {0.30f * B, 0.25f * B}, {0.27f * B, 0.27f * B}, {0.20f * B, 0.05f * B}}, 28); });
        sc.put(horn, translate(C + v3(0, hy + 0.44f * B, -0.15f * L)), kAlu);
    }
    sc.put(tubeOf("throttle", {tb0, tb1}, rt), M4{}, kAlu);
    inletDir = v3(0, 0, -1);
    return tb1;
}

// ------------------------------------------------------------------ exhaust
void buildExhaust(Scene& sc, const EngineParams& p, const EngineBits& eb, const EngineViewLive& live, bool turbo, V3& turbineIn) {
    if (eb.ex.empty()) return;
    const float B = eb.B;
    const float rp = clampv(p.primaryDia_mm * 0.5f, 0.12f * B, 0.26f * B);
    const float rt = clampv(p.pipeDia_mm * 0.5f, rp, 0.40f * B);
    auto egtOf = [&](int cyl) { return live.egtC[cyl] > 1.0f ? live.egtC[cyl] : live.egtGas; };
    std::vector<V3> collectors;
    for (int b = 0; b < 2; ++b) {
        std::vector<PortOut> ports;
        for (const PortOut& q : eb.ex) if (q.bank == b) ports.push_back(q);
        if (ports.empty()) continue;
        V3 avgP = v3(0, 0, 0), avgD = v3(0, 0, 0);
        float z0 = 1e9f, z1 = -1e9f, egtMean = 0;
        for (const PortOut& q : ports) { avgP = avgP + q.p; avgD = avgD + q.d; z0 = std::min(z0, q.p.z); z1 = std::max(z1, q.p.z); egtMean += egtOf(q.cyl); }
        avgP = avgP * (1.0f / ports.size()); avgD = norm(avgD); egtMean /= ports.size();
        const V3 down = v3(0, -1, 0);
        V3 C = avgP + avgD * (1.05f * B) + down * (1.15f * B);
        C.z = eb.zRear + 0.55f * B;
        if (!turbo) {
            // no turbo: the primaries leave the ports, turn down and run back along the side of the engine as a bundle,
            // into a collector cone beside the rear of the block; from there a long pipe to the rear, a silencer half
            // way, the tail pipe at the end (the view can be panned / zoomed out to follow it)
            const V3 side = norm(v3(avgD.x, 0, avgD.z));
            const V3 run = avgP + side * (0.95f * B) + down * (0.95f * B);   // the bundle's line along the engine
            const V3 C0 = v3(run.x, run.y - 0.15f * B, z1 + 0.80f * B);          // collector: where the primaries meet
            const int np = (int)ports.size();
            // the collector's mouth: wide enough for the primaries side by side on a ring
            const float ringMin = np > 1 ? rp * 1.08f / std::sin(kPiF / std::max(2, np)) : 0.0f;
            const float cr0 = std::max({rt * 1.7f, rp * 2.2f, ringMin + rp * 1.05f});
            std::vector<PortOut> byZ = ports;
            std::sort(byZ.begin(), byZ.end(), [](const PortOut& a, const PortOut& b2) { return a.p.z < b2.p.z; });
            if (p.header == HeaderType::Log && np > 1) {
                const V3 L0 = v3(avgP.x, avgP.y, z0 - 0.2f * B) + avgD * (0.60f * B);
                const V3 L1 = v3(L0.x, L0.y, z1 + 0.3f * B);
                sc.put(tubeOf("log", spline({L0, L1, L1 + v3(0, -0.4f * B, 0.25f * B) + side * (0.2f * B), C0}, 8), rt * 1.05f), M4{}, hot(egtMean, 0.8f));
                for (const PortOut& q : ports)
                    sc.put(tubeOf("stub", spline({q.p, q.p + q.d * (0.30f * B), v3(L0.x, L0.y, q.p.z)}, 6), rp), M4{}, hot(egtOf(q.cyl)));
            } else {
                for (int k = 0; k < np; ++k) {
                    const PortOut& q = byZ[k];
                    // a 4-1 header: out of the port and down, then one sweep back along the side of the engine to its
                    // own place on a ring round the collector's mouth, arriving parallel to the pipe
                    const float ang = 2.0f * kPiF * k / std::max(1, np) + 0.25f * kPiF;
                    const float ringR = np > 1 ? std::max(0.5f * cr0, ringMin) : 0.0f;
                    const V3 ring = side * (ringR * std::cos(ang)) + down * (ringR * std::sin(ang));
                    const V3 entry = C0 + ring;
                    const V3 b0 = q.p + q.d * (0.30f * B), b1 = q.p + q.d * (0.60f * B) + down * (0.50f * B);
                    std::vector<V3> ctrl = {q.p, b0, b1};
                    const float far = entry.z - b1.z;
                    if (far > 1.2f * B) ctrl.push_back(lerp(b1, entry - v3(0, 0, 0.5f * B), 0.5f) + down * (0.08f * B));
                    if (far > 0.6f * B) ctrl.push_back(entry - v3(0, 0, 0.45f * B));
                    ctrl.push_back(entry);
                    sc.put(tubeOf("primary", spline(ctrl, 8), rp), M4{}, hot(egtOf(q.cyl)));
                }
            }
            // collector cone into the pipe
            const float cl = 0.75f * B, cr = cr0;
            const Cached& cone = meshOf(key("collcone", {cr, rt, cl}), [&] { return mesh::lathe({{0, 0}, {cr, 0}, {cr, 0.15f * cl}, {rt, cl}, {0, cl}}, 28); });
            sc.put(cone, alongY(C0, v3(0, 0, 1)), hot(egtMean, 0.8f));
            const V3 E0 = C0 + v3(0, 0, cl);
            collectors.push_back(E0);
            tailWithFlame(sc, {E0, E0 + v3(0, -0.10f * B, 1.2f * B), E0 + v3(0, -0.30f * B, 3.0f * B), E0 + v3(0, -0.35f * B, 4.8f * B),
                               E0 + v3(0, -0.35f * B, 6.2f * B)}, rt, live, egtMean, b, 1.8f * B);
            continue;
        }
        if (turbo) {
            // turbo manifold: the turbine sits beside the block below the ports, its shaft along the crank (compressor
            // forward, as an RB / 2JZ); its inlet flange faces up. A 4-1 header comes down into it through a collector
            // cone; a log manifold runs along the ports and drops into it from its side
            const V3 side = norm(v3(avgD.x, 0, avgD.z));
            const float zT = 0.5f * (z0 + z1) + 0.25f * B;
            const V3 port = v3(avgP.x, avgP.y, zT) + side * (1.30f * B) + down * (0.85f * B);   // turbine inlet flange
            const V3 up = v3(0, 1, 0);
            if (p.header == HeaderType::Log && ports.size() > 1) {
                const V3 L0 = v3(avgP.x, avgP.y, z0 - 0.15f * B) + side * (0.62f * B), L1 = v3(L0.x, L0.y, z1 + 0.15f * B);
                sc.put(tubeOf("tlog", {L0, L1}, rt * 1.05f), M4{}, hot(egtMean, 0.8f));
                for (const PortOut& q : ports)
                    sc.put(tubeOf("stub", routePipe({q.p, q.p + q.d * (0.25f * B), v3(L0.x, L0.y, q.p.z)}, 0.15f * B), rp), M4{}, hot(egtOf(q.cyl)));
                sc.put(tubeOf("tlogout", routePipe({v3(L0.x, L0.y, zT), v3(port.x, L0.y, zT), port}, 0.30f * B), rt), M4{}, hot(egtMean, 0.8f));
            } else {
                const int np = (int)ports.size();
                const float ringMin = np > 1 ? rp * 1.08f / std::sin(kPiF / std::max(2, np)) : 0.0f;
                const float cr = std::max({rt * 1.5f, rp * 2.2f, ringMin + rp * 1.05f}), cl = 0.55f * B;
                const V3 C0 = port + up * cl;
                std::vector<PortOut> byZ = ports;
                std::sort(byZ.begin(), byZ.end(), [](const PortOut& a, const PortOut& b2) { return a.p.z < b2.p.z; });
                for (int k = 0; k < np; ++k) {
                    const PortOut& q = byZ[k];
                    const float ang = 2.0f * kPiF * k / std::max(1, np) + 0.25f * kPiF;
                    const float ringR = np > 1 ? std::max(0.5f * cr, ringMin) : 0.0f;
                    const V3 entry = C0 + side * (ringR * std::cos(ang)) + v3(0, 0, ringR * std::sin(ang));
                    std::vector<V3> ctrl = {q.p, q.p + q.d * (0.30f * B), q.p + q.d * (0.55f * B) + down * (0.10f * B)};
                    if (std::fabs(entry.z - q.p.z) > 0.8f * B) ctrl.push_back(v3(entry.x, entry.y + 0.45f * B, q.p.z + 0.6f * (entry.z - q.p.z)));
                    ctrl.push_back(entry + up * (0.30f * B));
                    ctrl.push_back(entry);
                    sc.put(tubeOf("tprimary", spline(ctrl, 8), rp), M4{}, hot(egtOf(q.cyl)));
                }
                const Cached& cone = meshOf(key("tcollcone", {cr, rt, cl}), [&] { return mesh::lathe({{0, 0}, {cr, 0}, {cr, 0.15f * cl}, {rt, cl}, {0, cl}}, 28); });
                sc.put(cone, alongY(C0, v3(0, -1, 0)), hot(egtMean, 0.8f));
            }
            collectors.push_back(port);
            continue;
        }
        if (p.header == HeaderType::Log && ports.size() > 1) {
            // log manifold: one pipe along the head, short stubs into it
            const V3 L0 = v3(avgP.x, avgP.y, z0 - 0.2f * B) + avgD * (0.60f * B);
            const V3 L1 = v3(L0.x, L0.y, z1 + 0.3f * B);
            sc.put(tubeOf("log", spline({L0, L1, L1 + v3(0, -0.5f * B, 0.5f * B) + avgD * (0.3f * B), C}, 8), rt * 1.05f), M4{}, hot(egtMean, 0.8f));
            for (const PortOut& q : ports)
                sc.put(tubeOf("stub", spline({q.p, q.p + q.d * (0.30f * B), v3(L0.x, L0.y, q.p.z)}, 6), rp), M4{}, hot(egtOf(q.cyl)));
        } else {
            for (const PortOut& q : ports) {
                const V3 Q = avgP + avgD * (1.05f * B) + down * (0.95f * B);
                const std::vector<V3> path = spline({q.p, q.p + q.d * (0.30f * B), q.p + q.d * (0.75f * B) + down * (0.40f * B), v3(Q.x, Q.y, q.p.z),
                                                     v3(C.x, C.y, q.p.z + (C.z - q.p.z) * 0.55f), C}, 8);
                sc.put(tubeOf("primary", path, rp), M4{}, hot(egtOf(q.cyl)));
            }
            const Cached& merge = meshOf(key("merge", {rt}), [&] { return mesh::lathe({{0, -1.2f * rt}, {1.25f * rt, -0.8f * rt}, {1.25f * rt, 0.8f * rt}, {0, 1.2f * rt}}, 24); });
            sc.put(merge, translate(C), hot(egtMean, 0.8f));
        }
        collectors.push_back(C);
        if (turbo) continue;
        tailWithFlame(sc, {C, C + v3(0, -0.15f * B, 0.7f * B), C + v3(0, -0.30f * B, 2.0f * B), C + v3(0, -0.32f * B, 3.4f * B)}, rt, live, egtMean, b);
    }
    if (turbo && !collectors.empty()) {
        turbineIn = collectors[0];   // (the turbine inlet flange: the gas goes down into it)
        // the other bank (V / boxer): its collector drops, runs back under the engine behind the flywheel and comes up
        // the turbo side into the turbine's collector
        const float rt = clampv(p.pipeDia_mm * 0.5f, 0.15f * B, 0.40f * B);
        for (size_t k = 1; k < collectors.size(); ++k) {
            const V3 a = collectors[k], t = collectors[0];
            const float yLow = std::min(a.y, t.y) - 0.9f * B, zX = eb.zRear + 0.75f * B;
            const float tSide = t.x >= 0 ? 1.0f : -1.0f;
            const V3 join = t + v3(0, 0.25f * B, 0) + v3(tSide * 0.0f, 0, 0);
            sc.put(tubeOf("crossover", routePipe({a, v3(a.x, yLow, a.z), v3(a.x, yLow, zX), v3(t.x + tSide * 0.55f * B, yLow, zX),
                                                  v3(t.x + tSide * 0.55f * B, join.y, zX), v3(t.x + tSide * 0.55f * B, join.y, join.z),
                                                  join}, 0.40f * B), rt), M4{}, hot(live.egtGas, 0.6f));
        }
    }
}

// ------------------------------------------------------------------ diesel
void buildDiesel(Scene& sc, const EngineParams& p, const EngineBits& eb, const EngineViewLive& live) {
    const float B = eb.B;
    // distributor injection pump: its body runs back from the pulley; one steel high-pressure line to each injector
    if (eb.hasPump) {
        const float L = 0.95f * B;
        const Cached& body = meshOf(key("injpump", {B}), [&] {
            Data d = mesh::lathe({{0, 0}, {0.20f * B, 0}, {0.22f * B, 0.06f * B}, {0.22f * B, 0.55f * L}, {0.17f * B, 0.62f * L}, {0.17f * B, L},
                                  {0.10f * B, L + 0.05f * B}, {0, L + 0.05f * B}}, 32);
            mesh::append(d, mesh::box(0.07f * B, 0.10f * B, 0.12f * B), translate(v3(0, 0.30f * L, 0.24f * B)));   // the advance unit
            return d; });
        sc.put(body, alongY(eb.pump + v3(0, 0, 0.10f * B), v3(0, 0, 1)), kAlu);
        const V3 head = eb.pump + v3(0, 0, 0.10f * B + L + 0.04f * B);
        const int n = (int)eb.injTop.size();
        if (p.dieselInjSystem == 1 && n > 0) {
            // common rail: the high-pressure pump feeds one forged rail along each bank's injectors, short lines down
            std::vector<V3> tops = eb.injTop;
            V3 lo = tops[0], hi = tops[0];
            for (const V3& t : tops) { if (t.z < lo.z) lo = t; if (t.z > hi.z) hi = t; }
            // one rail per side of the engine (a V's two banks), offset above and inboard of the injectors
            for (int side = -1; side <= 1; side += 2) {
                std::vector<V3> bank;
                for (const V3& t : tops) if ((t.x < 0) == (side < 0) || std::fabs(lo.x - hi.x) < 1e-3f) bank.push_back(t);
                if (bank.empty()) continue;
                V3 a = bank[0], b = bank[0], mid = v3(0, 0, 0);
                for (const V3& t : bank) { if (t.z < a.z) a = t; if (t.z > b.z) b = t; mid = mid + t; }
                mid = mid * (1.0f / bank.size());
                const V3 off = v3(-0.20f * B * (mid.x >= 0 ? 1.0f : -1.0f), 0.22f * B, 0);
                const V3 r0 = v3(mid.x, mid.y, a.z - 0.35f * B) + off, r1 = v3(mid.x, mid.y, b.z + 0.35f * B) + off;
                sc.put(tubeOf("rail", {r0, r1}, 0.07f * B, 16), M4{}, kSteel);
                for (const V3& t : bank)
                    sc.put(tubeOf("railline", spline({v3(t.x, mid.y, t.z) + off, v3(t.x, t.y + 0.18f * B, t.z) + off * 0.4f, t + v3(0, 0.04f * B, 0)}, 6), 0.018f * B, 8), M4{}, kSteel);
                {   // feed: from the pump up in front of the heads, over to the rail's front end (it used to cut through)
                    const float zf = std::min(r0.z - 0.25f * B, head.z);
                    const float yf = std::max(r0.y, eb.headTop) + 0.10f * B;
                    sc.put(tubeOf("pumpline", spline({head, head + v3(0, 0.12f * B, -0.10f * B), v3(head.x, yf - 0.1f * B, zf),
                                                      v3(0.5f * (head.x + r0.x), yf, zf), v3(r0.x, r0.y, r0.z - 0.12f * B), r0}, 10), 0.022f * B, 8), M4{}, kSteel);
                }
                if (std::fabs(lo.x - hi.x) < 1e-3f) break;
            }
        } else {
            const bool inlinePump = std::fabs(eb.pump.x) > 0.3f * B;
            // (lines in the order of the injectors along the engine: the nearest gets the innermost lane)
            std::vector<int> ord(n);
            for (int i = 0; i < n; ++i) ord[i] = i;
            std::sort(ord.begin(), ord.end(), [&](int a, int b2) { return eb.injTop[a].z < eb.injTop[b2].z; });
            for (int j = 0; j < n; ++j) {
                const int i = ord[j];
                const float a = 2.0f * kPiF * j / std::max(1, n);
                const V3 out = head + v3(0.09f * B * std::cos(a), 0.09f * B * std::sin(a), 0);
                const V3 t = eb.injTop[i];
                const float yTop = std::max(eb.headTop, t.y) + 0.10f * B + 0.025f * B * j;
                std::vector<V3> ctrl;
                if (inlinePump) {
                    const float sx = eb.pump.x < 0 ? -1.0f : 1.0f;
                    const float xL = sx * (0.80f * B + 0.035f * B * j), yLow = t.y - 0.95f * B - 0.03f * B * j;
                    const float zUp = t.z + 0.50f * B;   // between this cylinder's runner and the next
                    ctrl = {out, out + v3(0, 0.05f * B, 0.12f * B), v3(xL, yLow, out.z + 0.40f * B), v3(xL, yLow, zUp - 0.25f * B),
                            v3(xL, yLow + 0.25f * B, zUp), v3(xL, yTop - 0.10f * B, zUp), v3(0.5f * (xL + t.x), yTop, zUp - 0.10f * B),
                            v3(t.x, yTop, t.z + 0.12f * B), v3(t.x, t.y + 0.12f * B, t.z), t + v3(0, 0.04f * B, 0)};
                } else {   // a pump on top in the V: over to each injector above the heads
                    ctrl = {out, out + v3(0, 0.05f * B, 0.12f * B), v3(0.5f * (out.x + t.x), yTop, out.z + 0.30f * B), v3(t.x, yTop, t.z),
                            t + v3(0, 0.04f * B, 0)};
                }
                sc.put(tubeOf("hpline", spline(ctrl, 8), 0.024f * B, 8), M4{}, kSteel);
            }
        }
    }
    // glow plugs: a bus wire over their tops, glowing the app's orange while they heat
    if (eb.glowTop.size() > 1) {
        Material wm = kGold;
        if (live.glowPlugOn) wm = emissive(kGold, kAccent[0] * 0.35f, kAccent[1] * 0.20f, kAccent[2] * 0.05f);
        float x0 = 1e9f, x1 = -1e9f;
        for (const V3& q : eb.glowTop) { x0 = std::min(x0, q.x); x1 = std::max(x1, q.x); }
        const bool twoBanks = x1 - x0 > 0.8f * B;
        for (int side = 0; side < (twoBanks ? 2 : 1); ++side) {
            std::vector<V3> g;
            for (const V3& q : eb.glowTop) if (!twoBanks || ((q.x >= 0.5f * (x0 + x1)) == (side == 1))) g.push_back(q);
            if (g.size() < 2) continue;
            std::sort(g.begin(), g.end(), [](const V3& a, const V3& b2) { return a.z < b2.z; });
            std::vector<V3> w;
            for (const V3& q : g) { w.push_back(q + v3(0, 0.02f * B, -0.08f * B)); w.push_back(q + v3(0, 0.03f * B, 0)); w.push_back(q + v3(0, 0.02f * B, 0.08f * B)); }
            sc.put(tubeOf(side ? "glowbus1" : "glowbus0", spline(w, 3), 0.014f * B, 6), M4{}, wm);
        }
    }
    // EGR: from the exhaust manifold round the front of the head into the intake, the valve body at the intake end;
    // exhaust-coloured sparks run along it while it is open
    if (p.egrValveDia_mm > 0 && !eb.ex.empty() && eb.plenumLen > 0) {
        const PortOut& e0 = eb.ex.front();
        const float rr = clampv(p.egrValveDia_mm * 0.5f, 0.06f * B, 0.20f * B);
        const V3 a = e0.p + e0.d * (0.75f * B) + v3(0, -0.15f * B, 0);
        const V3 vlv = eb.plenum + v3(0, 0, -0.5f * eb.plenumLen + 0.25f * B) + v3(0, -0.40f * B, 0);
        // stainless tube as it is made: straight runs and rounded bends - forward from the manifold to in front of the
        // timing drive, up, across above it, down and back into the valve's side under the plenum; a flexible bellows on
        // the rising leg (it takes up the engine's heat growth), and a cooler body on the crossing leg if fitted
        const float zF = eb.zFront - 0.55f * B;
        const float yTop = std::max({a.y, vlv.y, eb.headTop}) + 0.25f * B;
        const V3 vIn = v3(vlv.x, vlv.y + 0.10f * B, vlv.z);
        const std::vector<V3> path = routePipe({a, v3(a.x, a.y, zF), v3(a.x, yTop, zF), v3(vlv.x, yTop, zF), v3(vIn.x, vIn.y, zF), vIn}, 0.30f * B);
        sc.put(tubeOf("egr", path, rr), M4{}, kSteel);
        {   // bellows: corrugation rings along the middle of the rising leg
            const Cached& ring = meshOf(key("egrbellow", {rr}), [&] {
                std::vector<V3> c;
                for (int k = 0; k < 20; ++k) { const float an = 2 * kPiF * k / 20; c.push_back(v3(rr * 1.18f * std::cos(an), 0, rr * 1.18f * std::sin(an))); }
                return mesh::tube(c, std::vector<float>(c.size(), 0.25f * rr), 6, true, false); });
            const float y0 = a.y + 0.30f * (yTop - a.y), y1 = a.y + 0.70f * (yTop - a.y);
            for (int k = 0; k < 7; ++k) sc.put(ring, translate(v3(a.x, y0 + (y1 - y0) * k / 6.0f, zF)), kSteel);
        }
        if (p.egrCooler) {   // the cooler: a shell round the tube bundle, end cones, coolant hoses in and out
            const float span = std::fabs(vlv.x - a.x), cl = std::min(1.0f * B, 0.55f * span), cr = std::max(0.22f * B, rr * 1.8f);
            const V3 mid = v3(0.5f * (a.x + vlv.x), yTop, zF);
            const V3 ax = v3(vlv.x > a.x ? 1.0f : -1.0f, 0, 0);
            const Cached& shell = meshOf(key("egrcooler", {cl, cr, rr}), [&] {
                return mesh::lathe({{rr, -0.5f * cl - 0.15f * B}, {cr, -0.5f * cl}, {cr, 0.5f * cl}, {rr, 0.5f * cl + 0.15f * B}}, 28); });
            sc.put(shell, alongY(mid, ax), mat(0.50f, 0.52f, 0.56f, 0.6f, 0.4f));
            for (int s2 = -1; s2 <= 1; s2 += 2) {
                const V3 h0 = mid + ax * (s2 * 0.32f * cl) + v3(0, cr, 0);
                sc.put(tubeOf("egrhose", routePipe({h0, h0 + v3(0, 0.18f * B, 0), h0 + v3(0, 0.18f * B, 0.45f * B)}, 0.10f * B), 0.05f * B, 10), M4{}, kRubber);
            }
        }
        const Cached& valve = meshOf(key("egrvalve", {B}), [&] {
            Data d = mesh::lathe({{0, 0}, {0.16f * B, 0}, {0.16f * B, 0.22f * B}, {0.10f * B, 0.30f * B}, {0, 0.30f * B}}, 28);
            mesh::append(d, mesh::box(0.06f * B, 0.08f * B, 0.06f * B), translate(v3(0, 0.36f * B, 0)));
            return d; });
        const float open = clampv(live.egr, 0.0f, 1.0f);
        sc.put(valve, translate(vlv), open > 0.05f ? emissive(kAlu, kAccent[0] * 0.10f * open, kAccent[1] * 0.06f * open, 0.01f) : kAlu);
        if (open > 0.05f) {
            for (int d = 0; d < 8; ++d) {
                const float u = std::fmod(rnd(21, d) + live.time * 0.6f, 1.0f);
                sc.glow(path[(size_t)(u * (path.size() - 1))], 0.6f * rr, kAccent[0], 0.50f, 0.20f, 0.45f * open);
            }
        }
        sc.label(vlv + v3(0, 0.55f * B, 0), "EGR");
    }
}

// ------------------------------------------------------------------ turbo / supercharger / intercooler
void buildBoost(Scene& sc, const EngineParams& p, const EngineBits& eb, const EngineViewLive& live, float crankDeg, V3 turbineIn, V3 throttleIn,
                V3 throttleDir) {
    const float B = eb.B;
    // spin: the real shaft speed slowed down by the view's slow motion, like the crank (Engine view > Slow motion), and
    // capped at 5 rev/s on screen (faster would strobe: the blades seem to stand or run backwards)
    static float spin = 0, lastT = -1;
    const float dt = lastT < 0 ? 0.0f : clampv(live.time - lastT, 0.0f, 0.1f);
    lastT = live.time;
    spin = std::fmod(spin + dt * 2 * kPiF * std::min(live.shaftRpm / 60.0f * std::max(0.0f, live.animSpeed), 5.0f), 2 * kPiF);
    const float rp = 0.20f * B;   // boost pipe radius
    // intercooler in front of the engine, if fitted: compressor -> core -> throttle (a Roots / screw blower sits on the
    // plenum and blows straight down into it: no front core)
    const bool frontIc = p.intercooler && (p.induction == Induction::Turbo || p.scType == Supercharger::Centrifugal);
    V3 icC = v3(0, 0, 0);
    float icHx = 0;
    if (frontIc) {
        icC = v3(0, -0.2f * B, eb.zFront - (p.induction == Induction::Turbo ? 1.6f : 2.3f) * B);
        const float hx = 1.3f * B, hy = 0.7f * B, hz = 0.14f * B;
        icHx = hx + 0.16f * B;
        const Cached& core = meshOf(key("iccore", {B}), [&] { return mesh::box(hx, hy, hz); });
        sc.put(core, translate(icC), mat(0.20f, 0.21f, 0.23f, 0.5f, 0.6f));
        const Cached& fin = meshOf(key("icfin", {B}), [&] { return mesh::box(hx * 1.02f, 0.008f * B, hz * 1.12f); });
        for (int k = 0; k < 12; ++k) sc.put(fin, translate(icC + v3(0, -hy + (k + 0.5f) * 2 * hy / 12, 0)), kAlu);
        const Cached& tank = meshOf(key("ictank", {B}), [&] { return mesh::box(0.16f * B, hy * 1.1f, hz * 1.4f); });
        sc.put(tank, translate(icC + v3(-icHx, 0, 0)), kAlu);
        sc.put(tank, translate(icC + v3(icHx, 0, 0)), kAlu);
    }
    // boost tubing: straight runs with rounded bends (routePipe), couplers at the joints
    auto pipe = [&](const std::vector<V3>& pts, const char* nm) {
        sc.put(tubeOf(nm, routePipe(pts, 0.45f * B), rp), M4{}, kAlu);
        const Cached& coupler = meshOf(key("coupler", {rp}), [&] { return mesh::lathe({{rp * 1.12f, -0.07f * B}, {rp * 1.12f, 0.07f * B}, {rp * 0.9f, 0.07f * B}, {rp * 0.9f, -0.07f * B}}, 20); });
        sc.put(coupler, alongY(pts.back(), pts.back() - pts[pts.size() - 2]), kRubber);
    };
    // compressor outlet -> (core ->) throttle. Out of the compressor along its outlet, forward at that height (outside the
    // engine), across behind the core to its end tank; out of the other tank, back and across in front of the engine to
    // the throttle's mouth. The core's inlet is on the side away from the throttle, so the two runs never cross.
    auto boostTo = [&](V3 from, V3 dir) {
        const V3 qa = throttleIn + throttleDir * (0.55f * B);   // in front of the throttle's mouth
        const V3 p1 = from + dir * (0.40f * B);
        if (!frontIc) {
            pipe({from, p1, v3(p1.x, p1.y, qa.z), v3(qa.x, p1.y, qa.z), qa, throttleIn}, "boostA");
            return;
        }
        const float tSide = std::fabs(throttleIn.x) > 0.2f * B ? (throttleIn.x < 0 ? -1.0f : 1.0f) : (from.x < 0 ? 1.0f : -1.0f);
        const V3 inP = icC + v3(-tSide * icHx, 0, 0.16f * B), outP = icC + v3(tSide * icHx, 0, 0.16f * B);
        const float zb = icC.z + 0.50f * B;   // the run behind the core
        pipe({from, p1, v3(p1.x, p1.y, zb), v3(inP.x, p1.y, zb), v3(inP.x, inP.y, zb), inP}, "boostA");
        pipe({outP, v3(outP.x, outP.y, zb), v3(outP.x, qa.y, zb), v3(qa.x, qa.y, zb), qa, throttleIn}, "boostB");
    };
    if (p.induction == Induction::Turbo) {
        // the turbo: shaft along the crank, turbine at the back on its inlet flange (turbineIn: the gas goes down into
        // it), the bearing housing, the compressor in front - solid housings, the compressor wheel seen in its inlet
        const Snail sT{0.30f * B, 0.20f * B, 0.12f * B, 0.25f * B, 0.28f * B}, sC{0.34f * B, 0.18f * B, 0.12f * B, 0.26f * B, 0.32f * B};
        const V3 axT = v3(0, 0, 1);
        M4 mT = snailFrame(sT, turbineIn, axT, v3(0, 1, 0));
        V3 E, D;
        snailPort(sT, E, D);
        mT = translate(turbineIn - xform(mT, E)) * mT;   // the port on the flange
        const V3 T = xform(mT, v3(0, 0, 0));
        const Cached& tv = meshOf(key("tsnail", {B}), [&] { return snailMesh(sT); });
        {   // a see-through housing, tinted and glowing with the exhaust heat, the turbine wheel turning in it
            const Material hm = hot(live.egtGas, 0.9f);
            Material gm = glassMat(0.32f, 0.62f, 0.53f, 0.48f);
            gm.er = hm.er; gm.eg = hm.eg; gm.eb = hm.eb;
            sc.put(tv, mT, gm);
            const Cached& tw = meshOf(key("turbwheel", {B}), [&] { return wheelMesh(0.25f * B, 0.22f * B, 11); });
            sc.put(tw, alongY(T + v3(0, 0, -0.10f * B), v3(0, 0, 1)) * rotY(-spin), emissive(kSteelDark, hm.er * 0.6f, hm.eg * 0.6f, hm.eb * 0.6f));
        }
        // bearing housing (CHRA) forward of the turbine, then the compressor
        const V3 chraA = T + v3(0, 0, -sT.w);
        const Cached& chra = meshOf(key("chra2", {B}), [&] { return mesh::lathe({{0, 0}, {0.17f * B, 0}, {0.21f * B, 0.06f * B}, {0.21f * B, 0.24f * B},
                                                                                  {0.17f * B, 0.30f * B}, {0, 0.30f * B}}, 28); });
        sc.put(chra, alongY(chraA, v3(0, 0, -1)), kSteelDark);
        const V3 Cc = chraA + v3(0, 0, -(0.30f * B + sC.w));
        const M4 mC = snailFrame(sC, Cc, v3(0, 0, -1), v3(0, 1, 0));
        const Cached& cv = meshOf(key("csnail", {B}), [&] { return snailMesh(sC); });
        sc.put(cv, mC, glassMat(0.30f, 0.78f, 0.79f, 0.82f));
        // inlet: an open bell on the front face, the wheel turning inside, a short tube and the air filter
        const V3 inA = Cc + v3(0, 0, -sC.w);
        const Cached& bell = meshOf(key("cbell", {B}), [&] { return mesh::lathe({{0.22f * B, 0}, {0.26f * B, 0}, {0.27f * B, 0.20f * B}, {0.24f * B, 0.22f * B}, {0.22f * B, 0.03f * B}}, 28); });
        sc.put(bell, alongY(inA, v3(0, 0, -1)), kAlu);
        const Cached& wh = meshOf(key("compwheel", {B}), [&] { return wheelMesh(0.30f * B, 0.24f * B, 10); });
        sc.put(wh, alongY(Cc + v3(0, 0, 0.10f * B), v3(0, 0, -1)) * rotY(spin), kSilver);
        const V3 f0 = inA + v3(0, 0, -0.22f * B);
        sc.put(tubeOf("cinlet", {f0, f0 + v3(0, 0, -0.25f * B)}, 0.23f * B), M4{}, kAlu);
        const Cached& filt = meshOf(key("airfilter", {B}), [&] { return mesh::lathe({{0, 0}, {0.26f * B, 0}, {0.30f * B, 0.08f * B}, {0.30f * B, 0.42f * B}, {0.22f * B, 0.50f * B}, {0, 0.50f * B}}, 28); });
        sc.put(filt, alongY(f0 + v3(0, 0, -0.25f * B), v3(0, 0, -1)), kRubber);
        // downpipe: out of the turbine's back face, down and back to the silencer and the tail
        const V3 o0 = T + v3(0, 0, sT.w);
        const float rtl = clampv(p.pipeDia_mm * 0.5f, 0.15f * B, 0.4f * B);
        sc.put(tubeOf("toutlet", {o0, o0 + v3(0, 0, 0.18f * B)}, 0.22f * B), M4{}, hot(live.egtGas, 0.8f));
        {   // straight back past the flywheel, then the silencer and the tail
            const float zb = std::max(o0.z + 0.6f * B, eb.zRear + 0.7f * B);
            const V3 d0 = o0 + v3(0, 0, 0.18f * B);
            tailWithFlame(sc, {d0, v3(d0.x, d0.y - 0.15f * B, zb), v3(d0.x, d0.y - 0.35f * B, zb + 1.6f * B), v3(d0.x, d0.y - 0.40f * B, zb + 3.2f * B),
                               v3(d0.x, d0.y - 0.40f * B, zb + 4.4f * B)}, rtl, live, live.egtGas, 7, 1.6f * B);
        }
        V3 cE, cD;
        snailPort(sC, cE, cD);
        boostTo(xform(mC, cE), norm(xdir(mC, cD)));
        if (p.vgt) {   // variable geometry: the vane actuator (a vacuum can) on the turbine housing
            const Cached& can = meshOf(key("vgtcan", {B}), [&] { return mesh::lathe({{0, 0}, {0.13f * B, 0}, {0.14f * B, 0.04f * B}, {0.14f * B, 0.15f * B}, {0, 0.17f * B}}, 24); });
            const V3 outSide = v3(T.x >= 0 ? 1.0f : -1.0f, 0, 0);
            sc.put(can, alongY(T + outSide * (0.62f * B) + v3(0, 0.10f * B, 0), outSide), kSteelDark);
        }
        sc.label(T + v3(0, 0.95f * B, 0), p.vgt ? "VGT turbo" : "turbo");
        return;
    }
    // supercharger: crank-driven by a belt
    const float ratio = std::max(0.5f, p.scRatio);
    const float ang = -crankDeg * kDegF * (p.scType == Supercharger::Centrifugal ? std::min(ratio, 3.0f) : ratio);
    const V3 cp = eb.crankFront;
    if (p.scType == Supercharger::Centrifugal) {
        // a centrifugal head unit (ProCharger / Vortech style) low beside the crankcase on its bracket: the pulley in the
        // crank pulley's plane, the gear case in front of it, the compressor housing ahead of that, its outlet up, the
        // inlet turned outboard to its filter; a straight belt from the crank pulley
        const float side = -1.0f;
        const V3 S = cp + v3(side * 1.75f * B, 0.40f * B, 0.0f);
        const V3 pul = v3(S.x, S.y, cp.z);
        const Cached& pulley = meshOf(key("scpulley", {B}), [&] { return mesh::extrude(mesh::gearOutline(0.26f * B, 0.28f * B, 36), -0.08f * B, 0.08f * B); });
        sc.put(pulley, translate(pul) * rotZ(ang * 0.3f), kSteel);
        const Cached& crankPul = meshOf(key("crankpulley", {B}), [&] { return mesh::extrude(mesh::gearOutline(0.40f * B, 0.42f * B, 40), -0.08f * B, 0.08f * B); });
        sc.put(crankPul, translate(cp) * rotZ(-crankDeg * kDegF), kSteel);
        sc.put(tubeOf("scbelt", beltAround(cp, 0.43f * B, pul, 0.29f * B), 0.025f * B, 8), M4{}, kRubber);
        const Cached& body = meshOf(key("scgearcase2", {B}), [&] { return mesh::lathe({{0, 0}, {0.26f * B, 0}, {0.30f * B, 0.05f * B}, {0.30f * B, 0.36f * B}, {0.24f * B, 0.42f * B}, {0, 0.42f * B}}, 28); });
        sc.put(body, alongY(pul + v3(0, 0, -0.09f * B), v3(0, 0, -1)), kSteelDark);
        const Snail sC{0.34f * B, 0.18f * B, 0.12f * B, 0.26f * B, 0.32f * B};
        const V3 Cc = pul + v3(0, 0, -(0.51f * B + sC.w));
        const M4 mC = snailFrame(sC, Cc, v3(0, 0, -1), v3(0, 1, 0));
        const Cached& cv = meshOf(key("csnail", {B}), [&] { return snailMesh(sC); });
        sc.put(cv, mC, glassMat(0.30f, 0.78f, 0.79f, 0.82f));
        const V3 inA = Cc + v3(0, 0, -sC.w);
        const Cached& bell = meshOf(key("cbell", {B}), [&] { return mesh::lathe({{0.22f * B, 0}, {0.26f * B, 0}, {0.27f * B, 0.20f * B}, {0.24f * B, 0.22f * B}, {0.22f * B, 0.03f * B}}, 28); });
        sc.put(bell, alongY(inA, v3(0, 0, -1)), kAlu);
        const Cached& wh = meshOf(key("compwheel", {B}), [&] { return wheelMesh(0.30f * B, 0.24f * B, 10); });
        sc.put(wh, alongY(Cc + v3(0, 0, 0.10f * B), v3(0, 0, -1)) * rotY(spin), kSilver);
        const V3 e0 = inA + v3(0, 0, -0.20f * B), e1 = e0 + v3(0, 0, -0.30f * B), e2 = e1 + v3(side * 0.45f * B, 0, 0);
        sc.put(tubeOf("scinlet", routePipe({e0, e1, e2}, 0.25f * B), 0.23f * B), M4{}, kAlu);
        const Cached& filt = meshOf(key("airfilter", {B}), [&] { return mesh::lathe({{0, 0}, {0.26f * B, 0}, {0.30f * B, 0.08f * B}, {0.30f * B, 0.42f * B}, {0.22f * B, 0.50f * B}, {0, 0.50f * B}}, 28); });
        sc.put(filt, alongY(e2, v3(side, 0, 0)), kRubber);
        V3 cE, cD;
        snailPort(sC, cE, cD);
        boostTo(xform(mC, cE), norm(xdir(mC, cD)));
        sc.label(Cc + v3(0, 0.85f * B, 0), "supercharger");
        return;
    }
    // Roots / twin-screw on the plenum
    const bool screw = p.scType == Supercharger::TwinScrew;
    const float L = std::max(1.2f * B, 0.85f * eb.plenumLen);
    const V3 S = eb.plenum + v3(0, 0.75f * B, 0);
    const float hx = 0.62f * B, hy = 0.42f * B;
    const Cached& casing = meshOf(key("sccase", {hx, hy, L}), [&] { return mesh::extrude(mesh::roundedRect(hx, hy, 0.30f * B), -0.5f * L, 0.5f * L); });
    sc.put(casing, translate(S), glassMat(0.26f, 0.78f, 0.79f, 0.82f));
    const float Rr = 0.27f * B;
    const Cached& rotor = meshOf(key("scrotor", {Rr, L, screw ? 1.0f : 0.0f}), [&] {
        const Poly lob = lobedOutline(Rr, screw ? 4 : 3, 0.38f);
        return screw ? mesh::twisted(lob, -0.48f * L, 0.48f * L, 2.2f, 30) : mesh::extrude(lob, -0.48f * L, 0.48f * L); });
    sc.put(rotor, translate(S + v3(-0.29f * B, 0, 0)) * rotZ(ang), kSilver);
    sc.put(rotor, translate(S + v3(0.29f * B, 0, 0)) * rotZ(-ang + kPiF / (screw ? 4 : 3)), kSilver);
    // drive: snout to the front, pulley in the crank pulley's plane, belt
    const V3 pul = v3(S.x, S.y + 0.1f * B, cp.z);
    sc.put(tubeOf("scsnout", {S + v3(0, 0, -0.5f * L), pul}, 0.07f * B, 12), M4{}, kSteel);
    const Cached& pulley = meshOf(key("scpulley", {B}), [&] { return mesh::extrude(mesh::gearOutline(0.26f * B, 0.28f * B, 36), -0.08f * B, 0.08f * B); });
    sc.put(pulley, translate(pul) * rotZ(ang), kSteel);
    const Cached& crankPul = meshOf(key("crankpulley", {B}), [&] { return mesh::extrude(mesh::gearOutline(0.40f * B, 0.42f * B, 40), -0.08f * B, 0.08f * B); });
    sc.put(crankPul, translate(cp) * rotZ(-crankDeg * kDegF), kSteel);
    blowerBelt(sc, cp, 0.43f * B, pul, 0.29f * B, 1.0f, false, B, ang * 0.3f);
    sc.label(S + v3(0, 0.75f * B, 0), screw ? "twin-screw" : "Roots");
    (void)throttleIn;
}

// ------------------------------------------------------------------ two-stroke cylinder
void buildTwoStrokeCyl(Scene& sc, const EngineParams& p, const M4& cf, double phi, float crankDeg, const EngineViewLive& live, float glassA,
                       float deck, float crownTop, int idx) {
    const float B = p.bore_mm, r = p.stroke_mm * 0.5f, l = std::max(p.rod_mm, r * 1.2f);
    const float wall = 0.07f * B;
    auto crownAt = [&](double deg) {   // crown height (cylinder frame) at a crank angle after TDC
        const float th = (float)(deg * kDeg), sn = std::sin(th), cs = std::cos(th);
        return r * cs + std::sqrt(std::max(1.0f, l * l - r * r * sn * sn)) + 0.30f * B;
    };
    const float epo = clampv(p.tsExhaustOpen, 45.0f, 150.0f), tpo = clampv(p.tsTransferOpen, epo, 170.0f);
    const float yBdc = crownAt(180.0), yEx = crownAt(epo), yTr = crownAt(tpo);
    const bool exOpen = phi > epo && phi < 360.0 - epo, trOpen = phi > tpo && phi < 360.0 - tpo;
    // head: a glass dome over the bore, spark plug in the middle
    const Cached& head = meshOf(key("2thead", {B, deck}), [&] {
        return mesh::lathe({{0.5f * B + wall, deck}, {0.5f * B + wall, deck + 0.12f * B}, {0.40f * B, deck + 0.24f * B}, {0.15f * B, deck + 0.30f * B}, {0, deck + 0.31f * B}}, 40); });
    sc.put(head, cf, glassMat(glassA * 0.9f));
    const Cached& plug = meshOf(key("plug", {B}), [&] { return plugMesh(B); });
    sc.put(plug, cf * translate(v3(0, deck + 0.24f * B, 0)), kCeramic);
    const double sinceSpark = wrapPos(phi + live.advance, 360.0);
    if (sinceSpark < 12.0) sc.glow(xform(cf, v3(0, deck - 0.02f * B, 0)), 0.16f * B, 1.0f, 0.92f, 0.60f, 1.0f - (float)(sinceSpark / 12.0));
    // exhaust port window (+x) and its duct; transfer windows on both sides (z) with their passages from the crankcase
    const float exW = clampv(p.tsExhaustWidth, 0.2f, 1.2f) * B, trW = clampv(p.tsTransferWidth, 0.2f, 1.6f) * B * 0.25f;
    const Cached& exWin = meshOf(key("2texwin", {B, yEx, yBdc, exW}), [&] { return mesh::box(0.03f * B, 0.5f * (yEx - yBdc + 0.12f * B), 0.5f * std::min(exW, 0.9f * B)); });
    const float blow = exOpen ? 1.0f - smooth01(0.0f, 60.0f, (float)(phi - epo)) : 0.0f;
    sc.put(exWin, cf * translate(v3(0.5f * B + 0.5f * wall, 0.5f * (yEx + yBdc - 0.12f * B), 0)),
           emissive(kSteelDark, kAccent[0] * 0.8f * blow, kAccent[1] * 0.4f * blow, kAccent[2] * 0.1f * blow));
    const Cached& trWin = meshOf(key("2ttrwin", {B, yTr, yBdc, trW}), [&] { return mesh::box(0.5f * trW * 2.0f, 0.5f * (yTr - yBdc + 0.10f * B), 0.03f * B); });
    Material trM = glassMat(0.45f, kCharge[0], kCharge[1], kCharge[2]);
    if (trOpen) trM = emissive(trM, kCharge[0] * 0.25f, kCharge[1] * 0.35f, kCharge[2] * 0.5f);
    for (int s = -1; s <= 1; s += 2) {
        sc.put(trWin, cf * translate(v3(-0.10f * B, 0.5f * (yTr + yBdc - 0.10f * B), s * (0.5f * B + 0.5f * wall))), trM);
        const std::vector<V3> duct = spline({v3(-0.15f * B, 0.25f * r, s * 0.55f * B), v3(-0.25f * B, 0.5f * (yBdc + 0.25f * r), s * 0.78f * B),
                                             v3(-0.15f * B, 0.5f * (yTr + yBdc), s * 0.62f * B), v3(-0.10f * B, 0.5f * (yTr + yBdc), s * (0.5f * B + wall))}, 8);
        std::vector<V3> dw(duct.size());
        for (size_t k = 0; k < duct.size(); ++k) dw[k] = xform(cf, duct[k]);
        sc.put(tubeOf("2tduct", dw, 0.13f * B), M4{}, glassMat(glassA * 1.4f, kCharge[0], kCharge[1], kCharge[2]));
        if (trOpen) for (int d = 0; d < 4; ++d) sc.glow(dw[(size_t)((rnd(idx * 7 + s, d) * 0.9f) * dw.size())], 0.05f * B, kCharge[0], kCharge[1], kCharge[2], 0.6f);
    }
    // crankcase: a glass box round the crank, with the reed block on the intake side (-x)
    const float ccHx = std::max(1.45f * r, 0.62f * B), ccY0 = -1.45f * r, ccY1 = std::min(1.1f * r, yBdc - 0.45f * B);
    const Cached& cc = meshOf(key("2tcc", {ccHx, ccY0, ccY1, B}), [&] { return mesh::extrude(mesh::roundedRect(ccHx, 0.5f * (ccY1 - ccY0), 0.35f * B), -0.55f * B, 0.55f * B); });
    sc.put(cc, cf * translate(v3(0, 0.5f * (ccY0 + ccY1), 0)), glassMat(glassA * 0.8f));
    // reed: open while the rising piston draws the crankcase down (~210 deg .. 40 deg after TDC), lagging behind
    double u = wrapPos(phi - 210.0, 360.0);
    const float open = u < 190.0 ? (float)std::pow(std::sin(kPi * u / 190.0), 0.7) : 0.0f;
    const V3 rb = v3(-ccHx - 0.05f * B, 0.15f * r, 0);
    const Cached& block = meshOf(key("2treed", {B}), [&] { return mesh::extrude({{0, -0.32f * B}, {0.30f * B, 0}, {0, 0.32f * B}, {-0.06f * B, 0.32f * B}, {-0.06f * B, -0.32f * B}}, -0.30f * B, 0.30f * B); });
    sc.put(block, cf * translate(rb), kSteelDark);
    const Cached& petal = meshOf(key("2tpetal", {B}), [&] { return mesh::box(0.19f * B, 0.005f * B, 0.27f * B); });
    for (int s = -1; s <= 1; s += 2) {
        const float a0 = s * 0.82f, a = a0 + s * open * 0.30f;   // petal angle: lies on the block, swings off it
        sc.put(petal, cf * translate(rb + v3(0, s * 0.32f * B, 0)) * rotZ(-a) * translate(v3(0.19f * B, 0, 0)), mat(0.18f, 0.18f, 0.2f, 0.1f, 0.5f));
    }
    const std::vector<V3> inl = {v3(rb.x - 0.05f * B, rb.y, 0), v3(rb.x - 0.55f * B, rb.y + 0.10f * B, 0), v3(rb.x - 0.95f * B, rb.y + 0.35f * B, 0)};
    std::vector<V3> inw;
    for (const V3& q : inl) inw.push_back(xform(cf, q));
    sc.put(tubeOf("2tintake", spline(inw, 6), clampv(p.throttleBore_mm * 0.5f, 0.12f * B, 0.30f * B)), M4{}, kAlu);
    if (open > 0.2f) for (int d = 0; d < 4; ++d) sc.glow(lerp(inw[0], inw[2], rnd(idx + 50, d)), 0.05f * B, kCharge[0], kCharge[1], kCharge[2], 0.5f * open);
    (void)crownTop; (void)crankDeg;

    // exhaust: the expansion chamber (Blair's design, as the simulation builds it) or a plain header pipe
    const V3 e0 = v3(0.5f * B + wall + 0.02f * B, 0.5f * (yEx + yBdc), 0);
    std::vector<V3> ctrl;
    std::vector<float> segLen, segD;
    float dStart = std::sqrt(4.0f * exW * (yEx - yBdc) / kPiF);
    if (p.tsPipe) {
        const TunedPipeGeom g = tunedPipeGeometry(p);
        for (int k = 0; k < 7; ++k) { segLen.push_back((float)g.len[k] * 1000.0f); segD.push_back((float)g.dia[k] * 1000.0f); }
        dStart = (float)g.dia[0] * 1000.0f;
    } else {
        segLen = {std::max(100.0f, p.primaryLen_mm[0])}; segD = {p.primaryDia_mm};
    }
    float total = 0;
    for (float s : segLen) total += s;
    // path: folded like a road-racing bike's - out of the port (forward, +x), down in front of the crankcase, back
    // underneath it beside the engine, up behind the cylinder and back under the seat; what the bends do not use runs
    // on straight back. (The pipe is ~3-4 times the engine's height long: folded, the view can still show the engine.)
    const float k1 = std::min(segLen[0], 1.0f * B);
    const float yLow = -1.45f * r - 0.95f * B, side = (idx % 2 == 0 ? 1.0f : -1.0f) * 1.1f * B;
    ctrl = {e0, e0 + v3(0.6f * k1, -0.08f * B, 0), e0 + v3(k1 + 0.40f * B, -0.65f * B, 0.25f * side),
            v3(e0.x + k1 + 0.55f * B, 0.5f * (e0.y + yLow), 0.7f * side), v3(e0.x + k1 + 0.10f * B, yLow + 0.35f * B, side),
            v3(e0.x - 0.20f * B, yLow, side), v3(-ccHx - 0.60f * B, yLow + 0.15f * B, side), v3(-ccHx - 1.50f * B, yLow + 0.90f * B, side),
            v3(-ccHx - 1.80f * B, 0.5f * (yLow + deck), side), v3(-ccHx - 2.10f * B, deck - 0.30f * B, side), v3(-ccHx - 3.00f * B, deck + 0.20f * B, side)};
    std::vector<V3> path = spline(ctrl, 10);
    const size_t nFold = path.size();
    float acc = 0;
    for (size_t k = 1; k < path.size(); ++k) acc += len(path[k] - path[k - 1]);
    const V3 dirEnd = norm(v3(-1.0f, 0.18f, 0.0f));
    const int extra = std::max(2, (int)((total - acc) / (0.08f * B)));
    const V3 last = path.back();
    for (int k = 1; k <= extra; ++k) path.push_back(last + dirEnd * (std::max(0.0f, total - acc) * k / extra));
    // radius along the path from the segment profile (diameters linear along each cone)
    std::vector<float> rad(path.size());
    float s = 0;
    for (size_t k = 0; k < path.size(); ++k) {
        if (k) s += len(path[k] - path[k - 1]);
        float x = s, d0 = dStart, d = segD.back();
        for (size_t j = 0; j < segLen.size(); ++j) {
            if (x <= segLen[j] || j + 1 == segLen.size()) { d = j == 0 || j == 4 || j == 6 ? segD[j] : d0 + (segD[j] - d0) * clampv(x / segLen[j], 0.0f, 1.0f); break; }
            x -= segLen[j]; d0 = segD[j];
        }
        rad[k] = 0.5f * d;
    }
    std::vector<V3> pw(path.size());
    for (size_t k = 0; k < path.size(); ++k) pw[k] = xform(cf, path[k]);
    const float egt = live.egtC[idx] > 1.0f ? live.egtC[idx] : live.egtGas;
    // the camera frames the engine and the folded part of the pipe; the straight run back and the silencer may leave
    // the view (zoom out to see them)
    sc.fit = false;
    sc.put(tubeVar("2tpipe", pw, rad, (float)idx + (p.tsPipe ? 0.5f : 0.0f)), M4{}, hot(egt, 0.7f));
    for (size_t k = 0; k < std::min(nFold, pw.size()); k += 3)
        for (int c = 0; c < 8; ++c) sc.fitPoint(pw[k] + v3(c & 1 ? rad[k] : -rad[k], c & 2 ? rad[k] : -rad[k], c & 4 ? rad[k] : -rad[k]));
    // the wave: the blowdown pulse runs down the pipe from the port opening, the reflection comes back by the time
    // the port closes (the pipe is tuned for that)
    if (exOpen) {
        const float frac = (float)((phi - epo) / (360.0 - 2.0 * epo));   // 0..1 over the exhaust period
        const float sw = frac < 0.5f ? frac * 2.0f * total : (2.0f - frac * 2.0f) * total;
        float acc2 = 0;
        for (size_t k = 1; k < pw.size(); ++k) {
            const float seg = len(path[k] - path[k - 1]);
            if (acc2 + seg >= sw) {
                const float t = (sw - acc2) / std::max(1e-3f, seg);
                const V3 at = lerp(pw[k - 1], pw[k], t);
                const float amp = frac < 0.5f ? 1.0f : 0.7f;
                sc.glow(at, rad[k] * 1.3f, kAccent[0], 0.50f, 0.15f, 0.55f * amp);
                break;
            }
            acc2 += seg;
        }
    }
    // silencer and tail after the pipe
    const V3 endW = pw.back(), dW = norm(pw.back() - pw[pw.size() - 2]);
    const V3 upW = norm(xdir(cf, v3(0, 1, 0)));
    tailWithFlame(sc, {endW, endW + dW * (0.8f * B) + upW * (0.15f * B), endW + dW * (2.2f * B) + upW * (0.6f * B), endW + dW * (3.2f * B) + upW * (0.9f * B)},
                  std::max(rad.back(), 0.10f * B), live, egt, idx + 20);
    sc.fit = true;
}

// ------------------------------------------------------------------ rotary
void buildRotaryEngine(Scene& sc, const EngineParams& p, const float phases[kMaxCyl], float shaftDeg, const EngineViewLive& live, float glassA,
                       bool turbo, EngineBits& eb, V3& turbineIn, V3& throttleIn, V3& throttleDir) {
    const int n = clampv(p.cylinders, 1, 4);
    const float R = p.rotorRadius_mm, e = p.eccentricity_mm, W = std::max(40.0f, p.rotorWidth_mm);
    const float pitch = W * 1.55f, z0 = -(n - 1) * pitch * 0.5f;
    const double Rin = R + 1.5, Rout = R + 1.5 + 0.22 * R;
    auto epi = [&](double t, double Rr) { return v3((float)(e * std::cos(3 * t) + Rr * std::cos(t)), (float)(e * std::sin(3 * t) + Rr * std::sin(t)), 0); };
    // housing ring (running surface and outer contour), its own mesh
    const Cached& housing = meshOf(key("rhousing", {R, e, W}), [&] {
        Data d;
        const int N = 120;
        const float hz = 0.5f * W;
        for (int k = 0; k < N; ++k) {
            const double t0 = 2 * kPi * k / N, t1 = 2 * kPi * (k + 1) / N;
            const V3 i0 = epi(t0, Rin), i1 = epi(t1, Rin), o0 = epi(t0, Rout), o1 = epi(t1, Rout);
            for (int f = 0; f < 2; ++f) {   // faces
                const float z = f ? hz : -hz;
                const V3 nn = v3(0, 0, f ? 1.0f : -1.0f);
                const uint32_t a = d.add(i0 + v3(0, 0, z), nn), b = d.add(i1 + v3(0, 0, z), nn), c = d.add(o1 + v3(0, 0, z), nn), q = d.add(o0 + v3(0, 0, z), nn);
                d.tri(a, b, c); d.tri(a, c, q);
            }
            for (int w = 0; w < 2; ++w) {   // inner / outer walls
                const V3 a0 = w ? o0 : i0, a1 = w ? o1 : i1;
                const V3 nn = norm(v3(a1.y - a0.y, -(a1.x - a0.x), 0) * (w ? 1.0f : -1.0f));
                const uint32_t a = d.add(a0 + v3(0, 0, -hz), nn), b = d.add(a1 + v3(0, 0, -hz), nn), c = d.add(a1 + v3(0, 0, hz), nn), q = d.add(a0 + v3(0, 0, hz), nn);
                d.tri(a, b, c); d.tri(a, c, q);
            }
        }
        return d; });
    const Cached& rotor = meshOf(key("rrotor", {R, W}), [&] {
        Poly rp;
        for (int k = 0; k < 3; ++k)
            for (int m = 0; m < 16; ++m) {
                const double u = m / 16.0, ang = 2 * kPi * k / 3 + u * 2 * kPi / 3;
                const double bulge = 1.0 - 0.40 * std::sin(u * kPi);
                rp.push_back({(float)(R * bulge * std::cos(ang)), (float)(R * bulge * std::sin(ang))});
            }
        Data d = mesh::extrude(rp, -0.45f * W, 0.45f * W);
        mesh::append(d, mesh::extrude(mesh::gearOutline(0.30f * R, 0.33f * R, 36), -0.46f * W, 0.46f * W), M4{});
        return d; });
    const Cached& shaft = meshOf(key("rshaft", {R, (float)n, pitch}), [&] { return mesh::extrude(mesh::circleOutline(0.12f * R, 28), z0 - pitch * 0.6f, -z0 + pitch * 0.6f); });
    sc.put(shaft, rotZ(-(float)(shaftDeg * kDeg)), kSteel);
    const Cached& lobe = meshOf(key("rlobe", {R, e, W}), [&] { return mesh::extrude(mesh::circleOutline(0.22f * R, 28, e, 0), -0.5f * W, 0.5f * W); });
    const Cached& statGear = meshOf(key("rstat", {R, W}), [&] { return mesh::extrude(mesh::gearOutline(0.19f * R, 0.215f * R, 24), -0.52f * W, -0.40f * W); });
    const Cached& plug = meshOf(key("plug", {R * 0.6f}), [&] { return plugMesh(R * 0.6f); });
    // crisp edges: the running surface traced in steel on both faces of each housing (the glass alone reads as a blur)
    const Cached& edge = meshOf(key("redge", {R, e, W}), [&] {
        std::vector<V3> path;
        for (int k = 0; k < 120; ++k) path.push_back(epi(2 * kPi * k / 120, Rin));
        Data d;
        for (int f = -1; f <= 1; f += 2) {
            std::vector<V3> q = path;
            for (V3& v : q) v.z = f * 0.5f * W;
            mesh::append(d, mesh::tube(q, std::vector<float>(q.size(), 0.012f * R), 8, true, false), M4{});
        }
        return d; });
    const float rRun = 0.10f * R;   // runner / port radius
    std::vector<V3> exEnds;
    for (int r = 0; r < n; ++r) {
        const float z = z0 + r * pitch;
        const double a = (shaftDeg - phases[r] + 90.0) * kDeg;   // +90: face 0 fires at the top (spark plugs)
        const float psi = (float)(a / 3.0);
        const float cx = (float)(e * std::cos(a)), cy = (float)(e * std::sin(a));
        sc.put(housing, translate(v3(0, 0, z)), glassMat(glassA * 1.1f));
        sc.put(edge, translate(v3(0, 0, z)), kSteel);
        sc.put(rotor, translate(v3(cx, cy, z)) * rotZ(psi), kSilver);
        sc.put(lobe, translate(v3(0, 0, z)) * rotZ((float)a), kGold);
        sc.put(statGear, translate(v3(0, 0, z)), kSteelDark);
        // intake: a side port in the housing's face, its runner out to the -x side and up to the plenum;
        // exhaust: a peripheral port at the bottom, its pipe down and out to the +x side
        const float yb = -(float)(Rin - e);
        const V3 in0 = v3(-0.45f * R, yb + 0.25f * R, z - 0.5f * W - 0.02f * R);
        sc.put(tubeOf("rrunner", spline({in0, in0 + v3(-0.15f * R, 0, -0.25f * R), v3(-(float)Rout - 0.35f * R, 0.15f * R, z - 0.5f * W - 0.25f * R),
                                         v3(-(float)Rout - 0.55f * R, 0.55f * R, z)}, 8), rRun), M4{}, kAlu);
        const V3 ex0 = v3(0.40f * R, yb + 0.05f * R, z);
        const V3 exEnd = v3((float)Rout + 0.35f * R, yb - 0.35f * R, z);
        sc.put(tubeOf("rexh", spline({ex0, ex0 + v3(0.05f * R, -0.35f * R, 0), v3(0.75f * R, yb - 0.40f * R, z), exEnd}, 8), rRun * 1.1f), M4{}, hot(live.egtGas));
        exEnds.push_back(exEnd);
        bool fire = false; double sinceMin = 1e9;
        for (int k = 0; k < 3; ++k) { const double s = wrapPos(shaftDeg - phases[r] + 360.0 * k + live.advance, 1080.0); sinceMin = std::min(sinceMin, s); if (s < 14) fire = true; }
        // leading / trailing plugs: screwed through the housing wall near the top, tips at the running surface
        for (int s = -1; s <= 1; s += 2) {
            const float px = s * R * 0.16f, py = (float)(Rin - e);
            sc.put(plug, translate(v3(px, py - 0.02f * R, z)), kCeramic);
            if (fire) sc.glow(v3(px, py - 0.06f * R, z), 0.12f * R, 1.0f, 0.92f, 0.6f, 1.0f);
        }
        // the burning chamber at the top: the app's orange
        if (sinceMin < 160.0) {
            const float k = 1.0f - (float)(sinceMin / 160.0);
            for (int d = 0; d < 12; ++d) {
                const float x = (rnd(r, d) - 0.5f) * 1.2f * R, y = (float)Rin * 0.55f + rnd(r + 9, d) * 0.25f * R, zz = (rnd(r + 5, d) - 0.5f) * 0.8f * W;
                sc.glow(v3(x, y, z + zz), 0.06f * R, kAccent[0], 0.55f, 0.18f, 0.6f * k);
            }
        }
        sc.label(v3(0, (float)Rout + 0.45f * R, z), "Rotor " + std::to_string(r + 1));
    }
    // intake plenum along the -x side, its throttle at the front
    const float zA = z0 - 0.5f * W, zB = -z0 + 0.5f * W;
    const V3 pc = v3(-(float)Rout - 0.55f * R, 0.75f * R, 0.5f * (zA + zB));
    const float pl = zB - zA + 0.3f * R;
    const Cached& plen = meshOf(key("rplenum", {R, pl}), [&] { return mesh::extrude(mesh::roundedRect(0.22f * R, 0.26f * R, 0.10f * R), -0.5f * pl, 0.5f * pl); });
    sc.put(plen, translate(pc), kAlu);
    sc.put(tubeOf("rthrottle", {pc + v3(0, 0, -0.5f * pl), pc + v3(0, 0, -0.5f * pl - 0.35f * R)}, 0.17f * R), M4{}, kAlu);
    throttleIn = pc + v3(0, 0, -0.5f * pl - 0.35f * R); throttleDir = v3(0, 0, -1);
    eb.B = 0.8f * R; eb.r = (float)e; eb.zFront = zA; eb.zRear = zB; eb.plenum = pc; eb.plenumLen = pl;
    eb.crankFront = v3(0, 0, z0 - pitch * 0.6f);
    // exhaust: the rotors' pipes join and run back to the tail (rotaries pop: the flame shows there)
    const V3 col = v3((float)Rout + 0.40f * R, -(float)(Rin - e) - 0.55f * R, zB + 0.35f * R);
    for (const V3& q : exEnds) sc.put(tubeOf("rexh2", spline({q, q + v3(0.05f * R, -0.1f * R, 0.25f * R), col}, 8), rRun * 1.1f), M4{}, hot(live.egtGas));
    turbineIn = col + v3(0, -0.45f * R, 0);
    if (turbo) sc.put(tubeOf("rexhdrop", {col, turbineIn}, rRun * 1.1f), M4{}, hot(live.egtGas));
    if (!turbo)
        tailWithFlame(sc, {col, col + v3(0, -0.10f * R, 0.5f * R), col + v3(0, -0.20f * R, 1.6f * R), col + v3(0, -0.22f * R, 2.6f * R)}, 0.16f * R, live,
                      live.egtGas, 11);
    // front pulley and rear flywheel on the e-shaft
    const float shR = -(float)(shaftDeg * kDeg);
    const Cached& pul = meshOf(key("rpulley", {R}), [&] { return mesh::extrude(mesh::gearOutline(0.36f * R, 0.38f * R, 40), -0.06f * R, 0.06f * R); });
    sc.put(pul, translate(v3(0, 0, z0 - pitch * 0.6f)) * rotZ(shR), kSteel);
    const float fwR = 1.05f * R;
    const Cached& fly = meshOf(key("rfly", {fwR}), [&] {
        Data d = mesh::extrude(mesh::gearOutline(fwR - 0.035f * R, fwR, 96), -0.05f * R, 0.05f * R);
        mesh::append(d, mesh::extrude(mesh::circleOutline(0.24f * R, 24), -0.14f * R, 0.14f * R), M4{});
        return d; });
    sc.put(fly, translate(v3(0, 0, -z0 + pitch * 0.6f)) * rotZ(shR), kSteelDark);
}

} // namespace es::ui::model
