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

void tailWithFlame(Scene& sc, const std::vector<V3>& ctrl, float rt, const EngineViewLive& live, float egt, int salt) {
    const std::vector<V3> path = spline(ctrl, 8);
    sc.put(tubeOf("tail", path, rt), M4{}, hot(egt, 0.35f));
    // silencer can on the way
    const V3 a = path[path.size() / 2], b = path[path.size() / 2 + 2 < path.size() ? path.size() / 2 + 2 : path.size() - 1];
    const Cached& can = meshOf(key("muffcan", {rt}), [&] { return mesh::lathe({{0, -2.6f * rt}, {2.1f * rt, -2.6f * rt}, {2.3f * rt, -2.2f * rt}, {2.3f * rt, 2.2f * rt},
                                                                              {2.1f * rt, 2.6f * rt}, {0, 2.6f * rt}}, 28); });
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
        turbineIn = collectors[0];
        // the other bank (V / boxer) crosses over to the turbine
        for (size_t k = 1; k < collectors.size(); ++k) {
            const V3 a = collectors[k], t = collectors[0];
            sc.put(tubeOf("crossover", spline({a, a + v3(0, -0.4f * B, 0.4f * B), lerp(a, t, 0.5f) + v3(0, -0.6f * B, 0.6f * B), t + v3(0, -0.3f * B, 0.4f * B), t}, 8), rt),
                   M4{}, hot(live.egtGas, 0.6f));
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
        for (int i = 0; i < n; ++i) {
            const float a = 2.0f * kPiF * i / std::max(1, n);
            const V3 out = head + v3(0.09f * B * std::cos(a), 0.09f * B * std::sin(a), 0);
            const V3 t = eb.injTop[i];
            const V3 up = v3(out.x * 0.5f + t.x * 0.5f, std::max(out.y, t.y) + 0.30f * B + 0.03f * B * i, out.z + 0.25f * B);
            sc.put(tubeOf("hpline", spline({out, out + v3(0, 0.05f * B, 0.12f * B), up, v3(t.x, t.y + 0.30f * B, t.z), t + v3(0, 0.04f * B, 0)}, 10), 0.024f * B, 8),
                   M4{}, kSteel);
        }
    }
    // glow plugs: a bus wire over their tops, glowing the app's orange while they heat
    if (eb.glowTop.size() > 1) {
        std::vector<V3> w;
        for (const V3& q : eb.glowTop) { w.push_back(q + v3(0, 0.02f * B, -0.08f * B)); w.push_back(q + v3(0, 0.03f * B, 0)); w.push_back(q + v3(0, 0.02f * B, 0.08f * B)); }
        Material wm = kGold;
        if (live.glowPlugOn) wm = emissive(kGold, kAccent[0] * 0.35f, kAccent[1] * 0.20f, kAccent[2] * 0.05f);
        sc.put(tubeOf("glowbus", spline(w, 3), 0.014f * B, 6), M4{}, wm);
    }
    // EGR: from the exhaust manifold round the front of the head into the intake, the valve body at the intake end;
    // exhaust-coloured sparks run along it while it is open
    if (p.egrValveDia_mm > 0 && !eb.ex.empty() && eb.plenumLen > 0) {
        const PortOut& e0 = eb.ex.front();
        const float rr = clampv(p.egrValveDia_mm * 0.5f, 0.06f * B, 0.20f * B);
        const V3 a = e0.p + e0.d * (0.75f * B) + v3(0, -0.15f * B, 0);
        const V3 vlv = eb.plenum + v3(0, 0, -0.5f * eb.plenumLen + 0.25f * B) + v3(0, -0.40f * B, 0);
        const std::vector<V3> path = spline({a, a + v3(0, 0.1f * B, -0.55f * B), v3(0.5f * (a.x + vlv.x), std::max(a.y, vlv.y) + 0.85f * B, eb.zFront - 0.55f * B),
                                             vlv + v3(0, 0.0f, -0.45f * B), vlv}, 10);
        sc.put(tubeOf("egr", path, rr), M4{}, kSteel);
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
    // spin: the real shaft speed slowed down to something the eye can follow (and capped well below strobing)
    static float spin = 0, lastT = -1;
    const float dt = lastT < 0 ? 0.0f : clampv(live.time - lastT, 0.0f, 0.1f);
    lastT = live.time;
    spin = std::fmod(spin + dt * 2 * kPiF * std::min(live.shaftRpm / 60.0f * 0.02f, 5.0f), 2 * kPiF);
    const float rp = 0.20f * B;   // boost pipe radius
    // intercooler in front of the engine, if fitted: compressor -> core -> throttle (a Roots / screw blower sits on the
    // plenum and blows straight down into it: no front core)
    const bool frontIc = p.intercooler && (p.induction == Induction::Turbo || p.scType == Supercharger::Centrifugal);
    V3 icC = v3(0, 0, 0);
    float icHx = 0;
    if (frontIc) {
        icC = v3(0, -0.2f * B, eb.zFront - 1.6f * B);
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
    auto pipe = [&](const std::vector<V3>& ctrl, const char* nm) { sc.put(tubeOf(nm, spline(ctrl, 10), rp), M4{}, kAlu); };
    auto pipeTo = [&](V3 from, V3 to, V3 fromDir, V3 toDir, const char* nm) {
        const float d = len(to - from);
        pipe({from, from + fromDir * (0.30f * d), to - toDir * (0.30f * d), to}, nm);
    };
    // compressor outlet -> (core ->) throttle. The core's ends take the pipes on their back faces, the compressor's side
    // first; pipes run along the engine's side at the core's height
    auto boostTo = [&](V3 from, V3 dir) {
        if (!frontIc) { pipeTo(from, throttleIn, dir, throttleDir * -1.0f, "boostA"); return; }
        const float sIn = from.x < 0 ? -1.0f : 1.0f;
        const V3 inP = icC + v3(sIn * icHx, 0, 0.20f * B), outP = icC + v3(-sIn * icHx, 0, 0.20f * B);
        const V3 side = v3(from.x, inP.y + 0.2f * B, 0.5f * (from.z + inP.z));
        pipe({from, from + dir * (0.6f * B), side, inP + v3(0, 0, 0.9f * B), inP}, "boostA");
        const V3 tin = throttleIn + throttleDir * (0.35f * B);
        pipe({outP, outP + v3(0, 0.1f * B, 0.5f * B), v3(lerp(outP, tin, 0.6f).x, lerp(outP, tin, 0.6f).y, std::max(outP.z + 0.6f * B, tin.z)), tin, throttleIn}, "boostB");
    };
    if (p.induction == Induction::Turbo) {
        // turbine beside the rear of the engine, its shaft across; the compressor on the outer end
        const V3 out = norm(v3(turbineIn.x == 0 ? 1.0f : turbineIn.x, 0, 0));
        const V3 T = turbineIn + v3(0, 0.25f * B, 0.75f * B);
        const M4 tf = alongY(T, out);   // +y along the turbo shaft, outward
        const Cached& tv = meshOf(key("turbvol", {B}), [&] { return voluteMesh(0.42f * B, 0.55f * B, 0.10f * B, 0.21f * B); });
        sc.put(tv, tf, hot(live.egtGas, 0.9f));
        const Cached& chra = meshOf(key("chra", {B}), [&] { return mesh::lathe({{0, 0}, {0.20f * B, 0}, {0.24f * B, 0.10f * B}, {0.24f * B, 0.30f * B}, {0.20f * B, 0.40f * B}, {0, 0.40f * B}}, 28); });
        sc.put(chra, tf * translate(v3(0, 0.12f * B, 0)), kSteelDark);
        const Cached& cv = meshOf(key("compvol", {B}), [&] { return voluteMesh(0.48f * B, 0.62f * B, 0.11f * B, 0.23f * B); });
        const M4 cf = tf * translate(v3(0, 0.62f * B, 0));
        sc.put(cv, cf, glassMat(0.30f, 0.78f, 0.79f, 0.82f));
        const Cached& wh = meshOf(key("compwheel", {B}), [&] { return wheelMesh(0.36f * B, 0.30f * B, 10); });
        sc.put(wh, cf * rotY(spin) * translate(v3(0, -0.12f * B, 0)), kSilver);
        // compressor inlet (open, outward) and the turbine's downpipe to the tail
        sc.put(tubeOf("compinlet", {xform(cf, v3(0, 0.10f * B, 0)), xform(cf, v3(0, 0.55f * B, 0))}, 0.24f * B), M4{}, kAlu);
        tailWithFlame(sc, {T + v3(0, -0.1f * B, 0.35f * B), T + v3(0, -0.4f * B, 1.0f * B), T + v3(0, -0.55f * B, 2.2f * B), T + v3(0, -0.6f * B, 3.4f * B)},
                      clampv(p.pipeDia_mm * 0.5f, 0.15f * B, 0.4f * B), live, live.egtGas, 7);
        const V3 compOut = xform(cf, v3(0.62f * B, 0, 0.30f * B));
        boostTo(compOut, norm(v3(0, 0.6f, -1)));
        if (p.vgt) {   // variable geometry: the vane actuator (a vacuum can) on the turbine housing
            const Cached& can = meshOf(key("vgtcan", {B}), [&] { return mesh::lathe({{0, 0}, {0.16f * B, 0}, {0.17f * B, 0.04f * B}, {0.17f * B, 0.16f * B}, {0, 0.18f * B}}, 24); });
            sc.put(can, tf * translate(v3(0.0f, -0.10f * B, 0.0f)) * translate(v3(0.62f * B, 0, -0.25f * B)) * rotX(kPiF * 0.5f), kSteelDark);
        }
        sc.label(T + v3(0, 0.9f * B, 0), p.vgt ? "VGT turbo" : "turbo");
        return;
    }
    // supercharger: crank-driven by a belt
    const float ratio = std::max(0.5f, p.scRatio);
    const float ang = -crankDeg * kDegF * (p.scType == Supercharger::Centrifugal ? std::min(ratio, 3.0f) : ratio);
    const V3 cp = eb.crankFront;
    if (p.scType == Supercharger::Centrifugal) {
        const V3 S = cp + v3(1.25f * B, 1.45f * B, 0.55f * B);
        const M4 sf = alongY(S, v3(0, 0, -1));   // shaft forward
        const Cached& cv = meshOf(key("compvol", {B}), [&] { return voluteMesh(0.48f * B, 0.62f * B, 0.11f * B, 0.23f * B); });
        sc.put(cv, sf, glassMat(0.30f, 0.78f, 0.79f, 0.82f));
        const Cached& wh = meshOf(key("compwheel", {B}), [&] { return wheelMesh(0.36f * B, 0.30f * B, 10); });
        sc.put(wh, sf * rotY(spin) * translate(v3(0, -0.12f * B, 0)), kSilver);
        const Cached& body = meshOf(key("scgearcase", {B}), [&] { return mesh::lathe({{0, 0}, {0.30f * B, 0}, {0.30f * B, 0.45f * B}, {0, 0.45f * B}}, 28); });
        sc.put(body, sf * translate(v3(0, -0.55f * B, 0)), kSteelDark);
        const V3 pul = v3(S.x, S.y, cp.z);
        const Cached& pulley = meshOf(key("scpulley", {B}), [&] { return mesh::extrude(mesh::gearOutline(0.26f * B, 0.28f * B, 36), -0.08f * B, 0.08f * B); });
        sc.put(pulley, translate(pul) * rotZ(ang * 0.3f), kSteel);
        const Cached& crankPul = meshOf(key("crankpulley", {B}), [&] { return mesh::extrude(mesh::gearOutline(0.40f * B, 0.42f * B, 40), -0.08f * B, 0.08f * B); });
        sc.put(crankPul, translate(cp) * rotZ(-crankDeg * kDegF), kSteel);
        sc.put(tubeOf("scbelt", beltAround(cp, 0.43f * B, pul, 0.29f * B), 0.025f * B, 8), M4{}, kRubber);
        boostTo(xform(sf, v3(0.62f * B, 0, 0.30f * B)), v3(0, 0.3f, 1));
        sc.label(S + v3(0, 0.8f * B, 0), "supercharger");
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
    sc.put(tubeOf("scbelt", beltAround(cp, 0.43f * B, pul, 0.29f * B), 0.025f * B, 8), M4{}, kRubber);
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
    turbineIn = col;
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
