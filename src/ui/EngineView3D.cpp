// Engine view in 2.5D: every part is a flat convex shape (vector-drawing style) extruded into a thin slab and
// placed in 3D - each cylinder's cutaway in its own plane along the crankshaft, the V banks tilted, the boxer
// opposed, rotors stacked. A small software projection onto ImGui's draw list, painter-sorted, back faces culled.
#include "ui/EngineView3D.h"
#include "ui/EngineModel.h"
#include "ui/Gpu3D.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace es::ui {
namespace {

struct V3 { float x = 0, y = 0, z = 0; };
inline V3 v3(float x, float y, float z) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
inline V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline V3 operator*(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

ImU32 rgba(float r, float g, float b, float a = 1) { return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, a)); }
ImU32 shade(ImU32 c, float k) {
    ImVec4 v = ImGui::ColorConvertU32ToFloat4(c);
    return rgba(v.x * k, v.y * k, v.z * k, v.w);
}

// ---- theme palette (the app's dark UI)
const ImU32 kBg = rgba(0.06f, 0.06f, 0.07f);
const ImU32 kBlock = rgba(0.30f, 0.31f, 0.34f), kBlockRim = rgba(0.46f, 0.47f, 0.52f);
const ImU32 kHead = rgba(0.35f, 0.36f, 0.40f);
const ImU32 kSilver = rgba(0.76f, 0.77f, 0.81f), kSilverRim = rgba(0.52f, 0.53f, 0.57f);
const ImU32 kSteel = rgba(0.58f, 0.59f, 0.63f), kSteelDark = rgba(0.40f, 0.41f, 0.45f);
const ImU32 kGold = rgba(0.85f, 0.70f, 0.30f);
const ImU32 kDark = rgba(0.16f, 0.16f, 0.19f);
const ImU32 kLabel = rgba(0.85f, 0.85f, 0.90f);

using Poly = std::vector<ImVec2>;

float cross2(ImVec2 o, ImVec2 a, ImVec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

// convex hull, counter-clockwise (Andrew's monotone chain)
Poly hull(Poly p) {
    std::sort(p.begin(), p.end(), [](const ImVec2& a, const ImVec2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    const int n = (int)p.size();
    if (n < 3) return p;
    Poly h(2 * n);
    int k = 0;
    for (int i = 0; i < n; ++i) { while (k >= 2 && cross2(h[k - 2], h[k - 1], p[i]) <= 0) --k; h[k++] = p[i]; }
    for (int i = n - 2, t = k + 1; i >= 0; --i) { while (k >= t && cross2(h[k - 2], h[k - 1], p[i]) <= 0) --k; h[k++] = p[i]; }
    h.resize(k - 1);
    return h;
}
float area2(const Poly& p) {
    float a = 0;
    for (size_t i = 0; i < p.size(); ++i) { const ImVec2 &u = p[i], &w = p[(i + 1) % p.size()]; a += u.x * w.y - w.x * u.y; }
    return a;
}
Poly circle(ImVec2 c, float r, int n = 22) {
    Poly p(n);
    for (int i = 0; i < n; ++i) { const float a = 2.0f * (float)kPi * i / n; p[i] = ImVec2(c.x + r * std::cos(a), c.y + r * std::sin(a)); }
    return p;
}
Poly join(Poly a, const Poly& b) { a.insert(a.end(), b.begin(), b.end()); return a; }
Poly capsule(ImVec2 a, ImVec2 b, float ra, float rb) { return hull(join(circle(a, ra, 16), circle(b, rb, 16))); }
Poly rect(float x0, float y0, float x1, float y1) { return {ImVec2(x0, y0), ImVec2(x1, y0), ImVec2(x1, y1), ImVec2(x0, y1)}; }

// a convex polygon in a plane, extruded to a slab of thickness t centred `off` along the plane normal
struct Slab {
    int first = 0, count = 0;   // into Scene::pts (plane coordinates)
    V3 O, U, V, N;
    float off = 0, t = 0;
    ImU32 fill = 0, line = 0;
    uint64_t sides = ~0ull;     // bit k: the side face of edge k exists (0: a flat decal)
    float bias = 0;             // depth bias, mm towards the viewer (decals on top of their part)
    int group = 0;              // plane group (one cylinder / rotor / crank segment): drawn far to near ...
    float gz = 0;               // ... by the group position along the crank axis, then layer by layer inside it
    bool noAA = false;          // pieces of a larger shape: no anti-aliased fringe (no seams between them)
};
struct Label { V3 at; std::string text; V3 N; };

struct Scene {
    std::vector<ImVec2> pts;
    std::vector<Slab> slabs;
    std::vector<Label> labels;
    V3 O, U = v3(1, 0, 0), V = v3(0, 1, 0);
    int grp = 0; float grpZ = 0; bool noAA = false;
    void clear() { pts.clear(); slabs.clear(); labels.clear(); grp = 0; grpZ = 0; noAA = false; }
    void group(float z) { ++grp; grpZ = z; }
    void frame(V3 o, V3 u, V3 v) { O = o; U = u; V = v; }
    void add(Poly poly, float off, float t, ImU32 fill, ImU32 line = 0, uint64_t sides = ~0ull, float bias = 0) {
        if (poly.size() < 3) return;
        if (area2(poly) < 0) {   // make it counter-clockwise (outward edge normals), keep the side mask aligned
            std::reverse(poly.begin(), poly.end());
            const int n = (int)poly.size();
            uint64_t m = 0;
            for (int k = 0; k < n && k < 64; ++k)   // edge k (i -> i+1) of the reversed polygon was edge n-2-k
                if (sides & (1ull << ((n - 2 - k + n) % n))) m |= 1ull << k;
            sides = m;
        }
        Slab s;
        s.first = (int)pts.size(); s.count = (int)std::min<size_t>(poly.size(), 64);
        pts.insert(pts.end(), poly.begin(), poly.begin() + s.count);
        s.O = O; s.U = U; s.V = V; s.N = cross(U, V);
        s.off = off; s.t = t; s.fill = fill; s.line = line; s.sides = t > 0 ? sides : 0; s.bias = bias;
        s.group = grp; s.gz = grpZ; s.noAA = noAA;
        slabs.push_back(s);
    }
    void decal(Poly poly, float off, ImU32 fill, float bias = 0.4f) { add(std::move(poly), off, 0, fill, 0, 0, bias); }
    void label(V3 at, const char* text) { labels.push_back({at, text, cross(U, V)}); }
};

ImU32 gasRaw(double phi, double adv, double cycle);
// gas colour of a cylinder / chamber by stroke (phi: cycle angle, 0 = firing TDC), opaque over a dark backing
ImU32 gasColour(double phi, double adv, double cycle) {
    const ImVec4 c = ImGui::ColorConvertU32ToFloat4(gasRaw(phi, adv, cycle));
    const float k = c.w, b = 0.12f * (1 - k);
    return rgba(c.x * k + b, c.y * k + b, c.z * k + b * 1.1f, 1.0f);
}
ImU32 gasRaw(double phi, double adv, double cycle) {
    const double since = wrapPos(phi + adv, cycle);
    const double burn = cycle > 800 ? 130.0 : 90.0;
    if (since < burn) { const float t = (float)(1 - since / burn); return rgba(1.0f, 0.45f + 0.4f * t, 0.12f, 0.35f + 0.6f * t); }
    const double q = cycle / 4;
    if (phi < q) return rgba(0.85f, 0.45f, 0.15f, 0.30f);         // late expansion
    if (phi < 2 * q) return rgba(0.45f, 0.36f, 0.30f, 0.60f);     // exhaust
    if (phi < 3 * q) return rgba(0.25f, 0.45f, 0.85f, 0.55f);     // intake
    return rgba(0.30f, 0.55f, 0.95f, 0.75f);                      // compression
}

double liftAt(double phi, double center, double dur, double lift) {
    const double d = wrapSym(phi - center, 720.0), h = dur * 0.5;
    if (std::fabs(d) >= h) return 0;
    return lift * std::pow(0.5 * (1.0 + std::cos(kPi * d / h)), 0.75);
}

// ------------------------------------------------------------------ piston engines
void buildPiston(Scene& sc, const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live, float depth) {
    const int n = clampv(p.cylinders, 1, kMaxCyl);
    const bool boxer = p.layout == Layout::Boxer, vee = p.layout == Layout::V;
    const float r = p.stroke_mm * 0.5f, l = std::max(p.rod_mm, r * 1.2f), B = p.bore_mm;
    const float wall = B * 0.12f, pistonH = B * 0.62f, pinOff = B * 0.28f;
    const float deck = r + l + r * 0.35f + B * 0.2f;
    // columns along the crank: cylinders of different banks share a throw position
    int bankCount[2] = {0, 0}, column[kMaxCyl];
    for (int i = 0; i < n; ++i) { const int b = p.layout == Layout::Inline ? 0 : clampv(p.bank[i], 0, 1); column[i] = bankCount[b]++; }
    const int cols = std::max(1, std::max(bankCount[0], bankCount[1]));
    const float pitch = B * 1.35f, bankOff = (vee || boxer) ? B * 0.55f : 0.0f;
    const float zOf0 = -(cols - 1) * pitch * 0.5f;
    const float d = std::max(0.2f, depth);
    // layers along the plane normal (negative = towards the front view)
    const float tBlock = 0.50f * B * d, tGas = 0.30f * B * d, tCrank = 0.22f * B * d, tRod = 0.14f * B * d, tPiston = 0.34f * B * d;
    const float oBlock = 0, oGas = 0.04f * B * d, oCrank = 0.22f * B * d, oRod = -0.02f * B * d, oPiston = -0.10f * B * d;
    const float oValve = -0.28f * B * d, oCam = -0.33f * B * d;
    const float vtec = p.vtec ? clampv(live.vtecBlend, 0.0f, 1.0f) : 0.0f;
    const double inC = 360.0 + p.intakeCenterline - p.intakeCamAdvance - (p.vtc ? live.vtcAdvance : 0.0f);
    const double exC = 360.0 - p.exhaustCenterline - p.exhaustCamAdvance;

    // crankshaft spine: main journals between the throws
    {
        const float rj = std::max(4.0f, r * 0.38f);
        const float z0 = zOf0 - pitch * 0.7f - bankOff * 0.5f, z1 = -zOf0 + pitch * 0.7f + bankOff * 0.5f;
        const int segs = std::max(2, (int)std::ceil((z1 - z0) / (pitch * 0.25f)));
        sc.noAA = true;
        for (int k = 0; k < segs; ++k) {   // short round pieces, each its own slice, so they sort against the throws
            const float a = z0 + (z1 - z0) * k / segs, b = z0 + (z1 - z0) * (k + 1) / segs;
            sc.frame(v3(0, 0, 0.5f * (a + b)), v3(1, 0, 0), v3(0, 1, 0));
            sc.group(0.5f * (a + b));
            sc.add(circle(ImVec2(0, 0), rj, 28), 0, b - a, kSteel, 0);
        }
        sc.noAA = false;
    }
    for (int i = 0; i < n; ++i) {
        const int b = p.layout == Layout::Inline ? 0 : clampv(p.bank[i], 0, 1);
        float axisDeg = 0;
        if (vee) axisDeg = (b == 0 ? -1.0f : 1.0f) * p.bankAngle * 0.5f;
        if (boxer) axisDeg = b == 0 ? -90.0f : 90.0f;
        const float z = zOf0 + column[i] * pitch + (b == 0 ? -0.5f : 0.5f) * bankOff;
        sc.frame(v3(0, 0, z), v3(1, 0, 0), v3(0, 1, 0));
        sc.group(z);
        const float ax = axisDeg * (float)kDeg;
        const ImVec2 up(std::sin(ax), std::cos(ax)), side(up.y, -up.x);   // plane coords: y up
        auto L = [&](float a, float s) { return ImVec2(side.x * a + up.x * s, side.y * a + up.y * s); };
        auto LP = [&](const Poly& loc) { Poly o(loc.size()); for (size_t k = 0; k < loc.size(); ++k) o[k] = L(loc[k].x, loc[k].y); return o; };

        const double phi = wrapPos(crankDeg - phases[i], 720.0);
        const double th = wrapPos(phi, 360.0) * kDeg;
        const float sn = (float)std::sin(th), cs = (float)std::cos(th);
        const float dist = r * cs + std::sqrt(std::max(1.0f, l * l - r * r * sn * sn));
        const float crown = dist + pinOff;
        const ImVec2 cpinL(r * sn, r * cs);   // crank pin, cylinder-local (across, along)

        // ---- crank throw: counterweight opposite the pin, web arm, journal and pin
        {
            const float pinA = std::atan2(cpinL.x, cpinL.y);   // angle of the pin from the cylinder axis
            Poly cw; cw.push_back(L(0, 0));
            for (int k = 0; k <= 12; ++k) {
                const float a = pinA + (float)kPi + (-0.45f + 0.9f * k / 12.0f) * (float)kPi;
                cw.push_back(L(r * 1.3f * std::sin(a), r * 1.3f * std::cos(a)));
            }
            sc.add(hull(cw), oCrank, tCrank, kSteel, kSteelDark);
            sc.add(LP(capsule(ImVec2(0, 0), cpinL, r * 0.55f, r * 0.42f)), oCrank, tCrank, kSteel, kSteelDark);
            sc.decal(LP(circle(ImVec2(0, 0), r * 0.30f, 16)), oCrank - tCrank * 0.5f, kSteelDark);
        }
        // ---- block: liner walls (with fin ticks), head with ports
        const float linerBottom = l - r - 0.1f * B;
        sc.add(LP(rect(-B / 2 - wall, linerBottom, -B / 2, deck)), oBlock, tBlock, kBlock, kBlockRim);
        sc.add(LP(rect(B / 2, linerBottom, B / 2 + wall, deck)), oBlock, tBlock, kBlock, kBlockRim);
        for (int f = 0; f < 4; ++f) {
            const float s = deck - B * (0.12f + 0.16f * f);
            if (s < linerBottom + B * 0.05f) break;
            sc.decal(LP(rect(-B / 2 - wall, s, -B / 2 - wall * 0.45f, s + B * 0.025f)), oBlock - tBlock * 0.5f, kDark);
            sc.decal(LP(rect(B / 2 + wall * 0.45f, s, B / 2 + wall, s + B * 0.025f)), oBlock - tBlock * 0.5f, kDark);
        }
        const float headW = B / 2 + wall + 0.12f * B, headTop = deck + 0.48f * B;
        sc.add(LP(rect(-headW, deck, headW, headTop)), oBlock, tBlock, kHead, kBlockRim);
        // ports (decals on the head face): intake on the left, exhaust on the right
        auto port = [&](float sgn, ImU32 c) {
            Poly q = {ImVec2(sgn * 0.14f * B, deck + 0.02f * B), ImVec2(sgn * 0.36f * B, deck + 0.02f * B),
                      ImVec2(sgn * headW, deck + 0.30f * B), ImVec2(sgn * headW, deck + 0.20f * B)};
            sc.decal(LP(hull(q)), oBlock - tBlock * 0.5f, c);
        };
        // ---- valves, followers, cams (intake left, exhaust right); valve axes lean out ~12 deg
        struct ValveDraw { float sgn; double lift, maxLift, dur, center; ImU32 lit, dim; };
        const double liLo = liftAt(phi, inC, p.intakeDuration, p.intakeLift_mm);
        const double leLo = liftAt(phi, exC, p.exhaustDuration, p.exhaustLift_mm);
        double li = liLo, le = leLo;
        if (vtec > 0) {
            const double liHi = liftAt(phi, inC, p.vtecInDuration, p.vtecInLift_mm), leHi = liftAt(phi, exC, p.vtecExDuration, p.vtecExLift_mm);
            li += vtec * (std::max(liHi, liLo) - liLo); le += vtec * (std::max(leHi, leLo) - leLo);
        }
        const ValveDraw vv[2] = {
            {-1, li, p.intakeLift_mm + vtec * (p.vtecInLift_mm - p.intakeLift_mm), p.intakeDuration + vtec * (p.vtecInDuration - p.intakeDuration), inC,
             rgba(0.40f, 0.70f, 1.0f), rgba(0.24f, 0.42f, 0.68f)},
            {1, le, p.exhaustLift_mm + vtec * (p.vtecExLift_mm - p.exhaustLift_mm), p.exhaustDuration + vtec * (p.vtecExDuration - p.exhaustDuration), exC,
             rgba(1.0f, 0.56f, 0.30f), rgba(0.70f, 0.38f, 0.20f)}};
        port(-1, rgba(0.17f, 0.24f, 0.36f));
        port(1, rgba(0.36f, 0.24f, 0.17f));
        // gas column (behind the piston and the valves)
        sc.add(LP(rect(-B / 2, crown, B / 2, deck)), oGas, tGas, gasColour(phi, live.advance, 720.0), 0, 0);
        for (const ValveDraw& v : vv) {
            const float tilt = v.sgn * 12.0f * (float)kDeg;
            const ImVec2 ax(std::sin(tilt), std::cos(tilt));   // along the valve axis, towards the cam (local)
            const ImVec2 seat(v.sgn * 0.24f * B, deck);
            const float lift = (float)v.lift;
            const ImVec2 face(seat.x - ax.x * lift, seat.y - ax.y * lift);
            const ImU32 c = v.lift > 0.3 ? v.lit : v.dim;
            const float discW = 0.17f * B, stemW = 0.025f * B, stemL = 0.62f * B;
            const ImVec2 sd(ax.y, -ax.x);   // across the valve
            auto Q = [&](ImVec2 o, float w, float h0, float h1) {   // box along the valve axis
                return Poly{ImVec2(o.x + sd.x * -w + ax.x * h0, o.y + sd.y * -w + ax.y * h0), ImVec2(o.x + sd.x * w + ax.x * h0, o.y + sd.y * w + ax.y * h0),
                            ImVec2(o.x + sd.x * w + ax.x * h1, o.y + sd.y * w + ax.y * h1), ImVec2(o.x + sd.x * -w + ax.x * h1, o.y + sd.y * -w + ax.y * h1)};
            };
            sc.add(LP(Q(face, discW, -0.025f * B, 0.03f * B)), oValve, 0.06f * B * d, c);
            sc.add(LP(Q(face, stemW, 0.0f, stemL)), oValve, 0.05f * B * d, c);
            // follower (bucket) riding on the stem end
            const ImVec2 top(face.x + ax.x * stemL, face.y + ax.y * stemL);
            sc.add(LP(Q(top, 0.075f * B, 0.0f, 0.07f * B)), oValve, 0.10f * B * d, kSilver, kSilverRim);
            // cam: base circle + nose; the nose meets the follower at full lift, the cam turns at half crank speed
            const float Rb = 0.12f * B, rn = Rb * (0.38f + 0.0012f * (float)v.dur);
            const ImVec2 cc(seat.x + ax.x * (stemL + 0.07f * B + Rb), seat.y + ax.y * (stemL + 0.07f * B + Rb));
            const float rot = (float)(wrapSym(phi - v.center, 720.0) * 0.5 * kDeg);
            const float a0 = std::atan2(-ax.x, -ax.y) - rot;   // nose direction (local, angle from the axis)
            const float nd = Rb + (float)v.maxLift - rn;
            const ImVec2 nose(cc.x + nd * std::sin(a0), cc.y + nd * std::cos(a0));
            sc.add(LP(hull(join(circle(cc, Rb, 22), circle(nose, rn, 14)))), oCam, 0.13f * B * d, v.lift > 0.3 ? v.lit : v.dim, shade(v.lit, 0.55f));
            sc.decal(LP(circle(cc, Rb * 0.32f, 12)), oCam - 0.065f * B * d, kDark);
        }
        // spark plug, and the flash while the charge lights
        sc.add(LP(rect(-0.035f * B, deck - 0.02f * B, 0.035f * B, headTop + 0.10f * B)), oValve, 0.05f * B * d, kSilver);
        const double sinceSpark = wrapPos(phi + live.advance, 720.0);
        if (sinceSpark < 14) sc.decal(LP(circle(ImVec2(0, deck - 0.06f * B), 0.09f * B, 14)), oValve - 0.05f * B * d, rgba(1, 0.95f, 0.55f, 0.95f), 1.0f);
        // ---- piston (crown + narrower skirt), ring grooves, pin
        {
            Poly pist = {ImVec2(-B / 2 + 0.5f, crown), ImVec2(B / 2 - 0.5f, crown), ImVec2(B / 2 - 0.5f, crown - 0.20f * B), ImVec2(-B / 2 + 0.5f, crown - 0.20f * B),
                         ImVec2(-0.41f * B, crown - pistonH), ImVec2(0.41f * B, crown - pistonH)};
            sc.add(LP(hull(pist)), oPiston, tPiston, kSilver, kSilverRim);
            const float fo = oPiston - tPiston * 0.5f;
            for (int g = 0; g < 3; ++g) {
                const float s = crown - 0.05f * B - g * 0.045f * B;
                sc.decal(LP(rect(-B / 2 + 0.5f, s - 0.012f * B, -B / 2 + 0.11f * B, s)), fo, kSteelDark);
                sc.decal(LP(rect(B / 2 - 0.11f * B, s - 0.012f * B, B / 2 - 0.5f, s)), fo, kSteelDark);
            }
            sc.decal(LP(circle(ImVec2(0, dist), 0.075f * B, 14)), fo, kSteelDark, 0.6f);
        }
        // ---- rod (small end at the pin, big end on the crank pin)
        sc.add(LP(capsule(ImVec2(0, dist), cpinL, 0.11f * B, 0.17f * B)), oRod, tRod, kSteel, kSteelDark);
        sc.decal(LP(circle(cpinL, 0.085f * B, 14)), oRod - tRod * 0.5f, kGold, 0.6f);
        sc.label(v3(L(0, headTop + 0.28f * B).x, L(0, headTop + 0.28f * B).y, z) + v3(0, 0, oCam - 0.1f * B * d), std::to_string(i + 1).c_str());
    }
}

// ------------------------------------------------------------------ rotary engines
void buildRotary(Scene& sc, const EngineParams& p, const float phases[kMaxCyl], float shaftDeg, const EngineViewLive& live, float depth) {
    const int n = clampv(p.cylinders, 1, 4);
    const float R = p.rotorRadius_mm, e = p.eccentricity_mm, W = std::max(40.0f, p.rotorWidth_mm);
    const float pitch = W * 1.55f, z0 = -(n - 1) * pitch * 0.5f;
    const float d = std::max(0.2f, depth);
    const float tH = 0.55f * W * d, tR = 0.45f * W * d, tC = 0.30f * W * d;
    const float oH = 0, oC = 0.02f * W * d, oR = -0.05f * W * d, oF = -0.30f * W * d;
    // eccentric shaft through all rotors
    sc.frame(v3(0, 0, 0), v3(0, 0, 1), v3(0, 1, 0));
    {
        const float a0 = z0 - pitch * 0.38f, a1 = -z0 + pitch * 0.38f;
        const int segs = std::max(2, (int)std::ceil((a1 - a0) / (pitch * 0.25f)));
        sc.noAA = true;
        for (int k = 0; k < segs; ++k) {
            const float a = a0 + (a1 - a0) * k / segs, b = a0 + (a1 - a0) * (k + 1) / segs;
            sc.frame(v3(0, 0, 0.5f * (a + b)), v3(1, 0, 0), v3(0, 1, 0));
            sc.group(0.5f * (a + b));
            sc.add(circle(ImVec2(0, 0), R * 0.12f, 28), 0, b - a, kSteel, 0);
        }
        sc.noAA = false;
    }
    auto epi = [&](double t, double Rr) { return ImVec2((float)(e * std::cos(3 * t) + Rr * std::cos(t)), (float)(e * std::sin(3 * t) + Rr * std::sin(t))); };
    for (int r = 0; r < n; ++r) {
        const float z = z0 + r * pitch;
        sc.frame(v3(0, 0, z), v3(1, 0, 0), v3(0, 1, 0));
        sc.group(z);
        const double a = (shaftDeg - phases[r] + 90.0) * kDeg;   // +90: face 0 fires at the top (spark plugs)
        const double psi = a / 3.0;
        const float cx = (float)(e * std::cos(a)), cy = (float)(e * std::sin(a));
        // housing ring: quads between the running surface and an outer contour (only the rims get side faces)
        const int N = 72;
        const double Rin = R + 1.5, Rout = R + 1.5 + 0.22 * R;
        sc.noAA = true;
        for (int k = 0; k < N; ++k) {
            const double t0 = 2 * kPi * k / N, t1 = 2 * kPi * (k + 1) / N;
            Poly q = {epi(t0, Rin), epi(t1, Rin), epi(t1, Rout), epi(t0, Rout)};
            sc.add(q, oH, tH, kBlock, 0, (1ull << 0) | (1ull << 2));
        }
        // chambers: the housing region of each face in its stroke colour (flat, behind the rotor)
        for (int k = 0; k < 3; ++k) {
            const double t0 = psi + 2 * kPi * k / 3, t1 = t0 + 2 * kPi / 3;
            const double phi = wrapPos(shaftDeg - phases[r] + 360.0 * k, 1080.0);
            const ImU32 gc = gasColour(phi, live.advance, 1080.0);
            const int M = 18;
            for (int m = 0; m < M; ++m) {
                const double ta = t0 + (t1 - t0) * m / M, tb = t0 + (t1 - t0) * (m + 1) / M;
                Poly tri = {ImVec2(cx, cy), epi(ta, R), epi(tb, R)};
                sc.add(tri, oC, tC, gc, 0, 0);
            }
        }
        sc.noAA = false;
        // rotor with slightly convex flanks, combustion recesses, internal gear, apex seals
        Poly rp;
        for (int k = 0; k < 3; ++k)
            for (int m = 0; m < 12; ++m) {
                const double u = m / 12.0, ang = psi + 2 * kPi * k / 3 + u * 2 * kPi / 3;
                const double bulge = 1.0 - 0.40 * std::sin(u * kPi);
                rp.push_back(ImVec2(cx + (float)(R * bulge * std::cos(ang)), cy + (float)(R * bulge * std::sin(ang))));
            }
        sc.add(hull(rp), oR, tR, kSilver, kSilverRim);
        const float fr = oR - tR * 0.5f;
        for (int k = 0; k < 3; ++k) {
            const double ang = psi + 2 * kPi * k / 3 + kPi / 3;
            sc.decal(circle(ImVec2(cx + (float)(R * 0.56 * std::cos(ang)), cy + (float)(R * 0.56 * std::sin(ang))), R * 0.08f, 14), fr, kSteelDark);
            const double ta = psi + 2 * kPi * k / 3;
            sc.decal(circle(ImVec2(cx + (float)(R * std::cos(ta)), cy + (float)(R * std::sin(ta))), R * 0.035f, 10), fr, kGold, 0.6f);
        }
        sc.decal(circle(ImVec2(cx, cy), R * 0.34f, 28), fr, kSteelDark);
        sc.decal(circle(ImVec2(cx, cy), R * 0.29f, 28), fr, kSteel, 0.6f);
        // eccentric journal and the shaft centre
        sc.add(circle(ImVec2(cx, cy), R * 0.20f, 20), oF, 0.06f * W * d, kGold);
        sc.add(circle(ImVec2(0, 0), R * 0.12f, 16), oF - 0.04f * W * d, 0.05f * W * d, kSteelDark);
        // ports (bottom) and spark plugs (top)
        const float yb = -(float)(R - e) - 0.10f * R;
        sc.add(rect(-R * 0.66f, yb - R * 0.24f, -R * 0.40f, yb + R * 0.05f), oH, tH * 0.45f, rgba(0.25f, 0.45f, 0.85f));
        sc.add(rect(R * 0.40f, yb - R * 0.24f, R * 0.66f, yb + R * 0.05f), oH, tH * 0.45f, rgba(0.90f, 0.45f, 0.20f));
        bool fire = false;
        for (int k = 0; k < 3; ++k) if (wrapPos(shaftDeg - phases[r] + 360.0 * k + live.advance, 1080.0) < 14) fire = true;
        for (int s = -1; s <= 1; s += 2) {
            const float px = s * R * 0.14f, py = (float)Rout - (float)e;
            sc.add(rect(px - R * 0.035f, py - R * 0.04f, px + R * 0.035f, py + R * 0.22f), oH, tH * 0.4f, kSilver);
            if (fire) sc.decal(circle(ImVec2(px, py - R * 0.06f), R * 0.06f, 12), oH - tH * 0.5f, rgba(1, 0.95f, 0.55f, 0.95f), 1.0f);
        }
        char lb[16];
        std::snprintf(lb, sizeof(lb), "Rotor %d", r + 1);
        sc.label(v3(0, (float)Rout + R * 0.45f, z + oF), lb);
    }
}

// ------------------------------------------------------------------ projection and painter's drawing
struct Cam3 {
    V3 center, eye;
    float cy, sy, cp, sp, D, f;
    ImVec2 scr;
    V3 view(V3 w) const {   // world -> camera space (camera on -z looking +z)
        const V3 q = w - center;
        const float x1 = q.x * cy + q.z * sy, z1 = -q.x * sy + q.z * cy;
        const float y2 = q.y * cp - z1 * sp, z2 = q.y * sp + z1 * cp;
        return v3(x1, y2, z2 + D);
    }
    ImVec2 proj(V3 w, float* depth = nullptr) const {
        const V3 c = view(w);
        const float zc = std::max(c.z, D * 0.05f);
        if (depth) *depth = zc;
        return ImVec2(scr.x + f * c.x / zc, scr.y - f * c.y / zc);
    }
};

void render(ImDrawList* dl, const Scene& sc, const Cam3& cam) {
    // every group sits in its own slice along the crank axis (z): draw the slices far to near, and inside a slice
    // the parallel layers far to near (exact for layered cutaways that do not intersect)
    struct Key { float g, l; int i; };
    std::vector<Key> order(sc.slabs.size());
    for (size_t i = 0; i < sc.slabs.size(); ++i) {
        const Slab& s = sc.slabs[i];
        const float g = std::fabs(cam.eye.z - s.gz);
        const float lz = s.gz + s.N.z * s.off;
        order[i] = {g + s.group * 1e-4f, std::fabs(cam.eye.z - lz) - s.bias, (int)i};
    }
    std::sort(order.begin(), order.end(), [](const Key& a, const Key& b) { return a.g != b.g ? a.g > b.g : a.l > b.l; });
    ImVec2 fa[64], fb[64], scr[64];
    for (const Key& o : order) {
        const Slab& s = sc.slabs[o.i];
        const V3 na = s.N * (s.off - s.t * 0.5f), nb = s.N * (s.off + s.t * 0.5f);
        V3 wa[64];
        for (int k = 0; k < s.count; ++k) {
            const ImVec2 q = sc.pts[s.first + k];
            const V3 base = s.O + s.U * q.x + s.V * q.y;
            wa[k] = base + na;
            fa[k] = cam.proj(base + na);
            fb[k] = cam.proj(base + nb);
        }
        // which face looks at the camera (face A's outward normal is -N)
        const bool showA = dot(s.N * -1.0f, cam.eye - wa[0]) > 0;
        if (s.t <= 0 && !showA) continue;   // decals are single-sided
        if (s.sides) {
            const V3 light = v3(-0.36f, 0.78f, -0.51f);   // from the upper left, in front
            for (int k = 0; k < s.count; ++k) {
                if (!(s.sides & (1ull << k))) continue;
                const int k1 = (k + 1) % s.count;
                const ImVec2 a = sc.pts[s.first + k], b = sc.pts[s.first + k1];
                const V3 nrm = s.U * (b.y - a.y) + s.V * (a.x - b.x);   // outward edge normal (CCW polygon)
                const V3 mid = (wa[k] + wa[k1]) * 0.5f + s.N * (s.t * 0.5f);
                if (dot(nrm, cam.eye - mid) <= 0) continue;
                const float nl = std::sqrt(std::max(1e-12f, dot(nrm, nrm)));
                const ImU32 sideCol = shade(s.fill, 0.40f + 0.48f * std::max(0.0f, dot(nrm, light) / nl));
                ImVec2 q[4] = {fa[k], fa[k1], fb[k1], fb[k]};
                if (cross2(q[0], q[1], q[2]) < 0) std::swap(q[1], q[3]);
                // side strips of one part meet edge to edge: no anti-aliased fringe there (it shows as hairlines)
                dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
                dl->AddConvexPolyFilled(q, 4, sideCol);
                dl->Flags |= ImDrawListFlags_AntiAliasedFill;
            }
        }
        const ImVec2* face = showA ? fa : fb;
        // screen-space clockwise order for ImGui's anti-aliased fill
        float sa = 0;
        for (int k = 0; k < s.count; ++k) { const ImVec2 &u = face[k], &w = face[(k + 1) % s.count]; sa += u.x * w.y - w.x * u.y; }
        for (int k = 0; k < s.count; ++k) scr[k] = sa > 0 ? face[k] : face[s.count - 1 - k];
        if (s.noAA) dl->Flags &= ~ImDrawListFlags_AntiAliasedFill;
        dl->AddConvexPolyFilled(scr, s.count, s.fill);
        if (s.noAA) dl->Flags |= ImDrawListFlags_AntiAliasedFill;
        if (s.line) dl->AddPolyline(scr, s.count, s.line, ImDrawFlags_Closed, 1.2f);
    }
    // labels nearest first; one that would overlap a label already placed is left out (cylinders behind each
    // other in the front view)
    std::vector<std::pair<float, int>> lo;
    for (int i = 0; i < (int)sc.labels.size(); ++i) {
        float z = 0; cam.proj(sc.labels[i].at, &z);
        lo.push_back({z, i});
    }
    std::sort(lo.begin(), lo.end());
    std::vector<ImVec4> placed;
    for (const auto& li : lo) {
        const Label& lb = sc.labels[li.second];
        if (dot(lb.N * -1.0f, cam.eye - lb.at) <= 0) continue;
        const ImVec2 q = cam.proj(lb.at);
        const ImVec2 ts = ImGui::CalcTextSize(lb.text.c_str());
        const ImVec4 rc(q.x - ts.x * 0.5f - 2, q.y - ts.y * 0.5f - 1, q.x + ts.x * 0.5f + 2, q.y + ts.y * 0.5f + 1);
        bool hit = false;
        for (const ImVec4& o : placed) if (rc.x < o.z && rc.z > o.x && rc.y < o.w && rc.w > o.y) { hit = true; break; }
        if (hit) continue;
        placed.push_back(rc);
        dl->AddText(ImVec2(rc.x + 2, rc.y + 1), kLabel, lb.text.c_str());
    }
}

} // namespace

// orbit (left drag), pan (right / middle drag), zoom (wheel), reset (double click) on an invisible button over the view
static void viewInput(ViewCam& cam, ImVec2 size) {
    ImGui::InvisibleButton("engine_view3d", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    if (ImGui::IsItemActive()) {
        const ImVec2 dlt = ImGui::GetIO().MouseDelta;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            cam.yaw = std::fmod(cam.yaw + dlt.x * 0.45f, 360.0f);
            cam.pitch = clampv(cam.pitch + dlt.y * 0.45f, -85.0f, 85.0f);
        } else { cam.panX += dlt.x; cam.panY += dlt.y; }
    }
    if (ImGui::IsItemHovered()) {
        const float wh = ImGui::GetIO().MouseWheel;
        if (wh != 0) cam.zoom = clampv(cam.zoom * std::pow(1.12f, wh), 0.25f, 8.0f);
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) { const bool f = cam.flat; const float g = cam.glass; cam = ViewCam{}; cam.flat = f; cam.glass = g; }
    }
}

// the flat (2.5D) view: vector cutaways as layered slabs, drawn with ImGui
static void EngineView3DFlat(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live,
                             ViewCam& cam, ImVec2 size) {
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    viewInput(cam, size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), kBg, 4);
    dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);

    static Scene sc;
    sc.clear();
    if (isRotary(p)) buildRotary(sc, p, phases, crankDeg, live, cam.depth);
    else buildPiston(sc, p, phases, crankDeg, live, cam.depth);

    // fit the front view: bounds of everything in x / y, depth range in z
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f, z0 = 1e9f, z1 = -1e9f;
    for (const Slab& s : sc.slabs)
        for (int k = 0; k < s.count; ++k) {
            const ImVec2 q = sc.pts[s.first + k];
            const V3 w = s.O + s.U * q.x + s.V * q.y + s.N * s.off;
            x0 = std::min(x0, w.x); x1 = std::max(x1, w.x); y0 = std::min(y0, w.y); y1 = std::max(y1, w.y); z0 = std::min(z0, w.z); z1 = std::max(z1, w.z);
        }
    if (x1 > x0 && y1 > y0) {
        Cam3 c;
        c.center = v3((x0 + x1) * 0.5f, (y0 + y1) * 0.5f, (z0 + z1) * 0.5f);
        const float yaw = cam.yaw * (float)kDeg, pitch = cam.pitch * (float)kDeg;
        c.cy = std::cos(yaw); c.sy = std::sin(yaw); c.cp = std::cos(pitch); c.sp = std::sin(pitch);
        const float ext = std::max(std::max(x1 - x0, y1 - y0), z1 - z0);
        c.D = ext * 3.2f;
        c.eye = c.center + v3(c.D * c.cp * c.sy, -c.D * c.sp, -c.D * c.cp * c.cy);
        // fit what this view shows (projected bounds at f = 1), then zoom / pan on top
        float u0 = 1e9f, u1 = -1e9f, w0 = 1e9f, w1 = -1e9f;
        for (const Slab& s : sc.slabs)
            for (int k = 0; k < s.count; ++k) {
                const ImVec2 q = sc.pts[s.first + k];
                const V3 vc = c.view(s.O + s.U * q.x + s.V * q.y + s.N * s.off);
                const float zc = std::max(vc.z, c.D * 0.05f);
                u0 = std::min(u0, vc.x / zc); u1 = std::max(u1, vc.x / zc); w0 = std::min(w0, vc.y / zc); w1 = std::max(w1, vc.y / zc);
            }
        const float fit = std::min(size.x / std::max(1e-6f, u1 - u0), (size.y - ImGui::GetFontSize() * 1.5f) / std::max(1e-6f, w1 - w0)) * 0.88f;
        c.f = fit * cam.zoom;
        c.scr = ImVec2(p0.x + size.x * 0.5f - c.f * (u0 + u1) * 0.5f + cam.panX,
                       p0.y + (size.y - ImGui::GetFontSize() * 1.5f) * 0.5f + c.f * (w0 + w1) * 0.5f + cam.panY);
        render(dl, sc, c);
    }
    dl->AddText(ImVec2(p0.x + 8, p0.y + size.y - ImGui::GetFontSize() - 6), rgba(0.45f, 0.45f, 0.5f),
                "drag: rotate   right-drag: pan   wheel: zoom   double-click: reset");
    dl->PopClipRect();
}

// ------------------------------------------------------------------ the GPU 3D view
void EngineView3D(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live,
                  ViewCam& cam, ImVec2 size) {
    if (cam.flat || !gpu::ready()) { EngineView3DFlat(p, phases, crankDeg, live, cam, size); return; }
    static EngineModel model;
    buildEngineModel(p, phases, crankDeg, live, cam.glass, model);
    if (model.items.empty() || !(model.hi.x > model.lo.x)) { EngineView3DFlat(p, phases, crankDeg, live, cam, size); return; }
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    viewInput(cam, size);
    const int w = std::max(8, (int)size.x), h = std::max(8, (int)size.y);
    // camera: orbit around the model's centre, far enough to see all of it

    const m3::V3 c = (model.lo + model.hi) * 0.5f;
    const float rad = 0.5f * m3::len(model.hi - model.lo);
    const float fov = 30.0f * (float)kDeg;
    const float aspect = (float)w / (float)h;
    const float yaw = cam.yaw * (float)kDeg, pitch = cam.pitch * (float)kDeg;
    const m3::V3 dir = m3::v3(std::cos(pitch) * std::sin(yaw), std::sin(pitch), -std::cos(pitch) * std::cos(yaw));
    // fit: look from a distance that surely shows everything, measure where the parts' corners land on screen, then
    // centre on them and move in until they fill ~88 % of the view (twice: the second pass corrects the perspective).
    // Eased over ~0.25 s so moving parts and engine changes do not make the view jump.
    const float t = std::tan(0.5f * fov);
    float D0 = rad / std::sin(0.5f * fov * std::min(1.0f, aspect));
    m3::V3 tc = c;
    std::vector<m3::V3> box2;
    const std::vector<m3::V3>& pts = model.hull.empty() ? (box2 = {model.lo, model.hi}) : model.hull;
    for (int pass = 0; pass < 2; ++pass) {
        const m3::V3 e0 = tc + dir * D0;
        const m3::M4 v0 = m3::lookAt(e0, tc, m3::v3(0, 1, 0));
        float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
        for (const m3::V3& q0 : pts) {
            const m3::V3 q = m3::xform(v0, q0);
            const float zc = std::max(1e-3f, -q.z);
            const float sx = q.x / (zc * t * aspect), sy = q.y / (zc * t);
            x0 = std::min(x0, sx); x1 = std::max(x1, sx); y0 = std::min(y0, sy); y1 = std::max(y1, sy);
        }
        const m3::V3 fwd = m3::norm(tc - e0), right = m3::norm(m3::cross(fwd, m3::v3(0, 1, 0))), up = m3::cross(right, fwd);
        tc = tc + right * (0.5f * (x0 + x1) * D0 * t * aspect) + up * (0.5f * (y0 + y1) * D0 * t);
        const float ext = std::max(0.5f * (x1 - x0), 0.5f * (y1 - y0));
        if (ext > 1e-3f) D0 *= std::clamp(ext / 0.88f, 0.2f, 2.0f);
    }
    static m3::V3 sTarget;
    static float sD = -1;
    {
        const float k = sD < 0 ? 1.0f : 1.0f - std::exp(-ImGui::GetIO().DeltaTime / 0.25f);
        if (sD < 0) { sTarget = tc; sD = D0; }
        sTarget = sTarget + (tc - sTarget) * k;
        sD += (D0 - sD) * k;
    }
    const float D = sD / cam.zoom;
    m3::V3 target = sTarget, eye = sTarget + dir * D;
    {   // pan: pixels -> world in the view plane
        const m3::V3 fwd = m3::norm(target - eye), right = m3::norm(m3::cross(fwd, m3::v3(0, 1, 0))), up = m3::cross(right, fwd);
        const float perPx = 2.0f * D * t / (float)h;
        const m3::V3 shift = right * (-cam.panX * perPx) + up * (cam.panY * perPx);
        target = target + shift; eye = eye + shift;
    }
    const m3::M4 view = m3::lookAt(eye, target, m3::v3(0, 1, 0));
    const float dc = m3::len(eye - c);
    const m3::M4 proj = m3::perspective(fov, aspect, std::max(1.0f, dc - rad * 1.3f), dc + rad * 1.5f);
    const float bg[3] = {0.065f, 0.067f, 0.075f};   // the app's window background
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (gpu::begin(w, h, bg, view, proj, eye)) {
        for (const ModelItem& it : model.items) gpu::draw(it.mesh, it.world, it.mat);
        for (const ModelGlow& g : model.glows) gpu::glow(g.p, g.size, g.r, g.g, g.b, g.a);
        gpu::end();
        dl->AddImage((ImTextureID)gpu::texture(), p0, ImVec2(p0.x + w, p0.y + h));
    }
    dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);
    // labels (cylinder numbers), projected
    const m3::M4 vp = proj * view;
    auto toScreen = [&](m3::V3 q, ImVec2& s) {
        const float x = vp.at(0, 0) * q.x + vp.at(0, 1) * q.y + vp.at(0, 2) * q.z + vp.at(0, 3);
        const float y = vp.at(1, 0) * q.x + vp.at(1, 1) * q.y + vp.at(1, 2) * q.z + vp.at(1, 3);
        const float ww = vp.at(3, 0) * q.x + vp.at(3, 1) * q.y + vp.at(3, 2) * q.z + vp.at(3, 3);
        if (ww <= 1e-4f) return false;
        s = ImVec2(p0.x + (x / ww * 0.5f + 0.5f) * w, p0.y + (1.0f - (y / ww * 0.5f + 0.5f)) * h);
        return true;
    };
    for (const ModelLabel& lb : model.labels) {
        ImVec2 s;
        if (!toScreen(lb.p, s)) continue;
        const ImVec2 ts = ImGui::CalcTextSize(lb.text.c_str());
        dl->AddText(ImVec2(s.x - ts.x * 0.5f, s.y - ts.y * 0.5f), lb.accent ? rgba(1.0f, 0.62f, 0.25f) : rgba(0.62f, 0.63f, 0.68f), lb.text.c_str());
    }
    // firing order along the bottom, the cylinder that fires now in the app's orange
    if (!isRotary(p)) {
        const int n = clampv(p.cylinders, 1, kMaxCyl);
        const double cycle = isTwoStroke(p) ? 360.0 : 720.0;
        int firing = -1; double best = 1e9;
        for (int i = 0; i < n; ++i) {
            const double since = wrapPos(wrapPos(crankDeg - phases[i], cycle) + live.advance, cycle);
            if (since < best) { best = since; firing = i; }
        }
        const float slot = ImGui::GetFontSize() * 2.4f, y = p0.y + size.y - ImGui::GetFontSize() * 3.0f;
        float x = p0.x + size.x * 0.5f - slot * n * 0.5f;
        for (int k = 0; k < n; ++k, x += slot) {
            const int cylIdx = clampv((int)p.firingOrder[k], 0, n - 1);
            const bool on = cylIdx == firing && best < cycle / n;
            const ImU32 col = on ? rgba(1.0f, 0.62f, 0.25f) : rgba(0.50f, 0.51f, 0.56f);
            char b[8]; std::snprintf(b, sizeof(b), "%d", cylIdx + 1);
            const ImVec2 ts = ImGui::CalcTextSize(b);
            dl->AddText(ImVec2(x + slot * 0.5f - ts.x * 0.5f, y), col, b);
            dl->AddRectFilled(ImVec2(x + slot * 0.15f, y + ts.y + 3), ImVec2(x + slot * 0.85f, y + ts.y + 5), col);
        }
    }
    dl->AddText(ImVec2(p0.x + 8, p0.y + size.y - ImGui::GetFontSize() - 6), rgba(0.45f, 0.45f, 0.5f),
                "drag: rotate   right-drag: pan   wheel: zoom   double-click: reset");
    dl->PopClipRect();
}

} // namespace es::ui
