// Live Dashboard > Flow map (Package 3): the gas path of the engine as configured - air filter, compressor /
// supercharger, intercooler, throttle (or ITB / carburettor), plenum, runners, cylinders, header, turbine and
// wastegate, catalyst, muffler - drawn from the parameters, not from a fixed picture. Particles move through the pipes
// at a speed that follows the gas velocity and with a colour that follows the gas temperature; gas pushed backwards
// (reversion, standoff in front of the throttle, compressor surge) shows as hollow particles moving the other way.
// Pipe thickness follows the pressure. Components are coloured by their health (sim/Health.*), hovering shows the live
// values, clicking jumps to the panel that sets them. Below the map: the health panel. Diagnosis mode puts every
// sensor at its mounting position and makes the faulty ones blink.
// Diesel: no throttle, the EGR loop (valve, cooler) from the exhaust manifold back into the plenum, a VGT's vane ring,
// the common rail / injection pump, glow plugs, engine-out NOx / smoke at the tailpipe. Two-stroke: the reed valve,
// the crankcase and the transfer ducts instead of runners and valves, the expansion chamber on the exhaust.
// Style: the app's own theme (dark, orange accent); green / yellow / red only for component states.
#include "ui/App.h"
#include "ui/Widgets.h"
#include "sim/Health.h"
#include "imgui.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace es {
using namespace ui;

namespace {

ImU32 rgba(float r, float g, float b, float a = 1) { return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, a)); }

ImVec4 levelRgb(int lv) {
    switch (lv) {
    case kHealthOk: return ImVec4(0.36f, 0.78f, 0.45f, 1);
    case kHealthWarn: return ImVec4(0.96f, 0.76f, 0.25f, 1);
    case kHealthLimit: return ImVec4(0.95f, 0.32f, 0.26f, 1);
    default: return ImVec4(0.46f, 0.47f, 0.53f, 1);
    }
}
ImU32 levelCol(int lv, float a = 1) { ImVec4 c = levelRgb(lv); c.w = a; return ImGui::ColorConvertFloat4ToU32(c); }

// gas temperature: cold blue -> pale -> amber -> orange -> red-orange (the app's accent at the hot end)
ImVec4 tempRgb(double K) {
    struct Stop { float c; float r, g, b; };
    static const Stop s[] = {{-20, 0.30f, 0.55f, 1.00f}, {25, 0.55f, 0.75f, 0.95f}, {120, 0.86f, 0.86f, 0.80f},
                             {400, 1.00f, 0.72f, 0.32f}, {850, 1.00f, 0.45f, 0.14f}, {1100, 1.00f, 0.26f, 0.14f}};
    const float c = (float)(K - 273.15);
    if (c <= s[0].c) return ImVec4(s[0].r, s[0].g, s[0].b, 1);
    for (int i = 1; i < 6; ++i)
        if (c <= s[i].c) {
            const float t = (c - s[i - 1].c) / (s[i].c - s[i - 1].c);
            return ImVec4(s[i - 1].r + (s[i].r - s[i - 1].r) * t, s[i - 1].g + (s[i].g - s[i - 1].g) * t, s[i - 1].b + (s[i].b - s[i - 1].b) * t, 1);
        }
    return ImVec4(s[5].r, s[5].g, s[5].b, 1);
}
ImU32 tempCol(double K, float a = 1) { ImVec4 c = tempRgb(K); c.w = a; return ImGui::ColorConvertFloat4ToU32(c); }

// node values along the path
enum NodeP { NpAmb, NpCharge, NpPlenum, NpPortIn, NpPortEx, NpTurbIn, NpTurbOut };
enum NodeT { NtAmb, NtCompOut, NtCharge, NtPlenum, NtPortIn, NtCollector, NtPostTurb, NtEgrCool };
// flows the FlowState does not carry (drawn from other telemetry)
enum { kSegEgr = 100 };
double nodeP(const FlowState& f, int id) {
    switch (id) {
    case NpCharge: return f.pCharge; case NpPlenum: return f.pPlenum; case NpPortIn: return f.pPortIn; case NpPortEx: return f.pPortEx;
    case NpTurbIn: return f.pTurbIn; case NpTurbOut: return f.pTurbOut; default: return f.pAmb;
    }
}
double g_egrCoolK = 300;   // (EGR gas after the cooler, set each frame)
double nodeT(const FlowState& f, int id) {
    switch (id) {
    case NtEgrCool: return g_egrCoolK;
    case NtCompOut: return f.tCompOut; case NtCharge: return f.tCharge; case NtPlenum: return f.tPlenum; case NtPortIn: return f.tPortIn;
    case NtCollector: return f.tCollector; case NtPostTurb: return f.tPostTurb; default: return f.tAmb;
    }
}

struct Pt { float u, v; };
struct FmPipe {
    Pt a, b, c;           // polyline (b == c when it has one leg)
    int legs = 1;
    int seg = 0;          // FlowSeg
    float share = 1;      // part of the section's flow this pipe carries (runners / primaries: 1 / n)
    float diaMm = 50;     // for the gas velocity
    int pid = NpAmb, tid = NtAmb;
    int cyl = -1;         // primary of this cylinder (per-cylinder EGT colours its gas when fitted)
    const char* name = "";
};
enum CompKind { CkFilter, CkCompressor, CkBlower, CkIntercooler, CkBov, CkThrottle, CkCarb, CkPlenum, CkAirbox, CkHead, CkHeader,
                CkTurbine, CkWastegate, CkCat, CkMuffler, CkTail, CkEgrValve, CkEgrCooler, CkReed, CkCrankcase, CkChamber, CkInjPump };
struct FmComp {
    int kind = 0, part = -1;   // FlowPart for the health colour (-1: none)
    Pt c;                       // centre
    float wEm = 2, hEm = 2;     // size in font heights (or a rectangle in u / v when rect is set)
    bool rect = false;
    Pt r0, r1;
    const char* name = "";
    int section = -1;           // where a click goes
    const char* tab = nullptr;
    int lab = 0;                // where its label goes: 0 below, 1 above, 2 right, 3 left
};

struct FmLayout {
    std::vector<FmPipe> pipes;
    std::vector<FmComp> comps;
    std::vector<float> cylU;
    float hx0 = 0.6f, hx1 = 0.95f, uF = 0.04f, uC = 0.13f, uIC = 0.3f, uBov = 0.38f, uThr = 0.45f, uT = 0.13f, uWg = 0.2f;
    float vIn = 0.13f, vPl0 = 0.05f, vPl1 = 0.21f, vHd0 = 0.37f, vHd1 = 0.57f, vCol = 0.72f, vTail = 0.90f, vRail = 0.29f;
    bool boosted = false, turbo = false, blower = false, ic = false, bov = false, itb = false, carb = false, rotary = false, efiPort = false;
    bool diesel = false, cr = false, vgt = false, egr = false, egrCooler = false, two = false, tsPipe = false;
    float uEgr = 0, vCc0 = 0.25f, vCc1 = 0.33f, uCh0 = 0, uCh1 = 0;   // EGR riser; two-stroke crankcase band; expansion chamber span
    int n = 1;
};

FmLayout buildLayout(const EngineParams& p) {
    FmLayout L;
    L.n = std::max(1, std::min((int)p.cylinders, kMaxCyl));
    L.boosted = isBoosted(p);
    L.turbo = p.induction == Induction::Turbo;
    L.blower = p.induction == Induction::Supercharger;
    L.ic = L.boosted && p.intercooler;
    L.bov = (L.turbo && p.bov) || (L.blower && p.scBypass);
    L.itb = p.itb != 0;
    L.carb = p.fuelSystem == FuelSystem::Carburetor;
    L.rotary = isRotary(p);
    L.diesel = isDiesel(p);
    L.efiPort = p.fuelSystem == FuelSystem::EFI && p.injection == Injection::Port;
    L.cr = L.diesel && p.dieselInjSystem == 1;
    L.vgt = L.turbo && p.vgt;
    L.egr = L.diesel && p.egrValveDia_mm > 0;
    L.egrCooler = L.egr && p.egrCooler;
    L.two = isTwoStroke(p);
    L.tsPipe = L.two && p.tsPipe;
    if (L.diesel) L.bov = false;   // (no throttle to close against: a diesel has no blow-off valve)
    const int n = L.n;
    // engine on the right (room left of the canvas edge for the block sensors), the intake parts spaced to its left;
    // with many cylinders the engine grows to the left as far as the boost parts allow
    L.hx1 = 0.86f;
    L.uC = 0.17f;
    L.uT = L.uC;
    L.uWg = L.uT + 0.10f;
    const float minThr = L.boosted ? L.uC + (L.ic ? 0.09f : 0.0f) + (L.bov ? 0.07f : 0.0f) + 0.075f : 0.14f;
    // (a diesel's EGR riser runs up the left of the engine: keep room for its valve and cooler there)
    L.hx0 = std::max(minThr + 0.085f, L.hx1 - clampv(0.07f * n + 0.10f, 0.22f, 0.48f));
    L.uThr = L.hx0 - 0.085f;
    L.uBov = L.uThr - 0.07f;
    L.uIC = L.uBov - (L.bov ? 0.08f : 0.01f);
    for (int i = 0; i < n; ++i) L.cylU.push_back(L.hx0 + (i + 0.5f) * (L.hx1 - L.hx0) / n);

    const float thr = std::max(25.0f, p.throttleBore_mm);
    auto pipe = [&](Pt a, Pt b, int seg, int pid, int tid, float dia, const char* name, float share = 1, int cyl = -1) {
        FmPipe q; q.a = a; q.b = q.c = b; q.seg = seg; q.pid = pid; q.tid = tid; q.diaMm = dia; q.name = name; q.share = share; q.cyl = cyl;
        L.pipes.push_back(q);
    };
    auto pipe2 = [&](Pt a, Pt b, Pt c, int seg, int pid, int tid, float dia, const char* name) {
        FmPipe q; q.a = a; q.b = b; q.c = c; q.legs = 2; q.seg = seg; q.pid = pid; q.tid = tid; q.diaMm = dia; q.name = name;
        L.pipes.push_back(q);
    };
    auto comp = [&](int kind, int part, Pt c, float w, float h, const char* name, int section, const char* tab, int lab = 0) {
        FmComp k; k.kind = kind; k.part = part; k.c = c; k.wEm = w; k.hEm = h; k.name = name; k.section = section; k.tab = tab; k.lab = lab;
        L.comps.push_back(k);
    };
    auto box = [&](int kind, int part, Pt r0, Pt r1, const char* name, int section, const char* tab) {
        FmComp k; k.kind = kind; k.part = part; k.rect = true; k.r0 = r0; k.r1 = r1; k.c = Pt{(r0.u + r1.u) * 0.5f, (r0.v + r1.v) * 0.5f};
        k.name = name; k.section = section; k.tab = tab;
        L.comps.push_back(k);
    };
    const float vIn = L.vIn, plL = L.hx0 - 0.03f;

    // ---- intake: filter -> (compressor -> intercooler -> blow-off) -> throttle -> plenum / air box -> runners
    comp(CkFilter, kPartFilter, Pt{L.uF, vIn}, 1.6f, 2.6f, "Air filter", App::kFuelTuning, "fuel_sys", 1);
    const float thrEnd = L.itb ? plL : L.uThr;
    if (L.boosted) {
        pipe(Pt{L.uF, vIn}, Pt{L.uC, vIn}, kFlowInlet, NpAmb, NtAmb, thr * 1.2f, "Compressor inlet");
        if (L.ic) {
            pipe(Pt{L.uC, vIn}, Pt{L.uIC, vIn}, kFlowInlet, NpCharge, NtCompOut, thr * 1.1f, "Charge pipe (hot side)");
            pipe(Pt{L.uIC, vIn}, Pt{thrEnd, vIn}, kFlowThrottle, NpCharge, NtCharge, thr * 1.1f, "Charge pipe (cold side)");
        } else pipe(Pt{L.uC, vIn}, Pt{thrEnd, vIn}, kFlowThrottle, NpCharge, NtCompOut, thr * 1.1f, "Charge pipe");
        comp(L.turbo || p.scType == Supercharger::Centrifugal ? CkCompressor : CkBlower, kPartCompressor, Pt{L.uC, vIn}, 3.0f, 3.0f,
             L.turbo ? "Turbo compressor" : "Supercharger", App::kBoostControl, "boost_hw", 1);
        if (L.ic) comp(CkIntercooler, kPartIntercooler, Pt{L.uIC, vIn}, 2.0f, 3.6f, "Intercooler", App::kBoostControl, "boost_hw");
        if (L.bov) {
            pipe(Pt{L.uBov, vIn}, Pt{L.uBov, 0.015f}, kFlowBov, NpCharge, NtCharge, 25.0f, L.turbo ? "Blow-off valve vent" : "Bypass valve vent");
            comp(CkBov, kPartBov, Pt{L.uBov, 0.055f}, 1.3f, 1.3f, L.turbo ? "Blow-off valve" : "Bypass valve", App::kBoostControl, "boost_hw", 2);
        }
    } else {
        pipe(Pt{L.uF, vIn}, Pt{thrEnd, vIn}, L.itb ? kFlowThrottle : kFlowInlet, NpAmb, NtAmb, thr * 1.1f, L.itb ? "Air box inlet" : "Intake duct");
    }
    if (!L.itb) {
        pipe(Pt{L.uThr, vIn}, Pt{plL, vIn}, kFlowThrottle, NpPlenum, NtPlenum, thr, L.diesel ? "Charge pipe -> plenum" : "Throttle -> plenum");
        if (!L.diesel)   // (a diesel has no throttle: the load is the fuel)
            comp(L.carb ? CkCarb : CkThrottle, kPartThrottle, Pt{L.uThr, vIn}, 2.0f, 2.0f, L.carb ? "Carburettor" : "Throttle body",
                 App::kEngineControl, "dbw");
        box(CkPlenum, kPartPlenum, Pt{plL, L.vPl0}, Pt{L.hx1 + 0.012f, L.vPl1}, L.two ? "Air box" : "Plenum", App::kFuelTuning, "fuel_sys");
    } else {
        box(CkAirbox, kPartPlenum, Pt{plL, L.vPl0}, Pt{L.hx1 + 0.012f, L.vPl1}, L.boosted ? "Charge box" : "Air box", App::kFuelTuning, "fuel_sys");
    }
    const float rDia = std::max(20.0f, p.runnerDia_mm);
    if (L.two) {
        // two-stroke: air box -> reed valve -> crankcase (pumped by the piston's underside) -> transfer ducts -> cylinder
        L.vCc0 = L.vPl1 + 0.075f; L.vCc1 = L.vHd0 - 0.035f;
        for (int i = 0; i < n; ++i) {
            pipe(Pt{L.cylU[i], L.vPl1}, Pt{L.cylU[i], L.vCc0}, kFlowThrottle, NpPlenum, NtPlenum, rDia, "Intake -> reed valve", 1.0f / n, i);
            comp(CkReed, -1, Pt{L.cylU[i], (L.vPl1 + L.vCc0) * 0.5f}, 1.6f, 1.1f, "Reed valve", App::kEngineSetup, "valv", 3);
            const float w = (L.hx1 - L.hx0) / n * 0.22f;
            for (int s = -1; s <= 1; s += 2)
                pipe(Pt{L.cylU[i] + s * w, L.vCc1}, Pt{L.cylU[i] + s * w, L.vHd0}, kFlowIntakeValve, NpPortIn, NtPortIn, rDia * 0.7f,
                     "Transfer duct", 0.5f / n, i);
        }
        box(CkCrankcase, -1, Pt{L.hx0 - 0.012f, L.vCc0}, Pt{L.hx1 + 0.012f, L.vCc1}, "Crankcase", App::kEngineSetup, "valv");
    } else
    for (int i = 0; i < n; ++i)
        pipe(Pt{L.cylU[i], L.vPl1}, Pt{L.cylU[i], L.vHd0}, kFlowIntakeValve, NpPortIn, NtPortIn, rDia, L.rotary ? "Intake port runner" : "Intake runner",
             1.0f / n, i);
    if (L.itb)
        for (int i = 0; i < n; ++i)
            comp(L.carb ? CkCarb : CkThrottle, kPartThrottle, Pt{L.cylU[i], 0.265f}, 1.4f, 1.4f, L.carb ? "Carburettor" : "Throttle (ITB)",
                 App::kEngineControl, "dbw");
    L.vRail = L.itb ? 0.32f : 0.29f;

    // ---- cylinders / rotors
    box(CkHead, kPartHead, Pt{L.hx0 - 0.012f, L.vHd0}, Pt{L.hx1 + 0.012f, L.vHd1}, L.rotary ? "Rotor housings" : L.two ? "Cylinder" : "Cylinder head",
        App::kIgnitionTuning, "knock");
    // diesel injection: a common rail over the cylinders fed by its high-pressure pump, or a cam-driven injection pump
    if (L.diesel)
        comp(CkInjPump, -1, Pt{L.hx1 + 0.045f, L.vRail}, 2.4f, 1.6f, L.cr ? "High-pressure pump" : "Injection pump", App::kFuelTuning, "fuel_sys", 1);

    // ---- exhaust: primaries -> collector (header) -> turbine / wastegate -> downpipe -> cat -> muffler -> tail
    const float pDia = std::max(20.0f, p.primaryDia_mm), cDia = std::max(30.0f, p.collectorDia_mm), dDia = std::max(30.0f, p.pipeDia_mm);
    for (int i = 0; i < n; ++i)
        pipe(Pt{L.cylU[i], L.vHd1}, Pt{L.cylU[i], L.vCol}, kFlowExhaustValve, NpPortEx, NtCollector, pDia, L.rotary ? "Exhaust port" : "Primary", 1.0f / n, i);
    if (!L.two) box(CkHeader, kPartHeader, Pt{L.hx0 - 0.012f, L.vHd1 + 0.035f}, Pt{L.hx1 + 0.012f, L.vCol + 0.04f}, "Header", App::kEngineSetup, "exhaust");
    const float colStart = L.cylU[n - 1];
    if (L.turbo) {
        pipe(Pt{colStart, L.vCol}, Pt{L.uWg, L.vCol}, kFlowExhaustValve, NpTurbIn, NtCollector, cDia, "Collector");
        pipe(Pt{L.uWg, L.vCol}, Pt{L.uT, L.vCol}, kFlowTurbine, NpTurbIn, NtCollector, cDia, "Turbine inlet");
        if (!L.vgt) pipe(Pt{L.uWg, L.vCol}, Pt{L.uWg, L.vTail}, kFlowWastegate, NpTurbIn, NtCollector, cDia * 0.6f, "Wastegate");
        pipe(Pt{L.uT, L.vCol}, Pt{L.uT, L.vTail}, kFlowTurbine, NpTurbOut, NtPostTurb, dDia, "Turbine outlet");
        comp(CkTurbine, kPartTurbine, Pt{L.uT, L.vCol}, 3.0f, 3.0f, L.vgt ? "VGT turbine" : "Turbine", App::kBoostControl, "boost_hw", 3);
        if (!L.vgt) comp(CkWastegate, kPartTurbine, Pt{L.uWg, (L.vCol + L.vTail) * 0.5f}, 1.3f, 1.3f, "Wastegate", App::kBoostControl, "boost_hw", 2);
    } else if (L.two) {
        // the expansion chamber: header, diffuser, belly, baffle and stinger along the bottom of the map
        pipe2(Pt{colStart, L.vCol}, Pt{L.uT, L.vCol}, Pt{L.uT, L.vTail}, kFlowExhaustValve, NpTurbIn, NtCollector, cDia,
              L.tsPipe ? "Expansion chamber" : "Exhaust pipe");
        if (L.tsPipe) {
            L.uCh0 = L.uT + 0.02f; L.uCh1 = std::max(L.uCh0 + 0.12f, L.hx0 - 0.04f);
            box(CkChamber, kPartHeader, Pt{L.uCh0, L.vCol - 0.045f}, Pt{L.uCh1, L.vCol + 0.045f}, "Expansion chamber", App::kEngineSetup, "valv");
        }
    } else {
        pipe2(Pt{colStart, L.vCol}, Pt{L.uT, L.vCol}, Pt{L.uT, L.vTail}, kFlowExhaustValve, NpTurbIn, NtCollector, cDia, "Collector / downpipe");
    }
    pipe(Pt{L.uT, L.vTail}, Pt{0.975f, L.vTail}, kFlowTail, NpTurbOut, NtPostTurb, dDia, "Exhaust pipe");
    if (p.cat) comp(CkCat, kPartCat, Pt{0.50f, L.vTail}, 3.2f, 1.7f, L.diesel ? "Oxidation catalyst" : "Catalyst", App::kEngineSetup, "exhaust", 1);
    // diesel EGR: from the exhaust manifold (before the turbine) up the left of the engine, through its cooler and the
    // valve into the plenum
    if (L.egr) {
        L.uEgr = L.hx0 - 0.026f;
        const float vV = (L.vPl1 + L.vHd0) * 0.5f + 0.02f, vC = (L.vHd0 + L.vHd1) * 0.5f;
        const float eDia = std::max(15.0f, p.egrValveDia_mm);
        if (L.egrCooler) {
            pipe(Pt{L.uEgr, L.vCol}, Pt{L.uEgr, vC}, kSegEgr, NpTurbIn, NtCollector, eDia, "EGR pipe (hot)");
            pipe(Pt{L.uEgr, vC}, Pt{L.uEgr, L.vPl1}, kSegEgr, NpPlenum, NtEgrCool, eDia, "EGR pipe (cooled)");
            comp(CkEgrCooler, -1, Pt{L.uEgr, vC}, 1.5f, 3.4f, "EGR cooler", App::kFuelTuning, "fuel_sys", 3);
        } else pipe(Pt{L.uEgr, L.vCol}, Pt{L.uEgr, L.vPl1}, kSegEgr, NpTurbIn, NtCollector, eDia, "EGR pipe");
        comp(CkEgrValve, -1, Pt{L.uEgr, vV}, 1.3f, 1.3f, "EGR valve", App::kEngineControl, nullptr, 3);
    }
    if (p.muffler != Muffler::None) comp(CkMuffler, kPartMuffler, Pt{0.72f, L.vTail}, 4.2f, 2.1f, "Muffler", App::kEngineSetup, "exhaust", 1);
    comp(CkTail, -1, Pt{0.975f, L.vTail}, 1.0f, 1.0f, "Tailpipe", App::kEngineSetup, "exhaust");
    return L;
}

uint64_t layoutKey(const EngineParams& p) {
    uint64_t k = (uint64_t)p.cylinders;
    k = k * 7 + (uint64_t)p.induction; k = k * 5 + (uint64_t)p.scType; k = k * 3 + (uint64_t)(p.intercooler != 0);
    k = k * 3 + (uint64_t)(p.bov != 0); k = k * 3 + (uint64_t)(p.scBypass != 0); k = k * 3 + (uint64_t)(p.itb != 0);
    k = k * 3 + (uint64_t)p.fuelSystem; k = k * 3 + (uint64_t)(p.cat != 0); k = k * 5 + (uint64_t)p.muffler; k = k * 3 + (uint64_t)isRotary(p);
    k = k * 3 + (uint64_t)isTwoStroke(p); k = k * 3 + (uint64_t)(p.tsPipe != 0); k = k * 3 + (uint64_t)(p.vgt != 0);
    k = k * 3 + (uint64_t)(p.egrValveDia_mm > 0); k = k * 3 + (uint64_t)(p.egrCooler != 0); k = k * 3 + (uint64_t)(p.dieselInjSystem != 0);
    return k;
}

float distToSeg(ImVec2 p, ImVec2 a, ImVec2 b) {
    const float dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    float t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
    t = clampv(t, 0.0f, 1.0f);
    const float ex = a.x + t * dx - p.x, ey = a.y + t * dy - p.y;
    return std::sqrt(ex * ex + ey * ey);
}
// dashed line (ImGui has none)
void dashed(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 c, float th, float dash) {
    const float dx = b.x - a.x, dy = b.y - a.y, len = std::sqrt(dx * dx + dy * dy);
    if (len < 1) return;
    for (float s = 0; s < len; s += dash * 2) {
        const float e = std::min(len, s + dash);
        dl->AddLine(ImVec2(a.x + dx * s / len, a.y + dy * s / len), ImVec2(a.x + dx * e / len, a.y + dy * e / len), c, th);
    }
}
void dashedRect(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 c, float th, float dash) {
    dashed(dl, a, ImVec2(b.x, a.y), c, th, dash); dashed(dl, ImVec2(b.x, a.y), b, c, th, dash);
    dashed(dl, b, ImVec2(a.x, b.y), c, th, dash); dashed(dl, ImVec2(a.x, b.y), a, c, th, dash);
}
void textAt(ImDrawList* dl, ImVec2 c, ImU32 col, const char* s, float size, bool centre = true) {
    ImFont* f = ImGui::GetFont();
    const ImVec2 ts = f->CalcTextSizeA(size, FLT_MAX, 0, s);
    dl->AddText(f, size, centre ? ImVec2(c.x - ts.x * 0.5f, c.y - ts.y * 0.5f) : c, col, s);
}

const char* kSegNames[kFlowSegCount] = {"inlet", "blow-off", "throttle", "intake valves", "exhaust valves", "turbine", "wastegate", "tailpipe"};

} // namespace

int App::tabJump(const char* key) {
    if (!jumpTab_.empty() && jumpTab_ == key) { jumpTab_.clear(); return ImGuiTabItemFlags_SetSelected; }
    return 0;
}
void App::jumpTo(int section, const char* tab) {
    if (section < 0) return;
    section_ = section;
    jumpTab_ = tab ? tab : "";
}

void App::panelFlowMap() {
    const float em = ImGui::GetFontSize();
    const EngineParams& p = p_;
    const FlowState& f = tel_.flow;
    const HealthReport h = evaluateHealth(p, tel_);
    const FmLayout L = buildLayout(p);
    const bool blink = std::fmod(ImGui::GetTime(), 0.7) < 0.4;
    g_egrCoolK = tel_.egrGasC + 273.15;
    // forward / backward flow of a pipe's segment (kg/s); the EGR loop from its own telemetry
    auto segF = [&](int seg) { return seg == kSegEgr ? std::max(0.0, (double)tel_.egrGs * 1e-3) : f.fwd[seg]; };
    auto segB = [&](int seg) { return seg == kSegEgr ? 0.0 : f.back[seg]; };

    // ---- toolbar
    ImGui::Checkbox("Diagnosis", &fmDiag_);
    Help("Every sensor at its mounting position with its reading. A sensor that has a fault injected (Sensor\n"
         "Configuration) or that the ECU has rejected blinks red. Sensors off the gas path are listed below the map.");
    ImGui::SameLine();
    ImGui::Checkbox("Values", &fmLabels_);
    ImGui::SameLine();
    ImGui::Checkbox("Particles", &fmParticles_);
    ImGui::SameLine(0, em * 1.5f);
    // legend: temperature scale and flow direction
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 c = ImGui::GetCursorScreenPos();
        const float w = em * 8, hh = em * 0.5f, y = c.y + (ImGui::GetFrameHeight() - hh) * 0.5f;
        ImGui::TextDisabled("gas");
        ImGui::SameLine();
        const ImVec2 g0 = ImGui::GetCursorScreenPos();
        for (int i = 0; i < 32; ++i) {
            const double K = 273.15 - 20 + 1120.0 * i / 31.0;
            dl->AddRectFilled(ImVec2(g0.x + w * i / 32, y), ImVec2(g0.x + w * (i + 1) / 32 + 0.5f, y + hh), tempCol(K));
        }
        ImGui::Dummy(ImVec2(w, ImGui::GetFrameHeight()));
        ImGui::SameLine();
        ImGui::TextDisabled("-20 .. 1100 C   o = flowing backwards");
        (void)c;
    }

    // ---- canvas
    const float healthH = em * 7.6f;
    ImVec2 avail = ImGui::GetContentRegionAvail();
    const float W = std::max(em * 20, avail.x);
    const float H = clampv(avail.y - healthH - (fmDiag_ ? em * 1.6f : 0.0f), em * 14, W * 0.85f);
    const ImVec2 o = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##flowmap", ImVec2(W, H));
    const bool canvasHover = ImGui::IsItemHovered();
    const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(o, ImVec2(o.x + W, o.y + H), true);
    dl->AddRectFilled(o, ImVec2(o.x + W, o.y + H), rgba(0.050f, 0.052f, 0.059f));
    {   // drafting-table dots and a soft vignette towards the edges (the app's dark theme, no colour)
        const float g = em * 1.6f;
        for (float y = o.y + g * 0.5f; y < o.y + H; y += g)
            for (float x = o.x + g * 0.5f; x < o.x + W; x += g) dl->AddRectFilled(ImVec2(x, y), ImVec2(x + 1.2f, y + 1.2f), rgba(1, 1, 1, 0.045f));
        const float v = std::min(W, H) * 0.12f;
        const ImU32 dark = rgba(0, 0, 0, 0.35f), none = rgba(0, 0, 0, 0);
        dl->AddRectFilledMultiColor(o, ImVec2(o.x + W, o.y + v), dark, dark, none, none);
        dl->AddRectFilledMultiColor(ImVec2(o.x, o.y + H - v), ImVec2(o.x + W, o.y + H), none, none, dark, dark);
        dl->AddRectFilledMultiColor(o, ImVec2(o.x + v, o.y + H), dark, none, none, dark);
        dl->AddRectFilledMultiColor(ImVec2(o.x + W - v, o.y), ImVec2(o.x + W, o.y + H), none, dark, dark, none);
    }
    auto P = [&](Pt q) { return ImVec2(o.x + q.u * W, o.y + q.v * H); };
    const float small = em * 0.82f;
    const ImU32 kWall = rgba(0.23f, 0.24f, 0.28f), kText = rgba(0.86f, 0.86f, 0.90f), kDim = rgba(0.55f, 0.56f, 0.62f);
    const ImU32 kAccent = rgba(1.0f, 0.62f, 0.25f);

    // ---- pipes: thickness by pressure, a tint of the gas temperature
    const int np = (int)L.pipes.size();
    std::vector<float> pipeLen(np), pipeTh(np);
    for (int i = 0; i < np; ++i) {
        const FmPipe& q = L.pipes[i];
        const double pk = nodeP(f, q.pid);
        const float th = em * 0.55f * clampv((float)std::sqrt(std::max(1.0, pk) / 101.325), 0.55f, 2.0f);
        pipeTh[i] = th;
        const ImVec2 a = P(q.a), b = P(q.b), c = P(q.c);
        pipeLen[i] = std::hypot(b.x - a.x, b.y - a.y) + (q.legs > 1 ? std::hypot(c.x - b.x, c.y - b.y) : 0.0f);
    }
    for (int pass = 0; pass < 3; ++pass)
        for (int i = 0; i < np; ++i) {
            const FmPipe& q = L.pipes[i];
            const ImVec2 a = P(q.a), b = P(q.b), c = P(q.c);
            const float th = pipeTh[i];
            double T = nodeT(f, q.tid);
            if (q.cyl >= 0 && q.seg == kFlowExhaustValve && tel_.egtPerCyl) T = tel_.egtCylC[q.cyl] + 273.15;
            const float hot = clampv((float)(T - 273.15 - 300.0) / 600.0f, 0.0f, 1.0f);
            auto seg = [&](ImVec2 s0, ImVec2 s1) {
                if (pass == 0) {
                    if (hot > 0) dl->AddLine(s0, s1, tempCol(T, 0.10f * hot), th * 2.6f);   // heat haze round a hot pipe
                    dl->AddLine(s0, s1, rgba(0.02f, 0.02f, 0.025f, 0.9f), th + 3.0f);
                } else if (pass == 1) dl->AddLine(s0, s1, kWall, th);
                else {
                    dl->AddLine(s0, s1, tempCol(T, 0.30f + 0.25f * hot), th * 0.52f);
                    dl->AddLine(s0, s1, rgba(1, 1, 1, 0.06f), std::max(1.0f, th * 0.14f));   // a highlight along the bore
                }
            };
            auto cap = [&](ImVec2 pt) {
                if (pass == 0) dl->AddCircleFilled(pt, (th + 3.0f) * 0.5f, rgba(0.02f, 0.02f, 0.025f, 0.9f), 12);
                else if (pass == 1) dl->AddCircleFilled(pt, th * 0.5f, kWall, 12);
                else dl->AddCircleFilled(pt, th * 0.26f, tempCol(T, 0.30f + 0.25f * hot), 10);
            };
            seg(a, b); cap(a); cap(b);
            if (q.legs > 1) { seg(b, c); cap(c); }
        }

    // ---- particles: speed = gas velocity (mass flow / density / area), spacing fixed, colour = temperature
    const uint64_t key = layoutKey(p);
    if (key != fmKey_ || (int)fmParts_.size() != np) { fmKey_ = key; fmParts_.assign(np, {}); fmSpawn_.assign(np * 2, 0.0f); }
    const float dt = tel_.paused ? 0.0f : std::min(0.05f, ImGui::GetIO().DeltaTime);
    const float spacing = em * 1.3f;
    auto visSpeed = [&](double v) { return (float)std::min((double)em * 24.0, em * 2.0 * std::sqrt(std::max(0.0, v))); };
    auto pipeVel = [&](const FmPipe& q, double kgs) {
        const double rho = std::max(0.05, nodeP(f, q.pid) * 1e3 / (kR * std::max(200.0, nodeT(f, q.tid))));
        const double A = kPi * 0.25 * q.diaMm * q.diaMm * 1e-6;
        return kgs / (rho * A);
    };
    auto along = [&](const FmPipe& q, float s, float len) {
        const ImVec2 a = P(q.a), b = P(q.b), c = P(q.c);
        const float l1 = std::hypot(b.x - a.x, b.y - a.y);
        if (q.legs == 1 || s <= l1) { const float t = l1 > 0 ? s / l1 : 0; return ImVec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t); }
        const float l2 = std::max(1e-3f, len - l1), t = (s - l1) / l2;
        return ImVec2(b.x + (c.x - b.x) * t, b.y + (c.y - b.y) * t);
    };
    for (int i = 0; i < np; ++i) {
        const FmPipe& q = L.pipes[i];
        const float len = pipeLen[i];
        std::vector<FlowParticle>& ps = fmParts_[i];
        const float vf = visSpeed(pipeVel(q, segF(q.seg) * q.share)), vb = visSpeed(pipeVel(q, segB(q.seg) * q.share));
        for (FlowParticle& x : ps) x.s += x.dir * (x.dir > 0 ? vf : vb) * dt;
        ps.erase(std::remove_if(ps.begin(), ps.end(), [len](const FlowParticle& x) { return x.s < 0 || x.s > len; }), ps.end());
        if (ps.empty() && vf > 1 && dt > 0)   // fill the pipe at once instead of waiting for the first ones to travel through
            for (float s = spacing * 0.5f; s < len; s += spacing) ps.push_back(FlowParticle{s, 1.0f});
        for (int d = 0; d < 2; ++d) {
            const float v = d == 0 ? vf : vb;
            float& acc = fmSpawn_[i * 2 + d];
            acc += v * dt / spacing;
            int guard = 0;
            while (acc >= 1.0f && guard++ < 8) {
                acc -= 1.0f;
                const float s0 = acc * spacing;   // how far it already travelled this frame
                ps.push_back(FlowParticle{d == 0 ? s0 : len - s0, d == 0 ? 1.0f : -1.0f});
            }
            if (acc > 1.0f) acc = 0;
        }
        if (!fmParticles_) continue;
        double T = nodeT(f, q.tid);
        if (q.cyl >= 0 && q.seg == kFlowExhaustValve && tel_.egtPerCyl) T = tel_.egtCylC[q.cyl] + 273.15;
        const ImU32 c = tempCol(T);
        const float r = std::max(1.5f, pipeTh[i] * 0.28f);
        const ImU32 halo = tempCol(T, 0.16f);
        for (const FlowParticle& x : ps) {
            const ImVec2 pos = along(q, x.s, len);
            if (x.dir > 0) { dl->AddCircleFilled(pos, r * 2.1f, halo, 10); dl->AddCircleFilled(pos, r, c, 8); }
            else dl->AddCircle(pos, r * 1.25f, c, 10, 1.6f);
        }
    }

    // ---- components
    auto compRect = [&](const FmComp& k, ImVec2& a, ImVec2& b) {
        if (k.rect) { a = P(k.r0); b = P(k.r1); return; }
        const ImVec2 c = P(k.c);
        a = ImVec2(c.x - k.wEm * em * 0.5f, c.y - k.hEm * em * 0.5f);
        b = ImVec2(c.x + k.wEm * em * 0.5f, c.y + k.hEm * em * 0.5f);
    };
    int hoverComp = -1;
    for (int i = 0; i < (int)L.comps.size(); ++i) {
        ImVec2 a, b; compRect(L.comps[i], a, b);
        if (canvasHover && mouse.x >= a.x && mouse.x <= b.x && mouse.y >= a.y && mouse.y <= b.y) hoverComp = i;   // last (topmost) wins
    }
    const float thrAng = (float)(kPi * 0.5 * (1.0 - clampv((double)tel_.throttle, 0.0, 1.0)));
    // a card: drop shadow, a body a shade lighter at the top, a hairline highlight, the border (health colour); tint on top
    auto card = [&](ImVec2 a, ImVec2 b, float rad, ImU32 border, float bw, ImU32 tint) {
        dl->AddRectFilled(ImVec2(a.x + 2, a.y + 3), ImVec2(b.x + 3, b.y + 4), rgba(0, 0, 0, 0.40f), rad);
        dl->AddRectFilled(a, b, rgba(0.108f, 0.111f, 0.125f, 0.98f), rad);
        dl->AddRectFilled(a, ImVec2(b.x, a.y + (b.y - a.y) * 0.42f), rgba(1, 1, 1, 0.028f), rad, ImDrawFlags_RoundCornersTop);
        if (tint) dl->AddRectFilled(a, b, tint, rad);
        dl->AddLine(ImVec2(a.x + rad, a.y + 1.0f), ImVec2(b.x - rad, a.y + 1.0f), rgba(1, 1, 1, 0.08f), 1.0f);
        dl->AddRect(a, b, border, rad, 0, bw);
    };
    // a round part (wheel housings, valves): shadow, body, a lighter rim at the top
    auto disc = [&](ImVec2 c, float R, ImU32 border, float bw, ImU32 tint) {
        dl->AddCircleFilled(ImVec2(c.x + 2, c.y + 3), R, rgba(0, 0, 0, 0.40f), 28);
        dl->AddCircleFilled(c, R, rgba(0.108f, 0.111f, 0.125f), 28);
        dl->AddCircleFilled(ImVec2(c.x, c.y - R * 0.12f), R * 0.86f, rgba(1, 1, 1, 0.025f), 28);
        if (tint) dl->AddCircleFilled(c, R, tint, 28);
        dl->AddCircle(c, R, border, 28, bw);
    };
    float phases[kMaxCyl];
    computePhases(p, phases);
    for (int i = 0; i < (int)L.comps.size(); ++i) {
        const FmComp& k = L.comps[i];
        const int lv = k.part >= 0 ? h.part[k.part].level : kHealthNone;
        const ImU32 edge = lv >= kHealthOk ? levelCol(lv, lv == kHealthOk ? 0.8f : 1.0f) : kDim;
        const ImU32 fill = lv > kHealthOk ? levelCol(lv, 0.20f) : rgba(0.12f, 0.12f, 0.14f, 0.92f);
        const float bw = i == hoverComp ? 2.6f : 1.5f;
        ImVec2 a, b; compRect(k, a, b);
        const ImVec2 c = P(k.c);
        const float R = std::min(b.x - a.x, b.y - a.y) * 0.5f;
        switch (k.kind) {
        case CkFilter:
            card(a, b, 3, edge, bw, lv > kHealthOk ? fill : 0);
            for (int z = 1; z < 7; ++z) {
                const float y0 = a.y + (b.y - a.y) * (z - 1) / 6.0f, y1 = a.y + (b.y - a.y) * z / 6.0f;
                dl->AddLine(ImVec2(z & 1 ? a.x + 3 : b.x - 3, y0), ImVec2(z & 1 ? b.x - 3 : a.x + 3, y1), kDim, 1.0f);
            }
            break;
        case CkCompressor: case CkTurbine: {
            disc(c, R, edge, bw, lv > kHealthOk ? fill : (k.kind == CkTurbine ? tempCol(f.tCollector, 0.10f) : rgba(0.30f, 0.55f, 0.95f, 0.06f)));
            // wheel: blades turning with the shaft speed
            const float rot = (float)std::fmod(ImGui::GetTime() * (0.5 + 6.0 * clampv((double)tel_.speedFrac, 0.0, 1.2)), 2 * kPi);
            for (int z = 0; z < 6; ++z) {
                const float an = rot + z * (float)kPi / 3;
                dl->AddLine(c, ImVec2(c.x + std::cos(an) * R * 0.7f, c.y + std::sin(an) * R * 0.7f), edge, 1.4f);
            }
            dl->AddCircleFilled(c, R * 0.18f, edge, 12);
            if (k.kind == CkTurbine && L.vgt) {   // the guide vanes round the wheel: turned across the flow when closed
                const float open = clampv(tel_.wastegate, 0.0f, 1.0f);
                for (int z = 0; z < 10; ++z) {
                    const float an = z * (float)kPi / 5, tilt = 0.3f + 1.1f * (1.0f - open);
                    const ImVec2 m(c.x + std::cos(an) * R * 1.28f, c.y + std::sin(an) * R * 1.28f);
                    const float dx = std::cos(an + tilt) * R * 0.22f, dy = std::sin(an + tilt) * R * 0.22f;
                    dl->AddLine(ImVec2(m.x - dx, m.y - dy), ImVec2(m.x + dx, m.y + dy), kAccent, 2.0f);
                }
            }
            break;
        }
        case CkBlower: {
            dl->AddRectFilled(a, b, fill, R * 0.5f); dl->AddRect(a, b, edge, R * 0.5f, 0, bw);
            const float rot = (float)std::fmod(ImGui::GetTime() * (0.5 + 6.0 * clampv((double)tel_.speedFrac, 0.0, 1.2)), 2 * kPi);
            for (int s = -1; s <= 1; s += 2) {
                const ImVec2 rc(c.x + s * R * 0.42f, c.y);
                for (int z = 0; z < 3; ++z) {
                    const float an = s * rot + z * 2 * (float)kPi / 3;
                    dl->AddLine(rc, ImVec2(rc.x + std::cos(an) * R * 0.38f, rc.y + std::sin(an) * R * 0.38f), edge, 2.0f);
                }
            }
            break;
        }
        case CkIntercooler:
            card(a, b, 2, edge, bw, lv > kHealthOk ? fill : rgba(0.30f, 0.55f, 0.95f, 0.06f));
            for (float x = a.x + 4; x < b.x - 2; x += 4) dl->AddLine(ImVec2(x, a.y + 3), ImVec2(x, b.y - 3), kDim, 1.0f);
            break;
        case CkBov: {
            const ImVec2 t0(c.x, a.y), t1(a.x, b.y), t2(b.x, b.y);
            dl->AddTriangleFilled(t0, t1, t2, tel_.bovOpen ? levelCol(kHealthOk, 0.5f) : fill);
            dl->AddTriangle(t0, t1, t2, edge, bw);
            break;
        }
        case CkThrottle: {
            disc(c, R, edge, bw, lv > kHealthOk ? fill : 0);
            // butterfly: across the bore when shut, along the flow when wide open (the pipe runs left -> right or down)
            const float base = k.c.v > 0.2f ? (float)kPi * 0.5f : 0.0f;
            const float an = base + thrAng;   // plate direction: shut = across the pipe, open = along it
            dl->AddLine(ImVec2(c.x - std::cos(an) * R * 0.85f, c.y - std::sin(an) * R * 0.85f),
                        ImVec2(c.x + std::cos(an) * R * 0.85f, c.y + std::sin(an) * R * 0.85f), kText, 2.2f);
            break;
        }
        case CkCarb: {
            // venturi: narrow waist in the middle
            const float wq = (b.x - a.x) * 0.5f, hq = (b.y - a.y) * 0.5f;
            ImVec2 poly[6] = {ImVec2(c.x - wq, c.y - hq), ImVec2(c.x + wq, c.y - hq), ImVec2(c.x + wq * 0.45f, c.y),
                              ImVec2(c.x + wq, c.y + hq), ImVec2(c.x - wq, c.y + hq), ImVec2(c.x - wq * 0.45f, c.y)};
            dl->AddConvexPolyFilled(poly, 3, fill); dl->AddConvexPolyFilled(poly + 3, 3, fill);
            dl->AddPolyline(poly, 6, edge, ImDrawFlags_Closed, bw);
            break;
        }
        case CkPlenum: {
            card(a, b, em * 0.6f, edge, bw, lv > kHealthOk ? fill : tempCol(f.tPlenum, 0.05f));
            // the pressure as a bar on its right wall (to 250 kPa), the app's orange
            const ImVec2 t0(b.x - em * 1.0f, a.y + em * 1.5f), t1(b.x - em * 0.55f, b.y - em * 0.5f);
            if (t1.y - t0.y > em) {
                const float fr = clampv((float)(f.pPlenum / 250.0), 0.0f, 1.0f);
                dl->AddRectFilled(t0, t1, rgba(0, 0, 0, 0.35f), 3);
                dl->AddRectFilled(ImVec2(t0.x, t1.y - (t1.y - t0.y) * fr), t1, rgba(1.0f, 0.62f, 0.25f, 0.75f), 3);
                const float ya = t1.y - (t1.y - t0.y) * (float)(101.325 / 250.0);   // atmospheric mark
                dl->AddLine(ImVec2(t0.x - 3, ya), ImVec2(t1.x + 3, ya), rgba(1, 1, 1, 0.35f), 1.0f);
            }
            break;
        }
        case CkAirbox:
            card(a, b, em * 0.6f, rgba(0, 0, 0, 0), bw, lv > kHealthOk ? fill : 0); dashedRect(dl, a, b, edge, bw, em * 0.5f);
            break;
        case CkHead: {
            card(a, b, 5, edge, bw, lv > kHealthOk ? fill : 0);
            const float cw = (P(Pt{L.hx1, 0}).x - P(Pt{L.hx0, 0}).x) / L.n;
            const float cr = std::min(cw * 0.36f, (b.y - a.y) * 0.32f);
            const int plug = h.part[kPartPlugs].level;
            const float fl = std::max(tel_.floatIn, tel_.floatEx);
            for (int z = 0; z < L.n; ++z) {
                const ImVec2 cc(P(Pt{L.cylU[z], 0}).x, (a.y + b.y) * 0.5f);
                const ImU32 ce = levelCol(h.cyl[z]);
                if (L.rotary) {
                    const float an = (float)std::fmod(tel_.crankDeg / 3.0 * kDeg + z * 2.0944, 2 * kPi);
                    ImVec2 tri[3];
                    for (int t = 0; t < 3; ++t) tri[t] = ImVec2(cc.x + std::cos(an + t * 2.0944f) * cr, cc.y + std::sin(an + t * 2.0944f) * cr);
                    dl->AddTriangleFilled(tri[0], tri[1], tri[2], h.cyl[z] > kHealthOk ? levelCol(h.cyl[z], 0.3f) : rgba(0.17f, 0.17f, 0.19f));
                    dl->AddTriangle(tri[0], tri[1], tri[2], ce, 1.5f);
                } else {
                    dl->AddCircleFilled(ImVec2(cc.x + 1.5f, cc.y + 2.5f), cr, rgba(0, 0, 0, 0.45f), 24);
                    dl->AddCircleFilled(cc, cr, h.cyl[z] > kHealthOk ? levelCol(h.cyl[z], 0.3f) : rgba(0.15f, 0.15f, 0.17f), 24);
                    dl->AddCircleFilled(ImVec2(cc.x, cc.y + cr * 0.12f), cr * 0.80f, rgba(0, 0, 0, 0.22f), 24);   // the bore's depth
                    if (tel_.running) {   // combustion: an orange flash fading over ~70 deg after it fires
                        const double cyc = isTwoStroke(p) ? 360.0 : 720.0;
                        const double since = std::fmod(std::fmod(animAngle_ - phases[z], cyc) + cyc, cyc);
                        if (since < 70.0) {
                            const float k2 = 1.0f - (float)(since / 70.0);
                            dl->AddCircleFilled(cc, cr * (1.0f + 0.35f * k2), rgba(1.0f, 0.62f, 0.25f, 0.10f * k2), 24);
                            dl->AddCircleFilled(cc, cr * 0.92f, rgba(1.0f, 0.55f, 0.18f, 0.55f * k2), 24);
                            dl->AddCircleFilled(cc, cr * 0.45f, rgba(1.0f, 0.85f, 0.55f, 0.55f * k2), 20);
                        }
                    }
                    dl->AddCircle(cc, cr, ce, 24, 1.5f);
                    if (L.two) {   // ports in the cylinder wall: transfers both sides (blue), exhaust below (orange)
                        for (int s = -1; s <= 1; s += 2)
                            dl->AddLine(ImVec2(cc.x + s * cr, cc.y - cr * 0.15f), ImVec2(cc.x + s * cr, cc.y + cr * 0.45f), rgba(0.30f, 0.55f, 0.95f), 3.0f);
                        dl->AddLine(ImVec2(cc.x - cr * 0.4f, cc.y + cr + 2), ImVec2(cc.x + cr * 0.4f, cc.y + cr + 2), kAccent, 3.0f);
                    } else {
                        // valves: intake on top, exhaust below (yellow / red when they float)
                        const ImU32 vc = fl > 0.5f ? levelCol(kHealthLimit) : fl > 0.05f ? levelCol(kHealthWarn) : kDim;
                        dl->AddLine(ImVec2(cc.x - cr * 0.5f, cc.y - cr - 3), ImVec2(cc.x + cr * 0.5f, cc.y - cr - 3), vc, 2.0f);
                        dl->AddLine(ImVec2(cc.x - cr * 0.5f, cc.y + cr + 3), ImVec2(cc.x + cr * 0.5f, cc.y + cr + 3), vc, 2.0f);
                    }
                }
                if (L.diesel) {   // injector at the centre; the glow plug beside it, glowing the app's orange as it heats
                    dl->AddCircleFilled(cc, std::max(2.0f, cr * 0.16f), rgba(0.85f, 0.70f, 0.30f), 10);
                    if (p.glowPlugs) {
                        const float g = clampv((tel_.glowTipC - 200.0f) / 900.0f, 0.0f, 1.0f);
                        dl->AddCircleFilled(ImVec2(cc.x + cr * 0.42f, cc.y - cr * 0.3f), std::max(1.8f, cr * 0.11f),
                                            g > 0.05f ? rgba(1.0f, 0.62f, 0.25f, 0.35f + 0.65f * g) : kDim, 8);
                    }
                } else   // spark plug: dot at the centre, coloured by the plugs' state
                    dl->AddCircleFilled(cc, std::max(2.0f, cr * 0.16f), levelCol(plug), 10);
                if (cw > em * 1.4f) {
                    char lb[8]; std::snprintf(lb, sizeof(lb), "%s%d", L.rotary ? "R" : "", z + 1);
                    textAt(dl, ImVec2(cc.x + cr * 0.75f, cc.y - cr * 0.95f), kDim, lb, small * 0.85f, false);
                }
            }
            break;
        }
        case CkHeader:
            dl->AddRectFilled(a, b, tempCol(f.tCollector, 0.035f), 6);
            dl->AddRect(a, b, levelCol(lv, 0.55f), 6, 0, bw);
            break;
        case CkWastegate: {
            // bow-tie valve, filled as far as it is open
            const ImVec2 q0(a.x, a.y), q1(b.x, a.y), q2(a.x, b.y), q3(b.x, b.y);
            dl->AddTriangleFilled(q0, q1, c, levelCol(lv, 0.15f + 0.6f * clampv(tel_.wastegate, 0.0f, 1.0f)));
            dl->AddTriangleFilled(q2, q3, c, levelCol(lv, 0.15f + 0.6f * clampv(tel_.wastegate, 0.0f, 1.0f)));
            dl->AddTriangle(q0, q1, c, edge, bw); dl->AddTriangle(q2, q3, c, edge, bw);
            break;
        }
        case CkCat:
            card(a, b, 3, kDim, bw, tempCol(f.tPostTurb, 0.08f));
            for (float x = a.x + 5; x < b.x - 3; x += 5) dl->AddLine(ImVec2(x, a.y + 3), ImVec2(x, b.y - 3), rgba(0.3f, 0.3f, 0.34f), 1.0f);
            break;
        case CkMuffler:
            card(a, b, R, kDim, bw, 0);
            break;
        case CkTail:
            dl->AddTriangleFilled(ImVec2(b.x, c.y), ImVec2(a.x, a.y), ImVec2(a.x, b.y), kDim);
            break;
        case CkEgrValve: {   // poppet seat: a bow-tie filled as far as it is open, in the exhaust's orange
            const float open = clampv(tel_.egrPos, 0.0f, 1.0f);
            const ImVec2 q0(a.x, a.y), q1(a.x, b.y), q2(b.x, a.y), q3(b.x, b.y);
            dl->AddTriangleFilled(q0, q1, c, rgba(1.0f, 0.62f, 0.25f, 0.15f + 0.6f * open));
            dl->AddTriangleFilled(q2, q3, c, rgba(1.0f, 0.62f, 0.25f, 0.15f + 0.6f * open));
            dl->AddTriangle(q0, q1, c, kDim, bw); dl->AddTriangle(q2, q3, c, kDim, bw);
            break;
        }
        case CkEgrCooler:   // a water jacket round the tube bundle: cold side blue
            card(a, b, 3, rgba(0.30f, 0.55f, 0.95f, 0.8f), bw, rgba(0.30f, 0.55f, 0.95f, 0.07f));
            for (float y = a.y + 4; y < b.y - 2; y += 4) dl->AddLine(ImVec2(a.x + 3, y), ImVec2(b.x - 3, y), rgba(0.3f, 0.3f, 0.34f), 1.0f);
            break;
        case CkReed: {   // V-shaped reed block, petals opening with the reed
            const float open = clampv(tel_.tsReed, 0.0f, 1.0f);
            dl->AddTriangleFilled(ImVec2(a.x, a.y), ImVec2(b.x, a.y), ImVec2(c.x, b.y), rgba(0.12f, 0.12f, 0.14f));
            dl->AddTriangle(ImVec2(a.x, a.y), ImVec2(b.x, a.y), ImVec2(c.x, b.y), kDim, bw);
            for (int s = -1; s <= 1; s += 2) {
                const float an = (float)kPi * 0.5f + s * (0.45f + 0.55f * open);
                dl->AddLine(ImVec2(c.x, b.y), ImVec2(c.x + std::cos(an) * R * 1.3f, b.y - std::sin(an) * R * 1.3f), kAccent, 2.0f);
            }
            break;
        }
        case CkCrankcase:
            card(a, b, em * 0.5f, kDim, bw, rgba(0.30f, 0.55f, 0.95f, 0.04f + 0.10f * clampv(tel_.tsReed, 0.0f, 1.0f)));
            break;
        case CkChamber: {   // header, diffuser, belly, baffle, stinger
            const float h0 = (b.y - a.y) * 0.18f, h1 = (b.y - a.y) * 0.5f, cy = c.y, wq = b.x - a.x;
            ImVec2 top[6] = {ImVec2(b.x, cy - h0), ImVec2(b.x - wq * 0.18f, cy - h0), ImVec2(b.x - wq * 0.45f, cy - h1),
                             ImVec2(a.x + wq * 0.32f, cy - h1), ImVec2(a.x + wq * 0.10f, cy - h0 * 0.6f), ImVec2(a.x, cy - h0 * 0.6f)};
            ImVec2 poly[12];
            for (int z = 0; z < 6; ++z) { poly[z] = top[z]; poly[11 - z] = ImVec2(top[z].x, 2 * cy - top[z].y); }
            const ImU32 hot = tempCol(f.tCollector, 0.25f);
            for (int z = 0; z < 5; ++z) {   // fill as quads along the length
                ImVec2 qd[4] = {poly[z], poly[z + 1], poly[10 - z], poly[11 - z]};
                dl->AddConvexPolyFilled(qd, 4, hot);
            }
            dl->AddPolyline(poly, 12, edge, ImDrawFlags_Closed, bw);
            break;
        }
        case CkInjPump:
            card(a, b, 3, rgba(0.85f, 0.70f, 0.30f, 0.9f), bw, rgba(0.85f, 0.70f, 0.30f, 0.06f));
            dl->AddCircle(ImVec2(a.x + (b.x - a.x) * 0.3f, c.y), (b.y - a.y) * 0.28f, rgba(0.85f, 0.70f, 0.30f, 0.9f), 12, 1.4f);
            break;
        default: break;
        }
    }
    // turbo shaft / supercharger drive
    if (L.turbo) dashed(dl, P(Pt{L.uC, L.vIn + 0.06f}), P(Pt{L.uT, L.vCol - 0.06f}), kDim, 2.0f, em * 0.4f);
    if (L.blower) {
        const float vm = (L.vHd0 + L.vHd1) * 0.5f;
        const ImVec2 pul = P(Pt{L.hx0 - 0.035f, vm});
        dashed(dl, P(Pt{L.uC, L.vIn + 0.06f}), P(Pt{L.uC, vm}), kDim, 2.0f, em * 0.4f);
        dashed(dl, P(Pt{L.uC, vm}), ImVec2(pul.x - em * 0.6f, pul.y), kDim, 2.0f, em * 0.4f);
        dl->AddCircle(pul, em * 0.6f, kDim, 16, 1.5f);
        if (fmLabels_) textAt(dl, ImVec2((P(Pt{L.uC, vm}).x + pul.x) * 0.5f, pul.y - small * 0.8f), kDim, "belt drive from the crank", small * 0.85f);
    }
    // fuel rail and injectors (port injection) / the fuel line
    if (L.efiPort) {
        const ImVec2 r0 = P(Pt{L.hx0 - 0.02f, L.vRail}), r1 = P(Pt{L.hx1 + 0.005f, L.vRail});
        dl->AddLine(r0, r1, rgba(0.62f, 0.52f, 0.25f, 0.8f), 3.0f);
        const float duty = clampv(tel_.injDuty, 0.0f, 1.0f);
        for (int z = 0; z < L.n; ++z) {
            const ImVec2 ic = P(Pt{L.cylU[z], L.vRail});
            dl->AddCircleFilled(ImVec2(ic.x + em * 0.35f, ic.y), em * 0.22f, duty > 0.9f ? levelCol(kHealthLimit) : rgba(0.85f, 0.70f, 0.30f), 8);
        }
    }

    // diesel injection: the common rail over the cylinders (fed by the high-pressure pump), or one line per cylinder from
    // a cam-driven injection pump
    if (L.diesel) {
        const ImU32 fuelC = rgba(0.85f, 0.70f, 0.30f, 0.85f);
        const ImVec2 pump = P(Pt{L.hx1 + 0.045f, L.vRail});
        if (L.cr) {
            const ImVec2 r0 = P(Pt{L.hx0 + 0.005f, L.vRail}), r1 = P(Pt{L.hx1 - 0.005f, L.vRail});
            const float load = clampv(tel_.railBar / std::max(300.0f, p.railMaxBar), 0.0f, 1.0f);
            dl->AddLine(ImVec2(pump.x - em * 1.2f, pump.y), r1, fuelC, 2.0f);
            dl->AddLine(r0, r1, fuelC, 3.0f + 3.0f * load);
            for (int z = 0; z < L.n; ++z) {
                const ImVec2 ic = P(Pt{L.cylU[z], L.vRail});
                dl->AddLine(ic, ImVec2(ic.x, P(Pt{0, L.vHd0}).y), fuelC, 1.4f);
                dl->AddCircleFilled(ic, em * 0.2f, fuelC, 8);
            }
        } else
            for (int z = 0; z < L.n; ++z) {   // one steel line per cylinder, side by side
                const ImVec2 ic = P(Pt{L.cylU[z], L.vHd0});
                const float y = P(Pt{0, L.vRail}).y + (z - (L.n - 1) * 0.5f) * em * 0.3f;
                const ImVec2 out(pump.x - em * 1.2f, pump.y + (z - (L.n - 1) * 0.5f) * em * 0.12f);
                dl->AddLine(out, ImVec2(ic.x + em * 0.3f, y), fuelC, 1.3f);
                dl->AddLine(ImVec2(ic.x + em * 0.3f, y), ImVec2(ic.x + em * 0.3f, ic.y), fuelC, 1.3f);
            }
    }
    // engine-out smoke at the tailpipe: black soot puffs with the smoke number (a diesel), drifting with the flow
    if (L.diesel && tel_.running) {
        const float fsn = clampv(tel_.smokeFsn, 0.0f, 6.0f);
        const ImVec2 tip = P(Pt{0.975f, L.vTail});
        const float tt = (float)ImGui::GetTime();
        for (int z = 0; z < 7; ++z) {
            const float ph = std::fmod(tt * 0.6f + z / 7.0f, 1.0f);
            const ImVec2 c(tip.x - em * 0.5f + ph * em * 0.4f, tip.y - ph * em * 3.0f + std::sin(tt * 2.0f + z) * em * 0.3f);
            const float a = (1.0f - ph) * clampv(fsn / 3.0f, 0.0f, 1.0f);
            if (a > 0.02f) dl->AddCircleFilled(c, em * (0.35f + ph * 0.9f), rgba(0.02f, 0.02f, 0.025f, 0.75f * a), 14);
        }
    }

    // ---- labels: name + live value, each on a dark rounded plate so it reads over the pipes and particles
    auto plate = [&](ImVec2 tl, ImVec2 br) {
        dl->AddRectFilled(ImVec2(tl.x - 4, tl.y - 2), ImVec2(br.x + 4, br.y + 2), rgba(0.035f, 0.036f, 0.042f, 0.78f), 4);
    };
    if (fmLabels_) {
        for (const FmComp& k : L.comps) {
            if (k.kind == CkTail || (k.kind == CkThrottle && L.itb) || (k.kind == CkCarb && L.itb)) continue;
            ImVec2 a, b; compRect(k, a, b);
            const char* val = k.part >= 0 ? h.part[k.part].value : "";
            static char vb[64];
            if (k.kind == CkWastegate) { std::snprintf(vb, sizeof(vb), "%.0f %% open", tel_.wastegate * 100); val = vb; }
            if (k.kind == CkTurbine && L.vgt) { std::snprintf(vb, sizeof(vb), "vanes %.0f %% open", tel_.wastegate * 100); val = vb; }
            if (k.kind == CkEgrValve) { std::snprintf(vb, sizeof(vb), "%.0f %%  %.0f %% EGR", tel_.egrPos * 100, tel_.egrRate * 100); val = vb; }
            if (k.kind == CkEgrCooler) { std::snprintf(vb, sizeof(vb), "out %.0f C", tel_.egrGasC); val = vb; }
            if (k.kind == CkReed) { std::snprintf(vb, sizeof(vb), "%.0f %% open", tel_.tsReed * 100); val = vb; }
            if (k.kind == CkCrankcase) { std::snprintf(vb, sizeof(vb), "%.0f kPa", tel_.tsCrankcaseKPa); val = vb; }
            if (k.kind == CkChamber) { std::snprintf(vb, sizeof(vb), "tuned %.0f rpm", p.tsPipeRpm); val = vb; }
            if (k.kind == CkInjPump) { if (L.cr) std::snprintf(vb, sizeof(vb), "rail %.0f bar", tel_.railBar); else std::snprintf(vb, sizeof(vb), "%.1f mg", tel_.dieselFuelMg + tel_.pilotMg); val = vb; }
            if (k.kind == CkReed && k.c.u != L.cylU[0]) continue;   // (one label for the reed valves)
            // (the header's label goes under its frame: the primaries fill the inside)
            ImVec2 at = k.rect ? ImVec2(a.x + em * 0.4f, (k.kind == CkHeader || k.kind == CkChamber) ? b.y + em * 0.15f : a.y + em * 0.15f) : ImVec2((a.x + b.x) * 0.5f, k.lab == 1 ? a.y - small * 2.1f : b.y + em * 0.1f);
            if (!k.rect && k.lab >= 2) {
                // beside the component: left-aligned to its right, or right-aligned to its left
                ImFont* fnt = ImGui::GetFont();
                const float w0 = fnt->CalcTextSizeA(small, FLT_MAX, 0, k.name).x, w1 = val ? fnt->CalcTextSizeA(small, FLT_MAX, 0, val).x : 0;
                const float y0 = (a.y + b.y) * 0.5f - small;
                const float x0 = k.lab == 2 ? b.x + em * 0.4f : a.x - em * 0.4f - w0, x1 = k.lab == 2 ? b.x + em * 0.4f : a.x - em * 0.4f - w1;
                plate(ImVec2(std::min(x0, x1), y0), ImVec2(std::max(x0 + w0, x1 + w1), y0 + small * ((val && *val) ? 2.0f : 1.0f)));
                dl->AddText(fnt, small, ImVec2(x0, y0), kDim, k.name);
                if (val && *val) dl->AddText(fnt, small, ImVec2(x1, y0 + small), kText, val);
                continue;
            }
            if (k.rect) {
                {
                    const float nw0 = ImGui::GetFont()->CalcTextSizeA(small, FLT_MAX, 0, k.name).x;
                    const float vw0 = (val && *val) ? ImGui::GetFont()->CalcTextSizeA(small, FLT_MAX, 0, val).x + em * 0.5f : 0.0f;
                    if (k.kind == CkHeader || k.kind == CkChamber) plate(at, ImVec2(at.x + nw0 + vw0, at.y + small));
                }
                dl->AddText(ImGui::GetFont(), small, at, kDim, k.name);
                if (val && *val) {
                    const float nw = ImGui::GetFont()->CalcTextSizeA(small, FLT_MAX, 0, k.name).x;
                    dl->AddText(ImGui::GetFont(), small, ImVec2(at.x + nw + em * 0.5f, at.y), kText, val);
                }
            } else {
                const float nw0 = ImGui::GetFont()->CalcTextSizeA(small, FLT_MAX, 0, k.name).x;
                const float vw0 = (val && *val) ? ImGui::GetFont()->CalcTextSizeA(small, FLT_MAX, 0, val).x : 0.0f;
                const float hw = std::max(nw0, vw0) * 0.5f;
                plate(ImVec2(at.x - hw, at.y), ImVec2(at.x + hw, at.y + small * ((val && *val) ? 2.0f : 1.0f)));
                textAt(dl, ImVec2(at.x, at.y + small * 0.5f), kDim, k.name, small);
                if (val && *val) textAt(dl, ImVec2(at.x, at.y + small * 1.5f), kText, val, small);
            }
        }
        if (L.itb) {
            char b[32]; std::snprintf(b, sizeof(b), "%s %.0f %%", L.carb ? "carbs" : "throttles", tel_.throttle * 100);
            textAt(dl, P(Pt{L.hx0 - 0.05f, 0.265f}), kText, b, small);
        }
        if (L.efiPort) textAt(dl, P(Pt{L.hx0 - 0.055f, L.vRail}), kDim, "fuel rail", small * 0.9f);
        if (L.diesel && tel_.running) {   // engine-out emissions at the tailpipe
            char b[64]; std::snprintf(b, sizeof(b), "NOx %.0f ppm   smoke FSN %.2f", tel_.noxPpm, tel_.smokeFsn);
            const ImVec2 at = P(Pt{0.84f, L.vTail + 0.045f});
            textAt(dl, at, rgba(1.0f, 0.78f, 0.5f), b, small * 0.9f);
        }
        if (L.two) {
            char b[80]; std::snprintf(b, sizeof(b), "delivery ratio %.2f   trapping %.2f   scavenging %.2f", tel_.tsDelivery, tel_.tsTrapping, tel_.tsScavenging);
            textAt(dl, P(Pt{(L.hx0 + L.hx1) * 0.5f, L.vHd1 + 0.02f}), kText, b, small * 0.85f);
        }
    }

    // ---- sensors at their mounting positions (lambda / EGT / EMAP always; everything in diagnosis mode)
    struct SensorSpot { int id; Pt at; bool always; bool left = false; };
    std::vector<SensorSpot> spots;
    {
        const float vMid = (L.vHd0 + L.vHd1) * 0.5f;
        const float uColMid = L.turbo ? (L.uWg + L.hx0) * 0.5f : (L.uT + L.hx0) * 0.5f;
        spots.push_back({kSnsMAF, Pt{L.uF + 0.045f, L.vIn + 0.06f}, false});
        spots.push_back({kSnsIAT, Pt{L.hx0 + 0.01f, L.vPl1 - 0.035f}, false});
        spots.push_back({kSnsMAP, Pt{L.hx1 - 0.02f, L.vPl1 - 0.035f}, false});
        spots.push_back({kSnsTPS, Pt{L.itb ? L.hx0 - 0.03f : L.uThr, L.itb ? 0.30f : L.vIn + 0.10f}, false});
        spots.push_back({kSnsAPP, Pt{L.itb ? L.hx0 - 0.065f : L.uThr - 0.035f, L.itb ? 0.33f : L.vIn + 0.135f}, false});
        spots.push_back({kSnsCMP, Pt{L.hx1 + 0.022f, L.vHd0 + 0.02f}, false});
        spots.push_back({kSnsECT, Pt{L.hx1 + 0.022f, L.vHd0 + 0.06f}, false});
        spots.push_back({kSnsKnock, Pt{L.hx1 + 0.022f, vMid + 0.02f}, false});
        spots.push_back({kSnsOilT, Pt{L.hx1 + 0.022f, vMid + 0.06f}, false});
        spots.push_back({kSnsOilP, Pt{L.hx1 + 0.022f, vMid + 0.10f}, false});
        spots.push_back({kSnsCKP, Pt{L.hx0 - 0.03f, L.vHd1 - 0.01f}, false, true});
        if (!L.diesel) spots.push_back({kSnsFuelP, Pt{L.hx1 + 0.022f, L.vRail}, false});
        if (L.cr) spots.push_back({kSnsRail, Pt{(L.hx0 + L.hx1) * 0.5f, L.vRail - 0.03f}, false});
        spots.push_back({kSnsFlex, Pt{L.hx0 - 0.035f, L.vRail + 0.035f}, false});
        const bool wbPost = tel_.widebandPost;
        const Pt wb = wbPost ? Pt{L.uWg + 0.05f, L.vTail - 0.045f} : L.two ? Pt{L.hx0 + 0.03f, L.vCol + 0.05f} : Pt{uColMid, L.vCol + 0.05f};
        spots.push_back({kSnsWideband, wb, true});
        spots.push_back({kSnsO2, Pt{wb.u, wb.v + (wbPost ? 0.09f : 0.045f)}, false});
        spots.push_back({kSnsEGT, Pt{L.hx0 - 0.04f, L.vCol - 0.045f}, true, true});
        if (tel_.emapFitted || L.turbo) spots.push_back({kSnsEMAP, Pt{L.turbo ? L.uT + 0.045f : uColMid - 0.06f, L.vCol - 0.045f}, tel_.emapFitted});
        if (p.egtPostTurbo && L.turbo) spots.push_back({kSnsEGT2, Pt{L.uT - 0.035f, (L.vCol + L.vTail) * 0.5f + 0.03f}, true, true});
    }
    int hoverSensor = -1;
    for (const SensorSpot& s : spots) {
        if (!s.always && !fmDiag_) continue;
        const SensorDef& d = sensorDef(s.id);
        const bool fault = tel_.snsFallback[s.id] || p.sensors[s.id].fault != SensorFault::None;
        const ImVec2 c = P(s.at);
        const float r = em * 0.38f;
        ImU32 col = fault ? (blink ? levelCol(kHealthLimit) : levelCol(kHealthLimit, 0.25f)) : kAccent;
        ImVec2 dia[4] = {ImVec2(c.x, c.y - r), ImVec2(c.x + r, c.y), ImVec2(c.x, c.y + r), ImVec2(c.x - r, c.y)};
        dl->AddConvexPolyFilled(dia, 4, col);
        dl->AddPolyline(dia, 4, rgba(0.05f, 0.05f, 0.06f), ImDrawFlags_Closed, 1.0f);
        if (canvasHover && std::fabs(mouse.x - c.x) < r * 1.6f && std::fabs(mouse.y - c.y) < r * 1.6f) hoverSensor = s.id;
        if (fmLabels_ || fmDiag_) {
            char b[48];
            double v = tel_.snsValue[s.id];
            if (std::fabs(v) < (d.hi - d.lo < 5 ? 0.005 : 0.5)) v = 0;   // no "-0" from the noise around zero
            if (s.id == kSnsWideband) std::snprintf(b, sizeof(b), "%.2f", v);
            else if (d.hi - d.lo < 5) std::snprintf(b, sizeof(b), "%.2f %s", v, d.unit);
            else std::snprintf(b, sizeof(b), "%.0f %s", v, d.unit);
            char nm[16]; std::snprintf(nm, sizeof(nm), "%.*s", (int)std::strcspn(d.name, " "), d.name);
            if (s.id == kSnsEGT2) std::snprintf(nm, sizeof(nm), "EGT2");
            if (s.id == kSnsWideband) std::snprintf(nm, sizeof(nm), "lambda");
            char lb[72]; std::snprintf(lb, sizeof(lb), "%s %s", nm, b);
            const bool right = !s.left && s.at.u < 0.9f;
            const float lw = ImGui::GetFont()->CalcTextSizeA(small * 0.9f, FLT_MAX, 0, lb).x;
            const ImVec2 tp(right ? c.x + r + 4 : c.x - r - 4 - lw, c.y - small * 0.45f);
            plate(tp, ImVec2(tp.x + lw, tp.y + small * 0.9f));
            dl->AddText(ImGui::GetFont(), small * 0.9f, tp, fault ? levelCol(kHealthLimit) : rgba(1.0f, 0.78f, 0.5f), lb);
        }
    }
    // per-cylinder EGT on the primaries
    if (tel_.egtPerCyl) {
        const float cw = (P(Pt{L.hx1, 0}).x - P(Pt{L.hx0, 0}).x) / L.n;
        for (int z = 0; z < L.n; ++z) {
            const ImVec2 c = P(Pt{L.cylU[z], (L.vHd1 + L.vCol) * 0.5f + 0.01f});
            dl->AddCircleFilled(c, em * 0.28f, tempCol(tel_.egtCylC[z] + 273.15), 10);
            if (cw > em * 2.6f && fmLabels_) {
                char b[16]; std::snprintf(b, sizeof(b), "%.0f", tel_.egtCylC[z]);
                dl->AddText(ImGui::GetFont(), small * 0.85f, ImVec2(c.x + em * 0.35f, c.y - small * 0.45f), rgba(1.0f, 0.78f, 0.5f), b);
            }
        }
    }
    if (!tel_.running) textAt(dl, ImVec2(o.x + W * 0.5f, o.y + H * 0.5f), kDim, tel_.rpm > 50 ? "cranking" : "engine stopped - no flow", em);
    dl->PopClipRect();
    dl->AddRect(o, ImVec2(o.x + W, o.y + H), rgba(0.20f, 0.21f, 0.24f), 6, 0, 1.0f);   // the canvas frame

    // ---- hover: sensor > component > pipe
    int hoverPipe = -1;
    if (canvasHover && hoverSensor < 0 && hoverComp < 0) {
        float best = 1e9f;
        for (int i = 0; i < np; ++i) {
            const FmPipe& q = L.pipes[i];
            float d = distToSeg(mouse, P(q.a), P(q.b));
            if (q.legs > 1) d = std::min(d, distToSeg(mouse, P(q.b), P(q.c)));
            if (d < pipeTh[i] * 0.5f + 4 && d < best) { best = d; hoverPipe = i; }
        }
    }
    auto flowLine = [&](const char* what, int seg, double share = 1.0) {
        ImGui::Text("%s: %.1f g/s forward", what, segF(seg) * share * 1e3);
        if (segB(seg) > 1e-5) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.7f, 0.4f, 1), " %.1f g/s backwards", segB(seg) * share * 1e3); }
    };
    if (hoverSensor >= 0) {
        const SensorDef& d = sensorDef(hoverSensor);
        static const char* st[] = {"OK", "signal too low", "signal too high", "no signal"};
        ImGui::BeginTooltip();
        ImGui::TextColored(ImVec4(1.0f, 0.68f, 0.32f, 1), "%s", d.name);
        ImGui::Text("ECU reads %.4g %s   (truth %.4g)", tel_.snsValue[hoverSensor], d.unit, tel_.snsTruth[hoverSensor]);
        ImGui::Text("Status: %s%s", st[clampv(tel_.snsStatus[hoverSensor], 0, 3)], tel_.snsFallback[hoverSensor] ? " - ECU uses its fallback" : "");
        if (p.sensors[hoverSensor].fault != SensorFault::None) ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "Fault injected in Sensor Configuration");
        ImGui::TextDisabled("Used for: %s", d.role);
        ImGui::TextDisabled("Click: Sensor Configuration");
        ImGui::EndTooltip();
        if (clicked) jumpTo(kSensorConfig, nullptr);
    } else if (hoverComp >= 0) {
        const FmComp& k = L.comps[hoverComp];
        ImGui::BeginTooltip();
        ImGui::TextColored(ImVec4(1.0f, 0.68f, 0.32f, 1), "%s", k.name);
        switch (k.kind) {
        case CkFilter: ImGui::Text("Ambient %.1f kPa, %.0f C", f.pAmb, f.tAmb - 273.15); flowLine("Air", L.boosted ? kFlowInlet : (L.itb ? kFlowThrottle : kFlowInlet)); break;
        case CkCompressor: case CkBlower:
            ImGui::Text("Speed %.0f rpm (%.0f %% of top)   pressure ratio %.2f", tel_.shaftRpm, tel_.speedFrac * 100, tel_.pr);
            ImGui::Text("Efficiency %.0f %%   corrected flow %.0f g/s   power %.1f kW", tel_.compEff * 100, tel_.compFlowCorr, tel_.fiPowerKw);
            ImGui::Text("Outlet %.0f C, %.0f kPa", f.tCompOut - 273.15, f.pCharge);
            if (L.turbo || p.scType == Supercharger::Centrifugal) ImGui::Text("Surge margin %.0f %%%s", tel_.surgeMargin * 100, tel_.surging ? "  SURGE" : "");
            flowLine("Flow", kFlowInlet);
            break;
        case CkIntercooler:
            ImGui::Text("In %.0f C -> out %.0f C   core %.0f C", f.tCompOut - 273.15, f.tCharge - 273.15, tel_.icCoreC);
            ImGui::Text("Cooling air %.0f km/h", tel_.coolAirKmh);
            break;
        case CkBov: ImGui::Text("%s", tel_.bovOpen ? "Open: venting the charge" : "Closed"); flowLine("Vent", kFlowBov); break;
        case CkThrottle: case CkCarb:
            ImGui::Text("Position %.1f %%   %s", tel_.throttle * 100, p.comp.throttleType == ThrottleType::DriveByWire ? "drive-by-wire" : "cable");
            if (tel_.throttleTarget >= 0) ImGui::Text("ECU target %.1f %%", tel_.throttleTarget * 100);
            flowLine("Through the throttle", kFlowThrottle);
            if (f.back[kFlowThrottle] > 1e-4) ImGui::TextDisabled("Backwards = gas pushed out in front of the throttle (standoff)");
            break;
        case CkPlenum: case CkAirbox:
            ImGui::Text("%.1f kPa, %.0f C", f.pPlenum, f.tPlenum - 273.15);
            ImGui::Text("Ports %.1f kPa, %.0f C", f.pPortIn, f.tPortIn - 273.15);
            break;
        case CkHead:
            flowLine("Intake valves", kFlowIntakeValve);
            if (f.back[kFlowIntakeValve] > 1e-4) ImGui::TextDisabled("Backwards = reversion into the ports");
            flowLine("Exhaust valves", kFlowExhaustValve);
            ImGui::Text("Advance %.1f deg   VE %.0f %%   knock index %.2f", tel_.advance, tel_.ve * 100, tel_.knock);
            for (int z = 0; z < L.n; ++z)
                ImGui::TextColored(levelRgb(h.cyl[z]), "%s%d  lambda %.2f  retard %.1f  pmax %.0f bar  IMEP %.1f bar", L.rotary ? "R" : "#", z + 1,
                                   tel_.cylLambda[z], tel_.knockRetard[z], tel_.cylPmax[z], tel_.cylImep[z]);
            ImGui::Separator();
            ImGui::TextColored(levelRgb(h.part[kPartPlugs].level), "%s: %s", healthItemName(kHealthPlugs, p), h.part[kPartPlugs].value);
            ImGui::TextUnformatted(h.part[kPartPlugs].detail);
            break;
        case CkHeader:
            ImGui::Text("Gas leaving the cylinders %.0f C, after the ports %.0f C", f.tCyl - 273.15, f.tCollector - 273.15);
            ImGui::Text("EGT thermocouple %.0f C   exhaust ports %.1f kPa", tel_.egt, f.pPortEx);
            if (tel_.egtPerCyl) for (int z = 0; z < L.n; ++z) ImGui::Text("  %s%d  %.0f C", L.rotary ? "R" : "#", z + 1, tel_.egtCylC[z]);
            break;
        case CkTurbine: case CkWastegate:
            ImGui::Text("Inlet %.1f kPa (EMAP), outlet %.1f kPa: expansion ratio %.2f", f.pTurbIn, f.pTurbOut, tel_.turbEr);
            ImGui::Text("Drive pressure %.2f (EMAP / MAP)", f.pTurbIn / std::max(10.0, f.pPlenum));
            ImGui::Text("Efficiency %.0f %%   U/C %.2f   reduced flow %.2f", tel_.turbEff * 100, tel_.turbUc, tel_.turbMfp);
            ImGui::Text("Gas %.0f C -> %.0f C   wastegate %.0f %% open", f.tCollector - 273.15, f.tPostTurb - 273.15, tel_.wastegate * 100);
            flowLine("Turbine", kFlowTurbine);
            flowLine("Wastegate", kFlowWastegate);
            break;
        case CkCat: case CkMuffler: case CkTail:
            ImGui::Text("Gas %.0f C", f.tPostTurb - 273.15);
            flowLine("Flow", kFlowTail);
            if (L.diesel) ImGui::Text("Engine-out NOx %.0f ppm (%.2f g/s)   soot %.1f mg/m3 = FSN %.2f", tel_.noxPpm, tel_.noxGs, tel_.sootMgM3, tel_.smokeFsn);
            break;
        case CkEgrValve: case CkEgrCooler:
            ImGui::Text("Valve %.0f %% open   %.1f g/s   %.0f %% of the charge", tel_.egrPos * 100, tel_.egrGs, tel_.egrRate * 100);
            ImGui::Text("Gas from the manifold %.0f C -> into the plenum %.0f C%s", f.tCollector - 273.15, tel_.egrGasC, L.egrCooler ? " (cooled)" : " (no cooler)");
            ImGui::Text("Engine-out NOx %.0f ppm, smoke FSN %.2f", tel_.noxPpm, tel_.smokeFsn);
            ImGui::TextDisabled("Opening: ECU > Engine Control > Diesel > EGR map");
            break;
        case CkInjPump:
            if (L.cr) ImGui::Text("Rail %.0f bar (target %.0f)   pump %.1f kW", tel_.railBar, tel_.railTargetBar, tel_.pumpKw);
            ImGui::Text("Main %.1f mg at %.1f deg BTDC   pilot %.1f mg%s", tel_.dieselFuelMg, tel_.soi, tel_.pilotMg, tel_.smokeLimited ? "   SMOKE LIMITED" : "");
            if (L.cr) ImGui::Text("Second pilot %.1f mg   post %.1f mg", tel_.pilot2Mg, tel_.postMg);
            ImGui::Text("Ignition delay %.1f deg", tel_.ignDelayDeg);
            break;
        case CkReed: case CkCrankcase:
            ImGui::Text("Crankcase %.0f kPa   reed %.0f %% open", tel_.tsCrankcaseKPa, tel_.tsReed * 100);
            flowLine("Through the reed valve", kFlowThrottle);
            flowLine("Up the transfer ducts", kFlowIntakeValve);
            ImGui::Text("Delivery ratio %.2f   trapping %.2f   scavenging %.2f", tel_.tsDelivery, tel_.tsTrapping, tel_.tsScavenging);
            break;
        case CkChamber:
            ImGui::Text("Tuned for %.0f rpm: the returning pressure wave plugs the exhaust port as it closes", p.tsPipeRpm);
            ImGui::Text("Gas %.0f C", f.tCollector - 273.15);
            flowLine("Out of the exhaust port", kFlowExhaustValve);
            break;
        default: break;
        }
        if (k.part >= 0 && h.part[k.part].detail[0] && k.kind != CkHead) { ImGui::Separator(); ImGui::TextUnformatted(h.part[k.part].detail); }
        else if (k.kind == CkHead && h.part[kPartHead].detail[0]) { ImGui::Separator(); ImGui::TextUnformatted(h.part[kPartHead].detail); }
        ImGui::TextDisabled("Click: open its settings");
        ImGui::EndTooltip();
        if (clicked) jumpTo(k.section, k.tab);
    } else if (hoverPipe >= 0) {
        const FmPipe& q = L.pipes[hoverPipe];
        double T = nodeT(f, q.tid);
        if (q.cyl >= 0 && q.seg == kFlowExhaustValve && tel_.egtPerCyl) T = tel_.egtCylC[q.cyl] + 273.15;
        ImGui::BeginTooltip();
        if (q.cyl >= 0) ImGui::TextColored(ImVec4(1.0f, 0.68f, 0.32f, 1), "%s %s%d", q.name, L.rotary ? "R" : "#", q.cyl + 1);
        else ImGui::TextColored(ImVec4(1.0f, 0.68f, 0.32f, 1), "%s", q.name);
        ImGui::Text("%.1f kPa   %.0f C", nodeP(f, q.pid), T - 273.15);
        ImGui::Text("Gas speed %.0f m/s forward", pipeVel(q, segF(q.seg) * q.share));
        if (segB(q.seg) > 1e-5) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.7f, 0.4f, 1), " %.0f m/s backwards", pipeVel(q, segB(q.seg) * q.share)); }
        flowLine(q.seg == kSegEgr ? "EGR" : L.two && q.seg == kFlowThrottle ? "reed valve" : L.two && q.seg == kFlowIntakeValve ? "transfer ports" : kSegNames[q.seg], q.seg, q.share);
        ImGui::EndTooltip();
    }

    // ---- sensors off the gas path (diagnosis)
    if (fmDiag_) {
        static const int off[] = {kSnsVSS, kSnsGear, kSnsBattery};
        ImGui::TextDisabled("Not on the gas path:");
        for (int id : off) {
            ImGui::SameLine();
            const bool fault = tel_.snsFallback[id] || p.sensors[id].fault != SensorFault::None;
            ImGui::TextColored(fault ? (blink ? levelRgb(kHealthLimit) : ImVec4(0.5f, 0.2f, 0.2f, 1)) : ImVec4(0.6f, 0.6f, 0.65f, 1), "%s %.4g %s",
                               sensorDef(id).name, tel_.snsValue[id], sensorDef(id).unit);
        }
    }

    // ---- health panel
    healthPanel(h);
}

// one row of tiles: icon + name + reading, coloured by level; click -> the panel that deals with it
void App::healthPanel(const HealthReport& h) {
    const float em = ImGui::GetFontSize();
    const int n = kHealthItemCount;
    const float sp = ImGui::GetStyle().ItemSpacing.x;
    const float avail = ImGui::GetContentRegionAvail().x;
    const int maxRow = std::max(3, std::min(n, (int)((avail + sp) / (em * 8.5f + sp))));
    const int rows = (n + maxRow - 1) / maxRow, perRow = (n + rows - 1) / rows;   // balanced rows
    const float tw = (avail - (perRow - 1) * sp) / perRow, th = em * 3.2f;
    struct Dest { int section; const char* tab; };
    const Dest dest[kHealthItemCount] = {{kEngineProtection, nullptr}, {kIgnitionTuning, "knock"}, {kEngineSetup, "components"},
                                         {kSensorConfig, nullptr}, {kEngineSetup, "valv"}, {kEngineSetup, "components"},
                                         {kEngineSetup, "components"}, {kEngineSetup, "components"}, {kEngineSetup, "components"}};
    const bool blink = std::fmod(ImGui::GetTime(), 0.7) < 0.4;
    for (int i = 0; i < n; ++i) {
        const HealthEntry& e = h.item[i];
        if (i % perRow) ImGui::SameLine();
        ImGui::PushID(i);
        const ImVec2 a = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##tile", ImVec2(tw, th));
        const bool hov = ImGui::IsItemHovered();
        const ImVec2 b(a.x + tw, a.y + th);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const int lv = e.level;
        const bool alarm = lv == kHealthLimit;
        dl->AddRectFilled(ImVec2(a.x + 1, a.y + 2), ImVec2(b.x + 2, b.y + 3), rgba(0, 0, 0, 0.35f), 4);
        dl->AddRectFilledMultiColor(a, b, rgba(0.125f, 0.128f, 0.142f), rgba(0.125f, 0.128f, 0.142f), rgba(0.085f, 0.087f, 0.098f),
                                    rgba(0.085f, 0.087f, 0.098f));
        if (alarm) dl->AddRectFilled(a, b, levelCol(lv, blink ? 0.30f : 0.16f), 4);
        else if (lv >= kHealthOk) dl->AddRectFilledMultiColor(a, ImVec2(a.x + (b.x - a.x) * 0.5f, b.y), levelCol(lv, 0.07f), rgba(0, 0, 0, 0), rgba(0, 0, 0, 0), levelCol(lv, 0.07f));
        dl->AddLine(ImVec2(a.x + 4, a.y + 1), ImVec2(b.x - 4, a.y + 1), rgba(1, 1, 1, 0.07f), 1.0f);
        dl->AddRect(a, b, hov ? rgba(1.0f, 0.62f, 0.25f) : rgba(0.22f, 0.23f, 0.26f), 4, 0, hov ? 2.0f : 1.0f);
        dl->AddRectFilled(a, ImVec2(a.x + 4, b.y), levelCol(lv), 3, ImDrawFlags_RoundCornersLeft);
        // icon
        const float s = em * 1.25f;
        const ImVec2 ic(a.x + 12 + s * 0.5f, a.y + th * 0.5f);
        const ImU32 c = levelCol(lv);
        dl->AddCircleFilled(ic, s * 0.72f, levelCol(lv, 0.12f), 24);
        dl->AddCircle(ic, s * 0.72f, levelCol(lv, 0.35f), 24, 1.0f);
        switch (i) {
        case kHealthCel:   // engine outline
            dl->AddRect(ImVec2(ic.x - s * 0.45f, ic.y - s * 0.3f), ImVec2(ic.x + s * 0.35f, ic.y + s * 0.35f), c, 2, 0, 1.6f);
            dl->AddLine(ImVec2(ic.x - s * 0.2f, ic.y - s * 0.3f), ImVec2(ic.x - s * 0.2f, ic.y - s * 0.5f), c, 1.6f);
            dl->AddLine(ImVec2(ic.x - s * 0.35f, ic.y - s * 0.5f), ImVec2(ic.x + s * 0.0f, ic.y - s * 0.5f), c, 1.6f);
            dl->AddLine(ImVec2(ic.x + s * 0.35f, ic.y), ImVec2(ic.x + s * 0.5f, ic.y - s * 0.12f), c, 1.6f);
            break;
        case kHealthKnock: {   // spike
            ImVec2 z[6] = {ImVec2(ic.x - s * 0.5f, ic.y), ImVec2(ic.x - s * 0.25f, ic.y), ImVec2(ic.x - s * 0.1f, ic.y - s * 0.45f),
                           ImVec2(ic.x + s * 0.1f, ic.y + s * 0.45f), ImVec2(ic.x + s * 0.25f, ic.y), ImVec2(ic.x + s * 0.5f, ic.y)};
            dl->AddPolyline(z, 6, c, 0, 1.8f);
            break;
        }
        case kHealthPlugs:     // spark plug
            dl->AddRect(ImVec2(ic.x - s * 0.15f, ic.y - s * 0.5f), ImVec2(ic.x + s * 0.15f, ic.y - s * 0.05f), c, 1, 0, 1.6f);
            dl->AddRectFilled(ImVec2(ic.x - s * 0.25f, ic.y - s * 0.05f), ImVec2(ic.x + s * 0.25f, ic.y + s * 0.15f), c, 1);
            dl->AddLine(ImVec2(ic.x, ic.y + s * 0.15f), ImVec2(ic.x, ic.y + s * 0.45f), c, 1.8f);
            dl->AddLine(ImVec2(ic.x, ic.y + s * 0.45f), ImVec2(ic.x + s * 0.2f, ic.y + s * 0.45f), c, 1.8f);
            break;
        case kHealthSensors:   // probe with a signal
            dl->AddCircle(ic, s * 0.42f, c, 16, 1.6f);
            dl->AddBezierCubic(ImVec2(ic.x - s * 0.3f, ic.y), ImVec2(ic.x - s * 0.1f, ic.y - s * 0.4f), ImVec2(ic.x + s * 0.1f, ic.y + s * 0.4f),
                               ImVec2(ic.x + s * 0.3f, ic.y), c, 1.6f);
            break;
        case kHealthValves:    // poppet valve
            dl->AddLine(ImVec2(ic.x, ic.y - s * 0.5f), ImVec2(ic.x, ic.y + s * 0.3f), c, 1.8f);
            dl->AddTriangleFilled(ImVec2(ic.x, ic.y + s * 0.15f), ImVec2(ic.x - s * 0.4f, ic.y + s * 0.45f), ImVec2(ic.x + s * 0.4f, ic.y + s * 0.45f), c);
            break;
        case kHealthRods:      // connecting rod
            dl->AddCircle(ImVec2(ic.x, ic.y - s * 0.32f), s * 0.14f, c, 12, 1.6f);
            dl->AddCircle(ImVec2(ic.x, ic.y + s * 0.28f), s * 0.22f, c, 14, 1.6f);
            dl->AddLine(ImVec2(ic.x, ic.y - s * 0.18f), ImVec2(ic.x, ic.y + s * 0.06f), c, 2.6f);
            break;
        case kHealthCrank: {   // crank throw
            ImVec2 z[6] = {ImVec2(ic.x - s * 0.5f, ic.y), ImVec2(ic.x - s * 0.25f, ic.y), ImVec2(ic.x - s * 0.25f, ic.y - s * 0.35f),
                           ImVec2(ic.x + s * 0.25f, ic.y - s * 0.35f), ImVec2(ic.x + s * 0.25f, ic.y), ImVec2(ic.x + s * 0.5f, ic.y)};
            dl->AddPolyline(z, 6, c, 0, 2.2f);
            break;
        }
        case kHealthPistons:   // piston with rings
            dl->AddRect(ImVec2(ic.x - s * 0.35f, ic.y - s * 0.4f), ImVec2(ic.x + s * 0.35f, ic.y + s * 0.3f), c, 2, 0, 1.6f);
            dl->AddLine(ImVec2(ic.x - s * 0.35f, ic.y - s * 0.2f), ImVec2(ic.x + s * 0.35f, ic.y - s * 0.2f), c, 1.2f);
            dl->AddLine(ImVec2(ic.x - s * 0.35f, ic.y - s * 0.07f), ImVec2(ic.x + s * 0.35f, ic.y - s * 0.07f), c, 1.2f);
            break;
        default:               // wrench
            dl->AddLine(ImVec2(ic.x - s * 0.4f, ic.y + s * 0.4f), ImVec2(ic.x + s * 0.15f, ic.y - s * 0.15f), c, 2.4f);
            dl->AddCircle(ImVec2(ic.x + s * 0.25f, ic.y - s * 0.25f), s * 0.2f, c, 12, 1.8f);
            break;
        }
        const float tx = a.x + 20 + s;
        dl->PushClipRect(a, ImVec2(b.x - 3, b.y), true);
        dl->AddText(ImGui::GetFont(), em * 0.85f, ImVec2(tx, a.y + th * 0.5f - em * 0.95f), rgba(0.62f, 0.63f, 0.68f), healthItemName(i, p_));
        dl->AddText(ImVec2(tx, a.y + th * 0.5f + em * 0.05f), lv >= kHealthOk ? levelCol(lv) : rgba(0.62f, 0.63f, 0.68f), e.value);
        dl->PopClipRect();
        const bool canRepair = i == kHealthRepair && (tel_.failure || tel_.damage > 0.001f);
        if (hov) {
            ImGui::BeginTooltip();
            ImGui::TextColored(levelRgb(lv), "%s: %s", healthItemName(i, p_), e.value);
            if (e.detail[0]) ImGui::TextUnformatted(e.detail);
            ImGui::TextDisabled(canRepair ? "Click: repair the engine (damage back to 0)" : "Click: open the related panel");
            ImGui::EndTooltip();
        }
        if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            if (canRepair) p_.comp.repairCount++;
            else jumpTo(dest[i].section, dest[i].tab);
        }
        ImGui::PopID();
    }
}

} // namespace es
