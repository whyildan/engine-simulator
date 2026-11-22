#pragma once
#include <tuple>
// Shared pieces of the parametric engine model (EngineModel.cpp builds the core; EngineModelParts.cpp the intake,
// exhaust, forced induction, two-stroke and rotary parts).
#include "ui/EngineModel.h"
#include "ui/EngineView3D.h"
#include <functional>
#include <string>
#include <vector>

namespace es::ui::model {

using m3::V3;
using m3::M4;

struct Cached { int id = -1; V3 lo, hi; };
const Cached& meshOf(const std::string& key, const std::function<mesh::Data()>& make);
std::string key(const char* name, std::initializer_list<float> v);
void cacheTrim();   // drop every cached shape when there are too many (call between frames)

struct Scene {
    EngineModel* out;
    bool fit = true;   // parts placed while false still draw, but the camera does not frame them
    void put(const Cached& c, const M4& w, const gpu::Material& m);
    void glow(V3 p, float s, float r, float g, float b, float a) { out->glows.push_back({p, s, r, g, b, a}); }
    void label(V3 p, const std::string& t, bool accent = false) { out->labels.push_back({p, t, accent}); }
    void fitPoint(V3 q) { out->hull.push_back(q); }
};

// palette
gpu::Material mat(float r, float g, float b, float metal, float rough);
gpu::Material glassMat(float a, float r = 0.70f, float g = 0.74f, float b = 0.80f);
gpu::Material emissive(gpu::Material m, float r, float g, float b);
extern const gpu::Material kSteel, kSilver, kSteelDark, kFrame, kGold, kRubber, kCeramic, kAlu, kHotPipe;
extern const float kAccent[3], kCharge[3];

// smooth path through control points (Catmull-Rom), `per` points per span
std::vector<V3> spline(const std::vector<V3>& ctrl, int per = 10);
float rnd(uint32_t a, uint32_t b);

// where the gas leaves / enters each cylinder head (world), for the manifolds
struct PortOut { V3 p, d; int cyl, bank; };
struct EngineBits {
    std::vector<PortOut> ex, in;
    float zFront = 0, zRear = 0, B = 80, r = 40;
    bool vee = false, boxer = false;
    V3 crankFront;              // the crank's front end (accessory belt pulley)
    float headTop = 0;          // top of the heads / valvetrain (piston engines): lines and wires route over it
    V3 plenum;                  // intake plenum centre (set by buildIntake) and its length along the crank
    float plenumLen = 0;
    // diesel: the injection pump's pulley (front plane), the injectors' and glow plugs' tops
    V3 pump; bool hasPump = false;
    std::vector<V3> injTop, glowTop;
};

// intake runners / plenum / throttle (returns the throttle inlet: where boost piping ends)
// a belt round pulleys (x, y, r) in the plane z; force: +1 wrap outside, -1 back side (an idler pressing on it), 0 auto
std::vector<V3> serpentine(const std::vector<std::tuple<float, float, float>>& pulleys, float z, const std::vector<int>& force);
V3 buildIntake(Scene& sc, const EngineParams& p, EngineBits& eb, V3& inletDir);
// headers, collector(s) and tail pipe(s); with a turbo the first collector feeds the turbine instead
void buildExhaust(Scene& sc, const EngineParams& p, const EngineBits& eb, const EngineViewLive& live, bool turbo, V3& turbineIn);
// diesel extras: injection pump and its high-pressure lines, the glow plugs' bus wire, the EGR valve and pipe
void buildDiesel(Scene& sc, const EngineParams& p, const EngineBits& eb, const EngineViewLive& live);
// turbo / supercharger / intercooler and their piping
void buildBoost(Scene& sc, const EngineParams& p, const EngineBits& eb, const EngineViewLive& live, float crankDeg, V3 turbineIn,
                V3 throttleIn, V3 throttleDir);
// a two-stroke cylinder's own parts (ports, transfers, crankcase, reed, head) and its expansion chamber
void buildTwoStrokeCyl(Scene& sc, const EngineParams& p, const M4& cf, double phi, float crankDeg, const EngineViewLive& live, float glassA,
                       float deck, float crownTop, int idx);
// the rotary engine; fills `eb` (sizes, front, plenum) and the turbine / throttle ends for buildBoost (with a turbo the
// exhaust stops at the collector: the turbo's downpipe carries on)
void buildRotaryEngine(Scene& sc, const EngineParams& p, const float phases[kMaxCyl], float shaftDeg, const EngineViewLive& live, float glassA,
                       bool turbo, EngineBits& eb, V3& turbineIn, V3& throttleIn, V3& throttleDir);

} // namespace es::ui::model
