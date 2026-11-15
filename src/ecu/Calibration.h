#pragma once
// ECU calibration: editable maps (2D tables over rpm x load) and curves (1D), plus scalar settings.
// Plain data (POD) so it can travel inside EngineParams to the simulation thread.
#include <cstdint>

namespace es {

constexpr int kTabMax = 16;      // max breakpoints per table axis
constexpr int kCurveMax = 10;    // max breakpoints of a 1D curve
constexpr int kCalMaxCyl = 16;

struct Table2D {
    int32_t nx, ny;              // columns (x = rpm), rows (y = load)
    float x[kTabMax], y[kTabMax];
    float v[kTabMax][kTabMax];   // v[row = y][col = x]
    int32_t locked;              // locked: no edits, no auto-tune
};

struct Curve1D {
    int32_t n;
    float x[kCurveMax];
    float v[kCurveMax];
    int32_t locked;
};

enum class LoadMode : int32_t { SpeedDensity = 0, AlphaN = 1, Maf = 2, Hybrid = 3 };

struct Calibration {
    LoadMode loadMode;
    float hybridTpsLo, hybridTpsHi;  // hybrid: speed density below lo, alpha-N above hi (TPS %)
    Table2D veMap;       // VE % over rpm x MAP (kPa)      - speed density / hybrid
    Table2D veTps;       // VE % over rpm x TPS (%)        - alpha-N / hybrid
    Table2D lambda;      // target lambda over rpm x MAP
    Table2D ignition;    // spark advance deg BTDC over rpm x MAP
    float powerTpsLo, powerTpsHi;  // power enrichment: blend to powerLambda as the throttle opens (TPS %)
    float powerLambda;
    float crankAdvance;  // spark advance while cranking
    Curve1D crankAfr;    // cranking target AFR over coolant C
    Curve1D warmup;      // warm-up enrichment % over coolant C
    float afterStartAfr; // richest target right after start, fading out over afterStartTime
    float afterStartTime;
    Curve1D accel;       // acceleration enrichment % over throttle opening rate (%/s)
    float accelDecay;    // s
    float accelColdMult; // extra multiplier on a stone-cold engine
    float dfcoExitEnrich; // % extra fuel when injection resumes after decel fuel cut (re-wets the ports)
    float dfcoExitDecay;  // s
    int32_t asyncTipIn;   // extra (asynchronous) injection when the load jumps after a port was already fuelled
    Curve1D iatFuel;     // fuel correction % over intake air C
    Curve1D iatSpark;    // spark correction deg over intake air C
    Curve1D ectSpark;    // spark correction deg over coolant C
    Curve1D deadTime;    // injector dead time ms over battery V
    float injFlow;       // injector flow the ECU assumes (cc/min at 3 bar)
    float cylFuel[kCalMaxCyl];   // per-cylinder fuel trim %
    float cylSpark[kCalMaxCyl];  // per-cylinder spark trim deg
    int32_t autoTune;    // learn the active VE table from the wideband
    int32_t closedLoop;  // short-term fuel trim on the wideband at part load
};

// table identities (UI / files)
enum CalTableId : int { kCalVeMap, kCalVeTps, kCalLambda, kCalIgnition, kCalTableCount };
enum CalCurveId : int { kCalCrankAfr, kCalWarmup, kCalAccel, kCalIatFuel, kCalIatSpark, kCalEctSpark, kCalDeadTime, kCalCurveCount };
struct TableInfo { const char* key; const char* name; const char* xName; const char* yName; const char* unit; float lo, hi; float step; };
const TableInfo& calTableInfo(int id);
const TableInfo& calCurveInfo(int id);
Table2D& calTable(Calibration& c, int id);
const Table2D& calTable(const Calibration& c, int id);
Curve1D& calCurve(Calibration& c, int id);
const Curve1D& calCurve(const Calibration& c, int id);

// interpolation (axes clamped at the ends)
struct CellWeights { int ix0, ix1, iy0, iy1; float wx, wy; };   // bilinear: (1-wx)(1-wy) at (iy0, ix0) ...
CellWeights tableWeights(const Table2D& t, double x, double y);
double tableLookup(const Table2D& t, double x, double y);
double curveLookup(const Curve1D& c, double x);

} // namespace es
