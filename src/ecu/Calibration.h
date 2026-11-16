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

// engine protection (Phase 3): one rule per monitored condition
enum ProtId : int { kProtOverRev, kProtOverboost, kProtCoolant, kProtOilTemp, kProtOilPress, kProtFuelPress, kProtLean,
                    kProtKnock, kProtEgt, kProtSensor, kProtCount };
enum ProtAction : int { kActWarn, kActRetard, kActBoostReduce, kActThrottleLimit, kActFuelLimit, kActLimp, kActShutdown, kActCount };
struct ProtRule {
    int32_t enabled;
    float warn, limit;   // thresholds in the condition's unit (high or low depending on the condition)
    float delay;         // s beyond the limit before the action engages
    float recover;       // hysteresis: the value must come back this far past the limit to clear
    int32_t action;      // ProtAction
};

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

    // ---- closed-loop lambda (STFT / LTFT)
    float stftGain;      // per s (fraction of the lambda error integrated per second)
    float stftLimit;     // +- %
    int32_t ltftLearn;   // move the short-term trim into the long-term table
    float ltftRate;      // fraction of the STFT moved into the LTFT cell per second
    float ltftLimit;     // +- %
    float clMinEct;      // C: closed loop / learning only above this coolant temperature
    float clDelay;       // s after start / a transient before closed loop resumes
    float clMaxMap;      // kPa: above this the engine runs open loop (power)
    Table2D ltft;        // learned long-term trim % over rpm x MAP

    // ---- knock control
    int32_t knockControl;
    float knockThreshold;   // knock sensor index that counts as a knock event
    float knockRetardStep;  // deg per knock event (per cylinder, via the crank-angle window)
    float knockMaxRetard;   // deg
    float knockRecover;     // deg per second back towards the map
    float knockMinRpm;

    // ---- idle / throttle / limiter
    Curve1D idleTarget;  // rpm over coolant C
    float idleKp, idleKi, idleKd;          // idle valve PID (fraction per rpm of error / rpm s / rpm per s)
    float idleSparkGain, idleSparkMin, idleSparkMax;   // idle spark control deg per rpm of error, limits
    Curve1D dbwMap;      // drive-by-wire: throttle % over pedal %
    float softLimitBand; // rpm below the hard limiter where the soft limiter starts (0 = off)
    float softLimitRetard;  // deg of retard at the hard limit
    float softLimitCut;     // fraction of sparks cut at the hard limit (0..1)

    // ---- boost control (turbo)
    int32_t boostControl;   // 0: wastegate spring only, 1: ECU closed loop on a solenoid
    Table2D boostTarget;    // bar gauge over rpm x TPS
    Curve1D boostGear;      // % of the target over gear (0 = neutral)
    float boostKp, boostKi, boostKd;   // duty per bar of error / bar s / bar per s
    float wgDutyMax;        // 0..1
    float overboostMargin;  // bar above target ...
    float overboostTime;    // ... for this long -> fuel cut until back at target
    float boostCutBar;      // bar gauge: instant fuel cut above this

    // ---- protection
    ProtRule prot[kProtCount];
    float limpThrottle;     // % max throttle in limp mode / throttle limitation
    float limpRpm;          // rpm cap in limp mode / fuel limitation
    float protRetard;       // deg of retard for the retard action / limp mode
};

// table identities (UI / files)
enum CalTableId : int { kCalVeMap, kCalVeTps, kCalLambda, kCalIgnition, kCalLtft, kCalBoostTarget, kCalTableCount };
enum CalCurveId : int { kCalCrankAfr, kCalWarmup, kCalAccel, kCalIatFuel, kCalIatSpark, kCalEctSpark, kCalDeadTime,
                        kCalIdleTarget, kCalDbwMap, kCalBoostGear, kCalCurveCount };
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

// protection metadata (UI / files / validation)
struct ProtInfo { const char* key; const char* name; const char* unit; bool high; float lo, hi; };
const ProtInfo& protInfo(int id);
const char* protActionName(int a);

} // namespace es
