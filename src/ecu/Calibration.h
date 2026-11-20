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

// trigger wheels (Phase 7): crank teeth with missing-tooth gaps; the ECU syncs on a gap (or on the cam tooth)
enum TriggerPattern : int { kTrig60_2, kTrig36_1, kTrig36_2_2_2, kTrig24_1, kTrig12_1, kTrig4Cam, kTrigCount };
struct TriggerInfo { const char* name; int teeth; int gaps; bool camRef; };   // camRef: sync only from the cam tooth
const TriggerInfo& triggerInfo(int pattern);
const char* triggerNames();     // ImGui combo list
// injection timing: sequential (each port at its own angle), semi-sequential (each port twice per cycle, half the
// fuel each), batch (all injectors together twice per cycle at one crank angle)
enum InjMode : int { kInjSequential, kInjSemiSequential, kInjBatch, kInjModeCount };
const char* injModeNames();

// engine protection (Phase 3): one rule per monitored condition
enum ProtId : int { kProtOverRev, kProtOverboost, kProtCoolant, kProtOilTemp, kProtOilPress, kProtFuelPress, kProtLean,
                    kProtKnock, kProtEgt, kProtSensor, kProtBackpressure, kProtCount };
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

    // ---- cam control (changes the air the engine breathes, so it sits before the trim / control settings
    //      that the base map cache ignores)
    float vtecRpm, vtecHyst;   // high cam above vtecRpm, back to the low cam below vtecRpm - hyst
    float vtecMinMap;          // kPa: only under load (part throttle cruising stays on the low cam)
    float vtecMinEct;          // C
    Table2D vtcMap;            // intake cam advance deg over rpm x MAP

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

    // ---- motorsport (Phase 5)
    // launch control (2-step) and the clutch-in limiters while moving (3-step / flat shift)
    int32_t launchControl;
    float launchRpm;        // 2-step: rpm held at standstill with the clutch in (or the transbrake on)
    float launchTiming;     // deg BTDC while a launch / 3-step limit is armed (turbo: retarded timing builds boost)
    float launchMinTps;     // % throttle that arms the limiters
    float launchMaxSpeed;   // km/h: below = standing (2-step), above = moving (3-step / flat shift)
    float launchCutBand;    // rpm: the cut ramps from 0 to 100 % across +-band around the limit
    int32_t launchCutMode;  // 0 spark cut (unburned fuel bangs in the exhaust), 1 fuel cut
    float launchEnrich;     // % extra fuel while a launch limit is holding
    int32_t clutchMovingMode;   // clutch in while moving: 0 off, 1 3-step (fixed rpm), 2 flat shift (hold the clutch-in rpm)
    float threeStepRpm;
    float flatShiftMargin;  // rpm above the clutch-in rpm
    float flatShiftRetard;  // deg
    int32_t shiftCut;       // spark cut on the gear-change signal (sequential / auto-clutch / automatic gearbox)
    float shiftCutTime;     // s
    float shiftCutMinTps;   // %
    // anti-lag (on lift-off) and rolling anti-lag (on its button)
    float alsTiming;        // deg BTDC
    float alsThrottle;      // % throttle crack
    float alsMaxTime;       // s after lift-off
    float alsMinRpm;
    int32_t rollingAls;
    float rollingAlsMinSpeed;   // km/h
    // traction control (driven wheel speed against the vehicle speed)
    int32_t tractionControl;
    Curve1D tcSlipTarget;   // allowed slip % over vehicle speed km/h
    float tcKp, tcKi;       // torque reduction (0..1) per % slip above the target / per % s
    float tcMaxRetard;      // deg: the first 40 % of the reduction is retard ...
    float tcMaxCut;         // ... the rest cuts up to this share of the sparks (0..1)
    // nitrous controller
    float n2oMinTps, n2oMinRpm, n2oMaxRpm, n2oMinEct, n2oMinSpeed;
    float n2oDelay;         // s the conditions must hold before the solenoid opens
    float n2oRampTime;      // s from the start duty to full (progressive controller)
    float n2oStartPct;      // % duty at the start of the ramp
    float n2oRetard;        // deg at full flow
    float n2oFlowGs;        // dry kit: the N2O flow (g/s at full duty) the ECU adds fuel for
    float n2oLeanLambda;    // wideband leaner than this while spraying -> nitrous off until the throttle is lifted

    // ---- decel fuel cut resume (after Phase 5)
    float dfcoResumeAbove;  // rpm above the idle target where fuel comes back (never below 1600 rpm)
    float dfcoAnticipate;   // s: resume earlier by the rpm the engine will lose in this time (falling revs)
    float dfcoDelay;        // s the throttle must be closed before fuel is cut (a blip or a quick lift does not cut)

    // ---- ECU configuration (Phase 7): trigger, injection timing, ignition dwell
    int32_t triggerPattern; // TriggerPattern
    int32_t camSensor;      // cam phase sensor fitted: sequential injection once synced (else semi-sequential)
    int32_t injMode;        // InjMode the ECU runs once it knows the cycle phase
    Curve1D dwell;          // coil dwell ms over battery V
    float dwellMaxDuty;     // the coil may charge at most this share of the time between its sparks

    // ---- torque management and gear-based control (Phase 7)
    int32_t torqueMgmt;
    Curve1D tqLimitGear;    // torque limit over gear, % of a full naturally aspirated charge (boost: > 100)
    float tqMaxRetard;      // deg: the first 15 % of a reduction is retard (with DBW: while the throttle closes) ...
    float tqMaxCut;         // ... the rest cuts up to this share of the sparks (0..1)
    float tqThrottleRate;   // DBW: how fast the throttle cap follows the limit (per s per % of error)
    int32_t gearControl;
    Curve1D gearRevLimit;   // rpm limit over gear (above the main rev limit: the main limiter)
    Curve1D gearSpark;      // spark trim deg over gear

    // ---- fuel and flex fuel (Package 1). The ECU's picture of the fuel: its stoichiometric AFR follows the
    //      alcohol content (flex fuel sensor, or the fallback below without one / when it fails), blending the
    //      two stoich values by mass. Curves over the alcohol content add fuel / spark / cold enrichment.
    float stoichGasoline;   // AFR of the gasoline part (14.7)
    float stoichAlcohol;    // AFR of the alcohol the sensor measures (ethanol 9.0, methanol 6.45)
    float flexFallbackPct;  // alcohol % the ECU assumes without a (working) flex fuel sensor
    Curve1D flexFuel;       // fuel trim % over alcohol %
    Curve1D flexSpark;      // spark added (deg) over alcohol %
    Curve1D flexCold;       // extra cranking / warm-up fuel % when fully cold, over alcohol %
    Curve1D flexKnock;      // knock threshold multiplier over alcohol %

    // ---- diesel (Package 4): the load is the fuel quantity (no throttle)
    Table2D dieselFuel;     // driver demand: injected fuel mg/stroke over rpm x pedal %
    Table2D soi;            // start of injection deg BTDC over rpm x fuel mg/stroke
    float smokeLambda;      // smoke limiter: never more fuel than the measured air / (smokeLambda x stoich)
    float dieselIdleKp, dieselIdleKi;   // idle governor: mg per rpm of error, mg per rpm s
    float dieselGovBand;    // rpm: the max-speed governor takes the fuel to zero over this band below the rev limit
    float dieselStartFuel;  // mg/stroke while cranking
    // glow plugs: pre-heat when the key turns on (time over coolant temperature), heating while cranking, post-heat
    // after the start until the coolant reaches afterglowEct or for afterglowTime at most
    Curve1D glowPreheat;    // s over coolant C
    float afterglowTime;    // s
    float afterglowEct;     // C
    // pilot injection: a small injection before the main one - it lights first and shortens the main injection's
    // ignition delay (less premixed burn, less combustion noise)
    float pilotMg;          // mg (0 = off)
    float pilotDwellUs;     // us from the end of the pilot to the start of the main injection (from 1500 rpm)
    float pilotDwellIdleUs; // ... at idle (blended to pilotDwellUs by 1500 rpm): the pilot needs time to light first
    float pilotMaxRpm;      // rpm: no pilot above
    // exhaust gas recirculation (Package 4b): valve opening over rpm x fuel, only with a warm engine
    Table2D egrMap;         // % over rpm x fuel mg/stroke
    float egrMinEct;        // C
};

// table identities (UI / files)
enum CalTableId : int { kCalVeMap, kCalVeTps, kCalLambda, kCalIgnition, kCalLtft, kCalBoostTarget, kCalVtc, kCalDieselFuel, kCalSoi, kCalEgr, kCalTableCount };
enum CalCurveId : int { kCalCrankAfr, kCalWarmup, kCalAccel, kCalIatFuel, kCalIatSpark, kCalEctSpark, kCalDeadTime,
                        kCalIdleTarget, kCalDbwMap, kCalBoostGear, kCalTcSlip, kCalDwell, kCalTqGear, kCalGearRev,
                        kCalGearSpark, kCalFlexFuel, kCalFlexSpark, kCalFlexCold, kCalFlexKnock, kCalGlowPreheat, kCalCurveCount };
// what the launch / shift limiters are doing (telemetry)
enum LaunchState : int { kLaunchOff, kLaunch2Step, kLaunch3Step, kLaunchFlatShift, kLaunchShiftCut, kLaunchRollingAls, kLaunchStateCount };
const char* launchStateName(int s);
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
