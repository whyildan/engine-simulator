#include "ecu/Calibration.h"
#include "sim/EngineParams.h"
#include "sim/Fuel.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace es {

static_assert(kCalMaxCyl == kMaxCyl, "calibration per-cylinder arrays must match kMaxCyl");

namespace {
const TableInfo kTables[kCalTableCount] = {
    {"ve_map",   "VE (speed density)", "rpm", "MAP kPa", "%",      5, 160, 0.5f},
    {"ve_tps",   "VE (alpha-N)",       "rpm", "TPS %",   "%",      1, 160, 0.5f},
    {"lambda",   "Target lambda",      "rpm", "MAP kPa", "lambda", 0.6f, 1.5f, 0.005f},
    {"ignition", "Ignition advance",   "rpm", "MAP kPa", "deg",    -20, 60, 0.5f},
    {"ltft",     "Long-term fuel trim", "rpm", "MAP kPa", "%",     -40, 40, 0.5f},
    {"boost_target", "Boost target",   "rpm", "TPS %",   "bar",    0, 4, 0.05f},
    {"vtc",      "Intake cam advance (VTC)", "rpm", "MAP kPa", "deg", 0, 60, 0.5f},
    {"diesel_fuel", "Diesel fuel demand", "rpm", "pedal %", "mg/str", 0, 250, 0.5f},
    {"soi",      "Start of injection", "rpm", "fuel mg", "deg BTDC", -10, 40, 0.25f},
    {"egr",      "EGR valve", "rpm", "fuel mg", "%", 0, 100, 1},
};
const TableInfo kCurves[kCalCurveCount] = {
    {"crank_afr", "Cranking AFR",           "coolant C",  "", "AFR", 6, 16, 0.1f},
    {"warmup",    "Warm-up enrichment",     "coolant C",  "", "%",   0, 150, 1},
    {"accel",     "Acceleration enrichment","TPS rate %/s","", "%",  0, 150, 1},
    {"iat_fuel",  "IAT fuel correction",    "intake C",   "", "%",   -40, 40, 0.5f},
    {"iat_spark", "IAT spark correction",   "intake C",   "", "deg", -20, 10, 0.5f},
    {"ect_spark", "ECT spark correction",   "coolant C",  "", "deg", -20, 15, 0.5f},
    {"dead_time", "Injector dead time",     "battery V",  "", "ms",  0, 5, 0.01f},
    {"idle_target", "Idle target",          "coolant C",  "", "rpm", 300, 3500, 10},
    {"dbw_map",   "Throttle (DBW) map",     "pedal %",    "", "%",   0, 100, 1},
    {"boost_gear", "Boost by gear",         "gear",       "", "%",   0, 200, 1},
    {"tc_slip",   "Traction control slip target", "speed km/h", "", "%", 0, 100, 0.5f},
    {"dwell",     "Coil dwell",             "battery V",  "", "ms",  0.5f, 12, 0.05f},
    {"tq_gear",   "Torque limit by gear",   "gear",       "", "%",   10, 250, 1},
    {"gear_rev",  "Rev limit by gear",      "gear",       "", "rpm", 1000, 20000, 50},
    {"gear_spark", "Spark trim by gear",    "gear",       "", "deg", -15, 10, 0.5f},
    {"flex_fuel",  "Flex fuel: fuel trim",  "alcohol %",  "", "%",   -30, 30, 0.5f},
    {"flex_spark", "Flex fuel: spark added", "alcohol %", "", "deg", -10, 15, 0.5f},
    {"flex_cold",  "Flex fuel: cold enrichment", "alcohol %", "", "%", 0, 400, 1},
    {"flex_knock", "Flex fuel: knock threshold", "alcohol %", "", "x", 0.5f, 3, 0.05f},
    {"glow_preheat", "Glow plug pre-heat time", "coolant C", "", "s", 0, 30, 0.5f},
};
const TriggerInfo kTriggers[kTrigCount] = {
    {"60-2 (Bosch)", 60, 1, false},
    {"36-1", 36, 1, false},
    {"36-2-2-2 (fast sync)", 36, 3, false},
    {"24-1", 24, 1, false},
    {"12-1", 12, 1, false},
    {"4 teeth + cam tooth (distributor style)", 4, 0, true},
};
const ProtInfo kProts[kProtCount] = {
    {"over_rev",   "Over-rev",               "rpm",   true,  0, 20000},
    {"overboost",  "Overboost",              "bar",   true,  0, 5},
    {"coolant",    "High coolant temperature", "C",   true,  60, 150},
    {"oil_temp",   "High oil temperature",   "C",     true,  60, 180},
    {"oil_press",  "Low oil pressure",       "bar",   false, 0, 6},
    {"fuel_press", "Low fuel pressure",      "bar",   false, 0, 8},
    {"lean",       "Lean at load",           "lambda", true, 0.8f, 2},
    {"knock",      "Excessive knock",        "deg",   true,  0, 30},
    {"egt",        "High EGT",               "C",     true,  500, 1300},
    {"sensor",     "Sensor failure",         "sensors", true, 0, 10},
    {"backpressure", "High drive pressure (EMAP / MAP)", "ratio", true, 1, 5},
};
const char* kActNames[kActCount] = {"Warning only", "Ignition retard", "Boost reduction", "Throttle limit", "Fuel limit (rpm cap)",
                                    "Limp mode", "Engine shutdown"};
} // namespace

const TriggerInfo& triggerInfo(int pattern) { return kTriggers[clampv(pattern, 0, kTrigCount - 1)]; }
const char* triggerNames() { return "60-2 (Bosch)\0" "36-1\0" "36-2-2-2 (fast sync)\0" "24-1\0" "12-1\0" "4 teeth + cam tooth (distributor style)\0"; }
const char* injModeNames() { return "Sequential\0Semi-sequential\0Batch fire\0"; }
const TableInfo& calTableInfo(int id) { return kTables[clampv(id, 0, kCalTableCount - 1)]; }
const TableInfo& calCurveInfo(int id) { return kCurves[clampv(id, 0, kCalCurveCount - 1)]; }
const ProtInfo& protInfo(int id) { return kProts[clampv(id, 0, kProtCount - 1)]; }
const char* protActionName(int a) { return kActNames[clampv(a, 0, kActCount - 1)]; }

Table2D& calTable(Calibration& c, int id) {
    switch (id) {
    case kCalVeTps: return c.veTps; case kCalLambda: return c.lambda; case kCalIgnition: return c.ignition;
    case kCalLtft: return c.ltft; case kCalBoostTarget: return c.boostTarget; case kCalVtc: return c.vtcMap;
    case kCalDieselFuel: return c.dieselFuel; case kCalSoi: return c.soi; case kCalEgr: return c.egrMap;
    default: return c.veMap;
    }
}
const Table2D& calTable(const Calibration& c, int id) { return calTable(const_cast<Calibration&>(c), id); }
Curve1D& calCurve(Calibration& c, int id) {
    switch (id) {
    case kCalWarmup: return c.warmup; case kCalAccel: return c.accel; case kCalIatFuel: return c.iatFuel;
    case kCalIatSpark: return c.iatSpark; case kCalEctSpark: return c.ectSpark; case kCalDeadTime: return c.deadTime;
    case kCalIdleTarget: return c.idleTarget; case kCalDbwMap: return c.dbwMap; case kCalBoostGear: return c.boostGear;
    case kCalTcSlip: return c.tcSlipTarget;
    case kCalDwell: return c.dwell; case kCalTqGear: return c.tqLimitGear; case kCalGearRev: return c.gearRevLimit;
    case kCalGearSpark: return c.gearSpark;
    case kCalFlexFuel: return c.flexFuel; case kCalFlexSpark: return c.flexSpark;
    case kCalFlexCold: return c.flexCold; case kCalFlexKnock: return c.flexKnock; case kCalGlowPreheat: return c.glowPreheat;
    default: return c.crankAfr;
    }
}
const char* launchStateName(int s) {
    static const char* n[kLaunchStateCount] = {"-", "2-STEP", "3-STEP", "FLAT SHIFT", "SHIFT CUT", "ROLLING ALS"};
    return n[clampv(s, 0, kLaunchStateCount - 1)];
}
const Curve1D& calCurve(const Calibration& c, int id) { return calCurve(const_cast<Calibration&>(c), id); }

static void axisPos(const float* a, int n, double x, int& i0, int& i1, float& w) {
    if (n <= 1 || x <= a[0]) { i0 = i1 = 0; w = 0; return; }
    if (x >= a[n - 1]) { i0 = i1 = n - 1; w = 0; return; }
    int i = 0;
    while (i < n - 2 && x > a[i + 1]) ++i;
    i0 = i; i1 = i + 1;
    double span = a[i1] - a[i0];
    w = span > 1e-9 ? (float)((x - a[i0]) / span) : 0.0f;
}

CellWeights tableWeights(const Table2D& t, double x, double y) {
    CellWeights cw;
    axisPos(t.x, clampv((int)t.nx, 1, kTabMax), x, cw.ix0, cw.ix1, cw.wx);
    axisPos(t.y, clampv((int)t.ny, 1, kTabMax), y, cw.iy0, cw.iy1, cw.wy);
    return cw;
}

double tableLookup(const Table2D& t, double x, double y) {
    CellWeights w = tableWeights(t, x, y);
    double a = t.v[w.iy0][w.ix0] + (t.v[w.iy0][w.ix1] - t.v[w.iy0][w.ix0]) * w.wx;
    double b = t.v[w.iy1][w.ix0] + (t.v[w.iy1][w.ix1] - t.v[w.iy1][w.ix0]) * w.wx;
    return a + (b - a) * w.wy;
}

double curveLookup(const Curve1D& c, double x) {
    int i0, i1; float w;
    axisPos(c.x, clampv((int)c.n, 1, kCurveMax), x, i0, i1, w);
    return c.v[i0] + (c.v[i1] - c.v[i0]) * w;
}

// ------------------------------------------------------------------ base map generator
static void setCurve(Curve1D& c, std::initializer_list<float> xs, double (*f)(double, const EngineParams&), const EngineParams& p) {
    c.n = 0; c.locked = 0;
    for (float x : xs) { c.x[c.n] = x; c.v[c.n] = (float)f(x, p); ++c.n; }
}

void generateCalibration(EngineParams& p) {
    Calibration& c = p.cal;
    c = Calibration();
    // individual throttle bodies give a weak, pulsating MAP signal: they run on throttle angle (alpha-N)
    c.loadMode = p.itb ? LoadMode::AlphaN : LoadMode::SpeedDensity;
    if (isDiesel(p)) c.loadMode = LoadMode::Maf;   // a diesel ECU measures its air (smoke limiter)
    // positive-displacement blowers with a bypass valve: MAP alone does not describe the air path ->
    // speed density at part throttle, throttle angle once the bypass closes (hybrid)
    // (positive-displacement blowers run fine on speed density once the blower leaks like a real one)
    c.hybridTpsLo = 4; c.hybridTpsHi = 20;
    const double top = std::max(3000.0, p.revLimit + 500.0);
    const bool boosted = isBoosted(p);
    const double maxBoost = boosted ? (p.induction == Induction::Turbo ? p.wastegateBar : 1.0) : 0.0;
    const double mapTop = boosted ? 101.3 * (1.0 + maxBoost + 0.35) : 105.0;
    static const float tpsAxis[kTabMax] = {0, 2, 4, 7, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100};
    for (int t = 0; t < kCalTableCount; ++t) {
        Table2D& tb = calTable(c, t);
        tb.nx = kTabMax; tb.ny = kTabMax; tb.locked = 0;
        for (int i = 0; i < kTabMax; ++i) {
            // rpm breakpoints dense around idle (500 .. 1500 in 250 steps), then even up to the top: the idle
            // and return-to-idle region needs its own cells, not an interpolation from cranking speed
            tb.x[i] = i < 5 ? (float)(500 + 250 * i)
                            : (float)(std::round((1500.0 + (top - 1500.0) * (i - 4) / (kTabMax - 5)) / 50.0) * 50.0);
            tb.y[i] = (t == kCalVeTps || t == kCalDieselFuel) ? tpsAxis[i] : (float)std::round(20.0 + (mapTop - 20.0) * i / (kTabMax - 1));
            if (t == kCalSoi || t == kCalEgr) tb.y[i] = (float)std::round(std::max(10.0f, p.dieselMaxFuel) * 1.2 * i / (kTabMax - 1) * 2.0) / 2.0f;
        }
        // VTEC: breakpoints just either side of the cam crossover, so the step in breathing is a step in the map
        // (and the base map rig measures both sides) instead of being smeared over a whole column
        if (p.vtec) {
            const float xo = (float)(std::round(0.68 * p.revLimit / 100.0) * 100.0);
            for (int k = 5; k + 2 < kTabMax; ++k)
                if (tb.x[k] <= xo && xo < tb.x[k + 1]) {
                    if (xo - 150.0f > tb.x[k - 1] + 50.0f) tb.x[k] = xo - 150.0f;
                    if (xo + 150.0f < tb.x[k + 2] - 50.0f) tb.x[k + 1] = xo + 150.0f;
                    break;
                }
        }
    }
    const double mechRpm = std::max(1000.0, (double)p.mechAdvanceRpm);
    // valve overlap (base cam position): the more overlap, the more burned gas the cylinder keeps at light load
    // and at low speed - a small-overlap cam (e.g. a VTEC low cam) traps clearly more air there
    double overlapF = 1.0;
    if (!isRotary(p) && !isTwoStroke(p)) {
        const double ivo = 360.0 + p.intakeCenterline - p.intakeCamAdvance - 0.5 * p.intakeDuration;
        const double evc = 360.0 - p.exhaustCenterline - p.exhaustCamAdvance + 0.5 * p.exhaustDuration;
        overlapF = clampv(std::max(0.0, evc - ivo) / 28.0, 0.15, 2.0);   // 1 = the generic 28 deg cam
    }
    for (int r = 0; r < kTabMax; ++r) {
        for (int k = 0; k < kTabMax; ++k) {
            const double rpm = c.veMap.x[k], map = c.veMap.y[r];
            const double load = map / 101.325;
            // VE: a generic guess - auto-tune / the tuner refine it
            // (at very low MAP the cylinder still traps more than MAP x Vd suggests: port pressure / backflow)
            c.veMap.v[r][k] = (float)(100.0 * std::min(1.05, (0.58 + 0.33 * std::min(1.0, load)) * (map + 6.0) / map));
            // small overlap: less residual at part load
            if (overlapF < 1.0) c.veMap.v[r][k] *= (float)(1.0 + 0.15 * (1.0 - overlapF) * (1.0 - std::min(1.0, load)));
            // cranking speeds: no ram effect, overlap backflow -> the cylinder fills poorly (scaled by the overlap)
            c.veMap.v[r][k] *= (float)(1.0 - std::min(0.5, 0.3 * overlapF) * (1.0 - smoothstep(500.0, 1200.0, rpm)));
            // alpha-N: air relative to an atmospheric cylinder, from a rough throttle -> MAP estimate
            const double tps = c.veTps.y[r] * 0.01;
            // throttle area (plus idle bypass) against the air the engine pulls at this speed
            const double aEff = (p.itb ? 0.02 : 0.012) + std::sqrt(tps);
            const double mapEst = 101.325 * clampv(aEff / (aEff + 0.0185 * rpm / 1000.0 * (p.itb ? 1.0 : 2.5)), 0.12, 1.0);
            c.veTps.v[r][k] = (float)(100.0 * (0.6 + 0.35 * std::min(1.0, mapEst / 101.325)) * mapEst / 101.325);
            // target lambda: cruise -> full load -> boost enrichment
            double afr = lerp((double)p.afrCruise, (double)p.afrWot, smoothstep(0.80, 0.97, load));
            if (boosted) afr = lerp(afr, (double)p.afrBoost, clampv((map - 101.325) / 100.0, 0.0, 1.0));
            c.lambda.v[r][k] = (float)(afr / 14.7);
            // ignition: the distributor-style curve, boost retard above atmospheric
            double adv = p.baseAdvance + p.mechAdvance * smoothstep(900.0, mechRpm, rpm) +
                         p.vacuumAdvance * (1.0 - smoothstep(0.40, 0.78, load));
            if (boosted) adv -= p.boostRetard * std::max(0.0, (map - 101.325) / 100.0);
            c.ignition.v[r][k] = (float)(std::round(adv * 2.0) / 2.0);
        }
    }
    // ---- diesel (Package 4): driver demand linear in the pedal up to the rated fuel; start of injection from the
    //      static timing plus the speed advance (a mechanical pump's timing device), the same at every quantity
    for (int r = 0; r < kTabMax; ++r)
        for (int k = 0; k < kTabMax; ++k) {
            const double rpm = c.dieselFuel.x[k];
            c.dieselFuel.v[r][k] = (float)(std::round(p.dieselMaxFuel * c.dieselFuel.y[r] * 0.01 * 2.0) / 2.0);
            const double soi = p.soiBase + p.soiAdvance * smoothstep((double)p.idleTargetRpm, std::max(p.idleTargetRpm + 500.0, 0.85 * p.revLimit), rpm);
            c.soi.v[r][k] = (float)(std::round(soi * 4.0) / 4.0);
            // EGR (a starting point, not from a source): open at idle and light load, closed towards full load and
            // high speed, where the engine needs all its air (the smoke limiter would take the fuel away)
            const double q = c.egrMap.y[r] / std::max(1.0f, p.dieselMaxFuel);
            c.egrMap.v[r][k] = (float)std::round(p.egrValveDia_mm > 0 ? 60.0 * (1.0 - smoothstep(0.25, 0.55, q)) * (1.0 - smoothstep(2500.0, 3500.0, rpm)) : 0.0);
        }
    c.egrMinEct = 60;
    // smoke limit: MIT 2.61 (lecture 16) "smoke-limit A/F ~ 20" -> lambda 20 / 14.5 = 1.38
    c.smokeLambda = 1.38f;
    c.dieselIdleKp = 0.02f; c.dieselIdleKi = 0.04f; c.dieselGovBand = 400; c.dieselStartFuel = 0.6f * p.dieselMaxFuel;
    // glow plugs: BorgWarner / BERU glow plug technical information p. 8 - pre-heat 0 C: 2 s, -5 C: ~5 s, -10 C: ~7 s
    // (colder: kept at 7 s, not in the source); "2-5 s at normal outside temperatures" (p. 7) -> 2 s up to 20 C, none
    // on a warm engine; post-heat up to 3 minutes, ending when the coolant reaches 70 C (p. 7)
    setCurve(c.glowPreheat, {-30, -10, -5, 0, 20, 40}, [](double x, const EngineParams&) {
        return x <= -10 ? 7.0 : x <= -5 ? 5.0 + (-5.0 - x) * 0.4 : x <= 0 ? 2.0 + (-x) * 0.6 : x <= 20 ? 2.0 : 0.0; }, p);
    c.afterglowTime = 180; c.afterglowEct = 70;
    // pilot injection: Busch et al. (Sandia, SAND2015-2001 / SAE 2015-01-0796) - 1.5 mg pilot at 1500 rpm; the noise
    // drop holds for dwells above ~200 us -> 300 us here; up to 3000 rpm (chosen). At idle the pilot's own delay
    // (~0.5 ms) is longer than that: with 300 us it has not lit when the main starts and does nothing -> 1000 us at
    // idle (chosen: ~6 dB less clatter at idle in the model)
    c.pilotMg = 1.5f; c.pilotDwellUs = 300; c.pilotDwellIdleUs = 1000; c.pilotMaxRpm = 3000;
    c.powerTpsLo = 60; c.powerTpsHi = 90; c.powerLambda = p.afrWot / 14.7f;
    c.crankAdvance = std::min(p.baseAdvance, 8.0f);
    auto cold = [](double ect) { return 1.0 - smoothstep(15.0, 75.0, ect); };
    setCurve(c.crankAfr, {-30, -10, 0, 10, 20, 40, 60, 80, 100}, [](double x, const EngineParams&) { return 11.5 - 1.0 * (1.0 - smoothstep(15.0, 75.0, x)); }, p);
    setCurve(c.warmup, {-30, -10, 0, 10, 20, 40, 60, 80, 100}, [](double x, const EngineParams&) { return 30.0 * (1.0 - smoothstep(15.0, 75.0, x)); }, p);
    (void)cold;
    c.afterStartAfr = 11.8f; c.afterStartTime = 3.0f;
    setCurve(c.accel, {0, 50, 100, 200, 400, 800, 1600}, [](double x, const EngineParams&) {
        static const double v[] = {0, 0, 3, 8, 15, 24, 32};
        static const double xs[] = {0, 50, 100, 200, 400, 800, 1600};
        for (int i = 0; i < 7; ++i) if (std::fabs(x - xs[i]) < 1e-6) return v[i];
        return 0.0; }, p);
    c.accelDecay = 0.3f; c.accelColdMult = 1.5f;
    // after a decel cut the exhaust is full of air: at high vacuum the residual gas in the chamber is about as much
    // as the fresh charge, so the first cycles back on fuel get a lot of air - strong, short enrichment
    c.dfcoExitEnrich = 150.0f; c.dfcoExitDecay = 0.25f;
    c.asyncTipIn = 1;
    setCurve(c.iatFuel, {-20, 0, 20, 40, 60, 80}, [](double, const EngineParams&) { return 0.0; }, p);
    setCurve(c.iatSpark, {-20, 0, 20, 40, 50, 60, 70, 80}, [](double x, const EngineParams&) { return x <= 40 ? 0.0 : -(x - 40.0) / 10.0; }, p);
    setCurve(c.ectSpark, {-20, 0, 20, 40, 60, 80, 100, 110}, [](double, const EngineParams&) { return 0.0; }, p);
    setCurve(c.deadTime, {8, 9, 10, 11, 12, 13, 14, 15, 16}, [](double v, const EngineParams& pp) {
        return std::round(pp.comp.injDeadTime_ms * (1.0 + 0.12 * (14.0 - v)) * 1000.0) / 1000.0; }, p);
    c.injFlow = p.injector_ccmin;
    for (int i = 0; i < kCalMaxCyl; ++i) { c.cylFuel[i] = 0; c.cylSpark[i] = 0; }
    c.autoTune = p.veLearning ? 1 : 0;
    c.closedLoop = p.veLearning ? 1 : 0;

    // ---- cam control: VTEC crossover at ~70 % of the limit under load, VTC advance for mid-range torque and
    //      little overlap at idle / light load (stable idle, little exhaust pulled back on the overrun)
    c.vtecRpm = (float)(std::round(0.68 * p.revLimit / 100.0) * 100.0);
    c.vtecHyst = 300; c.vtecMinMap = 60; c.vtecMinEct = 60;
    {
        Table2D& t = c.vtcMap;
        t = Table2D{};
        static const float maps[6] = {20, 35, 50, 65, 80, 100};
        t.nx = 10; t.ny = 6; t.locked = 0;
        const double range = p.vtc ? std::max(0.0f, p.vtcRange) : 0.0;
        for (int i = 0; i < 10; ++i) t.x[i] = (float)(std::round((800.0 + (top - 800.0) * i / 9.0) / 50.0) * 50.0);
        for (int r = 0; r < 6; ++r) {
            t.y[r] = maps[r];
            const double load = smoothstep(30.0, 70.0, maps[r]);
            for (int k = 0; k < 10; ++k) {
                const double rpmF = t.x[k] / std::max(2000.0, (double)p.revLimit);
                // advance peaks in the mid range under load and backs off towards the limit (late intake closing
                // fills the cylinder at high rpm); nearly none at idle
                const double mid = smoothstep(0.12, 0.35, rpmF) * (1.0 - 0.55 * smoothstep(0.55, 0.95, rpmF));
                t.v[r][k] = (float)(std::round(range * (0.06 + 0.52 * load) * mid * 2.0) / 2.0);
            }
        }
    }

    // ---- closed-loop lambda
    c.stftGain = 0.5f; c.stftLimit = 25; c.ltftLearn = 1; c.ltftRate = 0.15f; c.ltftLimit = 25;
    c.clMinEct = 60; c.clDelay = 2.0f; c.clMaxMap = 90;
    {   // learned trim: 8 x 8 cells over the same rpm / MAP span as the VE map, all 0 %
        Table2D& t = c.ltft;
        t = Table2D{};
        t.nx = 8; t.ny = 8; t.locked = 0;
        for (int i = 0; i < 8; ++i) {
            t.x[i] = (float)(std::round((500.0 + (top - 500.0) * i / 7.0) / 50.0) * 50.0);
            t.y[i] = (float)std::round(20.0 + (mapTop - 20.0) * i / 7.0);
        }
        for (int r = 0; r < kTabMax; ++r) for (int k = 0; k < kTabMax; ++k) t.v[r][k] = 0;
    }
    // ---- knock control
    c.knockControl = 1; c.knockThreshold = 1.0f; c.knockRetardStep = 1.5f; c.knockMaxRetard = 10; c.knockRecover = 1.0f;
    c.knockMinRpm = 1200;
    // ---- idle / throttle / limiter (the gains reproduce the previous fixed idle controller)
    {
        const double idle = p.idleTargetRpm;
        setCurve(c.idleTarget, {-30, -10, 0, 10, 20, 40, 60, 80, 100}, [](double x, const EngineParams& pp) {
            return std::round((pp.idleTargetRpm + 300.0 * (1.0 - smoothstep(15.0, 75.0, x))) / 10.0) * 10.0; }, p);
        (void)idle;
    }
    c.idleKp = 0.0007f; c.idleKi = 0.0002f; c.idleKd = 0.00012f;
    c.idleSparkGain = 0.04f; c.idleSparkMin = -8; c.idleSparkMax = 10;
    setCurve(c.dbwMap, {0, 10, 20, 30, 40, 50, 60, 70, 80, 100}, [](double x, const EngineParams&) { return x; }, p);
    c.softLimitBand = 200; c.softLimitRetard = 8; c.softLimitCut = 0.3f;
    // ---- boost control: off by default (wastegate spring); the target table starts at the spring pressure
    c.boostControl = 0;
    {
        Table2D& t = c.boostTarget;
        t = Table2D{};
        static const float tps[6] = {0, 20, 40, 60, 80, 100};
        t.nx = 10; t.ny = 6; t.locked = 0;
        const double spring = p.induction == Induction::Turbo ? p.wastegateBar : 0.5;
        for (int i = 0; i < 10; ++i) t.x[i] = (float)(std::round((1000.0 + (top - 1000.0) * i / 9.0) / 50.0) * 50.0);
        for (int r = 0; r < 6; ++r) {
            t.y[r] = tps[r];
            for (int k = 0; k < 10; ++k) {
                t.v[r][k] = (float)(std::round(spring * smoothstep(30.0, 90.0, tps[r]) * 20.0) / 20.0);
                // diesel (no throttle): the axis is the PEDAL; the target rises with the load and is full from ~1800
                // rpm (the turbo cannot make it lower down) - a starting point, not from a source
                if (isDiesel(p))
                    t.v[r][k] = (float)(std::round(spring * smoothstep(10.0, 70.0, tps[r]) * (0.35 + 0.65 * smoothstep(1000.0, 1800.0, t.x[k])) * 20.0) / 20.0);
            }
        }
    }
    // a variable geometry turbine has no spring to fall back on: its vanes are always the ECU's
    setCurve(c.boostGear, {0, 1, 2, 3, 4, 5, 6, 7, 8}, [](double, const EngineParams&) { return 100.0; }, p);
    c.boostKp = 0.6f; c.boostKi = 1.5f; c.boostKd = 0.03f; c.wgDutyMax = 0.9f;
    if (p.induction == Induction::Turbo && p.vgt) { c.boostControl = 1; c.wgDutyMax = 1.0f; }   // (vanes may close fully)
    c.overboostMargin = 0.3f; c.overboostTime = 0.5f;
    c.boostCutBar = p.induction == Induction::Turbo ? std::max(0.5f, p.wastegateBar * 1.8f + 0.3f) : 3.0f;
    // ---- protection rules: {enabled, warn, limit, delay s, recover, action}
    const bool turbo = p.induction == Induction::Turbo, efiFuel = p.fuelSystem == FuelSystem::EFI;
    c.prot[kProtOverRev]   = {1, p.revLimit + 200.0f, p.revLimit + 600.0f, 0.2f, 300, kActWarn};
    c.prot[kProtOverboost] = {boosted ? 1 : 0, (float)maxBoost + 0.2f, (float)maxBoost + 0.45f, 0.5f, 0.2f, kActBoostReduce};
    c.prot[kProtCoolant]   = {1, 105, 115, 3.0f, 8, kActLimp};
    c.prot[kProtOilTemp]   = {1, 130, 145, 3.0f, 10, kActLimp};
    c.prot[kProtOilPress]  = {1, 1.0f, 0.5f, 1.0f, 0.3f, kActLimp};      // at 3000 rpm; scaled down towards idle
    c.prot[kProtFuelPress] = {efiFuel ? 1 : 0, 2.0f, 1.5f, 1.0f, 0.3f, kActLimp};
    c.prot[kProtLean]      = {efiFuel ? 1 : 0, 1.0f, 1.08f, 0.8f, 0.05f, turbo ? kActBoostReduce : kActThrottleLimit};
    c.prot[kProtKnock]     = {1, 6, 9, 1.5f, 2, turbo ? kActBoostReduce : kActThrottleLimit};
    c.prot[kProtEgt]       = {1, 950, 1050, 2.0f, 50, kActWarn};
    c.prot[kProtSensor]    = {1, 1, 1, 0.5f, 0.5f, kActWarn};
    // exhaust backpressure against boost: a choked turbine housing (too small A/R) heats the charge, raises the
    // residual gas and the EGT - and blows head gaskets
    c.prot[kProtBackpressure] = {turbo ? 1 : 0, 2.0f, 2.6f, 1.0f, 0.2f, kActBoostReduce};
    c.limpThrottle = 35; c.limpRpm = 3500; c.protRetard = 5;

    // ---- motorsport (Phase 5): all off by default except where a switch arms it; engines built for
    //      anti-lag (drag / rally setups) come with launch control and the shift cut on
    const double lim = p.revLimit;
    c.launchControl = p.antiLag ? 1 : 0;
    c.launchRpm = (float)(std::round(std::min(turbo ? 0.6 * lim : 0.55 * lim, 5000.0) / 100.0) * 100.0);
    c.launchTiming = turbo ? -8.0f : 8.0f;
    c.launchMinTps = 70; c.launchMaxSpeed = 8; c.launchCutBand = 150; c.launchCutMode = 0; c.launchEnrich = 10;
    c.clutchMovingMode = 0;
    c.threeStepRpm = (float)(std::round(0.8 * lim / 100.0) * 100.0);
    c.flatShiftMargin = 150; c.flatShiftRetard = 10;
    c.shiftCut = p.antiLag ? 1 : 0; c.shiftCutTime = 0.12f; c.shiftCutMinTps = 70;
    c.alsTiming = -30; c.alsThrottle = 10; c.alsMaxTime = 2.0f; c.alsMinRpm = 2500;   // the former fixed anti-lag
    c.rollingAls = 1; c.rollingAlsMinSpeed = 10;
    c.tractionControl = 0;
    setCurve(c.tcSlipTarget, {0, 10, 20, 40, 80, 160}, [](double x, const EngineParams&) {
        static const double xs[6] = {0, 10, 20, 40, 80, 160}, v[6] = {25, 18, 14, 10, 8, 8};
        for (int i = 0; i < 6; ++i) if (std::fabs(x - xs[i]) < 1e-6) return v[i];
        return 10.0; }, p);
    c.tcKp = 0.02f; c.tcKi = 0.08f; c.tcMaxRetard = 15; c.tcMaxCut = 0.9f;
    c.n2oMinTps = 90; c.n2oMinRpm = (float)(std::round(std::max(2500.0, 0.35 * lim) / 100.0) * 100.0);
    c.n2oMaxRpm = (float)(lim - 300.0); c.n2oMinEct = 60; c.n2oMinSpeed = 0;
    c.n2oDelay = 0.2f; c.n2oRampTime = 0.5f; c.n2oStartPct = 50;
    c.n2oRetard = (float)std::round(std::max(0.0f, p.n2oShotHp) / 25.0f);   // ~1 deg per 25 hp
    c.n2oFlowGs = (float)(n2oFullFlowKgs(p) * 1000.0);
    c.n2oLeanLambda = 0.95f;
    c.dfcoResumeAbove = 1000; c.dfcoAnticipate = 0.3f;
    c.dfcoDelay = 0.25f;   // a quick lift or a blip does not cut the fuel (and so needs no recovery)

    // ---- ECU configuration (Phase 7): a 36-1 wheel + cam sensor, sequential injection (carburettor / rotary: no
    //      injection timing to choose); dwell = 1.3 x the coil's charge time, longer as the battery sags
    c.triggerPattern = isRotary(p) ? kTrig36_1 : (p.fuelSystem == FuelSystem::Carburetor ? kTrig4Cam : kTrig36_1);
    c.camSensor = 1;
    c.injMode = kInjSequential;
    setCurve(c.dwell, {8, 10, 12, 13, 14, 15, 16}, [](double v, const EngineParams& q) {
        return 1.3 * std::max(0.3, (double)q.comp.coilCharge_ms) * 14.0 / v; }, p);
    c.dwellMaxDuty = 0.7f;
    // ---- torque management / gear-based control: off, limits wide open
    c.torqueMgmt = 0;
    setCurve(c.tqLimitGear, {1, 2, 3, 4, 5, 6}, [](double, const EngineParams& q) { return isBoosted(q) ? 250.0 : 150.0; }, p);
    c.tqMaxRetard = 12; c.tqMaxCut = 0.8f; c.tqThrottleRate = 0.02f;
    c.gearControl = 0;
    setCurve(c.gearRevLimit, {1, 2, 3, 4, 5, 6}, [](double, const EngineParams& q) { return (double)q.revLimit; }, p);
    setCurve(c.gearSpark, {1, 2, 3, 4, 5, 6}, [](double, const EngineParams&) { return 0.0; }, p);

    // ---- fuel / flex fuel: the ECU is set up for the fuel the engine is filled with
    {
        const FuelBlend b = fuelTypeBlend(p.fuelType);
        c.stoichGasoline = 14.7f;
        c.stoichAlcohol = b.methanol > b.ethanol ? 6.45f : 9.0f;
        c.flexFallbackPct = (float)fuelProps(b).alcoholPct;
    }
    auto flexCurve = [](Curve1D& cv, std::initializer_list<float> v) {
        static const float xs[] = {0, 10, 20, 30, 50, 70, 85, 100};
        cv.n = 8; cv.locked = 0;
        int i = 0;
        for (float y : v) { cv.x[i] = xs[i]; cv.v[i] = y; ++i; }
    };
    // the cooler (denser) charge of an alcohol blend carries more air than the VE map made on gasoline says
    flexCurve(c.flexFuel, {0, 0.3f, 0.6f, 1, 1.8f, 2.6f, 3.1f, 3.6f});
    // more knock resistance with alcohol: a few degrees more where the gasoline map was knock limited
    flexCurve(c.flexSpark, {0, 0.5f, 1, 1.5f, 2, 3, 3.5f, 4});
    // alcohol stays liquid on cold walls: E85 needs about half again the cold fuel, E100 twice (much more floods it:
    // the liquid film keeps feeding the cylinders after the start)
    flexCurve(c.flexCold, {0, 0, 1, 3.5f, 10, 25, 48, 100});
    flexCurve(c.flexKnock, {1, 1, 1, 1, 1, 1, 1, 1});
}

} // namespace es
