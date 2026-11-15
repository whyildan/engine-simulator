#include "ecu/Calibration.h"
#include "sim/EngineParams.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>

namespace es {

static_assert(kCalMaxCyl == kMaxCyl, "calibration per-cylinder arrays must match kMaxCyl");

namespace {
const TableInfo kTables[kCalTableCount] = {
    {"ve_map",   "VE (speed density)", "rpm", "MAP kPa", "%",      5, 160, 0.5f},
    {"ve_tps",   "VE (alpha-N)",       "rpm", "TPS %",   "%",      1, 160, 0.5f},
    {"lambda",   "Target lambda",      "rpm", "MAP kPa", "lambda", 0.6f, 1.5f, 0.005f},
    {"ignition", "Ignition advance",   "rpm", "MAP kPa", "deg",    -20, 60, 0.5f},
};
const TableInfo kCurves[kCalCurveCount] = {
    {"crank_afr", "Cranking AFR",           "coolant C",  "", "AFR", 6, 16, 0.1f},
    {"warmup",    "Warm-up enrichment",     "coolant C",  "", "%",   0, 150, 1},
    {"accel",     "Acceleration enrichment","TPS rate %/s","", "%",  0, 150, 1},
    {"iat_fuel",  "IAT fuel correction",    "intake C",   "", "%",   -40, 40, 0.5f},
    {"iat_spark", "IAT spark correction",   "intake C",   "", "deg", -20, 10, 0.5f},
    {"ect_spark", "ECT spark correction",   "coolant C",  "", "deg", -20, 15, 0.5f},
    {"dead_time", "Injector dead time",     "battery V",  "", "ms",  0, 5, 0.01f},
};
} // namespace

const TableInfo& calTableInfo(int id) { return kTables[clampv(id, 0, kCalTableCount - 1)]; }
const TableInfo& calCurveInfo(int id) { return kCurves[clampv(id, 0, kCalCurveCount - 1)]; }

Table2D& calTable(Calibration& c, int id) {
    switch (id) { case kCalVeTps: return c.veTps; case kCalLambda: return c.lambda; case kCalIgnition: return c.ignition; default: return c.veMap; }
}
const Table2D& calTable(const Calibration& c, int id) { return calTable(const_cast<Calibration&>(c), id); }
Curve1D& calCurve(Calibration& c, int id) {
    switch (id) {
    case kCalWarmup: return c.warmup; case kCalAccel: return c.accel; case kCalIatFuel: return c.iatFuel;
    case kCalIatSpark: return c.iatSpark; case kCalEctSpark: return c.ectSpark; case kCalDeadTime: return c.deadTime;
    default: return c.crankAfr;
    }
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
            tb.x[i] = (float)(std::round((500.0 + (top - 500.0) * i / (kTabMax - 1)) / 50.0) * 50.0);
            tb.y[i] = t == kCalVeTps ? tpsAxis[i] : (float)std::round(20.0 + (mapTop - 20.0) * i / (kTabMax - 1));
        }
    }
    const double mechRpm = std::max(1000.0, (double)p.mechAdvanceRpm);
    for (int r = 0; r < kTabMax; ++r) {
        for (int k = 0; k < kTabMax; ++k) {
            const double rpm = c.veMap.x[k], map = c.veMap.y[r];
            const double load = map / 101.325;
            // VE: a generic guess - auto-tune / the tuner refine it
            // (at very low MAP the cylinder still traps more than MAP x Vd suggests: port pressure / backflow)
            c.veMap.v[r][k] = (float)(100.0 * std::min(1.05, (0.58 + 0.33 * std::min(1.0, load)) * (map + 6.0) / map));
            // cranking speeds: no ram effect, long overlap backflow -> the cylinder fills poorly
            c.veMap.v[r][k] *= (float)(1.0 - 0.3 * (1.0 - smoothstep(500.0, 1200.0, rpm)));
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
    c.dfcoExitEnrich = 60.0f; c.dfcoExitDecay = 0.35f;
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
}

} // namespace es
