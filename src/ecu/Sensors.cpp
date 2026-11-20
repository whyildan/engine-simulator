#include "ecu/Sensors.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
//                name               unit     lo     hi     noise  lag ms  fallback analog  role
const SensorDef kDefs[kSensorCount] = {
    {"TPS throttle position", "%",     0,   100,   0.15,    2,     0, true,  "DFCO, WOT enrichment, MAP fallback (alpha-N)"},
    {"APP pedal position",    "%",     0,   100,   0.15,    2,     0, true,  "idle control, anti-lag"},
    {"MAP manifold pressure", "kPa",  10,   310,   0.25,    3,   100, true,  "fuel load (speed density), vacuum advance, boost retard"},
    {"MAF air flow",          "g/s",   0,   800,   0.6,    10,     0, true,  "monitoring (MAF load mode: Phase 2)"},
    {"IAT intake air temp",   "C",   -40,   150,   0.1,  3000,    30, true,  "air density for the fuel calculation"},
    {"ECT coolant temp",      "C",   -40,   150,   0.1,  2000,    80, true,  "warm-up / cranking enrichment, fast idle, closed-loop enable"},
    {"O2 narrowband",         "V",     0,   1.1,   0.01,  120,  0.45, true,  "backup closed loop if the wideband fails"},
    {"Wideband lambda",       "lambda", 0.6, 1.6,  0.004,  50,     1, true,  "closed-loop fuel trim"},
    {"CKP crank position",    "rpm",   0, 20000,   0,       0,     0, false, "engine speed and angle: no signal = no spark / fuel"},
    {"CMP cam position",      "sync",  0,     1,   0,       0,     1, false, "cycle phase: lost = semi-sequential injection"},
    {"Knock sensor",          "index", 0,     5,   0.05,   10,     0, true,  "monitoring (knock control: Phase 3)"},
    {"EGT exhaust temp",      "C",     0,  1250,   2,     400,     0, true,  "monitoring (protection: Phase 3)"},
    {"Oil pressure",          "bar",   0,    10,   0.03,   50,     3, true,  "oil warning lamp (protection: Phase 3)"},
    {"Oil temperature",       "C",   -40,   160,   0.1,  2000,    90, true,  "monitoring (protection: Phase 3)"},
    {"Fuel pressure",         "bar",   0,    10,   0.02,   20,     3, true,  "injector flow compensation"},
    {"Vehicle speed",         "km/h",  0,   400,   0,       0,     0, false, "monitoring"},
    {"Gear position",         "gear",  0,     8,   0,       0,     0, false, "monitoring"},
    {"Battery voltage",       "V",     6,    18,   0.02,   20,    14, true,  "injector dead-time compensation"},
    // frequency output (50 Hz = E0 .. 150 Hz = E100): a broken wire is no signal at all
    {"Flex fuel (alcohol)",   "%",     0,   100,   0.3,   800,     0, false, "stoich AFR, injector flow, spark, cold enrichment by alcohol content"},
    {"EMAP exhaust pressure", "kPa",  10,   710,   0.5,    15,   101, true,  "backpressure, drive pressure ratio (EMAP / MAP) protection"},
    {"EGT after turbine",     "C",     0,  1250,   2,     400,     0, true,  "monitoring (turbine heat drop)"},
};

double toVolts(const SensorDef& d, double x) { return 0.5 + 4.0 * (x - d.lo) / (d.hi - d.lo); }
double fromVolts(const SensorDef& d, double v) { return d.lo + (v - 0.5) / 4.0 * (d.hi - d.lo); }
} // namespace

const SensorDef& sensorDef(int id) { return kDefs[clampv(id, 0, kSensorCount - 1)]; }
const char* sensorFaultNames() { return "OK\0Open circuit\0Short to 5 V\0Stuck\0Noisy\0"; }

void defaultSensorConfig(SensorConfig out[kSensorCount]) {
    for (int i = 0; i < kSensorCount; ++i) {
        out[i].noiseScale = 1.0f;
        out[i].lag_ms = (float)kDefs[i].lag_ms;
        out[i].fault = SensorFault::None;
    }
}

void SensorLayer::reset(const EngineParams&) { init_ = false; histW_ = 0; }

void SensorLayer::update(double dt, const SensorTruth& t, const EngineParams& p) {
    truth = t;
    r.in = t.in;
    // exhaust gas reaches the lambda sensors after a transport delay (shorter at high flow)
    lamHist_[histW_] = t.v[kSnsWideband];
    const double rpm = std::max(300.0, t.v[kSnsCKP]);
    const double delay = t.lamDelay >= 0 ? clampv(t.lamDelay, 0.005, 0.4) : clampv(0.015 + 25.0 / rpm, 0.02, 0.12);
    const int back = clampv((int)(delay / std::max(1e-5, dt)), 0, kHist - 1);
    const double lamDelayed = lamHist_[(histW_ - back + kHist) % kHist];
    histW_ = (histW_ + 1) % kHist;

    for (int i = 0; i < kSensorCount; ++i) {
        const SensorDef& d = kDefs[i];
        const SensorConfig& cfg = p.sensors[i];
        double x = t.v[i];
        if (i == kSnsWideband || i == kSnsO2) x = lamDelayed;
        // response lag
        const double tau = std::max(0.0, (double)cfg.lag_ms) * 1e-3;
        if (!init_) filt_[i] = x;
        else filt_[i] += (x - filt_[i]) * (tau > 1e-6 ? 1.0 - std::exp(-dt / tau) : 1.0);
        double m = filt_[i];
        // narrowband O2: steep switch around stoichiometric (rich = high voltage)
        if (i == kSnsO2) m = 0.45 + 0.42 * std::tanh(-(m - 1.0) * 25.0);
        // noise
        const SensorFault f = cfg.fault;
        double sigma = d.noise * clampv((double)cfg.noiseScale, 0.0, 10.0) * (f == SensorFault::Noisy ? 25.0 : 1.0);
        if (i == kSnsKnock) sigma *= 1.0 + t.v[kSnsCKP] / 6000.0;    // mechanical noise floor grows with rpm
        if (sigma > 0) m += sigma * rng_.gauss();
        // a stuck sensor freezes at the moment it fails
        if (f == SensorFault::Stuck && prevFault_[i] != SensorFault::Stuck) stuck_[i] = m;
        if (f == SensorFault::Stuck) m = stuck_[i];
        prevFault_[i] = f;

        int status = kStatusOk;
        double v = 0;
        if (d.analog) {
            v = (i == kSnsO2) ? m : toVolts(d, m);
            if (f == SensorFault::Open) v = 0.0;
            if (f == SensorFault::Short) v = 5.0;
            v = clampv(v, 0.0, 5.0);
            v = std::round(v / 5.0 * 1023.0) / 1023.0 * 5.0;       // 10-bit ADC
            m = (i == kSnsO2) ? v : fromVolts(d, v);
            if (i == kSnsO2) { if (v > 1.2) status = kStatusHigh; }
            else if (v < 0.2) status = kStatusLow;
            else if (v > 4.8) status = kStatusHigh;
        } else {
            // digital (tooth wheel / switch): a broken wire means no signal at all
            if (f == SensorFault::Open || f == SensorFault::Short) { m = 0; status = kStatusNoSignal; }
            if (i == kSnsCKP && f == SensorFault::Noisy && rng_.uniform() < 0.004) { m = 0; status = kStatusNoSignal; }   // lost sync
            v = status == kStatusNoSignal ? 0.0 : 5.0;
        }
        r.measured[i] = m;
        r.volts[i] = v;
        r.status[i] = status;
        r.fallback[i] = status != kStatusOk;
        r.value[i] = r.fallback[i] ? d.fallback : m;
    }
    // knock window: the same sensor, attributed to the cylinder whose combustion it follows
    const double knockOffset = r.measured[kSnsKnock] - t.v[kSnsKnock];
    for (int i = 0; i < kMaxCyl; ++i) {
        r.knockCyl[i] = r.fallback[kSnsKnock] ? 0.0 : std::max(0.0, t.knockCyl[i] + knockOffset);
        r.knockSeq[i] = t.knockSeq[i];
        // per-cylinder thermocouples share the EGT channel's noise class (no separate fault injection)
        r.egtCyl[i] = p.egtPerCyl ? t.egtCyl[i] + 2.0 * clampv((double)p.sensors[kSnsEGT].noiseScale, 0.0, 10.0) * rng_.gauss() : 0.0;
    }
    init_ = true;
}

} // namespace es
