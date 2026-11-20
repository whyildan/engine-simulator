// Package 2: exhaust sensors. The wideband reads the gas in the pipe (misfire, rich products, scavenging, position
// before / after a turbo), EMAP and the drive pressure ratio (protection), EGT after the turbine and per cylinder.
// usage: exhaust_sensor_test
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include "ecu/CalibrationIO.h"
#include "ecu/EngineFile.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static void run(Simulator& sim, double s) {
    static std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }
template <class F> static double avg(Simulator& sim, double sec, F f) {
    double s = 0; int n = 0;
    for (double t = 0; t < sec; t += 0.02) { run(sim, 0.02); s += f(tel(sim)); ++n; }
    return s / std::max(1, n);
}
static void start(Simulator& sim, EngineParams& p) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (double t = 0; t < 4.0; t += 0.01) {
        run(sim, 0.01);
        Telemetry x = tel(sim);
        if (x.running && x.rpm > starterReleaseRpm(p, x.snsValue[kSnsECT])) break;
    }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, 1.5);
}
static void hold(Simulator& sim, EngineParams& p, double rpm, double thr, double settle) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = (float)thr; sim.setParams(p);
    run(sim, settle);
}
struct Lam { double pipe, cyl, sensor; };
static Lam lam(Simulator& sim, double sec) {
    Lam l;
    l.pipe = avg(sim, sec, [](const Telemetry& t) { return t.lambdaPipe; });
    l.cyl = avg(sim, sec, [](const Telemetry& t) { return t.lambda; });
    l.sensor = avg(sim, sec, [](const Telemetry& t) { return t.snsValue[kSnsWideband]; });
    return l;
}
static EngineParams still(int preset) {   // no auto-tune / closed loop: the fuelling stays what it is
    EngineParams p = makeCalibratedPreset(preset);
    p.cal.autoTune = 0; p.cal.closedLoop = 0;
    return p;
}

int main() {
    {
        std::printf("[wideband] K20A: steady, rich, misfiring\n");
        EngineParams p = still(0);
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 3000, 0.25, 4.0);
        Lam a = lam(sim, 2.0);
        std::printf("    part load: pipe %.3f  cylinders %.3f  sensor %.3f\n", a.pipe, a.cyl, a.sensor);
        CHECK(std::fabs(a.pipe - a.cyl) < 0.03, "steady part load: the pipe gas reads what the cylinders burn (%.3f / %.3f)", a.pipe, a.cyl);
        hold(sim, p, 4500, 1.0, 4.0);
        Lam b = lam(sim, 2.0);
        std::printf("    full load: pipe %.3f  cylinders %.3f  sensor %.3f\n", b.pipe, b.cyl, b.sensor);
        CHECK(b.cyl < 0.92 && std::fabs(b.pipe - b.cyl) < 0.04, "rich at full load reads rich - the CO / H2 still count as fuel (%.3f / %.3f)",
              b.pipe, b.cyl);
        // misfires: the soft limiter drops a share of the sparks - the unburned charges reach the sensor
        p.cal.softLimitBand = 600; p.cal.softLimitCut = 0.5f; p.cal.softLimitRetard = 0; p.revLimit = 4700;
        hold(sim, p, 4500, 1.0, 3.0);
        Lam c = lam(sim, 2.0);
        std::printf("    sparks cut: pipe %.3f  cylinders %.3f  sensor %.3f  (misfires %d)\n", c.pipe, c.cyl, c.sensor, tel(sim).misfires);
        CHECK(c.pipe > c.cyl + 0.08, "misfires read lean at the sensor (pipe %.3f vs cylinders %.3f)", c.pipe, c.cyl);
    }
    {
        std::printf("[overlap] I4 ITB race (big overlap) at full load, low rpm\n");
        EngineParams p = still(11);
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 3500, 1.0, 4.0);
        Lam a = lam(sim, 2.0);
        std::printf("    pipe %.3f  cylinders %.3f\n", a.pipe, a.cyl);
        // (port injection: what blows through on the overlap is mixture, not air - it reads like the trapped charge)
        CHECK(std::fabs(a.pipe - a.cyl) < 0.05, "port-injected mixture blown through reads close to the charge (%.3f vs %.3f)", a.pipe, a.cyl);
    }
    {
        std::printf("[turbo] I4 2.0 turbo: EMAP, drive pressure, EGT across the turbine, wideband position\n");
        EngineParams p = still(16);
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 5000, 1.0, 6.0);
        const double emap = avg(sim, 1.0, [](const Telemetry& t) { return t.snsValue[kSnsEMAP]; });
        const double map = avg(sim, 1.0, [](const Telemetry& t) { return t.mapKPa; });
        const double pre = avg(sim, 1.0, [](const Telemetry& t) { return t.snsValue[kSnsEGT]; });
        const double post = avg(sim, 1.0, [](const Telemetry& t) { return t.snsValue[kSnsEGT2]; });
        std::printf("    EMAP %.0f kPa  MAP %.0f kPa  ratio %.2f   EGT before %.0f C  after %.0f C\n", emap, map, emap / map, pre, post);
        CHECK(map > 140 && emap > map && emap / map < 2.6, "under boost the manifold before the turbine is above the boost (%.0f > %.0f kPa, ratio %.2f)",
              emap, map, emap / map);
        CHECK(pre - post > 15 && pre - post < 400, "the probe after the turbine reads lower: %.0f K", pre - post);
        const Telemetry x = tel(sim);
        std::printf("    turbine: drop %.0f K in the gas, compressor %.1f kW\n", x.turbineDropK, x.fiPowerKw);
        CHECK(x.turbineDropK > 40 && x.turbineDropK < 300, "the turbine takes its work out of the gas: %.0f K", x.turbineDropK);
        // wideband position: the gas needs longer to reach a sensor after the turbine (and mixes more on the way)
        auto delay = [](int pos) {
            EngineParams q = still(16);
            q.widebandPos = pos;
            Simulator s2(q);
            start(s2, q);
            hold(s2, q, 3500, 0.6, 2.0);
            return s2.engineForTests()->sensors.truth.lamDelay;
        };
        const double dc = delay(0), dd = delay(1);
        std::printf("    exhaust valve -> wideband: %.0f ms (collector) / %.0f ms (downpipe)\n", dc * 1e3, dd * 1e3);
        CHECK(dd > dc + 0.02, "the downpipe wideband sees the gas later than the collector one");
    }
    {
        std::printf("[protection] a choking turbine housing\n");
        EngineParams p = still(16);
        p.turbineAR = 0.45f; p.wastegateBar = 1.6f;
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 5500, 1.0, 6.0);
        Telemetry t = tel(sim);
        std::printf("    EMAP %.0f kPa  MAP %.0f kPa  ratio %.2f  rule state %d\n", t.snsValue[kSnsEMAP], t.mapKPa, t.driveRatio, t.protState[kProtBackpressure]);
        CHECK(t.protState[kProtBackpressure] >= 1, "drive pressure ratio %.2f: the protection rule warns / acts (state %d)", t.driveRatio,
              t.protState[kProtBackpressure]);
    }
    {
        std::printf("[EGT per cylinder] K20A, cylinder 2 retarded by 10 deg\n");
        EngineParams p = still(0);
        p.egtPerCyl = 1;
        p.cal.cylSpark[1] = -10;
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 4500, 1.0, 8.0);
        Telemetry t = tel(sim);
        int hot = 0;
        for (int i = 1; i < 4; ++i) if (t.egtCylC[i] > t.egtCylC[hot]) hot = i;
        std::printf("    %.0f / %.0f / %.0f / %.0f C   collector %.0f C\n", t.egtCylC[0], t.egtCylC[1], t.egtCylC[2], t.egtCylC[3], t.snsValue[kSnsEGT]);
        CHECK(hot == 1 && t.egtCylC[1] - std::max(t.egtCylC[0], t.egtCylC[2]) > 15,
              "the late-burning cylinder shows up hot (%.0f vs %.0f C) - the collector only sees the mix", t.egtCylC[1], t.egtCylC[0]);
    }
    {
        std::printf("[files] sensor fit-out in the engine file\n");
        EngineParams p = makePreset(16);
        p.emapSensor = 0; p.widebandPos = 0; p.egtPerCyl = 1; p.egtPostTurbo = 0;
        EngineParams q = makePreset(0); EngineMeta m; std::string err;
        engineFromJson(engineToJson(p, m), q, m, err);
        CHECK(q.emapSensor == 0 && q.widebandPos == 0 && q.egtPerCyl == 1 && q.egtPostTurbo == 0, "EMAP / wideband position / EGT probes saved");
    }
    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
