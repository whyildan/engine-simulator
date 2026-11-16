// Where does an engine misfire and pop? Runs a preset through idle, cruise, blips, full load and a closed-throttle
// decel, and reports per phase: misfires (by cause) and afterfire pops.
// usage: misfire_probe [preset] [idle seconds]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace es;

static void run(Simulator& sim, double s) {
    static std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }

struct Snap { int mis, pops, cause[Engine::kMisCauseCount]; };
static Snap snap(Simulator& sim) {
    Snap s; Telemetry t = tel(sim); s.mis = t.misfires; s.pops = t.afterfires;
    const Engine* e = sim.engineForTests();
    for (int k = 0; k < Engine::kMisCauseCount; ++k) s.cause[k] = e ? e->misfireCause[k] : 0;
    return s;
}
static void report(const char* phase, const Snap& a, const Snap& b, Simulator& sim) {
    static const char* names[Engine::kMisCauseCount] = {"lean", "spark", "fouled", "rich", "dilution"};
    Telemetry t = tel(sim);
    std::printf("  %-28s misfires %4d (", phase, b.mis - a.mis);
    for (int k = 0; k < Engine::kMisCauseCount; ++k) std::printf("%s %d%s", names[k], b.cause[k] - a.cause[k], k + 1 < Engine::kMisCauseCount ? ", " : "");
    std::printf(")  pops %4d   | rpm %.0f AFR %.2f adv %.1f EGT %.0f\n", b.pops - a.pops, t.rpm, t.afr, t.advance, t.snsValue[kSnsEGT]);
}

int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 0;
    const double idleS = argc > 2 ? std::atof(argv[2]) : 20.0;
    EngineParams p = std::getenv("GENERIC") ? makePreset(preset) : makeCalibratedPreset(preset);   // GENERIC: no measured base map
    if (const char* e = std::getenv("EXIT_ENRICH")) p.cal.dfcoExitEnrich = (float)std::atof(e);
    if (const char* e = std::getenv("EXIT_DECAY")) p.cal.dfcoExitDecay = (float)std::atof(e);
    if (const char* e = std::getenv("RESUME")) p.cal.dfcoResumeAbove = (float)std::atof(e);
    if (const char* e = std::getenv("ANTIC")) p.cal.dfcoAnticipate = (float)std::atof(e);
    if (const char* e = std::getenv("DELAY")) p.cal.dfcoDelay = (float)std::atof(e);
    if (const char* e = std::getenv("FLY")) p.flywheel = (float)std::atof(e);
    if (std::getenv("NOTUNE")) { p.cal.autoTune = 0; p.cal.closedLoop = 1; }
    if (const char* e = std::getenv("IDLE")) for (int i = 0; i < p.cal.idleTarget.n; ++i) p.cal.idleTarget.v[i] += (float)std::atof(e);
    if (const char* e = std::getenv("IKP")) p.cal.idleKp = (float)std::atof(e);
    if (const char* e = std::getenv("IKI")) p.cal.idleKi = (float)std::atof(e);
    if (const char* e = std::getenv("IKD")) p.cal.idleKd = (float)std::atof(e);
    if (const char* e = std::getenv("ISG")) p.cal.idleSparkGain = (float)std::atof(e);
    if (const char* e = std::getenv("ASYNC")) p.cal.asyncTipIn = std::atoi(e);
    if (const char* e = std::getenv("AE")) for (int i = 0; i < p.cal.accel.n; ++i) p.cal.accel.v[i] *= (float)std::atof(e);
    if (const char* e = std::getenv("IGNOFS")) for (auto& row : p.cal.ignition.v) for (float& v : row) v += (float)std::atof(e);
    std::printf("[%s] rev limit %.0f\n", p.name, p.revLimit);
    Simulator sim(p);
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 400; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
    run(sim, 0.15); p.starter = 0; sim.setParams(p);
    Snap a = snap(sim);
    {   // idle: per-cylinder lambda spread (sampled)
        double lo[kMaxCyl], hi[kMaxCyl], sum[kMaxCyl] = {}; int n = 0;
        double rs = 0, rs2 = 0, rmin = 1e9, rmax = 0; int rn = 0;
        for (int i = 0; i < kMaxCyl; ++i) { lo[i] = 9; hi[i] = 0; }
        const int nc = std::min(p.cylinders, kMaxCyl);
        for (double t = 0; t < idleS; t += 0.02) {
            run(sim, 0.02); Telemetry x = tel(sim);
            if (std::getenv("IDLETRACE") && std::fmod(t, 2.0) < 0.02)
                std::printf("    idle t %4.1f  rpm %5.0f map %5.1f afr %5.2f tgt %5.2f iac %4.1f adv %5.1f stft %+5.1f ltft %+5.1f tune %d vtc %4.1f ect %3.0f\n",
                            t, x.rpm, x.mapKPa, x.afr, x.targetAfr, x.iac, x.advance, x.ecuStft, x.ecuLtft, (int)x.tuning, x.vtcAdvance, x.coolantC);
            if (t < 2) { if (t + 0.02 >= 2) { Snap s2 = snap(sim); report("first 2 s after start", a, s2, sim); a = s2; } continue; }
            rs += x.rpm; rs2 += x.rpm * x.rpm; rmin = std::min(rmin, (double)x.rpm); rmax = std::max(rmax, (double)x.rpm); ++rn;
            for (int i = 0; i < nc; ++i) { lo[i] = std::min(lo[i], (double)x.cylLambda[i]); hi[i] = std::max(hi[i], (double)x.cylLambda[i]); sum[i] += x.cylLambda[i]; }
            ++n;
        }
        { const double m = rs / std::max(1, rn); std::printf("  idle rpm mean %.0f  sd %.0f  min %.0f  max %.0f\n", m, std::sqrt(std::max(0.0, rs2 / std::max(1, rn) - m * m)), rmin, rmax); }
        std::printf("  idle lambda per cylinder (min / mean / max):");
        for (int i = 0; i < nc; ++i) std::printf("  %.2f/%.2f/%.2f", lo[i], sum[i] / std::max(1, n), hi[i]);
        std::printf("\n");
    }
    Snap b = snap(sim); report("idle", a, b, sim);
    a = b;
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 2500; p.throttle = 0.15f; sim.setParams(p);
    run(sim, 6.0);
    b = snap(sim); report("cruise 2500 rpm 15 %", a, b, sim);
    a = b;
    p.dynoMode = DynoMode::Off; p.throttle = 0; sim.setParams(p); run(sim, 3.0);
    a = snap(sim);
    if (std::getenv("BLIPDBG")) {   // one blip, printing every step where a misfire happens
        int m0 = tel(sim).misfires;
        for (double t = 0; t < 3.0; t += 0.005) {
            p.throttle = t < 0.35 ? 1.0f : 0.0f; sim.setParams(p); run(sim, 0.005);
            Telemetry x = tel(sim);
            if (x.misfires != m0) {
                std::printf("    t %.3f  +%d  rpm %5.0f tps %3.0f map %5.1f ecuAFR %5.2f afr %5.2f lam", t, x.misfires - m0, x.rpm, x.throttle * 100, x.mapKPa, x.targetAfr, x.afr);
                for (int i = 0; i < p.cylinders; ++i) std::printf(" %.2f", x.cylLambda[i]);
                std::printf("  duty %.3f ecuVE %.1f ecuAir %.1f mg trueVE %.1f", x.injDuty, x.ecuVe, x.ecuAirMg, x.ve * 100);
                std::printf("\n");
                m0 = x.misfires;
            }
        }
    }
    for (int k = 0; k < 5; ++k) { p.throttle = 1; sim.setParams(p); run(sim, 0.35); p.throttle = 0; sim.setParams(p); run(sim, 2.5); }
    b = snap(sim); report("5 blips (WOT 0.35 s)", a, b, sim);
    a = b;
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 0.75f * p.revLimit; p.throttle = 1; sim.setParams(p);
    run(sim, 4.0);
    b = snap(sim); report("WOT at 75 % of the limit", a, b, sim);
    { Telemetry x = tel(sim);
      std::printf("    vtec %.2f (cmd %d) vtc %.1f/%.1f deg  map %.1f  ecuVE %.1f trueVE %.1f  target AFR %.2f  STFT %+.1f  thr %.2f limited %d lean-prot %d\n",
                  x.vtecEngaged, (int)x.vtecCmd, x.vtcAdvance, x.vtcTarget, x.mapKPa, x.ecuVe, x.ve * 100, x.targetAfr, x.ecuStft, x.throttle, (int)x.throttleLimited, x.protState[kProtLean]);
      std::printf("    protection:"); for (int k = 0; k < kProtCount; ++k) if (x.protState[k]) std::printf(" %s=%d (%.3g)", protInfo(k).key, x.protState[k], x.protValue[k]);
      float kr = 0; for (int k = 0; k < 4; ++k) kr = std::max(kr, x.knockRetard[k]);
      std::printf("   knock %.2f retard %.1f  float %.2f/%.2f\n", x.knock, kr, x.floatIn, x.floatEx); }
    a = b;
    p.throttle = 0; sim.setParams(p); run(sim, 0.01);
    p.dynoMode = DynoMode::Off; sim.setParams(p);
    run(sim, 4.0);
    b = snap(sim); report("closed-throttle decel", a, b, sim);
    return 0;
}
