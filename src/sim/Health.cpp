#include "sim/Health.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace es {

namespace {
const char* kPartNames[kPartCount] = {"Air filter", "Compressor", "Intercooler", "Blow-off valve", "Throttle", "Plenum",
                                      "Cylinder head", "Spark plugs", "Header", "Turbine", "Catalyst", "Muffler"};
const char* kItemNames[kHealthItemCount] = {"Check engine", "Knock", "Spark plugs", "Sensors", "Valves", "Rods", "Crank",
                                            "Pistons", "Repair"};

void setValue(HealthEntry& e, const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); std::vsnprintf(e.value, sizeof(e.value), fmt, ap); va_end(ap);
}
// one reason: raises the level and adds a line to the tooltip
void reason(HealthEntry& e, int level, const char* fmt, ...) {
    e.level = std::max(e.level, level);
    const size_t n = std::strlen(e.detail);
    if (n + 2 >= sizeof(e.detail)) return;
    char* d = e.detail + n;
    size_t room = sizeof(e.detail) - n;
    if (n) { *d++ = '\n'; --room; }
    static const char* tag[] = {"ok", "WARNING", "LIMIT"};
    const int k = std::snprintf(d, room, "[%s] ", tag[clampv(level, 0, 2)]);
    if (k > 0 && (size_t)k < room) { va_list ap; va_start(ap, fmt); std::vsnprintf(d + k, room - k, fmt, ap); va_end(ap); }
}
void ok(HealthEntry& e) { e.level = std::max(e.level, (int)kHealthOk); }

// a protection rule: the calibration's warning / limit on a reading, and the ECU's own verdict on it
int ruleLevel(const EngineParams& p, const Telemetry& t, int rule, double v) {
    const ProtRule& r = p.cal.prot[rule];
    const bool high = protInfo(rule).high;
    int lv = healthLevel(v, r.warn, r.limit, high);
    return std::max(lv, clampv(t.protState[rule], 0, 2));
}
} // namespace

const char* flowPartName(int part) { return kPartNames[clampv(part, 0, kPartCount - 1)]; }
const char* healthItemName(int item) { return kItemNames[clampv(item, 0, kHealthItemCount - 1)]; }
const char* healthItemName(int item, const EngineParams& p) {
    if (item == kHealthPlugs && isDiesel(p)) return "Glow plugs";
    if (item == kHealthValves && (isTwoStroke(p) || isRotary(p))) return "Ports";
    return healthItemName(item);
}

HealthReport evaluateHealth(const EngineParams& p, const Telemetry& t) {
    HealthReport h;
    const bool boosted = isBoosted(p), turbo = p.induction == Induction::Turbo;
    const bool centrifugal = turbo || (p.induction == Induction::Supercharger && p.scType == Supercharger::Centrifugal);
    const bool rotary = isRotary(p);
    const int ports = std::max(1, std::min((int)p.cylinders, kMaxCyl));
    const double ambC = t.flow.tAmb - 273.15;
    ComponentModel cm;
    cm.build(p);

    // ---------------------------------------------------------------- gas path
    {   // air filter: nothing to judge (its restriction is part of the intake model)
        HealthEntry& e = h.part[kPartFilter];
        setValue(e, "%.1f kPa  %.0f C", t.flow.pAmb, ambC);
    }
    if (boosted) {
        HealthEntry& e = h.part[kPartCompressor];
        ok(e);
        // top speed: the highest speed line of the compressor map (turbo / centrifugal), the blower's rated speed
        reason(e, healthLevel(t.speedFrac, 0.95, 1.0), "%s speed %.0f rpm = %.0f %% of its top speed (warning 95 %%, limit 100 %%)",
               turbo ? "Shaft" : "Rotor", t.shaftRpm, t.speedFrac * 100);
        if (t.surging) reason(e, kHealthLimit, "SURGE: the flow stalls and reverses (throttle shut against boost)");
        else if (centrifugal && t.boostBar > 0.1 && t.surgeMargin < 0.05)
            reason(e, kHealthWarn, "Close to the surge line (margin %.0f %%)", t.surgeMargin * 100);
        setValue(e, "%.0fk rpm  PR %.2f", t.shaftRpm * 1e-3, t.pr);
    }
    if (boosted && p.intercooler) {
        HealthEntry& e = h.part[kPartIntercooler];
        ok(e);
        // heat soak: the core warms up when it gets heat faster than the air blown through it takes it away
        const double soak = t.icCoreC - ambC;
        reason(e, healthLevel(soak, 20.0, 40.0), "Core %.0f K above ambient: %s (warning 20 K, limit 40 K)", soak,
               soak >= 20.0 ? "heat soak - the charge gets hotter" : "cooling well");
        setValue(e, "core %.0f C  out %.0f C", t.icCoreC, t.chargeT);
    }
    if ((turbo && p.bov) || (p.induction == Induction::Supercharger && p.scBypass)) {
        HealthEntry& e = h.part[kPartBov];
        ok(e);
        setValue(e, "%s", t.bovOpen ? "open (venting)" : "closed");
    }
    {
        HealthEntry& e = h.part[kPartThrottle];
        ok(e);
        if (t.shutdown || t.limp) reason(e, kHealthLimit, "%s: the throttle is held back", t.shutdown ? "Engine shutdown" : "Limp mode");
        else if (t.throttleLimited) reason(e, kHealthLimit, "Limited by a protection action (throttle limit)");
        if (t.tqThrCap < 0.995f) reason(e, kHealthWarn, "Torque management caps the throttle at %.0f %%", t.tqThrCap * 100);
        if (t.snsFallback[kSnsTPS] || t.snsFallback[kSnsAPP])
            reason(e, kHealthWarn, "%s sensor fault: the ECU uses a fallback", t.snsFallback[kSnsTPS] ? "Throttle position" : "Pedal position");
        if (!e.detail[0]) reason(e, kHealthOk, "Free: the throttle follows the pedal");
        setValue(e, "%.0f %%", t.throttle * 100);
    }
    {
        HealthEntry& e = h.part[kPartPlenum];
        ok(e);
        if (boosted) {
            const int lv = ruleLevel(p, t, kProtOverboost, t.boostBar);
            reason(e, lv, "Boost %.2f bar (overboost rule: warning %.2f, limit %.2f)", t.boostBar, p.cal.prot[kProtOverboost].warn,
                   p.cal.prot[kProtOverboost].limit);
            if (t.overboostCut || t.boostCut) reason(e, kHealthLimit, "Boost cut by the ECU");
        }
        if (t.snsFallback[kSnsMAP]) reason(e, kHealthWarn, "MAP sensor fault: the ECU runs on alpha-N");
        setValue(e, p.itb ? "%.0f kPa (runners)" : "%.0f kPa  %.0f C", t.flow.pPlenum, t.flow.tPlenum - 273.15);
    }
    double maxRetard = 0;
    int knockCyl = 0;
    for (int i = 0; i < ports; ++i) if (t.knockRetard[i] > maxRetard) { maxRetard = t.knockRetard[i]; knockCyl = i; }
    const double authority = std::max(1.0f, p.cal.knockMaxRetard);
    // knock: retard pulled by the knock control against its authority (out of retard = the knock is not under control)
    auto knockLevel = [&](double r) { return healthLevel(r, std::max(1.0, 0.4 * authority), 0.9 * authority); };
    for (int i = 0; i < ports; ++i) h.cyl[i] = t.running ? knockLevel(t.knockRetard[i]) : kHealthOk;
    {
        HealthEntry& e = h.part[kPartHead];
        ok(e);
        if (maxRetard > 0.05)
            reason(e, knockLevel(maxRetard), "Knock retard %.1f deg on %s %d of %.0f deg authority (warning 40 %%, limit 90 %%)", maxRetard,
                   rotary ? "rotor" : "cylinder", knockCyl + 1, authority);
        if (t.knock > 1.0f) reason(e, kHealthWarn, "Knocking now (knock index %.2f)", t.knock);
        if (t.protState[kProtKnock]) reason(e, t.protState[kProtKnock], "Excessive-knock protection %s", t.protState[kProtKnock] == 2 ? "active" : "warning");
        if (!rotary) {
            const double fl = std::max(t.floatIn, t.floatEx);
            if (fl > 0.05) reason(e, healthLevel(fl, 0.05, 0.5), "Valve float %.0f %% (%s)", fl * 100, t.floatIn >= t.floatEx ? "intake" : "exhaust");
        }
        if (!e.detail[0]) reason(e, kHealthOk, "No knock, valves follow the cams");
        setValue(e, "retard %.1f deg", maxRetard);
    }
    {
        HealthEntry& e = h.part[kPartPlugs];
        ok(e);
        if (isDiesel(p)) {   // a diesel has glow plugs: their state, not a spark plug's
            if (!p.glowPlugs) reason(e, kHealthNone, "No glow plugs fitted (the compressed air alone lights the fuel)");
            else if (t.glowLight) reason(e, kHealthWarn, "Pre-heating: wait for the glow lamp before cranking");
            else reason(e, kHealthOk, t.glowOn ? "Heating (post-heat after a cold start)" : "Off - the engine is warm enough");
            setValue(e, p.glowPlugs ? "tip %.0f C%s" : "none", t.glowTipC, t.glowOn ? " on" : "");
            goto plugsDone;
        }
        // above ~850 C the tip itself lights the charge (pre-ignition); fouling makes plugs misfire (0.6 x fouling)
        reason(e, healthLevel(t.plugTempC, 850.0, 950.0), "Tip %.0f C (pre-ignition from 850 C: warning 850, limit 950)", t.plugTempC);
        if (t.fouling > 0.02f) reason(e, healthLevel(t.fouling, 0.15, 0.4), "Fouling %.0f %%: carbon from rich running on a cold plug (misfires)", t.fouling * 100);
        if (t.running && t.sparkMargin < 9.0f)
            reason(e, healthLevel(t.sparkMargin, 1.25, 1.0, false), "Spark energy %.2fx what the charge needs (below 1: misfires)", t.sparkMargin);
        if (t.fouling > 0.02f) setValue(e, "%.0f C  %.0f %% fouled", t.plugTempC, t.fouling * 100);
        else setValue(e, "tip %.0f C", t.plugTempC);
    plugsDone:;
    }
    {
        HealthEntry& e = h.part[kPartHeader];
        ok(e);
        double egt = t.egt;
        int hot = -1;
        if (t.egtPerCyl) for (int i = 0; i < ports; ++i) if (t.egtCylC[i] > egt) { egt = t.egtCylC[i]; hot = i; }
        const ProtRule& r = p.cal.prot[kProtEgt];
        reason(e, ruleLevel(p, t, kProtEgt, egt), "EGT %.0f C%s (EGT rule: warning %.0f, limit %.0f)", egt,
               hot >= 0 ? " (hottest primary)" : "", r.warn, r.limit);
        if (hot >= 0) setValue(e, "EGT %.0f C (#%d)", egt, hot + 1); else setValue(e, "EGT %.0f C", egt);
    }
    if (turbo) {
        HealthEntry& e = h.part[kPartTurbine];
        ok(e);
        const ProtRule& r = p.cal.prot[kProtEgt];
        reason(e, ruleLevel(p, t, kProtEgt, t.egt), "Inlet EGT %.0f C (EGT rule: warning %.0f, limit %.0f)", t.egt, r.warn, r.limit);
        // drive pressure: exhaust manifold over boost pressure - judged like the ECU's rule, under boost only
        const double dr = t.flow.pTurbIn / std::max(10.0, t.flow.pPlenum);
        if (t.boostBar > 0.1f) {
            const ProtRule& b = p.cal.prot[kProtBackpressure];
            reason(e, ruleLevel(p, t, kProtBackpressure, dr), "Drive pressure %.2f (EMAP %.0f / MAP %.0f kPa; rule: warning %.1f, limit %.1f)",
                   dr, t.flow.pTurbIn, t.flow.pPlenum, b.warn, b.limit);
        }
        setValue(e, "ER %.2f  WG %.0f %%", t.turbEr, t.wastegate * 100);
    }
    if (p.cat) setValue(h.part[kPartCat], "%.0f C", t.flow.tPostTurb - 273.15);
    if (p.muffler != Muffler::None) setValue(h.part[kPartMuffler], "%.1f g/s", t.flow.fwd[kFlowTail] * 1e3);

    // ---------------------------------------------------------------- health panel
    {
        HealthEntry& e = h.item[kHealthCel];
        ok(e);
        int codes = 0;
        if (t.shutdown) reason(e, kHealthLimit, "ENGINE SHUTDOWN by protection");
        else if (t.limp) reason(e, kHealthLimit, "LIMP MODE");
        if (t.syncLost) { reason(e, kHealthLimit, "No crank sync: spark and fuel off"); ++codes; }
        for (int k = 0; k < kProtCount; ++k) {
            if (!t.protState[k]) continue;
            ++codes;
            reason(e, t.protState[k], "%s: %s", protInfo(k).name, t.protState[k] == 2 ? protActionName(p.cal.prot[k].action) : "warning");
        }
        for (int k = 0; k < kSensorCount; ++k)
            if (t.snsFallback[k]) { ++codes; reason(e, kHealthWarn, "%s fault: fallback value", sensorDef(k).name); }
        if (!codes && e.level == kHealthOk) reason(e, kHealthOk, "No fault codes");
        if (t.shutdown) setValue(e, "SHUTDOWN"); else if (t.limp) setValue(e, "LIMP");
        else setValue(e, codes ? "%d code%s" : "no codes", codes, codes == 1 ? "" : "s");
    }
    {
        HealthEntry& e = h.item[kHealthKnock];
        ok(e);
        if (maxRetard > 0.05) reason(e, knockLevel(maxRetard), "Retard %.1f deg on %s %d (authority %.0f deg)", maxRetard,
                                     rotary ? "rotor" : "cylinder", knockCyl + 1, authority);
        if (t.knock > 1.0f) reason(e, kHealthWarn, "Knocking now (index %.2f)", t.knock);
        if (t.protState[kProtKnock]) reason(e, t.protState[kProtKnock], "Excessive-knock protection");
        if (t.snsFallback[kSnsKnock]) reason(e, kHealthWarn, "Knock sensor fault: safety retard");
        if (!e.detail[0]) reason(e, kHealthOk, "No knock");
        setValue(e, maxRetard > 0.05 ? "-%.1f deg #%d" : "none", maxRetard, knockCyl + 1);
    }
    h.item[kHealthPlugs] = h.part[kPartPlugs];
    {
        HealthEntry& e = h.item[kHealthSensors];
        ok(e);
        int bad = 0, hidden = 0;
        for (int k = 0; k < kSensorCount; ++k) {
            if (t.snsFallback[k]) { ++bad; reason(e, kHealthLimit, "%s: fault detected, fallback in use", sensorDef(k).name); }
            else if (p.sensors[k].fault != SensorFault::None) {
                ++hidden;
                reason(e, kHealthWarn, "%s: %s fault injected, not detected by the ECU", sensorDef(k).name,
                       p.sensors[k].fault == SensorFault::Stuck ? "stuck" : p.sensors[k].fault == SensorFault::Noisy ? "noisy" : "wiring");
            }
        }
        if (!bad && !hidden) reason(e, kHealthOk, "All sensors plausible");
        setValue(e, bad + hidden ? "%d fault%s" : "all ok", bad + hidden, bad + hidden == 1 ? "" : "s");
    }
    {
        HealthEntry& e = h.item[kHealthValves];
        if (isTwoStroke(p)) {   // no valves: the piston opens its ports
            ok(e);
            reason(e, kHealthOk, "Piston-controlled ports - last cycle: delivery ratio %.2f, trapping %.2f, scavenging %.2f",
                   t.tsDelivery, t.tsTrapping, t.tsScavenging);
            if (p.tsPowerValve) reason(e, kHealthOk, "Power valve %.0f %% lifted", t.tsPowerValve * 100.0f);
            if (p.tsOilRatio > 0)
                reason(e, p.tsOilRatio > 100.0f ? kHealthWarn : kHealthOk, "Premix %.0f:1%s - unburned oil out of the exhaust %.0f g/h",
                       p.tsOilRatio, p.tsOilRatio > 100.0f ? " (leaner on oil than any maker's recommendation: piston scuffing risk)" : "",
                       t.tsOilSmokeGh);
            else reason(e, kHealthOk, "Oil pump (no oil in the fuel)");
            setValue(e, "ports  DR %.2f", t.tsDelivery);
        } else if (!rotary) {
            ok(e);
            const double fl = std::max(t.floatIn, t.floatEx);
            reason(e, healthLevel(fl, 0.05, 0.5), "Float %.0f %% now; floats from %.0f rpm (intake) / %.0f rpm (exhaust)", fl * 100,
                   cm.floatRpmIn, cm.floatRpmEx);
            const float bounce = std::max(t.bounceIn, t.bounceEx);
            if (bounce > 0.1f) reason(e, kHealthWarn, "Valve bounces %.2f mm off its seat", bounce);
            if (t.failure == 4) reason(e, kHealthLimit, "A valve hit the piston");
            setValue(e, "float %.0f %%", fl * 100);
        } else setValue(e, "ports (rotary)");
    }
    static const char* kPartWord[3] = {"pistons", "rods", "crankshaft"};
    const int partItem[3] = {kHealthPistons, kHealthRods, kHealthCrank};
    for (int k = 0; k < 3; ++k) {
        HealthEntry& e = h.item[partItem[k]];
        if (rotary) { setValue(e, k == 2 ? "eccentric shaft" : "rotary: none"); continue; }
        ok(e);
        const double safe = cm.partSafeRpm[k];
        const double s = safe > 1 ? (t.rpm / safe) * (t.rpm / safe) : 0.0;
        h.partStress[k] = s;
        // inertia load against what the part takes (85 %: within a few hundred rpm of breaking)
        reason(e, healthLevel(s, 0.85, 1.0), "Inertia load %.0f %% of the %s limit (safe to %.0f rpm)", s * 100, kPartWord[k], safe);
        setValue(e, "%.0f %% load", s * 100);
    }
    if (!rotary) {
        if (t.failure == 1 || t.failure == 5) reason(h.item[kHealthPistons], kHealthLimit, "%s", t.failure == 5 ? "Piston destroyed by knock" : "Piston failure");
        else if (t.knock > 1.0f && p.comp.allowFailure) reason(h.item[kHealthPistons], kHealthWarn, "Knocking: the pistons take damage");
        if (t.failure == 2) reason(h.item[kHealthRods], kHealthLimit, "Connecting rod failure");
        if (t.failure == 3) reason(h.item[kHealthCrank], kHealthLimit, "Crankshaft failure");
    }
    {
        HealthEntry& e = h.item[kHealthRepair];
        ok(e);
        static const char* what[] = {"", "piston failure", "connecting rod failure", "crankshaft failure", "valve hit the piston",
                                     "piston destroyed by knock"};
        if (t.failure) { reason(e, kHealthLimit, "ENGINE FAILED: %s - repair it", what[clampv(t.failure, 0, 5)]); setValue(e, "FAILED"); }
        else if (!p.comp.allowFailure) { reason(e, kHealthOk, "Mechanical damage is off (Components)"); setValue(e, "damage off"); }
        else if (t.damage > 0.001f) {
            reason(e, healthLevel(t.damage, 0.0, 0.5), "Accumulated damage %.0f %% (fails at 100 %%)", t.damage * 100);
            setValue(e, "damage %.0f %%", t.damage * 100);
        } else { reason(e, kHealthOk, "No damage"); setValue(e, "no damage"); }
    }
    return h;
}

} // namespace es
