#pragma once
// Package 3: engine health for the flow map and its health panel. Every component on the gas path and every internal
// part gets a level - green (ok), yellow (warning), red (at the limit) - with the reading that decides it. The
// thresholds are not new numbers: they come from the protection rules of the calibration (EGT, overboost, drive
// pressure, knock) and from the component limits (ComponentModel: safe rpm per part, valve float), plus a few that
// follow the physics directly (spark plug pre-ignition / fouling, turbo top speed and surge, intercooler heat soak).
#include "sim/Simulator.h"

namespace es {

enum HealthLevel : int { kHealthNone = -1, kHealthOk = 0, kHealthWarn = 1, kHealthLimit = 2 };

// components drawn on the flow map
enum FlowPart : int { kPartFilter, kPartCompressor, kPartIntercooler, kPartBov, kPartThrottle, kPartPlenum, kPartHead,
                      kPartPlugs, kPartHeader, kPartTurbine, kPartCat, kPartMuffler, kPartCount };
// tiles of the health panel
enum HealthItem : int { kHealthCel, kHealthKnock, kHealthPlugs, kHealthSensors, kHealthValves, kHealthRods, kHealthCrank,
                        kHealthPistons, kHealthRepair, kHealthItemCount };

struct HealthEntry {
    int level = kHealthNone;    // kHealthNone: not fitted / nothing to judge (drawn neutral)
    char value[64] = {};        // short reading (tile / label)
    char detail[320] = {};      // what decides the colour (tooltip), one reason per line
};

struct HealthReport {
    HealthEntry part[kPartCount];
    HealthEntry item[kHealthItemCount];
    int cyl[kMaxCyl] = {};      // per cylinder / rotor: knock retard against the knock control's authority
    double partStress[3] = {};  // inertia load / limit at this rpm: piston, rod, crank
};

const char* flowPartName(int part);
const char* healthItemName(int item);
const char* healthItemName(int item, const EngineParams& p);   // (a diesel's glow plugs, a two-stroke's ports)
HealthReport evaluateHealth(const EngineParams& p, const Telemetry& t);

// level of a reading against a warning and a limit threshold (high = the value is bad when high)
inline int healthLevel(double v, double warn, double limit, bool high = true) {
    if (high) return v >= limit ? kHealthLimit : v >= warn ? kHealthWarn : kHealthOk;
    return v <= limit ? kHealthLimit : v <= warn ? kHealthWarn : kHealthOk;
}

} // namespace es
