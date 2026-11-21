#pragma once
// "Factory" base map: an automated calibration run on a virtual motoring dyno. A separate copy of
// the engine is held at fixed speeds while the throttle is swept; the air the cylinders actually
// trap (what an engineer measures with lab equipment on a real dyno) is written into the VE maps.
// The running ECU still only ever uses its tables - this is the calibration process, offline.
#include "sim/EngineParams.h"
#include <atomic>
#include <string>
#include <cstdint>

namespace es {

struct BaseMapProgress {
    std::atomic<float> fraction{0.0f};
    std::atomic<bool> cancel{false};
};

// whether the engine has a VE base map to measure: the ECU's VE maps compute the injected fuel (speed density / alpha-N
// / hybrid). A carburettor meters its own fuel from the air through its venturi and a diesel fuels by quantity, so
// for those the maps are never used to fuel and the rig run would be wasted
inline bool baseMapApplies(const EngineParams& p) { return !isDiesel(p) && p.fuelSystem != FuelSystem::Carburetor; }

// fills out.veMap and out.veTps (values only, axes kept) from measurements; false if cancelled
bool measureBaseMap(const EngineParams& p, Calibration& out, BaseMapProgress* prog = nullptr, double fs = 96000.0);

// worker threads per base map (0 = automatic: cores - 2). A caller that already runs several base maps at once
// (basemap_warm) sets 1
void setBaseMapThreads(int n);

// on-disk cache of measured base maps (keyed by the engine definition)
void setBaseMapCacheDir(const std::string& dir);
bool loadCachedBaseMap(const EngineParams& p, Calibration& out);
void saveCachedBaseMap(const EngineParams& p, const Calibration& c);
// identity of everything the measured air depends on (changes -> the base map needs measuring again)
uint64_t baseMapKey(const EngineParams& p);

// preset with a measured base map (computed once per preset index per process, then cached)
EngineParams makeCalibratedPreset(int idx);

} // namespace es
