#pragma once
// Virtual sensor catalogue (ids, ranges, defaults) and the per-sensor user configuration that is
// stored in EngineParams.
#include <cstdint>

namespace es {

enum SensorId : int {
    kSnsTPS, kSnsAPP, kSnsMAP, kSnsMAF, kSnsIAT, kSnsECT, kSnsO2, kSnsWideband, kSnsCKP, kSnsCMP,
    kSnsKnock, kSnsEGT, kSnsOilP, kSnsOilT, kSnsFuelP, kSnsVSS, kSnsGear, kSnsBattery, kSensorCount
};

enum class SensorFault : int32_t { None = 0, Open = 1, Short = 2, Stuck = 3, Noisy = 4 };

struct SensorConfig {
    float noiseScale;   // 1 = typical sensor noise, 0 = ideal
    float lag_ms;       // response time constant
    SensorFault fault;  // injected failure
};

struct SensorDef {
    const char* name;
    const char* unit;
    double lo, hi;        // measuring range (physical units) mapped to 0.5 .. 4.5 V
    double noise;         // typical 1-sigma noise (physical units)
    double lag_ms;        // typical response time
    double fallback;      // value the ECU substitutes when the signal is implausible
    bool analog;          // false: digital (frequency / switch) signal
    const char* role;     // what the ECU does with it today
};

const SensorDef& sensorDef(int id);
const char* sensorFaultNames();          // '\0'-separated, for combo boxes
void defaultSensorConfig(SensorConfig out[kSensorCount]);

} // namespace es
