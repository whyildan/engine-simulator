#pragma once
// Diesel common rail injector (D2): the nozzle holes are orifices fed at the rail pressure,
//   mdot = Cd A sqrt(2 rho (p_rail - p_cyl))       (A: all the holes' area; Payri et al., IJER 17(10) 2016, eqs. 4-5)
// with the needle lifting at a fixed rate while the injector is energized (full lift after kNeedleRampS) and closing
// at the same rate once it is not. The ECU decides each pulse's energizing time from its quantity, the rail pressure
// and an assumed chamber pressure; the engine delivers what the actual pressures give.
#include "sim/EngineParams.h"
#include <algorithm>
#include <cmath>

namespace es {

// discharge coefficient of a hole: 0.86 - ECN "Spray A" (Sandia), 0.090 mm hydro-eroded hole with diesel at 10 MPa
constexpr double kNozzleCd = 0.86;
constexpr double kNeedleRampS = 0.15e-3;    // needle full-lift time (chosen: no citable figure was found - D2)
constexpr double kInjAssumedCylBar = 60.0;  // chamber pressure the ECU assumes for the energizing time (chosen)

inline double nozzleAreaM2(const EngineParams& p) {
    const double d = std::max(0.01, (double)p.nozzleHoleDia_mm) * 1e-3;
    return std::max(1, (int)p.nozzleHoles) * 0.25 * 3.14159265358979 * d * d;
}
// full-lift flow, kg/s
inline double nozzleFlow(const EngineParams& p, double rhoFuel, double railPa, double cylPa) {
    return kNozzleCd * nozzleAreaM2(p) * std::sqrt(2.0 * rhoFuel * std::max(0.0, railPa - cylPa));
}
// needle lift share at `t` seconds after the start of a pulse energized for `dur` seconds: rising at 1 / ramp while
// energized (capped at full lift), then falling at the same rate from the lift reached. A long pulse delivers
// full x dur; a short one (ballistic: the needle never reaches full lift) full x dur^2 / ramp.
inline double needleOpen(double t, double dur) {
    if (t < 0 || dur <= 0) return 0.0;
    const double top = std::min(1.0, dur / kNeedleRampS);
    if (t <= dur) return std::min(1.0, t / kNeedleRampS);
    return std::max(0.0, top - (t - dur) / kNeedleRampS);
}
// the pulse is over (the needle is back on its seat)
inline bool needleClosed(double t, double dur) { return t > dur + std::min(1.0, dur / kNeedleRampS) * kNeedleRampS; }
// energizing time for a quantity at a full-lift flow (the inverse of the above)
inline double energizingFor(double q, double fullFlow) {
    if (q <= 0 || fullFlow <= 0) return 0.0;
    if (q >= fullFlow * kNeedleRampS) return q / fullFlow;
    return std::sqrt(q * kNeedleRampS / fullFlow);
}

} // namespace es
