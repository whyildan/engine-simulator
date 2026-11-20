#pragma once
// Diesel combustion noise (Package 4c): cylinder pressure -> radiated sound pressure through the structure
// attenuation of an engine block, and the constants of the combustion chamber pressure ringing.
//
// Structure attenuation: the AVL generic curve as printed in Dernotte, Dec & Ji (Sandia, OSTI 1123537, Fig. 2; read
// off the plot, +-2 dB): -150 dB at 200 Hz, -103 / -94 / -90 / -91 / -94 / -99 / -105 / -111 / -118 / -124 / -132 dB at
// 0.5 / 1 / 2 / 3 / 4 / 5 / 6 / 7 / 8 / 9 / 10 kHz (the method follows Shahlari et al. and the AVL combustion noise
// meter: cylinder pressure spectrum minus the attenuation = combustion noise). Fitted with three 2nd-order high-passes
// at 600 Hz and three 2nd-order low-passes at 4800 Hz (Q 0.85 each) and -93.25 dB: 1.1 dB rms over those points.
//
// Ringing: Eng (SAE 2002-01-2859) as used by Dernotte, Dec & Ji (OSTI 1123537, eq. 2): peak-to-peak amplitude of the
// chamber pressure oscillation = beta (dP/dt)max, beta = 0.05 ms (established on HCCI combustion; used here for the
// diesel's premixed burn). Frequency of the first tangential mode f = 1.841 c / (pi B) (Draper; e.g. US 10443535,
// f = c alpha / (pi B), alpha_1,0 = 1.841).
#include "audio/Filters.h"
#include <cmath>

namespace es {

constexpr double kRingBeta = 0.05e-3;      // s
constexpr double kRingAlpha10 = 1.841;
constexpr double kRingDecay = 1.0e-3;      // s - chosen (not from a source)
constexpr double kStructGainDb = -93.25;

struct StructureAttenuation {
    Biquad hp[3], lp[3];
    double gain = 0;
    void init(double fs) {
        for (int k = 0; k < 3; ++k) { hp[k].highpass(600.0, 0.85, fs); lp[k].lowpass(4800.0, 0.85, fs); }
        gain = std::pow(10.0, kStructGainDb / 20.0);
    }
    double tick(double pa) {
        for (int k = 0; k < 3; ++k) pa = lp[k].tick(hp[k].tick(pa));
        return pa * gain;
    }
};

// first tangential acoustic mode of the chamber gas (Hz): bore m, gas temperature K (gamma ~1.3 for hot burned gas;
// the source's example uses 1.28 at 2000 K)
inline double ringFrequency(double boreM, double T, double R = 287.0) {
    return kRingAlpha10 * std::sqrt(1.3 * R * std::max(300.0, T)) / (3.14159265358979 * std::max(0.01, boreM));
}

} // namespace es
