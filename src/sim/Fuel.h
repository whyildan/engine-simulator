#pragma once
// Fuels: gasoline of a given octane, ethanol and methanol, and their blends. The properties that matter to the
// engine come from the blend (by volume, as it is pumped): stoichiometric AFR, heating value, heat of vaporisation,
// density, octane (RON / MON), flame speed and how badly it vaporises on cold walls.
//
// Sources (ranges; the values below sit inside them):
//   IEA AMF fuel information (ethanol, methanol properties); NREL/TP-5400-63091 (heat of vaporisation of ethanol
//   blends); SAE 2010-01-2112 (stoichiometric AFR of ethanol-gasoline blends); Szybist, ORNL (knock limit of E85).
//   Gasoline: AFR 14.6-14.7, LHV 42.7-43.8 MJ/kg, HoV 305-380 kJ/kg, 0.72-0.775 kg/L.
//   Ethanol:  AFR 9.0, LHV 26.8-27.0 MJ/kg, HoV 900-920 kJ/kg, 0.789 kg/L, RON 108-109, MON 89-90.
//   Methanol: AFR 6.45, LHV 19.9-20.1 MJ/kg, HoV 1100-1170 kJ/kg, 0.792 kg/L, RON 108.7, MON 88.6.
//   Diesel (Package 4): LHV 42.79 MJ/kg and 3167 g/gal = 0.837 kg/L (GREET 1.8d.1, U.S. conventional diesel);
//   AFR 14.5 and CH1.80 (MIT 2.61 lecture 15 table; Colorado State: C12.5H22.2, AFR 14.5); HoV 225-280 kJ/kg
//   (IEA-AMF diesel fuel properties); cetane number >= 51 (EN 590), >= 40 (ASTM D975-24 grades 1-D / 2-D);
//   EN 590 density 0.820-0.845 kg/L. Diesel only runs in a diesel engine and a diesel only on diesel (no blends).
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace es {

// what is in the tank / fuel line, by volume (fractions add up to 1)
struct FuelBlend {
    double gasoline = 1.0, ethanol = 0.0, methanol = 0.0;
    double ron = 95, mon = 85;   // of the gasoline part
    double diesel = 0.0;         // diesel fuel (a diesel tank holds only this: diesel = 1)
    double cetane = 51;          // of the diesel
};

struct FuelProps {
    double stoich = 14.7;      // kg air per kg fuel
    double lhv = 43.5e6;       // J/kg
    double hvap = 350e3;       // J/kg (latent heat of vaporisation)
    double density = 0.745;    // kg/L
    double ron = 95, mon = 85;
    double slRatio = 1.0;      // laminar burning velocity relative to gasoline (Flame.h)
    double coldVap = 0.25;     // share of the fuel that stays liquid on fully cold walls
    double alcoholPct = 0;     // % alcohol by volume (what a flex fuel sensor reads)
    bool diesel = false;       // diesel fuel (compression ignition; cetane below)
    double cetane = 0;         // cetane number (diesel)
    // atoms per kg of fuel (mol): what the combustion products are made of (gasoline taken as CH2.0, which gives the
    // stoichiometric AFR 14.7 above; ethanol C2H6O, methanol CH4O)
    double molC = 1000.0 / 14.027, molH = 2000.0 / 14.027, molO = 0;
    double octaneIndex(double k = 0.3) const { return ron - k * (ron - mon); }   // Kalghatgi OI = RON - K S
};

FuelProps fuelProps(const FuelBlend& b);

// mixing two blends by volume (refuelling: what is in the tank plus what goes in)
FuelBlend mixBlends(const FuelBlend& a, double litresA, const FuelBlend& b, double litresB);

// the fuels at the pump
enum FuelType : int {
    kFuelRegular90, kFuelPremium92, kFuelPremium95, kFuelPremium98, kFuelRace, kFuelE10, kFuelE20, kFuelE85,
    kFuelE100, kFuelM100, kFuelDiesel, kFuelDieselCn40, kFuelTypeCount
};
inline bool isDieselFuel(int t) { return t == kFuelDiesel || t == kFuelDieselCn40; }
const char* fuelTypeName(int t);
const char* fuelTypeNames();         // '\0'-separated, for combo boxes
const char* fuelTypeNote(int t);
FuelBlend fuelTypeBlend(int t);

// share of the fuel's latent heat taken from the intake charge (the rest comes from the port walls / valve)
constexpr double kChargeCoolShare = 0.4;

// knock (Group A3): Livengood-Wu integral of the end-gas autoignition delay. Douaud & Eyzat (SAE 780080):
//   tau = 17.68 ms (ON / 100)^3.402 (p / atm)^-1.7 exp(3800 K / T)
// knock when the integral of dt / tau over the end-gas life (intake closing to the end of the flame) reaches 1.
// ON: the octane index RON - K (RON - MON) (Kalghatgi), K from the boost level (knockOnK below).
// The knock index reported to the sensor / ECU is 1 + ln(I) / 0.377: 1 at onset, +1 for every 1.46x of the integral
// (the delay change of ~110 K of end-gas temperature near 1050 K - the scale the old temperature index used).
// The prefactor is calibrated to this engine model (common practice: the correlation's constants are refitted per
// engine): with the model's isentropic end gas (no wall heat loss, no residual-dilution term) the published 17.68 ms
// puts the stock K20A calibration on premium 95 deep into knock at full load. kKnockDelayScale x 17.68 ms puts it at
// the knock limit again, where the calibration was made (and where the previous temperature criterion had it).
constexpr double kKnockDelayScale = 3.2;
inline double knockDelayS(double octane, double pPa, double T) {
    return kKnockDelayScale * 17.68e-3 * std::pow(std::max(50.0, octane) / 100.0, 3.402) * std::pow(std::max(0.05, pPa / 101325.0), -1.7) *
           std::exp(3800.0 / std::max(300.0, T));
}
inline double knockIndexFromIntegral(double I) { return I > 1e-12 ? std::clamp(1.0 + std::log(I) / 0.377, 0.0, 5.0) : 0.0; }
inline double knockOnK(double pIvcRatio) { return std::max(-0.35, std::min(0.35, 0.35 - 0.35 * (pIvcRatio - 1.0))); }

// diesel ignition delay (Package 4): Hardenberg & Hase (SAE 790493), as reproduced in Heywood eq. 10.37 (MIT 2.61
// lecture 16):
//   tau_id (deg CA) = (0.36 + 0.22 Sp) exp[ E_A (1 / (R T) - 1 / 17190) (21.2 / (p - 12.4))^0.63 ]
//   E_A = 618840 / (CN + 25) J/mol;  Sp mean piston speed m/s, T K, p bar, R = 8.314 J/(mol K)
// The correlation gives the delay for a charge state; the engine integrates dt / tau over the changing state from
// the start of injection (as the knock model does with its delay) - ignition when the integral reaches 1. Below
// 12.4 bar the expression has no meaning (no ignition there: returns a very long delay).
inline double dieselDelayDeg(double Sp, double T, double pBar, double cetane) {
    if (pBar <= 12.9 || T <= 300.0) return 1e9;
    const double ea = 618840.0 / (std::max(0.0, cetane) + 25.0);
    const double x = ea * (1.0 / (8.314 * T) - 1.0 / 17190.0) * std::pow(21.2 / (pBar - 12.4), 0.63);
    return (0.36 + 0.22 * Sp) * std::exp(std::min(60.0, x));
}

// Ignition delay used by the engine: Assanis, Filipi, Fiveland & Syrimis (J. Eng. Gas Turbines Power 125, 2003),
// as reproduced in Alkhulaifi & Hamdalla (WASET 5(10), 2011, eq. 3): tau = 2.4 phi^-0.2 p^-1.02 exp(2100 / T) ms
// (p bar, T K; fitted on a heavy-duty turbocharged DI diesel in steady and transient running; the validity range was
// not verified). Its temperature sensitivity (2100 K) is about twice Hardenberg-Hase's (E_A / R ~980 K at CN 51), so a
// cooler charge lights clearly later - what makes a pilot injection shorten the main injection's delay. It has no
// cetane term: the cetane number enters as Hardenberg-Hase's ratio at the same state, relative to CN 51 (the fuel of
// the fitted engine is not stated in the source - taken as EN 590 diesel).
inline double dieselDelayMs(double phi, double pBar, double T, double cetane) {
    if (pBar <= 1.0 || T <= 300.0) return 1e9;
    double tau = 2.4 * std::pow(std::max(0.05, phi), -0.2) * std::pow(pBar, -1.02) * std::exp(2100.0 / T);
    if (pBar > 12.9) {
        const double cn = std::max(0.0, cetane);
        const double dEa = 618840.0 / (cn + 25.0) - 618840.0 / (51.0 + 25.0);
        tau *= std::exp(std::min(60.0, dEa * (1.0 / (8.314 * T) - 1.0 / 17190.0) * std::pow(21.2 / (pBar - 12.4), 0.63)));
    }
    return tau;
}

// Main injection after a pilot that is already burning: Busch et al. (Sandia, SAND2015-2001 / SAE 2015-01-0796):
// the pilot's heat and radicals shorten the main injection's ignition delay and cut its premixed burn - about 6 dB
// less combustion noise at 1500 rpm, IMEPg 9 bar, 1.5 mg pilot, dwell > 200 us. The bulk temperature rise of a 0-D
// charge (~80 K from 1.5 mg) shortens the delay only ~17 %: the main spray actually enters the pilot's burning zone.
// So the main delay is divided by this factor on top - calibrated, not measured: it makes the model's 1500 rpm /
// ~9 bar IMEP point (preset 24) give ~6 dB, as Busch et al. measured.
constexpr double kPilotMainDelayDiv = 2.2;

// cold start: a diesel lights unaided only when the compressed charge is hot enough, else it needs the hot spot of a
// glow plug (Bosch glow plug brochure: at least 850 C needed at the plug for a reliable cold start; DI engines start
// without glow plugs above about -5 C - Bosch patent US 7900599: above -10 C). Real engines lose compression
// temperature at starter speed to ring blow-by and wall heat; this model has no blow-by and its cranking compression
// stays hotter, so the limit is calibrated, not measured: set so the 1.9 TDI-style preset starts unaided down to
// -7.5 C and not at -10 C (between the two Bosch figures; the charge temperature while the fuel lights includes the
// spray's evaporation cooling) - below it the charge alone does not light.
constexpr double kDieselColdIgnK = 815.0;
constexpr double kGlowIgnC = 850.0;

// diesel burn shape: the Watson-type double / triple Wiebe of Ricardo WAVE as given in Clean Energy 8(5):73 (2024),
// eqs. 1-7 - premixed  1 - [1 - (0.75 t)^2]^5000, diffusion 1 - [1 - (cd3 t)^1.75]^5000, tail 1 - [1 - (ct3 t)^2.5]^5000,
// t = (theta - theta_soc) / 125 deg, cd3 = 0.055 / (1 + 0.5 phi), ct3 = 3.7 cd3 / (1 + 1.12 phi), tail share of the
// non-premixed fuel 0.6 phi^2 (phi: overall equivalence ratio, capped at 0.85). (The paper scales the 125 deg with
// (rpm / BRPM)^0.3; the reference speed was not verified - left out, the shapes are taken in crank angle.)
// The premixed share is not from that paper (its source correlation, Watson et al. SAE 800029, was not verified):
// here it is the fuel that was injected during the ignition delay (Heywood's picture of premixed burning).
inline double wiebeWave(double coef, double expo, double t) {
    const double y = std::pow(std::max(0.0, coef * t), expo);
    return y >= 1.0 ? 1.0 : 1.0 - std::exp(5000.0 * std::log1p(-y));
}

} // namespace es
