#include "sim/Fuel.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
struct Pure { double stoich, lhv, hvap, density, ron, mon, slRatio, coldVap; };
// slRatio: laminar burning velocity / gasoline (Gulder 1982, one study: 0.455 / 0.47 / 0.50 m/s, see Flame.h)
//                       AFR    LHV J/kg  HoV J/kg  kg/L   RON    MON   S_L    cold
const Pure kGasoline = {14.7,  43.5e6,   350e3,  0.745,  95,    85,   1.00,  0.25};
const Pure kEthanol  = { 9.0,  26.9e6,   920e3,  0.789, 108.5,  90.0, 1.033, 0.75};
const Pure kMethanol = { 6.45, 20.0e6,  1100e3,  0.792, 108.7,  88.6, 1.099, 0.62};
// diesel (sources in Fuel.h): AFR 14.5, LHV 42.79 MJ/kg, HoV 250 kJ/kg (inside 225-280), 0.837 kg/L
constexpr double kDieselStoich = 14.5, kDieselLhv = 42.79e6, kDieselHvap = 250e3, kDieselDensity = 0.837;
constexpr double kDieselMolarMass = 12.5 * 12.011 + 22.2 * 1.008;   // C12.5H22.2, g/mol (Colorado State)

struct TypeDef { const char* name; const char* note; double ron, mon, eth, meth; };
const TypeDef kTypes[kFuelTypeCount] = {
    {"Regular (RON 90)",       "Pertalite / US 87 AKI class. For compression up to about 10:1.",                 90, 81.5, 0, 0},
    {"Premium 92 (RON 92)",    "Pertamax class.",                                                               92, 83,   0, 0},
    {"Premium 95 (RON 95)",    "Euro premium, US 91 AKI class. The fuel every base calibration is made on.",   95, 85,   0, 0},
    {"Premium 98 (RON 98)",    "Pertamax Turbo / Super Plus class.",                                            98, 87.5, 0, 0},
    {"Race fuel (RON 104)",    "Unleaded race gasoline: high knock resistance, same AFR as pump gasoline.",    104, 96,   0, 0},
    {"E10",                    "10 % ethanol in premium 92: about +3 RON, AFR ~14.1.",                          92, 83,   0.10, 0},
    {"E20",                    "20 % ethanol in premium 92: about +6 RON, AFR ~13.6.",                          92, 83,   0.20, 0},
    {"E85",                    "85 % ethanol: RON ~108, AFR ~9.8, about 35 % more fuel volume. Hard cold starts.", 92, 83, 0.85, 0},
    {"Ethanol (E100)",         "Pure ethanol: AFR 9.0, strong charge cooling. No cold start without help.",    92, 83,   1.0, 0},
    {"Methanol (M100)",        "Pure methanol: AFR 6.45, about twice the fuel volume, very strong charge cooling.", 92, 83, 0, 1.0},
    {"Diesel (EN 590)",        "Diesel engines only: cetane 51, AFR 14.5, LHV 42.8 MJ/kg.",                     0, 0,     0, 0},
    {"Diesel, cetane 40",      "Diesel at the ASTM D975 minimum cetane number: longer ignition delay, louder, harder cold starts.", 0, 0, 0, 0},
};
} // namespace

FuelProps fuelProps(const FuelBlend& in) {
    if (in.diesel > 0.5) {   // diesel: no blends with gasoline / alcohol (see Fuel.h)
        FuelProps f;
        f.stoich = kDieselStoich; f.lhv = kDieselLhv; f.hvap = kDieselHvap; f.density = kDieselDensity;
        f.ron = f.mon = 0; f.slRatio = 1.0; f.coldVap = 0; f.alcoholPct = 0;
        f.diesel = true; f.cetane = in.cetane;
        f.molC = 1000.0 * 12.5 / kDieselMolarMass; f.molH = 1000.0 * 22.2 / kDieselMolarMass; f.molO = 0;
        return f;
    }
    FuelBlend b = in;
    b.gasoline = std::max(0.0, b.gasoline); b.ethanol = std::max(0.0, b.ethanol); b.methanol = std::max(0.0, b.methanol);
    const double sumV = b.gasoline + b.ethanol + b.methanol;
    if (sumV <= 1e-9) { b.gasoline = 1; b.ethanol = b.methanol = 0; }
    else { b.gasoline /= sumV; b.ethanol /= sumV; b.methanol /= sumV; }
    // mass fractions (the AFR, heating value and heat of vaporisation mix by mass)
    const double mg = b.gasoline * kGasoline.density, me = b.ethanol * kEthanol.density, mm = b.methanol * kMethanol.density;
    const double mt = mg + me + mm;
    const double wg = mg / mt, we = me / mt, wm = mm / mt;
    FuelProps f;
    f.density = mt;   // kg per litre of blend
    f.stoich = wg * kGasoline.stoich + we * kEthanol.stoich + wm * kMethanol.stoich;
    f.lhv = wg * kGasoline.lhv + we * kEthanol.lhv + wm * kMethanol.lhv;
    f.hvap = wg * kGasoline.hvap + we * kEthanol.hvap + wm * kMethanol.hvap;
    f.slRatio = wg * kGasoline.slRatio + we * kEthanol.slRatio + wm * kMethanol.slRatio;
    // octane: alcohol blends non-linearly - the first litres raise it most (E10 ~ +3 RON, E20 ~ +6, E85 ~ 108)
    const double va = b.ethanol + b.methanol;
    if (va > 1e-9) {
        const double ronA = (b.ethanol * kEthanol.ron + b.methanol * kMethanol.ron) / va;
        const double monA = (b.ethanol * kEthanol.mon + b.methanol * kMethanol.mon) / va;
        const double s = va * (2.0 - va);
        f.ron = b.ron + (ronA - b.ron) * s;
        f.mon = b.mon + (monA - b.mon) * s;
    } else { f.ron = b.ron; f.mon = b.mon; }
    // cold walls: small alcohol shares even help (azeotropes raise the vapour pressure); the penalty comes with
    // the high blends (single boiling point, huge heat of vaporisation)
    const double wa = we + wm;
    const double coldA = wa > 1e-9 ? (we * kEthanol.coldVap + wm * kMethanol.coldVap) / wa : kGasoline.coldVap;
    f.coldVap = kGasoline.coldVap + (coldA - kGasoline.coldVap) * wa * wa;
    f.alcoholPct = va * 100.0;
    // atoms per kg: gasoline CH2.0 (14.027 g/mol C), ethanol C2H6O (46.069 g/mol), methanol CH4O (32.042 g/mol)
    f.molC = 1000.0 * (wg / 14.027 + we * 2.0 / 46.069 + wm / 32.042);
    f.molH = 1000.0 * (wg * 2.0 / 14.027 + we * 6.0 / 46.069 + wm * 4.0 / 32.042);
    f.molO = 1000.0 * (we / 46.069 + wm / 32.042);
    return f;
}

FuelBlend mixBlends(const FuelBlend& a, double la, const FuelBlend& b, double lb) {
    la = std::max(0.0, la); lb = std::max(0.0, lb);
    const double t = la + lb;
    if (t <= 1e-9) return a;
    FuelBlend m;
    m.gasoline = (a.gasoline * la + b.gasoline * lb) / t;
    m.ethanol = (a.ethanol * la + b.ethanol * lb) / t;
    m.methanol = (a.methanol * la + b.methanol * lb) / t;
    m.diesel = (a.diesel * la + b.diesel * lb) / t;
    m.cetane = a.diesel * la + b.diesel * lb > 1e-9 ? (a.cetane * a.diesel * la + b.cetane * b.diesel * lb) / (a.diesel * la + b.diesel * lb) : a.cetane;
    // octane of the gasoline parts mixes by their volume
    const double ga = a.gasoline * la, gb = b.gasoline * lb;
    if (ga + gb > 1e-9) { m.ron = (a.ron * ga + b.ron * gb) / (ga + gb); m.mon = (a.mon * ga + b.mon * gb) / (ga + gb); }
    else { m.ron = a.ron; m.mon = a.mon; }
    return m;
}

const char* fuelTypeName(int t) { return kTypes[clampv(t, 0, kFuelTypeCount - 1)].name; }
const char* fuelTypeNote(int t) { return kTypes[clampv(t, 0, kFuelTypeCount - 1)].note; }
const char* fuelTypeNames() {
    static char buf[512] = {};
    if (!buf[0]) {
        char* p = buf;
        for (const TypeDef& d : kTypes) { const char* s = d.name; while (*s) *p++ = *s++; *p++ = 0; }
        *p = 0;
    }
    return buf;
}
FuelBlend fuelTypeBlend(int t) {
    const TypeDef& d = kTypes[clampv(t, 0, kFuelTypeCount - 1)];
    FuelBlend b;
    if (isDieselFuel(t)) { b.gasoline = b.ethanol = b.methanol = 0; b.diesel = 1; b.cetane = t == kFuelDiesel ? 51 : 40; b.ron = b.mon = 0; return b; }
    b.ethanol = d.eth; b.methanol = d.meth; b.gasoline = 1.0 - d.eth - d.meth;
    b.ron = d.ron; b.mon = d.mon;
    return b;
}

} // namespace es
