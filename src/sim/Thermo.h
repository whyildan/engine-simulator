#pragma once
// Temperature-dependent gas properties for the cylinder charge (Group A1).
//
// Species data: NASA 7-coefficient polynomials of GRI-Mech 3.0 (as distributed with Cantera, data/gri30.yaml) for
// N2, O2, CO2, H2O, CO and H2. Checked against the standard formation enthalpies (CO2 -393.51, H2O -241.82,
// CO -110.53 kJ/mol), continuity at 1000 K and the heating values of CO (10.10 MJ/kg) and H2 (120.0 MJ/kg).
//
// The cylinder holds fresh air (+ fuel vapour, counted as air), and burned products whose composition follows the
// mixture they came from. Lean and stoichiometric charges burn completely (CO2, H2O; the excess air stays air); rich
// charges use all the oxygen and leave CO and H2, split by the water-gas shift equilibrium
// K = [CO][H2O] / ([CO2][H2]) = 3.5, frozen at ~1740 K (the usual engineering value; the GRI data give 3.58 at 1740 K).
// Their unreleased chemical energy is not part of the heat release - a rich burn releases less heat than the air
// could, and its exhaust is cooler. Dissociation at the peak temperature is not modelled.
//
// Energies are sensible: u(T) = h(T) - h(298.15 K) - R T per kg, so formation enthalpies only enter through the
// heat release.

namespace es {

enum ThermoSpecies : int { kSpN2, kSpO2, kSpCO2, kSpH2O, kSpCO, kSpH2, kSpCount };

struct GasComp { double y[kSpCount] = {}; };   // mass fractions

namespace thermo {

constexpr double kTmin = 200.0, kTmax = 3500.0;

// air (N2 / O2 79 / 21 by moles, 28.85 kg/kmol)
const GasComp& air();

// sensible internal energy and enthalpy (J/kg) and cv (J/kg K) of a mixture at T
double u(const GasComp& c, double T);
double h(const GasComp& c, double T);
double cv(const GasComp& c, double T);
double gasR(const GasComp& c);                     // J/kg K

// temperature of a two-part charge (air mass ma, burned mass mb of composition pb) holding sensible energy U (J);
// T0 = a nearby guess (the last temperature)
double temperature(double ma, double mb, const GasComp& pb, double U, double T0);
// sensible energy of that charge at T
double energy(double ma, double mb, const GasComp& pb, double T);
// enthalpy (J/kg) of gas with burned mass fraction xb (the rest air / fuel vapour) at T
double enthalpy(double xb, const GasComp& pb, double T);

// burning one kg of fuel (C, H, O in mol per kg) with `airPerFuel` kg of air:
struct Burn {
    GasComp products;    // composition of the products (lean: the stoichiometric part; the excess air stays air)
    double airUsed = 0;  // kg air that ends up in the products per kg fuel
    double energyLeft = 0;  // J per kg fuel kept as chemical energy in CO and H2 (rich) - not released
};
Burn burn(double molC, double molH, double molO, double airPerFuel);

// temperature of air compressed isentropically from T0 by the pressure ratio pr (variable cp: s(T) - R ln p = const);
// Tguess = a nearby value (the last result) for a quick search
double isentropicAir(double T0, double pr, double Tguess);

// heating values (J/kg) from the formation enthalpies
double lhvCO();
double lhvH2();

} // namespace thermo
} // namespace es
