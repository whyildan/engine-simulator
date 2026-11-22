#pragma once
// Engine hardware components (pistons, rods, crank, valvetrain, throttle, injectors, ignition,
// pumps) and the physical quantities the simulation derives from them: rotating / reciprocating
// mass and inertia, mechanical stress limit, valve-float rpm, spark energy, friction, oil pressure.
#include <cstdint>

namespace es {

struct EngineParams;

enum class PistonType : int32_t { Cast = 0, Hypereutectic = 1, Forged = 2 };
enum class RodType : int32_t { PowderedMetal = 0, ForgedSteel = 1, Titanium = 2, Aluminium = 3 };
enum class CrankType : int32_t { Cast = 0, Forged = 1, Billet = 2 };
enum class LifterType : int32_t { HydraulicFlat = 0, HydraulicRoller = 1, SolidBucket = 2, FingerFollower = 3 };
enum class ThrottleType : int32_t { Cable = 0, DriveByWire = 1 };
enum class CoilType : int32_t { Distributor = 0, WastedSpark = 1, CoilOnPlug = 2 };
enum class FuelRegulator : int32_t { MapReferenced = 0, Returnless = 1 };

struct ComponentSet {
    PistonType pistonType;
    float pistonMass_g;        // piston + pin + rings
    float ringTension;         // 1 = standard; low-tension race rings < 1
    RodType rodType;
    float rodMass_g;
    CrankType crankType;
    LifterType lifter;
    float springSeat_N;        // valve spring force on the seat
    float springOpen_N;        // valve spring force at full lift
    ThrottleType throttleType;
    float dbwResponse_ms;      // drive-by-wire motor time constant
    float injDeadTime_ms;      // injector opening delay (at 14 V)
    float fuelPressure_bar;    // regulator base pressure (gauge); injector flow is rated at 3 bar
    FuelRegulator fuelRegulator;
    float fuelPumpLph;         // pump flow at the base pressure (litres / hour)
    CoilType coilType;
    float coilEnergy_mJ;       // energy of a fully charged coil
    float coilCharge_ms;       // time to (nearly) fully charge
    int32_t plugHeatRange;     // NGK-style: 5 hot .. 9 cold, 6 = standard
    float plugGap_mm;
    float oilPumpCapacity;     // 1 = standard
    float oilRelief_bar;       // relief valve pressure
    float waterPumpCapacity;   // 1 = standard
    float radiatorSize;        // 1 = stock (sized for the engine's rated heat); bigger core > 1
    int32_t oilCooler;         // 0 none, 1 oil-to-water (on the block), 2 air-to-oil (in front of the radiator)
    int32_t allowFailure;      // mechanical damage can destroy the engine
    int32_t repairCount;       // incremented by the UI: repair a failed engine
};

const char* pistonTypeNames();   // '\0'-separated lists for combo boxes
const char* rodTypeNames();
const char* crankTypeNames();
const char* lifterTypeNames();
const char* throttleTypeNames();
const char* coilTypeNames();
const char* fuelRegulatorNames();

// sensible parts for the engine as configured (masses from bore / rod length, springs and
// materials sized so the engine is safe a bit beyond its rev limiter)
void applyComponentDefaults(EngineParams& p);

// quantities derived from the configuration (cheap; recomputed on every parameter change)
struct ComponentModel {
    bool piston = true;              // false for rotary engines (no pistons / rods / valves)
    bool valves = true;              // false for rotaries and two-strokes (ports, no valvetrain)
    double recipMass = 0;            // kg per cylinder (piston + 1/3 rod)
    double rotMass = 0;              // kg per cylinder (2/3 rod)
    double crankRadius = 0.04, rodLen = 0.14, pistonArea = 5e-3;
    double extraInertia = 0;         // kg m^2 added to the crank by pistons and rods
    double allowInertiaPa[3] = {};   // allowed peak inertia load / piston area: piston, rod, crank
    double safeRpm = 1e9;            // mechanical limit (weakest part)
    double partSafeRpm[3] = {1e9, 1e9, 1e9};   // the limit of each part: piston, rod, crank
    int weakest = 0;                 // 0 piston, 1 rod, 2 crank
    double moveMassIn = 0.05, moveMassEx = 0.05;  // effective moving valvetrain mass per valve (kg)
    double floatRpmIn = 1e9, floatRpmEx = 1e9;    // valve float onset
    double valveFrictionK = 1;       // lifter type friction factor
    int coilsPerCycle = 1;           // sparks per coil per working cycle (cycle time / this = time between sparks)

    void build(const EngineParams& p);
    // peak reciprocating inertia load relative to the weakest part (1 = at its limit)
    double stress(double rpm) const;
    // valve float severity 0..1 for intake / exhaust at this rpm
    double floatSeverity(double rpm, bool intake) const;
    // extra friction mean effective pressure (Pa) of rings, valve springs and pumps versus a
    // standard engine (the base friction model already contains standard parts)
    double extraFmep(const EngineParams& p, double rpm, double meanPistonSpeed, double displacementTotal) const;
    // spark energy available (mJ) at this rpm with this dwell (ms, as the ECU commands it) and battery voltage
    double sparkEnergy(const EngineParams& p, double rpm, double dwellMs = 1e9, double volts = 14.0) const;
    // time (s) between two sparks of one coil
    static double coilInterval(const EngineParams& p, double rpm);
    // spark energy needed (mJ) to ignite at this cylinder pressure and mixture
    static double sparkNeeded(const EngineParams& p, double cylPressurePa, double lambda);
    // oil pressure (bar) at this rpm and oil temperature
    static double oilPressure(const EngineParams& p, double rpm, double oilTempC);
};

} // namespace es
