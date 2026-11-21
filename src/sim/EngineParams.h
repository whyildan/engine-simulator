#pragma once
#include "sim/Components.h"
#include "ecu/SensorDefs.h"
#include "ecu/Calibration.h"
#include "sim/TurboMap.h"
#include <algorithm>
// All user-tweakable parameters. Plain-old-data so it can be copied between the
// UI thread and the simulation thread cheaply and compared with memcmp.
#include <cstdint>
#include <cmath>
#include <cstring>

namespace es {

constexpr int kMaxCyl = 16;

enum class Layout : int32_t { Inline = 0, V = 1, Boxer = 2 };
enum class FuelSystem : int32_t { EFI = 0, Carburetor = 1, DieselDI = 2 };   // DieselDI: compression ignition, direct injection (Package 4)
enum class Injection : int32_t { Port = 0, ThrottleBody = 1 };
enum class HeaderType : int32_t { Log = 0, Collector = 1, TriY = 2 };
enum class Crossover : int32_t { TrueDual = 0, HPipe = 1, XPipe = 2, MergeY = 3 };
enum class Muffler : int32_t { None = 0, StraightThrough = 1, Chambered = 2, Turbo = 3 };
enum class Limiter : int32_t { IgnitionCut = 0, FuelCut = 1 };
enum class ListenerPos : int32_t { Tailpipe = 0, EngineBay = 1, Cabin = 2 };
enum class DynoMode : int32_t { Off = 0, Hold = 1, Sweep = 2, Step = 3 };
enum class Environment : int32_t { OpenRoad = 0, Garage = 1, DynoCell = 2, Tunnel = 3 };
enum class EngineType : int32_t { Piston = 0, Rotary = 1, TwoStroke = 2 };   // TwoStroke: Package 5
enum class Induction : int32_t { Natural = 0, Turbo = 1, Supercharger = 2 };
enum class Supercharger : int32_t { Roots = 0, TwinScrew = 1, Centrifugal = 2 };
enum class TransMode : int32_t { Manual = 0, ManualAutoClutch = 1, Automatic = 2 };
// how the engine is built (the engine view draws it; the simulation does not use these yet)
enum class CamLayout : int32_t { DOHC = 0, SOHC = 1, OHV = 2 };   // OHV: one cam in the block, pushrods and rockers
enum class CamDrive : int32_t { Belt = 0, Chain = 1, Gears = 2 };
enum class Cooling : int32_t { Liquid = 0, Air = 1 };              // Air: finned cylinders and heads
constexpr int kMaxGears = 8;

struct EngineParams {
    char name[48];

    // ---------------------------------------------------------- geometry / crank
    int32_t cylinders;
    Layout layout;
    float bankAngle;          // deg (V)
    float bore_mm, stroke_mm, rod_mm;
    float compression;        // CR
    float flywheel;           // kg m^2 (total rotating inertia)
    int32_t firingOrder[kMaxCyl];   // 0-based cylinder index in firing sequence
    int32_t evenFire;               // bool: phases derived from firing order
    float phase[kMaxCyl];           // deg 0..720, used when !evenFire
    int32_t bank[kMaxCyl];          // 0/1 - which exhaust bank a cylinder belongs to

    // ---------------------------------------------------------- valvetrain
    int32_t intakeValves, exhaustValves;
    float intakeValveDia_mm, exhaustValveDia_mm;
    float intakeDuration, exhaustDuration;     // crank deg
    float intakeLift_mm, exhaustLift_mm;
    float intakeCenterline;   // deg ATDC (overlap TDC)
    float exhaustCenterline;  // deg BTDC (overlap TDC)
    float intakeCamAdvance, exhaustCamAdvance; // deg (+ = earlier)
    float valveLash_mm;
    CamLayout camLayout;      // valvetrain / timing drive / cooling hardware: drawn by the engine view only (for now)
    CamDrive camDrive;
    Cooling cooling;
    // two-stage cam switching (VTEC-style): a second, high-rpm profile the ECU locks the rockers onto
    int32_t vtec;
    float vtecInDuration, vtecInLift_mm, vtecExDuration, vtecExLift_mm;
    // continuously variable intake cam phaser (VTC-style): the ECU advances the intake cam from its map
    int32_t vtc;
    float vtcRange;           // crank deg of phaser authority (advance from the base centerline)

    // ---------------------------------------------------------- ignition
    float baseAdvance;        // deg BTDC
    float mechAdvance;        // extra deg at mechAdvanceRpm
    float mechAdvanceRpm;
    float vacuumAdvance;      // extra deg at high vacuum (part load)
    float revLimit;
    float redline;            // tach red zone / shift light (0 = 300 rpm below the rev limit)
    float frictionMult;       // mechanical friction multiplier (1 = as built)
    Limiter limiter;

    // ---------------------------------------------------------- intake
    float throttleBore_mm;
    int32_t itb;              // individual throttle bodies
    float plenum_L;
    float runnerLen_mm, runnerDia_mm;
    float airFilter;          // 0 = open / no filter, 1 = restrictive

    // ---------------------------------------------------------- fuel
    FuelSystem fuelSystem;
    // EFI
    Injection injection;
    float injector_ccmin;
    float afrCruise, afrWot;
    int32_t veLearning;       // ECU adapts its VE table (tuned ECU)
    float injectionAngle;     // deg (end of injection, 0..720 from firing TDC)
    float idleTargetRpm;
    // carburetor
    int32_t carbBarrels;
    float venturi_mm;
    float mainJet;            // 1.0 = nominal (AFR ~12.8)
    float idleMixture;        // 1.0 = nominal
    float idleScrew;          // throttle crack %, 0..10
    float accelPump;          // 0..2
    float choke;              // 0..1
    // fuel (Package 1): what the tank is filled with, and the ethanol (flex fuel) sensor in the fuel line
    int32_t fuelType;         // FuelType (sim/Fuel.h)
    int32_t flexSensor;       // flex fuel sensor fitted: the ECU reads the alcohol content of the fuel
    // diesel (Package 4, fuelSystem DieselDI): fuel injected straight into the cylinder near TDC, lit by the
    // compressed air after an ignition delay; no throttle (the load is the amount of fuel)
    float dieselInjRate;      // mg per ms one injector delivers at its rail pressure (sets the injection duration)
    float dieselMaxFuel;      // mg per stroke at full pedal (the driver's demand map; the smoke limiter can cut it)
    float soiBase;            // deg BTDC start of injection at idle (static timing)
    float soiAdvance;         // deg added by rated speed (speed advance, like a mechanical pump's timing device)
    int32_t glowPlugs;        // 0 none, 1 metal sheathed-element, 2 ceramic (heat-up: CombustionNoise.h / Engine)
    float egrValveDia_mm;     // exhaust gas recirculation valve (exhaust manifold -> intake), 0 = none (Package 4b)

    // ---------------------------------------------------------- exhaust sensors (Package 2)
    int32_t emapSensor;       // exhaust manifold pressure sensor (turbo: before the turbine)
    int32_t widebandPos;      // 0 = collector (before a turbine), 1 = downpipe (after the turbine)
    int32_t egtPerCyl;        // a thermocouple in every primary (per-cylinder EGT)
    int32_t egtPostTurbo;     // second thermocouple after the turbine

    // ---------------------------------------------------------- exhaust
    HeaderType header;
    float primaryLen_mm[kMaxCyl];
    float primaryDia_mm;
    float secondaryLen_mm;    // tri-Y / 4-2-1
    float collectorLen_mm, collectorDia_mm;
    Crossover crossover;
    float pipeDia_mm;
    float midpipeLen_mm;
    int32_t cat;
    int32_t resonator;        // 0 none, 1 inline chamber, 2 Helmholtz side branch (tuned: resonatorHz)
    Muffler muffler;
    float tailpipeLen_mm;
    float resonatorHz;        // Helmholtz resonator tuning, Hz (0 = auto: firing frequency at 2000 rpm - cruise drone)
    int32_t mufflerExtTubes;  // chambered muffler: extended inlet / outlet tubes (L/2, L/4) inside each chamber

    // ---------------------------------------------------------- driver controls
    float throttle;           // 0..1 pedal
    int32_t ignition;         // key on
    int32_t starter;          // cranking
    float ambientTemp_C;
    float baro_kPa;           // ambient (barometric) pressure, from the altitude; 0 = sea level

    // ---------------------------------------------------------- audio
    ListenerPos listener;
    float volume;             // 0..2
    int32_t oversample;       // 1..4  (sim rate = 48k * oversample)

    // ---------------------------------------------------------- dyno
    DynoMode dynoMode;
    float dynoHoldRpm;
    float sweepStartRpm, sweepEndRpm, sweepRate; // rpm/s
    int32_t sweepRequest;     // incremented by UI to start a sweep (sweep or step test)
    float stepRpm;            // step test: rpm between the held points
    float stepHold_s;         // step test: averaging time at each point (after it has settled)
    float coolingFan_kmh;     // dyno cell fan: air speed blown at the radiator / intercooler when not driving

    // ---------------------------------------------------------- rotary (Wankel)
    // In rotary mode `cylinders` is the number of rotors (1..4). Every rotor has 3 chambers
    // (faces); a chamber's 4-stroke cycle takes 1080 deg of eccentric shaft rotation.
    EngineType engineType;
    float rotorRadius_mm;     // generating radius R
    float eccentricity_mm;    // e
    float rotorWidth_mm;      // housing width B
    float rIntakeOpen;        // shaft deg ATDC (negative = BTDC -> overlap)
    float rIntakeClose;       // shaft deg ABDC
    float rIntakeArea_cm2;    // intake port area per rotor
    float rExhaustOpen;       // shaft deg BBDC
    float rExhaustClose;      // shaft deg ATDC
    float rExhaustArea_cm2;   // exhaust port area per rotor
    float apexLeak;           // 0 = perfect apex seals, 1 = worn out

    // ---------------------------------------------------------- two-stroke (Package 5)
    // Crankcase scavenged, loop scavenged: the piston uncovers the exhaust and transfer ports in the cylinder wall;
    // the charge comes in through a reed valve into the crankcase, which the underside of the piston pumps up into
    // the cylinder through the transfer ports. One working cycle per crank revolution; no valvetrain.
    float tsExhaustOpen;      // deg ATDC the piston crown uncovers the exhaust port (it closes as far before TDC)
    float tsTransferOpen;     // deg ATDC ... the transfer ports
    float tsExhaustWidth;     // exhaust port width (all its openings), share of the bore
    float tsTransferWidth;    // transfer ports, total width, share of the bore
    float tsCrankcaseCR;      // primary (crankcase) compression ratio: crankcase volume with the piston at TDC / at BDC
    float tsReedArea_cm2;     // reed valve flow area with the petals fully open
    int32_t tsScavenge;       // scavenging: which of Blair's measured cylinders (kScavCylinders, sim/Scavenge.h)
    int32_t tsPipe;           // exhaust: 0 = the ordinary header / pipes, 1 = an expansion chamber (tuned pipe) per cylinder
    float tsPipeRpm;          // ... the speed the expansion chamber is tuned for (sets its length)
    float tsPipeClass;        // ... its design (Blair): 0 = broad enduro pipe, 0.5 = motocross, 1 = road racing
    float tsBurnDeg;          // combustion: burn period (Vibe, deg) - the combustion chamber's (squish) speed
    float tsTransferLen;      // transfer passages: length of the gas column, share of the stroke (its inertia)

    // ---------------------------------------------------------- forced induction
    Induction induction;
    Supercharger scType;
    float compFlow_gs;        // compressor choke flow at max wheel speed (g/s)  -> compressor size
    float turbineAR;          // turbine housing A/R (0.3 small/fast .. 1.4 big/laggy)
    float turboInertia;       // rotating assembly inertia (kg cm^2)
    float wastegateBar;       // boost target (bar gauge)
    // variable geometry turbine (Package 4b): guide vanes instead of a wastegate - closed = small flow area (fast
    // spool, high back pressure), open = full area; the ECU positions them for its boost target
    int32_t vgt;
    float vgtMinArea;         // flow area with the vanes closed, share of the open area
    int32_t intercooler;
    float icEfficiency;       // 0..1
    float chargeVolume_L;     // compressor -> throttle piping + intercooler volume
    int32_t bov;              // blow-off valve (else: compressor surge flutter)
    int32_t antiLag;          // retard + throttle crack on lift-off to keep the turbo spooled
    float boostRetard;        // ignition retard per bar of boost (deg)
    float afrBoost;           // ECU target AFR at 1 bar boost
    float scDisplacement_L;   // positive displacement blower: litres per blower revolution
    float scRatio;            // blower : crank drive ratio (centrifugal: impeller : crank)
    int32_t scBypass;         // PD blower bypass valve at part throttle
    TurboMaps turbo;          // compressor / turbine maps (Phase 6); used when turbo.useMaps is set

    // ---------------------------------------------------------- vehicle / drivetrain
    int32_t vehicle;          // driving mode: the car is the load (dyno disabled)
    TransMode transMode;
    int32_t gears;            // number of forward gears
    float gearRatio[kMaxGears];
    float finalDrive;
    float mass_kg;
    float cdA;                // drag coefficient x frontal area (m^2)
    float crr;                // rolling resistance coefficient
    float tireRadius_mm;
    float grip;               // tyre friction coefficient
    float driveFrac;          // share of the car's weight on the driven wheels
    float clutchTorque;       // clutch capacity (N m)
    float drivelineEff;
    float grade_pct;          // road slope (+ uphill)
    // driver inputs for the car
    int32_t gear;             // manual: selected gear (0 = neutral); automatic: 0 = N, 1 = D
    float clutchPedal;        // 0 = released (engaged) .. 1 = pressed (disengaged)
    float brake;              // 0..1

    // ---------------------------------------------------------- motorsport hardware and switches (Phase 5)
    int32_t nitrous;          // nitrous system fitted
    float n2oShotHp;          // jetted for this many hp (sets the N2O flow)
    int32_t n2oWet;           // 1 = wet kit (its own fuel solenoid adds the fuel), 0 = dry (the ECU adds the fuel)
    float n2oBottleKg;        // bottle size
    int32_t n2oRefill;        // incremented by the UI: refill the bottle
    // driver switches - the ECU reads them as digital inputs (the clutch switch comes from clutchPedal)
    int32_t n2oArm;           // nitrous arming switch
    int32_t transbrake;       // transbrake button held (automatic / auto-clutch gearbox: holds the car)
    int32_t rollingAlsButton; // rolling anti-lag button held

    // ---------------------------------------------------------- exhaust tone / acoustics
    float exhaustRasp;        // turbulent jet noise at the tailpipe (0 = off, 1 = physical estimate)
    float exhaustPunch;       // low-frequency "body" (gas pulse pressure) mixed into the radiated sound
    float exhaustSteep;       // finite-amplitude wave steepening in the pipes (0 = linear acoustics, 1 = physical)
    int soundStyle;           // 0 = smooth (engine-sim style: pulse pressure, band-limited, IR), 1 = raw (original monopole only)
    float pressureMix;        // smooth style: share of the pulse-pressure signal vs the radiated dU/dt (0..1)
    float smoothHz;           // smooth style: 4th-order low-pass on the simulated exhaust signal (Hz)
    Environment environment;  // acoustic space around the car
    float reverbMix;          // 0..1
    int32_t irChoice;         // exhaust impulse response (see audio/ImpulseResponses.h)
    float irMix;              // 0 = dry simulated sound, 1 = fully convolved
    float imperfection;       // per-cylinder manufacturing spread + cycle-to-cycle jitter (0..2)

    // ---------------------------------------------------------- hardware components
    ComponentSet comp;

    // ---------------------------------------------------------- ECU / sensors / simulation
    float ecuRateHz;          // ECU control loop frequency (sensors sampled at this rate)
    int32_t coldStart;        // 1 = engine starts at ambient temperature, 0 = warm (88 C)
    // refuelling (bench): incremented by the UI - adds refuelLitres of refuelType to what is in the tank
    int32_t refuelRequest;
    int32_t refuelType;
    float refuelLitres;
    SensorConfig sensors[kSensorCount];

    // ---------------------------------------------------------- ECU calibration (maps, curves, settings)
    Calibration cal;
};

// fill p.cal with a base calibration derived from the engine (generic VE guess, lambda / ignition
// from the AFR and distributor-curve settings, enrichment curves, injector data)
void generateCalibration(EngineParams& p);

// imperfection actually used by the physics (the raw sound style is the original, perfectly even engine)
inline double effImperfection(const EngineParams& p) {
    return p.soundStyle == 1 ? 0.0 : std::min(3.0, std::max(0.0, (double)p.imperfection));
}

// the starter is released once the engine runs by itself above this speed (like a driver holding
// the key until the engine has really caught, not at the first cough)
// (cold: the fast-idle target is higher, so the engine has to be spinning faster before it is on its own)
inline double starterReleaseRpm(const EngineParams& p, double coolantC = 88.0) {
    double cold = 1.0 - std::min(1.0, std::max(0.0, (coolantC - 15.0) / 60.0));
    return std::max(600.0, 0.75 * ((double)p.idleTargetRpm + 300.0 * cold));
}

inline bool isBoosted(const EngineParams& p) { return p.induction != Induction::Natural; }
// redline (tach red zone / shift light): set, or 300 rpm below the rev limiter
inline float redlineRpm(const EngineParams& p) { return p.redline > 500.0f ? p.redline : p.revLimit - 300.0f; }
// mechanical friction multiplier (1 = the friction model as built)
inline double frictionScale(const EngineParams& p) { return p.frictionMult > 0.05f ? p.frictionMult : 1.0; }
// ambient pressure in Pa (sea level when unset)
inline double ambientPressurePa(const EngineParams& p) { return p.baro_kPa > 30.0f ? p.baro_kPa * 1000.0 : 101325.0; }
// standard atmosphere: pressure (kPa) at an altitude (m) and back
inline double baroAtAltitude(double m) { return 101.325 * std::pow(std::max(0.0, 1.0 - 2.25577e-5 * m), 5.25588); }
inline double altitudeOfBaro(double kPa) { return (1.0 - std::pow(std::max(1.0, kPa) / 101.325, 1.0 / 5.25588)) / 2.25577e-5; }

// Nitrous oxide is modelled as extra intake charge: 1 kg of N2O carries the oxygen of 1.57 kg of air and
// releases its decomposition heat on top, so it is fed to the ports as this much air-equivalent mass (and
// burns with the matching fuel). The flow per jetted hp follows from ~2.3 MJ of brake work per kg of N2O.
constexpr double kN2oAirEquiv = 2.2;
constexpr double kN2oKgsPerHp = 3.2e-4;
inline double n2oFullFlowKgs(const EngineParams& p) { return std::max(0.0f, p.n2oShotHp) * kN2oKgsPerHp; }

inline bool isRotary(const EngineParams& p) { return p.engineType == EngineType::Rotary; }
inline bool isDiesel(const EngineParams& p) { return p.fuelSystem == FuelSystem::DieselDI; }
inline bool isTwoStroke(const EngineParams& p) { return p.engineType == EngineType::TwoStroke; }
// length of one working cycle of a chamber in crank / eccentric-shaft degrees
inline double cycleDeg(const EngineParams& p) { return isRotary(p) ? 1080.0 : isTwoStroke(p) ? 360.0 : 720.0; }
// shaft revolutions between two firings of the same cylinder / rotor (four-stroke 2; a rotor and a two-stroke 1)
inline double revsPerFiring(const EngineParams& p) { return isRotary(p) || isTwoStroke(p) ? 1.0 : 2.0; }
// rated displacement in litres (rotary: Mazda convention = one face per rotor)
inline double ratedDisplacementL(const EngineParams& p) {
    if (isRotary(p)) return p.cylinders * 5.196152 * p.rotorRadius_mm * p.eccentricity_mm * p.rotorWidth_mm * 1e-6;
    return p.cylinders * 3.14159265 * 0.25 * p.bore_mm * p.bore_mm * p.stroke_mm * 1e-6;
}
// litres swept per two shaft revolutions (what a 4-stroke of that size would breathe)
inline double equivalentDisplacementL(const EngineParams& p) { return ratedDisplacementL(p) * (2.0 / revsPerFiring(p)); }

// Fields whose change requires rebuilding the engine object (with an audio crossfade).
struct StructuralKey {
    int32_t cylinders; Layout layout; int32_t bank[kMaxCyl];
    int32_t itb; FuelSystem fuelSystem; Injection injection; int32_t carbBarrels;
    HeaderType header; Crossover crossover; int32_t cat, resonator; Muffler muffler; float resonatorHz; int32_t mufflerExtTubes;
    int32_t oversample;
    int32_t evenFire; int32_t firingOrder[kMaxCyl]; // affects tri-Y pairing
    EngineType engineType;
    Induction induction; Supercharger scType;
    int32_t tsPipe; float tsPipeRpm;   // (the expansion chamber's geometry is built with the exhaust:
    float tsPipeGeo[5];                //  its design and what it is proportioned from - the exhaust port and the stroke)

    static StructuralKey of(const EngineParams& p) {
        StructuralKey k;
        std::memset(&k, 0, sizeof(k));
        k.cylinders = p.cylinders; k.layout = p.layout;
        for (int i = 0; i < kMaxCyl; ++i) { k.bank[i] = p.bank[i]; k.firingOrder[i] = p.firingOrder[i]; }
        k.itb = p.itb; k.fuelSystem = p.fuelSystem; k.injection = p.injection; k.carbBarrels = p.carbBarrels;
        k.header = p.header; k.crossover = p.crossover; k.cat = p.cat; k.resonator = p.resonator;
        k.muffler = p.muffler; k.oversample = p.oversample; k.evenFire = p.evenFire;
        k.resonatorHz = p.resonatorHz; k.mufflerExtTubes = p.mufflerExtTubes;
        k.engineType = p.engineType;
        k.induction = p.induction; k.scType = p.scType;
        k.tsPipe = isTwoStroke(p) ? p.tsPipe : 0; k.tsPipeRpm = isTwoStroke(p) && p.tsPipe ? p.tsPipeRpm : 0.0f;
        if (isTwoStroke(p) && p.tsPipe) {
            k.tsPipeGeo[0] = p.tsPipeClass; k.tsPipeGeo[1] = p.tsExhaustOpen; k.tsPipeGeo[2] = p.tsExhaustWidth;
            k.tsPipeGeo[3] = p.bore_mm; k.tsPipeGeo[4] = p.stroke_mm;
        }
        return k;
    }
    bool operator==(const StructuralKey& o) const { return std::memcmp(this, &o, sizeof(*this)) == 0; }
    bool operator!=(const StructuralKey& o) const { return !(*this == o); }
};

// firing phase (deg, 0..720) for each cylinder
void computePhases(const EngineParams& p, float out[kMaxCyl]);
// checks firing order is a permutation and phases are distinct; returns message or nullptr
const char* validateFiring(const EngineParams& p);

int presetCount();
const char* presetName(int i);
EngineParams makePreset(int i);
// re-derive dependent defaults when the user changes cylinder count / layout
void applyLayoutDefaults(EngineParams& p);
// rotary port timing presets: 0 stock side, 1 street, 2 bridge, 3 peripheral
void applyRotaryPorting(EngineParams& p, int style);
const char* rotaryPortingName(int style);
// sensible forced-induction defaults sized to the engine
void applyInductionDefaults(EngineParams& p, Induction ind, Supercharger sc = Supercharger::Roots);
// size a plausible car (or motorcycle) and gearbox to the engine
void applyVehicleDefaults(EngineParams& p);

} // namespace es
