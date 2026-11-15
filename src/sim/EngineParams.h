#pragma once
#include "sim/Components.h"
#include "ecu/SensorDefs.h"
#include "ecu/Calibration.h"
#include <algorithm>
// All user-tweakable parameters. Plain-old-data so it can be copied between the
// UI thread and the simulation thread cheaply and compared with memcmp.
#include <cstdint>
#include <cstring>

namespace es {

constexpr int kMaxCyl = 16;

enum class Layout : int32_t { Inline = 0, V = 1, Boxer = 2 };
enum class FuelSystem : int32_t { EFI = 0, Carburetor = 1 };
enum class Injection : int32_t { Port = 0, ThrottleBody = 1 };
enum class HeaderType : int32_t { Log = 0, Collector = 1, TriY = 2 };
enum class Crossover : int32_t { TrueDual = 0, HPipe = 1, XPipe = 2, MergeY = 3 };
enum class Muffler : int32_t { None = 0, StraightThrough = 1, Chambered = 2, Turbo = 3 };
enum class Limiter : int32_t { IgnitionCut = 0, FuelCut = 1 };
enum class ListenerPos : int32_t { Tailpipe = 0, EngineBay = 1, Cabin = 2 };
enum class DynoMode : int32_t { Off = 0, Hold = 1, Sweep = 2 };
enum class Environment : int32_t { OpenRoad = 0, Garage = 1, DynoCell = 2, Tunnel = 3 };
enum class EngineType : int32_t { Piston = 0, Rotary = 1 };
enum class Induction : int32_t { Natural = 0, Turbo = 1, Supercharger = 2 };
enum class Supercharger : int32_t { Roots = 0, TwinScrew = 1, Centrifugal = 2 };
enum class TransMode : int32_t { Manual = 0, ManualAutoClutch = 1, Automatic = 2 };
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

    // ---------------------------------------------------------- ignition
    float baseAdvance;        // deg BTDC
    float mechAdvance;        // extra deg at mechAdvanceRpm
    float mechAdvanceRpm;
    float vacuumAdvance;      // extra deg at high vacuum (part load)
    float revLimit;
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
    int32_t resonator;
    Muffler muffler;
    float tailpipeLen_mm;

    // ---------------------------------------------------------- driver controls
    float throttle;           // 0..1 pedal
    int32_t ignition;         // key on
    int32_t starter;          // cranking
    float ambientTemp_C;

    // ---------------------------------------------------------- audio
    ListenerPos listener;
    float volume;             // 0..2
    int32_t oversample;       // 1..4  (sim rate = 48k * oversample)

    // ---------------------------------------------------------- dyno
    DynoMode dynoMode;
    float dynoHoldRpm;
    float sweepStartRpm, sweepEndRpm, sweepRate; // rpm/s
    int32_t sweepRequest;     // incremented by UI to start a sweep

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

    // ---------------------------------------------------------- forced induction
    Induction induction;
    Supercharger scType;
    float compFlow_gs;        // compressor choke flow at max wheel speed (g/s)  -> compressor size
    float turbineAR;          // turbine housing A/R (0.3 small/fast .. 1.4 big/laggy)
    float turboInertia;       // rotating assembly inertia (kg cm^2)
    float wastegateBar;       // boost target (bar gauge)
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

inline bool isRotary(const EngineParams& p) { return p.engineType == EngineType::Rotary; }
// length of one working cycle of a chamber in crank / eccentric-shaft degrees
inline double cycleDeg(const EngineParams& p) { return isRotary(p) ? 1080.0 : 720.0; }
// rated displacement in litres (rotary: Mazda convention = one face per rotor)
inline double ratedDisplacementL(const EngineParams& p) {
    if (isRotary(p)) return p.cylinders * 5.196152 * p.rotorRadius_mm * p.eccentricity_mm * p.rotorWidth_mm * 1e-6;
    return p.cylinders * 3.14159265 * 0.25 * p.bore_mm * p.bore_mm * p.stroke_mm * 1e-6;
}
// litres swept per two shaft revolutions (what a 4-stroke of that size would breathe)
inline double equivalentDisplacementL(const EngineParams& p) { return ratedDisplacementL(p) * (isRotary(p) ? 2.0 : 1.0); }

// Fields whose change requires rebuilding the engine object (with an audio crossfade).
struct StructuralKey {
    int32_t cylinders; Layout layout; int32_t bank[kMaxCyl];
    int32_t itb; FuelSystem fuelSystem; Injection injection; int32_t carbBarrels;
    HeaderType header; Crossover crossover; int32_t cat, resonator; Muffler muffler;
    int32_t oversample;
    int32_t evenFire; int32_t firingOrder[kMaxCyl]; // affects tri-Y pairing
    EngineType engineType;
    Induction induction; Supercharger scType;

    static StructuralKey of(const EngineParams& p) {
        StructuralKey k;
        std::memset(&k, 0, sizeof(k));
        k.cylinders = p.cylinders; k.layout = p.layout;
        for (int i = 0; i < kMaxCyl; ++i) { k.bank[i] = p.bank[i]; k.firingOrder[i] = p.firingOrder[i]; }
        k.itb = p.itb; k.fuelSystem = p.fuelSystem; k.injection = p.injection; k.carbBarrels = p.carbBarrels;
        k.header = p.header; k.crossover = p.crossover; k.cat = p.cat; k.resonator = p.resonator;
        k.muffler = p.muffler; k.oversample = p.oversample; k.evenFire = p.evenFire;
        k.engineType = p.engineType;
        k.induction = p.induction; k.scType = p.scType;
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
