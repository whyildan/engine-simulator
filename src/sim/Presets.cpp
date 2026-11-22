#include "sim/EngineParams.h"
#include "sim/Fuel.h"
#include "core/MathUtil.h"
#include <cstdio>

namespace es {

void computePhases(const EngineParams& p, float out[kMaxCyl]) {
    int n = clampv(p.cylinders, 1, kMaxCyl);
    // rotary: every rotor fires once per shaft revolution -> rotors spaced 360/N apart
    // two-stroke: every cylinder fires once per revolution, too
    const double spacing = 360.0 * revsPerFiring(p);
    if (p.evenFire || isRotary(p)) {
        for (int k = 0; k < n; ++k) {
            int c = clampv(p.firingOrder[k], 0, n - 1);
            out[c] = (float)(spacing * k / n);
        }
    } else {
        for (int i = 0; i < n; ++i) out[i] = (float)wrapPos(p.phase[i], spacing);
    }
    for (int i = n; i < kMaxCyl; ++i) out[i] = 0;
}

const char* validateFiring(const EngineParams& p) {
    int n = p.cylinders;
    bool seen[kMaxCyl] = {};
    for (int k = 0; k < n; ++k) {
        int c = p.firingOrder[k];
        if (c < 0 || c >= n) return "Firing order references a cylinder that does not exist";
        if (seen[c]) return "Firing order lists a cylinder twice";
        seen[c] = true;
    }
    if (!p.evenFire) {
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if (std::fabs(wrapSym(p.phase[i] - p.phase[j], 720.0)) < 5.0)
                    return "Two cylinders fire within 5 deg of each other (big-bang?)";
    }
    return nullptr;
}

static void setOrder(EngineParams& p, std::initializer_list<int> oneBased) {
    int k = 0;
    for (int c : oneBased) p.firingOrder[k++] = c - 1;
}

static EngineParams base() {
    EngineParams p;
    std::memset(&p, 0, sizeof(p));
    std::snprintf(p.name, sizeof(p.name), "Custom");
    p.baro_kPa = 101.325f; p.coolingFan_kmh = 50; p.stepRpm = 500; p.stepHold_s = 1.5f;
    p.frictionMult = 1.0f; p.redline = 0;
    p.cylinders = 4; p.layout = Layout::Inline; p.bankAngle = 90;
    p.bore_mm = 86; p.stroke_mm = 86; p.rod_mm = 145; p.compression = 10.5f;
    p.flywheel = 0.12f;
    for (int i = 0; i < kMaxCyl; ++i) { p.firingOrder[i] = i; p.bank[i] = 0; p.primaryLen_mm[i] = 700; }
    p.evenFire = 1;

    p.intakeValves = 2; p.exhaustValves = 2;
    p.intakeValveDia_mm = 34; p.exhaustValveDia_mm = 29;
    p.intakeDuration = 248; p.exhaustDuration = 248;
    p.intakeLift_mm = 10; p.exhaustLift_mm = 9.5f;
    p.intakeCenterline = 108; p.exhaustCenterline = 112;
    p.valveLash_mm = 0.15f;
    p.camLayout = CamLayout::DOHC; p.camDrive = CamDrive::Chain; p.cooling = Cooling::Liquid;
    p.vtec = 0; p.vtecInDuration = 248; p.vtecInLift_mm = 10; p.vtecExDuration = 248; p.vtecExLift_mm = 9.5f;
    p.vtc = 0; p.vtcRange = 0;

    p.baseAdvance = 12; p.mechAdvance = 20; p.mechAdvanceRpm = 3500; p.vacuumAdvance = 10;
    p.revLimit = 7000; p.limiter = Limiter::FuelCut;

    p.throttleBore_mm = 62; p.itb = 0; p.plenum_L = 3.0f;
    p.runnerLen_mm = 350; p.runnerDia_mm = 40; p.airFilter = 0.4f;

    p.fuelSystem = FuelSystem::EFI; p.injection = Injection::Port;
    p.injector_ccmin = 280; p.afrCruise = 14.7f; p.afrWot = 12.8f; p.veLearning = 1;
    p.injectionAngle = 330; p.idleTargetRpm = 850;
    p.carbBarrels = 2; p.venturi_mm = 32; p.mainJet = 1.0f; p.idleMixture = 1.0f;
    p.idleScrew = 3.0f; p.accelPump = 1.0f; p.choke = 0;
    p.fuelType = 2; p.flexSensor = 1;   // premium RON 95 (kFuelPremium95), flex fuel sensor fitted
    p.dieselInjRate = 40; p.dieselMaxFuel = 40; p.soiBase = 4; p.soiAdvance = 8; p.glowPlugs = 0;   // (diesel presets only)
    p.egrValveDia_mm = 0; p.vgt = 0; p.vgtMinArea = 0.3f;
    p.egrCooler = 0; p.egrCoolerEff = 0.85f;
    p.dieselInjSystem = 0; p.railMaxBar = 1600; p.nozzleHoles = 7; p.nozzleHoleDia_mm = 0.12f;
    p.emapSensor = 1; p.widebandPos = 1; p.egtPerCyl = 0; p.egtPostTurbo = 1;
    p.refuelType = 7; p.refuelLitres = 20;   // E85

    p.header = HeaderType::Collector; p.primaryDia_mm = 38; p.secondaryLen_mm = 400;
    p.collectorLen_mm = 300; p.collectorDia_mm = 55; p.crossover = Crossover::TrueDual;
    p.pipeDia_mm = 55; p.midpipeLen_mm = 1500; p.cat = 1; p.resonator = 1;
    p.muffler = Muffler::Chambered; p.tailpipeLen_mm = 600; p.resonatorHz = 0; p.mufflerExtTubes = 0;
    p.exhaustSplit = 0; p.exhaustValveMode = 0; p.exhaustValveRpm = 3500; p.wgScreamer = 0;
    p.turboCount = 1; p.twinMode = 0; p.seqRpm = 4500;

    p.throttle = 0; p.ignition = 0; p.starter = 0; p.ambientTemp_C = 25;
    p.listener = ListenerPos::Tailpipe; p.volume = 1.0f; p.oversample = 2;
    p.dynoMode = DynoMode::Off; p.dynoHoldRpm = 3000;
    p.sweepStartRpm = 1500; p.sweepEndRpm = 7000; p.sweepRate = 600;

    p.engineType = EngineType::Piston;
    p.rotorRadius_mm = 105; p.eccentricity_mm = 15; p.rotorWidth_mm = 80;
    applyRotaryPorting(p, 0);
    p.apexLeak = 0.05f;
    // two-stroke ports and scavenging (two-stroke presets only)
    p.tsExhaustOpen = 82; p.tsTransferOpen = 116; p.tsExhaustWidth = 0.70f; p.tsTransferWidth = 1.10f;
    p.tsCrankcaseCR = 1.40f; p.tsReedArea_cm2 = 20; p.tsScavenge = 9;   // (YAM14: a good 5-port loop cylinder)
    p.tsPipe = 0; p.tsPipeRpm = 8000; p.tsPipeClass = 0.5f;
    p.tsBurnDeg = 50;   // (Blair's measured 41-64 deg: a middle value)
    p.tsTransferLen = 1.5f;
    p.tsPowerValve = 0; p.tsPvLowOpen = 95; p.tsPvRpmLow = 6000; p.tsPvRpmHigh = 8500;
    p.tsInlet = 0; p.tsInletOpen = 100; p.tsInletClose = 80; p.tsInletArea_cm2 = 12;   // (Blair Fig. 1.9: motocross / GP)
    p.tsOilRatio = 40;
    p.tsBurnFromSquish = 0; p.tsSquishArea = 0.5f; p.tsSquishClear_mm = 1.2f;   // (Blair's Prog.4.1 example)
    p.tsPipeCustom = 0;
    {   // (Blair's 125 cc racing engine, pipe "A2": sec. 5.5.2)
        const float L[7] = {83, 189, 209, 65, 78, 205, 250}, D[7] = {37.5f, 48.5f, 100, 116, 116, 21, 21};
        for (int j = 0; j < 7; ++j) { p.tsPipeLen_mm[j] = L[j]; p.tsPipeDia_mm[j] = D[j]; }
    }

    p.induction = Induction::Natural; p.scType = Supercharger::Roots;
    p.compFlow_gs = 220; p.turbineAR = 0.64f; p.turboInertia = 0.15f; p.wastegateBar = 1.0f;
    p.intercooler = 1; p.icEfficiency = 0.75f; p.chargeVolume_L = 3.0f; p.bov = 1; p.antiLag = 0;
    p.bovType = 0; p.bovSpringBar = 0.25f; p.compBlades = 12;   // (6 full + 6 splitter blades: a common automotive wheel)
    p.boostRetard = 4.0f; p.afrBoost = 11.6f;
    p.scDisplacement_L = 1.9f; p.scRatio = 1.6f; p.scBypass = 1;

    p.vehicle = 0; p.transMode = TransMode::ManualAutoClutch;
    p.gear = 0; p.clutchPedal = 0; p.brake = 0; p.grade_pct = 0;
    p.nitrous = 0; p.n2oShotHp = 100; p.n2oWet = 1; p.n2oBottleKg = 4.5f; p.n2oRefill = 0;   // 10 lb bottle
    p.n2oArm = 0; p.transbrake = 0; p.rollingAlsButton = 0;
    p.exhaustRasp = 0.3f; p.exhaustPunch = 0.4f; p.exhaustSteep = 0.0f;
    p.soundStyle = 0; p.pressureMix = 0.5f; p.smoothHz = 4500.0f; p.environment = Environment::OpenRoad; p.reverbMix = 0.35f;
    p.irChoice = 1; p.irMix = 0.7f; p.imperfection = 0.6f;
    p.ecuRateHz = 1000; p.coldStart = 0;
    defaultSensorConfig(p.sensors);
    return p;
}

void applyVehicleDefaults(EngineParams& p) {
    const double L = equivalentDisplacementL(p);
    const bool bike = p.cylinders <= 2 && L < 2.0 && !isRotary(p);
    const bool boosted = isBoosted(p);
    if (bike) {
        static const float r[] = {2.75f, 1.95f, 1.52f, 1.26f, 1.09f, 0.96f};
        p.gears = 6; for (int i = 0; i < kMaxGears; ++i) p.gearRatio[i] = i < 6 ? r[i] : 0.9f;
        p.finalDrive = (float)(L > 1.0 ? 3.6 : 5.2);   // primary x final chain drive
        p.mass_kg = (float)(L > 1.0 ? 380 : 230);      // incl. rider
        p.cdA = 0.48f; p.crr = 0.015f; p.tireRadius_mm = 315; p.grip = 1.1f; p.driveFrac = 0.55f;
        p.drivelineEff = 0.92f;
    } else {
        static const float r[] = {3.60f, 2.10f, 1.45f, 1.10f, 0.90f, 0.76f, 0.65f, 0.58f};
        p.gears = 6; for (int i = 0; i < kMaxGears; ++i) p.gearRatio[i] = r[i];
        p.mass_kg = (float)clampv(950.0 + 170.0 * L, 900.0, 2200.0);
        p.cdA = (float)clampv(0.58 + 0.025 * L, 0.55, 0.85);
        p.crr = 0.012f; p.tireRadius_mm = (float)clampv(300.0 + 6.0 * L, 300.0, 345.0);
        p.grip = 1.15f; p.driveFrac = L > 3.0 ? 0.5f : 0.6f; p.drivelineEff = 0.9f;
        // gear the car so top gear lands near the rev limit at a sensible top speed
        double vtop = clampv(140.0 + 25.0 * L * (boosted ? 1.4 : 1.0), 150.0, 330.0) / 3.6;
        double wheelRps = vtop * 1.3 / (2 * 3.14159265 * p.tireRadius_mm * 1e-3);  // relaxed cruising gear
        p.finalDrive = (float)clampv(p.revLimit * 0.95 / 60.0 / wheelRps / p.gearRatio[5], 2.4, 5.5);
    }
    p.clutchTorque = (float)clampv(170.0 * L * (boosted ? 1.9 : 1.0) * (bike ? 1.4 : 1.0), 120.0, 2000.0);
    p.grade_pct = 0;
}

void applyInductionDefaults(EngineParams& p, Induction ind, Supercharger sc) {
    p.induction = ind;
    p.scType = sc;
    double L = equivalentDisplacementL(p);
    if (ind == Induction::Turbo) {
        p.compFlow_gs = (float)clampv(140.0 * L, 60.0, 900.0);
        p.turbineAR = 0.64f; p.turboInertia = (float)clampv(0.08 * L, 0.05, 0.6);
        p.wastegateBar = 0.9f; p.chargeVolume_L = (float)clampv(1.4 * L, 1.0, 12.0);
    } else if (ind == Induction::Supercharger) {
        if (sc == Supercharger::Centrifugal) { p.compFlow_gs = (float)clampv(120.0 * L, 60.0, 900.0); p.scRatio = 9.0f; }
        else { p.scDisplacement_L = (float)clampv(0.42 * L, 0.3, 6.0); p.scRatio = 1.8f; }
        p.chargeVolume_L = (float)clampv(0.8 * L, 0.8, 8.0);
    }
    if (ind != Induction::Natural) {
        p.compression = std::min(p.compression, isRotary(p) ? 8.8f : 9.5f);
        p.injector_ccmin = std::max(p.injector_ccmin, (float)(L / p.cylinders * 650.0 * (isRotary(p) ? 0.5 : 1.0)));
        p.intercooler = 1; p.bov = 1; p.afrBoost = 11.6f; p.boostRetard = 4.0f;
        p.limiter = Limiter::IgnitionCut;
    }
}

// Two turbos for one engine: each parallel twin passes about half the air, so each compressor is a bit over half the
// size (0.55x - a pair of small ones has a little more total flow than the one big turbo they replace, chosen) and its
// rotor - the wheel's moment of inertia going with the diameter to about the 4th power - under half (0.45x, chosen):
// the reason twins spool quicker. A sequential pair's first turbo carries the whole engine alone up to the change-over:
// 0.70x / 0.63x (chosen - at 0.55x the 13B-style pair's first turbo ran past its top speed before the change-over, and
// the change-over dipped the boost by a third)
static float twinFlowK(const EngineParams& p) { return p.twinMode == 1 ? 0.70f : 0.55f; }
static float twinInertiaK(const EngineParams& p) { return p.twinMode == 1 ? 0.63f : 0.45f; }
void applyTurboCount(EngineParams& p, int count) {
    count = count >= 2 ? 2 : 1;
    const int was = p.turboCount >= 2 ? 2 : 1;
    if (count == was) { p.turboCount = count; return; }
    const float fk = twinFlowK(p), ik = twinInertiaK(p);
    p.compFlow_gs = clampv(count == 2 ? p.compFlow_gs * fk : p.compFlow_gs / fk, 40.0f, 900.0f);
    p.turboInertia = clampv(count == 2 ? p.turboInertia * ik : p.turboInertia / ik, 0.02f, 0.8f);
    p.turboCount = count;
}
void applyTwinMode(EngineParams& p, int mode) {
    mode = mode == 1 ? 1 : 0;
    if (p.turboCount >= 2 && mode != p.twinMode) {
        const float f0 = twinFlowK(p), i0 = twinInertiaK(p);
        p.twinMode = mode;
        p.compFlow_gs = clampv(p.compFlow_gs / f0 * twinFlowK(p), 40.0f, 900.0f);
        p.turboInertia = clampv(p.turboInertia / i0 * twinInertiaK(p), 0.02f, 0.8f);
    }
    p.twinMode = mode;
}

const char* rotaryPortingName(int style) {
    static const char* n[] = {"Stock side port", "Street port", "Bridge port", "Peripheral port"};
    return n[clampv(style, 0, 3)];
}

void applyRotaryPorting(EngineParams& p, int style) {
    switch (style) {
    default:
    case 0: // stock side intake ports, small overlap, smooth idle
        p.rIntakeOpen = 32; p.rIntakeClose = 40; p.rIntakeArea_cm2 = 8.0f;
        p.rExhaustOpen = 75; p.rExhaustClose = 48; p.rExhaustArea_cm2 = 6.5f; break;
    case 1: // enlarged side ports
        p.rIntakeOpen = 20; p.rIntakeClose = 55; p.rIntakeArea_cm2 = 9.5f;
        p.rExhaustOpen = 80; p.rExhaustClose = 52; p.rExhaustArea_cm2 = 7.0f; break;
    case 2: // bridge port: extra port above the apex seal path -> big overlap, brap-brap idle
        p.rIntakeOpen = -25; p.rIntakeClose = 65; p.rIntakeArea_cm2 = 12.0f;
        p.rExhaustOpen = 85; p.rExhaustClose = 62; p.rExhaustArea_cm2 = 8.0f; break;
    case 3: // peripheral intake port: race, huge overlap, no low end
        p.rIntakeOpen = -60; p.rIntakeClose = 75; p.rIntakeArea_cm2 = 14.0f;
        p.rExhaustOpen = 88; p.rExhaustClose = 68; p.rExhaustArea_cm2 = 8.5f; break;
    }
}

void applyLayoutDefaults(EngineParams& p) {
    if (isRotary(p)) {
        int n = p.cylinders = clampv(p.cylinders, 1, 4);
        p.layout = Layout::Inline;
        for (int i = 0; i < kMaxCyl; ++i) { p.bank[i] = 0; p.firingOrder[i] = i; p.phase[i] = 0; }
        p.evenFire = 1;
        if (p.crossover != Crossover::MergeY) p.crossover = Crossover::MergeY;
        if (n <= 2 && p.header == HeaderType::TriY) p.header = HeaderType::Collector;
        if (p.injectionAngle > 540) p.injectionAngle = 450;
        return;
    }
    int n = p.cylinders = clampv(p.cylinders, 1, kMaxCyl);
    if (p.layout != Layout::Inline && n < 2) p.layout = Layout::Inline;
    // banks: first half bank 0, second half bank 1
    for (int i = 0; i < kMaxCyl; ++i)
        p.bank[i] = (p.layout == Layout::Inline) ? 0 : (i < (n + 1) / 2 ? 0 : 1);
    // a sensible generic firing order
    switch (n) {
    case 1: setOrder(p, {1}); break;
    case 2: setOrder(p, {1, 2}); break;
    case 3: setOrder(p, {1, 3, 2}); break;
    case 4: if (p.layout == Layout::Boxer) setOrder(p, {1, 3, 2, 4}); else setOrder(p, {1, 3, 4, 2}); break;
    case 5: setOrder(p, {1, 2, 4, 5, 3}); break;
    case 6: if (p.layout == Layout::Inline) setOrder(p, {1, 5, 3, 6, 2, 4}); else setOrder(p, {1, 4, 2, 5, 3, 6}); break;
    case 8: if (p.layout == Layout::Inline) setOrder(p, {1, 6, 2, 5, 8, 3, 7, 4}); else setOrder(p, {1, 8, 4, 3, 6, 5, 7, 2}); break;
    case 10: setOrder(p, {1, 6, 5, 10, 2, 7, 3, 8, 4, 9}); break;
    case 12: setOrder(p, {1, 7, 5, 11, 3, 9, 6, 12, 2, 8, 4, 10}); break;
    default: {
        // alternate between banks / spread along the crank
        int k = 0;
        for (int i = 0; i < n; i += 2) p.firingOrder[k++] = i;
        for (int i = 1; i < n; i += 2) p.firingOrder[k++] = i;
    }
    }
    p.evenFire = 1;
    for (int i = 0; i < n; ++i) p.phase[i] = 0;
    float ph[kMaxCyl]; computePhases(p, ph);
    for (int i = 0; i < n; ++i) p.phase[i] = ph[i];
    if (p.layout == Layout::Inline && p.crossover != Crossover::MergeY) p.crossover = Crossover::MergeY;
    if (n <= 2 && p.header == HeaderType::TriY) p.header = HeaderType::Collector;
}

static void finish(EngineParams& p) {
    float ph[kMaxCyl]; computePhases(p, ph);
    for (int i = 0; i < p.cylinders; ++i) if (p.evenFire) p.phase[i] = ph[i];
}

// two-stroke: the geometric compression ratio that gives a trapped one (from the exhaust port's closing, the way the
// makers quote it) - the clearance volume is the trapped volume / (trapped ratio - 1)
static float trappedToGeometricCr(const EngineParams& p, float crTrapped) {
    const double r = p.stroke_mm * 0.5, l = std::max((double)p.rod_mm, r * 1.1), t = clampv((double)p.tsExhaustOpen, 45.0, 150.0) * 3.14159265 / 180.0;
    const double s = r * (1.0 - std::cos(t)) + l - std::sqrt(l * l - r * r * std::sin(t) * std::sin(t));
    const double vc = s / std::max(1.01, (double)crTrapped - 1.0);   // per unit piston area (mm)
    return (float)((p.stroke_mm + vc) / vc);
}

static const char* kNames[] = {
    "I4 2.0 K20A-style (i-VTEC)",
    "I3 1.0 EFI",
    "I6 3.0 EFI (straight six)",
    "V6 3.5 EFI",
    "V8 5.7 crossplane carb (muscle)",
    "V8 4.5 flatplane EFI (exotic)",
    "V10 5.2 EFI",
    "V12 6.0 EFI",
    "Flat-4 2.5 boxer, unequal headers",
    "V-twin 1.6 45deg carb",
    "Single 0.5 thumper carb",
    "I4 1.6 ITB race",
    "Rotary 2-rotor 1.3L (Mazda 13B-style)",
    "Rotary 3-rotor 2.0L (20B-style)",
    "Rotary 4-rotor 2.6L peripheral port race (26B-style)",
    "Rotary 1-rotor 0.65L carb",
    "I4 2.0 turbo EFI (hot hatch)",
    "V8 6.2 Roots blower (supercharged muscle)",
    "Rotary 2-rotor 1.3L sequential twin turbo (13B-REW-style)",
    "I6 3.0 big single turbo (drag)",
    "V6 3.5 centrifugal supercharger",
    "I5 2.5 EFI (five cylinder)",
    "Flat-6 3.6 EFI (boxer six)",
    "Rotary 2-rotor 1.3L bridge port (brap idle)",
    "I4 1.9 turbo diesel (TDI-style)",
    "1-cyl 250 two-stroke (MX-style)",
    "I4 2.0 common rail diesel (EA189-style)",
    "I6 5.9 inline-pump truck diesel (6BT-style)",
    "V8 6.6 common rail diesel (Duramax L5P-style)",
    "1-cyl 0.46 air-cooled diesel (Hatz 1B40-style)",
    "2-cyl 350 two-stroke YPVS (RD350-style)",
    "3-cyl 500 piston-port two-stroke (H1-style)",
    "1-cyl 300 two-stroke EFI (300 TPI-style)",
    "I6 3.0 sequential twin turbo (2JZ-GTE-style)",
    "I6 2.6 parallel twin turbo ITB (RB26DETT-style)",
    "V6 3.0 twin turbo, one per bank (VG30DETT-style)",
};

int presetCount() { return (int)(sizeof(kNames) / sizeof(kNames[0])); }
const char* presetName(int i) { return kNames[clampv(i, 0, presetCount() - 1)]; }

EngineParams makePreset(int idx) {
    EngineParams p = base();
    std::snprintf(p.name, sizeof(p.name), "%s", presetName(idx));
    switch (idx) {
    default:
    case 0: // I4 2.0 K20A-style: square 86 x 86, high compression, i-VTEC (two-stage cams + intake cam phaser),
            // revs to 8600. Low cam with almost no overlap for a clean idle, high cam from ~5800 rpm under load.
        p.cylinders = 4; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 86; p.stroke_mm = 86; p.rod_mm = 139; p.compression = 11.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 35; p.exhaustValveDia_mm = 30;
        p.intakeDuration = 236; p.intakeLift_mm = 9.8f; p.exhaustDuration = 232; p.exhaustLift_mm = 9.4f;
        p.intakeCenterline = 114; p.exhaustCenterline = 110; p.valveLash_mm = 0.2f;
        p.vtec = 1; p.vtecInDuration = 272; p.vtecInLift_mm = 12.0f; p.vtecExDuration = 266; p.vtecExLift_mm = 11.3f;
        p.vtc = 1; p.vtcRange = 50;
        p.baseAdvance = 12; p.mechAdvance = 18; p.mechAdvanceRpm = 4500; p.vacuumAdvance = 12;   // ~30 deg at WOT (CR 11.5)
        p.revLimit = 8600; p.limiter = Limiter::FuelCut; p.flywheel = 0.12f;
        p.throttleBore_mm = 62; p.plenum_L = 2.6f; p.runnerLen_mm = 260; p.runnerDia_mm = 38; p.airFilter = 0.3f;
        p.injector_ccmin = 310; p.idleTargetRpm = 850; p.afrWot = 12.8f;
        p.header = HeaderType::TriY; p.primaryDia_mm = 38; p.secondaryLen_mm = 380; p.collectorDia_mm = 54;
        for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = 380;
        p.collectorLen_mm = 250; p.pipeDia_mm = 57; p.muffler = Muffler::StraightThrough;
        p.sweepStartRpm = 1500; p.sweepEndRpm = 8400;
        break;
    case 1: // I3 1.0
        p.cylinders = 3; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 72; p.stroke_mm = 82; p.rod_mm = 130; p.compression = 11;
        p.intakeValveDia_mm = 28; p.exhaustValveDia_mm = 24; p.throttleBore_mm = 48;
        p.header = HeaderType::Log; for (int i = 0; i < 3; ++i) p.primaryLen_mm[i] = 140;
        p.primaryDia_mm = 32; p.collectorDia_mm = 42; p.pipeDia_mm = 42; p.revLimit = 6500;
        p.flywheel = 0.09f; p.injector_ccmin = 180; p.runnerDia_mm = 32; p.plenum_L = 1.6f;
        break;
    case 2: // I6 3.0
        p.cylinders = 6; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 84; p.stroke_mm = 89.6f; p.rod_mm = 135; p.compression = 10.2f;
        p.intakeValveDia_mm = 33; p.exhaustValveDia_mm = 29; p.throttleBore_mm = 72;
        p.header = HeaderType::TriY; p.primaryDia_mm = 38; p.secondaryLen_mm = 350;
        p.collectorDia_mm = 60; p.pipeDia_mm = 60; p.revLimit = 7000; p.flywheel = 0.16f;
        p.plenum_L = 4.0f;
        break;
    case 3: // V6 3.5 (60 deg)
        p.cylinders = 6; p.layout = Layout::V; p.bankAngle = 60; applyLayoutDefaults(p);
        p.bore_mm = 93; p.stroke_mm = 86; p.rod_mm = 152; p.compression = 10.5f;
        p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 30; p.throttleBore_mm = 70;
        p.crossover = Crossover::MergeY; p.header = HeaderType::Log;
        for (int i = 0; i < 6; ++i) p.primaryLen_mm[i] = 220;
        p.primaryDia_mm = 38; p.pipeDia_mm = 60; p.collectorDia_mm = 55; p.flywheel = 0.15f;
        p.plenum_L = 4.5f; p.injector_ccmin = 320; p.revLimit = 6800;
        break;
    case 4: // V8 crossplane carb
        p.cylinders = 8; p.layout = Layout::V; p.bankAngle = 90; applyLayoutDefaults(p);
        setOrder(p, {1, 8, 4, 3, 6, 5, 7, 2});
        // classic GM numbering: odd = left bank, even = right bank
        for (int i = 0; i < 8; ++i) p.bank[i] = (i % 2 == 0) ? 0 : 1;
        p.bore_mm = 101.6f; p.stroke_mm = 88.4f; p.rod_mm = 145; p.compression = 9.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 49; p.exhaustValveDia_mm = 39;
        p.intakeDuration = 270; p.exhaustDuration = 280; p.intakeLift_mm = 11.8f; p.exhaustLift_mm = 12.0f;
        p.intakeCenterline = 108; p.exhaustCenterline = 116;
        p.fuelSystem = FuelSystem::Carburetor; p.carbBarrels = 4; p.venturi_mm = 32; p.throttleBore_mm = 43;
        p.idleScrew = 4.0f; p.baseAdvance = 14; p.mechAdvance = 20; p.mechAdvanceRpm = 3000;
        p.header = HeaderType::Collector; for (int i = 0; i < 8; ++i) p.primaryLen_mm[i] = 800;
        p.primaryDia_mm = 44; p.collectorDia_mm = 76; p.pipeDia_mm = 63; p.crossover = Crossover::HPipe;
        p.cat = 0; p.resonator = 0; p.muffler = Muffler::Chambered; p.revLimit = 6200;
        p.flywheel = 0.3f; p.plenum_L = 4.0f; p.runnerLen_mm = 200; p.runnerDia_mm = 48;
        break;
    case 5: // V8 flatplane
        p.cylinders = 8; p.layout = Layout::V; p.bankAngle = 90; applyLayoutDefaults(p);
        setOrder(p, {1, 5, 3, 7, 4, 8, 2, 6}); // bank 0 = 1-4, bank 1 = 5-8 -> alternates banks
        p.bore_mm = 94; p.stroke_mm = 81; p.rod_mm = 138; p.compression = 12.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 40; p.exhaustValveDia_mm = 33;
        p.intakeDuration = 268; p.exhaustDuration = 264; p.intakeLift_mm = 12; p.exhaustLift_mm = 11;
        p.header = HeaderType::Collector; for (int i = 0; i < 8; ++i) p.primaryLen_mm[i] = 550;
        p.primaryDia_mm = 42; p.collectorDia_mm = 65; p.pipeDia_mm = 60; p.crossover = Crossover::XPipe;
        p.muffler = Muffler::StraightThrough; p.resonator = 0; p.revLimit = 9000; p.flywheel = 0.17f;
        p.mechAdvanceRpm = 5000; p.mechAdvance = 22; p.throttleBore_mm = 80; p.injector_ccmin = 450;
        p.plenum_L = 4; p.runnerLen_mm = 260; p.idleTargetRpm = 1100;
        break;
    case 6: // V10 (72 deg even fire)
        p.cylinders = 10; p.layout = Layout::V; p.bankAngle = 72; applyLayoutDefaults(p);
        p.bore_mm = 84.5f; p.stroke_mm = 92.8f; p.rod_mm = 150; p.compression = 12.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 30;
        p.intakeDuration = 264; p.exhaustDuration = 264; p.intakeLift_mm = 11.5f; p.exhaustLift_mm = 10.5f;
        for (int i = 0; i < 10; ++i) p.primaryLen_mm[i] = 600;
        p.primaryDia_mm = 40; p.collectorDia_mm = 63; p.pipeDia_mm = 63; p.crossover = Crossover::XPipe;
        p.muffler = Muffler::StraightThrough; p.revLimit = 8700; p.flywheel = 0.14f;
        p.throttleBore_mm = 82; p.injector_ccmin = 420; p.plenum_L = 6; p.idleTargetRpm = 1000;
        break;
    case 7: // V12
        p.cylinders = 12; p.layout = Layout::V; p.bankAngle = 60; applyLayoutDefaults(p);
        p.bore_mm = 89; p.stroke_mm = 80; p.rod_mm = 140; p.compression = 11.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 30;
        p.intakeDuration = 260; p.exhaustDuration = 260; p.intakeLift_mm = 11; p.exhaustLift_mm = 10;
        p.header = HeaderType::TriY; for (int i = 0; i < 12; ++i) p.primaryLen_mm[i] = 500;
        p.primaryDia_mm = 40; p.secondaryLen_mm = 350; p.collectorDia_mm = 63; p.pipeDia_mm = 63;
        p.crossover = Crossover::XPipe; p.muffler = Muffler::StraightThrough; p.revLimit = 8500;
        p.flywheel = 0.20f; p.throttleBore_mm = 90; p.injector_ccmin = 380; p.plenum_L = 7; p.idleTargetRpm = 900;
        break;
    case 8: // Flat-4 boxer, unequal length headers -> rumble
        p.cylinders = 4; p.layout = Layout::Boxer; p.bankAngle = 180; applyLayoutDefaults(p);
        setOrder(p, {1, 3, 2, 4});
        p.bank[0] = 0; p.bank[1] = 1; p.bank[2] = 0; p.bank[3] = 1;
        p.bore_mm = 99.5f; p.stroke_mm = 79; p.rod_mm = 130; p.compression = 10;
        p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 30;
        p.header = HeaderType::Collector; p.crossover = Crossover::MergeY;
        p.primaryLen_mm[0] = 450; p.primaryLen_mm[1] = 1150; p.primaryLen_mm[2] = 1000; p.primaryLen_mm[3] = 500;
        p.primaryDia_mm = 40; p.collectorDia_mm = 55; p.pipeDia_mm = 55; p.flywheel = 0.13f;
        p.muffler = Muffler::Turbo; p.revLimit = 6700; p.throttleBore_mm = 64;
        break;
    case 9: // 45 deg V-twin, odd fire 315/405
        p.cylinders = 2; p.layout = Layout::V; p.bankAngle = 45; applyLayoutDefaults(p);
        p.evenFire = 0; p.phase[0] = 0; p.phase[1] = 405;
        p.bank[0] = 0; p.bank[1] = 0;
        p.bore_mm = 101; p.stroke_mm = 101; p.rod_mm = 180; p.compression = 9.2f;
        p.intakeValveDia_mm = 49; p.exhaustValveDia_mm = 41; p.intakeDuration = 250; p.exhaustDuration = 255;
        p.intakeLift_mm = 12; p.exhaustLift_mm = 12;
        p.fuelSystem = FuelSystem::Carburetor; p.carbBarrels = 1; p.venturi_mm = 40; p.throttleBore_mm = 44;
        p.idleScrew = 4.0f; p.header = HeaderType::Collector; p.primaryLen_mm[0] = 700; p.primaryLen_mm[1] = 500;
        p.primaryDia_mm = 45; p.collectorDia_mm = 50; p.pipeDia_mm = 50; p.midpipeLen_mm = 400;
        p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough; p.tailpipeLen_mm = 300;
        p.revLimit = 5600; p.flywheel = 0.10f; p.plenum_L = 0.8f; p.runnerLen_mm = 180; p.runnerDia_mm = 44;
        p.baseAdvance = 10; p.mechAdvance = 22; p.mechAdvanceRpm = 2500;
        break;
    case 10: // single cylinder thumper
        p.cylinders = 1; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 92; p.stroke_mm = 75; p.rod_mm = 130; p.compression = 10.2f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 30;
        p.fuelSystem = FuelSystem::Carburetor; p.carbBarrels = 1; p.venturi_mm = 36; p.throttleBore_mm = 38;
        // accelerator pump 1.3 (was 1.8: a full stroke sprayed ~3x a WOT charge of fuel - lambda ~0.5, rich misfire and a
        // stall at the tip-in once the physics got more exact)
        p.idleScrew = 4.0f; p.accelPump = 1.3f; p.primaryLen_mm[0] = 900; p.primaryDia_mm = 42; p.collectorDia_mm = 45; p.pipeDia_mm = 45;
        p.midpipeLen_mm = 200; p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough; p.tailpipeLen_mm = 250;
        p.revLimit = 7500; p.flywheel = 0.055f; p.plenum_L = 0.4f; p.runnerLen_mm = 150; p.runnerDia_mm = 36;
        break;
    case 11: // I4 ITB race
        p.cylinders = 4; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 81; p.stroke_mm = 77.4f; p.rod_mm = 137; p.compression = 12.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 33; p.exhaustValveDia_mm = 28;
        p.intakeDuration = 290; p.exhaustDuration = 285; p.intakeLift_mm = 12.5f; p.exhaustLift_mm = 11.5f;
        p.intakeCenterline = 104; p.exhaustCenterline = 108;
        p.itb = 1; p.throttleBore_mm = 45; p.runnerLen_mm = 260; p.runnerDia_mm = 42; p.airFilter = 0.05f;
        p.header = HeaderType::TriY; for (int i = 0; i < 4; ++i) p.primaryLen_mm[i] = 780;
        p.primaryDia_mm = 40; p.collectorDia_mm = 57; p.pipeDia_mm = 57; p.cat = 0; p.resonator = 0;
        p.muffler = Muffler::StraightThrough; p.revLimit = 9200; p.flywheel = 0.07f;
        p.injector_ccmin = 380; p.idleTargetRpm = 1200; p.mechAdvanceRpm = 6000;
        p.mechAdvance = 18;   // 30 deg at WOT: at 36 (CR 12.5) the knock control ran out of retard and the protection cut in
        p.limiter = Limiter::IgnitionCut;
        break;
    case 12: case 13: case 14: case 15: { // Wankel rotaries
        p.engineType = EngineType::Rotary;
        p.cylinders = idx == 12 ? 2 : idx == 13 ? 3 : idx == 14 ? 4 : 1;
        applyLayoutDefaults(p);
        p.rotorRadius_mm = 105; p.eccentricity_mm = 15; p.rotorWidth_mm = 80; p.compression = 9.4f;
        p.flywheel = 0.15f; p.apexLeak = 0.05f;
        // rotary sound: the rasp of the abrupt exhaust port opening matters more than the low pressure pulse -
        // with the default (pressure-heavy) mix the lumpy idle of a ported rotary is buried under a steady rumble
        p.pressureMix = 0.1f; p.exhaustPunch = 0.15f;
        p.baseAdvance = 5; p.mechAdvance = 20; p.mechAdvanceRpm = 5000; p.vacuumAdvance = 6;
        p.revLimit = 8500; p.limiter = Limiter::FuelCut; p.injectionAngle = 450;
        p.throttleBore_mm = 60; p.plenum_L = 3.0f; p.runnerLen_mm = 420; p.runnerDia_mm = 44;
        p.injector_ccmin = 550; p.idleTargetRpm = 1000;
        p.header = HeaderType::Collector; p.primaryDia_mm = 50; p.collectorDia_mm = 60; p.pipeDia_mm = 60;
        for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = 600;
        p.cat = 1; p.resonator = 1; p.muffler = Muffler::Chambered;
        applyRotaryPorting(p, 0);
        if (idx == 13) {
            p.throttleBore_mm = 70; p.plenum_L = 4.0f; p.injector_ccmin = 650; p.flywheel = 0.19f; p.idleTargetRpm = 1150;   // street port: lots of overlap, rough idle
            p.header = HeaderType::Collector; p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough;
            p.pipeDia_mm = 70; p.collectorDia_mm = 70; p.revLimit = 8000;
            applyRotaryPorting(p, 1);
        } else if (idx == 14) {
            applyRotaryPorting(p, 3);
            p.itb = 1; p.throttleBore_mm = 55; p.runnerLen_mm = 300; p.runnerDia_mm = 52; p.airFilter = 0.05f;
            p.injector_ccmin = 900; p.flywheel = 0.20f; p.revLimit = 9500; p.idleTargetRpm = 1800;
            p.cat = 0; p.resonator = 0; p.muffler = Muffler::None; p.primaryDia_mm = 55; p.collectorDia_mm = 90;
            p.pipeDia_mm = 90; p.midpipeLen_mm = 600; p.tailpipeLen_mm = 400; p.compression = 10.0f;
            p.limiter = Limiter::IgnitionCut; p.mechAdvanceRpm = 6500;
            p.afrCruise = 12.5f;   // race peripheral port: heavy overlap + residual, it only idles cleanly rich
        } else if (idx == 15) {
            p.fuelSystem = FuelSystem::Carburetor; p.carbBarrels = 1; p.venturi_mm = 30; p.throttleBore_mm = 36;
            p.idleScrew = 4.5f; p.plenum_L = 0.6f; p.flywheel = 0.10f; p.cat = 0; p.resonator = 0;
            p.muffler = Muffler::StraightThrough; p.pipeDia_mm = 45; p.primaryDia_mm = 45; p.collectorDia_mm = 45;
            p.midpipeLen_mm = 500; p.tailpipeLen_mm = 300; p.limiter = Limiter::IgnitionCut; p.revLimit = 8000;
            applyRotaryPorting(p, 1);
        }
        p.sweepStartRpm = 2000; p.sweepEndRpm = p.revLimit - 200;
        break;
    }
    case 16: // turbo I4 (on the generic 2.0 I4 that preset 0 used to be)
        p = base();
        p.cylinders = 4; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 86; p.stroke_mm = 86; p.rod_mm = 143; p.compression = 10.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 34; p.exhaustValveDia_mm = 28;
        p.header = HeaderType::TriY; p.primaryDia_mm = 38; p.collectorDia_mm = 50; p.pipeDia_mm = 50;
        p.revLimit = 6800; p.flywheel = 0.12f;
        applyInductionDefaults(p, Induction::Turbo);
        p.compression = 9.3f; p.compFlow_gs = 270; p.turbineAR = 0.34f; p.turboInertia = 0.07f; p.wastegateBar = 1.1f; p.boostRetard = 5;
        p.injector_ccmin = 550; p.revLimit = 6800; p.mechAdvance = 11; p.muffler = Muffler::Chambered;
        p.sweepStartRpm = 1500; p.sweepEndRpm = 6700;
        break;
    case 17: // Roots blown V8
        p = makePreset(4);
        p.fuelSystem = FuelSystem::EFI; p.injection = Injection::Port; p.injectionAngle = 330; p.idleTargetRpm = 900;
        p.intakeDuration = 262; p.exhaustDuration = 268;   // blower cam: less overlap than the NA one
        p.bore_mm = 103.25f; p.stroke_mm = 92; p.throttleBore_mm = 92; p.carbBarrels = 1;
        applyInductionDefaults(p, Induction::Supercharger, Supercharger::Roots);
        p.scDisplacement_L = 2.3f; p.scRatio = 1.9f; p.compression = 9.0f; p.injector_ccmin = 650; p.mechAdvance = 12;
        p.muffler = Muffler::StraightThrough; p.crossover = Crossover::XPipe; p.revLimit = 6500;
        p.limiter = Limiter::IgnitionCut; p.sweepStartRpm = 1500; p.sweepEndRpm = 6400;
        break;
    case 18: // 13B-REW (FD RX-7, 1992-2002): the 13B with a sequential pair of Hitachi HT-12 turbos (Tashima et al., SAE
             // 941030), 9.0:1, ~0.75 bar (11 psi); 255-280 PS at 6500 rpm. The second turbo comes in around 4500 rpm
             // (owner / tuner sources; the paper's own figure is behind its paywall). Sizes: the old single turbo's,
             // split into a sequential pair (applyTurboCount / applyTwinMode: 0.70x flow, 0.63x inertia each) - chosen
        p = makePreset(12);
        applyInductionDefaults(p, Induction::Turbo);
        p.compFlow_gs = 300; p.turbineAR = 0.7f; p.turboInertia = 0.14f; p.wastegateBar = 0.75f;
        p.twinMode = 1; applyTurboCount(p, 2); p.seqRpm = 4500;
        p.compression = 9.0f; p.injector_ccmin = 950; p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough;
        p.pipeDia_mm = 76; p.revLimit = 8000; p.sweepEndRpm = 7800;
        p.idleTargetRpm = 1100;   // big injectors + low compression: a little more idle margin
        break;
    case 33: // Toyota 2JZ-GTE (Supra MkIV, 1993-2002): 86 x 86 mm, 2997 cc, 8.5:1, DOHC 24 valves, a sequential pair of
             // Hitachi CT12B (export) with intake / exhaust / bypass control valves (Supra 1997 repair manual); 280 PS at
             // 5600 rpm (JDM), 320 hp export. Rods 142 mm, valves, cams, the change-over speed (4000 rpm) and 0.8 bar
             // boost chosen
        p = makePreset(2);
        p.bore_mm = 86; p.stroke_mm = 86; p.rod_mm = 142; p.compression = 8.5f;
        p.intakeValveDia_mm = 33.5f; p.exhaustValveDia_mm = 29; p.intakeDuration = 236; p.exhaustDuration = 236;
        p.intakeLift_mm = 8.25f; p.exhaustLift_mm = 8.4f;
        applyInductionDefaults(p, Induction::Turbo);
        p.twinMode = 1; applyTurboCount(p, 2); p.seqRpm = 4000;
        p.turbineAR = 0.6f; p.wastegateBar = 0.8f; p.mechAdvance = 11; p.boostRetard = 5;
        p.header = HeaderType::Collector; p.injector_ccmin = 440; p.revLimit = 6800;
        p.cat = 1; p.resonator = 1; p.muffler = Muffler::Turbo; p.pipeDia_mm = 70;
        p.sweepStartRpm = 1500; p.sweepEndRpm = 6700;
        break;
    case 34: // Nissan RB26DETT (Skyline GT-R R32-R34, 1989-2002): 86 x 73.7 mm, 2568 cc, 8.5:1, DOHC 24 valves, six
             // individual throttle bodies, two Garrett T28-type ceramic turbos in parallel - the front three cylinders on
             // one, the rear three on the other; ~0.7 bar (10 psi), 280 PS at 6800 rpm. Rods 121.5 mm; cams chosen
        p = makePreset(2);
        p.bore_mm = 86; p.stroke_mm = 73.7f; p.rod_mm = 121.5f; p.compression = 8.5f;
        p.intakeValveDia_mm = 34.5f; p.exhaustValveDia_mm = 30; p.intakeDuration = 240; p.exhaustDuration = 236;
        p.intakeLift_mm = 8.6f; p.exhaustLift_mm = 8.6f;
        p.itb = 1; p.throttleBore_mm = 45; p.runnerLen_mm = 280; p.runnerDia_mm = 40;
        applyInductionDefaults(p, Induction::Turbo);
        p.twinMode = 0; applyTurboCount(p, 2);
        p.turbineAR = 0.6f; p.wastegateBar = 0.7f; p.mechAdvance = 11; p.boostRetard = 5;
        p.header = HeaderType::Collector; p.injector_ccmin = 444; p.revLimit = 8000;
        p.cat = 1; p.resonator = 0; p.muffler = Muffler::Chambered; p.pipeDia_mm = 76;
        p.sweepStartRpm = 2000; p.sweepEndRpm = 7900;
        break;
    case 35: // Nissan VG30DETT (300ZX Z32, 1989-2000): 60 deg V6, 87 x 83 mm, 2960 cc, 8.5:1, DOHC 24 valves, a Garrett
             // T22 on each bank (parallel), ~0.75 bar; 300 hp at 6400 rpm, 384 Nm at 3600. A downpipe and system per bank
             // (true dual). Rods, valves, cams chosen
        p = makePreset(3);
        p.bore_mm = 87; p.stroke_mm = 83; p.rod_mm = 154; p.compression = 8.5f;
        p.intakeValveDia_mm = 34; p.exhaustValveDia_mm = 29.5f; p.intakeDuration = 240; p.exhaustDuration = 240;
        p.intakeLift_mm = 9.0f; p.exhaustLift_mm = 8.8f;
        applyInductionDefaults(p, Induction::Turbo);
        p.twinMode = 0; applyTurboCount(p, 2);
        p.turbineAR = 0.55f; p.wastegateBar = 0.75f; p.mechAdvance = 11; p.boostRetard = 5;
        p.header = HeaderType::Collector; p.crossover = Crossover::TrueDual; p.injector_ccmin = 370; p.revLimit = 7000;
        p.cat = 1; p.resonator = 1; p.muffler = Muffler::Chambered; p.pipeDia_mm = 57;
        for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = 300;
        p.sweepStartRpm = 1500; p.sweepEndRpm = 6900;
        break;
    case 19: // big single I6
        p = makePreset(2);
        applyInductionDefaults(p, Induction::Turbo);
        p.compFlow_gs = 650; p.turbineAR = 1.1f; p.turboInertia = 0.45f; p.wastegateBar = 1.9f; p.compression = 8.8f;
        p.injector_ccmin = 1100; p.antiLag = 1; p.mechAdvance = 9; p.boostRetard = 5; p.cat = 0; p.resonator = 0; p.muffler = Muffler::None;
        p.pipeDia_mm = 90; p.collectorDia_mm = 76; p.revLimit = 7800; p.intakeDuration = 264; p.exhaustDuration = 264;
        p.sweepStartRpm = 2500; p.sweepEndRpm = 7600; p.idleTargetRpm = 1100;   // big cam: lumpy, high idle
        break;
    case 21: // I5 2.5: 144 deg firing interval, 1-2-4-5-3, the classic warble
        p.cylinders = 5; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 82.5f; p.stroke_mm = 92.8f; p.rod_mm = 144; p.compression = 10.3f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 33; p.exhaustValveDia_mm = 28;
        p.header = HeaderType::Collector; p.primaryDia_mm = 40; p.collectorDia_mm = 57; p.pipeDia_mm = 63;
        for (int i = 0; i < 5; ++i) p.primaryLen_mm[i] = 520;
        p.revLimit = 7000; p.flywheel = 0.14f; p.throttleBore_mm = 68; p.plenum_L = 3.5f; p.injector_ccmin = 330;
        p.muffler = Muffler::StraightThrough;
        break;
    case 22: // flat-6 3.6 boxer: equal-length headers per bank, smooth 120 deg firing
        p.cylinders = 6; p.layout = Layout::Boxer; p.bankAngle = 180; applyLayoutDefaults(p);
        p.bore_mm = 96; p.stroke_mm = 83; p.rod_mm = 127; p.compression = 11.3f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 37.5f; p.exhaustValveDia_mm = 32;
        p.intakeDuration = 268; p.exhaustDuration = 260; p.intakeLift_mm = 11.5f; p.exhaustLift_mm = 10.5f;
        p.header = HeaderType::Collector; for (int i = 0; i < 6; ++i) p.primaryLen_mm[i] = 600;
        p.primaryDia_mm = 42; p.collectorDia_mm = 57; p.pipeDia_mm = 60; p.crossover = Crossover::HPipe;
        p.revLimit = 7800; p.flywheel = 0.18f; p.throttleBore_mm = 76; p.plenum_L = 5.0f; p.injector_ccmin = 380; p.idleTargetRpm = 900;
        p.muffler = Muffler::StraightThrough;
        break;
    case 23: // bridge-ported 13B: a second intake port above the apex-seal path -> huge overlap. At idle the
             // exhaust floods back into the intake, the diluted charges burn partly or not at all and the
             // unburned charge pops in the hot exhaust: the lumpy "brap brap" idle
        p = makePreset(12);
        applyRotaryPorting(p, 2);
        std::snprintf(p.name, sizeof(p.name), "%s", "Rotary 2-rotor 1.3L bridge port (brap idle)");
        p.idleTargetRpm = 1450; p.afrCruise = 14.0f; p.injector_ccmin = 720; p.flywheel = 0.16f;
        // short common intake (both rotors' ports into a 0.3 L chamber behind one throttle, 50 mm runners): the
        // exhaust that one rotor reverts at idle reaches the other rotor within its next intake, so both rotors share
        // the slowly building dilution. That makes them fire and misfire *together* - runs of ~6 firing events, then
        // ~6 weak / misfiring ones, a 12-event cycle - like a recorded bridge-port 13B idle (2026-11-20, ECU_PLAN
        // "Target brap"). With the earlier 3 L plenum / long runners each rotor ran its own cycle, half a cycle apart,
        // and the misfires spread out into a fast flutter instead of the brap.
        p.throttleBore_mm = 65; p.plenum_L = 0.3f; p.runnerLen_mm = 50; p.revLimit = 9000;
        p.header = HeaderType::Collector; p.primaryDia_mm = 55; p.collectorDia_mm = 65; p.pipeDia_mm = 65;
        for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = 350;
        p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough;
        p.limiter = Limiter::IgnitionCut;
        p.sweepStartRpm = 2000; p.sweepEndRpm = p.revLimit - 200;
        break;
    case 24: // turbo diesel I4 (Package 4): 1.9 L long stroke, compression 19.5, direct injection, no throttle,
             // a small wastegated turbo. The load is the fuel per stroke (pedal map, smoke limiter, governors).
        p = base();
        p.cylinders = 4; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 79.5f; p.stroke_mm = 95.5f; p.rod_mm = 144; p.compression = 19.5f;
        p.intakeValves = 1; p.exhaustValves = 1; p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 31;
        p.intakeDuration = 224; p.exhaustDuration = 228; p.intakeCenterline = 112; p.exhaustCenterline = 112;
        p.header = HeaderType::Log; p.primaryDia_mm = 36; p.collectorDia_mm = 45; p.pipeDia_mm = 50;
        p.revLimit = 4800; p.flywheel = 0.16f; p.idleTargetRpm = 900;
        applyInductionDefaults(p, Induction::Turbo);
        p.compression = 19.5f;   // (after the turbo defaults, which set a petrol turbo's 9.5)
        p.compFlow_gs = 150; p.turbineAR = 0.30f; p.turboInertia = 0.04f; p.wastegateBar = 0.8f;
        p.fuelSystem = FuelSystem::DieselDI; p.fuelType = kFuelDiesel; p.flexSensor = 0;
        // timing / fuel set so the peak cylinder pressure stays at ~105-150 bar over the speed range (with 19.5:1
        // and 0.8 bar boost the motored pressure alone is ~100 bar): ~230 Nm at 2000 rpm, ~210 Nm at 4000
        p.dieselInjRate = 40; p.dieselMaxFuel = 45; p.soiBase = 2; p.soiAdvance = 5;
        p.glowPlugs = 1;   // metal sheathed-element glow plugs
        // variable geometry turbine (closed vanes: a fraction of the open flow area) and an uncooled high-pressure EGR
        // valve - starting values, set so the boost target is held from ~1500 rpm and EGR flows at part load
        p.vgt = 1; p.vgtMinArea = 0.12f; p.turbineAR = 0.8f; p.egrValveDia_mm = 25;
        p.limiter = Limiter::FuelCut; p.muffler = Muffler::Chambered;
        p.sweepStartRpm = 1000; p.sweepEndRpm = 4700;
        break;
    case 25: // two-stroke single (Package 5): 250 cc, 66.4 x 72 mm, crankcase scavenged through a reed valve, loop
             // scavenged; one carburettor; CDI ignition. Port timings / widths, primary compression and the reed
             // area are typical values for this class (ECU_PLAN, package 5). Bore x stroke and compression from the
             // 2002 Honda CR250R (Honda factbook: 66.4 x 72 mm, 8.5:1 - taken as the trapped ratio, the way
             // two-stroke makers quote it): geometric 16 gives 8.5 from the exhaust closing.
        p = base();
        p.engineType = EngineType::TwoStroke;
        p.cylinders = 1; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 66.4f; p.stroke_mm = 72; p.rod_mm = 130; p.compression = 16.0f;
        p.fuelSystem = FuelSystem::Carburetor; p.carbBarrels = 1; p.venturi_mm = 36; p.throttleBore_mm = 38;
        p.idleScrew = 1.0f; p.accelPump = 1.0f; p.idleTargetRpm = 1600;
        p.plenum_L = 0.2f; p.runnerLen_mm = 120; p.runnerDia_mm = 38; p.airFilter = 0.3f;
        // ignition: 14 deg at idle rising to 22 deg by 6000 rpm - best torque on the rig with Blair's burn (12 deg
        // delay + 41 deg), like the 20 deg of his measured GP engine
        p.baseAdvance = 14; p.mechAdvance = 8; p.mechAdvanceRpm = 6000; p.vacuumAdvance = 0;
        p.header = HeaderType::Collector; p.primaryLen_mm[0] = 900; p.primaryDia_mm = 42; p.collectorDia_mm = 45;
        p.collectorLen_mm = 100; p.pipeDia_mm = 45; p.midpipeLen_mm = 200; p.cat = 0; p.resonator = 0;
        p.muffler = Muffler::StraightThrough; p.tailpipeLen_mm = 250;
        p.revLimit = 9500; p.flywheel = 0.02f; p.limiter = Limiter::IgnitionCut;
        p.tsExhaustOpen = 82; p.tsTransferOpen = 116; p.tsExhaustWidth = 0.70f; p.tsTransferWidth = 1.10f;
        p.tsCrankcaseCR = 1.40f; p.tsReedArea_cm2 = 20; p.tsScavenge = 9;
        p.tsBurnDeg = 41;   // a racing chamber with a squish band: Blair's measured 125 GP engine
        // expansion chamber tuned for 8000 rpm after a 150 mm header; straight-through silencer
        p.tsPipe = 1; p.tsPipeRpm = 8000; p.primaryLen_mm[0] = 150;
        p.sweepStartRpm = 3000; p.sweepEndRpm = 9500;
        break;
    // ---- more diesels (D11). Bore x stroke, compression, valvetrain, induction, injection system and ratings from
    //      the sources named; rod lengths, injector rates / nozzles, fuel quantities and the timing / boost / EGR set
    //      points are chosen (calibration starting points sized from the rated torque - see docs/ECU_PLAN.md D11)
    case 26: // VW EA189 2.0 TDI common rail, 103 kW (VW SSP 826803: 81 x 95.5 mm, 16.5:1, DOHC 4 valves - a toothed
             // belt drives one camshaft, spur gears the other -, piezo injectors, 230-1800 bar, VTG turbo, EGR;
             // 103 kW at 4000 rpm, 320 Nm at 1750-2500 rpm). Eight-hole nozzles (Wikipedia, EA189); hole size chosen.
        p = makePreset(24);
        p.bore_mm = 81; p.stroke_mm = 95.5f; p.rod_mm = 144; p.compression = 16.5f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 27; p.exhaustValveDia_mm = 24;
        p.intakeDuration = 230; p.exhaustDuration = 232; p.intakeLift_mm = 8.5f; p.exhaustLift_mm = 8.5f;
        p.dieselInjSystem = 1; p.railMaxBar = 1800; p.nozzleHoles = 8; p.nozzleHoleDia_mm = 0.11f;
        p.dieselMaxFuel = 62; p.soiBase = 2; p.soiAdvance = 8; p.glowPlugs = 2;
        p.vgt = 1; p.vgtMinArea = 0.12f; p.turbineAR = 0.8f; p.compFlow_gs = 190; p.wastegateBar = 1.5f; p.intercooler = 1; p.turboInertia = 0.06f; p.chargeVolume_L = 2.8f;
        p.egrValveDia_mm = 28; p.egrCooler = 1; p.egrCoolerEff = 0.85f;
        p.revLimit = 5000; p.idleTargetRpm = 800; p.flywheel = 0.15f;
        p.sweepStartRpm = 1000; p.sweepEndRpm = 4900;
        break;
    case 27: // Cummins 6BT 5.9 12-valve with the Bosch P7100 inline pump (Dieselhub / Drivingline: 102 x 120 mm, 17.0:1,
             // OHV 2 valves, cam in the block; wastegated turbo, air-to-air intercooler; 1996-98 manual 215 hp at
             // 2600 rpm, 440 lb-ft at 1600; static timing 12 deg BTDC - Diesel World). No glow plugs (an intake grid
             // heater, not modelled).
        p = makePreset(24);
        p.cylinders = 6; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 102; p.stroke_mm = 120; p.rod_mm = 192; p.compression = 17.0f;
        p.intakeValves = 1; p.exhaustValves = 1; p.intakeValveDia_mm = 46; p.exhaustValveDia_mm = 42;
        p.intakeDuration = 230; p.exhaustDuration = 240; p.intakeLift_mm = 11; p.exhaustLift_mm = 11;
        p.dieselInjSystem = 0; p.dieselInjRate = 28; p.dieselMaxFuel = 74; p.soiBase = 12; p.soiAdvance = 0; p.glowPlugs = 0;
        p.vgt = 0; p.turbineAR = 0.45f; p.compFlow_gs = 260; p.wastegateBar = 1.3f; p.intercooler = 1; p.turboInertia = 0.4f; p.chargeVolume_L = 8.0f;
        p.egrValveDia_mm = 0;
        p.header = HeaderType::Log; p.primaryDia_mm = 45; p.collectorDia_mm = 65; p.pipeDia_mm = 89;
        p.revLimit = 2900; p.idleTargetRpm = 750; p.flywheel = 0.9f; p.muffler = Muffler::Turbo;
        p.sweepStartRpm = 900; p.sweepEndRpm = 2800;
        break;
    case 28: // Duramax L5P 6.6 V8 (WardsAuto / Dieselhub / Wikipedia: 103 x 99 mm, 16.0:1, pushrod OHV with four valves
             // per cylinder, Denso HP4 common rail 2000 bar, electronically actuated VGT without a wastegate; 445 hp
             // at 2800 rpm, 910 lb-ft at 1600 - 2017-23)
        p = makePreset(24);
        p.cylinders = 8; p.layout = Layout::V; p.bankAngle = 90; applyLayoutDefaults(p);
        for (int i = 0; i < 8; ++i) p.bank[i] = (i % 2 == 0) ? 0 : 1;
        p.bore_mm = 103; p.stroke_mm = 99; p.rod_mm = 162; p.compression = 16.0f;
        p.intakeValves = 2; p.exhaustValves = 2; p.intakeValveDia_mm = 32; p.exhaustValveDia_mm = 29;
        p.intakeDuration = 232; p.exhaustDuration = 236; p.intakeLift_mm = 10; p.exhaustLift_mm = 10;
        p.dieselInjSystem = 1; p.railMaxBar = 2000; p.nozzleHoles = 8; p.nozzleHoleDia_mm = 0.14f;
        p.dieselMaxFuel = 110; p.soiBase = 2; p.soiAdvance = 8; p.glowPlugs = 2;
        p.vgt = 1; p.vgtMinArea = 0.12f; p.turbineAR = 1.0f; p.compFlow_gs = 650; p.wastegateBar = 1.8f; p.intercooler = 1; p.turboInertia = 0.45f; p.chargeVolume_L = 9.0f;
        p.egrValveDia_mm = 35; p.egrCooler = 1; p.egrCoolerEff = 0.85f;
        p.header = HeaderType::Log; p.primaryDia_mm = 45; p.collectorDia_mm = 65; p.pipeDia_mm = 102;
        p.revLimit = 3400; p.idleTargetRpm = 680; p.flywheel = 0.6f; p.muffler = Muffler::Turbo;
        p.sweepStartRpm = 1000; p.sweepEndRpm = 3300;
        break;
    case 29: // Hatz 1B40 single (Hatz B-series data sheet 2022: 88 x 76 mm, 0.462 L, 20.5:1, direct injection, air-cooled,
             // one cam working the injection pump and - through rockers - the valves; 7.3 kW at 3600 rpm). Static
             // timing taken from the similar Yanmar L100 (15.5 deg BTDC, its spec sheet); no glow plugs.
        p = makePreset(24);
        p.cylinders = 1; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 88; p.stroke_mm = 76; p.rod_mm = 122; p.compression = 20.5f;
        p.intakeValves = 1; p.exhaustValves = 1; p.intakeValveDia_mm = 36; p.exhaustValveDia_mm = 31;
        p.intakeDuration = 228; p.exhaustDuration = 232; p.intakeLift_mm = 8; p.exhaustLift_mm = 8;
        applyInductionDefaults(p, Induction::Natural);
        p.vgt = 0; p.intercooler = 0;
        p.dieselInjSystem = 0; p.dieselInjRate = 15; p.dieselMaxFuel = 21; p.soiBase = 6; p.soiAdvance = 6; p.glowPlugs = 0;
        p.egrValveDia_mm = 0;
        p.header = HeaderType::Collector; p.primaryLen_mm[0] = 300; p.primaryDia_mm = 30; p.collectorDia_mm = 32; p.pipeDia_mm = 32;
        p.muffler = Muffler::Chambered; p.tailpipeLen_mm = 100; p.cat = 0; p.resonator = 0;
        p.revLimit = 3700; p.idleTargetRpm = 1200; p.flywheel = 0.12f;
        p.sweepStartRpm = 1500; p.sweepEndRpm = 3600;
        break;
    // ---- more two-strokes (T11, T13). Bore x stroke, ratings and what the sources name; port timings / widths, rods,
    //      reed / port areas, pipes and ignition curves are chosen (typical values for the class: Blair, Fig. 1.9) -
    //      see docs/ECU_PLAN.md "Paket 5 lanjutan"
    case 30: // Yamaha RD350 YPVS twin (1983-): 64 x 54 mm, 347 cc, reed valves, YPVS power valve - closed below ~6000 rpm,
             // fully open by 8000, exhaust duration 180 deg closed / 200 deg open (Yamaha service manual extract,
             // rd350lc.net); 59 hp at 9000 rpm, 40.2 Nm at 8000 (RD350F2 spec sheet). Trapped compression 7.0 (chosen).
        p = makePreset(25);
        p.cylinders = 2; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 64; p.stroke_mm = 54; p.rod_mm = 105;
        p.tsExhaustOpen = 80; p.tsTransferOpen = 116; p.tsExhaustWidth = 0.68f; p.tsTransferWidth = 1.10f;
        p.tsPowerValve = 1; p.tsPvLowOpen = 90; p.tsPvRpmLow = 6000; p.tsPvRpmHigh = 8000;
        p.tsCrankcaseCR = 1.40f; p.tsReedArea_cm2 = 14; p.tsBurnDeg = 45;
        p.compression = trappedToGeometricCr(p, 7.0f);
        p.venturi_mm = 26; p.throttleBore_mm = 26; p.carbBarrels = 1;
        p.runnerLen_mm = 180; p.runnerDia_mm = 30; p.plenum_L = 0.2f;
        p.tsPipe = 1; p.tsPipeRpm = 8500; p.tsPipeClass = 0.5f; p.primaryLen_mm[0] = 150;
        p.baseAdvance = 14; p.mechAdvance = 6; p.mechAdvanceRpm = 6000;
        p.revLimit = 10000; p.idleTargetRpm = 1300; p.flywheel = 0.03f;
        p.sweepStartRpm = 3000; p.sweepEndRpm = 9800;
        break;
    case 31: // Kawasaki H1 500 Mach III triple (1969-): 60 x 58.8 mm, 498 cc, 6.8:1 (as quoted - taken as trapped),
             // piston-ported, air-cooled, three carburettors; 60 hp at 7500 rpm, 56 Nm at 7000 (Moto-Data, 1970-71).
             // Its own port timings were not found: Blair's typical road / enduro values (Fig. 1.9 - exhaust 97, transfers
             // 120 deg ATDC, inlet opens 75 deg BTDC). Its pipes: broad expansion chambers (enduro design, chosen).
        p = makePreset(25);
        p.cylinders = 3; p.layout = Layout::Inline; applyLayoutDefaults(p);
        p.bore_mm = 60; p.stroke_mm = 58.8f; p.rod_mm = 110;
        p.tsExhaustOpen = 97; p.tsTransferOpen = 120; p.tsExhaustWidth = 0.72f; p.tsTransferWidth = 1.20f;
        p.tsInlet = 1; p.tsInletOpen = 75; p.tsInletArea_cm2 = 7;
        p.tsCrankcaseCR = 1.45f; p.tsScavenge = 9; p.tsBurnDeg = 50;
        p.compression = trappedToGeometricCr(p, 6.8f);
        p.venturi_mm = 28; p.throttleBore_mm = 28; p.carbBarrels = 1;
        p.runnerLen_mm = 160; p.runnerDia_mm = 30; p.plenum_L = 0.2f;
        p.tsPipe = 1; p.tsPipeRpm = 7000; p.tsPipeClass = 0.0f; p.primaryLen_mm[0] = 150;
        p.baseAdvance = 16; p.mechAdvance = 6; p.mechAdvanceRpm = 5000;
        p.revLimit = 8000; p.idleTargetRpm = 1200; p.flywheel = 0.05f;
        p.sweepStartRpm = 2500; p.sweepEndRpm = 7900;
        break;
    case 32: // KTM 300 EXC TPI (2018-): 72 x 72 mm, 293.2 cc, transfer port injection with a 39 mm throttle body and an
             // electronically regulated oil pump (KTM MY22 EXC technical data) - so no oil in the fuel; power valve.
             // Its sibling 300 SX (same engine family, throttle-body injection) made 48.8 hp at 8100 rpm and 32.2 lb-ft
             // (43.7 Nm) at 7500 at the rear wheel (Dirt Rider dyno test, 2023). Injection into the transfer ports is
             // taken as injection at the reed end of the intake tract (the fuel reaches the crankcase the same way).
        p = makePreset(25);
        p.bore_mm = 72; p.stroke_mm = 72; p.rod_mm = 130;
        p.tsExhaustOpen = 82; p.tsTransferOpen = 117; p.tsExhaustWidth = 0.70f; p.tsTransferWidth = 1.15f;
        p.tsPowerValve = 1; p.tsPvLowOpen = 94; p.tsPvRpmLow = 4500; p.tsPvRpmHigh = 7500;
        p.tsCrankcaseCR = 1.40f; p.tsReedArea_cm2 = 20; p.tsBurnDeg = 45; p.tsOilRatio = 0;
        p.compression = trappedToGeometricCr(p, 8.0f);
        p.fuelSystem = FuelSystem::EFI; p.injection = Injection::Port; p.injector_ccmin = 240; p.throttleBore_mm = 39;
        p.runnerLen_mm = 200; p.runnerDia_mm = 39; p.plenum_L = 0.3f;
        p.tsPipe = 1; p.tsPipeRpm = 7500; p.tsPipeClass = 0.0f; p.primaryLen_mm[0] = 150;
        p.baseAdvance = 14; p.mechAdvance = 8; p.mechAdvanceRpm = 6000;
        p.revLimit = 9000; p.idleTargetRpm = 1500; p.flywheel = 0.03f;
        p.sweepStartRpm = 2500; p.sweepEndRpm = 8800;
        break;
    case 20: // centrifugal supercharged V6
        p = makePreset(3);
        applyInductionDefaults(p, Induction::Supercharger, Supercharger::Centrifugal);
        p.compFlow_gs = 380; p.scRatio = 12.5f; p.compression = 9.5f; p.injector_ccmin = 550; p.mechAdvance = 11;
        p.revLimit = 7000; p.muffler = Muffler::StraightThrough; p.sweepEndRpm = 6900;
        break;
    }
    // valvetrain / timing drive / cooling as the engines each preset is modelled on are built (the base is DOHC, chain,
    // liquid): pushrod V8s, a belt-driven Japanese / German DOHC or SOHC, the air-cooled big V-twin and single
    {
        struct Build { int preset; CamLayout cam; CamDrive drive; Cooling cool; };
        static const Build kBuild[] = {
            {1, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},    // I3 1.0 (wet belt)
            {3, CamLayout::SOHC, CamDrive::Belt, Cooling::Liquid},    // 60 deg V6 3.5
            {4, CamLayout::OHV, CamDrive::Chain, Cooling::Liquid},    // crossplane carb V8
            {8, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},    // flat-4
            {9, CamLayout::OHV, CamDrive::Gears, Cooling::Air},       // 45 deg V-twin
            {10, CamLayout::SOHC, CamDrive::Chain, Cooling::Air},     // single thumper
            {16, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},   // turbo I4
            {17, CamLayout::OHV, CamDrive::Chain, Cooling::Liquid},   // Roots V8
            {20, CamLayout::SOHC, CamDrive::Belt, Cooling::Liquid},   // on the V6 3.5
            {21, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},   // I5 2.5
            {24, CamLayout::SOHC, CamDrive::Belt, Cooling::Liquid},   // 1.9 turbo diesel, two valves per cylinder
            {26, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},   // EA189 (belt to one cam, gears to the other)
            {27, CamLayout::OHV, CamDrive::Gears, Cooling::Liquid},   // 6BT (cam in the block; gear drive chosen)
            {28, CamLayout::OHV, CamDrive::Gears, Cooling::Liquid},   // L5P (pushrod; gear drive chosen)
            {29, CamLayout::OHV, CamDrive::Gears, Cooling::Air},      // Hatz single cam system (gears chosen)
            {31, CamLayout::DOHC, CamDrive::Chain, Cooling::Air},     // H1 triple: air-cooled (no camshafts - a two-stroke)
            {33, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},   // 2JZ-GTE: a toothed belt to both cams
            {34, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},   // RB26DETT
            {35, CamLayout::DOHC, CamDrive::Belt, Cooling::Liquid},   // VG30DETT
        };
        for (const Build& b : kBuild)
            if (b.preset == idx) { p.camLayout = b.cam; p.camDrive = b.drive; p.cooling = b.cool; }
    }
    finish(p);
    applyVehicleDefaults(p);
    if (idx == 0) {   // K20A: close-ratio six-speed in a ~1.2 t hatch / coupe
        static const float r[6] = {3.267f, 2.130f, 1.517f, 1.147f, 0.921f, 0.738f};
        p.gears = 6; for (int i = 0; i < kMaxGears; ++i) p.gearRatio[i] = i < 6 ? r[i] : 0.738f;
        p.finalDrive = 4.764f; p.mass_kg = 1250; p.tireRadius_mm = 310;
    }
    applyComponentDefaults(p);
    if (idx == 0) p.comp.plugHeatRange = 6;   // K20A street plug: hot enough to stay clean at idle
    // a nitrous kit sized to the engine (~30 hp per litre, in 25 hp jet steps)
    p.n2oShotHp = (float)(std::round(std::max(50.0, 30.0 * equivalentDisplacementL(p)) / 25.0) * 25.0);
    // Idle retune after Group A4 (burn duration from the laminar burning velocity): the idle charges now burn faster
    // than the old empirical factors made them (CA50 ~20-29 deg ATDC instead of 36-44), so the high-overlap engines
    // fired harder between their dilution misfires and idled rougher. Like a calibrator re-tuning for it: a little
    // higher idle speed (more manifold pressure, less residual) and / or less idle advance (`retard`: base advance
    // down, the same added to the mechanical advance - full-load timing at high rpm unchanged). Chosen from an idle
    // grid (idle +0/100/200 rpm x retard 0/4/8 deg) as the smallest change that idles at least as smoothly as before.
    {
        struct Retune { int preset; float idleAdd, retard; };
        static const Retune kRetune[] = {
            {1, 0, 4}, {8, 0, 4}, {5, 100, 4}, {6, 100, 8}, {7, 100, 8}, {22, 100, 8}, {19, 100, 8}, {17, 200, 0},
            {13, 200, 8}, {16, 100, 0},
        };
        for (const Retune& r : kRetune)
            if (r.preset == idx) { p.idleTargetRpm += r.idleAdd; p.baseAdvance -= r.retard; p.mechAdvance += r.retard; }
    }
    generateCalibration(p);
    if (idx == 0) {   // free-revving, small ports: the revs and the manifold react fast - less acceleration fuel,
                      // a little more idle valve damping
        for (int i = 0; i < p.cal.accel.n; ++i) p.cal.accel.v[i] = std::round(p.cal.accel.v[i] * 0.6f);
        p.cal.idleKd = 0.0002f;
    }
    if (idx == 20) {   // the supercharger's charge piping adds a lot of intake volume: the idle loop is slower, softer gains
        p.cal.idleKp *= 0.6f; p.cal.idleKi *= 0.6f; p.cal.idleKd *= 0.6f; p.cal.idleSparkGain *= 0.8f;
    }
    std::snprintf(p.name, sizeof(p.name), "%s", presetName(idx));
    return p;
}

// ======================================================================= preset groups (the engine picker)
static const char* kGroupNames[] = {"Petrol - inline and single", "Petrol - V and boxer", "Turbo and supercharged", "Rotary",
                                    "Diesel", "Two-stroke"};
static const signed char kPresetGroup[] = {
    0, 0, 0, 1, 1, 1, 1, 1, 1, 1,   //  0-9: I4 K20, I3, I6, V6, V8 carb, V8 flat, V10, V12, flat-4, V-twin
    0, 0, 3, 3, 3, 3, 2, 2, 3, 2,   // 10-19: single, I4 ITB, rotaries 12-15, turbo I4, Roots V8, turbo rotary, turbo I6
    2, 0, 1, 3, 4, 5, 4, 4, 4, 4,   // 20-29: centrifugal V6, I5, flat-6, bridge port, TDI, 2T single, diesels 26-29
    5, 5, 5,                        // 30-32: two-strokes
    2, 2, 2,                        // 33-35: twin turbos (2JZ, RB26, VG30)
};
static_assert(sizeof(kPresetGroup) == sizeof(kNames) / sizeof(kNames[0]), "every preset needs a group");
int presetGroupCount() { return (int)(sizeof(kGroupNames) / sizeof(kGroupNames[0])); }
const char* presetGroupName(int g) { return kGroupNames[clampv(g, 0, presetGroupCount() - 1)]; }
int presetGroup(int i) { return kPresetGroup[clampv(i, 0, presetCount() - 1)]; }

// ======================================================================= an engine from scratch
// Rules of thumb, chosen (labelled): valve, port, runner, throttle and header sizes follow the bore and the
// displacement in the proportions of the presets' real engines (K20A, small-block V8, EA189, RD350, 13B); the rev limit
// follows a mean piston speed per character. Diesel, two-stroke and rotary start from one of their presets (whose
// calibration points are researched) scaled to the size asked for.

int recipeCylinderChoices(BuildKind k, Layout l, int* out, int maxOut) {
    static const int kPetI[] = {1, 2, 3, 4, 5, 6, 8}, kPetV[] = {2, 4, 6, 8, 10, 12}, kPetB[] = {2, 4, 6};
    static const int kDieI[] = {1, 2, 3, 4, 5, 6}, kDieV[] = {6, 8, 10, 12}, kDieB[] = {4};
    static const int kTwo[] = {1, 2, 3, 4}, kRot[] = {1, 2, 3, 4};
    const int* s = kPetI; int n = 7;
    switch (k) {
    case BuildKind::Petrol: if (l == Layout::V) { s = kPetV; n = 6; } else if (l == Layout::Boxer) { s = kPetB; n = 3; } break;
    case BuildKind::Diesel:
        if (l == Layout::V) { s = kDieV; n = 4; } else if (l == Layout::Boxer) { s = kDieB; n = 1; } else { s = kDieI; n = 6; }
        break;
    case BuildKind::TwoStroke: s = kTwo; n = 4; break;
    case BuildKind::Rotary: s = kRot; n = 4; break;
    }
    n = std::min(n, maxOut);
    for (int i = 0; i < n; ++i) out[i] = s[i];
    return n;
}

void recipeDisplacementRange(BuildKind k, int cylinders, float& lo, float& hi) {
    // per cylinder (per rotor), L
    float a = 0.1f, b = 1.2f;
    if (k == BuildKind::Diesel) { a = 0.3f; b = 2.0f; }
    else if (k == BuildKind::TwoStroke) { a = 0.05f; b = 0.5f; }
    else if (k == BuildKind::Rotary) { a = 0.3f; b = 1.0f; }
    lo = a * std::max(1, cylinders); hi = b * std::max(1, cylinders);
}

EngineRecipe defaultRecipe(BuildKind k) {
    EngineRecipe r;
    r.kind = k;
    switch (k) {
    case BuildKind::Petrol: break;
    case BuildKind::Diesel:
        r.cylinders = 4; r.displacement_L = 2.0f; r.boreStroke = 0.85f; r.cam = CamLayout::DOHC; r.drive = CamDrive::Belt; r.aspiration = 1;
        break;
    case BuildKind::TwoStroke:
        r.cylinders = 1; r.displacement_L = 0.25f; r.boreStroke = 0.92f; r.valves = 0; r.character = 1;
        break;
    case BuildKind::Rotary:
        r.cylinders = 2; r.displacement_L = 1.3f; r.boreStroke = 1.0f; r.valves = 0;
        break;
    }
    return r;
}

void recipeAutoName(const EngineRecipe& r, char* out, int outSize) {
    const int n = r.cylinders;
    char head[24], size[24];
    if (r.kind == BuildKind::Rotary) std::snprintf(head, sizeof(head), "%d-rotor", n);
    else if (r.kind == BuildKind::TwoStroke) std::snprintf(head, sizeof(head), "%d-cyl", n);
    else if (n == 1) std::snprintf(head, sizeof(head), "Single");
    else if (r.layout == Layout::V) std::snprintf(head, sizeof(head), n == 2 ? "V-twin" : "V%d", n);
    else if (r.layout == Layout::Boxer) std::snprintf(head, sizeof(head), "Flat-%d", n);
    else std::snprintf(head, sizeof(head), "I%d", n);
    if (r.displacement_L < 1.0f || r.kind == BuildKind::TwoStroke) std::snprintf(size, sizeof(size), "%.0f cc", r.displacement_L * 1000.0f);
    else std::snprintf(size, sizeof(size), "%.1f", r.displacement_L);
    static const char* kAsp[] = {"", " turbo", " Roots", " twin-screw", " centrifugal"};
    const char* asp = r.kind == BuildKind::TwoStroke ? "" : kAsp[clampv(r.aspiration, 0, 4)];
    const char* tail = "";
    switch (r.kind) {
    case BuildKind::Petrol: tail = r.fuel == 1 ? " ITB" : r.fuel == 2 ? " carb" : ""; break;
    case BuildKind::Diesel: tail = " diesel"; break;
    case BuildKind::TwoStroke: tail = r.fuel == 1 ? " two-stroke EFI" : " two-stroke"; break;
    case BuildKind::Rotary: {
        static const char* kPort[] = {"", " street port", " bridge port", " peripheral port"};
        tail = kPort[clampv(r.character, 0, 3)];
        break;
    }
    }
    std::snprintf(out, outSize, "%s %s%s%s", head, size, asp, tail);
}

// combinations the model does not run reliably yet are swapped for the nearest one that does (the dialog shows them
// greyed out): a bridge port only on two rotors (its idle - the tiny shared plenum that keeps the rotors in step - was
// tuned on the 13B: on three rotors it idles at the end of the idle valve's travel and floods with boost), two-stroke
// injection only on a single (as the KTM TPI: injected multi-cylinder two-strokes stall at a street idle)
bool recipeAllowed(const EngineRecipe& r, int what, int value) {
    if (what == 0) return !(r.kind == BuildKind::Rotary && value == 2 && r.cylinders != 2);   // porting
    if (what == 1) return !(r.kind == BuildKind::TwoStroke && value == 1 && r.cylinders > 1);  // fuel
    return true;
}
void recipeLimit(EngineRecipe& r) {
    if (!recipeAllowed(r, 0, r.character)) r.character = 1;
    if (!recipeAllowed(r, 1, r.fuel)) r.fuel = 0;
}

static void sizeBoreStroke(EngineParams& p, double totalL, int n, double boreStroke) {
    const double v = totalL / std::max(1, n) * 1e6;   // mm3 per cylinder
    const double b = std::cbrt(4.0 * v * boreStroke / kPi);
    p.bore_mm = (float)(std::round(b * 10.0) / 10.0);
    p.stroke_mm = (float)(std::round(b / boreStroke * 10.0) / 10.0);
}
static float round1(double x) { return (float)(std::round(x * 10.0) / 10.0); }
static float roundTo(double x, double step) { return (float)(std::round(x / step) * step); }
// the usual vee angle for the count (chosen: the common production angles)
static float veeAngle(int n) { return n == 6 || n == 12 ? 60.0f : n == 10 ? 72.0f : 90.0f; }
static void forcedInduction(EngineParams& p, int aspiration) {
    if (aspiration == 1) applyInductionDefaults(p, Induction::Turbo);
    else if (aspiration >= 2)
        applyInductionDefaults(p, Induction::Supercharger, aspiration == 2 ? Supercharger::Roots : aspiration == 3 ? Supercharger::TwinScrew : Supercharger::Centrifugal);
}

static EngineParams buildPetrol(const EngineRecipe& r, int chr) {
    EngineParams p = base();
    const int n = r.cylinders;
    const double D = r.displacement_L, Vc = D / n;
    p.cylinders = n;
    p.layout = n == 1 ? Layout::Inline : r.layout;
    if (p.layout == Layout::V) p.bankAngle = veeAngle(n);
    applyLayoutDefaults(p);
    sizeBoreStroke(p, D, n, clampv((double)r.boreStroke, 0.7, 1.5));
    const double B = p.bore_mm, S = p.stroke_mm;
    p.rod_mm = (float)std::round(S * (chr == 2 ? 1.75 : 1.65));   // rod / stroke ratio, chosen (1.5-1.8 is usual)
    // valves: 4 per cylinder ~0.39 / 0.33 x bore (K20A 35 / 30 on 86), 2 per cylinder ~0.48 / 0.39 (small-block 49 / 39 on
    // 101.6)
    const bool four = r.valves >= 4;
    p.intakeValves = p.exhaustValves = four ? 2 : 1;
    p.intakeValveDia_mm = round1(B * (four ? 0.39 : 0.48));
    p.exhaustValveDia_mm = round1(B * (four ? 0.33 : 0.39));
    // cams per character (chosen: a road cam, a fast-road cam, a race cam), lift as a share of the valve size
    static const float kDurIn[3] = {232, 256, 282}, kDurEx[3] = {236, 260, 286}, kClIn[3] = {112, 108, 104}, kClEx[3] = {114, 112, 108};
    static const float kLift[3] = {1.0f, 1.08f, 1.18f};
    p.intakeDuration = kDurIn[chr]; p.exhaustDuration = kDurEx[chr];
    p.intakeCenterline = kClIn[chr]; p.exhaustCenterline = kClEx[chr];
    p.intakeLift_mm = round1(p.intakeValveDia_mm * (four ? 0.28 : 0.24) * kLift[chr]);
    p.exhaustLift_mm = round1(p.exhaustValveDia_mm * (four ? 0.31 : 0.30) * kLift[chr]);
    p.camLayout = r.cam; p.camDrive = r.drive;
    static const float kCr[3] = {10.5f, 11.3f, 12.5f};
    p.compression = kCr[chr] - (r.fuel == 2 ? 0.8f : 0.0f);
    // rev limit from the mean piston speed (m/s; chosen: road 17.5, sporty 20.5, race 23.5)
    static const float kCm[3] = {17.5f, 20.5f, 23.5f};
    p.revLimit = clampv(roundTo(kCm[chr] * 30000.0 / S, 100.0), 4000.0f, 12500.0f);
    p.limiter = chr == 2 ? Limiter::IgnitionCut : Limiter::FuelCut;
    p.baseAdvance = 12; p.mechAdvance = chr == 2 ? 16.0f : 18.0f; p.mechAdvanceRpm = roundTo(0.55 * p.revLimit, 100.0); p.vacuumAdvance = 10;
    static const float kIdle[3] = {800, 900, 1100};
    p.idleTargetRpm = kIdle[chr] + (n <= 2 ? 150.0f : 0.0f);
    static const float kFly[3] = {0.06f, 0.05f, 0.035f};   // kg m2 per litre (K20A 0.12 at 2.0, small-block 0.30 at 5.7)
    p.flywheel = clampv((float)(0.02 + D * kFly[chr]), 0.03f, 0.4f);
    // intake
    static const float kRunner[3] = {420, 330, 260}, kFilter[3] = {0.4f, 0.3f, 0.1f}, kInj[3] = {560, 620, 700};
    p.runnerLen_mm = kRunner[chr];
    p.runnerDia_mm = (float)std::round(p.intakeValveDia_mm * (four ? 1.1 : 0.98));
    p.airFilter = kFilter[chr];
    p.injector_ccmin = clampv(roundTo(Vc * kInj[chr], 10.0), 100.0f, 2500.0f);   // cc/min per litre of cylinder (K20A 310)
    if (r.fuel == 1) {        // individual throttle bodies, ~0.55 x bore (I4 ITB race: 45 on 81)
        p.itb = 1; p.throttleBore_mm = clampv((float)std::round(0.55 * B), 28.0f, 60.0f);
        p.runnerDia_mm = (float)std::round(p.runnerDia_mm * 1.15); p.airFilter = std::min(p.airFilter, 0.15f);
    } else if (r.fuel == 2) { // carburettor: venturi area from the small-block's 4 x 32 mm at 5.7 L / 6200 rpm; with
                              // fewer cylinders the carb sees the pulsed flow of one or two of them and is larger for
                              // its size ((4 / n)^0.8, chosen: the 0.5 L thumper's 36 mm)
        p.fuelSystem = FuelSystem::Carburetor;
        p.carbBarrels = n >= 6 ? 4 : n >= 2 ? 2 : 1;
        const double area = 0.091 * std::pow(std::max(1.0, 4.0 / n), 0.8) * D * p.revLimit;
        p.venturi_mm = clampv((float)std::round(std::sqrt(4.0 * area / (kPi * p.carbBarrels))), 18.0f, 46.0f);
        p.throttleBore_mm = (float)std::round(p.venturi_mm * 1.35);
        p.plenum_L = clampv((float)(0.7 * D), 0.3f, 6.0f);
        p.idleScrew = n <= 2 ? 4.0f : 3.5f;
        if (n <= 2) p.accelPump = 1.3f;   // (the thumper's)
    } else {                  // one throttle body (K20A 62 at 2.0 L, I6 72 at 3.0)
        p.throttleBore_mm = clampv((float)std::round(46.0 * std::sqrt(D)), 30.0f, 110.0f);
        p.plenum_L = clampv((float)(1.3 * D), 0.4f, 8.0f);
    }
    // exhaust: primaries ~1.27 x the exhaust valve (4 valves), ~1.13 (2 valves); one pipe per bank
    static const float kPrim[3] = {550, 700, 800};
    p.primaryDia_mm = (float)std::round(p.exhaustValveDia_mm * (four ? 1.27 : 1.13));
    for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = kPrim[chr];
    const int banks = p.layout == Layout::Inline ? 1 : 2, perBank = std::max(1, n / banks);
    p.header = p.layout == Layout::Inline && n == 4 && chr >= 1 ? HeaderType::TriY : HeaderType::Collector;
    p.collectorDia_mm = std::max(p.primaryDia_mm, (float)std::round(p.primaryDia_mm * std::sqrt((double)perBank) * 0.72));
    p.pipeDia_mm = clampv((float)std::round(40.0 * std::sqrt(D / banks)), 38.0f, 90.0f);
    if (banks == 2) p.crossover = chr == 0 ? Crossover::HPipe : chr == 1 ? Crossover::XPipe : Crossover::TrueDual;
    p.cat = chr < 2; p.resonator = chr == 0;
    p.muffler = chr == 0 ? Muffler::Chambered : Muffler::StraightThrough;
    forcedInduction(p, r.aspiration);
    if (isBoosted(p)) {
        p.mechAdvance = std::min(p.mechAdvance, 12.0f);   // (the boosted presets run 11-12)
        if (r.aspiration == 1 && chr == 0) p.muffler = Muffler::Turbo;
    }
    p.sweepStartRpm = 1500; p.sweepEndRpm = p.revLimit - 100;
    return p;
}

// diesel: a turbo one is the EA189 2.0 TDI (preset 26) scaled - its common rail, boost and EGR set points per litre;
// a naturally aspirated one the Hatz 1B40 (preset 29: injection pump, no turbo, EGR or glow plugs)
static EngineParams buildDiesel(const EngineRecipe& r) {
    EngineParams p = makePreset(r.aspiration ? 26 : 29);
    const bool four0 = p.intakeValves >= 2;
    const int n0 = p.cylinders;
    const double B0 = p.bore_mm, S0 = p.stroke_mm, D0 = ratedDisplacementL(p), Vc0 = D0 / n0;
    const int n = r.cylinders;
    const double D = r.displacement_L, Vc = D / n, kv = Vc / Vc0, kd = D / D0;
    p.cylinders = n;
    p.layout = n == 1 ? Layout::Inline : r.layout;
    if (p.layout == Layout::V) p.bankAngle = veeAngle(n);
    applyLayoutDefaults(p);
    sizeBoreStroke(p, D, n, clampv((double)r.boreStroke, 0.7, 1.2));
    const double B = p.bore_mm, S = p.stroke_mm;
    p.rod_mm = (float)std::round(S * 1.55);
    const bool four = r.valves >= 4;
    p.intakeValves = p.exhaustValves = four ? 2 : 1;
    const double kValve = four == four0 ? 1.0 : four ? 1.0 / 1.26 : 1.26;   // (one valve ~1.26 x the size of a pair's)
    p.intakeValveDia_mm = round1(p.intakeValveDia_mm * B / B0 * kValve);
    p.exhaustValveDia_mm = round1(p.exhaustValveDia_mm * B / B0 * kValve);
    p.camLayout = r.cam; p.camDrive = r.drive;
    p.revLimit = clampv(roundTo(p.revLimit * S0 / S, 100.0), 2400.0f, 5500.0f);   // the same piston speed
    p.dieselMaxFuel = (float)(p.dieselMaxFuel * kv); p.dieselInjRate = (float)(p.dieselInjRate * kv);
    p.nozzleHoleDia_mm = clampv((float)(p.nozzleHoleDia_mm * std::sqrt(kv)), 0.08f, 0.25f);
    p.flywheel = clampv((float)(p.flywheel * std::pow(kd, 0.8)), 0.06f, 1.2f);
    p.compFlow_gs = (float)(p.compFlow_gs * kd); p.chargeVolume_L = (float)(p.chargeVolume_L * kd);
    p.turboInertia = clampv((float)(p.turboInertia * kd), 0.03f, 0.6f);
    p.egrValveDia_mm = (float)std::round(p.egrValveDia_mm * std::sqrt(kd));
    p.plenum_L = (float)(p.plenum_L * kd); p.throttleBore_mm = (float)std::round(p.throttleBore_mm * std::sqrt(kd));
    p.runnerDia_mm = (float)std::round(p.runnerDia_mm * B / B0);
    p.injector_ccmin = (float)(p.injector_ccmin * kv);
    const int banks = p.layout == Layout::Inline ? 1 : 2;
    p.primaryDia_mm = (float)std::round(p.primaryDia_mm * std::sqrt(kv));
    p.collectorDia_mm = (float)std::round(p.collectorDia_mm * std::sqrt(kd / banks));
    p.pipeDia_mm = clampv((float)std::round(p.pipeDia_mm * std::sqrt(kd / banks)), 38.0f, 110.0f);
    if (n > 2) p.idleTargetRpm = std::min(p.idleTargetRpm, 800.0f);   // (the Hatz single idles at 1200)
    p.sweepStartRpm = 1000; p.sweepEndRpm = p.revLimit - 100;
    return p;
}

// two-stroke: the RD350 twin (preset 30, reed valves, power valve, expansion chambers) scaled; the pipe tuned for a
// mean piston speed per character
static EngineParams buildTwoStroke(const EngineRecipe& r, int chr) {
    EngineParams p = makePreset(30);
    const int n = r.cylinders;
    const double D = r.displacement_L, cc = D / n * 1000.0;
    p.cylinders = n; p.layout = Layout::Inline;
    applyLayoutDefaults(p);
    sizeBoreStroke(p, D, n, clampv((double)r.boreStroke, 0.8, 1.4));
    const double S = p.stroke_mm;
    p.rod_mm = (float)std::round(S * 1.9);   // (RD350 105 / 54)
    // port timing per character (deg ATDC the port opens; Blair Fig. 1.9: road ~95 / 120, racing ~80 / 115)
    static const float kExh[3] = {92, 84, 80}, kTr[3] = {120, 117, 114}, kCm[3] = {14.5f, 17.0f, 19.5f}, kCr[3] = {6.8f, 7.4f, 8.2f};
    p.tsExhaustOpen = kExh[chr]; p.tsTransferOpen = kTr[chr];
    p.tsPipeRpm = clampv(roundTo(kCm[chr] * 30000.0 / S, 100.0), 4000.0f, 14000.0f);
    p.revLimit = clampv(roundTo(p.tsPipeRpm * 1.15, 100.0), 5000.0f, 15000.0f);
    p.tsPowerValve = 1; p.tsPvRpmLow = roundTo(0.72 * p.tsPipeRpm, 100.0); p.tsPvRpmHigh = roundTo(0.95 * p.tsPipeRpm, 100.0);
    p.tsReedArea_cm2 = (float)std::round(0.48 * std::pow(cc, 2.0 / 3.0));   // (RD350 14 at 173 cc, CR250 20 at 250)
    p.compression = trappedToGeometricCr(p, kCr[chr]);
    p.venturi_mm = clampv((float)std::round(2.1 * std::sqrt(cc)), 16.0f, 44.0f);   // (RD350 26 at 173 cc)
    p.throttleBore_mm = p.venturi_mm; p.runnerDia_mm = p.venturi_mm + 4.0f;
    static const float kIdle[3] = {1250, 1400, 1600}, kScrew[3] = {1.0f, 1.4f, 1.8f};
    p.idleTargetRpm = kIdle[chr];
    p.idleScrew = kScrew[chr];   // (more idle air for the longer port timings: a racing 125 stalls on the road setting)
    p.flywheel = clampv((float)(0.01 + 0.06 * D), 0.015f, 0.12f);   // (RD350 0.03, CR250 0.02)
    if (r.fuel == 1) {   // injection (KTM TPI-like): oil from a pump, not in the fuel
        p.fuelSystem = FuelSystem::EFI; p.injection = Injection::Port;
        p.injector_ccmin = clampv(roundTo(0.82 * cc, 10.0), 80.0f, 600.0f);   // (TPI 240 at 293 cc)
        p.tsOilRatio = 0; p.plenum_L = 0.3f;
    }
    p.sweepStartRpm = 2500; p.sweepEndRpm = p.revLimit - 200;
    return p;
}

// rotary: the 13B family (presets 12 stock / 13 street port / 23 bridge port / 14 peripheral port) scaled - every
// rotor dimension by the same factor (the shape, K = R / e, kept)
static EngineParams buildRotary(const EngineRecipe& r, int porting) {
    static const int kTpl[4] = {12, 13, 23, 14};
    EngineParams p = makePreset(kTpl[clampv(porting, 0, 3)]);
    auto chamberL = [](const EngineParams& q) { return 3.0 * std::sqrt(3.0) * q.rotorRadius_mm * q.eccentricity_mm * q.rotorWidth_mm * 1e-6; };
    const double vf0 = chamberL(p), D0 = vf0 * p.cylinders;
    const int n = r.cylinders;
    const double D = r.displacement_L, vf = D / n, k = std::cbrt(vf / vf0), kd = D / D0;
    p.cylinders = n;
    applyLayoutDefaults(p);
    p.rotorRadius_mm = round1(p.rotorRadius_mm * k); p.eccentricity_mm = round1(p.eccentricity_mm * k); p.rotorWidth_mm = round1(p.rotorWidth_mm * k);
    p.rIntakeArea_cm2 = round1(p.rIntakeArea_cm2 * k * k); p.rExhaustArea_cm2 = round1(p.rExhaustArea_cm2 * k * k);
    p.throttleBore_mm = (float)std::round(p.throttleBore_mm * (p.itb ? k : std::sqrt(kd)));
    p.plenum_L = std::max(0.3f, (float)(p.plenum_L * kd));
    p.runnerDia_mm = (float)std::round(p.runnerDia_mm * k);
    p.injector_ccmin = (float)(p.injector_ccmin * vf / vf0);
    p.primaryDia_mm = (float)std::round(p.primaryDia_mm * k);
    p.collectorDia_mm = (float)std::round(p.collectorDia_mm * std::sqrt(kd));
    p.pipeDia_mm = clampv((float)std::round(p.pipeDia_mm * std::sqrt(kd)), 40.0f, 100.0f);
    p.flywheel = clampv((float)(p.flywheel * std::pow(kd, 0.8)), 0.06f, 0.5f);
    p.revLimit = clampv(roundTo(p.revLimit / k, 100.0), 6000.0f, 11000.0f);   // the same apex seal speed
    forcedInduction(p, r.aspiration);
    p.sweepEndRpm = p.revLimit - 200;
    return p;
}

EngineParams buildEngine(const EngineRecipe& in) {
    EngineRecipe r = in;
    {   // a cylinder count the kind / layout is built with (the nearest), a displacement in its range
        int ch[16];
        const int nch = recipeCylinderChoices(r.kind, r.layout, ch, 16);
        int best = ch[0];
        for (int i = 0; i < nch; ++i) if (std::abs(ch[i] - r.cylinders) < std::abs(best - r.cylinders)) best = ch[i];
        r.cylinders = best;
        float lo, hi;
        recipeDisplacementRange(r.kind, r.cylinders, lo, hi);
        r.displacement_L = clampv(r.displacement_L, lo, hi);
    }
    recipeLimit(r);
    const int chr = clampv(r.character, 0, r.kind == BuildKind::Rotary ? 3 : 2);
    EngineParams p;
    switch (r.kind) {
    default:
    case BuildKind::Petrol: p = buildPetrol(r, chr); break;
    case BuildKind::Diesel: p = buildDiesel(r); break;
    case BuildKind::TwoStroke: p = buildTwoStroke(r, chr); break;
    case BuildKind::Rotary: p = buildRotary(r, chr); break;
    }
    if (r.kind != BuildKind::Rotary) p.cooling = r.cooling;
    // finished like a preset: firing phases, a car to drive, components, the base calibration
    finish(p);
    applyVehicleDefaults(p);
    applyComponentDefaults(p);
    p.n2oShotHp = (float)(std::round(std::max(50.0, 30.0 * equivalentDisplacementL(p)) / 25.0) * 25.0);
    generateCalibration(p);
    if (r.name[0]) std::snprintf(p.name, sizeof(p.name), "%s", r.name);
    else recipeAutoName(r, p.name, (int)sizeof(p.name));
    return p;
}

} // namespace es
