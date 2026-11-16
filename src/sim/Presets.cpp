#include "sim/EngineParams.h"
#include "core/MathUtil.h"
#include <cstdio>

namespace es {

void computePhases(const EngineParams& p, float out[kMaxCyl]) {
    int n = clampv(p.cylinders, 1, kMaxCyl);
    // rotary: every rotor fires once per shaft revolution -> rotors spaced 360/N apart
    const double spacing = isRotary(p) ? 360.0 : 720.0;
    if (p.evenFire || isRotary(p)) {
        for (int k = 0; k < n; ++k) {
            int c = clampv(p.firingOrder[k], 0, n - 1);
            out[c] = (float)(spacing * k / n);
        }
    } else {
        for (int i = 0; i < n; ++i) out[i] = (float)wrapPos(p.phase[i], 720.0);
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

    p.header = HeaderType::Collector; p.primaryDia_mm = 38; p.secondaryLen_mm = 400;
    p.collectorLen_mm = 300; p.collectorDia_mm = 55; p.crossover = Crossover::TrueDual;
    p.pipeDia_mm = 55; p.midpipeLen_mm = 1500; p.cat = 1; p.resonator = 1;
    p.muffler = Muffler::Chambered; p.tailpipeLen_mm = 600;

    p.throttle = 0; p.ignition = 0; p.starter = 0; p.ambientTemp_C = 25;
    p.listener = ListenerPos::Tailpipe; p.volume = 1.0f; p.oversample = 2;
    p.dynoMode = DynoMode::Off; p.dynoHoldRpm = 3000;
    p.sweepStartRpm = 1500; p.sweepEndRpm = 7000; p.sweepRate = 600;

    p.engineType = EngineType::Piston;
    p.rotorRadius_mm = 105; p.eccentricity_mm = 15; p.rotorWidth_mm = 80;
    applyRotaryPorting(p, 0);
    p.apexLeak = 0.05f;

    p.induction = Induction::Natural; p.scType = Supercharger::Roots;
    p.compFlow_gs = 220; p.turbineAR = 0.64f; p.turboInertia = 0.15f; p.wastegateBar = 1.0f;
    p.intercooler = 1; p.icEfficiency = 0.75f; p.chargeVolume_L = 3.0f; p.bov = 1; p.antiLag = 0;
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
    "Rotary 2-rotor 1.3L turbo (13B-REW-style)",
    "I6 3.0 big single turbo (drag)",
    "V6 3.5 centrifugal supercharger",
    "I5 2.5 EFI (five cylinder)",
    "Flat-6 3.6 EFI (boxer six)",
    "Rotary 2-rotor 1.3L bridge port (brap idle)",
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
        p.idleScrew = 4.0f; p.accelPump = 1.8f; p.primaryLen_mm[0] = 900; p.primaryDia_mm = 42; p.collectorDia_mm = 45; p.pipeDia_mm = 45;
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
        p.injector_ccmin = 380; p.idleTargetRpm = 1200; p.mechAdvanceRpm = 6000; p.mechAdvance = 24;
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
    case 18: // turbo rotary
        p = makePreset(12);
        applyInductionDefaults(p, Induction::Turbo);
        p.compFlow_gs = 300; p.turbineAR = 0.7f; p.turboInertia = 0.14f; p.wastegateBar = 0.9f;
        p.compression = 9.0f; p.injector_ccmin = 950; p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough;
        p.pipeDia_mm = 76; p.revLimit = 8200; p.sweepEndRpm = 8000;
        p.idleTargetRpm = 1100;   // big injectors + low compression: a little more idle margin
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
        p.throttleBore_mm = 65; p.plenum_L = 3.0f; p.revLimit = 9000;
        p.header = HeaderType::Collector; p.primaryDia_mm = 55; p.collectorDia_mm = 65; p.pipeDia_mm = 65;
        for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = 350;
        p.cat = 0; p.resonator = 0; p.muffler = Muffler::StraightThrough;
        p.limiter = Limiter::IgnitionCut;
        p.sweepStartRpm = 2000; p.sweepEndRpm = p.revLimit - 200;
        break;
    case 20: // centrifugal supercharged V6
        p = makePreset(3);
        applyInductionDefaults(p, Induction::Supercharger, Supercharger::Centrifugal);
        p.compFlow_gs = 380; p.scRatio = 12.5f; p.compression = 9.5f; p.injector_ccmin = 550; p.mechAdvance = 11;
        p.revLimit = 7000; p.muffler = Muffler::StraightThrough; p.sweepEndRpm = 6900;
        break;
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

} // namespace es
