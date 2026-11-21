#pragma once
#include "sim/Cylinder.h"
#include "sim/Exhaust.h"
#include "sim/Intake.h"
#include "sim/Boost.h"
#include "sim/Components.h"
#include "sim/Thermal.h"
#include "sim/Valvetrain.h"
#include "sim/Fuel.h"
#include "sim/FlowStats.h"
#include "ecu/Sensors.h"
#include "ecu/Ecu.h"
#include "audio/Filters.h"
#include "sim/CombustionNoise.h"
#include <vector>

namespace es {

// The complete engine: crank + cylinders + intake + exhaust. Stepped at the simulation rate.
class Engine {
public:
    Engine(const EngineParams& p, double fs);

    void apply(const EngineParams& p);                  // live (non structural) changes
    // (a rebuild while running / a motoring rig: the ECU keeps its trigger sync)
    void setCrank(double omega, double thetaDeg) {
        omega_ = omega; theta_ = thetaDeg;
        if (omega > 2.0) { syncArmed_ = crankSync_ = camSync_ = true; ckpRpm_ = omega * 60.0 / (2.0 * kPi); }
    }

    // pedal 0..1, loadTorque = absorber (dyno) torque opposing rotation
    // signedLoad: the load can also drive the crank (clutch to a rolling car)
    void step(double dt, double pedal, double loadTorque, bool signedLoad = false);

    // ---- state / outputs
    double omega() const { return omega_; }
    double rpm() const { return omega_ * 60.0 / (2.0 * kPi); }
    double theta() const { return theta_; }
    double netTorque() const { return torqueAvg_; }      // indicated - friction, cycle averaged (N m)
    double instTorque() const { return torqueInst_; }
    double inertia() const { return totalInertia(); }
    double totalInertia() const {
        return std::max(0.01, (double)p_.flywheel) + (geom_.rotary ? 0.002 * nPorts_ : comp_.extraInertia);
    }
    double displacement() const { return geom_.Vd * nPorts_; }
    double cycle() const { return cycle_; }          // 720 piston, 1080 rotary
    bool rotary() const { return geom_.rotary; }
    bool twoStroke() const { return twoStroke_; }
    // two-stroke crankcase of a cylinder (pressure Pa, temperature K, reed petal opening 0..1)
    double crankcaseP(int i) const { return twoStroke_ && i < (int)cc_.size() ? cc_[i].g.p() : kPatm; }
    double crankcaseT(int i) const { return twoStroke_ && i < (int)cc_.size() ? cc_[i].g.T : ambT_; }
    double reedOpen(int i) const { return twoStroke_ && i < (int)cc_.size() ? cc_[i].reed : 0.0; }
    const GasNode* crankcaseGas(int i) const { return twoStroke_ && i < (int)cc_.size() ? &cc_[i].g : nullptr; }
    int ports() const { return nPorts_; }            // cylinders or rotors
    bool running() const { return running_; }
    bool limiterActive() const { return ecu.cmd.limiter; }
    double advance() const { return advance_; }

    // sound (Pa @ 1 m) for the current step
    double sExhL = 0, sExhR = 0, sIntake = 0, sMech = 0, sBoost = 0;

    // telemetry helpers
    const Cylinder& cyl(int i) const { return cyls_[i]; }
    int cylinders() const { return (int)cyls_.size(); }
    IntakeSystem intake;
    ExhaustSystem exhaust;
    ForcedInduction boost;
    bool antiLagActive = false;
    // thermal / electrical / fuel supply physics, sensors and the ECU
    ThermalModel thermal;
    Electrical elec;
    FuelRail rail;
    SensorLayer sensors;
    Ecu ecu;
    // the vehicle (if any) feeds radiator ram air and the speed / gear sensors
    // (+ the driven wheel speed for traction control and the gear-change signal for the shift cut)
    void setVehicleState(double speedMs, int gear, double wheelMs = -1, bool shifting = false) {
        vehSpeed_ = speedMs; vehGear_ = gear; wheelSpeed_ = wheelMs < 0 ? speedMs : wheelMs; shifting_ = shifting;
    }
    // carry thermal / electrical state (and the nitrous bottle) over a rebuild (structural change while running)
    void copyThermalFrom(const Engine& o) {
        thermal = o.thermal; elec = o.elec; rail = o.rail; n2oBottle = o.n2oBottle; n2oRefillSeen_ = o.n2oRefillSeen_;
        egtProbe_ = o.egtProbe_; egtWall_ = o.egtWall_; egtPost_ = o.egtPost_; egtPostWall_ = o.egtPostWall_;
        for (int i = 0; i < kMaxCyl; ++i) { egtCylProbe_[i] = o.egtCylProbe_[i]; egtCylWall_[i] = o.egtCylWall_[i]; }
        tank_ = o.tank_; line_ = o.line_; tankL_ = o.tankL_; fuel_ = o.fuel_; tankProps_ = o.tankProps_;
        refuelSeen_ = o.refuelSeen_; fuelTypeSeen_ = o.fuelTypeSeen_;
        intake.setFuel(fuel_);
    }
    // fuel (Package 1): the tank, and the fuel line that feeds the injectors - a new fuel reaches them gradually
    // (the return line circulates the rail contents back to the tank)
    const FuelProps& fuel() const { return fuel_; }          // what the engine burns now
    double glowTipC() const { return glowC_; }
    double egrPos() const { return egrPos_; }
    double egrFlow() const { return egrFlow_; }
    double egrRate() const { return egrFlow_ / std::max(1e-6, egrFlow_ + egrAir_); }   // share of the charge (averaged)
    double combNoise() const { return combNoise_; }   // diesel combustion noise, Pa (structure-attenuated)
    // its rms over every step since the last call (tests / diagnostics: sampling the signal now and then aliases
    // with the firing - the short bursts are hit or missed depending on the phase)
    double combNoiseRms() { const double r = cnN_ > 0 ? std::sqrt(cnE2_ / cnN_) : 0.0; cnE2_ = 0; cnN_ = 0; return r; }
    const FuelProps& tankFuel() const { return tankProps_; }
    double tankLitres() const { return tankL_; }
    static constexpr double kTankCapL = 50.0;
    // nitrous: what flows now and what is left in the bottle
    double n2oFlow = 0;           // kg/s of N2O
    double n2oBottle = 0;         // kg
    static constexpr int kTrace = 1080;
    float traceP[kTrace] = {}, traceV[kTrace] = {}, traceExh[kTrace] = {}, traceInt[kTrace] = {};
    double lambdaAvg = 1, veAvg = 0, knockMax = 0, egtAvg = 800;   // egtAvg: gas in the manifold (after the port)
    double egtProbe() const { return egtProbe_; }   // K: what an EGT thermocouple in the header reads
    double egtCyl = 800;          // gas leaving the cylinders (before the port heat loss)
    int misfires = 0, afterfires = 0;   // misfires: charges with fuel that did not light (fuel-cut cycles are not counted)
    enum MisCause : int { kMisLean, kMisSpark, kMisFouled, kMisRich, kMisDilution, kMisCauseCount };
    int misfireCause[kMisCauseCount] = {};
    // hardware components: derived model and live state
    const ComponentModel& components() const { return comp_; }
    double stress = 0;            // peak inertia load / limit of the weakest part
    double floatIn = 0, floatEx = 0;  // valve float severity 0..1
    // valvetrain dynamics of the cam profiles now in use (intake / exhaust, base cam; high cam with VTEC)
    const ValveDynamics& valveDyn(bool intake, bool hi = false) const { return intake ? (hi ? vdInHi_ : vdIn_) : (hi ? vdExHi_ : vdEx_); }
    // air blown through the radiator / intercooler (vehicle speed, or the dyno cell fan)
    double coolingAirMs() const { return p_.vehicle ? vehSpeed_ : std::max(vehSpeed_, std::max(0.0, (double)p_.coolingFan_kmh) / 3.6); }
    double sparkMargin = 9;       // lowest available / needed spark energy over the last moments
    double sparkAvail = 0;        // mJ available at the current rpm
    double oilBar = 0;            // oil pressure
    double plugTempC = 450;       // spark plug tip temperature
    double fouling = 0;           // plug fouling 0..1
    double damage = 0;            // accumulated mechanical damage 0..1 (only with failure enabled)
    int failure = 0;              // 0 none, 1 piston, 2 rod, 3 crank, 4 valve-to-piston contact, 5 knock (piston)
    const EngineParams& params() const { return p_; }
    const float* phases() const { return phase_; }
    double vtecEngaged() const { return vtecBlend_; }   // 0 low cam .. 1 high cam
    // trigger decoder (Phase 7)
    bool crankSynced() const { return crankSync_; }
    bool camSynced() const { return camSync_; }
    double lastSyncDeg = 0;       // crank degrees from the first movement to crank sync (last start)
    double afterfireKw() const { return afPower_ * 1e-3; }      // heat of fuel burning in the exhaust
    double exhFlowGs() const { return exhFlowAvg_ * 1e3; }      // exhaust mass flow (averaged)
    double vtcAdvance() const { return vtcNow_; }       // intake cam advance now, crank deg

    // exhaust sensors (Package 2)
    double lambdaPipe() const { return lamPipe_; }                 // gas at the wideband (before its transport delay)
    double emapKPa() const { return emapKPa_; }
    double egtPostProbe() const { return egtPost_; }               // K, thermocouple after the turbine
    double egtCylProbe(int port) const { return egtCylProbe_[clampv(port, 0, kMaxCyl - 1)]; }   // K
    static constexpr double kHcSensed = 0.6;   // unburned fuel as a wideband counts it (slow diffusion)

    // flow map (Package 3): forward / reverse flow per section of the gas path, pressures and temperatures
    const FlowState& flowState() const { return flow_; }


private:
    FlowState flow_;
    double flInF_ = 0, flInB_ = 0, flExF_ = 0, flExB_ = 0;   // valve flows this step (kg)
    double flTailNet_ = 0;
    void flowStep(double dt);
    void closeKnockWindow(Cylinder& c);
    double wbAirKg_ = 0, wbFuelKg_ = 0, wbA_ = 0, wbF_ = 0, lamPipe_ = 1, emapKPa_ = 101.3;
    double egtPost_ = 300, egtPostWall_ = 300, egtCylProbe_[kMaxCyl] = {}, egtCylWall_[kMaxCyl] = {}, egtSub_ = 0;
    FuelBlend tank_, line_;
    FuelProps fuel_, tankProps_;
    double tankL_ = 30;
    int refuelSeen_ = 0, fuelTypeSeen_ = -1;
    void fuelSetup(bool fresh);
    void fuelStep(double dt, double fuelKgs);
    double exhFlowAvg_ = 0;
    void stepCylinder(int i, double dt, double phiPrev, double phi, double rpm, bool spark, bool fuelCut);
    void updateValves();
    void setupChambers();
    void apexLeakage(double dt);
    // two-stroke (Package 5): the crankcase under each piston - pumped by the piston's underside, filled through the
    // reed valve, emptied through the transfer ports. Energy kept explicitly (U = m cv T).
    struct Crankcase { GasNode g; double U = 0; double reed = 0, reedV = 0; };   // reed: petal tip lift / its stop, and its rate (1/s)
    std::vector<Crankcase> cc_;
    bool twoStroke_ = false;
    double ccVmin_ = 1e-4;        // crankcase volume with the piston at BDC
    void crankcaseInit();
    void crankcaseStep(Crankcase& k, GasNode& ip, double vCyl, double dt);
    static void ccSync(Crankcase& k) { k.g.T = clampv(k.U / (std::max(1e-12, k.g.m) * kCv), 150.0, 2500.0); }
    // diesel (Package 4): injection, ignition delay and the burn of one cylinder for this step
    void dieselStep(Cylinder& c, double dt, double phiPrev, double phi, double rpm, bool exOpen, bool fuelCut,
                    double& dm, double& dmf, double& dmb, double& dmbNew, double& dU);
    double valveLift(const ValveSpec& v, double phi, const ValveDynamics& dyn) const;
    double camLift(const ValveSpec& lo, const ValveSpec& hi, double phi, bool intake) const;
    void updateValveDynamics(double rpm);
    ValvetrainSpec vtIn_, vtEx_;
    ValveDynamics vdIn_, vdEx_, vdInHi_, vdExHi_;
    double baroPa_ = 101325.0;
    // trigger decoder: crank degrees travelled since the crank started moving, needed for crank / cam sync
    void triggerStep(double dTheta, double dt);
    bool syncArmed_ = false, crankSync_ = false, camSync_ = false;
    double syncTravel_ = 0, syncNeedCrank_ = 0, syncNeedCam_ = 0;
    double toothAcc_ = 0, toothT_ = 0, ckpRpm_ = 0;
    ValveSpec vinHi_, vexHi_;     // VTEC high-cam profiles
    double vtecBlend_ = 0, vtcNow_ = 0;
    void updateHealth(double dt, double rpm, double mapPa);
    void ecuTick(double h, double pedal, double rpm);
    void injectPort(int port, double rpm, double share);
    double ecuAcc_ = 1e9, vehSpeed_ = 0, lastFricW_ = 0, lamFast_ = 1;
    int vehGear_ = 0;
    double wheelSpeed_ = 0;
    bool shifting_ = false;
    int n2oRefillSeen_ = -1;
    double n2oCool_ = 0;          // K the port charge is cooled by the evaporating nitrous
    double afHeatJ_ = 0, afPower_ = 0;
    double exhOutKg_ = 0;         // gas that left the cylinders this step
    double egtProbe_ = 300, egtWall_ = 300;   // EGT thermocouple tip and header wall (K)
    double sinceCutS_ = 10;       // s since the last fuel cut (limiter / decel cut)   // afterfire heat released in the manifold (J this step, W smoothed)
    void nitrousStep(double dt);
    bool fuelCutNow_ = false;
    ComponentModel comp_;
    int repairSeen_ = 0;
    double burnDuration(Cylinder& c, double rpm, double lambda);
    double transferFlow(Cylinder& c, Crankcase& cc, double cdA, double dt);

    EngineParams p_;
    double fs_;
    CylGeom geom_;
    ValveSpec vin_, vex_;
    PortSpec pin_, pex_;          // rotary housing ports
    double cycle_ = 720;
    int nPorts_ = 1;
    int portOf_[kMaxCyl] = {};    // chamber -> intake/exhaust port (cylinder or rotor)
    double sprPort_[2][kMaxCyl] = {}, sprCyl_[3][kMaxCyl] = {};   // cached cylinder-to-cylinder spread
    std::vector<Cylinder> cyls_;
    float phase_[kMaxCyl] = {};
    double omega_ = 0, theta_ = 0;
    double torqueInst_ = 0, torqueAvg_ = 0;
    double tqBins_[1080] = {}, tqSum_ = 0, cycleDegSeen_ = 0;
    int tqBin_ = 0;
    double advance_ = 10;
    bool running_ = false;
    bool diesel_ = false;          // compression ignition, direct injection (fuelSystem DieselDI)
    double glowC_ = 20;            // glow plug tip temperature, C
    double egrPos_ = 0, egrFlow_ = 0; // EGR valve opening 0..1, flow kg/s (averaged)
    double egrAir_ = 0;               // fresh air flow kg/s averaged the same way (the EGR share)
    double fireTimer_ = 10;    // time since last combustion
    double ambT_ = 298;
    Rng rng_{12345};

    // mechanical noise
    // structure-borne sound: block / sump modes driven by combustion pressure rise and pops,
    // head / valvetrain modes driven by valve seating; frequencies scale with the bore
    ModalBank block_, head_;
    // diesel combustion noise (Package 4c): the cylinder pressures (with their ringing) through the structure
    // attenuation of an engine block (AVL generic curve, fitted) -> radiated sound pressure, Pa
    StructureAttenuation cnStruct_;
    double combNoise_ = 0, cnE2_ = 0;
    long long cnN_ = 0;
    Biquad ping_, starterBp_;
    double blockIn_ = 0, tickIn_ = 0, pingIn_ = 0, starterPhase_ = 0;
};

} // namespace es
