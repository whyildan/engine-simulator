#pragma once
#include "sim/Cylinder.h"
#include "sim/Exhaust.h"
#include "sim/Intake.h"
#include "sim/Boost.h"
#include "sim/Components.h"
#include "sim/Thermal.h"
#include "ecu/Sensors.h"
#include "ecu/Ecu.h"
#include "audio/Filters.h"
#include <vector>

namespace es {

// The complete engine: crank + cylinders + intake + exhaust. Stepped at the simulation rate.
class Engine {
public:
    Engine(const EngineParams& p, double fs);

    void apply(const EngineParams& p);                  // live (non structural) changes
    void setCrank(double omega, double thetaDeg) { omega_ = omega; theta_ = thetaDeg; }

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
    void setVehicleState(double speedMs, int gear) { vehSpeed_ = speedMs; vehGear_ = gear; }
    // carry thermal / electrical state over a rebuild (structural change while running)
    void copyThermalFrom(const Engine& o) { thermal = o.thermal; elec = o.elec; rail = o.rail; }
    static constexpr int kTrace = 1080;
    float traceP[kTrace] = {}, traceV[kTrace] = {}, traceExh[kTrace] = {}, traceInt[kTrace] = {};
    double lambdaAvg = 1, veAvg = 0, knockMax = 0, egtAvg = 800;   // egtAvg: in the manifold (after the port)
    double egtCyl = 800;          // gas leaving the cylinders (before the port heat loss)
    int misfires = 0, afterfires = 0;
    // hardware components: derived model and live state
    const ComponentModel& components() const { return comp_; }
    double stress = 0;            // peak inertia load / limit of the weakest part
    double floatIn = 0, floatEx = 0;  // valve float severity 0..1
    double sparkMargin = 9;       // lowest available / needed spark energy over the last moments
    double sparkAvail = 0;        // mJ available at the current rpm
    double oilBar = 0;            // oil pressure
    double plugTempC = 450;       // spark plug tip temperature
    double fouling = 0;           // plug fouling 0..1
    double damage = 0;            // accumulated mechanical damage 0..1 (only with failure enabled)
    int failure = 0;              // 0 none, 1 piston, 2 rod, 3 crank, 4 valve-to-piston contact, 5 knock (piston)
    const EngineParams& params() const { return p_; }
    const float* phases() const { return phase_; }

private:
    double exhFlowAvg_ = 0;
    void stepCylinder(int i, double dt, double phiPrev, double phi, double rpm, bool spark, bool fuelCut);
    void updateValves();
    void setupChambers();
    void apexLeakage(double dt);
    double valveLift(const ValveSpec& v, double phi, double fl) const;
    void updateHealth(double dt, double rpm, double mapPa);
    void ecuTick(double h, double pedal, double rpm);
    void injectPort(int port, double rpm, double share);
    double ecuAcc_ = 1e9, vehSpeed_ = 0, lastFricW_ = 0, lamFast_ = 1;
    int vehGear_ = 0;
    bool fuelCutNow_ = false;
    ComponentModel comp_;
    int repairSeen_ = 0;
    double burnDuration(const Cylinder& c, double rpm, double lambda);

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
    double fireTimer_ = 10;    // time since last combustion
    double ambT_ = 298;
    Rng rng_{12345};

    // mechanical noise
    // structure-borne sound: block / sump modes driven by combustion pressure rise and pops,
    // head / valvetrain modes driven by valve seating; frequencies scale with the bore
    ModalBank block_, head_;
    Biquad ping_, starterBp_;
    double blockIn_ = 0, tickIn_ = 0, pingIn_ = 0, starterPhase_ = 0;
};

} // namespace es
