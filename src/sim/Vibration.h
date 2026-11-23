#pragma once
// Engine vibration as the car (or the test stand) feels it through the engine mounts - drives the game controller's
// rumble motors.
//
// Sources (Taylor, "The Internal-Combustion Engine in Theory and Practice" vol. 2 ch. 8; Heywood ch. 2):
//  - shaking forces of the reciprocating masses (piston + 1/3 rod): along each cylinder's axis
//      F = m_rec x'' = m_rec r w^2 (cos a + lambda cos 2a)            (lambda = r / l; rotating masses counterweighted)
//    summed as vectors over the cylinders (an I4's secondary, a single's / V-twin's primary), and their rocking
//    couples about the engine's middle (an I3's / I5's / 90-deg V6's primary couple ...). Each cylinder's primary
//    (cos a along its axis) is two vectors of half the size turning opposite ways; the crank's counterweights cancel
//    the sum that turns with the crank, force and couple, and the counter-turning sum stays - so a 90-deg V-twin and
//    a crossplane V8 come out primary-balanced and a single keeps half its primary (the usual 50 % balance factor);
//  - the roll reaction: the block takes back the torque the gas and the reciprocating masses put on the crank
//      T = (p A - m_rec x'') dx/da                                    (the gas part is the engine's own torqueInst)
//    without its mean - the firing pulses, misfires, a rough idle, the compression pulses while cranking.
// The engine sits on rubber mounts (a rigid-body mode ~8-14 Hz, damping ratio ~0.15 - Yu, Naganathan & Dukkipati,
// "A literature review of automotive vehicle engine mounting systems", Mech. Mach. Theory 36, 2001; chosen values
// within that range): each force / moment is passed through the mount (transmitted = k x + c x', a 2nd-order system),
// so the idle shake comes through and a high-rpm buzz is filtered down. The transmitted forces over a reference
// sprung mass (400 kg - chosen) give an acceleration (m/s^2), split at 35 Hz into a low band (the big, slow motor)
// and a high band (the small, fast one), each an RMS envelope over ~25 ms.
#include "sim/EngineParams.h"
#include "core/MathUtil.h"

namespace es {

struct ComponentModel;

class EngineVibration {
public:
    void setup(const EngineParams& p, const ComponentModel& comp);
    // theta: crank angle in cycle degrees; phases: each cylinder's phase (cycle deg); gasTorque: the crank torque from
    // the gas (N m, Engine::instTorque)
    void step(double dt, double theta, double omega, const double* phases, int n, double gasTorque);
    double low() const { return low_; }     // m/s^2 RMS, below ~35 Hz
    double high() const { return high_; }   // m/s^2 RMS, above ~35 Hz
    // the block's motion on its mounts over the last cycle, per cycle degree (the engine view replays it at its
    // slow-motion crank angle): lateral, vertical (m), roll about the crank (rad)
    static constexpr int kBins = 1080;
    float trX[kBins] = {}, trY[kBins] = {}, trRoll[kBins] = {};

private:
    struct Mode {           // one mount direction: x'' + 2 z wn x' + wn^2 x = input; transmitted = wn^2 x + 2 z wn x'
        double wn = 60, z = 0.15, x = 0, v = 0;
        double lp = 0, envLo = 0, envHi = 0, hp = 0;
    };
    double transmit(Mode& m, double in, double dt);
    bool piston_ = true;
    int n_ = 0;
    double mRec_ = 0.5, r_ = 0.04, lam_ = 0.28, cycle_ = 720, area_ = 5e-3;
    double ux_[kMaxCyl] = {}, uy_[kMaxCyl] = {}, z_[kMaxCyl] = {};
    double mass_ = 150, rollI_ = 5, halfLen_ = 0.3;
    double tqMean_ = 0;
    // primary the counterweights cancel: (1/2) r w^2 e^{s j theta} times these sums (force F, couple K; x + j y)
    bool cwReady_ = false;
    double cwFx_ = 0, cwFy_ = 0, cwKx_ = 0, cwKy_ = 0, cwSign_ = 1;
    Mode m_[5];             // lateral, vertical, roll, pitch, yaw
    double low_ = 0, high_ = 0;
    int bin_ = -1;
};

} // namespace es
