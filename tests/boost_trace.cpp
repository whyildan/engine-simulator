// Diagnostic: time trace of the turbo loop at a fixed engine speed and full throttle.
// usage: boost_trace [preset] [rpm]
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>

using namespace es;

int main(int argc, char** argv) {
    int preset = argc > 1 ? std::atoi(argv[1]) : 16;
    double rpm = argc > 2 ? std::atof(argv[2]) : 5000;
    EngineParams p = makeCalibratedPreset(preset);
    p.ignition = 1;
    Engine e(p, 96000);
    const double dt = 1.0 / 96000;
    for (int k = 0; k < 96000 * 5; ++k) {
        e.setCrank(rpm * 2 * kPi / 60, e.theta());
        e.step(dt, k > 96000 ? 1.0 : 0.0, 0);
        if (k > 96000 * 3 && k % 1920 == 0)
            std::printf("t %.2f boost %+.3f shaft %6.0f n %.2f wg %.2f flow %5.1f g/s eta? PR %.2f  egt %4.0fK lam %.2f  tq %.0f  turbIn %.2f bar out %.2f\n",
                        k * dt, e.boost.boostBar, e.boost.shaftRpm, e.boost.speedFrac, e.boost.wastegate, e.boost.compFlow * 1000,
                        e.boost.pr, e.egtAvg, e.lambdaAvg, e.netTorque(),
                        e.exhaust.turbineIn().p() / 1e5, e.exhaust.turbineOut().p() / 1e5);
    }
}
