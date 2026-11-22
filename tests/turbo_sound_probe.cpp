// Turbo sound probe: a turbo preset on the rig - full throttle, then a lift-off (with and without a blow-off valve) -
// logging the shaft speed, boost, surge state and the boost sound source, and writing the boost sound alone to a WAV.
// usage: turbo_sound_probe <preset> <bov 0/1> <out.wav>
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;

static void writeWav(const char* path, const std::vector<float>& x, int rate) {
    FILE* f = std::fopen(path, "wb");
    if (!f) return;
    const uint32_t n = (uint32_t)x.size(), bytes = n * 2;
    auto w32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); w32(36 + bytes); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); w32(16); w16(1); w16(1); w32(rate); w32(rate * 2); w16(2); w16(16);
    std::fwrite("data", 1, 4, f); w32(bytes);
    float pk = 1e-9f; for (float s : x) pk = std::max(pk, std::fabs(s));
    for (float s : x) w16((uint16_t)(int16_t)(s / pk * 0.9f * 32767));
    std::fclose(f);
}

int main(int argc, char** argv) {
    const int id = argc > 1 ? std::atoi(argv[1]) : 16;
    EngineParams p = makeCalibratedPreset(id);
    p.bov = argc > 2 ? std::atoi(argv[2]) : 0;
    if (argc > 4) p.bovSpringBar = (float)std::atof(argv[4]);
    if (argc > 5) p.bovType = std::atoi(argv[5]);
    const double fs = 48000.0 * std::max(1, (int)p.oversample), dt = 1.0 / fs;
    Engine e(p, fs);
    const double rpm = 4500, w = rpm * 2.0 * kPi / 60.0;
    p.ignition = 1; e.apply(p);
    e.setCrank(w, 0.0);
    std::vector<float> snd;
    int dec = (int)std::lround(fs / 48000.0), k = 0;
    bool wasSurge = false; int surgeEvents = 0; double firstSurge = -1, lastSurge = -1;
    // surge cycles: reverse-flow episodes (the flow dropping below zero) counted per 0.25 s window
    int revCount = 0; bool rev = false; double win = 4.0;
    for (double t = 0; t < 7.0; t += dt) {
        const double thr = t < 4.0 ? 1.0 : 0.0;
        e.setCrank(w, e.theta());
        e.step(dt, thr, 0.0);
        if (++k % dec == 0 && t > 3.0) snd.push_back((float)e.boost.sound());
        if (e.boost.surging && !wasSurge) { ++surgeEvents; if (firstSurge < 0) firstSurge = t; lastSurge = t; }
        if (e.boost.surging) lastSurge = t;
        {
            const bool r = rev ? e.boost.compFlow < 0.2 * e.boost.lineS : e.boost.compFlow < 0.0;
            if (r && !rev) ++revCount;
            rev = r;
            if (t >= win && t > 4.0) {
                std::printf("   cycles %.2f-%.2f s: %d -> %.1f Hz (shaft %.0f rpm, boost %.2f bar)\n", win - 0.25, win, revCount, revCount / 0.25,
                            e.boost.shaftRpm, e.boost.boostBar);
                revCount = 0; win += 0.25;
            }
        }
        wasSurge = e.boost.surging;
        static double next = 0;
        if (t >= next) {
            std::printf("t=%.2f thr=%.0f shaft=%6.0f rpm boost=%.2f bar flow=%.3f kg/s surge=%d bov=%d snd=%.2f\n", t, thr, e.boost.shaftRpm,
                        e.boost.boostBar, e.boost.compFlow, (int)e.boost.surging, (int)e.boost.bovOpen, e.boost.bovLift, e.boost.sound());
            next += t < 3.8 ? 0.5 : (t < 4.4 ? 0.02 : 0.1);
        }
    }
    std::printf("surge: %d onsets, from %.2f to %.2f s\n", surgeEvents, firstSurge, lastSurge);
    if (argc > 3) writeWav(argv[3], snd, 48000);
    return 0;
}
