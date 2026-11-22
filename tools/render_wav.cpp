// Headless renderer: runs a preset through a scripted drive cycle and writes a WAV file.
// usage: render_wav <presetIndex|-1 for list> <out.wav> [seconds-scale]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace es;

static void writeWav(const char* path, const std::vector<float>& lr, int rate) {
    FILE* f = std::fopen(path, "wb");
    if (!f) { std::perror(path); return; }
    uint32_t n = (uint32_t)lr.size();
    uint32_t dataBytes = n * 2;
    auto w32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto w16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); w32(36 + dataBytes); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); w32(16); w16(1); w16(2); w32(rate); w32(rate * 4); w16(4); w16(16);
    std::fwrite("data", 1, 4, f); w32(dataBytes);
    for (float s : lr) { int v = (int)(clampv(s, -1.f, 1.f) * 32767); w16((uint16_t)(int16_t)v); }
    std::fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        for (int i = 0; i < presetCount(); ++i) std::printf("%2d  %s\n", i, presetName(i));
        std::printf("usage: render_wav <preset> <out.wav>\n");
        return 0;
    }
    int idx = std::atoi(argv[1]);
    // idle=SECONDS: start the engine (calibrated preset, like the app) and record only the warm idle
    double idleSecs = 0;
    for (int i = 3; i < argc; ++i) if (std::strncmp(argv[i], "idle=", 5) == 0) idleSecs = std::atof(argv[i] + 5);
    EngineParams p = idleSecs > 0 ? makeCalibratedPreset(idx) : makePreset(idx);
    // key=value options (e.g. ir=0 rasp=1.2 imp=0) can follow the positional ones
    for (int i = 3; i < argc; ++i) {
        const char* eq = std::strchr(argv[i], '=');
        if (!eq) continue;
        std::string k(argv[i], eq - argv[i]); double v = std::atof(eq + 1);
        if (k == "ir") p.irChoice = (int)v;
        else if (k == "irmix") p.irMix = (float)v;
        else if (k == "rasp") p.exhaustRasp = (float)v;
        else if (k == "punch") p.exhaustPunch = (float)v;
        else if (k == "reverb") p.reverbMix = (float)v;
        else if (k == "imp") p.imperfection = (float)v;
        else if (k == "steep") p.exhaustSteep = (float)v;
        else if (k == "style") p.soundStyle = (int)v;
        else if (k == "pmix") p.pressureMix = (float)v;
        else if (k == "smooth") p.smoothHz = (float)v;
        else if (k == "listener") p.listener = (ListenerPos)(int)v;
        else if (k == "porting") { applyRotaryPorting(p, (int)v); }
        else if (k == "bov") p.bov = (int)v;
        else if (k == "bovtype") p.bovType = (int)v;
        else if (k == "bovspring") p.bovSpringBar = (float)v;
        argv[i] = (char*)"";
    }
    if (argc > 3 && *argv[3]) p.listener = (ListenerPos)std::atoi(argv[3]);
    if (argc > 4 && *argv[4]) p.revLimit = (float)std::atof(argv[4]);
    if (argc > 5 && *argv[5]) p.limiter = (Limiter)std::atoi(argv[5]);
    if (argc > 6 && *argv[6]) p.exhaustRasp = (float)std::atof(argv[6]);
    if (argc > 7 && *argv[7]) p.exhaustPunch = (float)std::atof(argv[7]);
    if (argc > 8 && *argv[8]) p.reverbMix = (float)std::atof(argv[8]);
    if (argc > 9 && *argv[9]) p.environment = (Environment)std::atoi(argv[9]);
    if (argc > 10 && *argv[10]) p.imperfection = (float)std::atof(argv[10]);
    if (argc > 11 && *argv[11]) p.irChoice = std::atoi(argv[11]);
    Simulator sim(p);
    const int rate = Simulator::kOutRate;
    const int block = 480; // 10 ms
    std::vector<float> all, buf(block * 2);
    if (idleSecs > 0) {
        Telemetry tel;
        p.ignition = 1; p.starter = 1; sim.setParams(p);
        double caught = 0;
        for (int k = 0; k < 600 && caught < 0.15; ++k) {
            sim.render(buf.data(), block); sim.getTelemetry(tel);
            caught = tel.running && tel.rpm > starterReleaseRpm(p, tel.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
        }
        p.starter = 0; sim.setParams(p);
        for (int k = 0; k < 300; ++k) sim.render(buf.data(), block);   // 3 s to settle
        double rmin = 1e9, rmax = 0;
        for (double t = 0; t < idleSecs; t += 0.01) {
            sim.render(buf.data(), block);
            all.insert(all.end(), buf.begin(), buf.end());
            sim.getTelemetry(tel);
            rmin = std::min(rmin, (double)tel.rpm); rmax = std::max(rmax, (double)tel.rpm);
        }
        writeWav(argv[2], all, rate);
        std::printf("wrote %s (%s idle, %.0f..%.0f rpm, misfires %d, afterfires %d)\n", argv[2], p.name, rmin, rmax, tel.misfires, tel.afterfires);
        return 0;
    }

    struct Step { double t; float throttle; int ign; int starter; };
    const Step script[] = {
        {0.0, 0.00f, 1, 1}, {1.2, 0.00f, 1, 0}, {3.5, 0.35f, 1, 0}, {3.9, 0.00f, 1, 0},
        {5.0, 1.00f, 1, 0}, {7.5, 0.00f, 1, 0}, {9.5, 0.15f, 1, 0}, {11.0, 0.00f, 1, 0}, {12.5, 0, 0, 0},
    };
    const double total = 13.0;
    double t = 0, nextPrint = 0;
    double rms = 0; int rmsN = 0;
    Telemetry tel;
    for (int b = 0; t < total; ++b) {
        int si = 0;
        for (int i = 0; i < (int)(sizeof(script) / sizeof(script[0])); ++i) if (script[i].t <= t) si = i;
        if (p.throttle != script[si].throttle || p.ignition != script[si].ign || p.starter != script[si].starter) {
            p.throttle = script[si].throttle; p.ignition = script[si].ign; p.starter = script[si].starter;
            sim.setParams(p);
        }
        sim.render(buf.data(), block);
        for (float s : buf) { rms += s * s; ++rmsN; }
        all.insert(all.end(), buf.begin(), buf.end());
        t += (double)block / rate;
        if (t >= nextPrint) {
            sim.getTelemetry(tel);
            std::printf("t=%5.2f thr=%.2f rpm=%6.0f tq=%7.1f map=%5.1f afr=%5.2f ve=%.2f adv=%4.1f iac=%4.1f pk=%.2f rms=%.3f load=%.2f mis=%d af=%d\n",
                        t, p.throttle, tel.rpm, tel.torque, tel.mapKPa, tel.afr, tel.ve, tel.advance, tel.iac,
                        tel.peak, std::sqrt(rms / std::max(1, rmsN)), tel.simLoad, tel.misfires, tel.afterfires);
            rms = 0; rmsN = 0;
            nextPrint += 0.25;
        }
    }
    writeWav(argv[2], all, rate);
    std::printf("wrote %s (%s)\n", argv[2], p.name);
    return 0;
}
