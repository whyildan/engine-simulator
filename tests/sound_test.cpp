// Group B (sound): the exhaust acoustics checked against published results.
//  - open pipe end: the reflection coefficient and end correction of the waveguide's open end against Levine &
//    Schwinger (Silva et al., arXiv 0811.3625, eqs. 21-22: a fit within 2 % of the exact solution for ka < 3)
// usage: sound_test
#include "sim/Waveguide.h"
#include "sim/Simulator.h"
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

using namespace es;
using cd = std::complex<double>;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static cd dft(const std::vector<double>& x, double f, double fs) {
    cd s = 0;
    for (size_t n = 0; n < x.size(); ++n) s += x[n] * std::exp(cd(0, -2.0 * kPi * f * n / fs));
    return s;
}

int main() {
    const double fs = 96000;
    {
        std::printf("[open end] reflection of an unflanged pipe end vs Levine & Schwinger\n");
        const double T = 700, c = std::sqrt(kGamma * kR * T);   // hot exhaust gas
        const double radius = 0.03;                               // 60 mm tailpipe
        WaveNetwork net;
        net.init(fs, c, kPatm / (kR * T), kPatm);
        const int pipe = net.addPipe(kPi * radius * radius, 3.0, 0.45 * fs, 0.0);
        net.addOpenEnd(PipeEnd{pipe, 1}, 0.f);
        net.reset(0);
        std::vector<double> in, out;
        net.pipes[pipe].out0 = 1.0;                    // one-sample pulse sent from the far end
        for (int n = 0; n < 16384; ++n) {
            net.step(1.0 / fs);
            if (n == 0) net.pipes[pipe].out0 = 0.0;    // the far end stays silent (no reflection back)
            in.push_back(net.pipes[pipe].in1);
            out.push_back(net.pipes[pipe].out1);
        }
        double worstR = 0, worstL = 0;
        std::printf("     ka    |R| sim   L&S    L/a sim   L&S\n");
        for (double ka = 0.1; ka <= 2.001; ka += 0.1) {
            const double f = ka * c / (2.0 * kPi * radius);
            const cd R = dft(out, f, fs) / dft(in, f, fs);
            const double k2 = ka * ka;
            // Silva et al. eqs. 21-22, unflanged (beta 0.5, eta 0.6133), table 1
            const double magLS = (1 + 0.800 * k2) / (1 + (0.5 + 0.800) * k2 + 0.266 * k2 * k2 + 0.0263 * k2 * k2 * k2);
            const double laLS = 0.6133 * (1 + 0.0599 * k2) / (1 + 0.238 * k2 - 0.0153 * k2 * k2 + 0.00150 * k2 * k2 * k2);
            const double mag = std::abs(R) / 0.985;    // (the flow-loss factor is not part of the radiation theory)
            const double la = -std::arg(-R) / (2.0 * ka);   // R = -|R| exp(-2 j k L)
            if ((int)std::lround(ka * 10) % 4 == 0 || ka < 0.15)
                std::printf("    %4.1f   %6.3f  %6.3f   %6.3f  %6.3f\n", ka, mag, magLS, la, laLS);
            worstR = std::max(worstR, std::fabs(mag - magLS) / magLS);
            if (ka >= 0.2) worstL = std::max(worstL, std::fabs(la - laLS) / laLS);
        }
        CHECK(worstR < 0.08, "|R| within 8 %% of Levine & Schwinger up to ka 2 (worst %.1f %%)", worstR * 100);
        CHECK(worstL < 0.10, "end correction within 10 %% up to ka 2 (worst %.1f %%)", worstL * 100);
    }
    {
        std::printf("[expansion chamber] transmission loss of a simple chamber vs plane-wave theory\n");
        // TL = 10 log10 [1 + 1/4 (m - 1/m)^2 sin^2 kL] (anechoic outlet), m = chamber / pipe area
        const double T = 600, c = std::sqrt(kGamma * kR * T), dx = c / fs;   // lengths in whole samples: no interpolation loss
        const double S = kPi * 0.025 * 0.025, m = 6.5, L = 64 * dx;            // 50 mm pipe, chamber 6.5x its area
        WaveNetwork net;
        net.init(fs, c, kPatm / (kR * T), kPatm);
        const int a = net.addPipe(S, 400 * dx, 0.45 * fs, 0.0), ch = net.addPipe(m * S, L, 0.45 * fs, 0.0), b = net.addPipe(S, 400 * dx, 0.45 * fs, 0.0);
        net.connect({PipeEnd{a, 1}, PipeEnd{ch, 0}});
        net.connect({PipeEnd{ch, 1}, PipeEnd{b, 0}});
        net.reset(0);   // (both outer ends unconnected: no reflection - anechoic)
        std::vector<double> inc, tr;
        net.pipes[a].out0 = 1.0;
        for (int n = 0; n < 32768; ++n) {
            net.step(1.0 / fs);
            if (n == 0) net.pipes[a].out0 = 0.0;
            inc.push_back(net.pipes[a].in1);
            tr.push_back(net.pipes[b].in1);
        }
        double worst = 0;
        std::printf("    kL/pi   TL sim   theory  dB\n");
        for (double kl = 0.05; kl <= 1.95; kl += 0.05) {
            const double f = kl * kPi * c / (2.0 * kPi * L);
            const double sim = 20.0 * std::log10(std::abs(dft(inc, f, fs)) / std::abs(dft(tr, f, fs)));
            const double s = std::sin(kl * kPi), th = 10.0 * std::log10(1.0 + 0.25 * (m - 1.0 / m) * (m - 1.0 / m) * s * s);
            if ((int)std::lround(kl * 20) % 5 == 0) std::printf("    %5.2f   %6.2f   %6.2f\n", kl, sim, th);
            worst = std::max(worst, std::fabs(sim - th));
        }
        CHECK(worst < 0.5, "chamber transmission loss within 0.5 dB of theory for kL 0..2 pi (worst %.2f dB)", worst);
    }
    // transmission loss of a two-port between anechoic pipes a (in) and b (out): build(net, a1, b0) joins them
    const double T = 600, c = std::sqrt(kGamma * kR * T), dx = c / fs, S = kPi * 0.025 * 0.025;
    auto measureTL = [&](auto build, std::vector<double>& inc, std::vector<double>& tr, std::vector<GasNode>& nodes) {
        WaveNetwork net;
        net.init(fs, c, kPatm / (kR * T), kPatm);
        const int a = net.addPipe(S, 600 * dx, 0.45 * fs, 0.0), b = net.addPipe(S, 600 * dx, 0.45 * fs, 0.0);
        build(net, PipeEnd{a, 1}, PipeEnd{b, 0}, nodes);
        net.reset(0);
        net.pipes[a].out0 = 1.0;
        for (int n = 0; n < 65536; ++n) {
            net.step(1.0 / fs);
            if (n == 0) net.pipes[a].out0 = 0.0;
            inc.push_back(net.pipes[a].in1);
            tr.push_back(net.pipes[b].in1);
        }
    };
    auto tlAt = [&](const std::vector<double>& inc, const std::vector<double>& tr, double f) {
        return 20.0 * std::log10(std::abs(dft(inc, f, fs)) / std::abs(dft(tr, f, fs)));
    };
    {
        std::printf("[side branch] closed quarter-wave side branch vs TL = 10 log [1 + (Sb / 2S)^2 tan^2 kl]\n");
        const double lb = 40 * dx;
        std::vector<double> inc, tr; std::vector<GasNode> nodes;
        measureTL([&](WaveNetwork& net, PipeEnd in, PipeEnd out, std::vector<GasNode>&) {
            const int br = net.addPipe(S, lb, 0.45 * fs, 0.0);
            net.connect({in, out, PipeEnd{br, 0}});
            net.addClosedEnd(PipeEnd{br, 1});
        }, inc, tr, nodes);
        double worst = 0;
        for (double x = 0.05; x <= 0.401; x += 0.05) {   // k lb / pi (the TL goes to infinity at 0.5)
            const double f = x * kPi * c / (2.0 * kPi * lb), t = std::tan(x * kPi);
            const double th = 10.0 * std::log10(1.0 + 0.25 * t * t), sim = tlAt(inc, tr, f);
            worst = std::max(worst, std::fabs(sim - th));
        }
        CHECK(worst < 0.5, "quarter-wave side branch within 0.5 dB of theory, k l 0.05..0.4 pi (worst %.2f dB)", worst);
    }
    {
        // (the model's gas volumes are isothermal: stiffness R T instead of the adiabatic c^2 = gamma R T - the
        //  exhaust builder sizes its resonators for that, and the lumped theory here uses the same stiffness)
        std::printf("[Helmholtz] neck + volume side branch vs lumped theory (f_r = 1 / 2pi sqrt(A R T / V l))\n");
        const double A = 0.5 * S, l = 20 * dx, fr = 150.0, w = 2.0 * kPi * fr, K = kR * T;
        const double V = K * A / (w * w * l);
        std::vector<double> inc, tr; std::vector<GasNode> nodes; nodes.reserve(1);
        measureTL([&](WaveNetwork& net, PipeEnd in, PipeEnd out, std::vector<GasNode>& nd) {
            GasNode g; g.V = V; g.T = T; g.setPressure(kPatm); nd.push_back(g);
            const int neck = net.addPipe(A, l, 0.45 * fs, 0.0);
            net.connect({in, out, PipeEnd{neck, 0}});
            net.addNodeEnd(PipeEnd{neck, 1}, &nd.back());
        }, inc, tr, nodes);
        double peakF = 0, peak = -1, worst = 0;
        for (double f = 60; f <= 300; f += 1.0) {
            const double sim = tlAt(inc, tr, f);
            if (sim > peak) { peak = sim; peakF = f; }
            const double om = 2.0 * kPi * f, x = om * l / A - K / (om * V);   // branch reactance / rho (lumped)
            const double th = 10.0 * std::log10(1.0 + std::pow(c / (2.0 * S * x), 2.0));
            if (std::fabs(f / fr - 1.0) > 0.08 && f >= 0.5 * fr && f <= 2.0 * fr) worst = std::max(worst, std::fabs(sim - th));
        }
        std::printf("    tuned %.0f Hz (V %.2f L): TL peak %.1f dB at %.0f Hz\n", fr, V * 1e3, peak, peakF);
        CHECK(std::fabs(peakF / fr - 1.0) < 0.04, "resonance at the tuned frequency (%.0f vs %.0f Hz)", peakF, fr);
        CHECK(worst < 1.5, "TL within 1.5 dB of the lumped theory over 0.5-2 f_r (worst %.2f dB)", worst);
    }
    {
        std::printf("[extended tubes] chamber with inlet L/2 and outlet L/4 inside: the TL troughs at kL = pi, 2 pi fill\n");
        const double m = 6.5, L = 64 * dx;
        auto chamberTL = [&](bool ext, double kl) {
            std::vector<double> inc, tr; std::vector<GasNode> nodes;
            measureTL([&](WaveNetwork& net, PipeEnd in, PipeEnd out, std::vector<GasNode>&) {
                if (!ext) {
                    const int ch = net.addPipe(m * S, L, 0.45 * fs, 0.0);
                    net.connect({in, PipeEnd{ch, 0}}); net.connect({PipeEnd{ch, 1}, out});
                    return;
                }
                const double li = 0.5 * L, lo = 0.25 * L;   // (same topology as ExhaustSystem::chamber)
                const int tin = net.addPipe(S, li, 0.45 * fs, 0.0), annIn = net.addPipe((m - 1) * S, li, 0.45 * fs, 0.0);
                const int mid = net.addPipe(m * S, L - li - lo, 0.45 * fs, 0.0), annOut = net.addPipe((m - 1) * S, lo, 0.45 * fs, 0.0);
                const int tout = net.addPipe(S, lo, 0.45 * fs, 0.0);
                net.connect({in, PipeEnd{tin, 0}});
                net.addClosedEnd(PipeEnd{annIn, 0});
                net.connect({PipeEnd{tin, 1}, PipeEnd{annIn, 1}, PipeEnd{mid, 0}});
                net.addClosedEnd(PipeEnd{annOut, 1});
                net.connect({PipeEnd{mid, 1}, PipeEnd{annOut, 0}, PipeEnd{tout, 0}});
                net.connect({PipeEnd{tout, 1}, out});
            }, inc, tr, nodes);
            return tlAt(inc, tr, kl * c / (2.0 * kPi * L));
        };
        const double s1 = chamberTL(false, kPi), e1 = chamberTL(true, kPi), s2 = chamberTL(false, 2 * kPi), e2 = chamberTL(true, 2 * kPi);
        std::printf("    TL at kL = pi: plain %.1f dB, extended %.1f dB;  kL = 2 pi: plain %.1f dB, extended %.1f dB\n", s1, e1, s2, e2);
        CHECK(s1 < 0.5 && s2 < 0.5 && e1 > 10 && e2 > 10, "the plain chamber passes at kL = pi / 2 pi, the extended tubes block it");
    }
    {
        std::printf("[full engine] Helmholtz resonator + extended-tube muffler in a running engine: stable, finite sound\n");
        EngineParams p = makePreset(0);
        p.resonator = 2; p.resonatorHz = 0; p.muffler = Muffler::Chambered; p.mufflerExtTubes = 1;
        Simulator sim(p);
        std::vector<float> buf(480 * 2);
        double peak = 0; bool finite = true;
        auto run = [&](double s) {
            for (double t = 0; t < s; t += 0.01) {
                sim.render(buf.data(), 480);
                for (float v : buf) { if (!std::isfinite(v)) finite = false; peak = std::max(peak, (double)std::fabs(v)); }
            }
        };
        p.ignition = 1; p.starter = 1; sim.setParams(p); run(1.5);
        p.starter = 0; sim.setParams(p); run(2.0);
        Telemetry t; sim.getTelemetry(t);
        const double idleRpm = t.rpm;
        p.throttle = 1; sim.setParams(p); run(2.0);
        p.throttle = 0; sim.setParams(p); run(1.0);
        std::printf("    idle %.0f rpm, peak sample %.2f\n", idleRpm, peak);
        CHECK(finite && peak < 4.0, "no NaN / blow-up in the audio (peak %.2f)", peak);
        CHECK(idleRpm > 600, "the engine idles with them fitted (%.0f rpm)", idleRpm);
    }
    {
        std::printf("[drone] the auto-tuned Helmholtz resonator against the firing order at 2000 rpm (dyno hold, 25 %% throttle)\n");
        auto firingLevel = [&](int resonator) {
            EngineParams p = makePreset(0);
            p.resonator = resonator; p.resonatorHz = 0;
            Simulator sim(p);
            std::vector<float> buf(480 * 2);
            p.ignition = 1; p.starter = 1; sim.setParams(p);
            for (double t = 0; t < 1.5; t += 0.01) sim.render(buf.data(), 480);
            p.starter = 0; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 2000; p.throttle = 0.25f; sim.setParams(p);
            for (double t = 0; t < 4.0; t += 0.01) sim.render(buf.data(), 480);
            std::vector<double> mono;
            for (double t = 0; t < 1.0; t += 0.01) {
                sim.render(buf.data(), 480);
                for (int i = 0; i < 480; ++i) mono.push_back(0.5 * (buf[2 * i] + buf[2 * i + 1]));
            }
            Telemetry tl; sim.getTelemetry(tl);
            const double f = 2.0 * tl.rpm / 60.0;   // firing frequency of a four-cylinder
            double e = 0;   // energy within +-3 % of it (Simulator output rate)
            for (double ff = 0.97 * f; ff <= 1.03 * f; ff += 0.5) e += std::norm(dft(mono, ff, Simulator::kOutRate));
            return 10.0 * std::log10(e + 1e-30);
        };
        const double none = firingLevel(0), inl = firingLevel(1), helm = firingLevel(2);
        std::printf("    firing order level: no resonator %.1f dB, inline chamber %.1f dB, Helmholtz %.1f dB\n", none, inl, helm);
        CHECK(helm < none - 3.0 && helm < inl - 3.0, "the Helmholtz resonator takes the drone down (%.1f dB vs none)", helm - none);
    }
    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
