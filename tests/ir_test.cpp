// Checks the partitioned FFT convolver against brute-force convolution, and the WAV IR loader.
// usage: ir_test [wav-file]
#include "audio/Convolver.h"
#include "audio/ImpulseResponses.h"
#include "core/MathUtil.h"
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace es;

int main(int argc, char** argv) {
    int failures = 0;
    // ---- convolver vs direct convolution (IR longer than several partitions)
    Rng rng(5);
    std::vector<float> h(3000), x(12000);
    for (float& v : h) v = (float)rng.bipolar();
    for (float& v : x) v = (float)rng.bipolar();
    Convolver cv;
    cv.setIR(h);
    double maxErr = 0, maxRef = 0;
    for (size_t n = 0; n < x.size(); ++n) {
        float y = cv.process(x[n]);
        long m = (long)n - Convolver::kBlock;          // the convolver is one block late
        if (m < 0) continue;
        double ref = 0;
        for (size_t k = 0; k < h.size() && (long)k <= m; ++k) ref += (double)h[k] * x[m - k];
        maxErr = std::max(maxErr, std::fabs(ref - y));
        maxRef = std::max(maxRef, std::fabs(ref));
    }
    std::printf("convolver: max error %.2e (signal peak %.1f) -> %s\n", maxErr, maxRef, maxErr < 1e-3 * maxRef ? "OK" : "FAIL");
    if (maxErr >= 1e-3 * maxRef) ++failures;

    // ---- built-in responses: unit energy, sensible lengths
    for (int c = IrRawPipe; c <= IrMotorcycle; ++c) {
        auto b = makeBuiltinIR(c, 48000);
        double e = 0; for (float v : b) e += v * v;
        std::printf("built-in IR %d: %zu samples (%.0f ms), energy %.3f\n", c, b.size(), b.size() / 48.0, e);
        if (std::fabs(e - 1.0) > 1e-3) ++failures;
        // colouration guard: octave-band levels 63 Hz..8 kHz must stay within 9 dB of each other
        // (strong ringing modes or a heavy low-pass made the exhaust sound like a tin can / muffled)
        double lo = 1e9, hi = -1e9;
        for (double fc = 63; fc < 9000; fc *= 2) {
            double acc = 0; int cnt = 0;
            for (double f = fc / 1.414; f < fc * 1.414; f *= 1.01, ++cnt) {
                double re = 0, im = 0;
                for (size_t k = 0; k < b.size(); ++k) { double w = 2 * kPi * f * k / 48000.0; re += b[k] * std::cos(w); im -= b[k] * std::sin(w); }
                acc += re * re + im * im;
            }
            double db = 10 * std::log10(acc / cnt + 1e-20);
            lo = std::min(lo, db); hi = std::max(hi, db);
        }
        std::printf("   octave-band spread %.1f dB -> %s\n", hi - lo, hi - lo < 9.0 ? "OK" : "FAIL");
        if (hi - lo >= 9.0) ++failures;
    }

    // ---- "dump <dir>": write the built-in responses as raw float32 for offline analysis
    if (argc > 2 && std::string(argv[1]) == "dump") {
        for (int c = IrRawPipe; c <= IrMotorcycle; ++c) {
            auto b = makeBuiltinIR(c, 48000);
            std::string fn = std::string(argv[2]) + "/ir" + std::to_string(c) + ".f32";
            if (FILE* f = std::fopen(fn.c_str(), "wb")) { std::fwrite(b.data(), 4, b.size(), f); std::fclose(f); }
        }
        return 0;
    }
    // ---- WAV loader
    if (argc > 1) {
        std::string a(argv[1]);
        std::wstring w(a.begin(), a.end());
        std::vector<float> ir;
        std::string err;
        bool ok = loadWavIR(w, 48000, ir, err);
        double e = 0; for (float v : ir) e += v * v;
        std::printf("WAV %s: %s, %zu samples @48k, energy %.3f %s\n", argv[1], ok ? "loaded" : "FAILED", ir.size(), e, err.c_str());
        if (!ok) ++failures;
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}
