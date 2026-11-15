#include "audio/ImpulseResponses.h"
#include "core/MathUtil.h"
#include <cstdio>
#include <cstring>
#include <fstream>

namespace es {

const char* irChoiceNames() {
    return "Off (dry)\0Auto (by exhaust)\0Raw pipe\0Sport\0Stock muffler\0Race / tuned\0Motorcycle\0Custom WAV file\0";
}

void normaliseIR(std::vector<float>& h) {
    double e = 0;
    for (float v : h) e += (double)v * v;
    if (e <= 1e-20) return;
    float g = (float)(1.0 / std::sqrt(e));
    for (float& v : h) v *= g;
}

namespace {
// Each response = direct sound + a dense, smeared early part (the pipe/shell reflections, so short
// and so dense that they colour without echoing) + a darker diffuse tail + one broad, heavily damped
// low "body" resonance. No long ringing sine modes and no sparse taps: those read as a tin can
// (narrow resonances / comb filtering in the 300 Hz - 3 kHz range where the ear is most sensitive).
struct IrSpec {
    double lenMs;
    double directShare;           // fraction of the response energy in the direct sound
    double earlyTauMs, earlyShare;// dense early part: decay time, share of the non-direct energy
    double tailTauMs;             // diffuse tail decay
    double brightStart, brightEnd;// tail low-pass corner at the start / end of the tail
    double bodyHz, bodyQ, bodyDb; // broad low resonance (shell / volume), gain over the direct sound
};
const IrSpec kSpecs[] = {
    //  len  direct early        tail  bright        body
    {   50, 0.70, 1.2, 0.80,   5,  9000, 3500,   150, 1.0, 2.5},   // raw pipe: almost dry, a bit of pipe
    {  110, 0.55, 2.0, 0.60,  14,  8000, 2500,   110, 1.2, 4.5},   // sport: fuller, short bloom
    {  160, 0.45, 3.0, 0.55,  22,  6000, 1800,    90, 1.0, 5.0},   // stock: warm, more diffuse
    {   70, 0.65, 1.2, 0.70,   8, 11000, 4500,   130, 1.2, 3.5},   // race / tuned: crisp, tight
    {   60, 0.65, 1.0, 0.75,   6, 10000, 3500,   190, 1.2, 3.0},   // motorcycle: small, bright
};
double energy(const std::vector<double>& v) { double e = 0; for (double x : v) e += x * x; return e; }
} // namespace

std::vector<float> makeBuiltinIR(int choice, double fs) {
    int idx = clampv(choice - IrRawPipe, 0, 4);
    const IrSpec& s = kSpecs[idx];
    const int n = (int)(s.lenMs * 1e-3 * fs);
    Rng rng(1000 + idx * 31);
    // dense early part: low-passed noise with a fast exponential decay (no gaps, no discrete taps)
    std::vector<double> early(n, 0.0), tail(n, 0.0);
    double lp1 = 0, lp2 = 0;
    const double aE = onePoleCoef(s.brightStart, fs);
    for (int i = 1; i < n; ++i) {
        double t = i / fs;
        lp1 += aE * (rng.bipolar() - lp1);
        early[i] = lp1 * std::exp(-t * 1e3 / s.earlyTauMs) * smoothstep(0.0, 0.3e-3, t);
    }
    // diffuse tail, getting darker as it decays
    for (int i = 1; i < n; ++i) {
        double t = i / fs;
        double fc = s.brightEnd + (s.brightStart - s.brightEnd) * std::exp(-t * 1e3 / (s.tailTauMs * 0.7));
        lp2 += onePoleCoef(fc, fs) * (rng.bipolar() - lp2);
        tail[i] = lp2 * std::exp(-t * 1e3 / s.tailTauMs) * smoothstep(0.5e-3, 3e-3, t);
    }
    // keep the low end deterministic (direct + body): random noise there just makes a lumpy bass response
    auto highpass = [&](std::vector<double>& v, double fc) {
        const double a = onePoleCoef(fc, fs);
        for (int pass = 0; pass < 2; ++pass) {
            double lp = 0;
            for (double& x : v) { lp += a * (x - lp); x -= lp; }
        }
    };
    highpass(early, 300.0);
    highpass(tail, 300.0);
    // scale: direct gets directShare of the energy, early/tail split the rest
    const double rest = (1.0 - s.directShare) / s.directShare;      // relative to a unit direct impulse
    const double gE = std::sqrt(rest * s.earlyShare / std::max(1e-12, energy(early)));
    const double gT = std::sqrt(rest * (1.0 - s.earlyShare) / std::max(1e-12, energy(tail)));
    std::vector<double> h(n, 0.0);
    h[0] = 1.0;
    for (int i = 1; i < n; ++i) h[i] += gE * early[i] + gT * tail[i];
    // broad body resonance: damped cosine (in phase with the direct sound at resonance), tau = Q/(pi f);
    // its peak gain over the direct sound ~ A*tau*fs/2
    if (s.bodyDb > 0) {
        const double tau = s.bodyQ / (kPi * s.bodyHz);
        const double A = (std::pow(10.0, s.bodyDb / 20.0) - 1.0) * 2.0 / (tau * fs);
        for (int i = 1; i < n; ++i) {
            double t = i / fs;
            h[i] += A * std::exp(-t / tau) * std::cos(2 * kPi * s.bodyHz * t);
        }
    }
    // gentle fade at the end
    int fade = std::min(n / 8, (int)(0.01 * fs));
    for (int i = 0; i < fade; ++i) h[n - 1 - i] *= (double)i / fade;
    std::vector<float> out(h.begin(), h.end());
    normaliseIR(out);
    return out;
}

static uint32_t rd32(const unsigned char* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const unsigned char* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

bool loadWavIR(const std::wstring& path, double fs, std::vector<float>& out, std::string& err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { err = "cannot open file"; return false; }
    std::vector<unsigned char> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (d.size() < 44 || std::memcmp(d.data(), "RIFF", 4) != 0 || std::memcmp(d.data() + 8, "WAVE", 4) != 0) {
        err = "not a RIFF/WAVE file"; return false;
    }
    int fmt = 0, ch = 0, bits = 0; uint32_t rate = 0;
    const unsigned char* data = nullptr; size_t dataLen = 0;
    size_t p = 12;
    while (p + 8 <= d.size()) {
        uint32_t len = rd32(&d[p + 4]);
        if (std::memcmp(&d[p], "fmt ", 4) == 0 && p + 8 + 16 <= d.size()) {
            fmt = rd16(&d[p + 8]); ch = rd16(&d[p + 10]); rate = rd32(&d[p + 12]); bits = rd16(&d[p + 22]);
            if (fmt == 0xFFFE && len >= 26) fmt = rd16(&d[p + 8 + 24]);   // WAVE_FORMAT_EXTENSIBLE subformat
        } else if (std::memcmp(&d[p], "data", 4) == 0) {
            data = &d[p + 8];
            dataLen = std::min<size_t>(len, d.size() - (p + 8));
        }
        p += 8 + len + (len & 1);
    }
    if (!data || ch <= 0 || rate == 0) { err = "missing fmt/data chunk"; return false; }
    if (!(fmt == 1 && (bits == 16 || bits == 24 || bits == 32)) && !(fmt == 3 && bits == 32)) {
        err = "unsupported WAV encoding (use PCM 16/24/32-bit or 32-bit float)"; return false;
    }
    const int bps = bits / 8, frame = bps * ch;
    const size_t frames = dataLen / frame;
    std::vector<float> src(frames);
    for (size_t i = 0; i < frames; ++i) {
        const unsigned char* s = data + i * frame;      // first channel only
        float v;
        if (fmt == 3) { std::memcpy(&v, s, 4); }
        else if (bits == 16) v = (int16_t)rd16(s) / 32768.0f;
        else if (bits == 24) { int32_t x = (s[0] << 8) | (s[1] << 16) | (s[2] << 24); v = x / 2147483648.0f; }
        else v = (int32_t)rd32(s) / 2147483648.0f;
        src[i] = v;
    }
    // trim trailing near-silence (like engine-sim: last sample above ~-50 dB of the peak)
    float peak = 0;
    for (float v : src) peak = std::max(peak, std::fabs(v));
    if (peak <= 0) { err = "file is silent"; return false; }
    size_t last = 0;
    for (size_t i = 0; i < src.size(); ++i) if (std::fabs(src[i]) > peak * 0.003f) last = i;
    src.resize(last + 1);
    // resample (linear) to the output rate
    const double ratio = rate / fs;
    const size_t outN = (size_t)(src.size() / ratio);
    out.assign(outN, 0.f);
    for (size_t i = 0; i < outN; ++i) {
        double x = i * ratio;
        size_t i0 = (size_t)x;
        double fr = x - i0;
        float a = src[std::min(i0, src.size() - 1)], b = src[std::min(i0 + 1, src.size() - 1)];
        out[i] = (float)(a + (b - a) * fr);
    }
    normaliseIR(out);
    return !out.empty();
}

} // namespace es
