#include "audio/Convolver.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
struct FftTables {
    int rev[Convolver::kFft];
    std::complex<float> tw[Convolver::kFft / 2];
    FftTables() {
        const int n = Convolver::kFft;
        int bits = 0;
        while ((1 << bits) < n) ++bits;
        for (int i = 0; i < n; ++i) {
            int r = 0;
            for (int b = 0; b < bits; ++b) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
            rev[i] = r;
        }
        for (int k = 0; k < n / 2; ++k) {
            double a = -2.0 * 3.14159265358979323846 * k / n;
            tw[k] = std::complex<float>((float)std::cos(a), (float)std::sin(a));
        }
    }
};
const FftTables& tables() { static FftTables t; return t; }
} // namespace

void fft512(std::complex<float>* a, bool inv) {
    const FftTables& t = tables();
    const int n = Convolver::kFft;
    for (int i = 0; i < n; ++i) if (i < t.rev[i]) std::swap(a[i], a[t.rev[i]]);
    for (int len = 2; len <= n; len <<= 1) {
        const int half = len / 2, step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int j = 0; j < half; ++j) {
                std::complex<float> w = t.tw[j * step];
                if (inv) w = std::conj(w);
                std::complex<float> u = a[i + j], v = a[i + j + half] * w;
                a[i + j] = u + v;
                a[i + j + half] = u - v;
            }
        }
    }
}

Convolver::Convolver() {
    in_.assign(kFft, 0.f);
    out_.assign(kBlock, 0.f);
    work_.assign(kFft, {});
    acc_.assign(kFft, {});
}

void Convolver::setIR(const std::vector<float>& h, int maxLen) {
    int len = std::min((int)h.size(), maxLen);
    int parts = std::max(1, (len + kBlock - 1) / kBlock);
    if (len == 0) { parts_ = 0; return; }
    H_.assign((size_t)parts * kFft, {});
    for (int p = 0; p < parts; ++p) {
        std::complex<float>* dst = &H_[(size_t)p * kFft];
        for (int i = 0; i < kBlock; ++i) {
            int k = p * kBlock + i;
            dst[i] = k < len ? h[k] : 0.f;
        }
        fft512(dst, false);
    }
    if (parts != parts_) { X_.assign((size_t)parts * kFft, {}); fdl_ = 0; }
    parts_ = parts;
}

void Convolver::processBlock() {
    // spectrum of the newest 2B input samples
    for (int i = 0; i < kFft; ++i) work_[i] = in_[i];
    fft512(work_.data(), false);
    std::copy(work_.begin(), work_.end(), X_.begin() + (size_t)fdl_ * kFft);
    // Y = sum_p X[n - p] * H[p]
    std::fill(acc_.begin(), acc_.end(), std::complex<float>());
    for (int p = 0; p < parts_; ++p) {
        int slot = (fdl_ - p + parts_) % parts_;
        const std::complex<float>* x = &X_[(size_t)slot * kFft];
        const std::complex<float>* hh = &H_[(size_t)p * kFft];
        for (int k = 0; k < kFft; ++k) acc_[k] += x[k] * hh[k];
    }
    fdl_ = (fdl_ + 1) % parts_;
    fft512(acc_.data(), true);
    const float norm = 1.0f / kFft;
    for (int i = 0; i < kBlock; ++i) out_[i] = acc_[kBlock + i].real() * norm;   // overlap-save: keep 2nd half
    // slide the input frame
    std::copy(in_.begin() + kBlock, in_.end(), in_.begin());
}

float Convolver::process(float x) {
    if (parts_ == 0) return x;
    in_[kBlock + pos_] = x;
    float y = out_[pos_];
    if (++pos_ == kBlock) { pos_ = 0; processBlock(); }
    return y;
}

} // namespace es
