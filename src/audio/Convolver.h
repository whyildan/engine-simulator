#pragma once
// Uniformly partitioned overlap-save FFT convolution (real time, fixed latency of one block).
// Used to run the exhaust sound through an impulse response (muffler / pipe / mic character).
#include <complex>
#include <vector>

namespace es {

class Convolver {
public:
    static constexpr int kBlock = 256;           // latency in samples
    static constexpr int kFft = 2 * kBlock;

    Convolver();
    // replaces the impulse response (any length; trimmed to maxLen). Keeps the input history.
    void setIR(const std::vector<float>& h, int maxLen = 24000);
    bool empty() const { return parts_ == 0; }
    float process(float x);                       // one sample in, one (delayed by kBlock) out

private:
    void processBlock();
    int parts_ = 0;
    std::vector<std::complex<float>> H_;          // parts x kFft spectra of the IR partitions
    std::vector<std::complex<float>> X_;          // frequency-domain delay line (parts x kFft)
    int fdl_ = 0;
    std::vector<float> in_;                       // last kFft input samples (overlap-save frame)
    std::vector<float> out_;                      // current output block
    int pos_ = 0;
    std::vector<std::complex<float>> work_, acc_;
};

// in-place radix-2 complex FFT of size kFft (inverse when inv = true, not normalised)
void fft512(std::complex<float>* a, bool inv);

} // namespace es
