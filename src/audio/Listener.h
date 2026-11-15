#pragma once
#include "audio/Filters.h"
#include "sim/EngineParams.h"
#include <vector>

namespace es {

// Turns the physical sound sources (Pa @ 1 m) into a stereo signal for a listener position:
// behind the tailpipe, in the engine bay, or in the cabin.
class Listener {
public:
    void init(double fs);
    void set(ListenerPos pos, float volume) { pos_ = pos; volume_ = volume; }
    void setRoom(Environment env, float mix);
    void process(double exL, double exR, double intake, double mech, double boost, float& outL, float& outR);
    double peak() const { return peak_; }

private:
    double fs_ = 48000;
    ListenerPos pos_ = ListenerPos::Tailpipe;
    float volume_ = 1;
    double w_[3] = {1, 0, 0};     // smoothed position weights
    double gainSmooth_ = 1;
    // per-position source filters
    Biquad tailInLp_, bayExLpL_, bayExLpR_;
    Biquad cabLp1_, cabLp2_, cabShelf_, cabBoom_, cabBoom2_, cabMid_;
    Biquad dcL_, dcR_;
    OnePole tailMechLp_, tailBoostLp_, cabBoostLp_;
    std::vector<float> erL_, erR_;
    int erW_ = 0;
    double peak_ = 0, env_ = 0.01, comp_ = 1;
    // feedback delay network reverb for the space around the car
    static constexpr int kFdn = 4;
    std::vector<float> fdnBuf_[kFdn];
    int fdnLen_[kFdn] = {}, fdnPos_[kFdn] = {};
    double fdnGain_ = 0, fdnDamp_[kFdn] = {}, fdnDampA_ = 0.3, wet_ = 0, wetTarget_ = 0;
    Environment room_ = Environment::OpenRoad;
    float lastMix_ = -1;
    Biquad wetHp_;
};

} // namespace es
