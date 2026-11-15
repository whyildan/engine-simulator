#define MINIAUDIO_IMPLEMENTATION
#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_GENERATION
#define MA_NO_RESOURCE_MANAGER
#define MA_NO_NODE_GRAPH
#define MA_NO_ENGINE
#include "miniaudio.h"

#include "audio/AudioOutput.h"
#include "sim/Simulator.h"

namespace es {

static void dataCallback(ma_device* dev, void* out, const void*, ma_uint32 frames) {
    auto* sim = static_cast<Simulator*>(dev->pUserData);
    sim->pullAudio(static_cast<float*>(out), frames);
}

bool AudioOutput::start(Simulator* sim) {
    stop();
    auto* dev = new ma_device;
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = Simulator::kOutRate;
    cfg.periodSizeInFrames = 480;
    cfg.dataCallback = dataCallback;
    cfg.pUserData = sim;
    if (ma_device_init(nullptr, &cfg, dev) != MA_SUCCESS) {
        err_ = "Could not open the audio output device";
        delete dev;
        return false;
    }
    name_ = dev->playback.name;
    if (ma_device_start(dev) != MA_SUCCESS) {
        err_ = "Could not start the audio output device";
        ma_device_uninit(dev);
        delete dev;
        return false;
    }
    dev_ = dev;
    return true;
}

void AudioOutput::stop() {
    if (!dev_) return;
    auto* dev = static_cast<ma_device*>(dev_);
    ma_device_uninit(dev);
    delete dev;
    dev_ = nullptr;
}

} // namespace es
