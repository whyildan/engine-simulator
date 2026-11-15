#pragma once
#include <string>

namespace es {

class Simulator;

// Plays the simulator's ring buffer through the default output device (miniaudio / WASAPI).
class AudioOutput {
public:
    ~AudioOutput() { stop(); }
    bool start(Simulator* sim);
    void stop();
    bool running() const { return dev_ != nullptr; }
    const std::string& deviceName() const { return name_; }
    const std::string& error() const { return err_; }

private:
    void* dev_ = nullptr;
    std::string name_, err_;
};

} // namespace es
