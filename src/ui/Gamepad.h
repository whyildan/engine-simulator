#pragma once
// Game controller input for driving: an Xbox-style pad through XInput (also a DualShock / DualSense run through Steam
// Input or DS4Windows), or a DualShock 4 / DualSense - or another pad - straight through DirectInput.
//   R2 / RT   throttle (analog)          L2 / LT   clutch (analog)
//   R1 / RB   shift up                   L1 / LB   shift down
//   Cross / A brake (full) - or the right stick pulled back (analog)
//   Triangle / Y neutral    Options / Start  start the engine    Share / Back  ignition on / off
//   Circle / B driving mode (car / dyno)     D-pad left / right  listener position
#include <cstdint>
#include <string>

namespace es::ui {

struct PadState {
    bool connected = false;
    std::string name;            // "DualSense (DirectInput)", "Xbox controller (XInput)" ...
    float throttle = 0;          // 0..1
    float clutch = 0;            // 0..1
    float brake = 0;             // 0..1 (Cross / A, or the right stick pulled back)
    // buttons this frame (pressed now - edges)
    bool shiftUp = false, shiftDown = false, neutral = false, start = false, ignition = false, driveMode = false;
    bool listenerPrev = false, listenerNext = false;
};

class Gamepad {
public:
    ~Gamepad();
    // reads the first connected pad (call once a frame; looks for a newly connected one every 2 s)
    void poll(void* hwnd);
    const PadState& state() const { return s_; }

private:
    PadState s_;
    struct Impl;
    Impl* impl_ = nullptr;
    uint32_t held_ = 0;          // buttons held on the last frame (for the edges)
    double nextScan_ = 0;
};

} // namespace es::ui
