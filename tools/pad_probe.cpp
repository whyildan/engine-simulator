// Game controller probe: which pad the app would use and what it reads (gas / clutch / brake, the buttons), for a few
// seconds. usage: pad_probe [seconds] [rumble]   (rumble: runs the big motor, then the small one, ~0.6 s each)
#include "ui/Gamepad.h"
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
    const double secs = argc > 1 ? std::atof(argv[1]) : 5.0;
    const bool rumble = argc > 2 && std::strcmp(argv[2], "rumble") == 0;
    es::ui::Gamepad pad;
    const ULONGLONG t0 = GetTickCount64();
    int lastPrint = -1;
    while ((GetTickCount64() - t0) * 1e-3 < secs) {
        pad.poll(GetConsoleWindow());
        const es::ui::PadState& g = pad.state();
        const double t = (GetTickCount64() - t0) * 1e-3;
        if (rumble) pad.setRumble(t > 0.5 && t < 1.1 ? 0.7f : 0.0f, t > 1.4 && t < 2.0 ? 0.7f : 0.0f);
        const int tick = (int)((GetTickCount64() - t0) / 250);
        if (tick != lastPrint || g.shiftUp || g.shiftDown || g.neutral || g.start || g.ignition || g.driveMode) {
            lastPrint = tick;
            if (!g.connected) std::printf("no controller\n");
            else std::printf("%s  gas %3.0f%%  clutch %3.0f%%  brake %3.0f%%  rumble %s%s%s%s%s%s%s\n", g.name.c_str(), g.throttle * 100, g.clutch * 100,
                             g.brake * 100, pad.rumbleSupported() ? "yes" : "no", g.shiftUp ? "  [shift up]" : "", g.shiftDown ? "  [shift down]" : "",
                             g.neutral ? "  [neutral]" : "", g.start ? "  [start]" : "", g.ignition ? "  [ignition]" : "", g.driveMode ? "  [drive mode]" : "");
        }
        Sleep(10);
    }
    pad.setRumble(0, 0);
    return 0;
}
