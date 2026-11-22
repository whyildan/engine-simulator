// Game controller probe: which pad the app would use and what it reads (gas / clutch / brake, the buttons), for a few
// seconds. usage: pad_probe [seconds]
#include "ui/Gamepad.h"
#include <windows.h>
#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    const double secs = argc > 1 ? std::atof(argv[1]) : 5.0;
    es::ui::Gamepad pad;
    const ULONGLONG t0 = GetTickCount64();
    int lastPrint = -1;
    while ((GetTickCount64() - t0) * 1e-3 < secs) {
        pad.poll(GetConsoleWindow());
        const es::ui::PadState& g = pad.state();
        const int tick = (int)((GetTickCount64() - t0) / 250);
        if (tick != lastPrint || g.shiftUp || g.shiftDown || g.neutral || g.start || g.ignition || g.driveMode) {
            lastPrint = tick;
            if (!g.connected) std::printf("no controller\n");
            else std::printf("%s  gas %3.0f%%  clutch %3.0f%%  brake %3.0f%%%s%s%s%s%s%s\n", g.name.c_str(), g.throttle * 100, g.clutch * 100,
                             g.brake * 100, g.shiftUp ? "  [shift up]" : "", g.shiftDown ? "  [shift down]" : "", g.neutral ? "  [neutral]" : "",
                             g.start ? "  [start]" : "", g.ignition ? "  [ignition]" : "", g.driveMode ? "  [drive mode]" : "");
        }
        Sleep(10);
    }
    return 0;
}
