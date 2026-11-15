// Diagnostic: print the generic and the measured base VE map of a preset (rows = MAP, cols = rpm)
// usage: map_dump <preset>
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>

using namespace es;

static void dump(const char* title, const Table2D& t) {
    std::printf("%s\n   MAP\\rpm", title);
    for (int k = 0; k < t.nx; k += 2) std::printf("%6.0f", t.x[k]);
    std::printf("\n");
    for (int r = t.ny - 1; r >= 0; --r) {
        std::printf("  %6.0f  ", t.y[r]);
        for (int k = 0; k < t.nx; k += 2) std::printf("%6.1f", t.v[r][k]);
        std::printf("\n");
    }
}

int main(int argc, char** argv) {
    int idx = argc > 1 ? std::atoi(argv[1]) : 0;
    EngineParams g = makePreset(idx);
    EngineParams c = makeCalibratedPreset(idx);
    dump("generic VE", g.cal.veMap);
    dump("measured base VE", c.cal.veMap);
    return 0;
}
