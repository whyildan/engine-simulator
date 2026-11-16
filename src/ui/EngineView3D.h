#pragma once
// Engine view: flat vector cutaways of every cylinder / rotor, placed as thin layered slabs along the crankshaft
// in a small 3D scene (2.5D). From the front it looks like a 2D drawing; drag to orbit around it.
#include "sim/EngineParams.h"
#include "imgui.h"

namespace es::ui {

struct ViewCam {
    float yaw = 28, pitch = 12; // deg (a gentle 3/4 view: every cylinder visible; yaw 0 / pitch 0 = flat front)
    float zoom = 1;
    float panX = 0, panY = 0;   // pixels
    float depth = 1;            // cutaway slab thickness scale
};

// live state the parameters alone do not tell
struct EngineViewLive {
    float advance = 15;         // spark advance now (deg BTDC)
    float vtecBlend = 0;        // 0 low cam .. 1 high cam
    float vtcAdvance = 0;       // intake cam advance now (crank deg)
};

// draws the engine (piston or rotary) into `size` at the cursor and handles orbit / zoom / pan on it
void EngineView3D(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live,
                  ViewCam& cam, ImVec2 size);

} // namespace es::ui
