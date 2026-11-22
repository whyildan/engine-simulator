#pragma once
// Engine view: a parametric 3D model of the engine drawn on the GPU (Direct3D 11) - pistons, rods, crank, valvetrain,
// timing drive, glass liners and heads, the gas in each cylinder by stroke. Falls back to the flat view (vector
// cutaways as thin layered slabs, 2.5D) when the GPU path is not available.
#include "sim/EngineParams.h"
#include "imgui.h"

namespace es::ui {

struct ViewCam {
    float yaw = 28, pitch = 12; // deg (a gentle 3/4 view: every cylinder visible; yaw 0 / pitch 0 = flat front)
    float zoom = 1;
    float panX = 0, panY = 0;   // pixels
    float depth = 1;            // cutaway slab thickness scale (flat view)
    float glass = 0.16f;        // opacity of the glass liners / heads (3D view)
    bool flat = false;          // use the flat (2.5D) view
};

// live state the parameters alone do not tell
struct EngineViewLive {
    float advance = 15;         // spark advance now (deg BTDC); a diesel: start of injection
    float vtecBlend = 0;        // 0 low cam .. 1 high cam
    float vtcAdvance = 0;       // intake cam advance now (crank deg)
    // GPU view extras
    float time = 0;             // s (wall clock: spinning turbo / supercharger, flicker)
    float shaftRpm = 0;         // turbo / supercharger shaft
    float animSpeed = 0.02f;
    float throttle = 0;         // throttle opening 0..1 (a carburettor's slide, a throttle body's plate)    // the view's slow motion (the crank angle runs at rpm x this): the turbo shaft too
    float boostBar = 0;
    float egtC[kMaxCyl] = {};   // exhaust gas temperature per cylinder (headers glow), 0 = use egtGas
    float egtGas = 0;
    bool glowPlugOn = false;    // diesel glow plugs
    float egr = 0;              // EGR valve opening 0..1
    float popFlash = 0;         // 0..1: an afterfire pop just now (flame at the tail pipe)
};

// draws the engine (piston or rotary) into `size` at the cursor and handles orbit / zoom / pan on it
void EngineView3D(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live,
                  ViewCam& cam, ImVec2 size);

} // namespace es::ui
