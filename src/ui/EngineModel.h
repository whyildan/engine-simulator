#pragma once
// Parametric 3D engine model for the GPU engine view: the scene for one frame (meshes placed by the crank angle).
#include "sim/EngineParams.h"
#include "ui/Gpu3D.h"
#include <string>
#include <vector>

namespace es::ui {

struct EngineViewLive;

struct ModelItem { int mesh; m3::M4 world; gpu::Material mat; };
struct ModelGlow { m3::V3 p; float size, r, g, b, a; };
struct ModelLabel { m3::V3 p; std::string text; bool accent = false; };
struct EngineModel {
    std::vector<ModelItem> items;
    std::vector<ModelGlow> glows;
    std::vector<ModelLabel> labels;
    m3::V3 lo = m3::v3(1e9f, 1e9f, 1e9f), hi = m3::v3(-1e9f, -1e9f, -1e9f);   // bounds
    std::vector<m3::V3> hull;   // corners of every part's own box (a tight camera fit)
    void clear() { items.clear(); glows.clear(); labels.clear(); hull.clear(); lo = m3::v3(1e9f, 1e9f, 1e9f); hi = m3::v3(-1e9f, -1e9f, -1e9f); }
};

void buildEngineModel(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, const EngineViewLive& live, float glassAlpha,
                      EngineModel& out);

} // namespace es::ui
