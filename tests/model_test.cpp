// 3D engine model sanity: every preset builds a model with finite bounds of a sensible size, at a few crank angles.
// The GPU is stubbed (mesh ids only). `model_test N` prints which item sets each side of preset N's bounds.
#include "ui/EngineModel.h"
#include "ui/EngineView3D.h"
#include "ui/Gpu3D.h"
#include "ui/Mesh3D.h"
#include "sim/EngineParams.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;
using namespace es::ui;

namespace {
struct Box { m3::V3 lo, hi; };
std::vector<Box> g_mesh;
} // namespace

namespace es::ui::gpu {
void setDevice(ID3D11Device*, ID3D11DeviceContext*) {}
void shutdown() {}
bool ready() { return true; }
int meshCreate(const mesh::Data& d) {
    if (d.v.empty() || d.i.empty()) return -1;
    Box b{m3::v3(1e9f, 1e9f, 1e9f), m3::v3(-1e9f, -1e9f, -1e9f)};
    for (const auto& q : d.v) {
        b.lo = m3::v3(std::min(b.lo.x, q.px), std::min(b.lo.y, q.py), std::min(b.lo.z, q.pz));
        b.hi = m3::v3(std::max(b.hi.x, q.px), std::max(b.hi.y, q.py), std::max(b.hi.z, q.pz));
    }
    g_mesh.push_back(b);
    return (int)g_mesh.size() - 1;
}
void meshClear() { g_mesh.clear(); }
int meshCount() { return (int)g_mesh.size(); }
bool begin(int, int, const float[3], const M4&, const M4&, V3) { return true; }
void draw(int, const M4&, const Material&) {}
void glow(V3, float, float, float, float, float) {}
void end() {}
void* texture() { return nullptr; }
} // namespace es::ui::gpu

static Box itemBox(const ModelItem& it) {
    const Box& b = g_mesh[it.mesh];
    Box r{m3::v3(1e9f, 1e9f, 1e9f), m3::v3(-1e9f, -1e9f, -1e9f)};
    for (int k = 0; k < 8; ++k) {
        const m3::V3 q = m3::xform(it.world, m3::v3(k & 1 ? b.hi.x : b.lo.x, k & 2 ? b.hi.y : b.lo.y, k & 4 ? b.hi.z : b.lo.z));
        r.lo = m3::v3(std::min(r.lo.x, q.x), std::min(r.lo.y, q.y), std::min(r.lo.z, q.z));
        r.hi = m3::v3(std::max(r.hi.x, q.x), std::max(r.hi.y, q.y), std::max(r.hi.z, q.z));
    }
    return r;
}

int main(int argc, char** argv) {
    const int only = argc > 1 ? std::atoi(argv[1]) : -1;
    int fails = 0;
    for (int i = 0; i < presetCount(); ++i) {
        if (only >= 0 && i != only) continue;
        const EngineParams p = makePreset(i);
        float ph[kMaxCyl];
        computePhases(p, ph);
        EngineViewLive live;
        live.egtGas = 650; live.shaftRpm = 60000; live.time = 1.0f; live.popFlash = 1.0f;
        EngineModel m;
        for (float crank : {0.0f, 137.0f, 411.0f}) {
            buildEngineModel(p, ph, crank, live, 0.16f, m);
            const m3::V3 sz = m.hi - m.lo;
            const float bore = (float)p.bore_mm, stroke = (float)p.stroke_mm;
            const float scale = std::max(bore, stroke);
            const bool finite = std::isfinite(sz.x) && std::isfinite(sz.y) && std::isfinite(sz.z);
            const float big = std::max({sz.x, sz.y, sz.z}) / scale;
            const bool ok = !m.items.empty() && finite && big > 2.0f && big < 30.0f;
            if (!ok || only >= 0)
                std::printf("%-2d %-34s crank %5.0f  items %4zu glows %3zu  size %6.0f x %6.0f x %6.0f mm (%.1f x bore)%s\n", i,
                            presetName(i), crank, m.items.size(), m.glows.size(), sz.x, sz.y, sz.z, big, ok ? "" : "  FAIL");
            if (!ok) ++fails;
            if (only >= 0 && crank == 0.0f) {
                // which items make each side of the bounds
                const char* side[6] = {"-x", "+x", "-y", "+y", "-z", "+z"};
                for (int s = 0; s < 6; ++s) {
                    int best = -1; float bv = 0;
                    for (size_t k = 0; k < m.items.size(); ++k) {
                        const Box b = itemBox(m.items[k]);
                        const float v = s == 0 ? -b.lo.x : s == 1 ? b.hi.x : s == 2 ? -b.lo.y : s == 3 ? b.hi.y : s == 4 ? -b.lo.z : b.hi.z;
                        if (best < 0 || v > bv) { best = (int)k; bv = v; }
                    }
                    const Box b = itemBox(m.items[best]);
                    const auto& mt = m.items[best].mat;
                    std::printf("   %s item %4d  box (%6.0f %6.0f %6.0f)..(%6.0f %6.0f %6.0f)  colour %.2f %.2f %.2f%s\n", side[s], best,
                                b.lo.x, b.lo.y, b.lo.z, b.hi.x, b.hi.y, b.hi.z, mt.r, mt.g, mt.b, mt.glass ? " glass" : "");
                }
            }
        }
    }
    // mesh churn: animating must reuse the cached shapes (new meshes every frame would stall the GPU upload)
    for (int i = 0; i < presetCount(); ++i) {
        if (only >= 0 && i != only) continue;
        const EngineParams p = makePreset(i);
        float ph[kMaxCyl];
        computePhases(p, ph);
        EngineViewLive live;
        EngineModel m;
        buildEngineModel(p, ph, 0.0f, live, 0.16f, m);
        int created = 0;
        for (int f = 1; f <= 240; ++f) {
            const int before = gpu::meshCount();
            live.time = f / 60.0f;
            buildEngineModel(p, ph, f * 3.0f, live, 0.16f, m);
            const int after = gpu::meshCount();
            created += after >= before ? after - before : after;
        }
        const float perFrame = created / 240.0f;
        if (perFrame > 2.0f || only >= 0)
            std::printf("%-2d %-34s new meshes per frame while turning: %.1f%s\n", i, presetName(i), perFrame, perFrame > 2.0f ? "  FAIL" : "");
        if (perFrame > 2.0f) ++fails;
    }
    std::printf("%s (%d failures)\n", fails ? "MODEL TEST FAILED" : "model test passed", fails);
    return fails ? 1 : 0;
}
