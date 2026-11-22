// Offscreen render of the 3D engine view to an image file, without a window (a D3D11 device of its own) - for checking
// the model while working on it.
// usage: model_render <preset> <out.bmp> [crankDeg] [yaw] [pitch] [zoom] [fx fy fz]
//   fx fy fz: the point to look at, as fractions of the model's box (0.5 0.5 0.5 = its centre); with zoom > 1 it
//   frames that part
#include "sim/EngineParams.h"
#include "ui/EngineView3D.h"
#include "ui/EngineModel.h"
#include "ui/Gpu3D.h"
#include <d3d11.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace es;
using namespace es::ui;

static bool writeBmp(const char* path, const std::vector<uint8_t>& rgba, int w, int h) {
    FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const int row = w * 3, pad = (4 - row % 4) % 4, size = 54 + (row + pad) * h;
    uint8_t hd[54] = {'B', 'M'};
    auto put32 = [&](int o, int v) { hd[o] = v & 255; hd[o + 1] = (v >> 8) & 255; hd[o + 2] = (v >> 16) & 255; hd[o + 3] = (v >> 24) & 255; };
    put32(2, size); put32(10, 54); put32(14, 40); put32(18, w); put32(22, h); hd[26] = 1; hd[28] = 24;
    std::fwrite(hd, 1, 54, f);
    const uint8_t zero[3] = {0, 0, 0};
    for (int y = h - 1; y >= 0; --y) {
        for (int x = 0; x < w; ++x) { const uint8_t* p = &rgba[((size_t)y * w + x) * 4]; const uint8_t bgr[3] = {p[2], p[1], p[0]}; std::fwrite(bgr, 1, 3, f); }
        std::fwrite(zero, 1, pad, f);
    }
    std::fclose(f);
    return true;
}

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("usage: model_render <preset> <out.bmp> [crankDeg] [yaw] [pitch] [zoom] [fx fy fz]\n"); return 1; }
    const int preset = std::atoi(argv[1]);
    const float crankDeg = argc > 3 ? (float)std::atof(argv[3]) : 40.0f;
    const float yawD = argc > 4 ? (float)std::atof(argv[4]) : 28.0f, pitchD = argc > 5 ? (float)std::atof(argv[5]) : 12.0f;
    const float zoom = argc > 6 ? (float)std::atof(argv[6]) : 1.0f;
    const float fx = argc > 9 ? (float)std::atof(argv[7]) : 0.5f, fy = argc > 9 ? (float)std::atof(argv[8]) : 0.5f, fz = argc > 9 ? (float)std::atof(argv[9]) : 0.5f;

    ID3D11Device* dev = nullptr; ID3D11DeviceContext* ctx = nullptr;
    D3D_FEATURE_LEVEL fl;
    if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &dev, &fl, &ctx)) &&
        FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &dev, &fl, &ctx))) {
        std::printf("no D3D11 device\n"); return 1;
    }
    gpu::setDevice(dev, ctx);
    if (!gpu::ready()) { std::printf("renderer init failed\n"); return 1; }

    EngineParams p = makePreset(preset);
    for (int i = 10; i < argc; ++i) {   // (key=value: inlet=0/1/2)
        const char* eq = std::strchr(argv[i], '=');
        if (!eq) continue;
        const std::string k(argv[i], eq - argv[i]);
        if (k == "inlet") p.tsInlet = std::atoi(eq + 1);
    }
    float ph[kMaxCyl]; computePhases(p, ph);
    EngineViewLive live;
    live.egtGas = 700; live.advance = 20; live.throttle = 0.6f;
    EngineModel model;
    buildEngineModel(p, ph, crankDeg, live, 0.16f, model);
    if (model.items.empty()) { std::printf("empty model\n"); return 1; }

    const int w = 1400, h = 1000;
    const float fov = 30.0f * 3.14159265f / 180.0f, aspect = (float)w / h, t = std::tan(0.5f * fov);
    const float yaw = yawD * 3.14159265f / 180.0f, pitch = pitchD * 3.14159265f / 180.0f;
    const m3::V3 dir = m3::v3(std::cos(pitch) * std::sin(yaw), std::sin(pitch), -std::cos(pitch) * std::cos(yaw));
    // fit the whole model as the app does, then zoom in on the chosen point
    m3::V3 tc = (model.lo + model.hi) * 0.5f;
    const float rad = 0.5f * m3::len(model.hi - model.lo);
    float D = rad / std::sin(0.5f * fov * std::min(1.0f, aspect));
    const std::vector<m3::V3>& pts = model.hull;
    for (int pass = 0; pass < 2; ++pass) {
        const m3::V3 e0 = tc + dir * D;
        const m3::M4 v0 = m3::lookAt(e0, tc, m3::v3(0, 1, 0));
        float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
        for (const m3::V3& q0 : pts) {
            const m3::V3 q = m3::xform(v0, q0);
            const float zc = std::max(1e-3f, -q.z);
            x0 = std::min(x0, q.x / (zc * t * aspect)); x1 = std::max(x1, q.x / (zc * t * aspect));
            y0 = std::min(y0, q.y / (zc * t)); y1 = std::max(y1, q.y / (zc * t));
        }
        const m3::V3 fwd = m3::norm(tc - e0), right = m3::norm(m3::cross(fwd, m3::v3(0, 1, 0))), up = m3::cross(right, fwd);
        tc = tc + right * (0.5f * (x0 + x1) * D * t * aspect) + up * (0.5f * (y0 + y1) * D * t);
        const float ext = std::max(0.5f * (x1 - x0), 0.5f * (y1 - y0));
        if (ext > 1e-3f) D *= std::min(2.0f, std::max(0.2f, ext / 0.88f));
    }
    if (argc > 9) tc = m3::v3(model.lo.x + fx * (model.hi.x - model.lo.x), model.lo.y + fy * (model.hi.y - model.lo.y), model.lo.z + fz * (model.hi.z - model.lo.z));
    D /= std::max(0.05f, zoom);
    const m3::V3 eye = tc + dir * D;
    const m3::M4 view = m3::lookAt(eye, tc, m3::v3(0, 1, 0));
    const float dc = m3::len(eye - tc);
    const m3::M4 proj = m3::perspective(fov, aspect, std::max(1.0f, dc - rad * 1.6f), dc + rad * 1.8f);
    const float bg[3] = {0.065f, 0.067f, 0.075f};
    if (!gpu::begin(w, h, bg, view, proj, eye)) { std::printf("begin failed\n"); return 1; }
    for (const ModelItem& it : model.items) gpu::draw(it.mesh, it.world, it.mat);
    for (const ModelGlow& g : model.glows) gpu::glow(g.p, g.size, g.r, g.g, g.b, g.a);
    gpu::end();
    std::vector<uint8_t> px; int rw = 0, rh = 0;
    if (!gpu::readPixels(px, rw, rh) || !writeBmp(argv[2], px, rw, rh)) { std::printf("readback failed\n"); return 1; }
    std::printf("%s: %d items, box %.0f..%.0f x %.0f..%.0f y %.0f..%.0f z mm\n", p.name, (int)model.items.size(), model.lo.x, model.hi.x,
                model.lo.y, model.hi.y, model.lo.z, model.hi.z);
    gpu::shutdown();
    ctx->Release(); dev->Release();
    return 0;
}
