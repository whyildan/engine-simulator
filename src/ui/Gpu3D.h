#pragma once
// GPU 3D renderer for the engine view (Direct3D 11, the app's own device): draws into an offscreen multisampled
// target with a depth buffer and hands the resolved image to ImGui as a texture.
#include "ui/Math3D.h"
#include "ui/Mesh3D.h"
#include <cstdint>
#include <vector>

struct ID3D11Device;
struct ID3D11DeviceContext;

namespace es::ui::gpu {

using m3::V3;
using m3::M4;

// set once by main() after the device exists (nullptr = no GPU view: the flat view is used)
void setDevice(ID3D11Device* dev, ID3D11DeviceContext* ctx);
void shutdown();
bool ready();

struct Material {
    float r = 0.7f, g = 0.7f, b = 0.72f, a = 1.0f;
    float metal = 0.0f, rough = 0.5f;
    float er = 0, eg = 0, eb = 0;   // emission (added light)
    bool glass = false;             // transparent, more opaque at grazing angles; drawn after the solid parts
};

int meshCreate(const mesh::Data& d);   // id, -1 on failure
void meshClear();                       // drop every mesh (they are rebuilt on demand)
int meshCount();

// one frame: begin, draw / glow, end; then texture() is the image (valid until the next begin)
bool begin(int w, int h, const float bg[3], const M4& view, const M4& proj, V3 eye);
void draw(int mesh, const M4& world, const Material& m);
void glow(V3 pos, float size, float r, float g, float b, float a);   // additive soft dot (flames, sparks, particles)
void end();
void* texture();
// the last frame's pixels (tools: offscreen renders)
bool readPixels(std::vector<uint8_t>& rgba, int& w, int& h);

} // namespace es::ui::gpu
