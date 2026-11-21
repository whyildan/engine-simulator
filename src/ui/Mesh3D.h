#pragma once
// Procedural meshes for the engine view: every part is built from the engine's own dimensions.
#include "ui/Math3D.h"
#include <cstdint>
#include <utility>
#include <vector>

namespace es::ui::mesh {

using m3::V3;
using m3::M4;

struct Vtx { float px, py, pz, nx, ny, nz; };
struct Data {
    std::vector<Vtx> v;
    std::vector<uint32_t> i;
    void tri(uint32_t a, uint32_t b, uint32_t c) { i.push_back(a); i.push_back(b); i.push_back(c); }
    uint32_t add(V3 p, V3 n) { v.push_back({p.x, p.y, p.z, n.x, n.y, n.z}); return (uint32_t)v.size() - 1; }
};
using Prof = std::vector<std::pair<float, float>>;   // (radius, height) points, bottom to top
using Poly = std::vector<std::pair<float, float>>;   // 2D outline, counter-clockwise

// surface of revolution about +y: each profile segment is a band, smooth around, crisp along the profile
Data lathe(const Prof& prof, int seg = 40);
// a 2D outline (simple polygon, any convexity) extruded along z from z0 to z1, flat side faces
Data extrude(const Poly& poly, float z0, float z1);
// a tube along a path (radius per point), `sides` around; caps at open ends
Data tube(const std::vector<V3>& path, const std::vector<float>& radius, int sides = 16, bool closed = false, bool caps = true);
Data box(float hx, float hy, float hz);
// an outline extruded along z and twisted by `twist` rad over its length (twin-screw rotors)
Data twisted(const Poly& poly, float z0, float z1, float twist, int slices = 24);
void append(Data& dst, const Data& src, const M4& xf);

// outlines
Poly circleOutline(float r, int n = 32, float cx = 0, float cy = 0);
Poly gearOutline(float rRoot, float rTip, int teeth);
Poly roundedRect(float hx, float hy, float r, int perCorner = 6);

} // namespace es::ui::mesh
