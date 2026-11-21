#pragma once
// Small 3D math for the engine view: vectors and 4x4 matrices (row-major storage, column vectors: p' = M p).
#include <cmath>

namespace es::ui::m3 {

struct V3 { float x = 0, y = 0, z = 0; };
inline V3 v3(float x, float y, float z) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
inline V3 operator+(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline V3 operator-(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline V3 operator-(V3 a) { return v3(-a.x, -a.y, -a.z); }
inline V3 operator*(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
inline float len(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 norm(V3 a) { const float l = len(a); return l > 1e-12f ? a * (1.0f / l) : v3(0, 1, 0); }
inline V3 lerp(V3 a, V3 b, float t) { return a + (b - a) * t; }

struct M4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    float& at(int r, int c) { return m[r * 4 + c]; }
    float at(int r, int c) const { return m[r * 4 + c]; }
};
inline M4 operator*(const M4& a, const M4& b) {
    M4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0;
            for (int k = 0; k < 4; ++k) s += a.at(i, k) * b.at(k, j);
            r.at(i, j) = s;
        }
    return r;
}
inline V3 xform(const M4& a, V3 p) {   // point
    return v3(a.at(0, 0) * p.x + a.at(0, 1) * p.y + a.at(0, 2) * p.z + a.at(0, 3),
              a.at(1, 0) * p.x + a.at(1, 1) * p.y + a.at(1, 2) * p.z + a.at(1, 3),
              a.at(2, 0) * p.x + a.at(2, 1) * p.y + a.at(2, 2) * p.z + a.at(2, 3));
}
inline V3 xdir(const M4& a, V3 d) {    // direction
    return v3(a.at(0, 0) * d.x + a.at(0, 1) * d.y + a.at(0, 2) * d.z, a.at(1, 0) * d.x + a.at(1, 1) * d.y + a.at(1, 2) * d.z,
              a.at(2, 0) * d.x + a.at(2, 1) * d.y + a.at(2, 2) * d.z);
}
inline M4 translate(V3 t) { M4 r; r.at(0, 3) = t.x; r.at(1, 3) = t.y; r.at(2, 3) = t.z; return r; }
inline M4 scale(V3 s) { M4 r; r.at(0, 0) = s.x; r.at(1, 1) = s.y; r.at(2, 2) = s.z; return r; }
inline M4 rotX(float a) { M4 r; const float c = std::cos(a), s = std::sin(a); r.at(1, 1) = c; r.at(1, 2) = -s; r.at(2, 1) = s; r.at(2, 2) = c; return r; }
inline M4 rotY(float a) { M4 r; const float c = std::cos(a), s = std::sin(a); r.at(0, 0) = c; r.at(0, 2) = s; r.at(2, 0) = -s; r.at(2, 2) = c; return r; }
inline M4 rotZ(float a) { M4 r; const float c = std::cos(a), s = std::sin(a); r.at(0, 0) = c; r.at(0, 1) = -s; r.at(1, 0) = s; r.at(1, 1) = c; return r; }
// a frame whose local +y points along `dir` (any perpendicular roll), at `origin`
inline M4 alongY(V3 origin, V3 dir) {
    const V3 y = norm(dir);
    const V3 ref = std::fabs(y.z) < 0.9f ? v3(0, 0, 1) : v3(1, 0, 0);
    const V3 x = norm(cross(y, ref)), z = cross(x, y);
    M4 r;
    r.at(0, 0) = x.x; r.at(1, 0) = x.y; r.at(2, 0) = x.z;
    r.at(0, 1) = y.x; r.at(1, 1) = y.y; r.at(2, 1) = y.z;
    r.at(0, 2) = z.x; r.at(1, 2) = z.y; r.at(2, 2) = z.z;
    r.at(0, 3) = origin.x; r.at(1, 3) = origin.y; r.at(2, 3) = origin.z;
    return r;
}
// normal matrix: inverse transpose of the upper 3x3 (non-uniform scale safe)
inline M4 normalMatrix(const M4& a) {
    const float a00 = a.at(0, 0), a01 = a.at(0, 1), a02 = a.at(0, 2), a10 = a.at(1, 0), a11 = a.at(1, 1), a12 = a.at(1, 2),
                a20 = a.at(2, 0), a21 = a.at(2, 1), a22 = a.at(2, 2);
    const float c00 = a11 * a22 - a12 * a21, c01 = a12 * a20 - a10 * a22, c02 = a10 * a21 - a11 * a20;
    const float det = a00 * c00 + a01 * c01 + a02 * c02;
    const float id = std::fabs(det) > 1e-20f ? 1.0f / det : 0.0f;
    M4 r;   // inverse transpose = cofactor matrix / det
    r.at(0, 0) = c00 * id; r.at(0, 1) = c01 * id; r.at(0, 2) = c02 * id;
    r.at(1, 0) = (a02 * a21 - a01 * a22) * id; r.at(1, 1) = (a00 * a22 - a02 * a20) * id; r.at(1, 2) = (a01 * a20 - a00 * a21) * id;
    r.at(2, 0) = (a01 * a12 - a02 * a11) * id; r.at(2, 1) = (a02 * a10 - a00 * a12) * id; r.at(2, 2) = (a00 * a11 - a01 * a10) * id;
    return r;
}
inline M4 lookAt(V3 eye, V3 target, V3 up) {   // right-handed view: camera looks down -z
    const V3 f = norm(target - eye), s = norm(cross(f, up)), u = cross(s, f);
    M4 r;
    r.at(0, 0) = s.x; r.at(0, 1) = s.y; r.at(0, 2) = s.z; r.at(0, 3) = -dot(s, eye);
    r.at(1, 0) = u.x; r.at(1, 1) = u.y; r.at(1, 2) = u.z; r.at(1, 3) = -dot(u, eye);
    r.at(2, 0) = -f.x; r.at(2, 1) = -f.y; r.at(2, 2) = -f.z; r.at(2, 3) = dot(f, eye);
    return r;
}
inline M4 perspective(float fovY, float aspect, float zn, float zf) {   // D3D clip depth 0..1
    const float h = 1.0f / std::tan(fovY * 0.5f);
    M4 r;
    r.at(0, 0) = h / aspect; r.at(1, 1) = h;
    r.at(2, 2) = zf / (zn - zf); r.at(2, 3) = zn * zf / (zn - zf);
    r.at(3, 2) = -1; r.at(3, 3) = 0;
    return r;
}

} // namespace es::ui::m3
