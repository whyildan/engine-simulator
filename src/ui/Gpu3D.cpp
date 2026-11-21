#include "ui/Gpu3D.h"
#include <d3d11.h>
#include <d3dcompiler.h>
#include <algorithm>
#include <cstring>
#include <vector>

namespace es::ui::gpu {

using namespace m3;

namespace {

ID3D11Device* g_dev = nullptr;
ID3D11DeviceContext* g_ctx = nullptr;
bool g_ok = false;

ID3D11VertexShader* vsMesh = nullptr;
ID3D11PixelShader* psMesh = nullptr;
ID3D11InputLayout* ilMesh = nullptr;
ID3D11VertexShader* vsGlow = nullptr;
ID3D11PixelShader* psGlow = nullptr;
ID3D11InputLayout* ilGlow = nullptr;
ID3D11Buffer* cbFrame = nullptr;
ID3D11Buffer* cbObj = nullptr;
ID3D11Buffer* vbGlow = nullptr;
int glowCap = 0;
ID3D11RasterizerState* rsSolid = nullptr;
ID3D11DepthStencilState* dsWrite = nullptr;
ID3D11DepthStencilState* dsRead = nullptr;
ID3D11BlendState* bsOpaque = nullptr;
ID3D11BlendState* bsAlpha = nullptr;
ID3D11BlendState* bsAdd = nullptr;

// render targets
int tw = 0, th = 0, tSamples = 1;
ID3D11Texture2D* texMs = nullptr;
ID3D11RenderTargetView* rtvMs = nullptr;
ID3D11Texture2D* texDepth = nullptr;
ID3D11DepthStencilView* dsv = nullptr;
ID3D11Texture2D* texOut = nullptr;
ID3D11ShaderResourceView* srvOut = nullptr;

struct Mesh { ID3D11Buffer* vb = nullptr; ID3D11Buffer* ib = nullptr; UINT count = 0; V3 center; };
std::vector<Mesh> meshes;

struct FrameCB { M4 viewProj; float eye[4]; float key[4]; float fill[4]; float rim[4]; };
struct ObjCB { M4 world; M4 nrm; float base[4]; float mat[4]; float emit[4]; };
struct Draw { int mesh; M4 world; Material m; float dist; };
std::vector<Draw> solid, glass;
struct GlowV { float x, y, z, u, v, r, g, b, a; };
struct GlowItem { V3 p; float size, r, g, b, a; };
std::vector<GlowItem> glows;
M4 gView, gProj, gViewProj;
V3 gEye;
float gBg[3] = {0, 0, 0};

const char* kShader = R"(
cbuffer Frame : register(b0) { row_major float4x4 viewProj; float4 eye; float4 keyDir; float4 fillDir; float4 rimDir; };
cbuffer Obj : register(b1) { row_major float4x4 world; row_major float4x4 nrm; float4 base; float4 mat; float4 emit; };
struct VSIn { float3 p : POSITION; float3 n : NORMAL; };
struct PSIn { float4 pos : SV_POSITION; float3 wp : TEXCOORD0; float3 wn : TEXCOORD1; };
PSIn VS(VSIn i) {
    PSIn o;
    float4 w = mul(world, float4(i.p, 1));
    o.wp = w.xyz;
    o.wn = mul((float3x3)nrm, i.n);
    o.pos = mul(viewProj, w);
    return o;
}
// a neutral studio around the engine: a soft light overhead, a dark floor, a warm soft box toward the key light
float3 env(float3 r) {
    float t = r.y;
    float3 top = float3(0.95, 0.96, 1.00), hor = float3(0.42, 0.43, 0.46), bot = float3(0.06, 0.06, 0.07);
    float3 c = t > 0 ? lerp(hor, top, pow(saturate(t), 0.55)) : lerp(hor, bot, pow(saturate(-t), 0.45));
    c += float3(1.0, 0.86, 0.72) * pow(saturate(dot(r, -keyDir.xyz)), 18) * 1.6;
    c += float3(0.75, 0.80, 0.90) * pow(saturate(dot(r, -fillDir.xyz)), 30) * 0.6;
    return c;
}
float4 PS(PSIn i) : SV_TARGET {
    float3 N = normalize(i.wn);
    float3 V = normalize(eye.xyz - i.wp);
    if (dot(N, V) < 0) N = -N;   // two-sided
    float metal = mat.x, rough = mat.y;
    float3 albedo = pow(base.rgb, 2.2);   // the palette is given in display (sRGB-like) values: light in linear terms
    float3 L1 = -keyDir.xyz, L2 = -fillDir.xyz, L3 = -rimDir.xyz;
    float d1 = saturate(dot(N, L1)), d2 = saturate(dot(N, L2)), d3 = saturate(dot(N, L3));
    float shin = exp2(10.0 * (1.0 - rough) + 1.0);
    float norm = (shin + 8.0) / 25.1;   // energy-normalised Blinn-Phong
    float s1 = pow(saturate(dot(N, normalize(L1 + V))), shin) * norm * d1;
    float s2 = pow(saturate(dot(N, normalize(L2 + V))), shin) * norm * d2;
    float s3 = pow(saturate(dot(N, normalize(L3 + V))), shin) * norm * d3;
    float nv = saturate(dot(N, V));
    float fres = pow(1.0 - nv, 5.0);
    float3 f0 = lerp(float3(0.04, 0.04, 0.04), albedo, metal);
    float3 F = f0 + (1.0 - f0) * fres * (1.0 - rough * 0.7);
    float3 amb = lerp(float3(0.10, 0.10, 0.11), float3(0.34, 0.345, 0.36), N.y * 0.5 + 0.5);
    float3 diff = albedo * (1.0 - metal * 0.45) * (amb + d1 * float3(1.0, 0.95, 0.88) * 1.25 + d2 * float3(0.60, 0.66, 0.78) * 0.55 + d3 * 0.35);
    float3 envc = env(reflect(-V, N)) * lerp(1.0, 0.35, rough);
    float3 spec = F * envc * (0.30 + 0.80 * metal) + (s1 * float3(1.0, 0.95, 0.88) * 2.2 + s2 * float3(0.6, 0.66, 0.78) * 0.9 + s3 * 1.2) * F;
    float3 c = diff + spec + pow(max(emit.rgb, 0), 2.2) * 2.0;
    float a = base.a;
    if (mat.w > 0.5) a = saturate(base.a + fres * 0.22 + (s1 + s2) * 0.15);
    c = c / (1.0 + c * 0.22);             // roll-off of the highlights
    c = pow(saturate(c), 1.0 / 2.2);      // to display values
    return float4(c, a);
}
struct GIn { float3 p : POSITION; float2 uv : TEXCOORD0; float4 c : COLOR0; };
struct GOut { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; float4 c : COLOR0; };
GOut VSG(GIn i) { GOut o; o.pos = mul(viewProj, float4(i.p, 1)); o.uv = i.uv; o.c = i.c; return o; }
float4 PSG(GOut i) : SV_TARGET {
    float r2 = dot(i.uv, i.uv);
    float k = saturate(1.0 - r2);
    k = k * k * (0.35 + 0.65 * k);
    return float4(i.c.rgb * k * i.c.a, 0);
}
)";

template <class T> void rel(T*& p) { if (p) { p->Release(); p = nullptr; } }

void releaseTargets() { rel(srvOut); rel(texOut); rel(dsv); rel(texDepth); rel(rtvMs); rel(texMs); tw = th = 0; }

bool makeTargets(int w, int h) {
    releaseTargets();
    UINT q = 0;
    tSamples = 4;
    if (FAILED(g_dev->CheckMultisampleQualityLevels(DXGI_FORMAT_R8G8B8A8_UNORM, 4, &q)) || q == 0) tSamples = 1;
    D3D11_TEXTURE2D_DESC td = {};
    td.Width = w; td.Height = h; td.MipLevels = 1; td.ArraySize = 1; td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    td.SampleDesc.Count = tSamples; td.Usage = D3D11_USAGE_DEFAULT; td.BindFlags = D3D11_BIND_RENDER_TARGET;
    if (FAILED(g_dev->CreateTexture2D(&td, nullptr, &texMs))) return false;
    if (FAILED(g_dev->CreateRenderTargetView(texMs, nullptr, &rtvMs))) return false;
    td.Format = DXGI_FORMAT_D24_UNORM_S8_UINT; td.BindFlags = D3D11_BIND_DEPTH_STENCIL;
    if (FAILED(g_dev->CreateTexture2D(&td, nullptr, &texDepth))) return false;
    if (FAILED(g_dev->CreateDepthStencilView(texDepth, nullptr, &dsv))) return false;
    td.Format = DXGI_FORMAT_R8G8B8A8_UNORM; td.SampleDesc.Count = 1; td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    if (FAILED(g_dev->CreateTexture2D(&td, nullptr, &texOut))) return false;
    if (FAILED(g_dev->CreateShaderResourceView(texOut, nullptr, &srvOut))) return false;
    tw = w; th = h;
    return true;
}

bool compile(const char* entry, const char* target, ID3DBlob** out) {
    ID3DBlob* err = nullptr;
    const HRESULT hr = D3DCompile(kShader, std::strlen(kShader), "engine3d", nullptr, nullptr, entry, target, D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, out, &err);
    if (err) { OutputDebugStringA((const char*)err->GetBufferPointer()); err->Release(); }
    return SUCCEEDED(hr);
}

bool init() {
    ID3DBlob *vb = nullptr, *pb = nullptr, *gvb = nullptr, *gpb = nullptr;
    if (!compile("VS", "vs_4_0", &vb) || !compile("PS", "ps_4_0", &pb) || !compile("VSG", "vs_4_0", &gvb) || !compile("PSG", "ps_4_0", &gpb)) {
        rel(vb); rel(pb); rel(gvb); rel(gpb);
        return false;
    }
    bool ok = SUCCEEDED(g_dev->CreateVertexShader(vb->GetBufferPointer(), vb->GetBufferSize(), nullptr, &vsMesh)) &&
              SUCCEEDED(g_dev->CreatePixelShader(pb->GetBufferPointer(), pb->GetBufferSize(), nullptr, &psMesh)) &&
              SUCCEEDED(g_dev->CreateVertexShader(gvb->GetBufferPointer(), gvb->GetBufferSize(), nullptr, &vsGlow)) &&
              SUCCEEDED(g_dev->CreatePixelShader(gpb->GetBufferPointer(), gpb->GetBufferSize(), nullptr, &psGlow));
    if (ok) {
        D3D11_INPUT_ELEMENT_DESC il[] = {{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
                                         {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0}};
        ok = SUCCEEDED(g_dev->CreateInputLayout(il, 2, vb->GetBufferPointer(), vb->GetBufferSize(), &ilMesh));
        D3D11_INPUT_ELEMENT_DESC gl[] = {{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
                                         {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
                                         {"COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 20, D3D11_INPUT_PER_VERTEX_DATA, 0}};
        ok = ok && SUCCEEDED(g_dev->CreateInputLayout(gl, 3, gvb->GetBufferPointer(), gvb->GetBufferSize(), &ilGlow));
    }
    rel(vb); rel(pb); rel(gvb); rel(gpb);
    if (!ok) return false;
    D3D11_BUFFER_DESC bd = {};
    bd.Usage = D3D11_USAGE_DYNAMIC; bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER; bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    bd.ByteWidth = (sizeof(FrameCB) + 15) & ~15u;
    if (FAILED(g_dev->CreateBuffer(&bd, nullptr, &cbFrame))) return false;
    bd.ByteWidth = (sizeof(ObjCB) + 15) & ~15u;
    if (FAILED(g_dev->CreateBuffer(&bd, nullptr, &cbObj))) return false;
    D3D11_RASTERIZER_DESC rd = {};
    rd.FillMode = D3D11_FILL_SOLID; rd.CullMode = D3D11_CULL_NONE; rd.DepthClipEnable = TRUE; rd.MultisampleEnable = TRUE; rd.AntialiasedLineEnable = TRUE;
    if (FAILED(g_dev->CreateRasterizerState(&rd, &rsSolid))) return false;
    D3D11_DEPTH_STENCIL_DESC dd = {};
    dd.DepthEnable = TRUE; dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL; dd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
    if (FAILED(g_dev->CreateDepthStencilState(&dd, &dsWrite))) return false;
    dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    if (FAILED(g_dev->CreateDepthStencilState(&dd, &dsRead))) return false;
    D3D11_BLEND_DESC bl = {};
    bl.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
    if (FAILED(g_dev->CreateBlendState(&bl, &bsOpaque))) return false;
    bl.RenderTarget[0].BlendEnable = TRUE;
    bl.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA; bl.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA; bl.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    bl.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE; bl.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA; bl.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    if (FAILED(g_dev->CreateBlendState(&bl, &bsAlpha))) return false;
    bl.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE; bl.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;
    bl.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ZERO; bl.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
    if (FAILED(g_dev->CreateBlendState(&bl, &bsAdd))) return false;
    return true;
}

void setObj(const M4& world, const Material& m) {
    D3D11_MAPPED_SUBRESOURCE ms;
    if (FAILED(g_ctx->Map(cbObj, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms))) return;
    ObjCB* o = (ObjCB*)ms.pData;
    o->world = world; o->nrm = normalMatrix(world);
    o->base[0] = m.r; o->base[1] = m.g; o->base[2] = m.b; o->base[3] = m.a;
    o->mat[0] = m.metal; o->mat[1] = m.rough; o->mat[2] = 0; o->mat[3] = m.glass ? 1.0f : 0.0f;
    o->emit[0] = m.er; o->emit[1] = m.eg; o->emit[2] = m.eb; o->emit[3] = 0;
    g_ctx->Unmap(cbObj, 0);
}

void drawMesh(const Draw& d) {
    const Mesh& mh = meshes[d.mesh];
    setObj(d.world, d.m);
    const UINT stride = sizeof(mesh::Vtx), off = 0;
    g_ctx->IASetVertexBuffers(0, 1, &mh.vb, &stride, &off);
    g_ctx->IASetIndexBuffer(mh.ib, DXGI_FORMAT_R32_UINT, 0);
    g_ctx->DrawIndexed(mh.count, 0, 0);
}

} // namespace

void setDevice(ID3D11Device* dev, ID3D11DeviceContext* ctx) {
    g_dev = dev; g_ctx = ctx;
    g_ok = dev && ctx && init();
}

void shutdown() {
    meshClear();
    releaseTargets();
    rel(vsMesh); rel(psMesh); rel(ilMesh); rel(vsGlow); rel(psGlow); rel(ilGlow); rel(cbFrame); rel(cbObj); rel(vbGlow);
    rel(rsSolid); rel(dsWrite); rel(dsRead); rel(bsOpaque); rel(bsAlpha); rel(bsAdd);
    g_ok = false;
}

bool ready() { return g_ok; }

int meshCreate(const mesh::Data& d) {
    if (!g_ok || d.v.empty() || d.i.empty()) return -1;
    Mesh m;
    D3D11_BUFFER_DESC bd = {};
    bd.Usage = D3D11_USAGE_IMMUTABLE; bd.BindFlags = D3D11_BIND_VERTEX_BUFFER; bd.ByteWidth = (UINT)(d.v.size() * sizeof(mesh::Vtx));
    D3D11_SUBRESOURCE_DATA sd = {}; sd.pSysMem = d.v.data();
    if (FAILED(g_dev->CreateBuffer(&bd, &sd, &m.vb))) return -1;
    bd.BindFlags = D3D11_BIND_INDEX_BUFFER; bd.ByteWidth = (UINT)(d.i.size() * sizeof(uint32_t)); sd.pSysMem = d.i.data();
    if (FAILED(g_dev->CreateBuffer(&bd, &sd, &m.ib))) { rel(m.vb); return -1; }
    m.count = (UINT)d.i.size();
    V3 lo = v3(1e9f, 1e9f, 1e9f), hi = v3(-1e9f, -1e9f, -1e9f);
    for (const auto& q : d.v) { lo = v3(std::min(lo.x, q.px), std::min(lo.y, q.py), std::min(lo.z, q.pz)); hi = v3(std::max(hi.x, q.px), std::max(hi.y, q.py), std::max(hi.z, q.pz)); }
    m.center = (lo + hi) * 0.5f;
    meshes.push_back(m);
    return (int)meshes.size() - 1;
}

void meshClear() {
    for (Mesh& m : meshes) { rel(m.vb); rel(m.ib); }
    meshes.clear();
}

int meshCount() { return (int)meshes.size(); }

bool begin(int w, int h, const float bg[3], const M4& view, const M4& proj, V3 eye) {
    if (!g_ok || w < 8 || h < 8) return false;
    if ((w != tw || h != th) && !makeTargets(w, h)) { releaseTargets(); return false; }
    gView = view; gProj = proj; gViewProj = proj * view; gEye = eye;
    std::memcpy(gBg, bg, sizeof(gBg));
    solid.clear(); glass.clear(); glows.clear();
    return true;
}

void draw(int mesh, const M4& world, const Material& m) {
    if (mesh < 0 || mesh >= (int)meshes.size()) return;
    Draw d{mesh, world, m, 0};
    if (m.glass || m.a < 0.999f) {
        const V3 c = xform(world, meshes[mesh].center);
        d.dist = len(c - gEye);
        glass.push_back(d);
    } else solid.push_back(d);
}

void glow(V3 pos, float size, float r, float g, float b, float a) { glows.push_back({pos, size, r, g, b, a}); }

void end() {
    if (!g_ok || !rtvMs) return;
    const float clear[4] = {gBg[0], gBg[1], gBg[2], 1.0f};
    g_ctx->OMSetRenderTargets(1, &rtvMs, dsv);
    g_ctx->ClearRenderTargetView(rtvMs, clear);
    g_ctx->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH, 1.0f, 0);
    D3D11_VIEWPORT vp = {0, 0, (float)tw, (float)th, 0, 1};
    g_ctx->RSSetViewports(1, &vp);
    g_ctx->RSSetState(rsSolid);
    {
        D3D11_MAPPED_SUBRESOURCE ms;
        if (SUCCEEDED(g_ctx->Map(cbFrame, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms))) {
            FrameCB* f = (FrameCB*)ms.pData;
            f->viewProj = gViewProj;
            f->eye[0] = gEye.x; f->eye[1] = gEye.y; f->eye[2] = gEye.z; f->eye[3] = 1;
            // lights in view space terms: key from the upper left behind the viewer, fill from the right, rim from behind
            const M4 iv = gView;   // view rows are the camera axes in world space
            const V3 right = v3(iv.at(0, 0), iv.at(0, 1), iv.at(0, 2)), up = v3(iv.at(1, 0), iv.at(1, 1), iv.at(1, 2)),
                     back = v3(iv.at(2, 0), iv.at(2, 1), iv.at(2, 2));
            const V3 key = norm(right * 0.55f - up * 0.75f - back * 0.55f) ;   // light travels toward -this... (direction of travel)
            const V3 fill = norm(right * -0.8f - up * 0.2f - back * 0.5f);
            const V3 rim = norm(up * -0.3f + back * 0.9f);
            const V3 k2 = v3(0, 0, 0) - key, f2 = v3(0, 0, 0) - fill, r2 = v3(0, 0, 0) - rim;
            f->key[0] = -k2.x; f->key[1] = -k2.y; f->key[2] = -k2.z; f->key[3] = 0;
            f->fill[0] = -f2.x; f->fill[1] = -f2.y; f->fill[2] = -f2.z; f->fill[3] = 0;
            f->rim[0] = -r2.x; f->rim[1] = -r2.y; f->rim[2] = -r2.z; f->rim[3] = 0;
            g_ctx->Unmap(cbFrame, 0);
        }
    }
    g_ctx->VSSetConstantBuffers(0, 1, &cbFrame);
    g_ctx->VSSetConstantBuffers(1, 1, &cbObj);
    g_ctx->PSSetConstantBuffers(0, 1, &cbFrame);
    g_ctx->PSSetConstantBuffers(1, 1, &cbObj);
    g_ctx->IASetInputLayout(ilMesh);
    g_ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    g_ctx->VSSetShader(vsMesh, nullptr, 0);
    g_ctx->PSSetShader(psMesh, nullptr, 0);
    g_ctx->GSSetShader(nullptr, nullptr, 0);
    const float bf[4] = {0, 0, 0, 0};
    // solid parts
    g_ctx->OMSetBlendState(bsOpaque, bf, 0xffffffff);
    g_ctx->OMSetDepthStencilState(dsWrite, 0);
    for (const Draw& d : solid) drawMesh(d);
    // glass and gas, far to near
    std::sort(glass.begin(), glass.end(), [](const Draw& a, const Draw& b) { return a.dist > b.dist; });
    g_ctx->OMSetBlendState(bsAlpha, bf, 0xffffffff);
    g_ctx->OMSetDepthStencilState(dsRead, 0);
    for (const Draw& d : glass) drawMesh(d);
    // glows: camera-facing quads, added
    if (!glows.empty()) {
        const int need = (int)glows.size() * 6;
        if (need > glowCap) {
            rel(vbGlow);
            glowCap = std::max(need, 6 * 1024);
            D3D11_BUFFER_DESC bd = {};
            bd.Usage = D3D11_USAGE_DYNAMIC; bd.BindFlags = D3D11_BIND_VERTEX_BUFFER; bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            bd.ByteWidth = (UINT)(glowCap * sizeof(GlowV));
            if (FAILED(g_dev->CreateBuffer(&bd, nullptr, &vbGlow))) { glowCap = 0; }
        }
        D3D11_MAPPED_SUBRESOURCE ms;
        if (vbGlow && SUCCEEDED(g_ctx->Map(vbGlow, 0, D3D11_MAP_WRITE_DISCARD, 0, &ms))) {
            GlowV* o = (GlowV*)ms.pData;
            const V3 right = v3(gView.at(0, 0), gView.at(0, 1), gView.at(0, 2)), up = v3(gView.at(1, 0), gView.at(1, 1), gView.at(1, 2));
            for (const GlowItem& gI : glows) {
                const V3 a = gI.p + (-right - up) * gI.size, b = gI.p + (right - up) * gI.size, c = gI.p + (right + up) * gI.size, e = gI.p + (-right + up) * gI.size;
                const GlowV q[6] = {{a.x, a.y, a.z, -1, -1, gI.r, gI.g, gI.b, gI.a}, {b.x, b.y, b.z, 1, -1, gI.r, gI.g, gI.b, gI.a},
                                    {c.x, c.y, c.z, 1, 1, gI.r, gI.g, gI.b, gI.a},   {a.x, a.y, a.z, -1, -1, gI.r, gI.g, gI.b, gI.a},
                                    {c.x, c.y, c.z, 1, 1, gI.r, gI.g, gI.b, gI.a},   {e.x, e.y, e.z, -1, 1, gI.r, gI.g, gI.b, gI.a}};
                std::memcpy(o, q, sizeof(q));
                o += 6;
            }
            g_ctx->Unmap(vbGlow, 0);
            const UINT stride = sizeof(GlowV), off = 0;
            g_ctx->IASetInputLayout(ilGlow);
            g_ctx->IASetVertexBuffers(0, 1, &vbGlow, &stride, &off);
            g_ctx->VSSetShader(vsGlow, nullptr, 0);
            g_ctx->PSSetShader(psGlow, nullptr, 0);
            g_ctx->OMSetBlendState(bsAdd, bf, 0xffffffff);
            g_ctx->Draw((UINT)need, 0);
        }
    }
    // resolve for ImGui
    ID3D11RenderTargetView* none = nullptr;
    g_ctx->OMSetRenderTargets(1, &none, nullptr);
    if (tSamples > 1) g_ctx->ResolveSubresource(texOut, 0, texMs, 0, DXGI_FORMAT_R8G8B8A8_UNORM);
    else g_ctx->CopyResource(texOut, texMs);
}

void* texture() { return srvOut; }

} // namespace es::ui::gpu
