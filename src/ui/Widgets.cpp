#include "ui/Widgets.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace es::ui {

static ImU32 col(float r, float g, float b, float a = 1) { return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, a)); }

void Help(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void Gauge(const char* id, float value, float vmin, float vmax, float redFrom, const char* label,
           const char* fmt, float size, int majorTicks, float tickScale) {
    ImGui::PushID(id);
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("g", ImVec2(size, size));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 c(p0.x + size * 0.5f, p0.y + size * 0.5f);
    float R = size * 0.46f;
    const float a0 = (float)(kPi * 0.75), a1 = (float)(kPi * 2.25);
    auto ang = [&](float v) { return a0 + (a1 - a0) * clampv((v - vmin) / (vmax - vmin), 0.f, 1.f); };

    dl->AddCircleFilled(c, R * 1.04f, col(0.07f, 0.07f, 0.08f), 64);
    dl->AddCircle(c, R * 1.04f, col(0.35f, 0.35f, 0.38f), 64, 2.0f);
    // red zone
    if (redFrom < vmax) {
        dl->PathArcTo(c, R * 0.93f, ang(redFrom), a1, 32);
        dl->PathStroke(col(0.85f, 0.12f, 0.1f), 0, R * 0.08f);
    }
    // ticks
    for (int i = 0; i <= majorTicks * 5; ++i) {
        float v = vmin + (vmax - vmin) * i / (majorTicks * 5.0f);
        float a = ang(v);
        bool major = i % 5 == 0;
        float r0 = major ? R * 0.80f : R * 0.87f;
        ImVec2 d(std::cos(a), std::sin(a));
        dl->AddLine(ImVec2(c.x + d.x * r0, c.y + d.y * r0), ImVec2(c.x + d.x * R * 0.95f, c.y + d.y * R * 0.95f),
                    v >= redFrom ? col(1, 0.3f, 0.25f) : col(0.85f, 0.85f, 0.85f), major ? 2.5f : 1.0f);
        if (major) {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%g", std::round(v * tickScale * 10) / 10);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            ImVec2 tp(c.x + d.x * R * 0.66f - ts.x * 0.5f, c.y + d.y * R * 0.66f - ts.y * 0.5f);
            dl->AddText(tp, col(0.9f, 0.9f, 0.9f), buf);
        }
    }
    // label + value
    ImVec2 ls = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(c.x - ls.x * 0.5f, c.y - R * 0.30f), col(0.6f, 0.6f, 0.65f), label);
    char vb[32];
    std::snprintf(vb, sizeof(vb), fmt, value);
    ImVec2 vs = ImGui::CalcTextSize(vb);
    dl->AddText(ImVec2(c.x - vs.x * 0.5f, c.y + R * 0.52f), col(1, 0.85f, 0.3f), vb);
    // needle
    float a = ang(value);
    ImVec2 d(std::cos(a), std::sin(a));
    dl->AddLine(ImVec2(c.x - d.x * R * 0.12f, c.y - d.y * R * 0.12f), ImVec2(c.x + d.x * R * 0.9f, c.y + d.y * R * 0.9f),
                col(1, 0.25f, 0.15f), 3.0f);
    dl->AddCircleFilled(c, R * 0.07f, col(0.75f, 0.75f, 0.78f));
    ImGui::PopID();
}

void Meter(const char* label, float value, float vmin, float vmax, const char* fmt, ImU32 color, float width) {
    float w = width > 0 ? width : ImGui::GetContentRegionAvail().x;
    float h = ImGui::GetFrameHeight();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(label, ImVec2(w, h));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const char* hid = std::strstr(label, "##");   // "##id" part is not drawn; a hidden label gets no label column
    const char* labelEnd = hid ? hid : label + std::strlen(label);
    float labelW = labelEnd > label ? w * 0.32f : 0.0f;
    dl->AddText(ImVec2(p0.x, p0.y + (h - ImGui::GetFontSize()) * 0.5f), col(0.75f, 0.75f, 0.78f), label, labelEnd);
    ImVec2 b0(p0.x + labelW, p0.y + 2), b1(p0.x + w, p0.y + h - 2);
    dl->AddRectFilled(b0, b1, col(0.12f, 0.12f, 0.14f), 3);
    float t = clampv((value - vmin) / (vmax - vmin), 0.f, 1.f);
    dl->AddRectFilled(b0, ImVec2(b0.x + (b1.x - b0.x) * t, b1.y), color, 3);
    char buf[48];
    std::snprintf(buf, sizeof(buf), fmt, value);
    ImVec2 ts = ImGui::CalcTextSize(buf);
    dl->AddText(ImVec2(b1.x - ts.x - 4, p0.y + (h - ts.y) * 0.5f), col(1, 1, 1), buf);
}

void Lamp(const char* label, bool on, ImU32 color) {
    float h = ImGui::GetFrameHeight();
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 ts = ImGui::CalcTextSize(label);
    ImGui::InvisibleButton(label, ImVec2(h + ts.x + 8, h));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 c(p0.x + h * 0.5f, p0.y + h * 0.5f);
    dl->AddCircleFilled(c, h * 0.32f, on ? color : col(0.18f, 0.18f, 0.2f));
    dl->AddCircle(c, h * 0.32f, col(0.4f, 0.4f, 0.4f));
    dl->AddText(ImVec2(p0.x + h + 2, p0.y + (h - ts.y) * 0.5f), on ? col(1, 1, 1) : col(0.5f, 0.5f, 0.5f), label);
}

// ------------------------------------------------------------------ exhaust schematic
void ExhaustSchematic(const EngineParams& p, ImVec2 size) {
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("exh_schem", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col(0.06f, 0.06f, 0.07f), 4);
    const int n = clampv(p.cylinders, 1, kMaxCyl);
    bool twoBanks = false;
    if (p.layout != Layout::Inline) {
        bool b0 = false, b1 = false;
        for (int i = 0; i < n; ++i) (p.bank[i] ? b1 : b0) = true;
        twoBanks = b0 && b1;
    }
    const ImU32 pipe = col(0.75f, 0.6f, 0.45f);
    const ImU32 box = col(0.35f, 0.36f, 0.4f);
    float x0 = p0.x + 30, w = size.x - 40;
    int banks = twoBanks ? 2 : 1;
    float bandH = size.y / banks;
    float collectorX = x0 + w * 0.28f;
    bool merged = !twoBanks || p.crossover == Crossover::MergeY;
    float outY[2];
    if (p.header == HeaderType::OpenStacks && p.induction != Induction::Turbo && !(isTwoStroke(p) && p.tsPipe)) {
        for (int i = 0, k = 0; i < n; ++i, ++k) {
            const float y = p0.y + size.y * (k + 1) / (n + 1);
            dl->AddRectFilled(ImVec2(x0 - 22, y - 6), ImVec2(x0 - 4, y + 6), col(0.5f, 0.5f, 0.55f), 2);
            char lb[8];
            std::snprintf(lb, sizeof(lb), "%d", i + 1);
            dl->AddText(ImVec2(x0 - 19, y - 7), col(1, 1, 1), lb);
            const float len = (float)clampv(p.primaryLen_mm[i] / 1000.0, 0.1, 1.5), xe = x0 + w * 0.12f + len * w * 0.25f;
            dl->AddLine(ImVec2(x0, y), ImVec2(xe, y), pipe, (float)clampv(p.primaryDia_mm / 12.0, 1.5, 5.0));
            dl->AddTriangleFilled(ImVec2(xe, y - 6), ImVec2(xe, y + 6), ImVec2(xe + 10, y), col(0.9f, 0.5f, 0.2f));
        }
        dl->AddText(ImVec2(x0 + w * 0.62f, p0.y + size.y * 0.5f - 8), col(0.6f, 0.6f, 0.65f), "OPEN STACKS");
        return;
    }
    for (int b = 0; b < banks; ++b) {
        float yMid = p0.y + bandH * (b + 0.5f);
        int cnt = 0;
        for (int i = 0; i < n; ++i) if (!twoBanks || p.bank[i] == b) ++cnt;
        int k = 0;
        for (int i = 0; i < n; ++i) {
            if (twoBanks && p.bank[i] != b) continue;
            float y = p0.y + bandH * b + bandH * (k + 1) / (cnt + 1);
            float len = (float)clampv(p.primaryLen_mm[i] / 1000.0, 0.1, 1.5);
            ImVec2 a(x0, y);
            dl->AddRectFilled(ImVec2(a.x - 22, a.y - 6), ImVec2(a.x - 4, a.y + 6), col(0.5f, 0.5f, 0.55f), 2);
            char lb[8];
            std::snprintf(lb, sizeof(lb), "%d", i + 1);
            dl->AddText(ImVec2(a.x - 19, a.y - 7), col(1, 1, 1), lb);
            // primary drawn with a wiggle proportional to its length
            ImVec2 m((a.x + collectorX) * 0.5f, y + (yMid - y) * 0.3f - (len - 0.6f) * 18.f);
            dl->AddBezierQuadratic(a, m, ImVec2(collectorX, yMid), pipe, (float)clampv(p.primaryDia_mm / 12.0, 1.5, 5.0));
            ++k;
        }
        dl->AddCircleFilled(ImVec2(collectorX, yMid), 6, pipe);
        outY[b] = yMid;
    }
    float x = collectorX;
    float yC = p0.y + size.y * 0.5f;
    float thick = (float)clampv(p.pipeDia_mm / 14.0, 2.0, 6.0);
    auto section = [&](float& xx, float y, const char* name, float len, float h) {
        if (name) {
            dl->AddRectFilled(ImVec2(xx, y - h), ImVec2(xx + len, y + h), box, 4);
            ImVec2 ts = ImGui::CalcTextSize(name);
            dl->AddText(ImVec2(xx + (len - ts.x) * 0.5f, y - ts.y * 0.5f), col(1, 1, 1), name);
        } else {
            dl->AddLine(ImVec2(xx, y), ImVec2(xx + len, y), pipe, thick);
        }
        xx += len;
    };
    float seg = w * 0.07f;
    auto path = [&](float y, float xs) {
        float xx = xs;
        section(xx, y, nullptr, seg, 0);
        if (p.cat) { section(xx, y, "CAT", seg * 1.1f, 11); section(xx, y, nullptr, seg * 0.5f, 0); }
        if (p.resonator) { section(xx, y, p.resonator == 2 ? "HELM" : "RES", seg * 1.1f, 9); section(xx, y, nullptr, seg * 0.5f, 0); }
        static const char* kMn[] = {nullptr, "STRAIGHT", "CHAMBERED", "TURBO", "GLASSPACK", "3-PASS", "VALVED"};
        const char* mn = kMn[clampv((int)p.muffler, 0, 6)];
        auto tip = [&](float yy, float xa) {
            float xb = xa;
            if (mn) section(xb, yy, mn, seg * (p.exhaustSplit ? 1.8f : 2.2f), p.exhaustSplit ? 9.0f : 13.0f);
            section(xb, yy, nullptr, seg * 1.2f, 0);
            dl->AddTriangleFilled(ImVec2(xb, yy - 6), ImVec2(xb, yy + 6), ImVec2(xb + 10, yy), col(0.9f, 0.5f, 0.2f));
        };
        if (p.exhaustSplit) {   // a Y into two silencers and tips
            const float dy = std::min(14.0f, size.y * 0.09f);
            dl->AddLine(ImVec2(xx, y), ImVec2(xx + seg * 0.5f, y - dy), pipe, thick * 0.8f);
            dl->AddLine(ImVec2(xx, y), ImVec2(xx + seg * 0.5f, y + dy), pipe, thick * 0.8f);
            tip(y - dy, xx + seg * 0.5f);
            tip(y + dy, xx + seg * 0.5f);
        } else tip(y, xx);
    };
    if (merged) {
        for (int b = 0; b < banks; ++b) dl->AddLine(ImVec2(x, outY[b]), ImVec2(x + seg, yC), pipe, thick);
        path(yC, x + seg);
    } else {
        float xs = x + seg;
        for (int b = 0; b < 2; ++b) dl->AddLine(ImVec2(x, outY[b]), ImVec2(xs, outY[b]), pipe, thick);
        if (p.crossover == Crossover::HPipe) dl->AddLine(ImVec2(xs - seg * 0.4f, outY[0]), ImVec2(xs - seg * 0.4f, outY[1]), pipe, thick);
        if (p.crossover == Crossover::XPipe) {
            dl->AddLine(ImVec2(xs - seg, outY[0]), ImVec2(xs, outY[1]), pipe, thick);
            dl->AddLine(ImVec2(xs - seg, outY[1]), ImVec2(xs, outY[0]), pipe, thick);
        }
        for (int b = 0; b < 2; ++b) path(outY[b], xs);
    }
}

// ---- step bar: icons drawn with lines (no icon font), in a box of side s centred on c
static void stepIcon(ImDrawList* dl, int icon, ImVec2 c, float s, ImU32 ink) {
    const float t = std::max(1.2f, s * 0.075f), h = 0.5f * s;
    auto P = [&](float x, float y) { return ImVec2(c.x + x * h, c.y + y * h); };   // (-1..1 box)
    switch (icon) {
    case IconBlock:   // an inline-four block: the body, four bores on the deck, the sump
        dl->AddRect(P(-0.95f, -0.35f), P(0.95f, 0.45f), ink, 0.08f * s, 0, t);
        for (int k = 0; k < 4; ++k) {
            const float x = -0.70f + 0.47f * k;
            dl->AddRect(P(x - 0.17f, -0.72f), P(x + 0.17f, -0.35f), ink, 0.04f * s, 0, t);
        }
        dl->AddLine(P(-0.6f, 0.45f), P(-0.45f, 0.75f), ink, t);
        dl->AddLine(P(0.6f, 0.45f), P(0.45f, 0.75f), ink, t);
        dl->AddLine(P(-0.45f, 0.75f), P(0.45f, 0.75f), ink, t);
        break;
    case IconPiston:  // a piston with its rings, the rod down to the big end
        dl->AddRect(P(-0.55f, -0.85f), P(0.55f, 0.05f), ink, 0.1f * s, 0, t);
        dl->AddLine(P(-0.55f, -0.55f), P(0.55f, -0.55f), ink, t * 0.8f);
        dl->AddLine(P(-0.55f, -0.38f), P(0.55f, -0.38f), ink, t * 0.8f);
        dl->AddLine(P(0, -0.15f), P(0, 0.62f), ink, t * 1.4f);
        dl->AddCircle(P(0, 0.72f), 0.22f * h, ink, 16, t);
        break;
    case IconCam:     // a camshaft: the shaft with four lobes
        dl->AddLine(P(-1.0f, 0), P(1.0f, 0), ink, t * 1.2f);
        for (int k = 0; k < 4; ++k) {
            const float x = -0.66f + 0.44f * k, d = (k % 2) ? -1.0f : 1.0f;
            dl->AddEllipse(P(x, 0.12f * d), ImVec2(0.14f * h, 0.42f * h), ink, 0, 16, t);
        }
        break;
    case IconTurbo: { // a turbo: the snail housing (a spiral), the wheel's hub, the outlet
        dl->PathClear();
        for (int k = 0; k <= 28; ++k) {
            const float a = 0.3f + 5.4f * k / 28.0f, r = 0.32f + 0.40f * k / 28.0f;
            dl->PathLineTo(P(r * std::cos(a), r * std::sin(a)));
        }
        dl->PathStroke(ink, 0, t);
        dl->AddCircleFilled(P(0, 0), 0.14f * h, ink, 12);
        dl->AddLine(P(0.72f, -0.10f), P(0.72f, -0.85f), ink, t);
        dl->AddLine(P(0.40f, -0.85f), P(1.0f, -0.85f), ink, t);
        break;
    }
    case IconIntake:  // an intake: the plenum, the throttle body on its end, runners down to the head
        dl->AddRect(P(-0.85f, -0.75f), P(0.55f, -0.30f), ink, 0.08f * s, 0, t);
        dl->AddRect(P(0.55f, -0.66f), P(0.95f, -0.39f), ink, 0, 0, t);
        for (int k = 0; k < 4; ++k) {
            const float x = -0.62f + 0.38f * k;
            dl->AddBezierQuadratic(P(x, -0.30f), P(x, 0.15f), P(x - 0.20f, 0.70f), ink, t);
        }
        break;
    case IconExhaust: // a header: three primaries sweeping into a collector, the pipe out
        for (int k = 0; k < 3; ++k) {
            const float y = -0.70f + 0.45f * k;
            dl->AddBezierQuadratic(P(-0.95f, y), P(-0.15f, y), P(0.20f, 0.10f), ink, t);
        }
        dl->AddLine(P(0.20f, 0.10f), P(0.95f, 0.10f), ink, t * 1.6f);
        dl->AddLine(P(0.80f, -0.10f), P(0.95f, 0.10f), ink, t * 0.8f);
        dl->AddLine(P(0.80f, 0.30f), P(0.95f, 0.10f), ink, t * 0.8f);
        break;
    case IconWheel:   // a wheel: tyre, rim, five spokes
        dl->AddCircle(P(0, 0), 0.92f * h, ink, 28, t * 1.4f);
        dl->AddCircle(P(0, 0), 0.62f * h, ink, 24, t);
        for (int k = 0; k < 5; ++k) {
            const float a = -1.5708f + 1.2566f * k;
            dl->AddLine(P(0.16f * std::cos(a), 0.16f * std::sin(a)), P(0.60f * std::cos(a), 0.60f * std::sin(a)), ink, t);
        }
        dl->AddCircleFilled(P(0, 0), 0.14f * h, ink, 12);
        break;
    case IconSound:   // a speaker and two wave arcs
        dl->AddRectFilled(P(-0.95f, -0.25f), P(-0.60f, 0.25f), ink);
        dl->AddTriangleFilled(P(-0.62f, -0.25f), P(-0.15f, -0.70f), P(-0.15f, 0.25f), ink);
        dl->AddTriangleFilled(P(-0.62f, 0.25f), P(-0.15f, -0.25f), P(-0.15f, 0.70f), ink);
        dl->PathArcTo(P(0.0f, 0), 0.40f * h, -0.9f, 0.9f, 12); dl->PathStroke(ink, 0, t);
        dl->PathArcTo(P(0.0f, 0), 0.78f * h, -0.9f, 0.9f, 16); dl->PathStroke(ink, 0, t);
        break;
    default: break;
    }
}

int StepBar(const char* id, const int* icons, const char* const* labels, int n, int current) {
    ImGui::PushID(id);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const float W = ImGui::GetContentRegionAvail().x, H = ImGui::GetFrameHeight() * 1.55f;
    const float tip = H * 0.32f, gap = 3.0f;
    const float w = (W - tip) / std::max(1, n);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    int clicked = -1;
    for (int i = 0; i < n; ++i) {
        const float x0 = p0.x + w * i, x1 = x0 + w - gap, y0 = p0.y, y1 = p0.y + H, ym = 0.5f * (y0 + y1);
        ImGui::SetCursorScreenPos(ImVec2(x0, y0));
        ImGui::PushID(i);
        if (ImGui::InvisibleButton("##step", ImVec2(w, H))) clicked = i;
        const bool hov = ImGui::IsItemHovered(), sel = i == current;
        if (hov && labels && labels[i]) ImGui::SetTooltip("%s", labels[i]);
        ImGui::PopID();
        // a chevron: notched on the left (but the first), pointed on the right
        ImVec2 pts[6];
        int np = 0;
        pts[np++] = ImVec2(x0, y0);
        pts[np++] = ImVec2(x1, y0);
        pts[np++] = ImVec2(x1 + tip, ym);
        pts[np++] = ImVec2(x1, y1);
        pts[np++] = ImVec2(x0, y1);
        if (i > 0) pts[np++] = ImVec2(x0 + tip, ym);
        const ImU32 fill = sel ? col(0.55f, 0.29f, 0.13f) : hov ? col(0.20f, 0.205f, 0.225f) : col(0.135f, 0.14f, 0.155f);
        dl->AddConcavePolyFilled(pts, np, fill);
        dl->AddPolyline(pts, np, sel ? col(1.0f, 0.62f, 0.25f) : col(0.26f, 0.27f, 0.30f), ImDrawFlags_Closed, sel ? 1.6f : 1.0f);
        const float cx = 0.5f * (x0 + (i > 0 ? tip : 0.0f) + x1 + 0.5f * tip);
        stepIcon(dl, icons[i], ImVec2(cx, ym), H * 0.62f, sel ? col(1.0f, 0.93f, 0.85f) : hov ? col(0.88f, 0.89f, 0.92f) : col(0.66f, 0.68f, 0.72f));
    }
    ImGui::SetCursorScreenPos(ImVec2(p0.x, p0.y + H + ImGui::GetStyle().ItemSpacing.y));
    ImGui::Dummy(ImVec2(W, 0));
    ImGui::PopID();
    return clicked;
}

} // namespace es::ui
