#include "ui/Widgets.h"
#include "core/MathUtil.h"
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
        if (p.resonator) { section(xx, y, "RES", seg * 1.1f, 9); section(xx, y, nullptr, seg * 0.5f, 0); }
        const char* mn = p.muffler == Muffler::StraightThrough ? "GLASSPACK" : p.muffler == Muffler::Chambered ? "CHAMBERED"
                       : p.muffler == Muffler::Turbo ? "TURBO" : nullptr;
        if (mn) section(xx, y, mn, seg * 2.2f, 13);
        section(xx, y, nullptr, seg * 1.2f, 0);
        dl->AddTriangleFilled(ImVec2(xx, y - 6), ImVec2(xx, y + 6), ImVec2(xx + 10, y), col(0.9f, 0.5f, 0.2f));
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

} // namespace es::ui
