#include "ui/Widgets.h"
#include "core/MathUtil.h"
#include <cmath>
#include <cstdio>

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
    float labelW = w * 0.32f;
    dl->AddText(ImVec2(p0.x, p0.y + (h - ImGui::GetFontSize()) * 0.5f), col(0.75f, 0.75f, 0.78f), label);
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

// ------------------------------------------------------------------ engine animation
static double liftAt(double phi, double center, double dur, double lift) {
    double d = wrapSym(phi - center, 720.0);
    double h = dur * 0.5;
    if (std::fabs(d) >= h) return 0;
    double x = d / h;
    return lift * std::pow(0.5 * (1.0 + std::cos(kPi * x)), 0.75);
}

void EngineAnimation(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, ImVec2 size) {
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("engine_anim", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col(0.06f, 0.06f, 0.07f), 4);
    dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);

    const int n = clampv(p.cylinders, 1, kMaxCyl);
    const bool boxer = p.layout == Layout::Boxer;
    const bool vee = p.layout == Layout::V;
    // columns: cylinders sharing a crank throw position are drawn in the same column
    int bankCount[2] = {0, 0};
    int column[kMaxCyl];
    for (int i = 0; i < n; ++i) {
        int b = (p.layout == Layout::Inline) ? 0 : clampv(p.bank[i], 0, 1);
        column[i] = bankCount[b]++;
    }
    int cols = std::max(bankCount[0], bankCount[1]);
    if (cols < 1) cols = 1;

    double r = p.stroke_mm * 0.5, l = std::max((double)p.rod_mm, r * 1.2), B = p.bore_mm;
    // vertical extent needed (mm): crank r + rod + piston + chamber + head
    double extent = r + l + r + B * 0.9;
    double colW = size.x / cols;
    double scale;
    if (boxer) scale = std::min(colW / (B * 1.5), (size.x * 0.5) / (extent * 1.1));
    else if (vee) scale = std::min(colW / (B * 2.6), size.y / (extent * 1.25));
    else scale = std::min(colW / (B * 1.5), size.y / (extent * 1.15));
    scale = std::min(scale, (double)size.y / (extent * 1.15));
    if (boxer) scale = std::min(scale, (double)size.y / (B * 2.0 * cols));

    const double adv = p.baseAdvance + p.mechAdvance * 0.5;
    const double inC = 360.0 + p.intakeCenterline - p.intakeCamAdvance;
    const double exC = 360.0 - p.exhaustCenterline - p.exhaustCamAdvance;

    for (int i = 0; i < n; ++i) {
        int b = (p.layout == Layout::Inline) ? 0 : clampv(p.bank[i], 0, 1);
        double axisDeg = 0;
        if (vee) axisDeg = (b == 0 ? -1 : 1) * p.bankAngle * 0.5;
        if (boxer) axisDeg = (b == 0 ? -90 : 90);
        ImVec2 crank;
        if (boxer) crank = ImVec2(p0.x + size.x * 0.5f, p0.y + (float)((column[i] + 0.5) * size.y / cols));
        else crank = ImVec2(p0.x + (float)((column[i] + 0.5) * colW), p0.y + size.y - (float)(r * scale) - 8);
        double ax = axisDeg * kDeg;
        ImVec2 up((float)std::sin(ax), (float)-std::cos(ax));    // along cylinder axis (towards head)
        ImVec2 side(-up.y, up.x);

        double phi = wrapPos(crankDeg - phases[i], 720.0);
        double th = wrapPos(phi, 360.0) * kDeg;
        double sn = std::sin(th), cs = std::cos(th);
        double dist = r * cs + std::sqrt(std::max(1.0, l * l - r * r * sn * sn)); // pin distance from crank
        auto P = [&](double along, double across) {
            return ImVec2(crank.x + (float)((up.x * along + side.x * across) * scale),
                          crank.y + (float)((up.y * along + side.y * across) * scale));
        };
        double topDeck = r + l + r * 0.35 + B * 0.2;  // cylinder head deck
        double pistonH = B * 0.55;
        double pinOff = B * 0.25;
        double crown = dist + pinOff;

        // gas colour by stroke
        ImU32 gas = col(0.15f, 0.15f, 0.18f);
        double sinceSpark = wrapPos(phi + adv, 720.0);
        if (sinceSpark < 90) { float t = (float)(1 - sinceSpark / 90); gas = col(1.0f, 0.45f + 0.4f * t, 0.1f, 0.35f + 0.6f * t); }
        else if (phi > 90 && phi < 360) gas = col(0.45f, 0.35f, 0.3f, 0.6f);
        else if (phi >= 360 && phi < 560) gas = col(0.25f, 0.45f, 0.85f, 0.6f);
        else if (phi >= 560) gas = col(0.3f, 0.55f, 0.95f, 0.75f);

        // cylinder walls + gas
        ImVec2 g0 = P(crown, -B / 2), g1 = P(topDeck, B / 2);
        ImVec2 quad[4] = {P(crown, -B / 2), P(crown, B / 2), P(topDeck, B / 2), P(topDeck, -B / 2)};
        dl->AddQuadFilled(quad[0], quad[1], quad[2], quad[3], gas);
        (void)g0; (void)g1;
        dl->AddLine(P(r + l - pistonH, -B / 2 - 3), P(topDeck, -B / 2 - 3), col(0.55f, 0.55f, 0.6f), 3);
        dl->AddLine(P(r + l - pistonH, B / 2 + 3), P(topDeck, B / 2 + 3), col(0.55f, 0.55f, 0.6f), 3);
        // head
        ImVec2 hq[4] = {P(topDeck, -B / 2 - 6), P(topDeck, B / 2 + 6), P(topDeck + B * 0.35, B / 2 + 6), P(topDeck + B * 0.35, -B / 2 - 6)};
        dl->AddQuadFilled(hq[0], hq[1], hq[2], hq[3], col(0.32f, 0.33f, 0.36f));
        // valves (intake left, exhaust right)
        double li = liftAt(phi, inC, p.intakeDuration, p.intakeLift_mm);
        double le = liftAt(phi, exC, p.exhaustDuration, p.exhaustLift_mm);
        double vd = B * 0.32;
        auto valve = [&](double across, double lift, ImU32 c) {
            double face = topDeck - lift;
            dl->AddLine(P(face, across - vd / 2), P(face, across + vd / 2), c, 4);
            dl->AddLine(P(face, across), P(topDeck + B * 0.45, across), c, 2);
        };
        valve(-B * 0.22, li, li > 0.3 ? col(0.4f, 0.7f, 1) : col(0.6f, 0.6f, 0.65f));
        valve(B * 0.22, le, le > 0.3 ? col(1, 0.55f, 0.3f) : col(0.6f, 0.6f, 0.65f));
        // spark
        if (sinceSpark < 12) {
            ImVec2 sp = P(topDeck - 2, 0);
            dl->AddCircleFilled(sp, 6, col(1, 1, 0.6f));
            dl->AddCircle(sp, 10, col(1, 0.9f, 0.3f), 0, 2);
        }
        // piston
        ImVec2 pq[4] = {P(crown, -B / 2 + 1), P(crown, B / 2 - 1), P(crown - pistonH, B / 2 - 1), P(crown - pistonH, -B / 2 + 1)};
        dl->AddQuadFilled(pq[0], pq[1], pq[2], pq[3], col(0.75f, 0.76f, 0.8f));
        dl->AddLine(P(crown - 4, -B / 2 + 1), P(crown - 4, B / 2 - 1), col(0.4f, 0.4f, 0.45f), 1.5f);
        // rod: pin -> crank pin
        ImVec2 pin = P(dist, 0);
        ImVec2 cpin((float)(crank.x + (up.x * cs + side.x * sn) * r * scale), (float)(crank.y + (up.y * cs + side.y * sn) * r * scale));
        dl->AddLine(pin, cpin, col(0.62f, 0.62f, 0.66f), (float)std::max(3.0, B * 0.12 * scale));
        dl->AddCircleFilled(pin, (float)(B * 0.06 * scale) + 1, col(0.3f, 0.3f, 0.33f));
        // crank
        dl->AddCircle(crank, (float)(r * scale), col(0.35f, 0.35f, 0.38f), 32, 1.0f);
        dl->AddLine(crank, cpin, col(0.85f, 0.7f, 0.3f), 5);
        dl->AddCircleFilled(crank, 5, col(0.5f, 0.5f, 0.55f));
        // label
        char lb[8];
        std::snprintf(lb, sizeof(lb), "%d", i + 1);
        ImVec2 lp = P(topDeck + B * 0.55, 0);
        dl->AddText(ImVec2(lp.x - 4, lp.y - 7), col(0.85f, 0.85f, 0.9f), lb);
    }
    dl->PopClipRect();
}

// ------------------------------------------------------------------ rotary animation
void RotaryAnimation(const EngineParams& p, const float phases[kMaxCyl], float shaftDeg, ImVec2 size) {
    ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("rotary_anim", size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, ImVec2(p0.x + size.x, p0.y + size.y), col(0.06f, 0.06f, 0.07f), 4);
    dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);
    const int n = clampv(p.cylinders, 1, 4);
    const double R = p.rotorRadius_mm, e = p.eccentricity_mm;
    const float cellW = size.x / n;
    const double scale = std::min(cellW * 0.42 / (R + e), size.y * 0.40 / (R + e));
    const double adv = p.baseAdvance + p.mechAdvance * 0.5;
    // stroke colour of a chamber at cycle angle phi (0 = firing TDC, 1080 cycle)
    auto gasColour = [&](double phi) -> ImU32 {
        double since = wrapPos(phi + adv, 1080.0);
        if (since < 130) { float t = (float)(1 - since / 130); return col(1.0f, 0.45f + 0.4f * t, 0.1f, 0.35f + 0.6f * t); }
        if (phi < 270) return col(0.85f, 0.45f, 0.15f, 0.35f);       // expansion
        if (phi < 540) return col(0.45f, 0.35f, 0.3f, 0.65f);        // exhaust
        if (phi < 810) return col(0.25f, 0.45f, 0.85f, 0.6f);        // intake
        return col(0.3f, 0.55f, 0.95f, 0.8f);                        // compression
    };
    for (int r = 0; r < n; ++r) {
        ImVec2 c0(p0.x + cellW * (r + 0.5f), p0.y + size.y * 0.5f);
        auto S = [&](double x, double y) { return ImVec2(c0.x + (float)(x * scale), c0.y - (float)(y * scale)); };
        // shaft angle seen by this rotor; +90 puts face 0's firing TDC at the top (spark plugs)
        double a = (shaftDeg - phases[r] + 90.0) * kDeg;
        double psi = a / 3.0;
        double cx = e * std::cos(a), cy = e * std::sin(a);
        // housing (epitrochoid, slightly offset outwards for the wall)
        const int N = 120;
        ImVec2 hp[N];
        for (int k = 0; k < N; ++k) {
            double t = 2 * kPi * k / N;
            hp[k] = S((e * std::cos(3 * t) + (R + 1.5) * std::cos(t)), (e * std::sin(3 * t) + (R + 1.5) * std::sin(t)));
        }
        dl->AddConvexPolyFilled(hp, N, col(0.12f, 0.12f, 0.14f));
        // chambers: fill the housing region of each face with its stroke colour
        ImVec2 apex[3];
        for (int k = 0; k < 3; ++k) {
            double t = psi + 2 * kPi * k / 3;
            apex[k] = S(cx + R * std::cos(t), cy + R * std::sin(t));
        }
        for (int k = 0; k < 3; ++k) {
            // chamber between apex k and k+1: walk the housing from t_k to t_k+1
            double t0 = psi + 2 * kPi * k / 3, t1 = t0 + 2 * kPi / 3;
            double phi = wrapPos(shaftDeg - phases[r] + 360.0 * k, 1080.0);
            ImU32 gc = gasColour(phi);
            const int M = 40;
            ImVec2 prev = S(e * std::cos(3 * t0) + R * std::cos(t0), e * std::sin(3 * t0) + R * std::sin(t0));
            for (int m = 1; m <= M; ++m) {
                double t = t0 + (t1 - t0) * m / M;
                ImVec2 cur = S(e * std::cos(3 * t) + R * std::cos(t), e * std::sin(3 * t) + R * std::sin(t));
                dl->AddTriangleFilled(S(cx, cy), prev, cur, gc);
                prev = cur;
            }
        }
        // rotor (slightly convex flanks) drawn over the chambers
        ImVec2 rp[3 * 12];
        int q = 0;
        for (int k = 0; k < 3; ++k) {
            double t0 = psi + 2 * kPi * k / 3;
            for (int m = 0; m < 12; ++m) {
                double u = (double)m / 12;
                double ang = t0 + u * 2 * kPi / 3;
                double bulge = 1.0 - 0.40 * std::sin(u * kPi);   // flank sits inside the housing
                rp[q++] = S(cx + R * bulge * std::cos(ang) * (1 - 0.0), cy + R * bulge * std::sin(ang));
            }
        }
        dl->AddConvexPolyFilled(rp, q, col(0.68f, 0.69f, 0.73f));
        dl->AddPolyline(rp, q, col(0.4f, 0.4f, 0.45f), ImDrawFlags_Closed, 1.5f);
        // combustion recesses
        for (int k = 0; k < 3; ++k) {
            double ang = psi + 2 * kPi * k / 3 + kPi / 3;
            dl->AddCircleFilled(S(cx + R * 0.58 * std::cos(ang), cy + R * 0.58 * std::sin(ang)), (float)(R * 0.07 * scale), col(0.5f, 0.5f, 0.55f));
        }
        // apex seals
        for (int k = 0; k < 3; ++k) dl->AddCircleFilled(apex[k], 3.5f, col(0.95f, 0.85f, 0.4f));
        // rotor gear + eccentric shaft
        dl->AddCircle(S(cx, cy), (float)(R * 0.33 * scale), col(0.45f, 0.45f, 0.5f), 32, 2);
        dl->AddCircleFilled(S(0, 0), (float)(R * 0.16 * scale), col(0.35f, 0.35f, 0.38f));
        dl->AddLine(S(0, 0), S(cx, cy), col(0.85f, 0.7f, 0.3f), 5);
        dl->AddCircleFilled(S(cx, cy), 4, col(0.85f, 0.7f, 0.3f));
        // housing outline
        dl->AddPolyline(hp, N, col(0.55f, 0.55f, 0.6f), ImDrawFlags_Closed, 3);
        // spark plugs at the top (leading / trailing), ports at the bottom
        bool fire = false;
        for (int k = 0; k < 3; ++k) if (wrapPos(shaftDeg - phases[r] + 360.0 * k + adv, 1080.0) < 14) fire = true;
        for (int s = -1; s <= 1; s += 2) {
            ImVec2 pl = S(s * R * 0.12, R - e + 4);
            dl->AddRectFilled(ImVec2(pl.x - 3, pl.y - 12), ImVec2(pl.x + 3, pl.y), col(0.8f, 0.8f, 0.85f));
            if (fire) dl->AddCircleFilled(ImVec2(pl.x, pl.y + 4), 5, col(1, 1, 0.6f));
        }
        ImVec2 ip = S(-R * 0.55, -(R - e) - 6), ep = S(R * 0.55, -(R - e) - 6);
        dl->AddRectFilled(ImVec2(ip.x - 14, ip.y - 4), ImVec2(ip.x + 14, ip.y + 10), col(0.25f, 0.45f, 0.85f));
        dl->AddRectFilled(ImVec2(ep.x - 14, ep.y - 4), ImVec2(ep.x + 14, ep.y + 10), col(0.9f, 0.45f, 0.2f));
        dl->AddText(ImVec2(ip.x - 10, ip.y + 12), col(0.6f, 0.75f, 1), "IN");
        dl->AddText(ImVec2(ep.x - 12, ep.y + 12), col(1, 0.7f, 0.5f), "EX");
        char lb[16];
        std::snprintf(lb, sizeof(lb), "Rotor %d", r + 1);
        ImVec2 lp = S(-R - e, R);
        dl->AddText(lp, col(0.85f, 0.85f, 0.9f), lb);
    }
    dl->PopClipRect();
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
