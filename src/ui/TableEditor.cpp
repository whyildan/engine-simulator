#include "ui/TableEditor.h"
#include "imgui.h"
#include "implot.h"
#include "implot3d.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace es::ui {

bool g_tableKeysCaptured = false;   // set while a grid has keyboard focus (App suppresses its shortcuts)

namespace {
enum Op { kSet, kAdd, kMul, kInterpH, kInterpV, kSmooth };

ImU32 heat(float t, float alpha = 1.0f) {
    ImVec4 c = ImPlot::SampleColormap(clampv(t, 0.0f, 1.0f), ImPlotColormap_Jet);
    // muted so the numbers stay readable on the dark theme
    return ImGui::ColorConvertFloat4ToU32(ImVec4(c.x * 0.55f + 0.05f, c.y * 0.55f + 0.05f, c.z * 0.55f + 0.05f, alpha));
}
ImU32 diverging(float d) {   // d in -1..1: blue (lower) .. grey .. red (higher)
    d = clampv(d, -1.0f, 1.0f);
    ImVec4 c = d >= 0 ? ImVec4(0.18f + 0.5f * d, 0.18f, 0.18f, 1) : ImVec4(0.18f, 0.18f + 0.15f * -d, 0.18f + 0.5f * -d, 1);
    return ImGui::ColorConvertFloat4ToU32(c);
}
const char* fmtFor(const TableInfo& info) { return info.step < 0.01f ? "%.3f" : info.step < 0.1f ? "%.2f" : info.step < 1.0f ? "%.1f" : "%.0f"; }
} // namespace

TableEditor::Sel TableEditor::norm() const {
    Sel s;
    s.r0 = std::min(curR_, ancR_); s.r1 = std::max(curR_, ancR_);
    s.c0 = std::min(curC_, ancC_); s.c1 = std::max(curC_, ancC_);
    return s;
}

void TableEditor::pushUndo(const Table2D& t) {
    undo_.push_back(t);
    if (undo_.size() > 100) undo_.erase(undo_.begin());
    redo_.clear();
}

bool TableEditor::applyOp(Table2D& t, int op, float arg) {
    if (t.locked) return false;
    Sel s = norm();
    pushUndo(t);
    switch (op) {
    case kSet: for (int r = s.r0; r <= s.r1; ++r) for (int c = s.c0; c <= s.c1; ++c) t.v[r][c] = arg; break;
    case kAdd: for (int r = s.r0; r <= s.r1; ++r) for (int c = s.c0; c <= s.c1; ++c) t.v[r][c] += arg; break;
    case kMul: for (int r = s.r0; r <= s.r1; ++r) for (int c = s.c0; c <= s.c1; ++c) t.v[r][c] *= 1.0f + arg * 0.01f; break;
    case kInterpH:   // straight line between the first and last selected column, by the rpm axis
        if (s.c1 - s.c0 < 2) return false;
        for (int r = s.r0; r <= s.r1; ++r) {
            float a = t.v[r][s.c0], b = t.v[r][s.c1], xa = t.x[s.c0], xb = t.x[s.c1];
            for (int c = s.c0 + 1; c < s.c1; ++c) t.v[r][c] = a + (b - a) * (t.x[c] - xa) / std::max(1e-6f, xb - xa);
        }
        break;
    case kInterpV:
        if (s.r1 - s.r0 < 2) return false;
        for (int c = s.c0; c <= s.c1; ++c) {
            float a = t.v[s.r0][c], b = t.v[s.r1][c], ya = t.y[s.r0], yb = t.y[s.r1];
            for (int r = s.r0 + 1; r < s.r1; ++r) t.v[r][c] = a + (b - a) * (t.y[r] - ya) / std::max(1e-6f, yb - ya);
        }
        break;
    case kSmooth: {  // 3x3 weighted average (centre 4, edges 2, corners 1) inside the selection
        Table2D src = t;
        for (int r = s.r0; r <= s.r1; ++r)
            for (int c = s.c0; c <= s.c1; ++c) {
                float sum = 0, w = 0;
                for (int dr = -1; dr <= 1; ++dr)
                    for (int dc = -1; dc <= 1; ++dc) {
                        int rr = r + dr, cc = c + dc;
                        if (rr < 0 || cc < 0 || rr >= t.ny || cc >= t.nx) continue;
                        float k = (dr == 0 && dc == 0) ? 4.0f : (dr == 0 || dc == 0) ? 2.0f : 1.0f;
                        sum += k * src.v[rr][cc]; w += k;
                    }
                t.v[r][c] = sum / w;
            }
        break;
    }
    default: return false;
    }
    return true;
}

void TableEditor::copySel(const Table2D& t) {
    Sel s = norm();
    std::string out;
    char b[32];
    for (int r = s.r1; r >= s.r0; --r) {          // top row first, like the grid shows it
        for (int c = s.c0; c <= s.c1; ++c) {
            std::snprintf(b, sizeof(b), "%g", t.v[r][c]);
            out += b;
            out += c < s.c1 ? "\t" : "";
        }
        out += "\n";
    }
    ImGui::SetClipboardText(out.c_str());
}

bool TableEditor::paste(Table2D& t) {
    if (t.locked) return false;
    const char* clip = ImGui::GetClipboardText();
    if (!clip || !*clip) return false;
    Sel s = norm();
    pushUndo(t);
    int r = s.r1, c = s.c0;                       // paste from the top-left of the selection
    const char* p = clip;
    bool any = false;
    while (*p && r >= 0) {
        char* end = nullptr;
        double v = std::strtod(p, &end);
        if (end != p) {
            if (c < t.nx) { t.v[r][c] = (float)v; any = true; }
            p = end;
        } else if (*p == '\t' || *p == ',' || *p == ';') { ++c; ++p; }
        else if (*p == '\n') { --r; c = s.c0; ++p; }
        else ++p;
    }
    return any;
}

bool TableEditor::draw(const char* id, Table2D& t, const TableInfo& info, OpPoint op, const std::vector<OpPoint>* trail,
                       const Table2D* compare, const char* compareName) {
    ImGui::PushID(id);
    bool changed = false;
    const int nx = clampv((int)t.nx, 1, kTabMax), ny = clampv((int)t.ny, 1, kTabMax);
    curR_ = clampv(curR_, 0, ny - 1); curC_ = clampv(curC_, 0, nx - 1);
    ancR_ = clampv(ancR_, 0, ny - 1); ancC_ = clampv(ancC_, 0, nx - 1);
    const char* fmt = fmtFor(info);
    static bool showDiff = true;

    // ------------------------------------------------------------------ toolbar
    {
        bool lk = t.locked != 0;
        if (ImGui::Checkbox("Lock", &lk)) { t.locked = lk ? 1 : 0; changed = true; }
        ImGui::SameLine();
        ImGui::BeginDisabled(lk);
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("##set", &setVal_, 0, 0, fmt);
        ImGui::SameLine();
        if (ImGui::Button("Set")) changed |= applyOp(t, kSet, setVal_);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(55);
        ImGui::InputFloat("##add", &addVal_, 0, 0, fmt);
        ImGui::SameLine();
        if (ImGui::Button("+")) changed |= applyOp(t, kAdd, addVal_);
        ImGui::SameLine();
        if (ImGui::Button("-")) changed |= applyOp(t, kAdd, -addVal_);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(55);
        ImGui::InputFloat("##mul", &mulPct_, 0, 0, "%.1f%%");
        ImGui::SameLine();
        if (ImGui::Button("x%")) changed |= applyOp(t, kMul, mulPct_);
        ImGui::SameLine();
        if (ImGui::Button("Interp H")) changed |= applyOp(t, kInterpH, 0);
        ImGui::SameLine();
        if (ImGui::Button("Interp V")) changed |= applyOp(t, kInterpV, 0);
        ImGui::SameLine();
        if (ImGui::Button("Smooth")) changed |= applyOp(t, kSmooth, 0);
        ImGui::SameLine();
        ImGui::EndDisabled();
        if (ImGui::Button("Copy")) copySel(t);
        ImGui::SameLine();
        ImGui::BeginDisabled(lk);
        if (ImGui::Button("Paste")) changed |= paste(t);
        ImGui::SameLine();
        ImGui::BeginDisabled(undo_.empty());
        if (ImGui::Button("Undo")) { redo_.push_back(t); t = undo_.back(); undo_.pop_back(); changed = true; }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(redo_.empty());
        if (ImGui::Button("Redo")) { undo_.push_back(t); t = redo_.back(); redo_.pop_back(); changed = true; }
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        if (ny > 1) { ImGui::SameLine(); ImGui::Checkbox("3D", &show3D); }
        if (compare) { ImGui::SameLine(); ImGui::Checkbox("Diff", &showDiff); }
        if (ImGui::IsItemHovered() && compare) ImGui::SetTooltip("Colour by the difference to '%s'", compareName ? compareName : "reference");
    }
    // value entry for the selection
    if (editing_) {
        ImGui::SetNextItemWidth(120);
        if (focusEdit_) { ImGui::SetKeyboardFocusHere(); focusEdit_ = false; }
        if (ImGui::InputText("##edit", editBuf_, sizeof(editBuf_), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsScientific)) {
            changed |= applyOp(t, kSet, (float)std::atof(editBuf_));
            editing_ = false;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("Enter = set selection, Esc = cancel");
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) editing_ = false;
    }

    // ------------------------------------------------------------------ grid
    float vmin = 1e30f, vmax = -1e30f;
    for (int r = 0; r < ny; ++r) for (int c = 0; c < nx; ++c) { vmin = std::min(vmin, t.v[r][c]); vmax = std::max(vmax, t.v[r][c]); }
    float dmax = 1e-6f;
    const bool diff = compare && showDiff && compare->nx == t.nx && compare->ny == t.ny;
    if (diff) for (int r = 0; r < ny; ++r) for (int c = 0; c < nx; ++c) dmax = std::max(dmax, std::fabs(t.v[r][c] - compare->v[r][c]));

    const float cellW = ImGui::CalcTextSize("-000.00").x + 6.0f;
    ImVec2 centers[kTabMax][kTabMax];
    // (no ScrollX: that gives the table its own child window, which would cover the operating-point overlay)
    const ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoPadOuterX;
    const float gridH = ImGui::GetFrameHeight() * (ny + 1.6f) + 4;
    ImGui::BeginChild("##grid", ImVec2(0, gridH), ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollbar);
    const bool gridFocused = ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows);
    if (ImGui::BeginTable("t", nx + 1, tf)) {
        ImGui::TableSetupColumn("##ax", ImGuiTableColumnFlags_WidthFixed, cellW);
        for (int c = 0; c < nx; ++c) ImGui::TableSetupColumn("##c", ImGuiTableColumnFlags_WidthFixed, cellW);
        // header: x axis
        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        ImGui::TableNextColumn();
        ImGui::TextDisabled("%s", ny > 1 ? info.yName : "");
        for (int c = 0; c < nx; ++c) {
            ImGui::TableNextColumn();
            char lb[32]; std::snprintf(lb, sizeof(lb), "%g##x%d", t.x[c], c);
            ImGui::Selectable(lb, false, ImGuiSelectableFlags_AllowDoubleClick);
            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0) && !t.locked) {
                axisEdit_ = c; std::snprintf(axisBuf_, sizeof(axisBuf_), "%g", t.x[c]); ImGui::OpenPopup("axis");
            }
        }
        for (int r = ny - 1; r >= 0; --r) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            char lb[32]; std::snprintf(lb, sizeof(lb), "%g##y%d", ny > 1 ? t.y[r] : 0.0f, r);
            if (ny > 1) {
                ImGui::Selectable(lb, false, ImGuiSelectableFlags_AllowDoubleClick);
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0) && !t.locked) {
                    axisEdit_ = 100 + r; std::snprintf(axisBuf_, sizeof(axisBuf_), "%g", t.y[r]); ImGui::OpenPopup("axis");
                }
            } else ImGui::TextDisabled("%s", info.unit);
            Sel s = norm();
            for (int c = 0; c < nx; ++c) {
                ImGui::TableNextColumn();
                float v = t.v[r][c];
                ImU32 bg = diff ? diverging((v - compare->v[r][c]) / dmax) : heat(vmax > vmin ? (v - vmin) / (vmax - vmin) : 0.5f);
                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, bg);
                char txt[48];
                if (diff) std::snprintf(txt, sizeof(txt), "%+.2f##%d_%d", v - compare->v[r][c], r, c);
                else { char f[16]; std::snprintf(f, sizeof(f), "%s##%%d_%%d", fmt); std::snprintf(txt, sizeof(txt), f, v, r, c); }
                bool sel = r >= s.r0 && r <= s.r1 && c >= s.c0 && c <= s.c1;
                if (ImGui::Selectable(txt, sel, 0, ImVec2(cellW - 6, 0))) {}
                ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
                centers[r][c] = ImVec2((mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f);
                if (ImGui::IsItemClicked()) {
                    curR_ = r; curC_ = c;
                    if (!ImGui::GetIO().KeyShift) { ancR_ = r; ancC_ = c; }
                    dragging_ = true; editing_ = false;
                }
                if (dragging_ && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem) && ImGui::IsMouseDown(0)) { curR_ = r; curC_ = c; }
                if (ImGui::IsItemHovered() && !dragging_) {
                    if (compare) ImGui::SetTooltip("%s %g, %s %g\nvalue %g  (ref %g)", info.xName, t.x[c], info.yName, t.y[r], v, compare->v[r][c]);
                    else ImGui::SetTooltip("%s %g, %s %g\nvalue %g", info.xName, t.x[c], info.yName, t.y[r], v);
                }
            }
        }
        // axis breakpoint editor
        if (ImGui::BeginPopup("axis")) {
            ImGui::Text("Breakpoint (must stay between its neighbours)");
            ImGui::SetNextItemWidth(120);
            if (ImGui::InputText("##ab", axisBuf_, sizeof(axisBuf_), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsScientific)) {
                float v = (float)std::atof(axisBuf_);
                float* a = axisEdit_ >= 100 ? t.y : t.x;
                int i = axisEdit_ >= 100 ? axisEdit_ - 100 : axisEdit_, n = axisEdit_ >= 100 ? ny : nx;
                bool ok = (i == 0 || v > a[i - 1]) && (i == n - 1 || v < a[i + 1]);
                if (ok) { pushUndo(t); a[i] = v; changed = true; }
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        ImGui::EndTable();
    }
    if (!ImGui::IsMouseDown(0)) dragging_ = false;

    // operating point: bilinear position between the cell centres, plus its recent trail
    ImDrawList* dl = ImGui::GetWindowDrawList();
    auto toScreen = [&](float x, float y) {
        CellWeights w = tableWeights(t, x, y);
        int iy0 = ny > 1 ? w.iy0 : 0, iy1 = ny > 1 ? w.iy1 : 0;
        float wy = ny > 1 ? w.wy : 0.0f;
        ImVec2 a = centers[iy0][w.ix0], b = centers[iy0][w.ix1], c = centers[iy1][w.ix0], d = centers[iy1][w.ix1];
        ImVec2 top(a.x + (b.x - a.x) * w.wx, a.y + (b.y - a.y) * w.wx), bot(c.x + (d.x - c.x) * w.wx, c.y + (d.y - c.y) * w.wx);
        return ImVec2(top.x + (bot.x - top.x) * wy, top.y + (bot.y - top.y) * wy);
    };
    if (trail)
        for (const OpPoint& p : *trail) if (p.valid) dl->AddCircleFilled(toScreen(p.x, p.y), 2.0f, IM_COL32(255, 255, 255, 70));
    if (op.valid) {
        ImVec2 p = toScreen(op.x, op.y);
        dl->AddCircle(p, 7.0f, IM_COL32(255, 255, 255, 230), 0, 2.0f);
        dl->AddCircleFilled(p, 3.0f, IM_COL32(255, 210, 60, 255));
    }

    // keyboard (grid focused): arrows move / extend, digits or Enter edit, +/- step, Ctrl+C/V/Z/Y
    g_tableKeysCaptured = g_tableKeysCaptured || gridFocused;
    if (gridFocused && !editing_ && axisEdit_ < 0) {
        ImGuiIO& io = ImGui::GetIO();
        auto mv = [&](int dr, int dc) {
            curR_ = clampv(curR_ + dr, 0, ny - 1); curC_ = clampv(curC_ + dc, 0, nx - 1);
            if (!io.KeyShift) { ancR_ = curR_; ancC_ = curC_; }
        };
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) mv(1, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) mv(-1, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) mv(0, -1);
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) mv(0, 1);
        if (io.KeyCtrl) {
            if (ImGui::IsKeyPressed(ImGuiKey_C)) copySel(t);
            if (ImGui::IsKeyPressed(ImGuiKey_V)) changed |= paste(t);
            if (ImGui::IsKeyPressed(ImGuiKey_Z) && !undo_.empty()) { redo_.push_back(t); t = undo_.back(); undo_.pop_back(); changed = true; }
            if (ImGui::IsKeyPressed(ImGuiKey_Y) && !redo_.empty()) { undo_.push_back(t); t = redo_.back(); redo_.pop_back(); changed = true; }
        } else if (!t.locked) {
            if (ImGui::IsKeyPressed(ImGuiKey_Equal) || ImGui::IsKeyPressed(ImGuiKey_KeypadAdd)) changed |= applyOp(t, kAdd, info.step);
            if (ImGui::IsKeyPressed(ImGuiKey_Minus) || ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract)) changed |= applyOp(t, kAdd, -info.step);
            if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)) {
                std::snprintf(editBuf_, sizeof(editBuf_), "%g", t.v[curR_][curC_]); editing_ = true; focusEdit_ = true;
            }
            for (int k = 0; k <= 9; ++k)
                if (ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_0 + k)) || ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_Keypad0 + k))) {
                    std::snprintf(editBuf_, sizeof(editBuf_), "%d", k); editing_ = true; focusEdit_ = true;
                }
        }
    }
    if (axisEdit_ >= 0 && !ImGui::IsPopupOpen("axis")) axisEdit_ = -1;
    ImGui::EndChild();

    // status line
    {
        Sel s = norm();
        double sum = 0; int n = 0;
        for (int r = s.r0; r <= s.r1; ++r) for (int c = s.c0; c <= s.c1; ++c) { sum += t.v[r][c]; ++n; }
        ImGui::TextDisabled("cell %s %g%s%s: %g   selection %dx%d  mean %.3g   range %.3g .. %.3g %s%s", info.xName, t.x[curC_],
                            ny > 1 ? ", " : "", ny > 1 ? info.yName : "", t.v[curR_][curC_], s.c1 - s.c0 + 1, s.r1 - s.r0 + 1,
                            n ? sum / n : 0.0, vmin, vmax, info.unit, t.locked ? "   [LOCKED]" : "");
    }

    // 3D surface
    if (show3D && ny > 1) {
        static std::vector<float> xs, ys, zs;
        xs.resize(nx * ny); ys.resize(nx * ny); zs.resize(nx * ny);
        for (int r = 0; r < ny; ++r)
            for (int c = 0; c < nx; ++c) { xs[r * nx + c] = t.x[c]; ys[r * nx + c] = t.y[r]; zs[r * nx + c] = t.v[r][c]; }
        if (ImPlot3D::BeginPlot("##surface", ImVec2(-1, 320), ImPlot3DFlags_NoLegend)) {
            ImPlot3D::SetupAxes(info.xName, info.yName, info.unit);
            ImPlot3D::PlotSurface("map", xs.data(), ys.data(), zs.data(), nx, ny);
            if (op.valid) {
                float px = op.x, py = op.y, pz = (float)tableLookup(t, op.x, op.y);
                ImPlot3D::PlotScatter("now", &px, &py, &pz, 1);
            }
            ImPlot3D::EndPlot();
        }
    }
    ImGui::PopID();
    return changed;
}

// 1D curve: edited as a one-row table, with a plot underneath
bool TableEditor::drawCurve(const char* id, Curve1D& cv, const TableInfo& info, float opX, bool opValid, const Curve1D* compare) {
    Table2D t{};
    t.nx = clampv((int)cv.n, 1, kCurveMax); t.ny = 1; t.locked = cv.locked;
    for (int i = 0; i < t.nx; ++i) { t.x[i] = cv.x[i]; t.v[0][i] = cv.v[i]; }
    Table2D ref{};
    if (compare) { ref = t; for (int i = 0; i < t.nx; ++i) ref.v[0][i] = compare->v[i]; }
    OpPoint op; op.x = opX; op.y = 0; op.valid = opValid;
    bool changed = draw(id, t, info, op, nullptr, compare ? &ref : nullptr, nullptr);
    if (changed) {
        cv.locked = t.locked;
        for (int i = 0; i < t.nx; ++i) { cv.x[i] = t.x[i]; cv.v[i] = t.v[0][i]; }
    }
    ImGui::PushID(id);
    if (ImPlot::BeginPlot("##curve", ImVec2(-1, 150), ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes(info.xName, info.unit, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::PlotLine("curve", cv.x, cv.v, t.nx);
        ImPlot::PlotScatter("pts", cv.x, cv.v, t.nx);
        if (compare) ImPlot::PlotLine("ref", compare->x, compare->v, clampv((int)compare->n, 1, kCurveMax));
        if (opValid) {
            double ox = opX, oy = curveLookup(cv, opX);
            ImPlot::PlotScatter("now", &ox, &oy, 1);
        }
        ImPlot::EndPlot();
    }
    ImGui::PopID();
    return changed;
}

} // namespace es::ui
