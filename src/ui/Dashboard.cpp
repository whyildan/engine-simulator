// Live Dashboard > Custom (Phase 4): a grid of widgets the user configures - any log channel as an analogue
// gauge, a digital readout or a bar, with warning / alarm thresholds (low and high side) - plus a sensor status
// strip. The layout is saved next to the executable (dashboard.cfg).
#include "ui/App.h"
#include "ui/Widgets.h"
#include "ecu/CalibrationIO.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace es {
using namespace ui;

static std::string dashPath() {
    std::filesystem::path p = std::filesystem::u8path(calibrationDir()).parent_path();
    return (p / "dashboard.cfg").u8string();
}

void App::dashDefaults() {
    auto w = [](const char* key, int type, float lo, float hi, int useLo, float warnLo, float alarmLo, int useHi, float warnHi, float alarmHi) {
        DashWidget d;
        d.ch = std::max(0, logChannelIndex(key)); d.type = type; d.lo = lo; d.hi = hi;
        d.useLo = useLo; d.warnLo = warnLo; d.alarmLo = alarmLo; d.useHi = useHi; d.warnHi = warnHi; d.alarmHi = alarmHi;
        return d;
    };
    dash_.clear();
    dash_.push_back(w("rpm", 0, 0, std::ceil((p_.revLimit + 1000) / 1000) * 1000, 0, 0, 0, 1, p_.revLimit - 300, p_.revLimit));
    dash_.push_back(w("map", 0, 0, isBoosted(p_) ? 300 : 110, 0, 0, 0, 0, 0, 0));
    dash_.push_back(w("afr", 1, 8, 20, 1, 11.0f, 10.0f, 1, 15.5f, 16.5f));
    dash_.push_back(w("advance", 1, -10, 50, 0, 0, 0, 0, 0, 0));
    dash_.push_back(w("ect", 2, 0, 130, 0, 0, 0, 1, 105, 112));
    dash_.push_back(w("oil_press", 2, 0, 8, 1, 1.0f, 0.6f, 0, 0, 0));
    dash_.push_back(w("egt", 2, 0, 1100, 0, 0, 0, 1, 900, 980));
    dash_.push_back(w("knock_retard", 1, 0, 15, 0, 0, 0, 1, 4, 8));
    dashCols_ = 4;
}

void App::dashSave() {
    std::ofstream f(std::filesystem::u8path(dashPath()), std::ios::binary);
    if (!f) return;
    f << "cols " << dashCols_ << "\n";
    for (const DashWidget& d : dash_)
        f << "w " << logChannel(d.ch).key << " " << d.type << " " << d.lo << " " << d.hi << " " << d.useLo << " " << d.warnLo << " " << d.alarmLo
          << " " << d.useHi << " " << d.warnHi << " " << d.alarmHi << "\n";
}

void App::dashLoad() {
    dashLoaded_ = true;
    std::ifstream f(std::filesystem::u8path(dashPath()), std::ios::binary);
    if (!f) { dashDefaults(); return; }
    std::vector<DashWidget> out;
    std::string line;
    int cols = 4;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string tag; ls >> tag;
        if (tag == "cols") ls >> cols;
        else if (tag == "w") {
            std::string key; DashWidget d;
            ls >> key >> d.type >> d.lo >> d.hi >> d.useLo >> d.warnLo >> d.alarmLo >> d.useHi >> d.warnHi >> d.alarmHi;
            d.ch = logChannelIndex(key.c_str());
            if (!ls.fail() && d.ch >= 0) out.push_back(d);
        }
    }
    if (out.empty()) { dashDefaults(); return; }
    dash_ = out;
    dashCols_ = std::clamp(cols, 1, 6);
}

// 0 ok, 1 warning, 2 alarm
static int dashLevel(const DashWidget& d, float v) {
    int lv = 0;
    if (d.useLo) { if (v <= d.alarmLo) lv = 2; else if (v <= d.warnLo) lv = std::max(lv, 1); }
    if (d.useHi) { if (v >= d.alarmHi) lv = 2; else if (v >= d.warnHi) lv = std::max(lv, 1); }
    return lv;
}

void App::panelCustomDash() {
    if (!dashLoaded_) dashLoad();
    const float em = ImGui::GetFontSize();
    // ---- alarms + sensor status strip
    std::string alarms;
    for (const DashWidget& d : dash_)
        if (dashLevel(d, logChannel(d.ch).get(tel_)) == 2) alarms += std::string(alarms.empty() ? "" : ", ") + logChannel(d.ch).name;
    const bool blink = std::fmod(ImGui::GetTime(), 0.8) < 0.5;
    if (!alarms.empty()) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, blink ? 1.0f : 0.5f), "ALARM: %s", alarms.c_str());
    else ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1), "No alarm");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - em * 9);
    ImGui::Checkbox("Edit layout", &dashEdit_);
    ImGui::TextDisabled("Sensors:");
    for (int i = 0; i < kSensorCount; ++i) {
        ImGui::SameLine();
        const bool bad = tel_.snsFallback[i];
        char nm[16];
        std::snprintf(nm, sizeof(nm), "%.*s", (int)std::strcspn(sensorDef(i).name, " "), sensorDef(i).name);
        ImGui::TextColored(bad ? ImVec4(1, 0.35f, 0.3f, 1) : ImVec4(0.45f, 0.75f, 0.45f, 1), "%s", nm);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s: %s", sensorDef(i).name, bad ? "FAULT - ECU uses a fallback value" : "ok");
    }
    ImGui::Separator();

    // ---- widget grid
    const int cols = std::clamp(dashCols_, 1, 6);
    const float cellW = (ImGui::GetContentRegionAvail().x - (cols - 1) * ImGui::GetStyle().ItemSpacing.x) / cols;
    int remove = -1;
    for (int i = 0; i < (int)dash_.size(); ++i) {
        DashWidget& d = dash_[i];
        const LogChannel& lc = logChannel(d.ch);
        const float v = lc.get(tel_);
        const int lv = dashLevel(d, v);
        if (i % cols) ImGui::SameLine();
        ImGui::PushID(i);
        const ImVec4 bg = lv == 2 ? ImVec4(0.55f, 0.08f, 0.08f, blink ? 0.9f : 0.45f) : lv == 1 ? ImVec4(0.45f, 0.32f, 0.05f, 0.7f) : ImVec4(0.12f, 0.12f, 0.14f, 1);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, bg);
        const float h = d.type == 0 ? cellW * 0.9f : em * 4.2f;
        ImGui::BeginChild("##w", ImVec2(cellW, dashEdit_ ? h + em * 8.5f : h), ImGuiChildFlags_Border,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        char fmt[32];
        std::snprintf(fmt, sizeof(fmt), "%%.%df %s", std::fabs(d.hi - d.lo) <= 30 ? 2 : 0, lc.unit);
        if (d.type == 0) {
            const float red = d.useHi ? d.warnHi : d.hi * 2;
            ImGui::TextDisabled("%s", lc.name);
            const float gs = std::min(cellW - em, h - em * 2.2f);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (ImGui::GetContentRegionAvail().x - gs) * 0.5f));
            Gauge("g", v, d.lo, d.hi, red, "", fmt, gs, 5, 1.0f);
        } else if (d.type == 1) {
            ImGui::TextDisabled("%s", lc.name);
            ImGui::SetWindowFontScale(2.2f);
            ImGui::Text(fmt, v);
            ImGui::SetWindowFontScale(1.0f);
        } else {
            ImGui::TextDisabled("%s", lc.name);
            const ImU32 col = lv == 2 ? IM_COL32(240, 70, 60, 255) : lv == 1 ? IM_COL32(240, 180, 60, 255) : IM_COL32(90, 170, 240, 255);
            Meter("##m", v, d.lo, d.hi, fmt, col);
        }
        if (dashEdit_) {
            ImGui::Separator();
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##ch", lc.name)) {
                for (int c = 0; c < logChannelCount(); ++c)
                    if (ImGui::Selectable(logChannel(c).name, c == d.ch)) d.ch = c;
                ImGui::EndCombo();
            }
            ImGui::SetNextItemWidth(-em * 2.5f);
            ImGui::Combo("##ty", &d.type, "Analogue gauge\0Digital\0Bar\0");
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) remove = i;
            ImGui::SetNextItemWidth(-1);
            ImGui::DragFloatRange2("##rg", &d.lo, &d.hi, std::max(0.01f, (d.hi - d.lo) * 0.005f), 0, 0, "min %.4g", "max %.4g");
            bool lo = d.useLo != 0, hi = d.useHi != 0;
            if (ImGui::Checkbox("low", &lo)) d.useLo = lo;
            ImGui::SameLine(); ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat2("##lo", &d.warnLo, 0.05f, 0, 0, "%.4g");
            if (ImGui::Checkbox("high", &hi)) d.useHi = hi;
            ImGui::SameLine(); ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat2("##hi", &d.warnHi, 0.05f, 0, 0, "%.4g");
            ImGui::TextDisabled("(warning, alarm)");
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::PopID();
    }
    if (remove >= 0) dash_.erase(dash_.begin() + remove);
    if (dashEdit_) {
        ImGui::Separator();
        if (ImGui::Button("Add widget")) { DashWidget d; d.ch = 0; d.lo = 0; d.hi = 8000; dash_.push_back(d); }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(em * 6);
        ImGui::SliderInt("columns", &dashCols_, 1, 6);
        ImGui::SameLine();
        if (ImGui::Button("Save layout")) dashSave();
        ImGui::SameLine();
        if (ImGui::Button("Defaults")) dashDefaults();
        ImGui::SameLine();
        ImGui::TextDisabled("saved to %s", dashPath().c_str());
    }
}

} // namespace es
