// Data Logging section (Phase 4): record every log channel at a chosen rate (in sim time), markers (manual and
// automatic), sessions tied to the calibration they ran on, stacked graphs over time or rpm with an overlaid
// comparison session, CSV export / import.
#include "ui/App.h"
#include "ui/FileDialog.h"
#include "ui/Widgets.h"
#include "imgui.h"
#include "implot.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>

namespace es {
using namespace ui;

static const float kLogRates[] = {10, 20, 50, 100};

static std::string nowStamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_s(&tm, &t);
    char b[32];
    std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tm);
    return b;
}

void App::logStart() {
    if (logRec_ >= 0) return;
    LogSession s;
    s.clear();
    char nm[96];
    std::snprintf(nm, sizeof(nm), "Log %d - %s", ++logCounter_, p_.name);
    s.name = nm;
    s.engine = p_.name;
    s.calName = calMeta_.name.empty() ? "(unsaved)" : calMeta_.name;
    s.calRev = calMeta_.revision;
    s.started = nowStamp();
    s.rateHz = kLogRates[std::clamp(logRateIdx_, 0, 3)];
    logSessions_.push_back(std::move(s));
    logRec_ = (int)logSessions_.size() - 1;
    logView_ = logRec_;
    logT0_ = -1;
    std::vector<double> drop; std::vector<float> dropV;
    sim_->takeLogSamples(drop, dropV);   // discard anything left from an earlier recording
    sim_->setLogging(true, logSessions_[logRec_].rateHz);
    for (int i = 0; i < kProtCount; ++i) logPrevProt_[i] = tel_.protState[i];
    logPrevGear_ = tel_.gear;
    logPrevLimp_ = tel_.limp;
}

void App::logStop() {
    if (logRec_ < 0) return;
    sim_->setLogging(false, 50);
    logFrame();   // take the last samples
    logRec_ = -1;
}

void App::logMarker(const std::string& label) {
    if (logRec_ < 0) return;
    LogSession& s = logSessions_[logRec_];
    s.markers.push_back({s.t.empty() ? 0.0 : s.t.back(), label});
}

// every frame: move the samples the simulator recorded into the session, set automatic markers
void App::logFrame() {
    if (logRec_ < 0) return;
    LogSession& s = logSessions_[logRec_];
    std::vector<double> times; std::vector<float> vals;
    sim_->takeLogSamples(times, vals);
    const int n = logChannelCount();
    for (size_t k = 0; k < times.size(); ++k) {
        if (logT0_ < 0) logT0_ = times[k];
        s.append(times[k] - logT0_, &vals[k * n]);
    }
    for (int i = 0; i < kProtCount; ++i) {
        if (tel_.protState[i] == 2 && logPrevProt_[i] != 2) logMarker(std::string("protection: ") + protInfo(i).name);
        logPrevProt_[i] = tel_.protState[i];
    }
    if (tel_.limp && !logPrevLimp_) logMarker("limp mode");
    logPrevLimp_ = tel_.limp;
    if (p_.vehicle && tel_.gear != logPrevGear_) logMarker("gear " + std::to_string(tel_.gear));
    logPrevGear_ = tel_.gear;
    // launch: the moment the 2-step lets go is the start of the run
    if (logPrevLaunch_ == kLaunch2Step && tel_.launchState != kLaunch2Step) logMarker("launch");
    logPrevLaunch_ = tel_.launchState;
    if (tel_.n2oDuty > 0 && !logPrevN2o_) logMarker("nitrous on");
    logPrevN2o_ = tel_.n2oDuty > 0;
}

// dyno runs -> CSV (every visible run, metric and imperial columns, the calibration each run was made with)
void App::dynoExportCsv() {
    std::string path;
    if (!fileDialog(true, "dyno.csv", path, L"CSV (*.csv)\0*.csv\0All files\0*.*\0", L"csv", L"Export dyno runs")) return;
    FILE* f = nullptr;
    if (_wfopen_s(&f, std::filesystem::u8path(path).wstring().c_str(), L"wb") != 0 || !f) return;   // UTF-8 path from the dialog
    const DynoCorrection corr = (DynoCorrection)dynoCorr_;
    std::fprintf(f, "# correction: %s\n", dynoCorrectionName(corr));
    std::fprintf(f, "run,label,engine,calibration,calibration_revision,ambient_C,baro_kPa,correction_factor,rpm,torque_Nm,torque_lbft,"
                    "power_kW,power_hp,afr,ve,boost_bar,egt_C,advance_deg,iat_C,bsfc_g_kWh\n");
    for (size_t i = 0; i < runs_.size(); ++i) {
        const DynoRun& r = runs_[i];
        if (!r.visible) continue;
        auto cell = [](std::string s) { for (char& c : s) if (c == ',' || c == '"' || c == '\n' || c == '\r') c = ';'; return s; };
        const std::string label = cell(r.label), engine = cell(r.engine), cal = cell(r.calName);
        const float cf = (float)dynoCorrectionFactor(corr, r.ambientC, r.baroKPa);
        for (const DynoPoint& d : r.pts)
            std::fprintf(f, "%zu,%s,%s,%s,%d,%.1f,%.2f,%.4f,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.0f,%.1f,%.1f,%.0f\n", i + 1, label.c_str(),
                         engine.c_str(), cal.c_str(), r.calRev, r.ambientC, r.baroKPa, cf, d.rpm, d.torque * cf, d.torque * cf * 0.737562f,
                         d.powerKw * cf, d.powerKw * cf * 1.341022f, d.afr, d.ve, d.boostBar, d.egtC, d.advance, d.iatC, d.bsfc);
    }
    std::fclose(f);
}

void App::sectionDataLogging() {
    const float em = ImGui::GetFontSize();
    if (logPanels_.empty()) {
        logPanels_ = {{logChannelIndex("rpm")},
                      {logChannelIndex("map"), logChannelIndex("throttle")},
                      {logChannelIndex("afr"), logChannelIndex("afr_target")},
                      {logChannelIndex("advance"), logChannelIndex("knock_retard")}};
        logExport_.assign(logChannelCount(), 1);
    }
    // ------------------------------------------------------------------ left: recording, sessions, channels
    ImGui::BeginChild("##log_left", ImVec2(ImGui::GetContentRegionAvail().x * 0.30f, 0), ImGuiChildFlags_Border);
    ImGui::SeparatorText("Recording");
    if (logRec_ < 0) {
        if (ImGui::Button("Record [R]", ImVec2(em * 7, 0))) logStart();
        ImGui::SameLine();
        ImGui::SetNextItemWidth(em * 6);
        ImGui::Combo("rate", &logRateIdx_, "10 Hz\0" "20 Hz\0" "50 Hz\0" "100 Hz\0");
    } else {
        const LogSession& r = logSessions_[logRec_];
        if (ImGui::Button("Stop [R]", ImVec2(em * 7, 0))) logStop();
        ImGui::SameLine();
        const bool blink = std::fmod(ImGui::GetTime(), 1.0) < 0.6;
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, blink ? 1.0f : 0.4f), "REC");
        ImGui::SameLine();
        ImGui::Text("%.1f s  %zu samples @ %.0f Hz", r.duration(), r.size(), r.rateHz);
    }
    ImGui::SetNextItemWidth(-em * 7);
    ImGui::InputTextWithHint("##mk", "marker label", logMarkText_, sizeof(logMarkText_));
    ImGui::SameLine();
    ImGui::BeginDisabled(logRec_ < 0);
    if (ImGui::Button("Marker [M]")) { logMarker(logMarkText_[0] ? logMarkText_ : "marker"); }
    ImGui::EndDisabled();
    ImGui::TextDisabled("Automatic markers: protection actions, limp mode, gear changes, launch, nitrous on.");

    ImGui::SeparatorText("Sessions");
    if (ImGui::BeginTable("##sess", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerV, ImVec2(0, em * 9))) {
        ImGui::TableSetupColumn("Session", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Length", ImGuiTableColumnFlags_WidthFixed, em * 3.5f);
        ImGui::TableSetupColumn("Calibration", ImGuiTableColumnFlags_WidthFixed, em * 7);
        ImGui::TableHeadersRow();
        for (int i = 0; i < (int)logSessions_.size(); ++i) {
            const LogSession& s = logSessions_[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            std::string label = s.name + (i == logRec_ ? "  (recording)" : "") + (i == logCmp_ ? "  [compare]" : "") + "##s" + std::to_string(i);
            if (ImGui::Selectable(label.c_str(), logView_ == i, ImGuiSelectableFlags_SpanAllColumns)) logView_ = i;
            ImGui::TableNextColumn(); ImGui::Text("%.0f s", s.duration());
            ImGui::TableNextColumn(); ImGui::Text("%s r%d", s.calName.c_str(), s.calRev);
        }
        if (logSessions_.empty()) { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextDisabled("no sessions yet"); }
        ImGui::EndTable();
    }
    const bool sel = logView_ >= 0 && logView_ < (int)logSessions_.size();
    if (ImGui::Button("Import CSV...")) {
        std::string path;
        if (fileDialog(false, "", path, L"Logs (*.csv)\0*.csv\0All files\0*.*\0", L"csv", L"Import log")) {
            LogSession s; std::string err;
            if (readLogCsv(path, s, err)) {
                if (s.name.empty()) s.name = path.substr(path.find_last_of("\\/") + 1);
                logSessions_.push_back(std::move(s));
                logView_ = (int)logSessions_.size() - 1;
                logMsg_ = "Imported " + path;
            } else logMsg_ = "Import failed: " + err;
        }
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!sel);
    if (ImGui::Button("Export CSV...") && sel) {
        std::vector<int> cols;
        for (int c = 0; c < logChannelCount(); ++c) if (logExport_[c]) cols.push_back(c);
        std::string path, err;
        std::string suggested = logSessions_[logView_].name + ".csv";
        for (char& ch : suggested) if (std::strchr("<>:\"/\\|?*", ch)) ch = '_';
        if (fileDialog(true, suggested, path, L"Logs (*.csv)\0*.csv\0All files\0*.*\0", L"csv", L"Export log"))
            logMsg_ = writeLogCsv(path, logSessions_[logView_], cols, err) ? "Exported " + path : "Export failed: " + err;
    }
    ImGui::SameLine();
    if (ImGui::Button(logCmp_ == logView_ && sel ? "Uncompare" : "Compare") && sel) logCmp_ = logCmp_ == logView_ ? -1 : logView_;
    ImGui::SameLine();
    if (ImGui::Button("Delete") && sel && logView_ != logRec_) {
        logSessions_.erase(logSessions_.begin() + logView_);
        if (logCmp_ == logView_) logCmp_ = -1; else if (logCmp_ > logView_) --logCmp_;
        if (logRec_ > logView_) --logRec_;
        logView_ = std::min(logView_, (int)logSessions_.size() - 1);
    }
    ImGui::EndDisabled();
    if (sel) {
        LogSession& s = logSessions_[logView_];
        ImGui::TextDisabled("%s  |  %s  |  %s r%d  |  %zu markers", s.started.c_str(), s.engine.c_str(), s.calName.c_str(), s.calRev, s.markers.size());
        char nb[96]; std::snprintf(nb, sizeof(nb), "%s", s.name.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##rn", nb, sizeof(nb))) s.name = nb;
        char notes[512]; std::snprintf(notes, sizeof(notes), "%s", s.notes.c_str());
        if (ImGui::InputTextMultiline("##notes", notes, sizeof(notes), ImVec2(-1, em * 2.6f))) s.notes = notes;
    }
    if (!logMsg_.empty()) ImGui::TextWrapped("%s", logMsg_.c_str());

    ImGui::SeparatorText("Channels in the CSV export");
    if (ImGui::SmallButton("All")) logExport_.assign(logChannelCount(), 1);
    ImGui::SameLine();
    if (ImGui::SmallButton("None")) logExport_.assign(logChannelCount(), 0);
    ImGui::BeginChild("##chans", ImVec2(0, 0));
    const char* group = "";
    for (int c = 0; c < logChannelCount(); ++c) {
        const LogChannel& lc = logChannel(c);
        if (std::strcmp(group, lc.group) != 0) { group = lc.group; ImGui::TextDisabled("%s", group); }
        bool on = logExport_[c] != 0;
        char lb[96]; std::snprintf(lb, sizeof(lb), "%s%s%s%s##ex%d", lc.name, *lc.unit ? " [" : "", lc.unit, *lc.unit ? "]" : "", c);
        if (ImGui::Checkbox(lb, &on)) logExport_[c] = on;
    }
    ImGui::EndChild();
    ImGui::EndChild();

    // ------------------------------------------------------------------ right: graphs
    ImGui::SameLine();
    ImGui::BeginChild("##log_right", ImVec2(0, 0), ImGuiChildFlags_Border);
    ImGui::TextUnformatted("X axis");
    ImGui::SameLine();
    int xr = logXRpm_ ? 1 : 0;
    if (ImGui::RadioButton("time", xr == 0)) logXRpm_ = false;
    ImGui::SameLine();
    if (ImGui::RadioButton("rpm", xr == 1)) logXRpm_ = true;
    ImGui::SameLine();
    int np = (int)logPanels_.size();
    ImGui::SetNextItemWidth(em * 5);
    if (ImGui::SliderInt("panels", &np, 1, 6)) logPanels_.resize(np);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) logFitReq_ = true;
    for (int pi = 0; pi < (int)logPanels_.size(); ++pi) {
        ImGui::SameLine();
        char lb[24]; std::snprintf(lb, sizeof(lb), "Panel %d...", pi + 1);
        if (ImGui::SmallButton(lb)) ImGui::OpenPopup(("##pp" + std::to_string(pi)).c_str());
        if (ImGui::BeginPopup(("##pp" + std::to_string(pi)).c_str())) {
            std::vector<int>& chans = logPanels_[pi];
            const char* g = "";
            for (int c = 0; c < logChannelCount(); ++c) {
                const LogChannel& lc = logChannel(c);
                if (std::strcmp(g, lc.group) != 0) { g = lc.group; ImGui::SeparatorText(g); }
                bool on = std::find(chans.begin(), chans.end(), c) != chans.end();
                if (ImGui::Checkbox((std::string(lc.name) + "##pc" + std::to_string(c)).c_str(), &on)) {
                    if (on) chans.push_back(c); else chans.erase(std::remove(chans.begin(), chans.end(), c), chans.end());
                }
            }
            ImGui::EndPopup();
        }
    }
    if (!sel) {
        ImGui::Spacing();
        ImGui::TextDisabled("Press Record (or R) to start logging, then pick the session to see its graphs.");
        ImGui::TextDisabled("Zoom: mouse wheel / drag a box with the right button, pan: drag, double-click: fit.");
        ImGui::EndChild();
        return;
    }
    const LogSession& s = logSessions_[logView_];
    const LogSession* cmp = logCmp_ >= 0 && logCmp_ < (int)logSessions_.size() && logCmp_ != logView_ ? &logSessions_[logCmp_] : nullptr;
    if (cmp) ImGui::TextDisabled("Overlay (faint): %s  (%s r%d)", cmp->name.c_str(), cmp->calName.c_str(), cmp->calRev);
    const int rpmCh = logChannelIndex("rpm");
    // x range: follow the last 30 s while recording, the whole session when it is picked / on Fit / axis change
    if (logView_ != logFitView_ || logXRpm_ != logFitRpm_) { logFitReq_ = true; logFitView_ = logView_; logFitRpm_ = logXRpm_; }
    const bool live = logView_ == logRec_;
    double x0 = 0, x1 = 1;
    if (!s.t.empty()) {
        if (logXRpm_) {
            x0 = 1e9; x1 = 0;
            for (float r : s.ch[rpmCh]) { x0 = std::min(x0, (double)r); x1 = std::max(x1, (double)r); }
            x0 = std::max(0.0, x0 - 100); x1 += 100;
        } else {
            x1 = std::max(1.0, s.t.back());
            x0 = live ? std::max(0.0, x1 - 30.0) : 0.0;
        }
    }
    const bool setX = logFitReq_ || live;
    if (ImPlot::BeginSubplots("##logplots", (int)logPanels_.size(), 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkAllX)) {
        for (int pi = 0; pi < (int)logPanels_.size(); ++pi) {
            if (!ImPlot::BeginPlot(("##lp" + std::to_string(pi)).c_str())) continue;
            ImPlot::SetupAxes(logXRpm_ ? "rpm" : "s", nullptr, 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
            if (setX) ImPlot::SetupAxisLimits(ImAxis_X1, x0, x1, ImPlotCond_Always);
            ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
            auto draw = [&](const LogSession& d, bool faint) {
                if (d.t.empty() || (int)d.ch.size() != logChannelCount()) return;
                const int n = (int)d.t.size();
                static std::vector<double> xs, ys;
                xs.resize(n); ys.resize(n);
                for (int k = 0; k < n; ++k) xs[k] = logXRpm_ ? d.ch[rpmCh][k] : d.t[k];
                for (int c : logPanels_[pi]) {
                    if (c < 0 || c >= logChannelCount()) continue;
                    const LogChannel& lc = logChannel(c);
                    for (int k = 0; k < n; ++k) ys[k] = d.ch[c][k];
                    char lb[96];
                    std::snprintf(lb, sizeof(lb), "%s%s%s%s%s", lc.name, *lc.unit ? " [" : "", lc.unit, *lc.unit ? "]" : "", faint ? " (cmp)" : "");
                    if (faint) ImPlot::SetNextLineStyle(ImVec4(0.7f, 0.7f, 0.7f, 0.45f), 1.0f);
                    if (logXRpm_) { ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.5f); ImPlot::PlotScatter(lb, xs.data(), ys.data(), n); }
                    else ImPlot::PlotLine(lb, xs.data(), ys.data(), n);
                }
            };
            if (cmp) draw(*cmp, true);
            draw(s, false);
            if (!logXRpm_)
                for (size_t m = 0; m < s.markers.size(); ++m) {
                    double x = s.markers[m].t;
                    ImPlot::SetNextLineStyle(ImVec4(1, 0.75f, 0.3f, 0.8f));
                    ImPlot::PlotInfLines(("##mk" + std::to_string(m)).c_str(), &x, 1);
                    if (pi == 0) ImPlot::TagX(x, ImVec4(1, 0.75f, 0.3f, 1), "%s", s.markers[m].label.c_str());
                }
            ImPlot::EndPlot();
        }
        ImPlot::EndSubplots();
    }
    logFitReq_ = false;
    ImGui::EndChild();
}

} // namespace es
