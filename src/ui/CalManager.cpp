// Calibration Manager section: calibration files per engine (JSON), version history / restore,
// compare against a reference, notes, map locking and validation.
#include "ui/App.h"
#include "ecu/CalibrationIO.h"
#include "imgui.h"
#include <windows.h>
#include <commdlg.h>
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace es {

static std::string toUtf8(const wchar_t* w) {
    int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    std::string s(n > 0 ? n - 1 : 0, '\0');
    if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
    return s;
}

static bool fileDialog(bool save, const std::string& suggested, std::string& out) {
    wchar_t file[MAX_PATH] = L"";
    MultiByteToWideChar(CP_UTF8, 0, suggested.c_str(), -1, file, MAX_PATH);
    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = GetActiveWindow();
    ofn.lpstrFilter = L"Calibration (*.json)\0*.json\0All files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = L"json";
    ofn.lpstrTitle = save ? L"Export calibration" : L"Import calibration";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR | (save ? OFN_OVERWRITEPROMPT : OFN_FILEMUSTEXIST);
    if (!(save ? GetSaveFileNameW(&ofn) : GetOpenFileNameW(&ofn))) return false;
    out = toUtf8(file);
    return true;
}

void App::calMessage(int severity, const std::string& s) { calMsg_ = s; calMsgSev_ = severity; }

void App::calRefresh() {
    calList_ = listCalibrations(p_.name);
    calHist_ = calMeta_.name.empty() ? std::vector<CalFileInfo>{} : listHistory(p_.name, calMeta_.name);
    calListEngine_ = p_.name;
    calListDirty_ = false;
}

// a newly chosen engine starts with an unsaved calibration
void App::calOnPresetChanged() {
    calMeta_ = CalMeta{};
    calSaved_ = p_.cal;
    calNotes_[0] = 0;
    calName_[0] = 0;
    calListDirty_ = true;
    calSel_ = -1;
    if (hasRef_ && refName_ != "generated base") hasRef_ = false;
}

void App::calApplyLoaded(const Calibration& c, const CalMeta& m, const char* what) {
    p_.cal = c;
    calMeta_ = m;
    calSaved_ = c;
    std::snprintf(calName_, sizeof(calName_), "%s", m.name.c_str());
    std::snprintf(calNotes_, sizeof(calNotes_), "%s", m.notes.c_str());
    calListDirty_ = true;
    calIssues_ = validateCalibration(p_.cal, p_);
    int errs = 0;
    for (auto& i : calIssues_) errs += i.severity == 2;
    char b[256];
    std::snprintf(b, sizeof(b), "%s '%s' rev %d%s%s", what, m.name.c_str(), m.revision,
                  !m.engine.empty() && m.engine != p_.name ? " - WARNING: made for another engine (" : "",
                  !m.engine.empty() && m.engine != p_.name ? (m.engine + ")").c_str() : "");
    calMessage(errs ? 2 : (!m.engine.empty() && m.engine != p_.name ? 1 : 0), errs ? std::string(b) + " - validation found errors" : b);
}

bool App::calSave(const char* name) {
    CalMeta m = calMeta_;
    if (m.name != name) m.revision = 0;   // save-as / duplicate starts its own history
    m.name = name;
    m.notes = calNotes_;
    std::string err;
    if (!saveCalibrationFile(p_.name, m, p_.cal, err)) { calMessage(2, "Save failed: " + err); return false; }
    calMeta_ = m;
    calSaved_ = p_.cal;
    calListDirty_ = true;
    calMessage(0, "Saved '" + m.name + "' as revision " + std::to_string(m.revision));
    return true;
}

static void sevText(int sev, const char* s) {
    static const ImVec4 col[3] = {ImVec4(0.6f, 0.85f, 0.6f, 1), ImVec4(1, 0.78f, 0.35f, 1), ImVec4(1, 0.4f, 0.35f, 1)};
    static const char* tag[3] = {"info ", "warn ", "error"};
    ImGui::TextColored(col[std::clamp(sev, 0, 2)], "%s", tag[std::clamp(sev, 0, 2)]);
    ImGui::SameLine();
    ImGui::TextWrapped("%s", s);
}

void App::sectionCalibrationManager() {
    if (calListDirty_ || calListEngine_ != p_.name) calRefresh();
    const bool modified = std::memcmp(&p_.cal, &calSaved_, sizeof(Calibration)) != 0;
    const float em = ImGui::GetFontSize();

    // ------------------------------------------------------------------ left: files
    ImGui::BeginChild("##cal_files", ImVec2(ImGui::GetContentRegionAvail().x * 0.42f, 0), ImGuiChildFlags_Border);
    ImGui::SeparatorText("Current calibration");
    ImGui::TextDisabled("Engine");  ImGui::SameLine(em * 5); ImGui::TextUnformatted(p_.name);
    ImGui::TextDisabled("Loaded");  ImGui::SameLine(em * 5);
    if (calMeta_.name.empty()) ImGui::TextUnformatted("(unsaved - generated base map)");
    else ImGui::Text("%s  rev %d  %s", calMeta_.name.c_str(), calMeta_.revision, calMeta_.saved.c_str());
    if (modified) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.78f, 0.35f, 1), "* modified"); }
    if (p_.cal.autoTune) ImGui::TextDisabled("Auto-tune is on: learned VE changes count as modifications.");

    ImGui::SetNextItemWidth(-em * 9);
    ImGui::InputTextWithHint("##calname", "calibration name", calName_, sizeof(calName_));
    ImGui::SameLine();
    const bool nameOk = calName_[0] != 0;
    ImGui::BeginDisabled(!nameOk);
    if (ImGui::Button("Save", ImVec2(em * 4, 0))) calSave(calName_);
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("Writes calibrations/<engine>/<name>.json and keeps a numbered copy in the version history.\nA new name duplicates the calibration under that name (Save as).");
    ImGui::SameLine();
    if (ImGui::Button("Revert", ImVec2(em * 4, 0))) { p_.cal = calSaved_; calMessage(0, "Reverted to the last saved / loaded state"); }
    ImGui::TextDisabled("Notes");
    ImGui::InputTextMultiline("##calnotes", calNotes_, sizeof(calNotes_), ImVec2(-1, em * 4.5f));

    if (ImGui::Button("New from base map")) {
        generateCalibration(p_);
        calMeta_ = CalMeta{}; calName_[0] = 0; calNotes_[0] = 0;
        calSaved_ = p_.cal;
        baseMapInfo_.clear();
        startBaseMap();   // measured VE: from the cache, or measured in the background
        calMessage(0, "New calibration from the generated base map (VE from the virtual dyno)");
    }
    ImGui::SameLine();
    if (ImGui::Button("Import...")) {
        std::string path;
        if (fileDialog(false, "", path)) {
            Calibration c = p_.cal; CalMeta m; std::string err;
            if (loadCalibrationFile(path, c, m, err)) {
                if (m.name.empty()) m.name = "imported";
                m.revision = 0;   // its own history starts on the first save here
                const Calibration lastSaved = calSaved_;
                calApplyLoaded(c, m, "Imported");
                calSaved_ = lastSaved;   // not saved in this engine's folder yet: shows as modified, Revert undoes the import
            } else calMessage(2, "Import failed: " + err);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Export...")) {
        std::string path;
        CalMeta m = calMeta_; m.name = calName_[0] ? calName_ : "calibration"; m.notes = calNotes_; m.engine = p_.name;
        if (fileDialog(true, m.name + ".json", path)) {
            const bool ok = writeTextFile(path, calibrationToJson(p_.cal, m));
            calMessage(ok ? 0 : 2, ok ? "Exported to " + path : "Export failed: " + path);
        }
    }

    ImGui::SeparatorText("Saved calibrations for this engine");
    if (ImGui::SmallButton("Refresh")) calListDirty_ = true;
    ImGui::SameLine();
    ImGui::TextDisabled("%s", (calibrationDir() + "\\" + engineFolderName(p_.name)).c_str());
    if (ImGui::BeginTable("##callist", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY,
                          ImVec2(0, em * 9))) {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Rev", ImGuiTableColumnFlags_WidthFixed, em * 2.5f);
        ImGui::TableSetupColumn("Saved", ImGuiTableColumnFlags_WidthFixed, em * 9);
        ImGui::TableHeadersRow();
        for (int i = 0; i < (int)calList_.size(); ++i) {
            const CalFileInfo& f = calList_[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            const bool cur = f.name == calMeta_.name;
            if (ImGui::Selectable((f.name + (cur ? "  (loaded)" : "") + "##f" + std::to_string(i)).c_str(), calSel_ == i,
                                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
                calSel_ = i;
                if (ImGui::IsMouseDoubleClicked(0)) {
                    Calibration c = p_.cal; CalMeta m; std::string err;
                    if (loadCalibrationFile(f.path, c, m, err)) { m.name = f.name; calApplyLoaded(c, m, "Loaded"); }
                    else calMessage(2, "Load failed: " + err);
                }
            }
            ImGui::TableNextColumn(); ImGui::Text("%d", f.revision);
            ImGui::TableNextColumn(); ImGui::TextUnformatted(f.saved.c_str());
        }
        if (calList_.empty()) { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextDisabled("none saved yet"); }
        ImGui::EndTable();
    }
    const bool sel = calSel_ >= 0 && calSel_ < (int)calList_.size();
    ImGui::BeginDisabled(!sel);
    if (ImGui::Button("Load")) {
        Calibration c = p_.cal; CalMeta m; std::string err;
        if (loadCalibrationFile(calList_[calSel_].path, c, m, err)) { m.name = calList_[calSel_].name; calApplyLoaded(c, m, "Loaded"); }
        else calMessage(2, "Load failed: " + err);
    }
    ImGui::SameLine();
    if (ImGui::Button("Duplicate")) {
        Calibration c = p_.cal; CalMeta m; std::string err;
        if (loadCalibrationFile(calList_[calSel_].path, c, m, err)) {
            std::string base = calList_[calSel_].name + " copy", nn = base;
            for (int k = 2; std::any_of(calList_.begin(), calList_.end(), [&](const CalFileInfo& x) { return x.name == nn; }); ++k)
                nn = base + " " + std::to_string(k);
            Calibration keep = p_.cal;
            p_.cal = c; m.name = nn; m.revision = 0; calMeta_ = m;
            std::snprintf(calNotes_, sizeof(calNotes_), "%s", m.notes.c_str());
            if (calSave(nn.c_str())) std::snprintf(calName_, sizeof(calName_), "%s", nn.c_str());
            else p_.cal = keep;
        } else calMessage(2, "Duplicate failed: " + err);
    }
    ImGui::SameLine();
    if (ImGui::Button("Compare")) {
        Calibration c = p_.cal; CalMeta m; std::string err;
        if (loadCalibrationFile(calList_[calSel_].path, c, m, err)) {
            refCal_ = c; hasRef_ = true; refName_ = calList_[calSel_].name + " r" + std::to_string(m.revision);
            calTab_ = 1; calTabForce_ = true;
            calMessage(0, "Reference set: " + refName_ + " (the table editors' Diff view uses it too)");
        } else calMessage(2, "Load failed: " + err);
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete...")) ImGui::OpenPopup("Delete calibration?");
    ImGui::EndDisabled();
    if (ImGui::BeginPopupModal("Delete calibration?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (sel) ImGui::Text("Delete '%s' and its whole version history?\nThis cannot be undone.", calList_[calSel_].name.c_str());
        if (ImGui::Button("Delete", ImVec2(em * 6, 0)) && sel) {
            const std::string n = calList_[calSel_].name;
            const bool ok = deleteCalibrationFile(p_.name, n);
            calMessage(ok ? 0 : 2, ok ? "Deleted '" + n + "'" : "Could not delete '" + n + "'");
            if (calMeta_.name == n) { calMeta_.name.clear(); calMeta_.revision = 0; }
            calSel_ = -1; calListDirty_ = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(em * 6, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (!calMsg_.empty()) { ImGui::Spacing(); sevText(calMsgSev_, calMsg_.c_str()); }
    ImGui::EndChild();

    // ------------------------------------------------------------------ right: history / compare / validation / locks
    ImGui::SameLine();
    ImGui::BeginChild("##cal_side", ImVec2(0, 0), ImGuiChildFlags_Border);
    if (ImGui::BeginTabBar("##caltabs")) {
        const char* names[4] = {"Version history", "Compare", "Validation", "Map locks"};
        for (int t = 0; t < 4; ++t) {
            ImGuiTabItemFlags fl = calTab_ == t && calTabForce_ ? ImGuiTabItemFlags_SetSelected : 0;
            if (!ImGui::BeginTabItem(names[t], nullptr, fl)) continue;
            if (!calTabForce_) calTab_ = t;
            if (t == 0) {
                if (calMeta_.name.empty()) ImGui::TextDisabled("Save the calibration (or load one) to start its version history.");
                else {
                    ImGui::Text("History of '%s' (%d revisions)", calMeta_.name.c_str(), (int)calHist_.size());
                    if (ImGui::BeginTable("##hist", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY)) {
                        ImGui::TableSetupColumn("Rev", ImGuiTableColumnFlags_WidthFixed, em * 3);
                        ImGui::TableSetupColumn("Saved", ImGuiTableColumnFlags_WidthFixed, em * 9);
                        ImGui::TableSetupColumn("Notes", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("##act", ImGuiTableColumnFlags_WidthFixed, em * 9);
                        ImGui::TableHeadersRow();
                        for (int i = 0; i < (int)calHist_.size(); ++i) {
                            const CalFileInfo& h = calHist_[i];
                            ImGui::PushID(i);
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn(); ImGui::Text("r%d", h.revision);
                            ImGui::TableNextColumn(); ImGui::TextUnformatted(h.saved.c_str());
                            ImGui::TableNextColumn();
                            const bool ok = h.readable;
                            std::string first = h.notes.substr(0, h.notes.find('\n'));
                            ImGui::TextUnformatted(ok ? first.c_str() : "(unreadable)");
                            if (ok && !h.notes.empty() && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", h.notes.c_str());
                            ImGui::TableNextColumn();
                            ImGui::BeginDisabled(!ok);
                            const bool restore = ImGui::SmallButton("Restore");
                            ImGui::SameLine();
                            const bool compare = ImGui::SmallButton("Compare");
                            Calibration c = p_.cal; CalMeta m; std::string err;
                            if ((restore || compare) && !loadCalibrationFile(h.path, c, m, err)) calMessage(2, "Load failed: " + err);
                            else if (restore) {
                                const Calibration lastSaved = calSaved_;
                                m.name = calMeta_.name; m.revision = calMeta_.revision;
                                calApplyLoaded(c, m, "Restored");
                                calSaved_ = lastSaved;   // restoring is a change until saved as a new revision
                                calMessage(0, "Restored r" + std::to_string(h.revision) + " - Save to make it the newest revision");
                            } else if (compare) {
                                refCal_ = c; hasRef_ = true; refName_ = calMeta_.name + " r" + std::to_string(h.revision);
                                calTab_ = 1; calTabForce_ = true;
                            }
                            ImGui::EndDisabled();
                            ImGui::PopID();
                        }
                        ImGui::EndTable();
                    }
                }
            } else if (t == 1) {
                if (ImGui::Button("Reference: generated base")) {
                    EngineParams g = p_; generateCalibration(g);
                    refCal_ = g.cal; hasRef_ = true; refName_ = "generated base";
                }
                ImGui::SameLine();
                if (ImGui::Button("Reference: last saved")) { refCal_ = calSaved_; hasRef_ = true; refName_ = "last saved"; }
                ImGui::SameLine();
                ImGui::BeginDisabled(!hasRef_);
                if (ImGui::Button("Clear")) hasRef_ = false;
                ImGui::EndDisabled();
                if (!hasRef_) ImGui::TextDisabled("No reference. Pick one here, or Compare a saved file / history revision.");
                else {
                    ImGui::Text("Current calibration vs '%s'", refName_.c_str());
                    ImGui::TextDisabled("Table editors show the difference with their Diff button.");
                    if (ImGui::BeginTable("##cmp", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
                        ImGui::TableSetupColumn("Map / curve", ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn("Changed", ImGuiTableColumnFlags_WidthFixed, em * 5);
                        ImGui::TableSetupColumn("Max |d|", ImGuiTableColumnFlags_WidthFixed, em * 5);
                        ImGui::TableSetupColumn("Mean |d|", ImGuiTableColumnFlags_WidthFixed, em * 5);
                        ImGui::TableSetupColumn("Axes", ImGuiTableColumnFlags_WidthFixed, em * 5);
                        ImGui::TableHeadersRow();
                        auto row = [&](const TableInfo& in, const CalDiff& d) {
                            ImGui::TableNextRow();
                            ImGui::TableNextColumn();
                            if (d.changed) ImGui::TextColored(ImVec4(1, 0.78f, 0.35f, 1), "%s", in.name); else ImGui::TextDisabled("%s", in.name);
                            ImGui::TableNextColumn(); ImGui::Text("%d / %d", d.changed, d.cells);
                            ImGui::TableNextColumn(); ImGui::Text("%.3g %s", d.maxAbs, in.unit);
                            ImGui::TableNextColumn(); ImGui::Text("%.3g", d.meanAbs);
                            ImGui::TableNextColumn(); ImGui::TextUnformatted(d.axesDiffer ? "differ" : "same");
                        };
                        for (int i = 0; i < kCalTableCount; ++i) row(calTableInfo(i), diffTable(calTable(refCal_, i), calTable(p_.cal, i)));
                        for (int i = 0; i < kCalCurveCount; ++i) row(calCurveInfo(i), diffCurve(calCurve(refCal_, i), calCurve(p_.cal, i)));
                        ImGui::EndTable();
                    }
                    std::vector<std::string> sc;
                    diffScalars(refCal_, p_.cal, &sc);
                    ImGui::SeparatorText("Settings");
                    if (sc.empty()) ImGui::TextDisabled("no setting differs");
                    for (auto& s : sc) ImGui::BulletText("%s", s.c_str());
                }
            } else if (t == 2) {
                if (ImGui::Button("Validate now") || calIssues_.empty()) calIssues_ = validateCalibration(p_.cal, p_);
                ImGui::SameLine();
                int e = 0, w = 0;
                for (auto& i : calIssues_) { e += i.severity == 2; w += i.severity == 1; }
                ImGui::Text("%d error(s), %d warning(s)", e, w);
                ImGui::TextDisabled("Checks axes, value ranges, neighbour jumps (typos), rpm coverage, injector data, trims.");
                ImGui::Separator();
                for (auto& i : calIssues_) sevText(i.severity, i.text.c_str());
            } else {
                ImGui::TextWrapped("A locked map cannot be edited and is not changed by auto-tune or the measured base map.");
                if (ImGui::Button("Lock all")) {
                    for (int i = 0; i < kCalTableCount; ++i) calTable(p_.cal, i).locked = 1;
                    for (int i = 0; i < kCalCurveCount; ++i) calCurve(p_.cal, i).locked = 1;
                }
                ImGui::SameLine();
                if (ImGui::Button("Unlock all")) {
                    for (int i = 0; i < kCalTableCount; ++i) calTable(p_.cal, i).locked = 0;
                    for (int i = 0; i < kCalCurveCount; ++i) calCurve(p_.cal, i).locked = 0;
                }
                ImGui::Separator();
                for (int i = 0; i < kCalTableCount; ++i) {
                    bool l = calTable(p_.cal, i).locked != 0;
                    if (ImGui::Checkbox(calTableInfo(i).name, &l)) calTable(p_.cal, i).locked = l;
                }
                for (int i = 0; i < kCalCurveCount; ++i) {
                    bool l = calCurve(p_.cal, i).locked != 0;
                    if (ImGui::Checkbox(calCurveInfo(i).name, &l)) calCurve(p_.cal, i).locked = l;
                }
            }
            ImGui::EndTabItem();
        }
        calTabForce_ = false;
        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

} // namespace es
