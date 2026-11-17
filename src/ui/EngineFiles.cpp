// Engine files in the UI: the preset list with your own engines, save / export / import / delete.
#include "ui/App.h"
#include "ui/Widgets.h"
#include "ui/FileDialog.h"
#include "imgui.h"
#include <cstdio>
#include <cstring>

namespace es {
using namespace ui;

// switch to another engine (a preset or a file); the test bench (key, throttle, listener, dyno, ambient air,
// driving mode and gearbox choice) stays as it is
void App::switchEngine(const EngineParams& next, int presetIdx, bool fileCalibration) {
    EngineParams keep = p_;
    stopBaseMap();
    p_ = next;
    p_.ignition = keep.ignition;
    p_.throttle = keep.throttle;
    p_.listener = keep.listener;
    p_.volume = keep.volume;
    p_.oversample = keep.oversample;
    p_.dynoMode = DynoMode::Off;
    p_.sweepRequest = keep.sweepRequest;
    p_.ambientTemp_C = keep.ambientTemp_C;
    p_.baro_kPa = keep.baro_kPa; p_.coolingFan_kmh = keep.coolingFan_kmh;
    p_.stepRpm = keep.stepRpm; p_.stepHold_s = keep.stepHold_s;
    p_.vehicle = keep.vehicle;
    p_.transMode = keep.transMode;
    p_.gear = keep.transMode == TransMode::Automatic ? keep.gear : 0;
    p_.environment = keep.environment; p_.reverbMix = keep.reverbMix;
    std::memcpy(p_.sensors, keep.sensors, sizeof(p_.sensors));
    presetIdx_ = presetIdx;
    resetPeak();
    baseMapInfo_.clear();
    calOnPresetChanged();
    if (fileCalibration) {
        // the file brings its own (tuned) calibration: no base map measured over its VE maps
        calSaved_ = p_.cal;
        bmKeyApplied_ = bmKeySeen_ = airPathKey(p_);
        bmVeApplied_ = p_.cal.veMap; bmTpsApplied_ = p_.cal.veTps; bmStale_ = false;
        baseMapInfo_ = "calibration from the engine file";
    } else startBaseMap();
    equalPrimaries_ = true;
    for (int c = 1; c < p_.cylinders; ++c) if (p_.primaryLen_mm[c] != p_.primaryLen_mm[0]) equalPrimaries_ = false;
}

void App::loadPreset(int i) { switchEngine(makePreset(i), i, false); }

bool App::loadEngineFrom(const std::string& path, bool keepCopy) {
    EngineParams q = p_;
    EngineMeta m;
    std::string err;
    if (!loadEngineFile(path, q, m, err)) { engMsg_ = "Not loaded: " + err; engMsgErr_ = true; return false; }
    if (keepCopy) {   // an imported engine joins "My engines"
        std::string e2;
        if (!saveEngineFile(q, m, e2)) { engMsg_ = "Loaded, but not copied to My engines: " + e2; engMsgErr_ = true; }
        engListDirty_ = true;
    }
    switchEngine(q, -1, m.hasCalibration);
    std::snprintf(engAuthor_, sizeof(engAuthor_), "%s", m.author.c_str());
    std::snprintf(engNotes_, sizeof(engNotes_), "%s", m.notes.c_str());
    if (!engMsgErr_ || !keepCopy)
        engMsg_ = std::string(keepCopy ? "Imported '" : "Loaded '") + q.name + "'" + (m.author.empty() ? "" : " by " + m.author) +
                  (m.hasCalibration ? " with its calibration" : " (base calibration generated)");
    engMsgErr_ = false;
    return true;
}

const char* App::engineLabel() const { return presetIdx_ >= 0 ? presetName(presetIdx_) : p_.name; }

// the engine list: built-in presets, then your own engines
void App::presetCombo(const char* id) {
    if (engListDirty_) { engList_ = listEngineFiles(); engListDirty_ = false; }
    if (ImGui::BeginCombo(id, engineLabel())) {
        for (int i = 0; i < presetCount(); ++i)
            if (ImGui::Selectable(presetName(i), i == presetIdx_)) loadPreset(i);
        if (!engList_.empty()) {
            ImGui::SeparatorText("My engines");
            for (size_t k = 0; k < engList_.size(); ++k) {
                const EngineFileInfo& f = engList_[k];
                ImGui::PushID((int)k);
                ImGui::BeginDisabled(!f.readable);
                if (ImGui::Selectable(f.name.c_str(), presetIdx_ < 0 && f.name == p_.name)) loadEngineFrom(f.path, false);
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered() && (!f.author.empty() || !f.notes.empty()))
                    ImGui::SetTooltip("%s%s%s", f.author.empty() ? "" : ("by " + f.author).c_str(), f.author.empty() || f.notes.empty() ? "" : "\n",
                                      f.notes.c_str());
                ImGui::PopID();
            }
        }
        ImGui::EndCombo();
    }
}

// Engine Setup > Engine: save as your own engine, export / import a file to share
void App::engineFileUi() {
    if (ImGui::CollapsingHeader("Save / share this engine")) {
        ImGui::InputText("Author", engAuthor_, sizeof(engAuthor_));
        ImGui::InputTextMultiline("Notes", engNotes_, sizeof(engNotes_), ImVec2(-170, ImGui::GetTextLineHeight() * 3.2f));
        EngineMeta m;
        m.author = engAuthor_; m.notes = engNotes_; m.app = "EngineSim 0.6";
        if (ImGui::Button("Save to My engines")) {
            std::string err, path;
            if (saveEngineFile(p_, m, err, &path)) { engMsg_ = std::string("Saved '") + p_.name + "' to My engines"; engMsgErr_ = false; presetIdx_ = -1; }
            else { engMsg_ = "Not saved: " + err; engMsgErr_ = true; }
            engListDirty_ = true;
        }
        Help("Stores the whole engine - hardware, car, sound character and the current ECU calibration - under its\n"
             "name (Name above) in the engines folder next to the exe. It then appears under My engines in the\n"
             "preset list. Saving under the same name replaces it.");
        ImGui::SameLine();
        if (ImGui::Button("Export file...")) {
            std::string path, err;
            std::string suggested = engineFolderName(p_.name) + ".engine.json";
            if (fileDialog(true, suggested, path, L"EngineSim engine (*.engine.json)\0*.engine.json\0JSON (*.json)\0*.json\0", L"json", L"Export engine")) {
                if (exportEngineFile(path, p_, m, err)) { engMsg_ = "Exported to " + path; engMsgErr_ = false; }
                else { engMsg_ = "Not exported: " + err; engMsgErr_ = true; }
            }
        }
        Help("A single file to share: someone else imports it and gets the same engine with your calibration.");
        ImGui::SameLine();
        if (ImGui::Button("Import file...")) {
            std::string path;
            if (fileDialog(false, "", path, L"EngineSim engine (*.engine.json)\0*.engine.json\0JSON (*.json)\0*.json\0All files\0*.*\0", L"json",
                           L"Import engine"))
                loadEngineFrom(path, true);
        }
        Help("Loads an engine file and keeps a copy under My engines. Test-bench settings (throttle, dyno, ambient\n"
             "air, sensor faults) are not part of a file and stay as they are.");
        if (presetIdx_ < 0) {
            for (const EngineFileInfo& f : engList_)
                if (f.name == p_.name) {
                    ImGui::SameLine();
                    if (ImGui::Button("Delete from My engines")) ImGui::OpenPopup("##deleng");
                    if (ImGui::BeginPopup("##deleng")) {
                        ImGui::Text("Delete '%s' from My engines? (exported files are not touched)", f.name.c_str());
                        if (ImGui::Button("Delete")) {
                            deleteEngineFile(f.path);
                            engListDirty_ = true;
                            engMsg_ = "Deleted '" + f.name + "' from My engines"; engMsgErr_ = false;
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
                        ImGui::EndPopup();
                    }
                    break;
                }
        }
        if (!engMsg_.empty())
            ImGui::TextColored(engMsgErr_ ? ImVec4(1, 0.45f, 0.35f, 1) : ImVec4(0.55f, 0.85f, 0.55f, 1), "%s", engMsg_.c_str());
        ImGui::TextDisabled("Folder: %s", engineDir().c_str());
    }
}

} // namespace es
