// Engine files in the UI: the preset list with your own engines, save / export / import / delete.
#include "ui/App.h"
#include "ui/Widgets.h"
#include "ui/FileDialog.h"
#include "imgui.h"
#include <cstdio>
#include <algorithm>
#include <cctype>
#include <cfloat>
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

// a preset's one-line summary in the list: "V8  5.7 L  carb", "2-rotor  1.3 L  turbo  EFI", "1-cyl  250 cc  two-stroke"
static std::string presetBrief(const EngineParams& q) {
    char a[24], b[24];
    const int n = q.cylinders;
    if (isRotary(q)) std::snprintf(a, sizeof(a), "%d-rotor", n);
    else if (n == 1) std::snprintf(a, sizeof(a), "1-cyl");
    else if (q.layout == Layout::V) std::snprintf(a, sizeof(a), "V%d", n);
    else if (q.layout == Layout::Boxer) std::snprintf(a, sizeof(a), "Flat-%d", n);
    else std::snprintf(a, sizeof(a), "I%d", n);
    const double L = ratedDisplacementL(q);
    if (L < 0.995) std::snprintf(b, sizeof(b), "%.0f cc", L * 1000.0);
    else std::snprintf(b, sizeof(b), "%.1f L", L);
    std::string s = std::string(a) + "  " + b;
    if (q.induction == Induction::Turbo) s += "  turbo";
    else if (q.induction == Induction::Supercharger)
        s += q.scType == Supercharger::Centrifugal ? "  centrifugal" : q.scType == Supercharger::TwinScrew ? "  twin-screw" : "  Roots";
    if (isDiesel(q)) s += "  diesel";
    else if (isTwoStroke(q)) s += q.fuelSystem == FuelSystem::EFI ? "  two-stroke EFI" : "  two-stroke";
    else s += q.fuelSystem == FuelSystem::Carburetor ? "  carb" : q.itb ? "  ITB" : "  EFI";
    return s;
}

// every word of the search in the text (case-insensitive)
static bool matchesAll(const std::string& text, const char* find) {
    auto lower = [](std::string s) { for (char& c : s) c = (char)std::tolower((unsigned char)c); return s; };
    const std::string hay = lower(text);
    std::string word;
    for (const char* c = find;; ++c) {
        if (*c == 0 || *c == ' ') {
            if (!word.empty() && hay.find(lower(word)) == std::string::npos) return false;
            word.clear();
            if (*c == 0) return true;
        } else word += *c;
    }
}

// the engine list: New engine, the presets by group (with a search), then your own engines
void App::presetCombo(const char* id) {
    if (engListDirty_) { engList_ = listEngineFiles(); engListDirty_ = false; }
    if (presetBrief_.empty()) {
        const int n = presetCount();
        presetBrief_.resize(n);
        std::vector<double> size(n);
        std::vector<int> cyl(n);
        for (int i = 0; i < n; ++i) {
            const EngineParams q = makePreset(i);
            presetBrief_[i] = presetBrief(q);
            size[i] = ratedDisplacementL(q); cyl[i] = q.cylinders;
        }
        presetOrder_.resize(n);
        for (int i = 0; i < n; ++i) presetOrder_[i] = i;
        std::stable_sort(presetOrder_.begin(), presetOrder_.end(), [&](int x, int y) {
            if (presetGroup(x) != presetGroup(y)) return presetGroup(x) < presetGroup(y);
            if (cyl[x] != cyl[y]) return cyl[x] < cyl[y];
            return size[x] < size[y];
        });
    }
    const float em = ImGui::GetFontSize();
    ImGui::SetNextWindowSizeConstraints(ImVec2(em * 30.0f, 0), ImVec2(FLT_MAX, em * 36.0f));
    if (ImGui::BeginCombo(id, engineLabel(), ImGuiComboFlags_HeightLargest)) {
        if (ImGui::IsWindowAppearing()) { presetFind_[0] = 0; ImGui::SetKeyboardFocusHere(); }
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputTextWithHint("##find", "Search: V8, diesel, turbo, 2.0, rotary...", presetFind_, sizeof(presetFind_));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.62f, 0.25f, 1));
        if (ImGui::Selectable("+  New engine from scratch...")) newEngineReq_ = true;
        ImGui::PopStyleColor();
        const float right = ImGui::GetContentRegionMax().x;
        int lastGroup = -1, shown = 0;
        for (int i : presetOrder_) {
            const int g = presetGroup(i);
            if (presetFind_[0] && !matchesAll(std::string(presetName(i)) + " " + presetBrief_[i] + " " + presetGroupName(g), presetFind_)) continue;
            if (g != lastGroup) { ImGui::SeparatorText(presetGroupName(g)); lastGroup = g; }
            ImGui::PushID(i);
            if (ImGui::Selectable(presetName(i), i == presetIdx_)) loadPreset(i);
            if (i == presetIdx_ && ImGui::IsWindowAppearing()) ImGui::SetScrollHereY();
            const char* brief = presetBrief_[i].c_str();
            ImGui::SameLine(std::max(ImGui::GetCursorPosX(), right - ImGui::CalcTextSize(brief).x));
            ImGui::TextDisabled("%s", brief);
            ImGui::PopID();
            ++shown;
        }
        if (presetFind_[0] && shown == 0) ImGui::TextDisabled("No preset matches '%s'", presetFind_);
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

// New engine: build one from scratch - the kind, size and a few build choices; bore, stroke, valves, cams, intake,
// exhaust, the car and the base calibration are sized from them (sim/Presets.cpp, buildEngine). A live summary shows
// what comes out; everything stays editable in Engine Setup afterwards.
void App::newEngineUi() {
    if (newEngineReq_) {
        newEngineReq_ = false;
        ImGui::OpenPopup("New engine");
    }
    const float em = ImGui::GetFontSize();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(em * 38.0f, 0), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal("New engine", nullptr, ImGuiWindowFlags_NoSavedSettings)) return;
    EngineRecipe& r = recipe_;
    ImGui::TextDisabled("Pick the basics: the rest is sized from them. Everything stays editable in Engine Setup.");
    ImGui::PushItemWidth(-em * 11.0f);

    // kind
    {
        static const char* kKinds[] = {"Petrol", "Diesel", "Two-stroke", "Rotary"};
        for (int k = 0; k < 4; ++k) {
            if (k) ImGui::SameLine();
            if (ImGui::RadioButton(kKinds[k], (int)r.kind == k) && (int)r.kind != k) {
                char keep[sizeof(r.name)];
                std::memcpy(keep, r.name, sizeof(keep));
                r = defaultRecipe((BuildKind)k);
                std::memcpy(r.name, keep, sizeof(keep));
            }
        }
    }
    const bool fourStroke = r.kind == BuildKind::Petrol || r.kind == BuildKind::Diesel;
    ImGui::SeparatorText("Size");
    if (fourStroke) {
        int lay = (int)r.layout;
        bool ch = ImGui::RadioButton("Inline", &lay, 0); ImGui::SameLine();
        ch |= ImGui::RadioButton("V (vee)", &lay, 1); ImGui::SameLine();
        ch |= ImGui::RadioButton("Boxer (flat)", &lay, 2);
        ImGui::SameLine(); ImGui::TextDisabled("  layout");
        if (ch) r.layout = (Layout)lay;
    }
    {
        int choice[16];
        const int nc = recipeCylinderChoices(r.kind, r.layout, choice, 16);
        bool in = false;
        for (int i = 0; i < nc; ++i) in |= choice[i] == r.cylinders;
        if (!in) {   // the nearest count this layout has
            int best = choice[0];
            for (int i = 0; i < nc; ++i) if (std::abs(choice[i] - r.cylinders) < std::abs(best - r.cylinders)) best = choice[i];
            r.cylinders = best;
        }
        char cur[16];
        std::snprintf(cur, sizeof(cur), "%d", r.cylinders);
        if (ImGui::BeginCombo(r.kind == BuildKind::Rotary ? "Rotors" : "Cylinders", cur)) {
            for (int i = 0; i < nc; ++i) {
                char lb[16];
                std::snprintf(lb, sizeof(lb), "%d", choice[i]);
                if (ImGui::Selectable(lb, choice[i] == r.cylinders)) {
                    // keep the size per cylinder when the count changes
                    r.displacement_L *= (float)choice[i] / (float)std::max(1, r.cylinders);
                    r.cylinders = choice[i];
                }
            }
            ImGui::EndCombo();
        }
    }
    {
        float lo, hi;
        recipeDisplacementRange(r.kind, r.cylinders, lo, hi);
        r.displacement_L = clampv(r.displacement_L, lo, hi);
        if (hi <= 2.0f) {
            float cc = r.displacement_L * 1000.0f;
            if (ImGui::SliderFloat("Displacement", &cc, lo * 1000.0f, hi * 1000.0f, "%.0f cc")) r.displacement_L = cc / 1000.0f;
        } else ImGui::SliderFloat("Displacement", &r.displacement_L, lo, hi, "%.2f L");
        ImGui::SameLine(); Help(r.kind == BuildKind::Rotary ? "All rotors together (a 13B: 2 x 654 cc = 1.3 L)." : "All cylinders together.");
    }
    if (r.kind != BuildKind::Rotary) {
        const float lo = r.kind == BuildKind::Diesel ? 0.75f : r.kind == BuildKind::TwoStroke ? 0.85f : 0.75f;
        const float hi = r.kind == BuildKind::Diesel ? 1.10f : r.kind == BuildKind::TwoStroke ? 1.30f : 1.40f;
        r.boreStroke = clampv(r.boreStroke, lo, hi);
        ImGui::SliderFloat("Bore / stroke", &r.boreStroke, lo, hi, r.boreStroke < 0.97f ? "%.2f  long stroke" : r.boreStroke > 1.03f ? "%.2f  oversquare" : "%.2f  square");
        ImGui::SameLine(); Help("Long stroke (below 1): torque low down, a lower rev limit. Oversquare (above 1): room for bigger\n"
                                "valves and higher revs. Diesels are long-stroke; racing engines oversquare.");
    }

    ImGui::SeparatorText("Build");
    if (fourStroke) {
        int cam = (int)r.cam;
        if (ImGui::Combo("Valvetrain", &cam, "DOHC (twin cam)\0SOHC (single cam)\0OHV (pushrods, cam in the block)\0")) r.cam = (CamLayout)cam;
        int v = r.valves >= 4 ? 1 : 0;
        if (ImGui::RadioButton("2 valves", &v, 0)) r.valves = 2;
        ImGui::SameLine();
        if (ImGui::RadioButton("4 valves per cylinder", &v, 1)) r.valves = 4;
        int drv = (int)r.drive;
        if (ImGui::Combo("Cam drive", &drv, "Toothed belt\0Chain\0Gears\0")) r.drive = (CamDrive)drv;
    }
    if (r.kind == BuildKind::Petrol) ImGui::Combo("Fuel", &r.fuel, "Port injection (EFI)\0Individual throttle bodies (ITB)\0Carburettor\0");
    else if (r.kind == BuildKind::TwoStroke) {
        static const char* kFuel2[] = {"Carburettor (premix)", "Injection (oil pump)"};
        recipeLimit(r);
        if (ImGui::BeginCombo("Fuel", kFuel2[clampv(r.fuel, 0, 1)])) {
            for (int f = 0; f < 2; ++f) {
                const bool ok = recipeAllowed(r, 1, f);
                ImGui::BeginDisabled(!ok);
                if (ImGui::Selectable(kFuel2[f], f == r.fuel)) r.fuel = f;
                ImGui::EndDisabled();
                if (!ok && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Single cylinder only for now (like the KTM TPI): an injected multi-cylinder\ntwo-stroke does not idle reliably in the simulation yet.");
            }
            ImGui::EndCombo();
        }
    }
    if (r.kind == BuildKind::Petrol || r.kind == BuildKind::Rotary)
        ImGui::Combo("Induction", &r.aspiration, "Naturally aspirated\0Turbocharger\0Roots supercharger\0Twin-screw supercharger\0Centrifugal supercharger\0");
    else if (r.kind == BuildKind::Diesel) {
        r.aspiration = std::min(r.aspiration, 1);
        ImGui::Combo("Induction", &r.aspiration, "Naturally aspirated\0Turbocharger (VGT, EGR)\0");
    }
    if (r.kind == BuildKind::Rotary) {
        r.character = clampv(r.character, 0, 3);
        if (ImGui::BeginCombo("Porting", rotaryPortingName(r.character))) {
            for (int s = 0; s < 4; ++s) {
                const bool ok = recipeAllowed(r, 0, s);
                ImGui::BeginDisabled(!ok);
                if (ImGui::Selectable(rotaryPortingName(s), s == r.character)) r.character = s;
                ImGui::EndDisabled();
                if (!ok && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Two rotors only for now: the bridge port's idle (the small shared plenum that keeps the\nrotors in step) is tuned on the 13B.");
            }
            ImGui::EndCombo();
        }
    } else if (r.kind != BuildKind::Diesel) {
        r.character = clampv(r.character, 0, 2);
        ImGui::Combo("Character", &r.character, r.kind == BuildKind::TwoStroke ? "Road (broad pipe, mild ports)\0Sport\0Race (peaky pipe)\0"
                                                                                : "Street (mild cams, quiet)\0Sport\0Race (wild cams, open exhaust)\0");
    }
    if (r.kind != BuildKind::Rotary) {
        int c = (int)r.cooling;
        if (ImGui::RadioButton("Liquid cooled", &c, 0)) r.cooling = Cooling::Liquid;
        ImGui::SameLine();
        if (ImGui::RadioButton("Air cooled (fins)", &c, 1)) r.cooling = Cooling::Air;
    }
    recipeLimit(r);   // (a rotor count or cylinder count change can rule out a choice made before)
    char autoName[48];
    recipeAutoName(r, autoName, sizeof(autoName));
    ImGui::InputTextWithHint("Name", autoName, r.name, sizeof(r.name));
    ImGui::PopItemWidth();

    // what comes out (rebuilt when a choice changes)
    if (!recipePreviewValid_ || std::memcmp(&recipePreviewOf_, &r, sizeof(r)) != 0) {
        recipePreview_ = buildEngine(r);
        recipePreviewOf_ = r;
        recipePreviewValid_ = true;
    }
    const EngineParams& q = recipePreview_;
    ImGui::SeparatorText("What you get");
    const double L = ratedDisplacementL(q);
    if (isRotary(q))
        ImGui::Text("Rotor R %.1f  e %.1f  width %.1f mm   %.2f L (%d x %.0f cc)   CR %.1f : 1", q.rotorRadius_mm, q.eccentricity_mm, q.rotorWidth_mm, L,
                    q.cylinders, L * 1000.0 / q.cylinders, q.compression);
    else
        ImGui::Text("Bore x stroke %.1f x %.1f mm   rod %.0f mm   %.2f L   CR %.1f : 1", q.bore_mm, q.stroke_mm, q.rod_mm, L, q.compression);
    if (isTwoStroke(q))
        ImGui::Text("Exhaust port opens %.0f deg ATDC, transfers %.0f   pipe tuned to %.0f rpm   reeds %.0f cm2", q.tsExhaustOpen, q.tsTransferOpen,
                    q.tsPipeRpm, q.tsReedArea_cm2);
    else if (!isRotary(q))
        ImGui::Text("Valves %d x %.1f in / %d x %.1f ex mm   cams %.0f / %.0f deg, lift %.1f / %.1f mm", q.intakeValves, q.intakeValveDia_mm, q.exhaustValves,
                    q.exhaustValveDia_mm, q.intakeDuration, q.exhaustDuration, q.intakeLift_mm, q.exhaustLift_mm);
    else ImGui::Text("Ports: %s", rotaryPortingName(r.character));
    if (isDiesel(q)) ImGui::Text("Diesel %s, up to %.0f mg per stroke   %s", q.dieselInjSystem ? "common rail" : "injection pump", q.dieselMaxFuel,
                                 q.induction == Induction::Turbo ? (q.vgt ? "VGT turbo, EGR" : "turbo") : "naturally aspirated");
    else if (q.fuelSystem == FuelSystem::Carburetor) ImGui::Text("Carburettor %d x %.0f mm venturi", (int)q.carbBarrels, q.venturi_mm);
    else ImGui::Text("%s %.0f mm   injectors %.0f cc/min", q.itb ? "Throttle bodies" : "Throttle", q.throttleBore_mm, q.injector_ccmin);
    if (isBoosted(q) && !isDiesel(q))
        ImGui::Text("%s   intercooled, %.1f L charge piping", q.induction == Induction::Turbo ? "Turbo" : q.scType == Supercharger::Centrifugal ? "Centrifugal supercharger" :
                    q.scType == Supercharger::TwinScrew ? "Twin-screw supercharger" : "Roots supercharger", q.chargeVolume_L);
    ImGui::Text("Rev limit %.0f rpm   idle %.0f rpm   flywheel %.3f kg m2", q.revLimit, q.idleTargetRpm, q.flywheel);
    ImGui::Text("In a %.0f kg %s, %d gears", q.mass_kg, q.cylinders <= 2 && L < 2.0 && !isRotary(q) ? "motorcycle" : "car", (int)q.gears);
    if (baseMapApplies(q)) ImGui::TextDisabled("The ECU's base map is measured on the virtual dyno after you create it (as for a preset).");

    ImGui::Spacing();
    if (ImGui::Button("Create engine", ImVec2(em * 9.0f, 0))) {
        switchEngine(recipePreview_, -1, false);
        engMsg_ = std::string("Built '") + recipePreview_.name + "' - save it under Save / share this engine to keep it in My engines";
        engMsgErr_ = false;
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(em * 6.0f, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

// Engine Setup > Engine: save as your own engine, export / import a file to share
void App::engineFileUi() {
    if (ImGui::CollapsingHeader("Save / share this engine")) {
        ImGui::InputText("Author", engAuthor_, sizeof(engAuthor_));
        ImGui::InputTextMultiline("Notes", engNotes_, sizeof(engNotes_), ImVec2(-170, ImGui::GetTextLineHeight() * 3.2f));
        EngineMeta m;
        m.author = engAuthor_; m.notes = engNotes_; m.app = "EngineSim 0.8";
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
