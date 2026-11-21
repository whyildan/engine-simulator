// Application shell: header bar (engine controls + key readouts, always visible), a left
// navigation with the 12 ECU-software sections, and the work area of the selected section.
// Sections that have no implementation yet say so plainly (no placeholder controls).
#include "ui/App.h"
#include "ui/Widgets.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace es {
using namespace ui;

static ImU32 col(float r, float g, float b, float a = 1) { return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, a)); }

namespace {
struct SectionInfo { const char* name; const char* subtitle; };
const SectionInfo kSections[App::kSectionCount] = {
    {"Engine Setup",         "Engine geometry, valvetrain / ports, intake and exhaust hardware, vehicle, sound"},
    {"ECU Setup",            "Controller settings, live ECU state, diagnostics, simulation control"},
    {"Fuel Tuning",          "VE / lambda maps, enrichments, corrections, injector data, load model, base map"},
    {"Ignition Tuning",      "Ignition map, temperature / per-cylinder corrections, limiter"},
    {"Boost Control",        "Turbo / supercharger hardware, ECU boost control, anti-lag, nitrous"},
    {"Engine Control",       "Idle, limiters, launch, flat shift, VVT, torque management"},
    {"Sensor Configuration", "Virtual sensors: calibration, noise, delay, failures"},
    {"Engine Protection",    "Warning thresholds, protection actions and limp mode"},
    {"Live Dashboard",       "Gauges, readouts, flow map and engine health, cylinder pressure and sound scopes, engine animation"},
    {"Data Logging",         "Channel recording, graphs, CSV export, session comparison"},
    {"Virtual Dyno",         "Steady-state and sweep tests, run comparison"},
    {"Calibration Manager",  "Save, load, version and compare calibrations"},
};
} // namespace

// ------------------------------------------------------------------------------------ header
void App::headerBar() {
    const float h = ImGui::GetFrameHeight();
    // engine controls
    bool ign = p_.ignition != 0;
    ImGui::PushStyleColor(ImGuiCol_Button, ign ? ImVec4(0.13f, 0.42f, 0.18f, 1) : ImVec4(0.36f, 0.13f, 0.13f, 1));
    if (ImGui::Button(ign ? "IGN ON [I]" : "IGN OFF [I]", ImVec2(h * 4.6f, 0))) p_.ignition = !p_.ignition;
    ImGui::PopStyleColor();
    ImGui::SameLine();
    if (ImGui::Button(autoStart_ ? "Starting..." : "START ENGINE", ImVec2(h * 5.2f, 0))) { autoStart_ = true; autoStartTimer_ = 0; }
    ImGui::SameLine();
    ImGui::Button("Crank [S]", ImVec2(h * 3.6f, 0));
    if (ImGui::IsItemActive()) { p_.starter = 1; p_.ignition = 1; }
    else if (ImGui::IsItemDeactivated()) p_.starter = 0;
    ImGui::SameLine();
    {
        bool paused = sim_->paused();
        if (ImGui::Button(paused ? "Resume" : "Pause", ImVec2(h * 3.2f, 0))) sim_->setPaused(!paused);
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(h * 2.6f, 0))) { sim_->requestReset(); p_.ignition = 0; p_.starter = 0; }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Fresh engine at rest (cold or warm start per ECU Setup)");
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(h * 6.0f);
    if (ImGui::SliderFloat("##thr", &p_.throttle, 0, 1, "Throttle %.2f")) keyThrottle_ = false;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(h * 9.5f);
    presetCombo("##preset");
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Engine preset or one of My engines (swaps the engine live)");

    // key readouts, compact digital style
    ImGui::SameLine(0, h);
    auto readout = [&](const char* label, const char* value, ImVec4 c) {
        ImGui::BeginGroup();
        ImGui::TextDisabled("%s", label);
        ImGui::SameLine(0, 4);
        ImGui::TextColored(c, "%s", value);
        ImGui::EndGroup();
        ImGui::SameLine(0, h * 0.8f);
    };
    char b[48];
    const ImVec4 white(0.92f, 0.92f, 0.92f, 1), dim(0.55f, 0.55f, 0.6f, 1);
    std::snprintf(b, sizeof(b), "%5.0f", tel_.rpm);
    readout("RPM", b, tel_.rpm > redlineRpm(p_) ? ImVec4(1, 0.35f, 0.3f, 1) : ImVec4(1, 0.7f, 0.3f, 1));
    std::snprintf(b, sizeof(b), "%5.1f kPa", tel_.mapKPa);
    readout("MAP", b, white);
    std::snprintf(b, sizeof(b), "%5.2f", tel_.afr);
    readout("AFR", b, tel_.afr < 11.5f ? ImVec4(1, 0.5f, 0.3f, 1) : tel_.afr > 15.5f ? ImVec4(1, 0.9f, 0.3f, 1) : ImVec4(0.4f, 0.9f, 0.5f, 1));
    std::snprintf(b, sizeof(b), "%4.1f deg", tel_.advance);
    readout("ADV", b, white);
    if (isBoosted(p_)) { std::snprintf(b, sizeof(b), "%+.2f bar", tel_.boostBar); readout("BOOST", b, ImVec4(0.95f, 0.45f, 0.8f, 1)); }
    std::snprintf(b, sizeof(b), "%4.0f C", tel_.egt);
    readout("EGT", b, white);
    std::snprintf(b, sizeof(b), "%3.0f C", tel_.snsValue[kSnsECT]);
    readout("ECT", b, tel_.snsValue[kSnsECT] > 105 ? ImVec4(1, 0.35f, 0.3f, 1) : tel_.snsValue[kSnsECT] < 60 ? ImVec4(0.45f, 0.7f, 1, 1) : white);
    std::snprintf(b, sizeof(b), "%5.0f Nm", tel_.torque);
    readout("TQ", b, white);
    if (p_.vehicle) {
        if (tel_.gear == 0) std::snprintf(b, sizeof(b), "N  %3.0f km/h", tel_.speedKmh);
        else std::snprintf(b, sizeof(b), "%s%d %3.0f km/h", p_.transMode == TransMode::Automatic ? "D" : "", tel_.gear, tel_.speedKmh);
        readout("GEAR", b, white);
    }
    // status lamps
    Lamp("Run", tel_.running, col(0.2f, 0.9f, 0.3f)); ImGui::SameLine();
    Lamp("Lim", tel_.limiter, col(1.0f, 0.2f, 0.2f)); ImGui::SameLine();
    Lamp("Shift", tel_.rpm >= redlineRpm(p_), col(0.3f, 0.6f, 1.0f)); ImGui::SameLine();
    if (isDiesel(p_)) { Lamp("Glow", tel_.glowLight, col(1.0f, 0.75f, 0.1f)); ImGui::SameLine(); }   // pre-heating: wait before cranking
    else { Lamp("Knock", tel_.knock > 1.0f, col(1.0f, 0.5f, 0.1f)); ImGui::SameLine(); }
    Lamp("Float", std::max(tel_.floatIn, tel_.floatEx) > 0.1f, col(1.0f, 0.6f, 0.2f)); ImGui::SameLine();
    Lamp("Oil", tel_.running && tel_.snsValue[kSnsOilP] < 0.7f, col(1.0f, 0.25f, 0.2f)); ImGui::SameLine();
    { int nf = 0; for (int k = 0; k < kSensorCount; ++k) nf += tel_.snsFallback[k] ? 1 : 0;
      int np = 0; for (int k = 0; k < kProtCount; ++k) np += tel_.protState[k] == 2 ? 1 : 0;
      Lamp("CEL", nf > 0 || tel_.syncLost || np > 0, col(1.0f, 0.65f, 0.1f)); ImGui::SameLine(); }
    if (tel_.shutdown) { ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "SHUTDOWN"); ImGui::SameLine(); }
    else if (tel_.limp) { ImGui::TextColored(ImVec4(1, 0.55f, 0.2f, 1), "LIMP"); ImGui::SameLine(); }
    std::snprintf(b, sizeof(b), "sim %3.0f%%", tel_.simLoad * 100);
    ImGui::TextColored(tel_.simLoad > 0.9f ? ImVec4(1, 0.4f, 0.3f, 1) : dim, "%s", b);
    if (bmRunning_) { ImGui::SameLine(); ImGui::TextColored(ImVec4(0.5f, 0.8f, 1, 1), "base map %.0f%%", bmProg_.fraction.load() * 100); }
    else if (bmStale_) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "VE map out of date"); }
    if (tel_.paused) { ImGui::SameLine(); ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "PAUSED"); }
    if (tel_.failure) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0.3f, 0.25f, 1), "ENGINE FAILED - repair in Engine Setup > Components");
    }
}

// ------------------------------------------------------------------------------------ navigation
void App::navBar() {
    ImGui::TextDisabled("ENGINESIM");
    ImGui::Separator();
    for (int i = 0; i < kSectionCount; ++i) {
        char label[64];
        std::snprintf(label, sizeof(label), "%2d  %s", i + 1, kSections[i].name);
        const bool sel = section_ == i, ready = sectionImplemented(i);
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.55f, 0.29f, 0.13f, 0.85f));
        // sections that are not built yet are dimmed (they only describe what is planned)
        if (!ready && !sel) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.45f, 0.45f, 0.5f, 1));
        if (ImGui::Selectable(label, sel, 0, ImVec2(0, ImGui::GetFrameHeight()))) { section_ = i; jumpTab_.clear(); }
        if (!ready && !sel) ImGui::PopStyleColor();
        ImGui::PopStyleColor();
        if (!ready && ImGui::IsItemHovered()) ImGui::SetTooltip("Planned - see docs/ECU_PLAN.md");
    }
    ImGui::Separator();
    ImGui::TextDisabled("dimmed = planned");
    ImGui::Separator();
    ImGui::TextDisabled("misfires %d", tel_.misfires);
    ImGui::TextDisabled("pops %d", tel_.afterfires);
    ImGui::TextDisabled("audio buf %.0f ms", tel_.ringFill * 1000);
}

bool App::sectionImplemented(int s) const {
    switch (s) {
    case kEngineSetup: case kEcuSetup: case kFuelTuning: case kIgnitionTuning: case kBoostControl: case kSensorConfig:
    case kLiveDashboard: case kVirtualDyno: case kCalibrationManager: case kEngineControl: case kEngineProtection: case kDataLogging:
        return true;
    default: return false;
    }
}

// a section that is not built yet: say what it will contain and where today's equivalents are
void App::plannedSection(const char* phase, const char* const* items, int n, const char* existing) {
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.35f, 1), "Not implemented yet - planned for %s.", phase);
    ImGui::TextDisabled("Nothing here changes the simulation yet; see docs/ECU_PLAN.md for the full plan.");
    ImGui::Spacing();
    ImGui::SeparatorText("Planned");
    for (int i = 0; i < n; ++i) ImGui::BulletText("%s", items[i]);
    if (existing) {
        ImGui::Spacing();
        ImGui::SeparatorText("Available today");
        ImGui::TextWrapped("%s", existing);
    }
}

// two side-by-side children; returns after the left one is begun (caller ends it with nextColumn/endColumns)
static void beginSplit(const char* id, float frac) {
    ImGui::BeginChild(id, ImVec2(ImGui::GetContentRegionAvail().x * frac, 0), ImGuiChildFlags_Border);
}
static void splitNext(const char* id) {
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild(id, ImVec2(0, 0), ImGuiChildFlags_Border);
}
static void endSplit() { ImGui::EndChild(); }

// ------------------------------------------------------------------------------------ sections
void App::workArea() {
    ImGui::TextColored(ImVec4(1.0f, 0.68f, 0.32f, 1), "%d  %s", section_ + 1, kSections[section_].name);
    ImGui::SameLine();
    ImGui::TextDisabled("  %s", kSections[section_].subtitle);
    // motorsport states, right-aligned on the title line (the header row is full on boosted engines)
    {
        std::string tags;
        if (tel_.vtecEngaged > 0.5f) tags += "VTEC  ";
        if (tel_.launchState != kLaunchOff) tags += std::string(launchStateName(tel_.launchState)) + "  ";
        if (tel_.transbrake) tags += "TRANSBRAKE  ";
        if (tel_.tcReduction > 0.02f) tags += "TC  ";
        if (tel_.n2oDuty > 0) tags += "N2O  ";
        else if (p_.nitrous && p_.n2oArm) tags += "N2O armed  ";
        if (!tags.empty()) {
            const float w = ImGui::CalcTextSize(tags.c_str()).x;
            ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - w));
            ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1), "%s", tags.c_str());
        }
    }
    ImGui::Separator();

    switch (section_) {
    case kEngineSetup: {
        beginSplit("##es_cfg", 0.56f);
        if (ImGui::BeginTabBar("engtabs", ImGuiTabBarFlags_FittingPolicyScroll)) {
            if (ImGui::BeginTabItem("Engine", nullptr, tabJump("engine"))) { panelEngine(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem(isRotary(p_) || isTwoStroke(p_) ? "Ports###valv" : "Valvetrain###valv", nullptr, tabJump("valv"))) { panelValvetrain(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Exhaust", nullptr, tabJump("exhaust"))) { panelExhaust(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Vehicle")) { panelVehicle(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Sound")) { panelAudio(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Components", nullptr, tabJump("components"))) { panelComponents(); ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        splitNext("##es_view");
        panelEngineView();
        endSplit();
        break;
    }
    case kEcuSetup: sectionEcuSetup(); break;
    case kFuelTuning: sectionFuelTuning(); break;
    case kIgnitionTuning: sectionIgnitionTuning(); break;
    case kBoostControl: {
        beginSplit("##boost_cfg", 0.50f);
        if (ImGui::BeginTabBar("boosttabs")) {
            if (ImGui::BeginTabItem("Hardware", nullptr, tabJump("boost_hw"))) { panelBoost(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("ECU boost control")) { tabBoostControl(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Anti-lag")) { tabAntiLag(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Nitrous")) { tabNitrous(); ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        splitNext("##boost_graph");
        panelBoostGraph();
        endSplit();
        break;
    }
    case kEngineControl: sectionEngineControl(); break;
    case kSensorConfig: sectionSensors(); break;
    case kEngineProtection: sectionProtection(); break;
    case kLiveDashboard: {
        beginSplit("##dash", 0.55f);
        if (ImGui::BeginTabBar("dashkind")) {
            if (ImGui::BeginTabItem("Gauges", nullptr, tabJump("gauges"))) { panelDashboard(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Custom", nullptr, tabJump("custom"))) { panelCustomDash(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Flow map", nullptr, tabJump("flowmap"))) { panelFlowMap(); ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        splitNext("##dash_right");
        if (ImGui::BeginTabBar("dashtabs")) {
            if (ImGui::BeginTabItem("Scope")) { panelScope(); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Engine view")) { panelEngineView(); ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        endSplit();
        break;
    }
    case kDataLogging: sectionDataLogging(); break;
    case kVirtualDyno: {
        beginSplit("##dyno", 0.64f);
        panelDyno();
        splitNext("##dyno_side");
        panelDashboard();
        endSplit();
        break;
    }
    case kCalibrationManager: sectionCalibrationManager(); break;
    default: break;
    }
}

void App::shell() {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::SetNextWindowViewport(vp->ID);
    const ImGuiWindowFlags fl = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                ImGuiWindowFlags_NoDocking;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
    ImGui::Begin("##shell", nullptr, fl);
    ImGui::PopStyleVar(2);

    ImGui::BeginChild("##header", ImVec2(0, ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().WindowPadding.y * 2),
                      ImGuiChildFlags_Border, ImGuiWindowFlags_NoScrollbar);
    headerBar();
    ImGui::EndChild();

    const float navW = ImGui::GetFontSize() * 16.0f;
    ImGui::BeginChild("##nav", ImVec2(navW, 0), ImGuiChildFlags_Border);
    navBar();
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("##work", ImVec2(0, 0), ImGuiChildFlags_None);
    workArea();
    ImGui::EndChild();
    ImGui::End();
}

// ------------------------------------------------------------------------------------ ECU setup
static void kv(const char* k, const char* fmt, ...) {
    ImGui::TextDisabled("%s", k);
    ImGui::SameLine(ImGui::GetFontSize() * 14.0f);
    va_list ap; va_start(ap, fmt); ImGui::TextV(fmt, ap); va_end(ap);
}

void App::sectionEcuSetup() {
    ImGui::BeginChild("##ecu_cfg", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f, 0), ImGuiChildFlags_Border);
    ImGui::PushItemWidth(-220);
    ImGui::SeparatorText("Controller");
    ImGui::SliderFloat("ECU loop rate", &p_.ecuRateHz, 100, 5000, "%.0f Hz");
    Help("How often the ECU samples its sensors and runs idle, spark, limiter and closed-loop control.\n"
         "Injection and spark events are still scheduled by crank angle.");
    {
        bool tuned = p_.veLearning != 0;
        if (ImGui::Checkbox("Calibrated VE map (tuned ECU)", &tuned)) p_.veLearning = tuned ? 1 : 0;
        Help("On: the VE map matches this engine and the wideband closed loop is active.\n"
             "Off: a generic VE guess, open loop - the mixture wanders with load and rpm.");
    }
    if (p_.fuelSystem == FuelSystem::EFI) {
        int inj = (int)p_.injection;
        if (ImGui::Combo("Injector position", &inj, "Port injection\0Throttle-body injection\0")) p_.injection = (Injection)inj;
        ImGui::SliderFloat("End of injection", &p_.injectionAngle, 0, (float)cycleDeg(p_), "%.0f deg");
    } else {
        ImGui::TextDisabled("Carburettor: the ECU is the ignition module only (curve + limiter).");
    }
    ImGui::TextDisabled("Load calculation: speed density / alpha-N / MAF / hybrid (Fuel Tuning).");
    ecuConfigUi();

    ImGui::SeparatorText("Simulation");
    {
        bool cold = p_.coldStart != 0;
        if (ImGui::Checkbox("Cold start (engine at ambient temperature)", &cold)) p_.coldStart = cold ? 1 : 0;
        Help("Applies on the next reset / engine rebuild. Cold: thick oil, wet port walls, poor fuel\n"
             "vaporisation -> the ECU adds cranking / warm-up enrichment and fast idle; carburettors use\n"
             "the automatic choke and fast-idle cam.");
        ImGui::SliderFloat("Ambient temperature", &p_.ambientTemp_C, -30, 45, "%.0f C");
        altitudeUi("Altitude##sim");
    }
    bool paused = sim_->paused();
    if (ImGui::Button(paused ? "Resume" : "Pause")) sim_->setPaused(!paused);
    ImGui::SameLine();
    if (ImGui::Button("Reset engine")) { sim_->requestReset(); p_.ignition = 0; p_.starter = 0; }
    Help("Reset: a fresh engine at rest with the start temperature above (car stopped, dyno idle).");
    ImGui::PopItemWidth();
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("##ecu_live", ImVec2(0, 0), ImGuiChildFlags_Border);
    ImGui::SeparatorText("ECU live");
    kv("Engine speed (CKP)", "%.0f rpm %s", tel_.snsValue[kSnsCKP], tel_.syncLost ? "  -- NO SYNC: spark and fuel off" : "");
    kv("Load (MAP used)", "%.1f kPa%s", tel_.ecuMap, tel_.snsFallback[kSnsMAP] ? "  (alpha-N fallback: MAP sensor fault)" : "");
    kv("Target AFR", tel_.targetAfr > 0 ? "%.2f" : "fuel cut", tel_.targetAfr);
    kv("Measured lambda (wideband)", "%.3f  (AFR %.2f)", tel_.snsValue[kSnsWideband], tel_.snsValue[kSnsWideband] * 14.7f);
    kv("Closed loop", "%s   trim %+.1f %%", tel_.closedLoop ? "active" : "open loop", (tel_.ecuTrim - 1) * 100);
    kv("Warm-up enrichment", "%+.0f %%  (coolant %.0f C)", tel_.ecuWarmup * 100, tel_.snsValue[kSnsECT]);
    kv("Injector duty", "%.0f %%", tel_.injDuty * 100);
    kv("Dead-time compensation", "%.2f ms at %.1f V", tel_.ecuDeadMs, tel_.snsValue[kSnsBattery]);
    kv("Flow compensation", "x%.3f at %.2f bar rail", tel_.ecuFlowComp, tel_.snsValue[kSnsFuelP]);
    kv("Spark advance", "%.1f deg BTDC", tel_.advance);
    kv("Idle air valve", "%.0f %%", tel_.iac);
    kv("Rev limiter", "%s", tel_.limiter ? "CUTTING" : "-");
    {
        static const char* im[] = {"sequential", "semi-sequential", "batch fire"};
        kv("Injection mode", "%s%s", im[clampv(tel_.injMode, 0, 2)],
           tel_.injMode != p_.cal.injMode && tel_.injMode == 1 ? "  (no cycle phase: cam not synced)" : "");
        kv("Trigger", "%s   crank %s, cam %s (synced after %.0f deg)", triggerInfo(p_.cal.triggerPattern).name,
           tel_.crankSync ? "sync" : "NO SYNC", tel_.camSync ? "sync" : "-", tel_.syncDeg);
        kv("Coil dwell", "%.2f ms   spark %.0f / %.0f mJ", tel_.dwellMs, tel_.sparkAvail, p_.comp.coilEnergy_mJ);
    }
    int nf = 0;
    for (int k = 0; k < kSensorCount; ++k) nf += tel_.snsFallback[k] ? 1 : 0;
    ImGui::SeparatorText("Diagnostics");
    if (!nf) ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.5f, 1), "No sensor faults detected.");
    for (int k = 0; k < kSensorCount; ++k) {
        if (!tel_.snsFallback[k]) continue;
        static const char* st[] = {"ok", "signal too low", "signal too high", "no signal"};
        ImGui::TextColored(ImVec4(1, 0.45f, 0.3f, 1), "%s: %s -> using fallback", sensorDef(k).name, st[clampv(tel_.snsStatus[k], 0, 3)]);
    }
    ImGui::TextDisabled("A stuck sensor is not detected: the ECU keeps trusting it.");
    ImGui::EndChild();
}

// ------------------------------------------------------------------------------------ sensors
void App::sectionSensors() {
    if (ImGui::Button("Reset all sensors to default")) defaultSensorConfig(p_.sensors);
    ImGui::SameLine();
    ImGui::TextDisabled("Truth = simulation (diagnostics only). The ECU only sees the ECU value column.");
    const ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("sensors", 8, tf)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Sensor", 0, 2.2f);
        ImGui::TableSetupColumn("Truth", 0, 1.0f);
        ImGui::TableSetupColumn("ECU value", 0, 1.0f);
        ImGui::TableSetupColumn("Signal", 0, 0.8f);
        ImGui::TableSetupColumn("Status", 0, 1.1f);
        ImGui::TableSetupColumn("Noise", 0, 1.1f);
        ImGui::TableSetupColumn("Lag", 0, 1.1f);
        ImGui::TableSetupColumn("Fault", 0, 1.3f);
        ImGui::TableHeadersRow();
        for (int k = 0; k < kSensorCount; ++k) {
            const SensorDef& d = sensorDef(k);
            SensorConfig& c = p_.sensors[k];
            ImGui::PushID(k);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(d.name);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\nRange %.4g .. %.4g %s\nUsed for: %s", d.name, d.lo, d.hi, d.unit, d.role);
            const char* fmt = d.hi - d.lo < 5 ? "%.3f %s" : d.hi - d.lo < 200 ? "%.1f %s" : "%.0f %s";
            ImGui::TableNextColumn(); ImGui::TextDisabled(fmt, tel_.snsTruth[k], d.unit);
            ImGui::TableNextColumn(); ImGui::Text(fmt, tel_.snsValue[k], d.unit);
            ImGui::TableNextColumn();
            if (d.analog) ImGui::Text("%.2f V", tel_.snsVolts[k]); else ImGui::TextDisabled(tel_.snsStatus[k] == 3 ? "none" : "pulses");
            ImGui::TableNextColumn();
            static const char* st[] = {"OK", "LOW", "HIGH", "NO SIGNAL"};
            int s = clampv(tel_.snsStatus[k], 0, 3);
            ImGui::TextColored(s ? ImVec4(1, 0.4f, 0.3f, 1) : ImVec4(0.45f, 0.9f, 0.5f, 1), "%s%s", st[s], tel_.snsFallback[k] ? " (fallback)" : "");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1);
            if (d.noise > 0) ImGui::SliderFloat("##n", &c.noiseScale, 0, 10, "%.1fx");
            else ImGui::TextDisabled("-");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1);
            if (d.analog) ImGui::SliderFloat("##l", &c.lag_ms, 0, 5000, "%.0f ms", ImGuiSliderFlags_Logarithmic);
            else ImGui::TextDisabled("-");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1);
            int f = (int)c.fault;
            if (ImGui::Combo("##f", &f, sensorFaultNames())) c.fault = (SensorFault)f;
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
}

} // namespace es
