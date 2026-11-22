// All ImGui panels. Every widget edits App::p_ directly; App::frame() pushes the changed
// parameters to the simulation thread, so every change is heard immediately.
#include "sim/Fuel.h"
#include "sim/Scavenge.h"
#include "ui/App.h"
#include "sim/Thermal.h"
#include "ui/Widgets.h"
#include "imgui.h"
#include "implot.h"
#include "audio/ImpulseResponses.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>

namespace es {

using namespace ui;

static ImU32 rgba(float r, float g, float b, float a = 1) { return ImGui::ColorConvertFloat4ToU32(ImVec4(r, g, b, a)); }

static bool SliderF(const char* label, float* v, float lo, float hi, const char* fmt, const char* help = nullptr) {
    bool ch = ImGui::SliderFloat(label, v, lo, hi, fmt);
    if (help) Help(help);
    return ch;
}
static bool SliderI(const char* label, int32_t* v, int lo, int hi, const char* help = nullptr) {
    int t = *v;
    bool ch = ImGui::SliderInt(label, &t, lo, hi);
    if (ch) *v = t;
    if (help) Help(help);
    return ch;
}
static bool Check(const char* label, int32_t* v) {
    bool b = *v != 0;
    bool ch = ImGui::Checkbox(label, &b);
    if (ch) *v = b ? 1 : 0;
    return ch;
}
template <class E> static bool Combo(const char* label, E* v, const char* items) {
    int t = (int)*v;
    bool ch = ImGui::Combo(label, &t, items);
    if (ch) *v = (E)t;
    return ch;
}

static double displacementL(const EngineParams& p) { return ratedDisplacementL(p); }
// what the engine breathes like (rotary 1.3L ~ 2.6L four-stroke) - used to scale gauges
static double breathL(const EngineParams& p) { return equivalentDisplacementL(p); }
// road speed at the rev limiter in a gear (km/h)
static float gearSpeed(const EngineParams& p, int g) {
    double ratio = std::max(0.2f, p.gearRatio[g]) * std::max(0.5f, p.finalDrive);
    return (float)(p.revLimit / 60.0 * 2 * kPi / ratio * p.tireRadius_mm * 1e-3 * 3.6);
}
static float vehicleTopSpeed(const EngineParams& p) { return gearSpeed(p, clampv((int)p.gears, 1, kMaxGears) - 1); }

// ======================================================================= dashboard
void App::panelDashboard() {
    const float avail = ImGui::GetContentRegionAvail().x;
    const float gsize = std::clamp(avail * 0.36f, 160.0f, 320.0f);
    float maxRpm = std::ceil((p_.revLimit + 1000.0f) / 1000.0f) * 1000.0f;

    ImGui::BeginGroup();
    Gauge("tach", tel_.rpm, 0, maxRpm, redlineRpm(p_), "RPM x1000", "%.0f rpm", gsize, (int)(maxRpm / 1000), 0.001f);
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    float tqMax = std::max(100.0f, (float)std::ceil(breathL(p_) * 130.0 / 50.0) * 50.0f);
    if (p_.vehicle) {
        float vmax = 40.0f * std::ceil(std::max(160.0f, vehicleTopSpeed(p_) * 1.1f) / 40.0f);
        Gauge("speed", tel_.speedKmh, 0, vmax, vmax * 2, "km/h", "%.0f km/h", gsize * 0.62f, (int)(vmax / 40), 1.0f);
    } else
        Gauge("torque", tel_.torque, 0, tqMax, tqMax * 2, "Torque", "%.0f Nm", gsize * 0.62f, 5, 1.0f);
    float hpMax = std::max(50.0f, (float)std::ceil(breathL(p_) * 110.0 / 50.0) * 50.0f);
    Gauge("power", tel_.powerKw * 1.341f, 0, hpMax, hpMax * 2, "Power", "%.0f hp", gsize * 0.62f, 5, 1.0f);
    ImGui::EndGroup();
    ImGui::SameLine();

    ImGui::BeginGroup();
    // (ignition, starter and throttle live in the header bar, visible in every section)
    Lamp("Running", tel_.running, rgba(0.2f, 0.9f, 0.3f));
    ImGui::SameLine();
    Lamp("Starter", p_.starter != 0, rgba(0.9f, 0.8f, 0.2f));
    Lamp("Limiter", tel_.limiter, rgba(1.0f, 0.2f, 0.2f));
    ImGui::SameLine();
    Lamp("Knock", tel_.knock > 1.0f, rgba(1.0f, 0.5f, 0.1f));
    ImGui::Spacing();
    ImGui::TextDisabled("W/Up: gas  Space: WOT  1-9,0: fixed");
    ImGui::TextDisabled("S: crank  I: ignition  D: dyno  L: listener");
    if (p_.vehicle) {
        ImGui::TextDisabled("E/Q: gear up/down  C: clutch  B/Down: brake");
        // big gear indicator
        char gbuf[8];
        if (tel_.gear == 0) std::snprintf(gbuf, sizeof(gbuf), "N");
        else if (p_.transMode == TransMode::Automatic) std::snprintf(gbuf, sizeof(gbuf), "D%d", tel_.gear);
        else std::snprintf(gbuf, sizeof(gbuf), "%d", tel_.gear);
        ImGui::SetWindowFontScale(2.4f);
        ImGui::TextColored(tel_.shifting ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(0.4f, 1, 0.5f, 1), "%s", gbuf);
        ImGui::SetWindowFontScale(1.0f);
        ImGui::SameLine();
        ImGui::BeginGroup();
        Lamp("Clutch slip", !tel_.clutchLocked && tel_.clutchEng > 0.02f && tel_.gear > 0, rgba(1, 0.8f, 0.2f));
        Lamp("Wheelspin", std::fabs(tel_.wheelSlip) > 0.2f, rgba(1, 0.3f, 0.2f));
        ImGui::EndGroup();
    } else ImGui::TextDisabled("V: drive the car (vehicle load + gearbox)");
    ImGui::EndGroup();

    ImGui::Separator();
    // readouts in two columns
    if (ImGui::BeginTable("readouts", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        Meter("MAP", tel_.mapKPa, 0, 110, "%.1f kPa", rgba(0.3f, 0.6f, 1.0f));
        Meter("AFR", tel_.afr, 8, 20, "%.2f", tel_.afr < 12.0f ? rgba(0.9f, 0.4f, 0.2f) : tel_.afr > 15.5f ? rgba(0.9f, 0.9f, 0.2f) : rgba(0.3f, 0.85f, 0.4f));
        Meter("Ign. advance", tel_.advance, 0, 50, "%.1f deg", rgba(0.85f, 0.75f, 0.2f));
        Meter("EGT", tel_.egt, 0, 1000, "%.0f C", rgba(0.95f, 0.45f, 0.2f));
        Meter("Vol. efficiency", tel_.ve * 100, 0, 120, "%.0f %%", rgba(0.5f, 0.8f, 0.9f));
        ImGui::TableNextColumn();
        Meter("Air flow", tel_.airFlow, 0, (float)breathL(p_) * 70.0f, "%.1f g/s", rgba(0.4f, 0.7f, 1.0f));
        Meter("Fuel flow", tel_.fuelFlow, 0, (float)breathL(p_) * 6.0f, "%.2f g/s", rgba(0.9f, 0.6f, 0.2f));
        if (p_.fuelSystem == FuelSystem::EFI) {
            Meter("Injector duty", tel_.injDuty * 100, 0, 100, "%.0f %%", tel_.injDuty > 0.9f ? rgba(1, 0.2f, 0.2f) : rgba(0.6f, 0.6f, 0.9f));
            Meter("Idle valve", tel_.iac, 0, 100, "%.0f %%", rgba(0.6f, 0.9f, 0.6f));
        } else {
            Meter("Target AFR", 12.8f / std::max(0.3f, p_.mainJet), 8, 20, "%.1f (main jet)", rgba(0.6f, 0.6f, 0.9f));
            Meter("Choke", p_.choke * 100, 0, 100, "%.0f %%", rgba(0.6f, 0.9f, 0.6f));
        }
        Meter("Knock index", tel_.knock, 0, 3, "%.2f", tel_.knock > 1 ? rgba(1, 0.3f, 0.1f) : rgba(0.5f, 0.5f, 0.5f));
        ImGui::EndTable();
    }
    // engine health from the sensors (what an instrument cluster / ECU dash shows)
    if (ImGui::BeginTable("health", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        Meter("Coolant", tel_.snsValue[kSnsECT], -20, 130, "%.0f C", tel_.snsValue[kSnsECT] > 105 ? rgba(1, 0.3f, 0.2f) : rgba(0.4f, 0.7f, 1.0f));
        Meter("Oil temp", tel_.snsValue[kSnsOilT], -20, 150, "%.0f C", tel_.snsValue[kSnsOilT] > 130 ? rgba(1, 0.3f, 0.2f) : rgba(0.9f, 0.7f, 0.3f));
        Meter("Intake air", tel_.snsValue[kSnsIAT], -20, 80, "%.0f C", rgba(0.5f, 0.8f, 0.9f));
        ImGui::TableNextColumn();
        Meter("Oil pressure", tel_.snsValue[kSnsOilP], 0, 8, "%.1f bar", tel_.running && tel_.snsValue[kSnsOilP] < 0.7f ? rgba(1, 0.3f, 0.2f) : rgba(0.9f, 0.75f, 0.3f));
        Meter("Fuel pressure", tel_.snsValue[kSnsFuelP], 0, 8, "%.2f bar", rgba(0.9f, 0.6f, 0.2f));
        Meter("Battery", tel_.snsValue[kSnsBattery], 8, 16, "%.1f V", tel_.snsValue[kSnsBattery] < 12 ? rgba(1, 0.4f, 0.3f) : rgba(0.5f, 0.85f, 0.5f));
        ImGui::EndTable();
    }
    if (p_.vehicle) {
        if (ImGui::BeginTable("vehro", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            Meter("Clutch", tel_.clutchEng * 100, 0, 100, tel_.clutchLocked ? "%.0f %% locked" : "%.0f %% slipping", rgba(0.9f, 0.8f, 0.3f));
            Meter("Brake", p_.brake * 100, 0, 100, "%.0f %%", rgba(1, 0.3f, 0.3f));
            ImGui::TableNextColumn();
            Meter("Wheel slip", std::fabs(tel_.wheelSlip) * 100, 0, 100, "%.0f %%", rgba(1, 0.5f, 0.3f));
            Meter("Distance", tel_.distance, 0, 402, "%.0f m", rgba(0.5f, 0.7f, 1.0f));
            ImGui::EndTable();
        }
        ImGui::Text("0-100 km/h: %s   1/4 mile: %s", tel_.t100 > 0 ? (std::to_string(tel_.t100).substr(0, 4) + " s").c_str() : "--",
                    tel_.tQuarter > 0 ? (std::to_string(tel_.tQuarter).substr(0, 5) + " s @ " + std::to_string((int)tel_.vQuarter) + " km/h").c_str() : "--");
    }
    if (isBoosted(p_)) {
        float bmax = std::max(1.5f, (p_.induction == Induction::Turbo ? p_.wastegateBar : 1.0f) + 0.6f);
        if (ImGui::BeginTable("boostro", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            Meter("Boost", tel_.boostBar, -1.0f, bmax, "%+.2f bar", tel_.boostBar > 0 ? rgba(0.95f, 0.35f, 0.75f) : rgba(0.5f, 0.5f, 0.6f));
            Meter(p_.induction == Induction::Turbo ? "Turbo speed" : "Blower speed", tel_.shaftRpm / 1000.0f, 0,
                  p_.induction == Induction::Turbo ? 240.0f : (p_.scType == Supercharger::Centrifugal ? 70.0f : 18.0f),
                  "%.0fk rpm", rgba(0.6f, 0.75f, 1.0f));
            ImGui::TableNextColumn();
            Meter("Charge temp", tel_.chargeT, 0, 150, "%.0f C", rgba(0.95f, 0.6f, 0.3f));
            if (p_.induction == Induction::Turbo) Meter("Wastegate", tel_.wastegate * 100, 0, 100, "%.0f %%", rgba(0.7f, 0.7f, 0.7f));
            else Meter("Blower power", tel_.fiPowerKw, 0, (float)breathL(p_) * 40.0f, "%.1f kW", rgba(0.7f, 0.7f, 0.7f));
            ImGui::EndTable();
        }
        Lamp("BOV", tel_.bovOpen, rgba(0.4f, 0.8f, 1.0f)); ImGui::SameLine();
        Lamp("Surge", tel_.surging, rgba(1.0f, 0.6f, 0.2f)); ImGui::SameLine();
        Lamp("Anti-lag", tel_.antiLag, rgba(1.0f, 0.3f, 0.2f));
    }
    ImGui::TextDisabled("misfires %d   afterfire pops %d   sim load %.0f%%   audio buffer %.0f ms",
                        tel_.misfires, tel_.afterfires, tel_.simLoad * 100, tel_.ringFill * 1000);
    if (tel_.simLoad > 0.9f) ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "Simulation is near real-time limit: lower oversampling in Audio.");
}

// ======================================================================= engine
void App::panelEngine() {
    ImGui::PushItemWidth(-170);
    presetCombo("Preset");
    Help("Loading a preset while the engine runs swaps the engine live (the sound crossfades).\n"
         "My engines: engines you saved or imported (Save / share this engine below).");
    if (ImGui::Button("New engine from scratch...")) newEngineReq_ = true;
    Help("Build an engine of your own: petrol, diesel, two-stroke or rotary, its layout, cylinders, size and a few\n"
         "build choices - bore, stroke, valves, cams, intake, exhaust, the car and the base calibration are sized\n"
         "from them. Save it under Save / share this engine to keep it.");
    ImGui::InputText("Name", p_.name, sizeof(p_.name));
    engineFileUi();
    {
        int et = (int)p_.engineType;
        if (ImGui::Combo("Engine type", &et, "Piston (reciprocating)\0Rotary (Wankel)\0Two-stroke (crankcase scavenged)\0")) {
            p_.engineType = (EngineType)et;
            if (isRotary(p_)) { p_.cylinders = clampv((int)p_.cylinders, 1, 4); if (p_.cylinders > 2) p_.cylinders = 2; }
            else if (isTwoStroke(p_)) { p_.cylinders = clampv((int)p_.cylinders, 1, 4); if (isDiesel(p_)) p_.fuelSystem = FuelSystem::Carburetor; }
            else if (p_.cylinders < 1) p_.cylinders = 4;
            applyLayoutDefaults(p_);
            if (isRotary(p_)) { p_.idleTargetRpm = std::max(p_.idleTargetRpm, 950.0f); p_.flywheel = std::max(p_.flywheel, 0.14f); }
            if (isTwoStroke(p_)) { p_.idleTargetRpm = std::max(p_.idleTargetRpm, 1400.0f); p_.evenFire = 1; }
            if (equalPrimaries_) for (int i = 1; i < kMaxCyl; ++i) p_.primaryLen_mm[i] = p_.primaryLen_mm[0];
        }
        Help("Rotary (Wankel): triangular rotors orbit an eccentric shaft. Every rotor has 3 chambers;\n"
             "each chamber does intake-compression-power-exhaust during one rotor turn (= 3 shaft turns),\n"
             "so every rotor fires once per shaft revolution. No valves: the rotor flanks open the ports.\n"
             "Two-stroke: one power stroke every revolution. The piston uncovers the exhaust and transfer ports in\n"
             "the cylinder wall; the crankcase under it pumps the fresh charge (in through a reed valve) up through the\n"
             "transfer ports, and the expansion chamber's pressure waves help fill the cylinder. (Ports: the Ports tab.)");
    }
    if (isRotary(p_)) { panelRotary(); ImGui::PopItemWidth(); return; }
    ImGui::SeparatorText("Layout");
    bool layoutChanged = Combo("Layout", &p_.layout, "Inline\0V (vee)\0Boxer (flat)\0");
    layoutChanged |= SliderI("Cylinders", &p_.cylinders, 1, kMaxCyl, "Changing the cylinder count rebuilds the engine and resets the firing order.");
    if (layoutChanged) {
        if (p_.layout != Layout::Inline && p_.cylinders < 2) p_.cylinders = 2;
        applyLayoutDefaults(p_);
        if (equalPrimaries_) for (int i = 1; i < kMaxCyl; ++i) p_.primaryLen_mm[i] = p_.primaryLen_mm[0];
    }
    if (p_.layout == Layout::V) SliderF("Bank angle", &p_.bankAngle, 15, 135, "%.0f deg", "Visual + for your firing-phase design. Use the firing phases below for odd-fire engines.");
    Combo("Cooling", &p_.cooling, "Liquid (water jacket)\0Air (finned cylinders)\0");
    Help("How the cylinders are cooled. Drawn by the engine view (fins on an air-cooled engine); the\n"
         "simulation's warm-up and temperatures do not use it yet.");

    ImGui::SeparatorText("Geometry");
    SliderF("Bore", &p_.bore_mm, 40, 140, "%.1f mm");
    SliderF("Stroke", &p_.stroke_mm, 40, 140, "%.1f mm");
    SliderF("Rod length", &p_.rod_mm, std::max(60.0f, p_.stroke_mm * 1.2f), 260, "%.0f mm", "Longer rods: piston dwells longer at TDC.");
    SliderF("Compression", &p_.compression, 6, isDiesel(p_) ? 25.0f : 16.0f, "%.1f : 1", isDiesel(p_) ? "A diesel lights by compression: ~16-22:1. Lower = later ignition, harder cold starts." : "Higher CR = more torque but more knock.");
    SliderF("Flywheel inertia", &p_.flywheel, 0.02f, 0.6f, "%.3f kg m2", "Light = revs fast and sounds raw; heavy = smooth, slow revving.");
    SliderF("Imperfection", &p_.imperfection, 0, 2.5f, "%.2f", "Real engines are never perfectly even: each cylinder's port flow, injector, spark timing,\n"
            "burn and header length differ a little, and every cycle burns slightly differently.\n0 = machine-perfect (sterile), 1 = typical engine, 2+ = tired / lumpy.");
    double disp = displacementL(p_);
    ImGui::Text("Displacement %.2f L (%.0f cc/cyl)   bore/stroke %.2f", disp, disp * 1000 / p_.cylinders, p_.bore_mm / p_.stroke_mm);
    ImGui::Text("Mean piston speed at limiter: %.1f m/s", 2 * p_.stroke_mm * 1e-3 * p_.revLimit / 60.0);

    ImGui::SeparatorText("Firing order");
    const int n = p_.cylinders;
    Check("Even firing (from order)", &p_.evenFire);
    Help("Even firing spaces combustion events 720/N degrees apart, in the order below.\n"
         "Untick to set every cylinder's firing phase yourself (odd-fire V-twins, big-bang, crossplane tricks...).");
    {
        std::string ord;
        for (int k = 0; k < n; ++k) { ord += std::to_string(p_.firingOrder[k] + 1); if (k + 1 < n) ord += "-"; }
        ImGui::Text("Order: %s", ord.c_str());
    }
    if (ImGui::BeginTable("fo", p_.evenFire ? 3 : 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingFixedFit)) {
        for (int k = 0; k < n; ++k) {
            ImGui::PushID(k);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%2d.", k + 1);
            ImGui::TableNextColumn();
            int c = p_.firingOrder[k] + 1;
            ImGui::SetNextItemWidth(90);
            if (ImGui::InputInt("##cyl", &c)) p_.firingOrder[k] = clampv(c, 1, n) - 1;
            ImGui::TableNextColumn();
            if (ImGui::ArrowButton("##up", ImGuiDir_Up) && k > 0) std::swap(p_.firingOrder[k], p_.firingOrder[k - 1]);
            ImGui::SameLine();
            if (ImGui::ArrowButton("##dn", ImGuiDir_Down) && k + 1 < n) std::swap(p_.firingOrder[k], p_.firingOrder[k + 1]);
            if (!p_.evenFire) {
                ImGui::TableNextColumn();
                int cyl = p_.firingOrder[k];
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("##ph", &p_.phase[cyl], 0, 720, "fires @ %.0f deg");
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (!p_.evenFire && ImGui::Button("Reset phases to even spacing")) {
        EngineParams t = p_; t.evenFire = 1;
        float ph[kMaxCyl]; computePhases(t, ph);
        for (int i = 0; i < n; ++i) p_.phase[i] = ph[i];
    }
    if (const char* err = validateFiring(p_)) ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "%s", err);

    if (p_.layout != Layout::Inline) {
        ImGui::SeparatorText("Exhaust bank of each cylinder");
        Help("Which header (left / right) each cylinder exhausts into. This is what makes a crossplane V8 burble.");
        for (int i = 0; i < n; ++i) {
            ImGui::PushID(1000 + i);
            bool right = p_.bank[i] != 0;
            char lb[24]; std::snprintf(lb, sizeof(lb), "%d%s", i + 1, right ? "R" : "L");
            if (ImGui::Button(lb, ImVec2(42, 0))) p_.bank[i] = right ? 0 : 1;
            if ((i + 1) % 8 != 0 && i + 1 < n) ImGui::SameLine();
            ImGui::PopID();
        }
    }
    ImGui::PopItemWidth();
}

// ======================================================================= rotary engine
void App::panelRotary() {
    ImGui::SeparatorText("Rotors");
    if (SliderI("Rotors", &p_.cylinders, 1, 4, "Rotors are phased 360/N shaft degrees apart: 2-rotor fires every 180 deg (like an I4)."))
        applyLayoutDefaults(p_);
    SliderF("Generating radius R", &p_.rotorRadius_mm, 60, 150, "%.1f mm", "Centre-to-apex radius of the rotor (13B: 105 mm).");
    SliderF("Eccentricity e", &p_.eccentricity_mm, 8, 25, "%.1f mm", "Offset of the eccentric shaft lobe (13B: 15 mm). Bigger = more displacement, more torque.");
    SliderF("Rotor width B", &p_.rotorWidth_mm, 40, 120, "%.0f mm", "Housing width (13B: 80 mm, 12A: 70 mm, Renesis: 80 mm).");
    SliderF("Compression", &p_.compression, 7, 12, "%.1f : 1", "Set by the depth of the rotor face recess.");
    SliderF("Flywheel inertia", &p_.flywheel, 0.04f, 0.6f, "%.3f kg m2");
    SliderF("Imperfection", &p_.imperfection, 0, 2.5f, "%.2f", "Rotor-to-rotor and face-to-face spread plus cycle-to-cycle variation.");
    SliderF("Apex seal wear", &p_.apexLeak, 0, 1, "%.2f", "Worn apex seals leak compression into the next chamber:\nhard starting, rough idle, less power.");
    double vf = 3.0 * std::sqrt(3.0) * p_.rotorRadius_mm * p_.eccentricity_mm * p_.rotorWidth_mm * 1e-3;
    ImGui::Text("Chamber displacement %.0f cc  -> rated %.2f L (%d x %.0f cc)", vf, displacementL(p_), p_.cylinders, vf);
    ImGui::Text("Breathes like a %.1f L four-stroke   K = R/e = %.2f", breathL(p_), p_.rotorRadius_mm / p_.eccentricity_mm);
    ImGui::Text("Apex seal speed at limiter: %.1f m/s", (p_.rotorRadius_mm / 3.0 + p_.eccentricity_mm) * 1e-3 * p_.revLimit / 60.0 * 2 * kPi);
    ImGui::SeparatorText("Rotor phasing");
    for (int r = 0; r < p_.cylinders; ++r)
        ImGui::BulletText("Rotor %d: fires at %.0f deg of every shaft revolution", r + 1, 360.0 * r / p_.cylinders);
    ImGui::TextDisabled("Ports & timing are in the Ports tab. All angles are eccentric-shaft degrees.");
}

void App::panelPorts() {
    ImGui::PushItemWidth(-170);
    ImGui::SeparatorText("Porting style");
    for (int s = 0; s < 4; ++s) {
        if (s) ImGui::SameLine();
        if (ImGui::Button(rotaryPortingName(s))) applyRotaryPorting(p_, s);
    }
    Help("Side port: smooth idle, torque. Bridge port: extra hole across the apex seal path, huge overlap,\n"
         "the famous brap-brap idle. Peripheral port: intake in the rotor housing, race only.");
    ImGui::SeparatorText("Intake port");
    SliderF("Intake opens", &p_.rIntakeOpen, -90, 80, "%.0f ATDC", "Negative = before TDC: overlap with the exhaust port.");
    SliderF("Intake closes", &p_.rIntakeClose, 10, 110, "%.0f ABDC", "Later closing = more top end, less low-rpm torque.");
    SliderF("Intake port area", &p_.rIntakeArea_cm2, 2, 20, "%.1f cm2");
    ImGui::SeparatorText("Exhaust port");
    SliderF("Exhaust opens", &p_.rExhaustOpen, 40, 110, "%.0f BBDC");
    SliderF("Exhaust closes", &p_.rExhaustClose, 20, 100, "%.0f ATDC");
    SliderF("Exhaust port area", &p_.rExhaustArea_cm2, 2, 15, "%.1f cm2");
    ImGui::PopItemWidth();
    double io = 540 + p_.rIntakeOpen, ec = 540 + p_.rExhaustClose;
    ImGui::Text("Overlap %.0f deg   intake duration %.0f   exhaust duration %.0f", std::max(0.0, ec - io),
                270 + p_.rIntakeClose - p_.rIntakeOpen, 270 + p_.rExhaustOpen + p_.rExhaustClose);

    static float xs[1081], ai[1081], ae[1081], vol[1081];
    double R = p_.rotorRadius_mm, e = p_.eccentricity_mm, B = p_.rotorWidth_mm;
    double vd = 3.0 * std::sqrt(3.0) * R * e * B * 1e-3, vc = vd / (std::max(4.0f, p_.compression) - 1);
    auto port = [](double phi, double open, double close, double area, double ramp) {
        double len = wrapPos(close - open, 1080.0), d = wrapPos(phi - open, 1080.0);
        if (d >= len) return 0.0;
        return area * smoothstep(0.0, 1.0, std::min(d, len - d) / ramp);
    };
    float maxA = std::max(p_.rIntakeArea_cm2, p_.rExhaustArea_cm2);
    for (int a = 0; a <= 1080; ++a) {
        xs[a] = (float)a;
        ai[a] = (float)port(a, wrapPos(540 + p_.rIntakeOpen, 1080), wrapPos(810 + p_.rIntakeClose, 1080), p_.rIntakeArea_cm2, 30);
        ae[a] = (float)port(a, wrapPos(270 - p_.rExhaustOpen, 1080), wrapPos(540 + p_.rExhaustClose, 1080), p_.rExhaustArea_cm2, 18);
        vol[a] = (float)((vc + vd * 0.5 * (1 - std::cos(a * kDeg * 2.0 / 3.0))) / (vc + vd) * maxA);
    }
    if (ImPlot::BeginPlot("Port area (one chamber, one rotor turn)", ImVec2(-1, 260))) {
        ImPlot::SetupAxes("eccentric shaft angle (0 = firing TDC)", "port area cm2");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1080, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, maxA * 1.15, ImPlotCond_Always);
        ImPlot::SetNextFillStyle(ImVec4(0.3f, 0.6f, 1, 1), 0.25f);
        ImPlot::PlotShaded("Intake", xs, ai, 1081);
        ImPlot::SetNextLineStyle(ImVec4(0.3f, 0.6f, 1, 1), 2);
        ImPlot::PlotLine("Intake", xs, ai, 1081);
        ImPlot::SetNextFillStyle(ImVec4(1, 0.5f, 0.2f, 1), 0.25f);
        ImPlot::PlotShaded("Exhaust", xs, ae, 1081);
        ImPlot::SetNextLineStyle(ImVec4(1, 0.5f, 0.2f, 1), 2);
        ImPlot::PlotLine("Exhaust", xs, ae, 1081);
        ImPlot::SetNextLineStyle(ImVec4(0.7f, 0.7f, 0.7f, 0.6f), 1);
        ImPlot::PlotLine("Chamber volume (scaled)", xs, vol, 1081);
        double tdc[] = {0, 540, 1080};
        ImPlot::PlotInfLines("TDC", tdc, 3);
        ImPlot::EndPlot();
    }
}

// ======================================================================= valvetrain
// two-stroke (Package 5): port timing / widths, crankcase, reed, scavenging and the expansion chamber
void App::panelTwoStroke() {
    ImGui::PushItemWidth(-170);
    ImGui::SeparatorText("Cylinder ports");
    SliderF("Exhaust port opens", &p_.tsExhaustOpen, 60, 120, "%.0f deg ATDC", "The piston crown uncovers the exhaust port: earlier = longer exhaust\n"
            "duration, more top-end power (with a pipe tuned for it), less low-end.");
    SliderF("Transfer ports open", &p_.tsTransferOpen, std::min(170.0f, p_.tsExhaustOpen + 4.0f), 140, "%.0f deg ATDC",
            "After the exhaust: the time between the two (blowdown) lets the cylinder pressure fall below the\n"
            "crankcase's before the fresh charge comes in.");
    SliderF("Exhaust port width", &p_.tsExhaustWidth, 0.3f, 1.2f, "%.2f x bore");
    SliderF("Transfer ports width", &p_.tsTransferWidth, 0.3f, 2.0f, "%.2f x bore", "All transfer ports together.");
    {
        const double eo = p_.tsExhaustOpen, to = std::max((double)p_.tsTransferOpen, eo + 2.0);
        ImGui::Text("Exhaust duration %.0f deg   transfer duration %.0f deg   blowdown %.0f deg", 360.0 - 2.0 * eo, 360.0 - 2.0 * to, to - eo);
    }
    ImGui::SeparatorText("Power valve");
    Check("Power valve (exhaust timing control)", &p_.tsPowerValve);
    Help("A valve in the exhaust port's roof (Yamaha YPVS, Honda HPP, KTM...) that lowers the port's timing edge at low\n"
         "speed: a shorter exhaust period and a higher trapped compression ratio where the pipe cannot help, the full\n"
         "port up top. A servo lifts it with engine speed. It never seals like the piston: a little leaks past it.\n"
         "Blair's 125 racing engine: lowered it opens ~95-100 deg ATDC; fully lifted by 9500 rpm.");
    if (p_.tsPowerValve) {
        SliderF("Lowered: exhaust opens", &p_.tsPvLowOpen, p_.tsExhaustOpen, std::min(150.0f, p_.tsTransferOpen - 2.0f), "%.0f deg ATDC");
        SliderF("Starts to lift at", &p_.tsPvRpmLow, 1000, 15000, "%.0f rpm");
        SliderF("Fully lifted at", &p_.tsPvRpmHigh, p_.tsPvRpmLow, 16000, "%.0f rpm");
        ImGui::Text("Now %.0f %% lifted: exhaust duration %.0f .. %.0f deg", tel_.tsPowerValve * 100.0f, 360.0f - 2.0f * p_.tsPvLowOpen,
                    360.0f - 2.0f * p_.tsExhaustOpen);
    }
    ImGui::SeparatorText("Crankcase and inlet");
    SliderF("Crankcase compression", &p_.tsCrankcaseCR, 1.15f, 1.8f, "%.2f : 1", "Primary compression: crankcase volume with the piston at TDC over BDC.\n"
            "Higher = the crankcase pumps harder (more low-end), but holds less charge.");
    Combo("Inlet", &p_.tsInlet, "Reed valve\0Piston port (skirt)\0Disc (rotary) valve\0");
    Help("How the charge gets into the crankcase.\n"
         "Reed valve: spring petals that open whenever the crankcase is below the intake - one way, they adapt to speed.\n"
         "Piston port: the piston skirt uncovers the port around TDC - symmetric timing, and the crankcase spits some\n"
         "charge back out as the piston comes down again (strong ramming at its tuned speed, weak elsewhere).\n"
         "Disc valve: a cutaway disc on the crank - opens early, shuts early (MZ, Rotax racers).");
    if (p_.tsInlet == 0)
        SliderF("Reed valve area", &p_.tsReedArea_cm2, 4, 60, "%.0f cm2", "Flow area with the petals fully lifted (a spring-mass petal, 150 Hz).");
    else if (p_.tsInlet == 1) {
        SliderF("Inlet port opens", &p_.tsInletOpen, 40, 130, "%.0f deg BTDC", "The skirt uncovers it; it shuts as far after TDC. Blair (Fig. 1.9): ~65 deg on\n"
                "industrial / moped engines, ~75 enduro, ~100 motocross / GP.");
        SliderF("Inlet port area", &p_.tsInletArea_cm2, 2, 40, "%.1f cm2");
    } else {
        SliderF("Disc opens", &p_.tsInletOpen, 90, 170, "%.0f deg BTDC", "Blair: ~150 / 65 on a scooter, ~140 / 80 on a racing engine.");
        SliderF("Disc closes", &p_.tsInletClose, 30, 110, "%.0f deg ATDC");
        SliderF("Inlet port area", &p_.tsInletArea_cm2, 2, 40, "%.1f cm2", "The port window the disc uncovers (it opens and shuts over 56 deg).");
    }
    ImGui::TextDisabled("Intake tract (carb / throttle to the inlet): runner length and bore on the Intake tab - 1D gas dynamics.");
    SliderF("Transfer passage length", &p_.tsTransferLen, 0.3f, 3.0f, "%.2f x stroke",
            "Length of the gas column in the transfer passages, crankcase to port window. Its momentum keeps the charge\n"
            "flowing into the cylinder as the cylinder pressure comes back up: longer = more delivery at high revs.");
    ImGui::SeparatorText("Scavenging");
    {
        const int cur = clampv((int)p_.tsScavenge, 0, 9);
        char lbl[96];
        std::snprintf(lbl, sizeof(lbl), "%s - %s", kScavCylinders[cur].name, kScavCylinders[cur].type);
        if (ImGui::BeginCombo("Cylinder scavenging", lbl)) {
            for (int k = 0; k < 10; ++k) {
                std::snprintf(lbl, sizeof(lbl), "%s - %s", kScavCylinders[k].name, kScavCylinders[k].type);
                if (ImGui::Selectable(lbl, k == cur)) p_.tsScavenge = k;
            }
            ImGui::EndCombo();
        }
        Help("How well this cylinder's transfer ports sweep the burned gas out: cylinders measured on Blair's QUB\n"
             "single-cycle scavenging rig (Design and Simulation of Two-Stroke Engines, Fig. 3.16). YAM14 is a good\n"
             "5-port loop cylinder, YAM12 a poor one: at the same delivery a poor cylinder loses more fresh charge out\n"
             "of the exhaust and keeps more burned gas.");
    }
    ImGui::Text("Last cycle: delivery ratio %.2f   trapping eff. %.0f %%   scavenging eff. %.0f %%", tel_.tsDelivery,
                tel_.tsTrapping * 100.0f, tel_.tsScavenging * 100.0f);
    ImGui::Text("Crankcase %.0f kPa   reed petals %.0f %% open", tel_.tsCrankcaseKPa, tel_.tsReed * 100.0f);
    ImGui::SeparatorText("Combustion");
    SliderF("Burn period", &p_.tsBurnDeg, 30, 75, "%.0f deg", "How fast the combustion chamber burns the charge once it has lit (Vibe burn period). Set by\n"
            "the chamber and its squish, not by load or speed. Measured (Blair): 41 deg for a 125 cc Grand Prix\n"
            "engine, 49-60 deg for 400 cc research engines, 64 deg for a chainsaw. Before it comes an ignition\n"
            "delay of ~10-14 deg, longer with more burned gas left in the cylinder (up to ~23 deg at light load).");
    {
        ImGui::SeparatorText("Squish band");
        SliderF("Squish area ratio", &p_.tsSquishArea, 0.0f, 0.8f, "%.2f", "Share of the bore area covered by the squish band (central chamber).");
        SliderF("Squish clearance", &p_.tsSquishClear_mm, 0.6f, 4.0f, "%.2f mm", "Piston crown to head at TDC across the band.");
        ImGui::Text("Max squish velocity %.1f m/s, squished kinetic energy %.1f mJ (now, Blair Prog.4.1)", tel_.tsSquishMs, tel_.tsSquishKeMJ);
        Help("As the piston comes up, the band squeezes the charge into the bowl: the jet stirs the charge and speeds\n"
             "the flame. Blair: aim for 15-20 m/s at peak-power speed with a central chamber on gasoline - more and the\n"
             "end gas can knock; natural gas wants 30+.");
        Check("Burn period from the squish band", &p_.tsBurnFromSquish);
        Help("Flame speed = a part that grows with piston speed + the squish velocity (Blair eq. 4.3.1, Kee's flame\n"
             "speeds), over ~1 bore. Calibrated on one engine (QUB LS400): it gives the 125 GP engine ~47-60 deg where\n"
             "Blair measured 41 - a rough guide, not a prediction. Off: the burn period slider above.");
        if (p_.tsBurnFromSquish) ImGui::Text("Burn period in use: %.0f deg", tel_.tsBurnDeg);
    }
    ImGui::SeparatorText("Lubrication");
    SliderF("Premix fuel : oil", &p_.tsOilRatio, 0, 120, p_.tsOilRatio < 1.0f ? "oil pump (none in the fuel)" : "%.0f : 1",
            "Oil mixed into the fuel. The carburettor meters liquid by volume, so the oil takes the place of some\n"
            "gasoline: more oil runs leaner. About half of it burns (chosen); the rest leaves as blue smoke.\n"
            "0 = a separate oil pump (oil injection) - nothing in the fuel.");
    if (p_.tsOilRatio >= 1.0f) ImGui::Text("Unburned oil out of the exhaust: %.0f g/h", tel_.tsOilSmokeGh);
    ImGui::SeparatorText("Expansion chamber");
    Check("Expansion chamber (tuned pipe)", &p_.tsPipe);
    Help("Header, diverging cone, belly, converging cone, stinger. The blowdown pulse reflects off the diverging cone\n"
         "as suction (it pulls the burned gas out and the fresh charge up) and off the converging cone as a pressure\n"
         "wave that pushes the charge that ran into the header back into the cylinder just before the port closes.\n"
         "Off: an ordinary header / pipe (Exhaust tab).");
    if (p_.tsPipe) {
        SliderF("Tuned for", &p_.tsPipeRpm, 4000, 14000, "%.0f rpm", "Sets the pipe length: the wave's round trip takes the time the port is open.");
        SliderF("Pipe design", &p_.tsPipeClass, 0, 1, p_.tsPipeClass < 0.25f ? "%.2f enduro" : p_.tsPipeClass < 0.75f ? "%.2f motocross" : "%.2f road race",
                "Blair's empirical expansion chamber: broad enduro pipe (0) .. motocross (0.5) .. road racing (1).\n"
                "Racing: a first pipe barely wider than the port, a fatter belly and a narrower tail pipe -\n"
                "stronger suction and plugging, over a narrower speed range.");
        Check("Measured dimensions", &p_.tsPipeCustom);
        Help("Your own pipe's lengths and diameters (Blair's Fig. 5.7 segments) instead of the empirical design.\n"
             "The defaults are Blair's 125 cc racing engine pipe \"A2\".");
        if (p_.tsPipeCustom) {
            if (ImGui::SmallButton("Copy the empirical design")) {
                EngineParams q = p_; q.tsPipeCustom = 0;
                const TunedPipeGeom e = tunedPipeGeometry(q);
                for (int j = 0; j < 7; ++j) { p_.tsPipeLen_mm[j] = (float)(e.len[j] * 1e3); p_.tsPipeDia_mm[j] = (float)(e.dia[j] * 1e3); }
            }
            static const char* kSeg[7] = {"L1 first pipe", "L2 diffuser 1", "L3 diffuser 2", "L4 diffuser 3", "L5 belly", "L6 rear cone", "L7 tail pipe"};
            static const char* kDia[7] = {"d1 first pipe", "d2 end of L2", "d3 end of L3", "d4 belly", "d5 belly end", "d6 end of rear cone", "d7 tail pipe"};
            for (int j = 0; j < 7; ++j) {
                ImGui::PushID(j);
                SliderF(kSeg[j], &p_.tsPipeLen_mm[j], 10, 600, "%.0f mm");
                SliderF(kDia[j], &p_.tsPipeDia_mm[j], 10, 200, "%.1f mm");
                ImGui::PopID();
            }
        }
        const TunedPipeGeom g = tunedPipeGeometry(p_);
        ImGui::Text("Tuned length %.0f mm (port to tail pipe, waves at %.0f C)", g.lt * 1000.0, g.texcC);
        ImGui::Text("Lengths   %.0f / %.0f / %.0f / %.0f / %.0f / %.0f / %.0f mm", g.len[0] * 1e3, g.len[1] * 1e3, g.len[2] * 1e3,
                    g.len[3] * 1e3, g.len[4] * 1e3, g.len[5] * 1e3, g.len[6] * 1e3);
        ImGui::Text("Diameters %.0f (port) / %.0f / %.0f / %.0f / %.0f / %.0f / %.0f mm", g.d0 * 1e3, g.dia[0] * 1e3, g.dia[1] * 1e3,
                    g.dia[2] * 1e3, g.dia[3] * 1e3, g.dia[5] * 1e3, g.dia[6] * 1e3);
    }
    ImGui::PopItemWidth();
}

void App::panelValvetrain() {
    if (isRotary(p_)) { panelPorts(); return; }
    if (isTwoStroke(p_)) { panelTwoStroke(); return; }
    ImGui::PushItemWidth(-170);
    ImGui::SeparatorText("Layout");
    Combo("Camshafts", &p_.camLayout, "DOHC (two cams per head)\0SOHC (one cam per head)\0OHV (cam in the block, pushrods)\0");
    Combo("Cam drive", &p_.camDrive, "Toothed belt\0Chain\0Gears\0");
    Help("How the valves are worked and how the crank turns the cams. Drawn by the engine view; the\n"
         "simulation does not use them yet (the cam profiles below set the valve motion).");
    ImGui::SeparatorText("Intake");
    SliderI("Intake valves / cyl", &p_.intakeValves, 1, 3);
    SliderF("Intake valve dia", &p_.intakeValveDia_mm, 15, 60, "%.1f mm");
    SliderF("Intake duration", &p_.intakeDuration, 180, 340, "%.0f deg", "Longer duration = more top-end power, lumpier idle.");
    SliderF("Intake lift", &p_.intakeLift_mm, 4, 18, "%.1f mm");
    SliderF("Intake centerline", &p_.intakeCenterline, 90, 130, "%.0f ATDC");
    SliderF("Intake cam advance", &p_.intakeCamAdvance, -30, 40, "%.0f deg", "Variable cam timing (VVT): advance = earlier, more low-end.");
    ImGui::SeparatorText("Exhaust");
    SliderI("Exhaust valves / cyl", &p_.exhaustValves, 1, 3);
    SliderF("Exhaust valve dia", &p_.exhaustValveDia_mm, 15, 55, "%.1f mm");
    SliderF("Exhaust duration", &p_.exhaustDuration, 180, 340, "%.0f deg");
    SliderF("Exhaust lift", &p_.exhaustLift_mm, 4, 18, "%.1f mm");
    SliderF("Exhaust centerline", &p_.exhaustCenterline, 90, 130, "%.0f BTDC");
    SliderF("Exhaust cam advance", &p_.exhaustCamAdvance, -30, 40, "%.0f deg");
    SliderF("Valve lash", &p_.valveLash_mm, 0, 0.6f, "%.2f mm", "Too much lash shortens effective duration and clatters.");
    ImGui::SeparatorText("VTEC (two-stage cams)");
    Check("VTEC fitted", &p_.vtec);
    Help("A second, longer and higher cam profile per valve. Above the switch-over (Engine Control > Cam control)\n"
         "and under load the ECU locks the rockers onto it: the low cam gives a clean idle and torque down low,\n"
         "the high cam the top end. The switch takes ~80 ms of oil pressure.");
    if (p_.vtec) {
        SliderF("High cam intake duration", &p_.vtecInDuration, 180, 340, "%.0f deg");
        SliderF("High cam intake lift", &p_.vtecInLift_mm, 4, 18, "%.1f mm");
        SliderF("High cam exhaust duration", &p_.vtecExDuration, 180, 340, "%.0f deg");
        SliderF("High cam exhaust lift", &p_.vtecExLift_mm, 4, 18, "%.1f mm");
    }
    ImGui::SeparatorText("VTC (intake cam phaser)");
    Check("VTC fitted", &p_.vtc);
    Help("Continuously variable intake cam timing: the ECU advances the intake cam from its map (Engine Control >\n"
         "Cam control) - little overlap at idle and on the overrun, more mid-range torque under load.");
    if (p_.vtc) SliderF("Phaser range", &p_.vtcRange, 10, 60, "%.0f deg crank");
    ImGui::PopItemWidth();
    if (p_.vtec || p_.vtc)
        ImGui::Text("Now: %s cam, intake advanced %.1f deg", tel_.vtecEngaged > 0.5f ? "HIGH" : "low", tel_.vtcAdvance);

    double ivo = 360 + p_.intakeCenterline - p_.intakeCamAdvance - p_.intakeDuration / 2;
    double evc = 360 - p_.exhaustCenterline - p_.exhaustCamAdvance + p_.exhaustDuration / 2;
    double lsa = 0.5 * (p_.intakeCenterline - p_.intakeCamAdvance + p_.exhaustCenterline + p_.exhaustCamAdvance);
    ImGui::Text("Overlap %.0f deg   LSA %.1f deg", std::max(0.0, evc - ivo), lsa);

    static float xs[721], li[721], le[721], pis[721], hi[721], he[721];
    const double vtcNow = p_.vtc ? tel_.vtcAdvance : 0.0;
    double r = p_.stroke_mm * 0.5, l = p_.rod_mm;
    for (int a = 0; a <= 720; ++a) {
        xs[a] = (float)a;
        auto lift = [&](double center, double dur, double L) {
            double d = wrapSym(a - center, 720.0), h = dur / 2;
            if (std::fabs(d) >= h) return 0.0;
            return std::max(0.0, L * std::pow(0.5 * (1 + std::cos(kPi * d / h)), 0.75) - p_.valveLash_mm);
        };
        li[a] = (float)lift(360 + p_.intakeCenterline - p_.intakeCamAdvance - vtcNow, p_.intakeDuration, p_.intakeLift_mm);
        le[a] = (float)lift(360 - p_.exhaustCenterline - p_.exhaustCamAdvance, p_.exhaustDuration, p_.exhaustLift_mm);
        hi[a] = (float)lift(360 + p_.intakeCenterline - p_.intakeCamAdvance - vtcNow, p_.vtecInDuration, p_.vtecInLift_mm);
        he[a] = (float)lift(360 - p_.exhaustCenterline - p_.exhaustCamAdvance, p_.vtecExDuration, p_.vtecExLift_mm);
        double th = a * kDeg;
        double s = r * (1 - std::cos(th)) + l - std::sqrt(l * l - r * r * std::sin(th) * std::sin(th));
        pis[a] = (float)(s / (2 * r) * std::max(p_.intakeLift_mm, p_.exhaustLift_mm));
    }
    if (ImPlot::BeginPlot("Valve lift", ImVec2(-1, 260))) {
        ImPlot::SetupAxes("crank angle (0 = firing TDC)", "lift mm");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 720, ImPlotCond_Always);
        const float topLift = std::max(std::max(p_.intakeLift_mm, p_.exhaustLift_mm), p_.vtec ? std::max(p_.vtecInLift_mm, p_.vtecExLift_mm) : 0.0f);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, topLift * 1.15, ImPlotCond_Always);
        ImPlot::SetNextFillStyle(ImVec4(0.3f, 0.6f, 1, 1), 0.25f);
        ImPlot::PlotShaded("Intake", xs, li, 721);
        ImPlot::SetNextLineStyle(ImVec4(0.3f, 0.6f, 1, 1), 2);
        ImPlot::PlotLine("Intake", xs, li, 721);
        ImPlot::SetNextFillStyle(ImVec4(1, 0.5f, 0.2f, 1), 0.25f);
        ImPlot::PlotShaded("Exhaust", xs, le, 721);
        ImPlot::SetNextLineStyle(ImVec4(1, 0.5f, 0.2f, 1), 2);
        ImPlot::PlotLine("Exhaust", xs, le, 721);
        if (p_.vtec) {
            ImPlot::SetNextLineStyle(ImVec4(0.55f, 0.8f, 1, 0.9f), 1.5f);
            ImPlot::PlotLine("Intake (VTEC high cam)", xs, hi, 721);
            ImPlot::SetNextLineStyle(ImVec4(1, 0.75f, 0.45f, 0.9f), 1.5f);
            ImPlot::PlotLine("Exhaust (VTEC high cam)", xs, he, 721);
        }
        ImPlot::SetNextLineStyle(ImVec4(0.7f, 0.7f, 0.7f, 0.6f), 1);
        ImPlot::PlotLine("Piston (scaled)", xs, pis, 721);
        double tdc[] = {0, 360, 720};
        ImPlot::PlotInfLines("TDC", tdc, 3);
        ImPlot::EndPlot();
    }
}

// ======================================================================= ignition
void App::panelIgnition() {
    ImGui::PushItemWidth(-170);
    SliderF("Base advance", &p_.baseAdvance, -10, 40, "%.1f BTDC", "Static timing at idle.");
    SliderF("Mechanical advance", &p_.mechAdvance, 0, 35, "%.1f deg", "Extra advance added with rpm (centrifugal).");
    SliderF("All-in by", &p_.mechAdvanceRpm, 1000, 8000, "%.0f rpm");
    SliderF("Vacuum advance", &p_.vacuumAdvance, 0, 20, "%.1f deg", "Extra advance at light load (high manifold vacuum).");
    ImGui::SeparatorText("Rev limiter");
    SliderF("Rev limit", &p_.revLimit, 1500, 16000, "%.0f rpm");
    {
        float rl = redlineRpm(p_);
        if (SliderF("Redline", &rl, 1000, std::max(1500.0f, p_.revLimit), "%.0f rpm", "Start of the tach red zone and the shift light (default: 300 rpm below the limiter)."))
            p_.redline = rl;
    }
    SliderF("Friction", &p_.frictionMult, 0.5f, 2.0f, "x%.2f", "Mechanical friction multiplier on top of the parts (rings, springs, pumps, oil\n"
            "temperature): < 1 = a freshly built race engine, > 1 = worn / tight / heavy oil.");
    Combo("Limiter type", &p_.limiter, "Ignition cut (pops & bangs)\0Fuel cut (EFI only)\0");
    Help("Ignition cut keeps fuel flowing: unburned fuel ignites in the hot header = crackles.\nCarbureted engines always use ignition cut.");
    ImGui::PopItemWidth();

    static float rp[100], wot[100], part[100];
    for (int i = 0; i < 100; ++i) {
        float rpm = 500 + i * (p_.revLimit + 500) / 100.0f;
        rp[i] = rpm;
        double mech = p_.mechAdvance * smoothstep(900, std::max(1000.0f, p_.mechAdvanceRpm), rpm);
        wot[i] = (float)(p_.baseAdvance + mech);
        part[i] = wot[i] + p_.vacuumAdvance;
    }
    if (ImPlot::BeginPlot("Advance curve", ImVec2(-1, 240))) {
        ImPlot::SetupAxes("rpm", "deg BTDC");
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, p_.revLimit + 1000, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 60, ImPlotCond_Always);
        ImPlot::PlotLine("Full load", rp, wot, 100);
        ImPlot::PlotLine("Part load", rp, part, 100);
        double now[] = {tel_.rpm}, adv[] = {tel_.advance};
        ImPlot::PlotScatter("Now", now, adv, 1);
        ImPlot::EndPlot();
    }
    ImGui::Text("Knock index %.2f %s", tel_.knock, tel_.knock > 1 ? "  <- KNOCKING: retard timing or lower CR" : "");
}

// ======================================================================= fuel & intake
void App::panelFuelIntake() {
    ImGui::PushItemWidth(-170);
    ImGui::SeparatorText("Air path");
    SliderF("Throttle bore", &p_.throttleBore_mm, 20, 110, "%.0f mm", "Per barrel (carb) / per cylinder (ITB).");
    Check("Individual throttle bodies (ITB)", &p_.itb);
    Help("One throttle per cylinder, no plenum: sharp response and loud induction howl.");
    if (!p_.itb) SliderF("Plenum volume", &p_.plenum_L, 0.3f, 12, "%.1f L");
    SliderF("Runner length", &p_.runnerLen_mm, 60, 800, "%.0f mm", "Intake ram tuning: long = low-rpm torque, short = top end.");
    SliderF("Runner diameter", &p_.runnerDia_mm, 20, 70, "%.0f mm");
    SliderF("Air filter restriction", &p_.airFilter, 0, 1, "%.2f", "0 = open velocity stacks (loud), 1 = restrictive stock airbox.");
    SliderF("Ambient temp", &p_.ambientTemp_C, -20, 45, "%.0f C");
    altitudeUi("Altitude");

    ImGui::SeparatorText("Fuel");
    if (Combo("Fill the tank with", &p_.fuelType, fuelTypeNames())) {}
    Help("Choosing a fuel drains the tank and fills it with that fuel. It reaches the injectors over about half a\n"
         "minute (the fuel line and rail are purged through the return line). Each fuel brings its own stoichiometric\n"
         "AFR, heating value, octane, charge cooling, flame speed and cold-start behaviour.\n"
         "Without a flex fuel sensor the ECU keeps calculating with its calibration's fuel - re-tune after a change.");
    ImGui::TextDisabled("%s", fuelTypeNote(p_.fuelType));
    {
        char b[160];
        std::snprintf(b, sizeof(b), "%.1f / %.0f L", tel_.tankLitres, tel_.tankCap);
        ImGui::ProgressBar(tel_.tankCap > 0 ? tel_.tankLitres / tel_.tankCap : 0, ImVec2(-170, 0), b);
        ImGui::SameLine(); ImGui::TextUnformatted("Tank");
        ImGui::Text("Tank: %.0f %% alcohol, RON %.1f      Fuel line (burning now): %.0f %% alcohol, RON %.1f / MON %.1f",
                    tel_.tankAlcohol, tel_.tankRon, tel_.fuelAlcohol, tel_.fuelRon, tel_.fuelMon);
        ImGui::Text("Stoich AFR %.2f   heating value %.1f MJ/kg   lambda %.3f", tel_.fuelStoich, tel_.fuelLhv, tel_.lambda);
    }
    ImGui::PushItemWidth(ImGui::GetFontSize() * 14);
    Combo("##refueltype", &p_.refuelType, fuelTypeNames());
    ImGui::PopItemWidth();
    ImGui::SameLine();
    ImGui::PushItemWidth(ImGui::GetFontSize() * 8);
    ImGui::SliderFloat("##refuellitres", &p_.refuelLitres, 1, 50, "%.0f L");
    ImGui::PopItemWidth();
    ImGui::SameLine();
    if (ImGui::Button("Refuel")) ++p_.refuelRequest;
    ImGui::SameLine();
    if (ImGui::Button("Empty tank")) ++p_.drainRequest;
    if (tel_.fuelSupply < 0.98f || tel_.tankLitres < 0.5f)
        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), tel_.fuelSupply < 0.02f ? "Out of fuel - the pump is sucking air. Refuel to run again."
                                                                             : "Fuel low - the pickup is gulping air (fuel supply %.0f %%)", tel_.fuelSupply * 100);
    if (isDiesel(p_) != isDieselFuel(p_.fuelType))
        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), isDiesel(p_) ? "Petrol / alcohol in a diesel: outside what the model covers (its ignition delay is a diesel-fuel correlation)."
                                                                  : "Diesel in a petrol engine is not supported - it will not run on it.");
    Help("Adds this many litres of that fuel to what is in the tank (as far as it fits): blends mix by volume.\n"
         "E.g. 20 L of E85 on top of 30 L of premium gives about E34 - the flex fuel sensor follows it.\n"
         "Empty tank drains it: below about half a litre the pickup gulps air, the fuel pressure sags and the engine\n"
         "stumbles and dies (a carburettor first runs its float bowl dry; a diesel's injection pump loses its prime\n"
         "and needs a few seconds with the ignition on after refuelling while the lift pump pushes the air out).");
    if (p_.fuelSystem == FuelSystem::EFI) {
        Check("Flex fuel sensor fitted", &p_.flexSensor);
        Help("Measures the alcohol content in the fuel line (50 Hz = E0 .. 150 Hz = E100). The ECU then sets its\n"
             "stoichiometric AFR, injector mass flow, spark and cold enrichment from it (Fuel Tuning > Flex fuel).\n"
             "Without it (or when it fails) the ECU uses the fallback alcohol % of its calibration.");
    }

    ImGui::SeparatorText("Fuel system");
    int fs = (int)p_.fuelSystem;
    ImGui::RadioButton("Fuel injection (EFI)", &fs, 0); ImGui::SameLine();
    ImGui::RadioButton("Carburetor", &fs, 1); ImGui::SameLine();
    ImGui::RadioButton("Diesel (direct injection)", &fs, 2);
    Help("Diesel: the fuel is injected into the cylinder near top dead centre and lights by itself in the hot\n"
         "compressed air (needs a high compression ratio, ~16-22:1). No throttle: the load is the amount of fuel.\n"
         "Switching fills the tank with the matching fuel.");
    if ((FuelSystem)fs != p_.fuelSystem) {
        const bool toDiesel = (FuelSystem)fs == FuelSystem::DieselDI;
        if (toDiesel && !isDieselFuel(p_.fuelType)) p_.fuelType = kFuelDiesel;
        if (!toDiesel && isDieselFuel(p_.fuelType)) p_.fuelType = kFuelPremium95;
    }
    p_.fuelSystem = (FuelSystem)fs;
    if (p_.fuelSystem == FuelSystem::DieselDI) {
        Combo("Injection system", &p_.dieselInjSystem, "Cam-driven pump / unit injector\0Common rail\0");
        Help("Pump: the fuel goes in at a fixed rate (the pump's delivery). Common rail: a high-pressure pump keeps the\n"
             "rail at the ECU's pressure; each injector's nozzle holes flow like orifices at that pressure, so higher\n"
             "pressure = shorter injection. Common rail also allows a second pilot and a post injection (ECU tab > Diesel).");
        if (p_.dieselInjSystem == 1) {
            SliderF("Rail pressure, maximum", &p_.railMaxBar, 300, 2700, "%.0f bar", "The top of the ECU's rail pressure map (EA189: 1800 bar; Duramax L5P: 2000 bar).\nRegenerate the calibration after a big change.");
            SliderI("Nozzle holes", &p_.nozzleHoles, 1, 12);
            SliderF("Hole diameter", &p_.nozzleHoleDia_mm, 0.05f, 0.30f, "%.3f mm", "Flow ~ holes x diameter^2 x sqrt(rail - chamber pressure); Cd 0.86 (ECN Spray A).");
            ImGui::Text("Rail %.0f bar (target %.0f)   high-pressure pump %.1f kW", tel_.railBar, tel_.railTargetBar, tel_.pumpKw);
        } else
            SliderF("Injection rate", &p_.dieselInjRate, 5, 150, "%.0f mg/ms", "What one injector delivers per millisecond at its rail pressure: sets\nhow long an injection lasts (more fuel or a weaker injector = a longer, later-burning injection).");
        SliderF("Full-pedal fuel", &p_.dieselMaxFuel, 5, 200, "%.0f mg/stroke", "The driver's demand map scales to this (the smoke limiter can hold it back).\nRegenerate the calibration (ECU tab) after a big change.");
        SliderF("Injection timing at idle", &p_.soiBase, -5, 20, "%.1f deg BTDC");
        SliderF("Speed advance", &p_.soiAdvance, 0, 25, "%.1f deg", "Added towards rated speed (like a mechanical pump's timing device):\nthe ignition delay takes more crank angle at speed.");
        SliderF("Idle target", &p_.idleTargetRpm, 500, 1500, "%.0f rpm");
        Combo("Glow plugs", &p_.glowPlugs, "None\0Metal (850 C in < 5 s)\0Ceramic (1000 C in < 2 s)\0");
        Help("The hot tip lights the fuel when the compressed air alone is too cold (a DI diesel needs them below about\n"
             "-5 to -10 C). Key on: the glow lamp stays on while they pre-heat - wait for it before cranking. They keep\n"
             "heating while cranking and after the start until the coolant reaches 70 C (quieter, less white smoke).\n"
             "Heat-up times: Bosch glow plug brochure; pre-heat schedule: BorgWarner / BERU (ECU tab > Diesel).");
        ImGui::Text("Glow plug tip %.0f C  %s", tel_.glowTipC, tel_.glowLight ? "PRE-HEATING (wait)" : tel_.glowOn ? "heating" : "off");
        SliderF("EGR valve", &p_.egrValveDia_mm, 0, 50, "%.0f mm", "Exhaust gas recirculation: a valve from the exhaust manifold (before the turbine) into the\n"
                "intake (0 = none). Burned gas in the charge takes the place of fresh air - less NOx in a real engine, a\n"
                "little less oxygen. Hot unless a cooler is fitted. Opening: ECU tab > Diesel > EGR map.");
        if (p_.egrValveDia_mm > 0) {
            Check("EGR cooler (water-cooled)", &p_.egrCooler);
            Help("Cools the recirculated gas towards the coolant: T_out = T_in - effectiveness x (T_in - T_coolant). A cooler\n"
                 "charge holds more oxygen and lowers the flame temperature further (less NOx). Clean coolers measure\n"
                 "~0.87-0.97, fouling takes 20-30 %% off (patents US 7281529, SAE 2008-01-2475).");
            if (p_.egrCooler) SliderF("Cooler effectiveness", &p_.egrCoolerEff, 0.3f, 0.98f, "%.2f");
            ImGui::Text("EGR valve %.0f %%   %.1f g/s   %.0f %% of the charge   gas in %.0f C", tel_.egrPos * 100.0f, tel_.egrGs, tel_.egrRate * 100.0f, tel_.egrGasC);
        }
        ImGui::Text("Engine-out: NOx %.0f ppm (%.2f g/s)   soot %.1f mg/m3 = FSN %.2f", tel_.noxPpm, tel_.noxGs, tel_.sootMgM3, tel_.smokeFsn);
        Help("NOx: Zeldovich formation (Heywood's rate) in the burned gas; soot: Hiroyasu's formation / oxidation; the smoke\n"
             "number from the AVL relation. Readouts only - their absolute levels are plausibility calibrations (trends:\n"
             "EGR and later timing cut NOx; EGR and a richer mixture raise soot). docs/ECU_PLAN.md D3.");
        ImGui::Text("Fuel %.1f mg/stroke   start of injection %.1f deg BTDC   ignition delay %.1f deg   %s", tel_.dieselFuelMg, tel_.soi,
                    tel_.ignDelayDeg, tel_.smokeLimited ? "SMOKE LIMITED" : "");
        if (p_.compression < 15.0f)
            ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Compression %.1f:1 is low for a diesel - it may not light (16-22:1 typical).", p_.compression);
    } else if (p_.fuelSystem == FuelSystem::EFI) {
        Combo("Injection", &p_.injection, "Port injection\0Throttle body injection\0");
        SliderF("Injector size", &p_.injector_ccmin, 60, 1200, "%.0f cc/min", "Too small = runs lean at high rpm (watch injector duty).");
        SliderF("AFR cruise", &p_.afrCruise, 11, 17, "%.1f");
        SliderF("AFR full load", &p_.afrWot, 10, 15, "%.1f", "Best power ~12.5-13. Leaner = hotter and knock prone.");
        SliderF("End of injection", &p_.injectionAngle, 0, (float)cycleDeg(p_), "%.0f deg");
        SliderF("Idle target", &p_.idleTargetRpm, 500, 2000, "%.0f rpm");
        Check("Tuned ECU (VE learning + closed loop)", &p_.veLearning);
        Help("On: the ECU tracks the real airflow (well tuned car).\nOff: fixed VE guess - AFR wanders with rpm and load like a bad tune.");
        ImGui::Text("Injector duty %.0f%%   target AFR %.1f", tel_.injDuty * 100, tel_.targetAfr);
    } else {
        SliderI("Barrels", &p_.carbBarrels, 1, 4);
        SliderF("Venturi", &p_.venturi_mm, 18, 50, "%.0f mm");
        SliderF("Main jet", &p_.mainJet, 0.6f, 1.4f, "x%.2f", "1.00 = ~12.8 AFR at full throttle. Bigger = richer.");
        SliderF("Idle mixture", &p_.idleMixture, 0.5f, 1.5f, "x%.2f");
        SliderF("Idle speed screw", &p_.idleScrew, 0, 12, "%.1f", "Opens the throttle a crack: sets idle rpm.");
        SliderF("Accelerator pump", &p_.accelPump, 0, 2.5f, "x%.2f", "Squirts fuel when you stab the throttle. 0 = bog / stumble.");
        SliderF("Choke", &p_.choke, 0, 1, "%.2f");
        ImGui::Text("Main-jet AFR ~%.1f", 12.8f / std::max(0.3f, p_.mainJet));
    }
    ImGui::Text("Measured AFR %.2f   MAP %.1f kPa   air %.1f g/s", tel_.afr, tel_.mapKPa, tel_.airFlow);
    ImGui::PopItemWidth();
}

// ======================================================================= exhaust
void App::panelExhaust() {
    ExhaustSchematic(p_, ImVec2(ImGui::GetContentRegionAvail().x, 170));
    ImGui::PushItemWidth(-170);
    ImGui::SeparatorText("Headers");
    if (p_.induction == Induction::Turbo) {
        // a turbo manifold: what the cylinders' pulses do before the turbine
        static const HeaderType kT[] = {HeaderType::Log, HeaderType::Collector, HeaderType::TwinScroll};
        static const char* kTn[] = {"Cast log manifold", "Tubular (equal length)", "Twin-scroll / divided"};
        int cur = p_.header == HeaderType::Log ? 0 : p_.header == HeaderType::TwinScroll ? 2 : 1;
        if (ImGui::Combo("Turbo manifold", &cur, kTn, 3)) p_.header = kT[cur];
        Help("Log: compact cast manifold - the pulses run into each other before the turbine.\n"
             "Tubular: a header of equal-length runners into the turbine flange.\n"
             "Twin-scroll / divided: the cylinders in two groups by firing order (an I4: 1+4 and 2+3, an I6: 1-2-3 and\n"
             "4-5-6), each into its own scroll of the turbine housing - one cylinder's blowdown cannot push back into\n"
             "another's exhaust stroke, and the pulses keep their energy for the turbine (Watson & Janota: pulse systems).\n"
             "Twin turbos: each turbo gets its own manifold (per bank, or front / rear).");
    } else
    Combo("Header type", &p_.header, "Log manifold\0N-into-1 collector\0Tri-Y / 4-2-1\0Open stacks (zoomies)\0");
    Help("Log: compact cast manifold, muffled.\nCollector: long tube headers, every pulse separate.\nTri-Y: pairs cylinders 360 deg apart (smooth mid-range).\n"
         "Open stacks: every primary ends in the open air - no collector, cat or silencer (drag / race stacks).\n"
         "With a turbo the primaries go to the turbine instead.");
    const bool stacks = p_.header == HeaderType::OpenStacks && p_.induction != Induction::Turbo && !(isTwoStroke(p_) && p_.tsPipe);
    SliderF("Primary diameter", &p_.primaryDia_mm, 25, 60, "%.0f mm");
    if (ImGui::Checkbox("Equal length primaries", &equalPrimaries_) && equalPrimaries_)
        for (int i = 1; i < kMaxCyl; ++i) p_.primaryLen_mm[i] = p_.primaryLen_mm[0];
    if (equalPrimaries_) {
        if (SliderF("Primary length", &p_.primaryLen_mm[0], 80, 1500, "%.0f mm"))
            for (int i = 1; i < kMaxCyl; ++i) p_.primaryLen_mm[i] = p_.primaryLen_mm[0];
    } else {
        Help("Unequal lengths make pulses arrive unevenly at the collector: the classic boxer rumble.");
        for (int i = 0; i < p_.cylinders; ++i) {
            char lb[32]; std::snprintf(lb, sizeof(lb), "Primary %d length", i + 1);
            ImGui::SliderFloat(lb, &p_.primaryLen_mm[i], 80, 1500, "%.0f mm");
        }
    }
    if (p_.header == HeaderType::TriY) SliderF("Secondary length", &p_.secondaryLen_mm, 80, 1000, "%.0f mm");
    if (!stacks) {
        SliderF("Collector length", &p_.collectorLen_mm, 50, 800, "%.0f mm");
        SliderF("Collector diameter", &p_.collectorDia_mm, 30, 110, "%.0f mm");

        bool twoBanks = false;
        if (p_.layout != Layout::Inline) {
            bool b0 = false, b1 = false;
            for (int i = 0; i < p_.cylinders; ++i) (p_.bank[i] ? b1 : b0) = true;
            twoBanks = b0 && b1;
        }
        ImGui::SeparatorText("System");
        if (twoBanks) {
            Combo("Crossover", &p_.crossover, "True dual (separate)\0H-pipe\0X-pipe\0Y (merge into one)\0");
            Help("True dual: each bank on its own (left/right stereo).\nH-pipe: deep, balanced.\nX-pipe: smoother, raspier.\nY: single exit.");
        }
        SliderF("Pipe diameter", &p_.pipeDia_mm, 30, 110, "%.0f mm");
        SliderF("Mid-pipe length", &p_.midpipeLen_mm, 100, 4000, "%.0f mm");
        Combo("Catalytic converter", &p_.cat, "None\0Ceramic (OEM)\0High-flow metallic\0");
        Help("Ceramic: the OEM brick - wide, lossy, takes the edge off.\nHigh-flow metallic: thin foil, bigger cells - less\n"
             "back pressure, more of the sound through.");
        Combo("Resonator", &p_.resonator, "None\0Inline chamber\0Helmholtz (tuned side branch)\0");
        Help("Inline chamber: a short expansion chamber in the pipe.\n"
             "Helmholtz: a neck into a closed volume beside the pipe, sized for one frequency - it cancels the pipe's wave\n"
             "there (the cruise \"drone\"). Auto tunes it to the firing frequency at 2000 rpm.");
        if (p_.resonator == 2) {
            const double perRev = p_.cylinders / revsPerFiring(p_);
            char lb[48];
            std::snprintf(lb, sizeof(lb), p_.resonatorHz > 1 ? "%.0f Hz" : "auto (%.0f Hz)", p_.resonatorHz > 1 ? p_.resonatorHz : perRev * 2000.0 / 60.0);
            ImGui::SliderFloat("Resonator tuning", &p_.resonatorHz, 0, 400, lb);
            Help("Frequency the Helmholtz resonator cancels; 0 = auto. Firing frequency = cylinders / 2 x rpm / 60.");
        }
        {
            static const Muffler kOrder[] = {Muffler::None, Muffler::Glasspack, Muffler::StraightThrough, Muffler::Chambered, Muffler::Valved,
                                             Muffler::Turbo, Muffler::ReverseFlow};
            static const char* kName[] = {"None (straight pipe)", "Straight-through (perforated, packed)", "Chambered", "Turbo (quiet)", "Glasspack",
                                          "Reverse-flow 3-pass (OEM)", "Valved / active (flap bypass)"};
            if (ImGui::BeginCombo("Muffler", kName[clampv((int)p_.muffler, 0, 6)])) {   // (loudest first)
                for (Muffler m : kOrder) if (ImGui::Selectable(kName[(int)m], m == p_.muffler)) p_.muffler = m;
                ImGui::EndCombo();
            }
            Help("None: open pipe.\nGlasspack: a short perforated core in a thin wrap - loud, raspy.\n"
                 "Straight-through: a longer packed core - absorbs the highs, keeps the bass.\n"
                 "Chambered: reactive chambers - the resonant muscle-car tone.\n"
                 "Valved: a chambered box with a straight bypass behind a flap (active exhaust): quiet when cruising, open\n"
                 "when pushed.\nTurbo: chambers and absorbing tubes - quiet, mellow.\n"
                 "Reverse-flow: the OEM three-pass box - the gas turns twice through perforated tubes; the quietest.");
        }
        if (p_.muffler == Muffler::Valved) {
            Combo("Flap", &p_.exhaustValveMode, "Auto (speed / load)\0Always open (sport)\0Always shut (quiet)\0");
            if (p_.exhaustValveMode == 0) {
                SliderF("Opens above", &p_.exhaustValveRpm, 1500, 8000, "%.0f rpm");
                Help("Auto: opens above this speed or past 60 % pedal; shuts 300 rpm below it with the pedal under 45 %.");
            }
            if (tel_.exhValve >= 0) ImGui::Text("Flap %.0f %% open", tel_.exhValve * 100.0f);
        }
        if (p_.muffler == Muffler::Chambered) {
            Check("Extended inlet / outlet tubes", &p_.mufflerExtTubes);
            Help("Tubes reaching L/2 (inlet) and L/4 (outlet) into each chamber: the closed spaces around them are\n"
                 "quarter-wave resonators that fill the chambers' transmission-loss dips - fewer pass bands.");
        }
        if (p_.induction == Induction::Turbo && !p_.vgt) {
        Check("Screamer pipe (wastegate to the air)", &p_.wgScreamer);
        Help("An external wastegate with its own short pipe out to the air instead of back into the downpipe: when it\n"
             "opens on boost its gas skips the silencers - the open-pipe rasp of a race turbo car.");
    }
    Check("Split into two mufflers and tips", &p_.exhaustSplit);
        Help("A Y after the cat / resonator: each branch (0.6 x the pipe's area) gets its own muffler and tip - a single\n"
             "pipe becomes a dual exit, a true dual becomes four tips.");
        SliderF("Tailpipe length", &p_.tailpipeLen_mm, 50, 2500, "%.0f mm");
    } else ImGui::TextDisabled("Open stacks: the primaries are the whole exhaust (no collector, cat or silencer).");

    // ---- exhaust sensors (Package 2)
    ImGui::SeparatorText("Exhaust sensors");
    const bool turbo = p_.induction == Induction::Turbo;
    Check("EMAP sensor", &p_.emapSensor);
    Help("Exhaust manifold pressure: before the turbine on a turbo engine. With the boost it gives the drive\n"
         "pressure ratio (EMAP / MAP) - above ~2 the turbine is choking the engine: hot charge, high residual gas,\n"
         "EGT and head gasket stress (protection rule 'High drive pressure').");
    if (turbo) {
        Combo("Wideband position", &p_.widebandPos, "Collector (before the turbine)\0Downpipe (after the turbine)\0");
        Help("The wideband reads the gas in the pipe, not the cylinders: misfires read lean (unburned fuel diffuses\n"
             "through the sensor more slowly than oxygen), fresh air blown through on the overlap reads lean, rich\n"
             "products read rich. After the turbine the gas is better mixed but reaches the sensor later.");
        Check("EGT after the turbine", &p_.egtPostTurbo);
        Help("A second thermocouple in the downpipe: the drop across the turbine shows the work it takes out.");
    }
    Check("EGT in every primary", &p_.egtPerCyl);
    Help("One thermocouple per cylinder: a lean or misfiring cylinder shows up at once (the collector probe only\n"
         "sees the mix). The hottest one also feeds the High EGT protection rule.");
    ImGui::PopItemWidth();
    ImGui::Text("Wideband %.3f (gas at the sensor %.3f, in the cylinders %.3f)", tel_.snsValue[kSnsWideband], tel_.lambdaPipe, tel_.lambda);
    if (p_.emapSensor) {
        if (tel_.mapKPa > tel_.baroKPa + 5)
            ImGui::Text("EMAP %.0f kPa   MAP %.0f kPa   drive pressure ratio %.2f", tel_.snsValue[kSnsEMAP], tel_.mapKPa, tel_.driveRatio);
        else ImGui::Text("EMAP %.0f kPa   MAP %.0f kPa   (drive pressure ratio: under boost)", tel_.snsValue[kSnsEMAP], tel_.mapKPa);
    }
    ImGui::Text("EGT collector %.0f C", tel_.snsValue[kSnsEGT]);
    if (turbo && p_.egtPostTurbo) { ImGui::SameLine(); ImGui::Text("   after the turbine %.0f C (turbine drop %.0f K)", tel_.snsValue[kSnsEGT2], tel_.turbineDropK); }
    if (p_.egtPerCyl && tel_.cylinders > 0) {
        const int n = std::min(tel_.cylinders, kMaxCyl);
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < n; ++i) { lo = std::min(lo, tel_.egtCylC[i]); hi = std::max(hi, tel_.egtCylC[i]); }
        ImGui::Text("EGT per cylinder (spread %.0f K):", hi - lo);
        const float w = (ImGui::GetContentRegionAvail().x - 8.0f * (n - 1)) / n;
        for (int i = 0; i < n; ++i) {
            if (i) ImGui::SameLine(0, 8);
            char b[32]; std::snprintf(b, sizeof(b), "%d: %.0f", i + 1, tel_.egtCylC[i]);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, tel_.egtCylC[i] >= hi - 0.5f && hi - lo > 15 ? ImVec4(1, 0.45f, 0.2f, 1) : ImVec4(0.55f, 0.3f, 0.13f, 1));
            ImGui::ProgressBar(std::clamp(tel_.egtCylC[i] / 1100.0f, 0.0f, 1.0f), ImVec2(w, 0), b);
            ImGui::PopStyleColor();
        }
    }
    ImGui::TextDisabled("Topology changes (header type, crossover, cat/resonator, muffler) rebuild the\nwave network live with a short crossfade; lengths & diameters glide smoothly.");
}

// ======================================================================= boost
void App::panelBoost() {
    ImGui::PushItemWidth(-170);
    int ind = (int)p_.induction;
    ImGui::RadioButton("Naturally aspirated", &ind, 0); ImGui::SameLine();
    ImGui::RadioButton("Turbocharger", &ind, 1); ImGui::SameLine();
    ImGui::RadioButton("Supercharger", &ind, 2);
    if (ind != (int)p_.induction) applyInductionDefaults(p_, (Induction)ind, p_.scType);
    if (!isBoosted(p_)) {
        ImGui::TextDisabled("Pick a turbo or a supercharger to force more air into the engine.");
        ImGui::PopItemWidth();
        return;
    }
    if (p_.induction == Induction::Turbo) {
        ImGui::SeparatorText("Turbocharger");
        {
            int cnt = p_.turboCount >= 2 ? 2 : 1;
            if (ImGui::RadioButton("Single turbo", &cnt, 1)) applyTurboCount(p_, 1);
            ImGui::SameLine();
            if (ImGui::RadioButton("Twin turbo", &cnt, 2)) applyTurboCount(p_, 2);
            Help("Twin: two smaller turbos (each sized for half the engine: they spool quicker). Sizes below are each one's.");
            if (p_.turboCount >= 2) {
                int mode = p_.twinMode;
                if (ImGui::Combo("Arrangement", &mode, "Parallel\0Sequential\0")) applyTwinMode(p_, mode);
                if (p_.twinMode == 0)
                    Help(p_.layout == Layout::Inline ? "Parallel on an inline engine: the front and the rear cylinders each drive their own turbo\n(a split manifold, RB26-style); both blow into one intercooler." :
                         "Parallel on a V / boxer: one turbo per bank, each bank's exhaust drives its own; both blow into\none intercooler. True dual crossover: a downpipe and exhaust per bank, else they merge.");
                else {
                    SliderF("Second turbo from", &p_.seqRpm, 2500, 7000, "%.0f rpm");
                    Help("Sequential (Mazda 13B-REW, Toyota 2JZ-GTE): one manifold; the first turbo takes all the exhaust down low.\n"
                         "1200 rpm before the change-over a bypass valve starts spinning the second (pre-spool); at it the exhaust\n"
                         "control valve opens to it, and its compressor joins the charge only once it has caught up. Under load\n"
                         "(pedal past 40 %) only.");
                    static const char* kStage[] = {"first turbo alone", "second turbo spooling", "both turbos"};
                    ImGui::Text("Now: %s   (second %.0f krpm)", kStage[clampv(tel_.seqStage, 0, 2)], tel_.shaftRpm2 / 1000.0f);
                }
                if (p_.twinMode == 0) ImGui::Text("Turbo speeds %.0f / %.0f krpm", tel_.shaftRpm / 1000.0f, tel_.shaftRpm2 / 1000.0f);
            }
        }
        turboModelUi();
        if (!p_.turbo.useMaps) {
            SliderF("Compressor size", &p_.compFlow_gs, 60, 900, "%.0f g/s max flow", "Bigger compressor = more top-end power, but needs more exhaust energy to spin (lag).");
            SliderF("Turbine A/R", &p_.turbineAR, 0.3f, 1.4f, "%.2f", "Small A/R: spools early, chokes up top (backpressure).\nLarge A/R: lazy spool, big top end.");
        }
        SliderF("Rotor inertia", &p_.turboInertia, 0.03f, 0.8f, "%.2f kg cm2", "Heavier wheels = more lag.");
        Check("Variable geometry (VGT)", &p_.vgt);
        Help("Guide vanes in the turbine housing instead of a wastegate: closed they make a small, fast turbine (boost\n"
             "low down, high back pressure), open a big one (no choking up top). The ECU positions them for its boost\n"
             "target (Boost Control > ECU boost control). Typical on modern diesels.");
        if (p_.vgt) {
            SliderF("Vanes closed: flow area", &p_.vgtMinArea, 0.05f, 0.6f, "%.2f of open", "Turbine flow area with the vanes fully closed, as a share of the open area\n(the open area is the A/R above).");
            SliderF("Max boost target", &p_.wastegateBar, 0.2f, 2.5f, "%.2f bar", "Top of the generated boost target map.");
            ImGui::Text("Vanes open %.0f %%", tel_.vgtOpen * 100.0f);
        } else
        SliderF("Wastegate (boost)", &p_.wastegateBar, 0.2f, 2.5f, "%.2f bar", "Boost target: the wastegate opens and bypasses the turbine above this.");
        Check("Blow-off valve", &p_.bov);
        Help("Vents boost when you lift: 'pssshh'.\nWithout it the compressor surges against the closed throttle: 'stu-tu-tu' flutter -\n"
             "the air in the boost pipes pushes back through the wheel and refills, ~15-30 times a second\n"
             "(a smaller charge volume flutters faster).");
        ImGui::SameLine();
        Check("Anti-lag (ALS)", &p_.antiLag);
        Help("Rally anti-lag: on lift-off the ECU cracks the throttle and fires very late so the burn\ncontinues into the exhaust manifold - the turbo stays spooled, bangs and pops.");
        if (p_.bov) {
            Combo("Blow-off valve type", &p_.bovType, "Vent to atmosphere (loud)\0Recirculating / diverter (quiet)\0");
            SliderF("Opens at", &p_.bovSpringBar, 0.05f, 0.8f, "%.2f bar", "Pressure across the valve (charge over manifold) that cracks it open. A spring too\n"
                    "stiff for the boost leaves it fluttering half open; too soft and it leaks at part throttle.");
            ImGui::Text("Valve lift %.0f %%", tel_.bovLift * 100.0f);
        }
        {
            int nb = p_.compBlades;
            if (ImGui::SliderInt("Compressor blades", &nb, 6, 16)) p_.compBlades = nb;
            Help("Blades at the wheel's exit (full + splitter, e.g. 6 + 6). The blade-pass tone is blades x shaft speed -\n"
                 "above hearing at full speed, audible while spooling; the whistle you hear up top is the shaft order, the\n"
                 "buzz-saw of a near-supersonic tip and the tip-clearance noise.");
        }
    } else {
        ImGui::SeparatorText("Supercharger");
        int t = (int)p_.scType;
        if (ImGui::Combo("Type", &t, "Roots blower\0Twin-screw\0Centrifugal\0")) applyInductionDefaults(p_, Induction::Supercharger, (Supercharger)t);
        Help("Roots: instant boost, classic whine, inefficient (hot).\nTwin-screw: instant boost, more efficient.\nCentrifugal: boost rises with rpm like a belt-driven turbo.");
        if (p_.scType == Supercharger::Centrifugal) {
            turboModelUi();
            if (!p_.turbo.useMaps) SliderF("Compressor size", &p_.compFlow_gs, 60, 900, "%.0f g/s max flow");
            SliderF("Drive ratio", &p_.scRatio, 4, 16, "x%.1f", "Impeller speed / crank speed (pulley x internal step-up).");
        } else {
            SliderF("Blower size", &p_.scDisplacement_L, 0.3f, 6, "%.2f L/rev");
            SliderF("Pulley ratio", &p_.scRatio, 0.6f, 3.0f, "x%.2f", "Blower speed / crank speed. Smaller crank pulley = more boost.");
            Check("Bypass valve", &p_.scBypass);
            Help("Recirculates the blower at light throttle (no boost, less drag).");
        }
    }
    ImGui::SeparatorText("Charge air & tune");
    Check("Intercooler", &p_.intercooler);
    if (p_.intercooler) {
        ImGui::SameLine(); ImGui::SetNextItemWidth(160); ImGui::SliderFloat("efficiency", &p_.icEfficiency, 0.3f, 0.95f, "%.2f");
        ImGui::SameLine(); ImGui::TextDisabled("core %.0f C", tel_.icCoreC);
        Help("The core takes the compressor heat and gives it to the air blown through it (vehicle speed, or the\n"
             "dyno cell fan in Virtual Dyno). Parked or on a dyno without a fan it heat-soaks: charge temperature\n"
             "climbs over a long pull.");
    }
    SliderF("Charge pipe volume", &p_.chargeVolume_L, 0.5f, 15, "%.1f L", "More piping + intercooler volume = slower boost response.");
    SliderF("Boost retard", &p_.boostRetard, 0, 12, "%.1f deg/bar", "Pull ignition timing under boost to avoid knock.");
    SliderF("AFR at 1 bar", &p_.afrBoost, 10, 13, "%.1f", "ECU enrichment under boost (cooler, safer).");
    SliderF("Injector size", &p_.injector_ccmin, 60, 2000, "%.0f cc/min", "Boost needs much more fuel: watch the injector duty.");
    SliderF("Compression", &p_.compression, 6, 14, "%.1f : 1");
    ImGui::PopItemWidth();

    ImGui::Separator();
    ImGui::Text("Boost %+.2f bar   PR %.2f   %s %.0fk rpm   flow %.0f g/s   charge %.0f C   %.1f kW",
                tel_.boostBar, tel_.pr, p_.induction == Induction::Turbo ? "turbo" : "blower", tel_.shaftRpm / 1000,
                tel_.compFlow, tel_.chargeT, tel_.fiPowerKw);
    Lamp("BOV / bypass", tel_.bovOpen, rgba(0.4f, 0.8f, 1.0f)); ImGui::SameLine();
    Lamp("Surge", tel_.surging, rgba(1.0f, 0.6f, 0.2f)); ImGui::SameLine();
    Lamp("Anti-lag", tel_.antiLag, rgba(1.0f, 0.3f, 0.2f)); ImGui::SameLine();
    Lamp("Knock", tel_.knock > 1, rgba(1.0f, 0.4f, 0.1f));

    bool centrifugal = p_.induction == Induction::Turbo || p_.scType == Supercharger::Centrifugal;
    if (centrifugal && p_.turbo.useMaps) {
        turboMapPlot(std::max(220.0f, ImGui::GetContentRegionAvail().y));
    } else if (centrifugal) {
        // operating point trail
        mapTrailX_.push_back(tel_.compFlow); mapTrailY_.push_back(tel_.pr);
        if (mapTrailX_.size() > 400) { mapTrailX_.erase(mapTrailX_.begin()); mapTrailY_.erase(mapTrailY_.begin()); }
        if (ImPlot::BeginPlot("Compressor map", ImVec2(-1, std::max(220.0f, ImGui::GetContentRegionAvail().y)))) {
            ImPlot::SetupAxes("corrected flow (g/s)", "pressure ratio");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, p_.compFlow_gs * 1.1, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.8, 4.0, ImPlotCond_Always);
            static float sx[8][40], sy[8][40], surgeX[9], surgeY[9];
            for (int k = 0; k < 8; ++k) {
                double n = 0.3 + 0.1 * k;
                double pr0 = 1.0 + 2.6 * n * n, mch = p_.compFlow_gs * (0.12 + 0.88 * n);
                for (int j = 0; j < 40; ++j) {
                    double pr = 1.0 + (pr0 - 1.0) * j / 39.0;
                    sy[k][j] = (float)pr;
                    sx[k][j] = (float)(mch * std::sqrt(std::max(0.0, 1.0 - (pr - 1.0) / (pr0 - 1.0))));
                }
                char lb[24]; std::snprintf(lb, sizeof(lb), "##speed%d", k);
                ImPlot::SetNextLineStyle(ImVec4(0.55f, 0.55f, 0.6f, 0.8f), 1);
                ImPlot::PlotLine(lb, sx[k], sy[k], 40);
                char tx[16]; std::snprintf(tx, sizeof(tx), "%d%%", (int)(n * 100 + 0.5));
                ImPlot::PlotText(tx, sx[k][39] + p_.compFlow_gs * 0.03, sy[k][39] + 0.06);
                surgeX[k] = (float)(0.25 * mch); surgeY[k] = (float)(1.0 + (pr0 - 1.0) * (1.0 - 0.0625));
            }
            ImPlot::SetNextLineStyle(ImVec4(1, 0.35f, 0.2f, 1), 2);
            ImPlot::PlotLine("surge region", surgeX, surgeY, 8);
            ImPlot::SetNextLineStyle(ImVec4(0.95f, 0.8f, 0.3f, 0.6f), 1.5f);
            ImPlot::PlotLine("trail", mapTrailX_.data(), mapTrailY_.data(), (int)mapTrailX_.size());
            double ox[] = {tel_.compFlow}, oy[] = {tel_.pr};
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 7, tel_.surging ? ImVec4(1, 0.3f, 0.2f, 1) : ImVec4(1, 1, 1, 1));
            ImPlot::PlotScatter("operating point", ox, oy, 1);
            ImPlot::EndPlot();
        }
    }
}

// ======================================================================= vehicle
void App::panelVehicle() {
    ImGui::PushItemWidth(-170);
    Check("Driving mode [V]", &p_.vehicle);
    Help("The engine drives a car through the clutch and gearbox instead of the dyno.\nE/Q change gear, C = clutch, B/Down = brake, W/Space = gas.");
    {
        const ui::PadState& g = pad_.state();
        if (g.connected) {
            ImGui::Text("Controller: %s", g.name.c_str());
            ImGui::TextDisabled("gas %3.0f %%   clutch %3.0f %%   brake %3.0f %%", g.throttle * 100, g.clutch * 100, g.brake * 100);
        } else ImGui::TextDisabled("No game controller found (Xbox via XInput; DualShock 4 / DualSense directly or via Steam / DS4Windows)");
        Help("R2 / RT = gas, L2 / LT = clutch (both analog), R1 / RB = shift up, L1 / LB = shift down.\n"
             "Brake: Cross / A (full) or the right stick pulled back (analog). Triangle / Y = neutral.\n"
             "Options / Start = start the engine, Share / Back = ignition, Circle / B = driving mode,\n"
             "D-pad left / right = listener position. Works while this window has the focus.");
    }
    int tm = (int)p_.transMode;
    if (ImGui::Combo("Transmission", &tm, "Manual (you work the clutch)\0Manual, automatic clutch\0Automatic\0")) {
        p_.transMode = (TransMode)tm;
        p_.gear = 0;
    }
    Help("Manual: dump the clutch at idle and the engine stalls; shift without the clutch and it bangs.\n"
         "Automatic clutch: you pick the gear (E/Q), the clutch is handled for you.\n"
         "Automatic: E = Drive, Q = Neutral; shifts by itself (upshift later with more throttle, kickdown).");
    if (p_.transMode == TransMode::Automatic) {
        int sel = p_.gear > 0 ? 1 : 0;
        ImGui::RadioButton("N", &sel, 0); ImGui::SameLine(); ImGui::RadioButton("D", &sel, 1);
        p_.gear = sel;
    } else {
        int g = p_.gear;
        if (ImGui::SliderInt("Gear [E/Q]", &g, 0, p_.gears, g == 0 ? "N" : "%d")) p_.gear = g;
    }
    SliderF("Clutch pedal [C]", &p_.clutchPedal, 0, 1, "%.2f", "1 = pedal down (disengaged). Slip it gently to launch.");
    SliderF("Brake [B]", &p_.brake, 0, 1, "%.2f");

    ImGui::SeparatorText("Gearbox");
    if (SliderI("Gears", &p_.gears, 1, kMaxGears)) p_.gear = std::min(p_.gear, p_.gears);
    if (ImGui::BeginTable("ratios", 3, ImGuiTableFlags_SizingStretchProp)) {
        for (int i = 0; i < p_.gears; ++i) {
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%d", i + 1);
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::SliderFloat("##r", &p_.gearRatio[i], 0.4f, 5.0f, "%.3f");
            ImGui::TableNextColumn(); ImGui::Text("%3.0f km/h", gearSpeed(p_, i));
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    SliderF("Final drive", &p_.finalDrive, 1.5f, 7.0f, "%.2f", "Shorter (higher number) = quicker acceleration, lower top speed.");
    SliderF("Clutch capacity", &p_.clutchTorque, 80, 2500, "%.0f N m", "Less than the engine's torque = the clutch slips under full throttle.");
    SliderF("Driveline efficiency", &p_.drivelineEff, 0.7f, 0.98f, "%.2f");

    ImGui::SeparatorText("Car");
    SliderF("Mass", &p_.mass_kg, 150, 3000, "%.0f kg");
    SliderF("Drag (Cd x A)", &p_.cdA, 0.2f, 1.5f, "%.2f m2");
    SliderF("Rolling resistance", &p_.crr, 0.005f, 0.04f, "%.3f");
    SliderF("Tyre radius", &p_.tireRadius_mm, 200, 420, "%.0f mm");
    SliderF("Tyre grip", &p_.grip, 0.3f, 1.8f, "%.2f", "Road / tyre friction: 1.0 dry street, 0.5 wet, 1.5 slicks.");
    SliderF("Weight on driven wheels", &p_.driveFrac, 0.3f, 1.0f, "%.2f", "FWD ~0.6, RWD ~0.5 (more at launch), AWD 1.0.");
    SliderF("Road grade", &p_.grade_pct, -15, 25, "%.0f %%");
    if (ImGui::Button("Size car & gearing to this engine")) {
        EngineParams keep = p_;
        applyVehicleDefaults(p_);
        p_.vehicle = keep.vehicle; p_.transMode = keep.transMode; p_.gear = std::min(keep.gear, p_.gears);
    }
    ImGui::PopItemWidth();
    ImGui::Separator();
    ImGui::Text("%.0f km/h   gear %d   clutch %.0f %% %s   wheel slip %.0f %%", tel_.speedKmh, tel_.gear, tel_.clutchEng * 100,
                tel_.clutchLocked ? "locked" : "slip", std::fabs(tel_.wheelSlip) * 100);
    ImGui::Text("Top speed at the limiter in top gear: %.0f km/h (drag may stop it earlier)", vehicleTopSpeed(p_));
    ImGui::Text("0-100 km/h %s   1/4 mile %s", tel_.t100 > 0 ? (std::to_string(tel_.t100).substr(0, 4) + " s").c_str() : "--",
                tel_.tQuarter > 0 ? (std::to_string(tel_.tQuarter).substr(0, 5) + " s").c_str() : "--");
    ImGui::TextDisabled("Timers start when you pull away from a standstill.");
}

// ======================================================================= audio
void App::panelAudio() {
    ImGui::PushItemWidth(-170);
    int lp = (int)p_.listener;
    ImGui::RadioButton("Behind the car (tailpipe)", &lp, 0);
    ImGui::RadioButton("Engine bay (hood open)", &lp, 1);
    ImGui::RadioButton("Cabin (driver seat)", &lp, 2);
    p_.listener = (ListenerPos)lp;
    SliderF("Volume", &p_.volume, 0, 3, "%.2f");
    ImGui::SeparatorText("Exhaust tone");
    {
        const char* styles = "Smooth (engine-sim style)\0Raw (original pure physics)\0";
        ImGui::Combo("Sound style", &p_.soundStyle, styles);
        Help("Smooth: like engine-sim, the sound is mostly the exhaust pulse pressure itself, band-limited and\n"
             "then shaped by the impulse response below - full and smooth, no buzz.\n"
             "Raw: the original model - only the radiated sound of the tailpipe flow, no extra layers,\n"
             "no impulse response, perfectly even cylinders.");
    }
    if (p_.soundStyle == 1) ImGui::BeginDisabled();
    SliderF("Pulse pressure mix", &p_.pressureMix, 0, 1, "%.2f", "How much of the signal is the pulse pressure (full, round) versus the radiated\nsound of the flow (brighter, sharper). engine-sim uses almost pure pressure.");
    SliderF("Smoothing", &p_.smoothHz, 1000, 12000, "%.0f Hz", "Low-pass on the simulated exhaust signal. Lower = smoother and darker;\nthe impulse response adds the brightness back as a real muffler would.");
    SliderF("Rasp (air noise)", &p_.exhaustRasp, 0, 3, "%.2f", "Gas noise riding on each pulse: low-passed air noise on the pulse pressure plus\nthe jet noise of the tailpipe. Higher = breathier / raspier.");
    SliderF("Punch (pulse body)", &p_.exhaustPunch, 0, 1.5f, "%.2f", "Extra low-frequency pressure pulses (what a microphone close to the tailpipe hears).\nMore = thicker, thumpier.");
    SliderF("Wave steepening", &p_.exhaustSteep, 0, 1.5f, "%.2f", "Strong pressure pulses travel a bit faster than weak ones, so pulses sharpen on their\nway down the pipes. Adds bite but also roughness: 0 (off) by default.");
    if (p_.soundStyle == 1) ImGui::EndDisabled();
    ImGui::SeparatorText("Exhaust impulse response (convolution)");
    {
        int ch = p_.irChoice;
        if (ImGui::Combo("Response", &ch, irChoiceNames())) {
            if (ch == IrCustom && irFileName_.empty()) loadIrFile();
            else p_.irChoice = ch;
        }
        Help("The simulated exhaust is convolved with an impulse response: the dense echoes and ringing of a\n"
             "real muffler shell, pipes and tips that a clean pipe model can't produce on its own.\n"
             "Auto picks one that matches the muffler. You can also load any WAV impulse response\n"
             "(e.g. the MIT-licensed ones from the engine-sim project's ir_lib).");
        SliderF("Response mix", &p_.irMix, 0, 1, "%.2f", "0 = only the simulated pipes, 1 = fully through the impulse response.");
        if (ImGui::Button("Load WAV...")) loadIrFile();
        ImGui::SameLine();
        if (!irFileName_.empty()) ImGui::TextDisabled("%s", irFileName_.c_str());
        else ImGui::TextDisabled("no custom file loaded");
        if (!irError_.empty()) ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "WAV error: %s", irError_.c_str());
        const char* names[] = {"off", "auto", "raw pipe", "sport", "stock muffler", "race", "motorcycle", "custom WAV"};
        int act = sim_->activeIR();
        ImGui::TextDisabled("active: %s", act >= 0 && act <= 7 ? names[act] : "-");
    }
    ImGui::SeparatorText("Space");
    Combo("Environment", &p_.environment, "Open road\0Garage\0Dyno cell\0Tunnel\0");
    SliderF("Reverb", &p_.reverbMix, 0, 1, "%.2f", "How much of the space you hear: garage = hard and boomy, tunnel = long echo.");
    SliderI("Oversampling", &p_.oversample, 1, 4, "Physics rate = 48 kHz x N. Higher = more accurate high rpm + pipes, more CPU.");
    ImGui::PopItemWidth();
    ImGui::Separator();
    if (audio_.running()) ImGui::Text("Output: %s (48 kHz stereo)", audio_.deviceName().c_str());
    else {
        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "Audio off: %s", audio_.error().c_str());
        if (ImGui::Button("Retry audio")) audio_.start(sim_.get());
    }
    Meter("Sim CPU load", tel_.simLoad * 100, 0, 100, "%.0f %%", tel_.simLoad > 0.8f ? rgba(1, 0.3f, 0.2f) : rgba(0.4f, 0.8f, 0.4f));
    Meter("Audio buffer", tel_.ringFill * 1000, 0, 150, "%.0f ms", rgba(0.4f, 0.6f, 0.9f));
    Meter("Output peak", tel_.peak, 0, 1.5f, "%.2f", tel_.peak > 1 ? rgba(1, 0.3f, 0.2f) : rgba(0.4f, 0.8f, 0.4f));
    ImGui::Checkbox("ImPlot demo", &showDemo_);
}

// ======================================================================= dyno
void App::panelDyno() {
    if (p_.vehicle) ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "Driving mode is on: the car is the load (dyno absorber disabled). Press V to return.");
    int mode = (int)p_.dynoMode;
    ImGui::RadioButton("Free rev", &mode, 0); ImGui::SameLine();
    ImGui::RadioButton("Hold RPM", &mode, 1); ImGui::SameLine();
    ImGui::RadioButton("WOT sweep", &mode, 2); ImGui::SameLine();
    ImGui::RadioButton("WOT step test", &mode, 3);
    Help("Step test: holds every step at full throttle until rpm has settled, then averages it - steady-state\n"
         "points without the inertia / sweep-rate effects of a sweep (slower, closer to a brake dyno).");
    p_.dynoMode = (DynoMode)mode;
    const bool stepMode = p_.dynoMode == DynoMode::Step;
    ImGui::PushItemWidth(150);
    if (p_.dynoMode == DynoMode::Hold) {
        ImGui::SameLine();
        ImGui::SliderFloat("Hold at", &p_.dynoHoldRpm, 800, p_.revLimit, "%.0f rpm");
    }
    ImGui::SliderFloat("Start", &p_.sweepStartRpm, 800, 6000, "%.0f rpm"); ImGui::SameLine();
    ImGui::SliderFloat("End", &p_.sweepEndRpm, 2000, 16000, "%.0f rpm"); ImGui::SameLine();
    if (stepMode) {
        ImGui::SliderFloat("Step", &p_.stepRpm, 100, 1000, "%.0f rpm"); ImGui::SameLine();
        ImGui::SliderFloat("Average", &p_.stepHold_s, 0.5f, 5.0f, "%.1f s");
    } else ImGui::SliderFloat("Ramp", &p_.sweepRate, 100, 2000, "%.0f rpm/s");
    ImGui::SliderFloat("Cell fan", &p_.coolingFan_kmh, 0, 120, "%.0f km/h");
    Help("Air blown at the radiator and intercooler while on the dyno (not when driving).\n"
         "No fan: the coolant runs hotter and an intercooler heat-soaks over long pulls.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 13);
    ImGui::Combo("Correction", &dynoCorr_, dynoCorrectionNames());
    Help("Corrects every run to the standard's reference air from the air it was run in (temperature and\n"
         "pressure, see Altitude / Ambient temp): SAE J1349 25 C / 99 kPa, DIN 70020 20 C / 1013 mbar,\n"
         "ISO 1585 / EEC 25 C / 99 kPa, SAE J607 15.6 C / 29.92 inHg. Observed = as measured.");
    if (dynoCorr_ > 0) { ImGui::SameLine(); ImGui::TextDisabled("CF now %.3f", dynoCorrectionFactor((DynoCorrection)dynoCorr_, p_.ambientTemp_C, ambientPressurePa(p_) * 1e-3)); }
    ImGui::PopItemWidth();
    if (ImGui::Button(stepMode ? "RUN STEP TEST [D]" : "RUN SWEEP [D]")) {
        if (!stepMode) p_.dynoMode = DynoMode::Sweep;
        p_.sweepRequest++;
        if (!tel_.running) { autoStart_ = true; autoStartTimer_ = 0; }
    }
    ImGui::SameLine();
    const char* st[] = {"idle", "holding", "settling at start rpm", "sweeping...", "done", "stepping..."};
    ImGui::Text("Absorber: %s  load %.0f Nm", st[clampv(tel_.dynoState, 0, 5)], tel_.dynoLoad);
    if (tel_.dynoState == 3 || tel_.dynoState == 5) { ImGui::SameLine(); ImGui::ProgressBar(tel_.dynoProgress, ImVec2(-1, 0)); }

    bool& kw = dynoKw_;
    ImGui::SameLine();
    ImGui::Checkbox("kW", &kw);
    ImGui::SameLine();
    ImGui::Checkbox("lb-ft", &imperial_);
    Help("Imperial torque (lb-ft). Power in hp is then lb-ft x rpm / 5252.");
    ImGui::SameLine();
    if (ImGui::SmallButton("Export CSV...")) dynoExportCsv();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    {
        std::string items = "no baseline";
        items.push_back('\0');
        for (size_t i = 0; i < runs_.size(); ++i) { items += "Run " + std::to_string(i + 1); items.push_back('\0'); }
        int b = dynoBase_ + 1;
        if (ImGui::Combo("##base", &b, items.c_str())) dynoBase_ = b - 1;
        if (dynoBase_ >= (int)runs_.size()) dynoBase_ = -1;
        Help("Before / after: pick the baseline run - the table shows every run's peaks against it and a\n"
             "difference plot (torque and power delta over rpm) appears under the curves.");
    }
    ImGui::Checkbox("Live trace", &showLive_); Help("Torque / power of the last 4 seconds against rpm: rev the engine and watch the curve being drawn.");
    ImGui::SameLine(); ImGui::Checkbox("Peak hold", &showPeak_); Help("Highest torque and power seen at every rpm since the last reset.\nOne full-throttle rev from idle to the limiter draws the whole curve.");
    ImGui::SameLine();
    if (ImGui::SmallButton("Reset peak")) resetPeak();
    ImGui::SameLine();
    if (ImGui::SmallButton("Save peak as run")) {
        DynoRun r;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "#%d %s | peak hold (free rev%s)", (int)runs_.size() + 1, p_.name, p_.vehicle ? " / driving" : "");
        r.label = buf;
        r.engine = p_.name;
        r.calName = calMeta_.name.empty() ? "(unsaved)" : calMeta_.name;
        r.calRev = calMeta_.revision;
        r.label += " | cal " + r.calName + " r" + std::to_string(r.calRev);
        r.ambientC = p_.ambientTemp_C; r.baroKPa = (float)(ambientPressurePa(p_) * 1e-3);
        static const float pal[][3] = {{0.95f, 0.85f, 0.3f}, {0.3f, 0.9f, 0.85f}, {0.9f, 0.3f, 0.7f}, {0.6f, 0.5f, 1.0f}};
        const float* col = pal[runs_.size() % 4];
        r.color[0] = col[0]; r.color[1] = col[1]; r.color[2] = col[2]; r.color[3] = 1;
        for (int b = 0; b < kPeakBins; ++b) {
            if (peakTq_[b] <= 0) continue;
            DynoPoint d; d.rpm = b * 50.0f + 25.0f; d.torque = peakTq_[b]; d.powerKw = peakPw_[b]; d.afr = 0; d.ve = 0;
            r.pts.push_back(d);
            float hp = d.powerKw * 1.341f;
            if (d.torque > r.peakTorque) { r.peakTorque = d.torque; r.peakTorqueRpm = d.rpm; }
            if (hp > r.peakPower) { r.peakPower = hp; r.peakPowerRpm = d.rpm; }
        }
        if (!r.pts.empty()) runs_.push_back(r);
    }

    // fixed, readable axes: rpm 0..limit, torque / power from what has been seen so far
    float tqTop = 50, pwTop = 30;
    for (int b = 0; b < kPeakBins; ++b) { tqTop = std::max(tqTop, peakTq_[b]); pwTop = std::max(pwTop, peakPw_[b]); }
    const DynoCorrection corr = (DynoCorrection)dynoCorr_;
    auto cfOf = [corr](const DynoRun& r) { return (float)dynoCorrectionFactor(corr, r.ambientC, r.baroKPa); };
    for (auto& r : runs_) if (r.visible) { const float cf = cfOf(r); for (auto& d : r.pts) { tqTop = std::max(tqTop, d.torque * cf); pwTop = std::max(pwTop, d.powerKw * cf); } }
    tqTop = std::max(tqTop, tel_.torque);
    pwTop = std::max(pwTop, tel_.powerKw);
    const float pwScale = kw ? 1.0f : 1.341f;
    const float tqScale = imperial_ ? 0.737562f : 1.0f;      // N m -> lb-ft
    const char* tqUnit = imperial_ ? "lb-ft" : "N m";
    tqTop *= tqScale;
    const bool delta = dynoBase_ >= 0 && dynoBase_ < (int)runs_.size() && runs_.size() > 1;
    const float plotH = std::max(200.0f, ImGui::GetContentRegionAvail().y - 120 - (delta ? 170.0f : 0.0f));
    if (ImPlot::BeginPlot("##dyno", ImVec2(-1, plotH))) {
        ImPlot::SetupAxes("rpm", imperial_ ? "torque (lb-ft)" : "torque (N m)", 0, 0);
        ImPlot::SetupAxis(ImAxis_Y2, kw ? "power (kW)" : "power (hp)", ImPlotAxisFlags_AuxDefault);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, std::ceil((p_.revLimit + 700) / 500.0) * 500.0, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, std::ceil(tqTop * 1.15f / 50.0f) * 50.0f, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y2, 0, std::ceil(pwTop * pwScale * 1.15f / 25.0f) * 25.0f, ImPlotCond_Always);
        for (size_t i = 0; i < runs_.size(); ++i) {
            DynoRun& r = runs_[i];
            if (!r.visible || r.pts.empty()) continue;
            std::vector<float> x, t, pw;
            const float cf = cfOf(r);
            for (auto& d : r.pts) { x.push_back(d.rpm); t.push_back(d.torque * tqScale * cf); pw.push_back(d.powerKw * pwScale * cf); }
            ImVec4 cc(r.color[0], r.color[1], r.color[2], 1);
            char id[32];
            std::snprintf(id, sizeof(id), "Run %zu torque", i + 1);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
            ImPlot::SetNextLineStyle(cc, 2.5f);
            ImPlot::PlotLine(id, x.data(), t.data(), (int)x.size());
            std::snprintf(id, sizeof(id), "Run %zu power", i + 1);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            ImPlot::SetNextLineStyle(ImVec4(cc.x * 0.8f, cc.y * 0.8f, cc.z * 0.8f, 0.9f), 1.5f);
            ImPlot::PlotLine(id, x.data(), pw.data(), (int)x.size());
        }
        if (showPeak_) {
            // fill short gaps (fast revs skip bins) by interpolation, then smooth lightly
            static float px[kPeakBins], pt[kPeakBins], pp[kPeakBins];
            static float ft[kPeakBins], fp[kPeakBins];
            int first = -1, last = -1;
            for (int b = 0; b < kPeakBins; ++b) if (peakTq_[b] > 0) { if (first < 0) first = b; last = b; }
            int n = 0;
            if (first >= 0) {
                int prev = first;
                for (int b = first; b <= last; ++b) {
                    if (peakTq_[b] > 0) { ft[b] = peakTq_[b]; fp[b] = peakPw_[b]; prev = b; continue; }
                    int next = b; while (next <= last && peakTq_[next] <= 0) ++next;
                    float u = (float)(b - prev) / (float)(next - prev);
                    ft[b] = peakTq_[prev] + (peakTq_[next] - peakTq_[prev]) * u;
                    fp[b] = peakPw_[prev] + (peakPw_[next] - peakPw_[prev]) * u;
                }
                for (int b = first; b <= last; ++b) {
                    int a0 = std::max(first, b - 2), a1 = std::min(last, b + 2);
                    float st = 0, sp = 0; int cnt = 0;
                    for (int k = a0; k <= a1; ++k) { st += ft[k]; sp += fp[k]; ++cnt; }
                    px[n] = b * 50.0f + 25.0f; pt[n] = st / cnt * tqScale; pp[n] = sp / cnt * pwScale; ++n;
                }
            }
            if (n > 1) {
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
                ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.55f, 0.15f, 0.95f), 2.0f);
                ImPlot::PlotLine("peak torque", px, pt, n);
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
                ImPlot::SetNextLineStyle(ImVec4(0.35f, 0.75f, 1.0f, 0.95f), 2.0f);
                ImPlot::PlotLine("peak power", px, pp, n);
            }
        }
        if (showLive_ && liveRpm_.size() > 1) {
            // fading trail: draw it in a few segments, newer = brighter
            const int n = (int)liveRpm_.size();
            static std::vector<float> lp, lt;
            lp.resize(n); lt.resize(n);
            for (int i = 0; i < n; ++i) { lp[i] = livePw_[i] * pwScale; lt[i] = liveTq_[i] * tqScale; }
            const int seg = 4;
            for (int s = 0; s < seg; ++s) {
                int a = n * s / seg, b = std::min(n, n * (s + 1) / seg + 1);
                if (b - a < 2) continue;
                float al = 0.25f + 0.75f * (s + 1) / seg;
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
                ImPlot::SetNextLineStyle(ImVec4(1, 1, 1, al), 1.5f);
                ImPlot::PlotLine("##live torque", liveRpm_.data() + a, lt.data() + a, b - a);
                ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
                ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.9f, 1.0f, al), 1.5f);
                ImPlot::PlotLine("##live power", liveRpm_.data() + a, lp.data() + a, b - a);
            }
        }
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
        double rx[] = {tel_.rpm}, ry[] = {tel_.torque * tqScale};
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 6, ImVec4(1, 1, 1, 1));
        ImPlot::PlotScatter("now torque", rx, ry, 1);
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
        double py[] = {tel_.powerKw * pwScale};
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 6, ImVec4(0.6f, 0.9f, 1.0f, 1));
        ImPlot::PlotScatter("now power", rx, py, 1);
        // live readout in the corner
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
        char txt[96];
        std::snprintf(txt, sizeof(txt), "%.0f rpm   %.0f %s   %.0f %s", tel_.rpm, tel_.torque * tqScale, tqUnit, tel_.powerKw * pwScale, kw ? "kW" : "hp");
        ImPlot::Annotation(ImPlot::GetPlotLimits().X.Max, ImPlot::GetPlotLimits().Y.Max, ImVec4(0.1f, 0.1f, 0.12f, 0.85f), ImVec2(-8, 8), true, "%s", txt);
        ImPlot::EndPlot();
    }
    // before / after: every visible run minus the baseline, over the rpm they share
    if (delta && ImPlot::BeginPlot("##dyno_delta", ImVec2(-1, 160))) {
        const DynoRun& base = runs_[dynoBase_];
        const float cfB = cfOf(base);
        ImPlot::SetupAxes("rpm", imperial_ ? "delta torque (lb-ft)" : "delta torque (N m)", 0, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y2, kw ? "delta power (kW)" : "delta power (hp)", ImPlotAxisFlags_AuxDefault | ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, std::ceil((p_.revLimit + 700) / 500.0) * 500.0, ImPlotCond_Always);
        // a curve is "steep" where torque changes by more than 8 % of its peak per 100 rpm (sweep start, limiter
        // cliff): there a few rpm of misalignment between runs reads as a huge, meaningless difference
        auto steep = [](const DynoRun& run, size_t k) {
            const DynoPoint &a = run.pts[k - 1], &b = run.pts[k];
            return b.rpm > a.rpm && std::fabs(b.torque - a.torque) / (b.rpm - a.rpm) * 100.0f > 0.08f * run.peakTorque;
        };
        auto baseAt = [&](float rpm, bool power, bool& ok) {
            ok = false;
            for (size_t k = 1; k < base.pts.size(); ++k)
                if (rpm >= base.pts[k - 1].rpm && rpm <= base.pts[k].rpm) {
                    const DynoPoint &a = base.pts[k - 1], &b = base.pts[k];
                    const float u = b.rpm > a.rpm ? (rpm - a.rpm) / (b.rpm - a.rpm) : 0.0f;
                    ok = !steep(base, k);
                    return cfB * (power ? a.powerKw + (b.powerKw - a.powerKw) * u : a.torque + (b.torque - a.torque) * u);
                }
            return 0.0f;
        };
        double zero[2] = {0, 0};
        ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.6f, 0.6f, 0.6f));
        ImPlot::PlotInfLines("##zero", zero, 1, ImPlotInfLinesFlags_Horizontal);
        for (size_t i = 0; i < runs_.size(); ++i) {
            const DynoRun& r = runs_[i];
            if ((int)i == dynoBase_ || !r.visible) continue;
            const float cf = cfOf(r);
            std::vector<float> x, dt, dp;
            for (size_t k = 0; k < r.pts.size(); ++k) {
                const DynoPoint& d = r.pts[k];
                bool ok1, ok2;
                const float bt = baseAt(d.rpm, false, ok1), bp = baseAt(d.rpm, true, ok2);
                if (!ok1 || !ok2) continue;
                if ((k > 0 && steep(r, k)) || (k + 1 < r.pts.size() && steep(r, k + 1))) continue;
                x.push_back(d.rpm); dt.push_back((d.torque * cf - bt) * tqScale); dp.push_back((d.powerKw * cf - bp) * pwScale);
            }
            if (x.size() < 2) continue;
            const ImVec4 cc(r.color[0], r.color[1], r.color[2], 1);
            char id[40];
            std::snprintf(id, sizeof(id), "Run %zu - Run %d torque", i + 1, dynoBase_ + 1);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
            ImPlot::SetNextLineStyle(cc, 2.0f);
            ImPlot::PlotLine(id, x.data(), dt.data(), (int)x.size());
            std::snprintf(id, sizeof(id), "Run %zu - Run %d power", i + 1, dynoBase_ + 1);
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            ImPlot::SetNextLineStyle(ImVec4(cc.x * 0.8f, cc.y * 0.8f, cc.z * 0.8f, 0.8f), 1.2f);
            ImPlot::PlotLine(id, x.data(), dp.data(), (int)x.size());
        }
        ImPlot::EndPlot();
    }
    if (ImGui::BeginTable("runs", 4, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
                          ImVec2(-1, std::max(60.0f, ImGui::GetContentRegionAvail().y - 4)))) {
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 50);
        ImGui::TableSetupColumn("Run");
        ImGui::TableSetupColumn("Peaks", ImGuiTableColumnFlags_WidthFixed, 300);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 24);
        int del = -1;
        for (size_t i = 0; i < runs_.size(); ++i) {
            DynoRun& r = runs_[i];
            ImGui::PushID((int)i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Checkbox("##v", &r.visible); ImGui::SameLine(0, 2);
            ImGui::ColorEdit3("##c", r.color, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", r.label.c_str());   // long labels (engine | cal name rev) wrap instead of being cut
            ImGui::TableNextColumn();
            const float cfR = cfOf(r);
            ImGui::Text("%.0f %s @%.0f  %.0f hp @%.0f%s", r.peakTorque * tqScale * cfR, tqUnit, r.peakTorqueRpm, r.peakPower * cfR, r.peakPowerRpm,
                        corr != DynoCorrection::None ? " (corr.)" : "");
            if (delta && (int)i != dynoBase_) {
                const DynoRun& b = runs_[dynoBase_];
                const float cfB2 = cfOf(b);
                const float dT = (r.peakTorque * cfR - b.peakTorque * cfB2) * tqScale, dP = r.peakPower * cfR - b.peakPower * cfB2;
                ImGui::TextColored(dP >= 0 ? ImVec4(0.45f, 0.9f, 0.45f, 1) : ImVec4(1, 0.45f, 0.4f, 1), "vs Run %d: %+.1f %s  %+.1f hp (%+.1f %%)",
                                   dynoBase_ + 1, dT, tqUnit, dP, b.peakPower > 0 ? 100.0f * dP / (b.peakPower * cfB2) : 0.0f);
            } else if ((int)i == dynoBase_) ImGui::TextDisabled("baseline");
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("x")) del = (int)i;
            ImGui::PopID();
        }
        if (del >= 0) {
            runs_.erase(runs_.begin() + del);
            if (dynoBase_ == del) dynoBase_ = -1; else if (dynoBase_ > del) --dynoBase_;
        }
        ImGui::EndTable();
    }
}

// ======================================================================= scope
static void fft(std::vector<std::complex<float>>& a) {
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)kPi / (float)len;
        std::complex<float> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<float> w(1);
            for (size_t j = 0; j < len / 2; ++j) {
                auto u = a[i + j], v = a[i + j + len / 2] * w;
                a[i + j] = u + v; a[i + j + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

void App::panelScope() {
    if (!ImGui::BeginTabBar("scopetabs")) return;
    static float xs[Telemetry::kTrace];
    for (int i = 0; i < Telemetry::kTrace; ++i) xs[i] = (float)i;
    const int TL = clampv(tel_.traceLen, 1, (int)Telemetry::kTrace);
    if (ImGui::BeginTabItem("Pressure")) {
        static bool logY = false;
        ImGui::Checkbox("log scale", &logY);
        ImGui::SameLine();
        ImGui::TextDisabled("cylinder 1, intake & exhaust ports, last cycle");
        if (ImPlot::BeginPlot("##ptheta", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("crank angle (0 = firing TDC)", "bar abs");
            if (logY) ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, TL, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, logY ? 0.05 : 0, 120, ImPlotCond_Once);
            ImPlot::SetNextLineStyle(ImVec4(1, 0.8f, 0.3f, 1), 2);
            ImPlot::PlotLine(isRotary(p_) ? "chamber" : "cylinder", xs, tel_.traceP, TL);
            ImPlot::SetNextLineStyle(ImVec4(1, 0.4f, 0.2f, 1), 1.5f);
            ImPlot::PlotLine("exhaust port", xs, tel_.traceExh, TL);
            ImPlot::SetNextLineStyle(ImVec4(0.3f, 0.6f, 1, 1), 1.5f);
            ImPlot::PlotLine("intake port", xs, tel_.traceInt, TL);
            ImPlot::EndPlot();
        }
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("P-V")) {
        if (ImPlot::BeginPlot("##pv", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("volume (cc)", "pressure (bar)");
            ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
            ImPlot::SetupAxisScale(ImAxis_Y1, ImPlotScale_Log10);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.05, 150, ImPlotCond_Once);
            ImPlot::SetupAxisLimits(ImAxis_X1, 20, 2000, ImPlotCond_Once);
            ImPlot::SetNextLineStyle(ImVec4(1, 0.8f, 0.3f, 1), 2);
            ImPlot::PlotLine("cycle", tel_.traceV, tel_.traceP, TL);
            ImPlot::EndPlot();
        }
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Boost")) { panelBoostGraph(); ImGui::EndTabItem(); }
    if (ImGui::BeginTabItem("Fuel map")) { panelFuelMap(); ImGui::EndTabItem(); }
    if (ImGui::BeginTabItem("Cylinders")) {
        int n = tel_.cylinders;
        static float idx[kMaxCyl];
        for (int i = 0; i < kMaxCyl; ++i) idx[i] = (float)(i + 1);
        if (ImPlot::BeginPlot("##cyls", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("cylinder", "", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotBars("IMEP (bar)", idx, tel_.cylImep, n, 0.25, -0.15);
            ImPlot::PlotBars("Pmax / 10 (bar)", idx, [&] { static float t[kMaxCyl]; for (int i = 0; i < n; ++i) t[i] = tel_.cylPmax[i] / 10; return t; }(), n, 0.25, 0.15);
            ImPlot::EndPlot();
        }
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Sound")) {
        const int N = Telemetry::kScope;
        static float wave[Telemetry::kScope], t[Telemetry::kScope];
        for (int i = 0; i < N; ++i) { wave[i] = tel_.scope[(tel_.scopePos + i) % N]; t[i] = i * 1000.0f / Simulator::kOutRate; }
        float h = ImGui::GetContentRegionAvail().y;
        if (ImPlot::BeginPlot("##wave", ImVec2(-1, h * 0.4f))) {
            ImPlot::SetupAxes("ms", "", 0, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, N * 1000.0 / Simulator::kOutRate, ImPlotCond_Always);
            ImPlot::PlotLine("output", t, wave, N);
            ImPlot::EndPlot();
        }
        std::vector<std::complex<float>> a(N);
        for (int i = 0; i < N; ++i) a[i] = wave[i] * (0.5f - 0.5f * std::cos(2 * (float)kPi * i / (N - 1)));
        fft(a);
        static float fx[Telemetry::kScope / 2], fy[Telemetry::kScope / 2];
        if (fftMag_.size() != (size_t)N / 2) fftMag_.assign(N / 2, -120);
        for (int i = 0; i < N / 2; ++i) {
            fx[i] = (float)i * Simulator::kOutRate / N;
            float db = 20 * std::log10(std::abs(a[i]) / (N / 4) + 1e-7f);
            fftMag_[i] = std::max(db, fftMag_[i] - 1.5f);  // peak hold with decay
            fy[i] = fftMag_[i];
        }
        float fire = tel_.rpm / (60.0f * (float)revsPerFiring(p_)) * p_.cylinders;
        if (ImPlot::BeginPlot("##fft", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Hz", "dB");
            ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
            ImPlot::SetupAxisLimits(ImAxis_X1, 15, 20000, ImPlotCond_Once);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -110, 0, ImPlotCond_Once);
            ImPlot::PlotLine("spectrum", fx + 1, fy + 1, N / 2 - 1);
            if (fire > 1) { double f[] = {fire}; ImPlot::PlotInfLines("firing freq", f, 1); }
            ImPlot::EndPlot();
        }
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
}

// ======================================================================= scope recording
void App::recordScope(float dt) {
    // history at ~60 Hz, last 20 s
    histAccum_ += dt;
    hist_.now += dt;
    if (histAccum_ >= 1.0f / 60.0f) {
        histAccum_ = 0;
        auto push = [](std::vector<float>& v, float x) { v.push_back(x); };
        push(hist_.t, hist_.now);
        push(hist_.boost, tel_.boostBar != 0 || isBoosted(p_) ? tel_.boostBar : (tel_.mapKPa - tel_.baroKPa) / 100.0f);
        if (!isBoosted(p_)) hist_.boost.back() = (tel_.mapKPa - tel_.baroKPa) / 100.0f;
        push(hist_.target, p_.induction == Induction::Turbo ? p_.wastegateBar : 0.0f);
        push(hist_.shaft, tel_.shaftRpm / 1000.0f);
        push(hist_.wg, tel_.wastegate * 100.0f);
        push(hist_.thr, tel_.throttle * 100.0f);
        push(hist_.afr, tel_.afr);
        // live dyno trace (last 4 s) and peak hold while the engine makes power
        if (tel_.running && tel_.rpm > 300) {
            liveRpm_.push_back(tel_.rpm); liveTq_.push_back(tel_.torque); livePw_.push_back(tel_.powerKw); liveAge_.push_back(hist_.now);
            while (!liveAge_.empty() && hist_.now - liveAge_.front() > 4.0f) {
                liveRpm_.erase(liveRpm_.begin()); liveTq_.erase(liveTq_.begin());
                livePw_.erase(livePw_.begin()); liveAge_.erase(liveAge_.begin());
            }
            int b = clampv((int)(tel_.rpm / 50.0f), 0, kPeakBins - 1);
            if (tel_.torque > peakTq_[b]) peakTq_[b] = tel_.torque;
            if (tel_.powerKw > peakPw_[b]) peakPw_[b] = tel_.powerKw;
        }
        while (!hist_.t.empty() && hist_.t.back() - hist_.t.front() > 20.0f) {
            for (auto* v : {&hist_.t, &hist_.boost, &hist_.target, &hist_.shaft, &hist_.wg, &hist_.thr, &hist_.afr}) v->erase(v->begin());
        }
    }
    // measured maps: only while the engine runs and the readings have settled
    if (tel_.running && tel_.rpm > 400 && tel_.afr > 7 && tel_.afr < 22) {
        int r = clampv((int)(tel_.rpm / 500.0f + 0.5f), 0, kMapR - 1);
        int m = clampv((int)(tel_.mapKPa / 10.0f + 0.5f), 0, kMapM - 1);
        int i = r * kMapM + m;
        float a = hitMap_[i] ? 0.08f : 1.0f;
        afrMap_[i] += (tel_.afr - afrMap_[i]) * a;
        dutyMap_[i] += (tel_.injDuty * 100.0f - dutyMap_[i]) * a;
        advMap_[i] += (tel_.advance - advMap_[i]) * a;
        hitMap_[i] = 1;
        opTrailX_.push_back(tel_.rpm); opTrailY_.push_back(tel_.mapKPa);
        if (opTrailX_.size() > 240) { opTrailX_.erase(opTrailX_.begin()); opTrailY_.erase(opTrailY_.begin()); }
    }
}

// ======================================================================= boost graph
void App::panelBoostGraph() {
    const bool boosted = isBoosted(p_);
    ImGui::TextDisabled(boosted ? "boost / manifold pressure, last 20 s" : "manifold pressure relative to atmosphere (vacuum < 0), last 20 s");
    if (hist_.t.size() < 2) return;
    const int n = (int)hist_.t.size();
    float t1 = hist_.t.back();
    float h = ImGui::GetContentRegionAvail().y;
    if (ImPlot::BeginPlot("##boostgraph", ImVec2(-1, h * 0.62f))) {
        ImPlot::SetupAxes(nullptr, "bar", ImPlotAxisFlags_NoTickLabels, 0);
        ImPlot::SetupAxisLimits(ImAxis_X1, t1 - 20, t1, ImPlotCond_Always);
        float top = boosted ? std::max(1.0f, (p_.induction == Induction::Turbo ? p_.wastegateBar : 1.0f) + 0.6f) : 0.3f;
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1.0, top, ImPlotCond_Always);
        if (boosted) {
            ImPlot::SetupAxis(ImAxis_Y2, p_.induction == Induction::Turbo ? "turbo k rpm" : "blower k rpm", ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_Y2, 0, p_.induction == Induction::Turbo ? 250 : (p_.scType == Supercharger::Centrifugal ? 80 : 20), ImPlotCond_Always);
        }
        double zero[] = {0};
        ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.6f, 0.6f, 0.5f), 1);
        ImPlot::PlotInfLines("##atm", zero, 1, ImPlotInfLinesFlags_Horizontal);
        ImPlot::SetNextFillStyle(ImVec4(0.95f, 0.35f, 0.75f, 1), 0.25f);
        ImPlot::PlotShaded("boost", hist_.t.data(), hist_.boost.data(), n, 0.0);
        ImPlot::SetNextLineStyle(ImVec4(0.95f, 0.35f, 0.75f, 1), 2.5f);
        ImPlot::PlotLine("boost", hist_.t.data(), hist_.boost.data(), n);
        if (p_.induction == Induction::Turbo) {
            ImPlot::SetNextLineStyle(ImVec4(1, 1, 1, 0.5f), 1);
            ImPlot::PlotLine("wastegate target", hist_.t.data(), hist_.target.data(), n);
        }
        if (boosted) {
            ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
            ImPlot::SetNextLineStyle(ImVec4(0.45f, 0.75f, 1.0f, 1), 1.5f);
            ImPlot::PlotLine(p_.induction == Induction::Turbo ? "turbo speed" : "blower speed", hist_.t.data(), hist_.shaft.data(), n);
        }
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##thrwg", ImVec2(-1, -1))) {
        ImPlot::SetupAxes("s", "%", 0, 0);
        ImPlot::SetupAxisLimits(ImAxis_X1, t1 - 20, t1, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 105, ImPlotCond_Always);
        ImPlot::SetupAxis(ImAxis_Y2, "AFR", ImPlotAxisFlags_AuxDefault);
        ImPlot::SetupAxisLimits(ImAxis_Y2, 9, 18, ImPlotCond_Always);
        ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.9f, 0.4f, 1), 1.5f);
        ImPlot::PlotLine("throttle", hist_.t.data(), hist_.thr.data(), n);
        if (p_.induction == Induction::Turbo) {
            ImPlot::SetNextLineStyle(ImVec4(0.8f, 0.8f, 0.8f, 1), 1.5f);
            ImPlot::PlotLine("wastegate", hist_.t.data(), hist_.wg.data(), n);
        }
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
        ImPlot::SetNextLineStyle(ImVec4(1, 0.65f, 0.2f, 1), 1.5f);
        ImPlot::PlotLine("AFR", hist_.t.data(), hist_.afr.data(), n);
        ImPlot::EndPlot();
    }
}

// ======================================================================= fuel map
void App::panelFuelMap() {
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    const char* views[] = {"VE table (ECU)", "Target AFR (ECU)", "Measured AFR", "Injector duty", "Ignition advance"};
    ImGui::SetNextItemWidth(170);
    ImGui::Combo("##fmview", &fuelMapView_, views, 5);
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear measured")) {
        std::fill(std::begin(afrMap_), std::end(afrMap_), 0.0f); std::fill(std::begin(dutyMap_), std::end(dutyMap_), 0.0f);
        std::fill(std::begin(advMap_), std::end(advMap_), 0.0f); std::fill(std::begin(hitMap_), std::end(hitMap_), (unsigned char)0);
        opTrailX_.clear(); opTrailY_.clear();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%.0f rpm  %.0f kPa  AFR %.1f", tel_.rpm, tel_.mapKPa, tel_.afr);

    // rows = MAP (top = high), cols = rpm
    const int R = kMapR, M = kMapM;
    const int mMax = isBoosted(p_) ? M : 12;          // NA engines never go above ~110 kPa
    static float grid[kMapR * kMapM];
    float lo = 0, hi = 1;
    const char* unit = "";
    bool usesHits = fuelMapView_ >= 2;
    for (int m = 0; m < mMax; ++m) {
        for (int r = 0; r < R; ++r) {
            int row = mMax - 1 - m;                    // flip so high MAP is at the top
            float v = 0;
            int i = r * kMapM + m;
            switch (fuelMapView_) {
            case 0: v = (float)tableLookup(p_.cal.veMap, r * 500.0, m * 10.0); break;
            case 1: {
                double map = m * 10.0, load = map / 101.3;
                double wot = smoothstep(0.80, 0.97, load);
                double afr = lerp((double)p_.afrCruise, (double)p_.afrWot, wot);
                if (isBoosted(p_)) afr = lerp(afr, (double)p_.afrBoost, clampv((map - 101.3) / 100.0, 0.0, 1.0));
                v = (float)afr; break;
            }
            case 2: v = hitMap_[i] ? afrMap_[i] : 0; break;
            case 3: v = hitMap_[i] ? dutyMap_[i] : 0; break;
            case 4: v = hitMap_[i] ? advMap_[i] : 0; break;
            }
            grid[row * R + r] = v;
        }
    }
    switch (fuelMapView_) {
    case 0: lo = 20; hi = 120; unit = "VE %"; break;
    case 1: lo = 10.5f; hi = 16; unit = "AFR"; break;
    case 2: lo = 10.5f; hi = 16; unit = "AFR"; break;
    case 3: lo = 0; hi = 100; unit = "duty %"; break;
    case 4: lo = 0; hi = 45; unit = "deg BTDC"; break;
    }
    if (!efi && (fuelMapView_ == 0 || fuelMapView_ == 1 || fuelMapView_ == 3)) {
        ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "Carburetor: there is no ECU fuel table. Use Measured AFR / Ignition advance.");
        return;
    }
    ImPlot::PushColormap(fuelMapView_ == 2 || fuelMapView_ == 1 ? ImPlotColormap_Spectral : ImPlotColormap_Jet);
    float w = ImGui::GetContentRegionAvail().x;
    if (ImPlot::BeginPlot("##fuelmap", ImVec2(w - 90, -1))) {
        ImPlot::SetupAxes("rpm", "MAP kPa", 0, 0);
        ImPlot::SetupAxisLimits(ImAxis_X1, -250, (R - 0.5) * 500, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -5, (mMax - 0.5) * 10, ImPlotCond_Always);
        ImPlot::PlotHeatmap("##hm", grid, mMax, R, lo, hi, nullptr, ImPlotPoint(-250, -5), ImPlotPoint((R - 0.5) * 500, (mMax - 0.5) * 10));
        if (!opTrailX_.empty()) {
            ImPlot::SetNextLineStyle(ImVec4(1, 1, 1, 0.55f), 1.5f);
            ImPlot::PlotLine("##trail", opTrailX_.data(), opTrailY_.data(), (int)opTrailX_.size());
        }
        double ox[] = {tel_.rpm}, oy[] = {tel_.mapKPa};
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 7, ImVec4(1, 1, 1, 1), 2, ImVec4(0, 0, 0, 1));
        ImPlot::PlotScatter("##op", ox, oy, 1);
        double atm[] = {101.3};
        ImPlot::SetNextLineStyle(ImVec4(1, 1, 1, 0.35f), 1);
        ImPlot::PlotInfLines("##atm", atm, 1, ImPlotInfLinesFlags_Horizontal);
        ImPlot::EndPlot();
    }
    ImGui::SameLine();
    ImPlot::ColormapScale(unit, lo, hi, ImVec2(80, -1));
    ImPlot::PopColormap();
    if (usesHits) ImGui::TextDisabled("cells fill in as you drive / rev through them");
}

// ======================================================================= engine view
void App::panelEngineView() {
    ImGui::SetNextItemWidth(200);
    ImGui::SliderFloat("Slow motion", &animSpeed_, 0.002f, 0.2f, "x%.3f", ImGuiSliderFlags_Logarithmic);
    Help("Above ~120 rpm the animation runs in slow motion (the real engine is far too fast to see).");
    ImGui::SameLine();
    if (isRotary(p_))
        ImGui::Text("e-shaft %.0f deg   rotary, %d rotor%s, %.2f L", animAngle_, p_.cylinders, p_.cylinders > 1 ? "s" : "", displacementL(p_));
    else
        ImGui::Text("crank %.0f deg   %s, %d cyl, %.2f L", animAngle_,
                    p_.layout == Layout::Inline ? "inline" : p_.layout == Layout::V ? "V" : "boxer", p_.cylinders, displacementL(p_));
    // camera presets and the cutaway thickness
    if (ImGui::SmallButton("Front")) { engCam_.yaw = 0; engCam_.pitch = 0; engCam_.panX = engCam_.panY = 0; }
    ImGui::SameLine();
    if (ImGui::SmallButton("Side")) { engCam_.yaw = 90; engCam_.pitch = 8; engCam_.panX = engCam_.panY = 0; }
    ImGui::SameLine();
    if (ImGui::SmallButton("3/4")) { engCam_.yaw = 38; engCam_.pitch = 18; engCam_.panX = engCam_.panY = 0; }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140);
    if (engCam_.flat) {
        ImGui::SliderFloat("Cutaway depth", &engCam_.depth, 0.3f, 2.5f, "%.1f");
        Help("Each cylinder / rotor is a flat cutaway in its own plane along the crankshaft. Drag to orbit the\n"
             "view, right-drag to pan, wheel to zoom, double-click to reset. Cutaway depth sets how thick the\n"
             "layered parts are.");
    } else {
        ImGui::SliderFloat("Glass", &engCam_.glass, 0.0f, 0.6f, "%.2f");
        Help("How solid the see-through cylinders and heads look (0 = invisible). Drag to orbit the view,\n"
             "right-drag to pan, wheel to zoom, double-click to reset. The model is built from the engine's own\n"
             "dimensions: bore, stroke, rod, layout, valves, cams, induction and exhaust.");
    }
    ImGui::SameLine();
    ImGui::Checkbox("Flat view", &engCam_.flat);
    Help("The older flat cutaway drawing (also used when the 3D renderer is not available).");
    float phases[kMaxCyl];
    computePhases(p_, phases);
    ui::EngineViewLive live;
    live.advance = tel_.running ? tel_.advance : p_.baseAdvance + p_.mechAdvance * 0.5f;
    live.vtecBlend = tel_.vtecEngaged;
    live.vtcAdvance = tel_.vtcAdvance;
    if (isDiesel(p_) && tel_.running) live.advance = tel_.soi;   // start of injection, deg BTDC
    live.time = (float)ImGui::GetTime();
    live.shaftRpm = tel_.running ? tel_.shaftRpm : 0.0f;
    live.shaftRpm2 = tel_.running ? tel_.shaftRpm2 : 0.0f;
    live.animSpeed = animSpeed_;
    live.throttle = tel_.throttle;
    live.boostBar = tel_.boostBar;
    live.egtGas = tel_.running ? tel_.egtGas : 0.0f;
    if (tel_.running && tel_.egtPerCyl)
        for (int i = 0; i < kMaxCyl; ++i) live.egtC[i] = tel_.egtCylC[i];
    live.glowPlugOn = tel_.glowOn;
    live.egr = tel_.running ? tel_.egrPos : 0.0f;
    {   // an afterfire pop flashes a flame at the tail pipe, fading over ~0.25 s
        static int lastAf = 0;
        static float flash = 0, lastT = 0;
        const float now = live.time;
        flash *= std::exp(-std::max(0.0f, now - lastT) / 0.08f);
        lastT = now;
        if (tel_.afterfires != lastAf) { if (tel_.afterfires > lastAf) flash = 1.0f; lastAf = tel_.afterfires; }
        live.popFlash = flash;
    }
    ImVec2 avail = ImGui::GetContentRegionAvail();
    EngineView3D(p_, phases, animAngle_, live, engCam_, ImVec2(avail.x, std::max(120.0f, avail.y)));
}

// ======================================================================= components
// one line "label: value" with a colour judging it against a limit
static void Judged(const char* label, const char* value, bool good, bool warn = false) {
    ImGui::TextDisabled("%s", label);
    ImGui::SameLine(ImGui::GetFontSize() * 13.0f);
    ImVec4 c = good ? ImVec4(0.45f, 0.9f, 0.5f, 1) : warn ? ImVec4(1.0f, 0.8f, 0.3f, 1) : ImVec4(1.0f, 0.4f, 0.3f, 1);
    ImGui::TextColored(c, "%s", value);
}

void App::panelComponents() {
    ImGui::PushItemWidth(-200);
    ComponentSet& c = p_.comp;
    ComponentModel m;
    m.build(p_);
    const float lim = p_.revLimit;
    char b[128];

    if (ImGui::Button("Size parts to this engine")) applyComponentDefaults(p_);
    Help("Pick materials, masses and springs that suit the current bore, cam and rev limit\n"
         "(safe a little beyond the limiter). Changing the engine later does NOT resize the parts:\n"
         "raise the rev limit on stock parts and they will float valves or break.");
    ImGui::SameLine();
    Check("Allow mechanical failure", &c.allowFailure);
    Help("When on, over-stressed parts, heavy knock and valve-to-piston contact accumulate damage\n"
         "until something breaks and the engine locks up. When off you only get warnings.");
    if (tel_.failure) {
        static const char* what[] = {"", "piston failure", "connecting rod failure", "crankshaft failure",
                                     "valve hit the piston", "piston destroyed by knock"};
        ImGui::TextColored(ImVec4(1, 0.3f, 0.25f, 1), "ENGINE FAILED: %s", what[clampv(tel_.failure, 0, 5)]);
        ImGui::SameLine();
        if (ImGui::Button("Repair engine")) c.repairCount++;
    } else if (c.allowFailure) {
        Meter("Damage", tel_.damage * 100, 0, 100, "%.0f %%", tel_.damage > 0.5f ? rgba(1, 0.3f, 0.2f) : rgba(0.9f, 0.6f, 0.2f));
    }

    if (m.piston) {
        ImGui::SeparatorText("Rotating assembly");
        Combo("Pistons", &c.pistonType, pistonTypeNames());
        Help("Cast: cheap, weakest. Hypereutectic: stronger but brittle - knock cracks it quickly.\n"
             "Forged: strongest and survives knock far better.");
        SliderF("Piston mass", &c.pistonMass_g, 150, 1200, "%.0f g", "Piston + pin + rings. Lighter = less inertia load, revs higher safely.");
        SliderF("Ring tension", &c.ringTension, 0.5f, 1.8f, "%.2f x", "Ring pressure on the bore: low-tension race rings cut friction (and seal a bit less).");
        Combo("Connecting rods", &c.rodType, rodTypeNames());
        SliderF("Rod mass", &c.rodMass_g, 200, 1400, "%.0f g", "1/3 of the rod moves with the piston, 2/3 rotates with the crank.");
        Combo("Crankshaft", &c.crankType, crankTypeNames());
        std::snprintf(b, sizeof(b), "%.0f g reciprocating, %.0f g rotating per cylinder", m.recipMass * 1000, m.rotMass * 1000);
        Judged("Moving mass", b, true);
        std::snprintf(b, sizeof(b), "+%.4f kg m2 on the crank (flywheel %.3f)", m.extraInertia, p_.flywheel);
        Judged("Added inertia", b, true);
        static const char* part[] = {"pistons", "rods", "crankshaft"};
        std::snprintf(b, sizeof(b), "%.0f rpm (limited by the %s)  vs limiter %.0f", m.safeRpm, part[m.weakest], lim);
        Judged("Safe speed", b, m.safeRpm > lim * 1.05, m.safeRpm > lim);
        Meter("Inertia stress now", tel_.stress * 100, 0, 150, "%.0f %% of limit", tel_.stress > 1 ? rgba(1, 0.25f, 0.2f) : rgba(0.5f, 0.7f, 0.9f));

        ImGui::SeparatorText("Valvetrain");
        Combo("Lifters / followers", &c.lifter, lifterTypeNames());
        Help("Moving mass the spring must control per valve. Pushrod lifters are heavy (float early) and\n"
             "flat tappets rub the most; DOHC buckets and finger followers are light.");
        SliderF("Spring force at full lift", &c.springOpen_N, 150, 2000, "%.0f N", "Must exceed (moving mass x cam deceleration) or the valve floats.\nStiffer springs add valvetrain friction.");
        SliderF("Spring seat force", &c.springSeat_N, 80, 1000, "%.0f N", "Holds the valve shut; part of the valvetrain friction.");
        std::snprintf(b, sizeof(b), "intake %.0f rpm, exhaust %.0f rpm  (moving mass %.0f / %.0f g)", m.floatRpmIn, m.floatRpmEx,
                      m.moveMassIn * 1000, m.moveMassEx * 1000);
        double fl = std::min(m.floatRpmIn, m.floatRpmEx);
        Judged("Valve float from", b, fl > lim * 1.05, fl > lim);
        Meter("Valve float now", std::max(tel_.floatIn, tel_.floatEx) * 100, 0, 100, "%.0f %%", rgba(1, 0.6f, 0.2f));
        valveDynamicsUi();
        ImGui::TextDisabled("Cam profile (duration, lift, timing): Valvetrain tab.");
    } else {
        ImGui::SeparatorText("Rotating assembly / valvetrain");
        ImGui::TextDisabled("Rotary engine: no pistons, rods or valves. Rotors, ports and apex seals are set in the Engine and Ports tabs.");
    }

    ImGui::SeparatorText("Air path");
    {
        // catalogue pickers: show what is fitted now, choosing an entry applies its typical dimensions
        int im = -1;
        const char* imNow = p_.itb ? "individual throttle bodies" : p_.runnerLen_mm >= 350 ? "long-runner street manifold" : "short-runner plenum";
        if (ImGui::BeginCombo("Intake manifold", imNow)) {
            if (ImGui::Selectable("Long-runner street manifold")) im = 0;
            if (ImGui::Selectable("Short-runner race plenum")) im = 1;
            if (ImGui::Selectable("Individual throttle bodies")) im = 2;
            ImGui::EndCombo();
        }
        if (im >= 0) {
            if (im == 0) { p_.itb = 0; p_.runnerLen_mm = 450; p_.plenum_L = std::max(2.0f, (float)equivalentDisplacementL(p_) * 1.4f); }
            if (im == 1) { p_.itb = 0; p_.runnerLen_mm = 200; p_.plenum_L = std::max(2.5f, (float)equivalentDisplacementL(p_) * 2.0f); }
            if (im == 2) { p_.itb = 1; p_.runnerLen_mm = 280; }
        }
        Help("Sets runner length / plenum volume / ITB (fine-tune them in Fuel Tuning > Air path).\n"
             "Long runners: torque low down. Short runners + big plenum: power up top.");
        int ex = -1;
        const char* exNow = p_.header == HeaderType::Log ? "cast log manifold" : p_.header == HeaderType::TriY ? "tri-Y / 4-2-1 header"
                            : p_.header == HeaderType::OpenStacks ? "open stacks" : p_.primaryLen_mm[0] >= 600 ? "long-tube header" : "shorty header";
        if (!isRotary(p_) && !(isTwoStroke(p_) && p_.tsPipe) && ImGui::BeginCombo("Exhaust manifold", exNow)) {
            if (ImGui::Selectable("Cast log manifold")) ex = 0;
            if (ImGui::Selectable("Shorty header")) ex = 1;
            if (ImGui::Selectable("Long-tube header")) ex = 2;
            if (ImGui::Selectable("Tri-Y / 4-2-1 header")) ex = 3;
            if (ImGui::Selectable("Open stacks (zoomies)")) ex = 4;
            ImGui::EndCombo();
        }
        if (ex >= 0) {
            float len[] = {140, 300, 800, 450, 350};
            HeaderType ht[] = {HeaderType::Log, HeaderType::Collector, HeaderType::Collector, HeaderType::TriY, HeaderType::OpenStacks};
            p_.header = ht[ex];
            for (int i = 0; i < kMaxCyl; ++i) p_.primaryLen_mm[i] = len[ex];
        }
        Help("Sets the header type and primary length (details in Engine Setup > Exhaust).");
    }
    Combo("Throttle body", &c.throttleType, throttleTypeNames());
    Help("Cable: the butterfly follows the pedal at once. Drive-by-wire: an electric motor moves it\n"
         "(response time below); the ECU throttle control arrives in Phase 3.");
    SliderF("Throttle bore", &p_.throttleBore_mm, 20, 120, "%.0f mm");
    if (c.throttleType == ThrottleType::DriveByWire) SliderF("DBW response", &c.dbwResponse_ms, 5, 200, "%.0f ms", "Motor time constant: slower = softer, laggier tip-in.");

    if (p_.fuelSystem == FuelSystem::EFI) {
        ImGui::SeparatorText("Injectors");
        SliderF("Flow", &p_.injector_ccmin, 80, 2400, "%.0f cc/min");
        SliderF("Dead time", &c.injDeadTime_ms, 0.2f, 2.5f, "%.2f ms", "Opening delay: part of every pulse that sprays nothing. Eats into the\nmaximum fuel per cycle at high rpm (duty includes it).");
        Meter("Injector duty", tel_.injDuty * 100, 0, 100, "%.0f %%", tel_.injDuty > 0.9f ? rgba(1, 0.2f, 0.2f) : rgba(0.6f, 0.6f, 0.9f));
        ImGui::SeparatorText("Fuel supply");
        SliderF("Fuel pressure", &c.fuelPressure_bar, 1.5f, 6.0f, "%.1f bar", "Regulator base pressure. Injector flow is rated at 3 bar and grows with the\nsquare root of the pressure across the nozzle.");
        Combo("Regulator", &c.fuelRegulator, fuelRegulatorNames());
        Help("Manifold referenced: rail pressure rises with boost, so the pressure across the injector stays\n"
             "constant. Returnless: fixed rail pressure - boost pushes back against the injector and it flows less\n"
             "(the ECU compensates using the fuel pressure and MAP sensors).");
        SliderF("Fuel pump", &c.fuelPumpLph, 20, 600, "%.0f L/h", "Pump flow at the base pressure. If the engine needs more, the rail pressure sags\nand the engine runs lean.");
        Meter("Rail pressure now", tel_.fuelBar, 0, 8, "%.2f bar", tel_.fuelBar < c.fuelPressure_bar * 0.85f && tel_.running ? rgba(1, 0.3f, 0.2f) : rgba(0.9f, 0.7f, 0.3f));
    }

    ImGui::SeparatorText("Ignition");
    Combo("Coils", &c.coilType, coilTypeNames());
    Help("Distributor: one coil fires every cylinder, so it has the least time to charge at high rpm.\n"
         "Wasted spark: one coil per pair, fires every revolution. Coil on plug: one per cylinder.");
    SliderF("Coil energy", &c.coilEnergy_mJ, 10, 150, "%.0f mJ");
    SliderF("Coil charge time", &c.coilCharge_ms, 1, 8, "%.1f ms", "Time to charge fully. Less dwell available than this = a weaker spark.");
    SliderI("Plug heat range", &c.plugHeatRange, 4, 10, "NGK-style: lower = hotter tip. Too hot under boost / high load -> the tip glows\n"
                                                       "(pre-ignition -> knock). Too cold at idle running rich -> carbon fouling -> misfires.");
    SliderF("Plug gap", &c.plugGap_mm, 0.4f, 1.6f, "%.2f mm", "Wider gap: lights leaner mixtures and burns a bit faster, but needs more energy\n"
                                                           "(misfires under boost or with a weak coil).");
    {
        double avail = m.sparkEnergy(p_, lim);
        double pc = (14.0 + 14.0 * std::max(0.0, (double)tel_.boostBar)) * 1e5;
        double need = ComponentModel::sparkNeeded(p_, pc, 0.85);
        std::snprintf(b, sizeof(b), "%.1f mJ available at the limiter, ~%.1f mJ needed at full load", avail, need);
        Judged("Spark energy", b, avail > need * 1.3, avail > need);
        Meter("Spark margin now", std::min(3.0f, tel_.sparkMargin) * 100, 0, 300, "%.0f %%", tel_.sparkMargin < 1 ? rgba(1, 0.3f, 0.2f) : rgba(0.4f, 0.8f, 0.5f));
        Meter("Plug tip temperature", tel_.plugTempC, 300, 1000, "%.0f C", tel_.plugTempC > 850 ? rgba(1, 0.3f, 0.2f) : tel_.plugTempC < 450 ? rgba(0.4f, 0.6f, 1) : rgba(0.9f, 0.7f, 0.3f));
        if (tel_.fouling > 0.01f) Meter("Plug fouling", tel_.fouling * 100, 0, 100, "%.0f %%", rgba(0.6f, 0.5f, 0.4f));
    }

    ImGui::SeparatorText("Pumps");
    SliderF("Oil pump capacity", &c.oilPumpCapacity, 0.4f, 2.0f, "%.2f x", "Pressure rises with rpm until the relief valve opens. A bigger pump costs drive power.");
    SliderF("Oil relief pressure", &c.oilRelief_bar, 2, 9, "%.1f bar");
    std::snprintf(b, sizeof(b), "%.1f bar at 900 rpm, %.1f bar at the limiter (oil at 90 C)", ComponentModel::oilPressure(p_, 900, 90),
                  ComponentModel::oilPressure(p_, lim, 90));
    Judged("Oil pressure", b, ComponentModel::oilPressure(p_, 900, 90) > 0.8, ComponentModel::oilPressure(p_, 900, 90) > 0.5);
    Meter("Oil pressure now", tel_.oilBar, 0, 8, "%.1f bar", tel_.running && tel_.oilBar < 0.7f ? rgba(1, 0.3f, 0.2f) : rgba(0.9f, 0.75f, 0.3f));
    SliderF("Water pump capacity", &c.waterPumpCapacity, 0.4f, 2.0f, "%.2f x", "Coolant circulation through the radiator (and its drive power).\nToo small: the engine runs hot under load.");
    Meter("Coolant now", tel_.coolantC, 0, 130, "%.0f C", tel_.coolantC > 105 ? rgba(1, 0.3f, 0.2f) : rgba(0.4f, 0.7f, 1.0f));
    ImGui::SeparatorText("Cooling");
    SliderF("Radiator size", &c.radiatorSize, 0.6f, 2.0f, "%.2f x",
            "1 = the stock core, sized for this engine's heat at its rated output: with the fan on and 50 km/h of air it holds\n"
            "the coolant near 100 C at full power. Bigger (a thicker / race core) runs cooler under sustained load; the\n"
            "thermostat keeps it from overcooling.");
    Combo("Oil cooler", &c.oilCooler, "None\0Oil-to-water (on the block)\0Air-to-oil (in front of the radiator)\0");
    Help("Oil-to-water: the oil gives its heat to the coolant (2.5x the block's own exchange) - it also warms up faster.\n"
         "Air-to-oil: its own core in the air stream, sized for ~5 % of the rated fuel power - the oil can run below the\n"
         "coolant at speed. Turbos add heat to the oil through their bearings.");
    {
        char cb[96];
        std::snprintf(cb, sizeof(cb), "radiator %.0f kW now, %.0f kW of rated fuel power designed for", tel_.radiatorKw,
                      ThermalModel::ratedFuelPowerW(p_) * 1e-3);
        ImGui::TextDisabled("%s", cb);
    }
    Meter("Oil now", tel_.oilC, 0, 150, "%.0f C", tel_.oilC > 130 ? rgba(1, 0.3f, 0.2f) : rgba(0.9f, 0.75f, 0.3f));
    std::snprintf(b, sizeof(b), "%+.1f kPa at 1500 rpm, %+.1f kPa at the limiter (vs standard parts)",
                  m.extraFmep(p_, 1500, 2 * p_.stroke_mm * 1e-3 * 1500 / 60, equivalentDisplacementL(p_) * 5e-4) * 1e-3,
                  m.extraFmep(p_, lim, 2 * p_.stroke_mm * 1e-3 * lim / 60, equivalentDisplacementL(p_) * 5e-4) * 1e-3);
    Judged("Extra friction", b, true);
    ImGui::PopItemWidth();
}

} // namespace es
