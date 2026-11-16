// Phase 5 UI: launch control / 3-step / flat shift / shift cut / transbrake (Engine Control), traction control
// (Engine Control), anti-lag + rolling anti-lag and nitrous (Boost Control).
#include "ui/App.h"
#include "ui/Widgets.h"
#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace es {
using namespace ui;

static void statusLine(const char* k, const char* v, bool active) {
    ImGui::TextDisabled("%s", k);
    ImGui::SameLine(ImGui::GetFontSize() * 15.0f);
    if (active) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1), "%s", v);
    else ImGui::TextUnformatted(v);
}

static bool checkI(const char* label, int32_t* v) {
    bool b = *v != 0;
    if (!ImGui::Checkbox(label, &b)) return false;
    *v = b ? 1 : 0;
    return true;
}

// ------------------------------------------------------------------ Engine Control > Launch / shift
void App::tabLaunch() {
    Calibration& c = p_.cal;
    char b[160];
    std::snprintf(b, sizeof(b), "%s%s   clutch switch %s   %.0f km/h", launchStateName(tel_.launchState),
                  tel_.launchLimitRpm > 0 ? (" @ " + std::to_string((int)std::lround(tel_.launchLimitRpm)) + " rpm").c_str() : "",
                  p_.clutchPedal > 0.5f ? "ON" : "off", tel_.speedKmh);
    statusLine("Now", b, tel_.launchState != kLaunchOff);
    ImGui::TextDisabled("Keys: hold C (clutch) + full throttle at a standstill = 2-step. T (hold) = transbrake.");
    ImGui::PushItemWidth(-260);

    ImGui::SeparatorText("Launch control (2-step)");
    checkI("Launch control enabled", &c.launchControl);
    Help("Standing still (below the speed below), clutch switch on - or the transbrake held - and the throttle\n"
         "past the minimum: the ECU holds the revs at the launch rpm by cutting sparks (or fuel) and runs the\n"
         "launch timing. With a spark cut the fuel of the cut cylinders burns in the hot exhaust: the 2-step bangs.\n"
         "On a turbo engine retarded launch timing puts heat into the turbine and builds boost before the launch.");
    ImGui::SliderFloat("Launch rpm", &c.launchRpm, 1500, std::max(2000.0f, p_.revLimit), "%.0f rpm");
    ImGui::SliderFloat("Launch timing", &c.launchTiming, -30, 40, "%.1f deg BTDC");
    ImGui::SliderFloat("Armed above throttle", &c.launchMinTps, 0, 100, "%.0f %%");
    ImGui::SliderFloat("Standing below", &c.launchMaxSpeed, 0, 40, "%.0f km/h");
    ImGui::Combo("Cut", &c.launchCutMode, "Spark cut\0Fuel cut\0");
    ImGui::SliderFloat("Cut band", &c.launchCutBand, 30, 600, "+-%.0f rpm");
    Help("The share of cut events ramps from 0 at limit - band to 100 % at limit + band.");
    ImGui::SliderFloat("Enrichment while holding", &c.launchEnrich, 0, 50, "%.0f %%");

    ImGui::SeparatorText("Clutch in while moving (3-step / flat shift)");
    ImGui::Combo("Mode", &c.clutchMovingMode, "Off\0Three-step (fixed rpm)\0Flat shift (hold the clutch-in rpm)\0");
    Help("Moving, throttle past the minimum, clutch switch on:\n"
         "  three-step - a second fixed limit (rolling starts, burnouts), with the launch timing;\n"
         "  flat shift - the revs are held where the clutch went in (+ margin), so the driver can shift\n"
         "  without lifting: the next gear bites at the same revs and the turbo stays spooled.");
    if (c.clutchMovingMode == 1) ImGui::SliderFloat("Three-step rpm", &c.threeStepRpm, 1500, std::max(2000.0f, p_.revLimit), "%.0f rpm");
    if (c.clutchMovingMode == 2) {
        ImGui::SliderFloat("Flat shift margin", &c.flatShiftMargin, 0, 800, "+%.0f rpm");
        ImGui::SliderFloat("Flat shift retard", &c.flatShiftRetard, 0, 30, "%.0f deg");
    }

    ImGui::SeparatorText("Shift cut (sequential / auto-clutch / automatic gearbox)");
    checkI("Shift cut enabled", &c.shiftCut);
    Help("On the gear-change signal all sparks are cut for the cut time (throttle past the minimum): the\n"
         "load comes off the gearbox and the revs do not flare while the clutch is out.");
    float ms = c.shiftCutTime * 1000.0f;
    if (ImGui::SliderFloat("Cut time", &ms, 20, 400, "%.0f ms")) c.shiftCutTime = ms * 0.001f;
    ImGui::SliderFloat("Above throttle", &c.shiftCutMinTps, 0, 100, "%.0f %%");

    ImGui::SeparatorText("Transbrake");
    const bool tbOk = p_.transMode != TransMode::Manual;
    ImGui::BeginDisabled(!tbOk);
    checkI("Transbrake button (hold T)", &p_.transbrake);
    ImGui::EndDisabled();
    Help("Automatic / auto-clutch gearbox in gear: the transbrake holds the car while the engine sits on the\n"
         "2-step (the button arms the launch control like the clutch switch). Release it to launch.");
    if (!tbOk) ImGui::TextDisabled("Needs an automatic or auto-clutch gearbox (Engine Setup > Vehicle).");
    else if (tel_.transbrake) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1), "TRANSBRAKE holding the car");
    ImGui::PopItemWidth();
}

// ------------------------------------------------------------------ Engine Control > Traction control
void App::tabTraction() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    char b[180];
    std::snprintf(b, sizeof(b), "slip %.0f %% (target %.0f %%)   reduction %.0f %%: retard %.1f deg, cut %.0f %%   wheels %.0f / car %.0f km/h",
                  tel_.tcSlip, tel_.tcTarget, tel_.tcReduction * 100, tel_.tcRetard, tel_.tcCut * 100, tel_.wheelKmh, tel_.speedKmh);
    statusLine("Now", b, tel_.tcReduction > 0.01f);
    if (!p_.vehicle) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Traction control works in driving mode (V) - the dyno has no wheels.");
    checkI("Traction control enabled", &c.tractionControl);
    Help("Slip = driven wheel speed against the vehicle speed. Above the target slip the ECU takes torque out:\n"
         "the first 40 %% of the reduction is ignition retard (smooth), the rest cuts sparks (strong).\n"
         "A PI controller: P reacts to the slip error at once, I holds the reduction the surface needs.");
    ImGui::PushItemWidth(-260);
    ImGui::SliderFloat("P gain", &c.tcKp, 0, 0.1f, "%.3f per %% slip");
    ImGui::SliderFloat("I gain", &c.tcKi, 0, 0.5f, "%.3f per %% s");
    ImGui::SliderFloat("Max retard", &c.tcMaxRetard, 0, 30, "%.0f deg");
    ImGui::SliderFloat("Max spark cut", &c.tcMaxCut, 0, 1, "%.2f");
    ImGui::PopItemWidth();
    ImGui::SeparatorText("Allowed slip (%) over vehicle speed");
    curveEd_[kCalTcSlip].drawCurve("tcslip", c.tcSlipTarget, calCurveInfo(kCalTcSlip), tel_.speedKmh, p_.vehicle != 0, ref ? &ref->tcSlipTarget : nullptr);
    Help("Tyres grip best at roughly 10-20 % slip; more is allowed at low speed where the slip ratio is noisy.");
}

// ------------------------------------------------------------------ Boost Control > Anti-lag
void App::tabAntiLag() {
    Calibration& c = p_.cal;
    const bool turbo = p_.induction == Induction::Turbo;
    if (!turbo) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Anti-lag needs a turbo (Hardware tab). Rolling anti-lag still holds the revs, but builds no boost.");
    char b[128];
    std::snprintf(b, sizeof(b), "%s   boost %.2f bar   EGT %.0f C", tel_.antiLag ? (tel_.launchState == kLaunchRollingAls ? "ROLLING ANTI-LAG" : "ANTI-LAG") : "off",
                  tel_.boostBar, tel_.snsValue[kSnsEGT]);
    statusLine("Now", b, tel_.antiLag);
    ImGui::PushItemWidth(-260);
    ImGui::SeparatorText("Anti-lag on lift-off");
    checkI("Anti-lag (ALS) enabled", &p_.antiLag);
    Help("Off the throttle above the minimum rpm, for up to the max time: the throttle is cracked open and\n"
         "the spark is so late that the mixture is still burning when the exhaust valve opens - the energy goes\n"
         "into the turbine and keeps it spinning. Hard on the turbo and the exhaust (EGT), loud.");
    ImGui::SliderFloat("ALS timing", &c.alsTiming, -40, 10, "%.0f deg BTDC");
    ImGui::SliderFloat("Throttle crack", &c.alsThrottle, 0, 40, "%.0f %%");
    ImGui::SliderFloat("Max time after lift-off", &c.alsMaxTime, 0.2f, 10, "%.1f s");
    ImGui::SliderFloat("Active above", &c.alsMinRpm, 1000, 6000, "%.0f rpm");
    ImGui::SeparatorText("Rolling anti-lag (button)");
    checkI("Rolling anti-lag enabled (hold G)", &c.rollingAls);
    Help("Moving, throttle past the launch arming minimum, button held: the revs are held where the button went\n"
         "in (spark cut over the launch cut band) with the ALS timing - the car rolls at a steady speed while the\n"
         "turbo builds boost. Release the button for full power with the boost already there.");
    checkI("Button held", &p_.rollingAlsButton);
    ImGui::SliderFloat("Active above", &c.rollingAlsMinSpeed, 0, 80, "%.0f km/h");
    ImGui::PopItemWidth();
}

// ------------------------------------------------------------------ Boost Control > Nitrous
void App::tabNitrous() {
    Calibration& c = p_.cal;
    char b[180];
    const char* st = !p_.nitrous ? "not fitted" : tel_.n2oLeanLock ? "LEAN - shut off until the throttle is lifted"
                   : tel_.n2oDuty > 0 ? "SPRAYING" : p_.n2oArm ? "armed" : "disarmed";
    std::snprintf(b, sizeof(b), "%s   duty %.0f %%   %.0f g/s   bottle %.2f / %.1f kg   IAT %.0f C",
                  st, tel_.n2oDuty * 100, tel_.n2oFlowGs, tel_.n2oBottleKg, p_.n2oBottleKg, tel_.snsValue[kSnsIAT]);
    statusLine("Now", b, tel_.n2oDuty > 0 || tel_.n2oLeanLock);
    ImGui::PushItemWidth(-260);
    ImGui::SeparatorText("Kit");
    checkI("Nitrous system fitted", &p_.nitrous);
    ImGui::SliderFloat("Shot (jetted for)", &p_.n2oShotHp, 25, 500, "%.0f hp");
    Help("The jets set the N2O flow (about 0.032 kg/s per 100 hp). N2O carries more oxygen than air and gives\n"
         "off heat as it breaks down; the liquid flashing to gas cools the charge.");
    ImGui::Combo("Fuel", &p_.n2oWet, "Dry (the ECU adds the fuel)\0Wet (the kit's own fuel solenoid)\0");
    ImGui::SliderFloat("Bottle", &p_.n2oBottleKg, 1, 15, "%.1f kg");
    ImGui::SameLine();
    if (ImGui::SmallButton("Refill")) p_.n2oRefill++;
    checkI("Arming switch (X)", &p_.n2oArm);
    ImGui::SeparatorText("ECU nitrous controller");
    ImGui::SliderFloat("Above throttle", &c.n2oMinTps, 0, 100, "%.0f %%");
    ImGui::DragFloatRange2("rpm window", &c.n2oMinRpm, &c.n2oMaxRpm, 25, 0, 15000, "from %.0f", "to %.0f rpm");
    ImGui::SliderFloat("Min coolant", &c.n2oMinEct, 0, 100, "%.0f C");
    ImGui::SliderFloat("Min speed", &c.n2oMinSpeed, 0, 100, "%.0f km/h");
    ImGui::SliderFloat("Delay", &c.n2oDelay, 0, 3, "%.2f s");
    Help("The conditions must hold this long before the solenoid opens (no nitrous on a short stab).");
    ImGui::SliderFloat("Progressive: start", &c.n2oStartPct, 0, 100, "%.0f %% duty");
    ImGui::SliderFloat("Progressive: ramp", &c.n2oRampTime, 0, 5, "%.1f s to 100 %%");
    Help("The solenoid is pulsed: the flow starts at the start duty and rises to full over the ramp time,\n"
         "so the tyres are not hit with the whole shot at once.");
    ImGui::SliderFloat("Timing retard at full flow", &c.n2oRetard, 0, 20, "%.1f deg");
    Help("Nitrous burns faster and hotter: rule of thumb ~1 deg per 25 hp of shot.");
    ImGui::BeginDisabled(p_.n2oWet != 0);
    ImGui::SliderFloat("Dry: ECU N2O flow", &c.n2oFlowGs, 0, 200, "%.1f g/s");
    ImGui::SameLine();
    if (ImGui::SmallButton("Match jet")) c.n2oFlowGs = (float)(n2oFullFlowKgs(p_) * 1000.0);
    ImGui::EndDisabled();
    Help("A dry kit sprays only N2O; the ECU adds the fuel for the flow it believes the jet delivers. A wrong\n"
         "value runs the engine lean (dangerous) or rich.");
    ImGui::SliderFloat("Lean shut-off", &c.n2oLeanLambda, 0.8f, 1.2f, "lambda %.2f");
    Help("Wideband leaner than this while spraying: the nitrous is shut off until the throttle is lifted.");
    ImGui::PopItemWidth();
}

} // namespace es
