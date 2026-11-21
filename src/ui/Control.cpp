// Phase 3 UI: closed-loop fuel trims, knock control, Engine Control (idle, drive-by-wire, rev limiter),
// ECU boost control and Engine Protection.
#include "ui/App.h"
#include "ui/Widgets.h"
#include "imgui.h"
#include "implot.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace es {
using namespace ui;

static void rowLabel(const char* k, const char* v) {
    ImGui::TextDisabled("%s", k);
    ImGui::SameLine(ImGui::GetFontSize() * 15.0f);
    ImGui::TextUnformatted(v);
}

static void statusChip(int state) {
    static const char* txt[3] = {"OK", "WARNING", "ACTIVE"};
    static const ImVec4 col[3] = {ImVec4(0.45f, 0.85f, 0.45f, 1), ImVec4(1, 0.8f, 0.3f, 1), ImVec4(1, 0.35f, 0.3f, 1)};
    const int s = std::clamp(state, 0, 2);
    ImGui::TextColored(col[s], "%s", txt[s]);
}

// ------------------------------------------------------------------ Fuel Tuning > Closed loop
void App::tabFuelTrims() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    ImGui::PushItemWidth(-260);
    ImGui::SeparatorText("Status");
    char b[160];
    std::snprintf(b, sizeof(b), "%s", c.autoTune ? "auto-tune is on: it is the closed loop (STFT held at 0)" :
                                      tel_.closedLoop ? "CLOSED LOOP" : "open loop");
    rowLabel("Mode", b);
    std::snprintf(b, sizeof(b), "STFT %+.1f %%   LTFT %+.1f %%   total %+.1f %%", tel_.ecuStft, tel_.ecuLtft, (tel_.ecuTrim - 1) * 100);
    rowLabel("Trims now", b);
    ImGui::SeparatorText("Closed loop (short-term trim)");
    bool cl = c.closedLoop != 0;
    if (ImGui::Checkbox("Closed loop enabled", &cl)) c.closedLoop = cl;
    Help("The short-term fuel trim (STFT) integrates the wideband error while the conditions below hold\n"
         "(warm, settled after start / transients, part load with a target leaner than AFR 13.5).\n"
         "Outside them it fades back to 0 %%. Auto-tune (Setup tab) replaces it when enabled.");
    ImGui::SliderFloat("STFT gain", &c.stftGain, 0.05f, 3.0f, "%.2f /s");
    Help("How fast the STFT follows the lambda error: 0.5 = half of the error corrected per second.");
    ImGui::SliderFloat("STFT limit", &c.stftLimit, 2, 40, "+-%.0f %%");
    ImGui::SeparatorText("Learning conditions");
    ImGui::SliderFloat("Min coolant temperature", &c.clMinEct, 0, 100, "%.0f C");
    ImGui::SliderFloat("Delay after start / transient", &c.clDelay, 0, 10, "%.1f s");
    ImGui::SliderFloat("Max MAP (open loop above)", &c.clMaxMap, 40, 250, "%.0f kPa");
    ImGui::SeparatorText("Long-term trim");
    bool lt = c.ltftLearn != 0;
    if (ImGui::Checkbox("Learn and apply long-term trim", &lt)) c.ltftLearn = lt;
    Help("What the STFT keeps correcting in a load cell is moved into the LTFT table, so the correction is\n"
         "already there next time - also in open loop. Large LTFT values mean the VE map (or the injector\n"
         "data) is off in that area: fix the map and clear the table.");
    ImGui::SliderFloat("LTFT learning rate", &c.ltftRate, 0.01f, 1.0f, "%.2f /s");
    ImGui::SliderFloat("LTFT limit", &c.ltftLimit, 2, 40, "+-%.0f %%");
    if (ImGui::Button("Clear long-term trims")) for (auto& row : c.ltft.v) for (float& v : row) v = 0;
    ImGui::PopItemWidth();
    ImGui::SeparatorText("Long-term trim table (% over rpm x MAP)");
    OpPoint op; op.x = tel_.ecuLoadX; op.y = tel_.ecuLoadY; op.valid = tel_.running;
    tabEd_[kCalLtft].draw("ltft", c.ltft, calTableInfo(kCalLtft), op, &trailMap_, ref ? &ref->ltft : nullptr, refName_.c_str());
}

// ------------------------------------------------------------------ Ignition Tuning > Knock control
void App::tabKnock() {
    Calibration& c = p_.cal;
    ImGui::PushItemWidth(-260);
    bool on = c.knockControl != 0;
    if (ImGui::Checkbox("Knock control enabled", &on)) c.knockControl = on;
    Help("The knock sensor signal is windowed by crank angle, so each combustion's knock is attributed to\n"
         "its cylinder. A cylinder that knocks above the threshold is retarded one step; all cylinders creep\n"
         "back to the map at the recovery rate. A failed knock sensor gives a fixed safety retard (half the max).");
    ImGui::SliderFloat("Knock threshold", &c.knockThreshold, 0.2f, 4.0f, "%.2f index");
    ImGui::SliderFloat("Retard per knock event", &c.knockRetardStep, 0.25f, 5.0f, "%.2f deg");
    ImGui::SliderFloat("Max knock retard", &c.knockMaxRetard, 1, 20, "%.0f deg");
    ImGui::SliderFloat("Recovery rate", &c.knockRecover, 0.1f, 10.0f, "%.1f deg/s");
    ImGui::SliderFloat("Active above", &c.knockMinRpm, 0, 4000, "%.0f rpm");
    ImGui::PopItemWidth();
    ImGui::SeparatorText("Live");
    ImGui::Text("Knock sensor %.2f   events %d", tel_.snsValue[kSnsKnock], tel_.knockEvents);
    const int n = std::clamp((int)(isRotary(p_) ? p_.cylinders : p_.cylinders), 1, kMaxCyl);
    if (ImPlot::BeginPlot("##kr", ImVec2(-1, ImGui::GetFontSize() * 12), ImPlotFlags_NoLegend | ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes(isRotary(p_) ? "rotor" : "cylinder", "retard deg", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, std::max(2.0f, c.knockMaxRetard), ImPlotCond_Always);
        float xs[kMaxCyl], ys[kMaxCyl];
        for (int i = 0; i < n; ++i) { xs[i] = (float)(i + 1); ys[i] = tel_.knockRetard[i]; }
        ImPlot::SetNextFillStyle(ImVec4(1, 0.45f, 0.25f, 0.9f));
        ImPlot::PlotBars("retard", xs, ys, n, 0.6);
        ImPlot::EndPlot();
    }
}

// ------------------------------------------------------------------ Engine Control
void App::sectionEngineControl() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    if (ImGui::BeginTabBar("ctltabs")) {
        if (ImGui::BeginTabItem("Idle")) {
            if (!efi) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Carburettor: idle speed is set by the idle screw (Fuel system); only idle spark control applies.");
            char b[128];
            std::snprintf(b, sizeof(b), "target %.0f rpm   now %.0f rpm   idle valve %.0f %%", tel_.idleTarget, tel_.rpm, tel_.iac);
            rowLabel("Idle", b);
            ImGui::SeparatorText("Idle target over coolant temperature");
            curveEd_[kCalIdleTarget].drawCurve("idlet", c.idleTarget, calCurveInfo(kCalIdleTarget), tel_.snsValue[kSnsECT], true, ref ? &ref->idleTarget : nullptr);
            ImGui::PushItemWidth(-260);
            ImGui::SeparatorText("Idle valve PID");
            ImGui::SliderFloat("P gain", &c.idleKp, 0, 0.003f, "%.5f /rpm");
            Help("Valve opening per rpm of error (full strength below the target, 43 %% above it - the stall side matters more).");
            ImGui::SliderFloat("I gain", &c.idleKi, 0, 0.001f, "%.5f /rpm s");
            ImGui::SliderFloat("D gain", &c.idleKd, 0, 0.0005f, "%.5f /(rpm/s)");
            Help("Damps on the rpm rate: opens the valve while the revs are falling towards the target.");
            ImGui::SeparatorText("Idle spark control");
            ImGui::SliderFloat("Spark gain", &c.idleSparkGain, 0, 0.15f, "%.3f deg/rpm");
            Help("Fast idle control through timing: advance when the revs sag, retard when they flare.");
            ImGui::SliderFloat("Max retard", &c.idleSparkMin, -20, 0, "%.0f deg");
            ImGui::SliderFloat("Max advance", &c.idleSparkMax, 0, 20, "%.0f deg");
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Throttle (DBW)", nullptr, tabJump("dbw"))) {
            const bool dbw = p_.comp.throttleType == ThrottleType::DriveByWire;
            if (!dbw) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Cable throttle: the pedal moves the butterfly directly - this map is not used (Engine Setup > Components > Throttle).");
            char b[128];
            std::snprintf(b, sizeof(b), "pedal %.0f %%   throttle %.0f %%%s", tel_.snsValue[kSnsAPP], tel_.throttle * 100,
                          tel_.throttleLimited ? "   LIMITED (protection)" : "");
            rowLabel("Now", b);
            ImGui::SeparatorText("Pedal -> throttle map");
            curveEd_[kCalDbwMap].drawCurve("dbw", c.dbwMap, calCurveInfo(kCalDbwMap), tel_.snsValue[kSnsAPP], dbw, ref ? &ref->dbwMap : nullptr);
            Help("Throttle opening the ECU asks for at each pedal position. A progressive map (less throttle at\n"
                 "small pedal) makes a sensitive engine easier to drive. The throttle motor response time is a\n"
                 "hardware property (Components). A failed pedal sensor leaves the throttle at idle.");
            ImGui::PushItemWidth(-260);
            if (dbw) ImGui::SliderFloat("Throttle motor response", &p_.comp.dbwResponse_ms, 5, 200, "%.0f ms");
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Rev limiter")) {
            ImGui::PushItemWidth(-260);
            ImGui::SliderFloat("Hard limit", &p_.revLimit, 1500, 12000, "%.0f rpm");
            int lim = (int)p_.limiter;
            if (ImGui::Combo("Hard cut", &lim, "Ignition cut\0Fuel cut\0")) p_.limiter = (Limiter)lim;
            ImGui::SeparatorText("Soft limiter");
            ImGui::SliderFloat("Soft band", &c.softLimitBand, 0, 1000, "%.0f rpm below the limit");
            Help("Across this band before the hard limit the ECU retards the timing and drops a growing share of\n"
                 "sparks, so the revs level off instead of bouncing on the hard cut. 0 = off.");
            ImGui::SliderFloat("Retard at the limit", &c.softLimitRetard, 0, 25, "%.0f deg");
            ImGui::SliderFloat("Sparks cut at the limit", &c.softLimitCut, 0, 1, "%.2f");
            ImGui::PopItemWidth();
            char b[96];
            std::snprintf(b, sizeof(b), "%s   cut %.0f %%", tel_.limiter ? "LIMITER ON" : "-", tel_.cutFraction * 100);
            rowLabel("Now", b);
            ImGui::EndTabItem();
        }
        if (isDiesel(p_) && ImGui::BeginTabItem("Diesel")) {
            char b[160];
            std::snprintf(b, sizeof(b), "%.1f mg/stroke   SOI %.1f deg BTDC   ignition delay %.1f deg   %s", tel_.dieselFuelMg, tel_.soi,
                          tel_.ignDelayDeg, tel_.smokeLimited ? "SMOKE LIMITED" : "");
            rowLabel("Now", b);
            ImGui::PushItemWidth(-260);
            ImGui::SliderFloat("Smoke limit lambda", &c.smokeLambda, 1.0f, 2.0f, "%.2f");
            Help("Never more fuel than the measured air allows at this lambda. Richer (lower) = more torque and black\n"
                 "smoke. 1.38 = air-fuel 20, the smoke limit quoted for diesels (MIT 2.61 lecture notes).");
            ImGui::SliderFloat("Idle governor P", &c.dieselIdleKp, 0, 0.1f, "%.3f mg/rpm");
            ImGui::SliderFloat("Idle governor I", &c.dieselIdleKi, 0, 0.2f, "%.3f mg/rpm s");
            ImGui::SliderFloat("Max-speed governor band", &c.dieselGovBand, 0, 1500, "%.0f rpm");
            Help("The fuel falls to zero over this band below the rev limit (a diesel governs its speed with the fuel).");
            ImGui::SliderFloat("Cranking fuel", &c.dieselStartFuel, 0, 150, "%.0f mg/stroke");
            ImGui::SeparatorText("Pilot injection");
            ImGui::PushItemWidth(-260);
            ImGui::SliderFloat("Pilot quantity", &c.pilotMg, 0, 5, "%.1f mg (0 = off)");
            ImGui::SliderFloat("Dwell (from 1500 rpm)", &c.pilotDwellUs, 0, 2000, "%.0f us");
            ImGui::SliderFloat("Dwell at idle", &c.pilotDwellIdleUs, 0, 2000, "%.0f us");
            ImGui::SliderFloat("Pilot up to", &c.pilotMaxRpm, 0, 6000, "%.0f rpm");
            ImGui::PopItemWidth();
            Help("A small injection before the main one: it lights first, and the main injection then lights sooner, with\n"
                 "less fuel burning all at once - less clatter (Sandia, Busch et al.: about 6 dB at 1500 rpm, 1.5 mg pilot,\n"
                 "dwell above 200 us). Too short a dwell: the pilot has not lit yet and does nothing.");
            ImGui::Text("Now: pilot %.1f mg at %.1f deg BTDC, main %.1f mg at %.1f deg; main delay after the pilot %.1f deg",
                        tel_.pilotMg, tel_.pilotSoi, tel_.dieselFuelMg, tel_.soi, tel_.mainDelayDeg);
            if (p_.dieselInjSystem == 1) {
                ImGui::SeparatorText("More injections (common rail)");
                ImGui::PushItemWidth(-260);
                ImGui::SliderFloat("Second pilot", &c.pilot2Mg, 0, 5, "%.1f mg (0 = off)");
                ImGui::SliderFloat("Its dwell", &c.pilot2DwellUs, 0, 2000, "%.0f us");
                ImGui::SliderFloat("Post injection", &c.postMg, 0, 10, "%.1f mg (0 = off)");
                ImGui::SliderFloat("Post dwell after the main", &c.postDwellUs, 0, 2000, "%.0f us");
                ImGui::SliderFloat("Post up to", &c.postMaxRpm, 0, 6000, "%.0f rpm");
                ImGui::PopItemWidth();
                Help("Post: a small injection right after the main (within a few degrees). It burns in the main's hot gas;\n"
                     "engines use it against soot (Sandia measured 16-55 %% less). This 0-D model burns it fully but does not\n"
                     "resolve the mixing that oxidises the soot - its soot effect here is small.");
                ImGui::Text("Now: second pilot %.1f mg, post %.1f mg at %.1f deg ATDC", tel_.pilot2Mg, tel_.postMg, -tel_.postSoi);
                ImGui::SeparatorText("Rail pressure: bar over rpm x fuel mg/stroke");
                OpPoint ro; ro.valid = tel_.running; ro.x = tel_.rpm; ro.y = tel_.dieselFuelMg;
                tabEd_[kCalRail].draw("rail", c.railMap, calTableInfo(kCalRail), ro, nullptr, ref ? &ref->railMap : nullptr, refName_.c_str());
                ImGui::Text("Now: rail %.0f bar (target %.0f)", tel_.railBar, tel_.railTargetBar);
            }
            ImGui::SeparatorText("Glow plugs");
            ImGui::SliderFloat("Post-heat at most", &c.afterglowTime, 0, 600, "%.0f s");
            ImGui::SliderFloat("Post-heat until coolant", &c.afterglowEct, 0, 100, "%.0f C");
            Help("BorgWarner / BERU: post-heating up to 3 minutes, ending when the coolant reaches 70 C - it cuts white\n"
                 "smoke and the cold-start knock (none when the engine was already that warm at the start).");
            curveEd_[kCalGlowPreheat].drawCurve("glowpre", c.glowPreheat, calCurveInfo(kCalGlowPreheat), tel_.snsValue[kSnsECT], true,
                                                ref ? &ref->glowPreheat : nullptr);
            ImGui::Text("Tip %.0f C  %s", tel_.glowTipC, tel_.glowLight ? "PRE-HEATING" : tel_.glowOn ? "heating" : "off");
            ImGui::SeparatorText("EGR valve: % over rpm x fuel mg/stroke");
            if (p_.egrValveDia_mm <= 0) ImGui::TextDisabled("No EGR valve fitted (Engine Setup > Intake / fuel).");
            ImGui::PushItemWidth(-260);
            ImGui::SliderFloat("EGR only above coolant", &c.egrMinEct, 0, 100, "%.0f C");
            ImGui::SliderFloat("EGR held back below lambda", &c.egrMinLambda, 1.0f, 3.0f, "%.2f");
            Help("An air-mass floor: when the measured fresh air per stroke falls below this lambda for the fuel, a trim closes\n"
                 "the valve (diesel ECUs run their EGR on an air-mass target).");
            ImGui::PopItemWidth();
            {
                OpPoint eo; eo.valid = tel_.running; eo.x = tel_.rpm; eo.y = tel_.dieselFuelMg;
                tabEd_[kCalEgr].draw("egr", c.egrMap, calTableInfo(kCalEgr), eo, nullptr, ref ? &ref->egrMap : nullptr, refName_.c_str());
            }
            Help("Open at idle and light load, closed towards full load (the engine needs all its air there - the smoke\n"
                 "limiter would take the fuel away). The starting map is a guess, not from a source.");
            ImGui::Text("Now: valve %.0f %%   %.1f g/s   %.0f %% of the charge", tel_.egrPos * 100.0f, tel_.egrGs, tel_.egrRate * 100.0f);
            ImGui::PopItemWidth();
            OpPoint op; op.valid = tel_.running;
            ImGui::SeparatorText("Driver demand: fuel mg/stroke over rpm x pedal %");
            op.x = tel_.rpm; op.y = tel_.snsValue[kSnsAPP];
            tabEd_[kCalDieselFuel].draw("dieselfuel", c.dieselFuel, calTableInfo(kCalDieselFuel), op, nullptr, ref ? &ref->dieselFuel : nullptr, refName_.c_str());
            ImGui::SeparatorText("Start of injection: deg BTDC over rpm x fuel mg/stroke");
            op.y = tel_.dieselFuelMg;
            tabEd_[kCalSoi].draw("soi", c.soi, calTableInfo(kCalSoi), op, nullptr, ref ? &ref->soi : nullptr, refName_.c_str());
            Help("Earlier = higher peak pressure, louder premixed burn, more efficient up to a point. Later = quieter,\n"
                 "hotter exhaust, less torque.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Cam control")) {
            if (!p_.vtec && !p_.vtc)
                ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "This engine has neither VTEC nor a VTC phaser (Engine Setup > Valvetrain).");
            char b[160];
            std::snprintf(b, sizeof(b), "%s cam%s   intake advance %.1f deg (target %.1f)", tel_.vtecEngaged > 0.5f ? "HIGH" : "low",
                          tel_.vtecCmd && tel_.vtecEngaged < 0.99f ? " (switching)" : "", tel_.vtcAdvance, tel_.vtcTarget);
            rowLabel("Now", b);
            ImGui::PushItemWidth(-260);
            ImGui::SeparatorText("VTEC switch-over");
            ImGui::BeginDisabled(!p_.vtec);
            ImGui::SliderFloat("Switch to the high cam above", &c.vtecRpm, 2000, std::max(3000.0f, p_.revLimit), "%.0f rpm");
            Help("Best where the high-cam torque curve crosses the low-cam one (a dyno sweep with VTEC off and on shows\n"
                 "it). The base map is measured just either side of it, so moving it far needs a new base map.");
            ImGui::SliderFloat("Hysteresis", &c.vtecHyst, 0, 1000, "%.0f rpm");
            ImGui::SliderFloat("Only above MAP", &c.vtecMinMap, 20, 110, "%.0f kPa");
            Help("Part-throttle cruising stays on the low cam (economy, smoothness); the high cam needs load.");
            ImGui::SliderFloat("Only above coolant", &c.vtecMinEct, 0, 100, "%.0f C");
            ImGui::EndDisabled();
            ImGui::PopItemWidth();
            ImGui::SeparatorText("VTC: intake cam advance (deg) over rpm x MAP");
            OpPoint op; op.x = tel_.rpm; op.y = tel_.ecuMap; op.valid = tel_.running;
            tabEd_[kCalVtc].draw("vtc", c.vtcMap, calTableInfo(kCalVtc), op, nullptr, ref ? &ref->vtcMap : nullptr, refName_.c_str());
            Help("More advance = earlier intake closing and more overlap: mid-range torque under load. Keep it small at\n"
                 "idle and light load (stable idle, little exhaust pulled back). Changing it changes the air the engine\n"
                 "breathes - re-measure the base map or let auto-tune follow.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Launch / shift")) { tabLaunch(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Traction control")) { tabTraction(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Torque / gear")) { tabTorqueGear(); ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }
}

// ------------------------------------------------------------------ Boost Control > ECU boost control
void App::tabBoostControl() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    const bool turbo = p_.induction == Induction::Turbo;
    if (!turbo) {
        ImGui::TextWrapped("ECU boost control drives a turbo wastegate. %s",
                           isBoosted(p_) ? "A supercharger's boost follows the pulley ratio and the bypass valve (Hardware tab)." :
                                           "This engine is naturally aspirated.");
        ImGui::PushItemWidth(-260);
        if (isBoosted(p_)) ImGui::SliderFloat("Boost cut", &c.boostCutBar, 0.2f, 4, "%.2f bar");
        ImGui::PopItemWidth();
        return;
    }
    char b[160];
    std::snprintf(b, sizeof(b), "target %.2f   actual %.2f bar   duty %s   wastegate %.0f %%%s%s", tel_.boostTargetBar, tel_.ecuBoostBar,
                  tel_.wgDuty >= 0 ? (std::to_string((int)std::lround(tel_.wgDuty * 100)) + " %").c_str() : "-", tel_.wastegatePos * 100,
                  tel_.overboostCut ? "   OVERBOOST CUT" : "", tel_.boostCut ? "   BOOST CUT" : "");
    rowLabel("Now", b);
    bool on = c.boostControl != 0;
    if (ImGui::Checkbox("ECU boost control (wastegate solenoid)", &on)) c.boostControl = on;
    Help("Off: the wastegate spring alone sets the boost (Hardware > wastegate spring).\n"
         "On: a solenoid bleeds the actuator line - the actuator sees less pressure, so the gate opens later and\n"
         "boost rises above the spring. The ECU runs a PID on the MAP sensor towards the target table, with a\n"
         "feed-forward duty from the spring pressure. It can only add boost: below the spring it is at 0 % duty.");
    ImGui::PushItemWidth(-260);
    ImGui::SliderFloat("P gain", &c.boostKp, 0, 3, "%.2f duty/bar");
    ImGui::SliderFloat("I gain", &c.boostKi, 0, 10, "%.2f duty/bar s");
    ImGui::SliderFloat("D gain", &c.boostKd, 0, 0.3f, "%.3f duty/(bar/s)");
    ImGui::SliderFloat("Max duty", &c.wgDutyMax, 0.1f, 1, "%.2f");
    ImGui::SeparatorText("Overboost protection");
    ImGui::SliderFloat("Overboost margin", &c.overboostMargin, 0.05f, 1, "%.2f bar above target");
    ImGui::SliderFloat("for longer than", &c.overboostTime, 0.05f, 3, "%.2f s");
    Help("Fuel cut until the boost is back down to the target.");
    ImGui::SliderFloat("Boost cut", &c.boostCutBar, 0.2f, 4, "%.2f bar");
    Help("Instant fuel cut above this boost, with or without ECU boost control.");
    ImGui::PopItemWidth();
    ImGui::SeparatorText("Target boost (bar) over rpm x throttle");
    OpPoint op; op.x = tel_.rpm; op.y = tel_.ecuTps; op.valid = tel_.running;
    tabEd_[kCalBoostTarget].draw("bst", c.boostTarget, calTableInfo(kCalBoostTarget), op, nullptr, ref ? &ref->boostTarget : nullptr, refName_.c_str());
    ImGui::SeparatorText("Boost by gear (% of the table, 0 = neutral)");
    curveEd_[kCalBoostGear].drawCurve("bgear", c.boostGear, calCurveInfo(kCalBoostGear), (float)tel_.gear, tel_.vehicle, ref ? &ref->boostGear : nullptr);
}

// ------------------------------------------------------------------ Engine Protection
void App::sectionProtection() {
    Calibration& c = p_.cal;
    if (tel_.shutdown) {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "ENGINE SHUTDOWN by protection - fuel and spark cut until the engine has stopped.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear now")) sim_->requestClearProtection();
    } else if (tel_.limp) ImGui::TextColored(ImVec4(1, 0.55f, 0.2f, 1), "LIMP MODE: throttle / rpm limited, boost to the spring, timing retarded.");
    else ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1), "No protection action active.");
    ImGui::TextDisabled("Each rule watches one sensor value. Past the limit for longer than the delay, its action engages; it clears "
                        "once the value is back past the limit by the recovery margin. Oil pressure limits are for 3000 rpm and scale down to 25 %% at idle.");
    const ImGuiTableFlags fl = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("##prot", 9, fl)) {
        const float em = ImGui::GetFontSize();
        ImGui::TableSetupColumn("On");
        ImGui::TableSetupColumn("Condition", ImGuiTableColumnFlags_WidthFixed, em * 13);
        ImGui::TableSetupColumn("Now", ImGuiTableColumnFlags_WidthFixed, em * 6);
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, em * 5);
        ImGui::TableSetupColumn("Warning", ImGuiTableColumnFlags_WidthFixed, em * 6);
        ImGui::TableSetupColumn("Limit", ImGuiTableColumnFlags_WidthFixed, em * 6);
        ImGui::TableSetupColumn("Delay s", ImGuiTableColumnFlags_WidthFixed, em * 5);
        ImGui::TableSetupColumn("Recover", ImGuiTableColumnFlags_WidthFixed, em * 5);
        ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        std::string actions;
        for (int a = 0; a < kActCount; ++a) { actions += protActionName(a); actions.push_back('\0'); }
        for (int i = 0; i < kProtCount; ++i) {
            ProtRule& r = c.prot[i];
            const ProtInfo& in = protInfo(i);
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); bool en = r.enabled != 0; if (ImGui::Checkbox("##en", &en)) r.enabled = en;
            ImGui::TableNextColumn(); ImGui::Text("%s %s", in.name, in.high ? "(>)" : "(<)");
            ImGui::TableNextColumn(); ImGui::Text("%.4g %s", tel_.protValue[i], in.unit);
            ImGui::TableNextColumn(); if (r.enabled) statusChip(tel_.protState[i]); else ImGui::TextDisabled("off");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::DragFloat("##w", &r.warn, (in.hi - in.lo) * 0.002f, in.lo, in.hi, "%.4g");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::DragFloat("##l", &r.limit, (in.hi - in.lo) * 0.002f, in.lo, in.hi, "%.4g");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::DragFloat("##d", &r.delay, 0.02f, 0, 30, "%.2f");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1); ImGui::DragFloat("##r", &r.recover, (in.hi - in.lo) * 0.001f, 0, in.hi, "%.4g");
            ImGui::TableNextColumn(); ImGui::SetNextItemWidth(-1);
            int a = r.action;
            if (ImGui::Combo("##a", &a, actions.c_str())) r.action = a;
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::SeparatorText("Actions");
    ImGui::PushItemWidth(-260);
    ImGui::SliderFloat("Throttle limit / limp throttle", &c.limpThrottle, 5, 100, "%.0f %%");
    Help("Drive-by-wire only. With a cable throttle a throttle limitation becomes an rpm cap (fuel cut).");
    ImGui::SliderFloat("Fuel limit / limp rpm cap", &c.limpRpm, 1500, 8000, "%.0f rpm");
    ImGui::SliderFloat("Ignition retard / limp retard", &c.protRetard, 0, 20, "%.0f deg");
    ImGui::PopItemWidth();
    ImGui::TextDisabled("Boost reduction: boost control back to the wastegate spring (anti-lag off). "
                        "Limp mode = throttle limit + rpm cap + boost reduction + retard. Shutdown: fuel and spark cut until the engine stops.");
}

} // namespace es
