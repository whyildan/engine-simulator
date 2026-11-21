// Fuel Tuning and Ignition Tuning sections: calibration maps / curves / settings of the ECU, the
// measured base map (virtual dyno calibration run in the background) and the physical fuel system.
#include "ui/App.h"
#include "ui/Widgets.h"
#include "imgui.h"
#include "implot.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace es {
using namespace ui;

// ------------------------------------------------------------------ base map job (background thread)
void App::startBaseMap(bool force) {
    stopBaseMap();
    if (isDiesel(p_)) {
        // a diesel fuels by quantity and holds its smoke limit on the air flow meter: the throttle-sweep VE rig
        // does not apply (the VE maps only back up a failed MAF sensor) - nothing to measure
        baseMapInfo_ = "diesel: no base map needed (fuel by quantity, smoke limit on the MAF; VE maps only back up a failed MAF)";
        bmKeyApplied_ = airPathKey(p_); bmVeApplied_ = p_.cal.veMap; bmTpsApplied_ = p_.cal.veTps; bmStale_ = false;
        return;
    }
    if (!baseMapApplies(p_)) {
        // a carburettor meters the fuel itself from the air through its venturi: the VE maps do not fuel it
        baseMapInfo_ = "carburettor: no base map needed (the carburettor meters the fuel; the VE maps are not used to fuel)";
        bmKeyApplied_ = airPathKey(p_); bmVeApplied_ = p_.cal.veMap; bmTpsApplied_ = p_.cal.veTps; bmStale_ = false;
        return;
    }
    if (!force) {
        Calibration c = p_.cal;
        if (loadCachedBaseMap(p_, c)) {
            if (!p_.cal.veMap.locked) std::memcpy(p_.cal.veMap.v, c.veMap.v, sizeof(c.veMap.v));
            if (!p_.cal.veTps.locked) std::memcpy(p_.cal.veTps.v, c.veTps.v, sizeof(c.veTps.v));
            baseMapInfo_ = "measured base map (from cache)";
            bmKeyApplied_ = airPathKey(p_); bmVeApplied_ = p_.cal.veMap; bmTpsApplied_ = p_.cal.veTps; bmStale_ = false;
            return;
        }
    }
    bmFor_ = p_;
    bmProg_.fraction = 0; bmProg_.cancel = false;
    bmDone_ = false; bmFailed_ = false; bmRunning_ = true;
    bmVeBefore_ = p_.cal.veMap; bmTpsBefore_ = p_.cal.veTps;
    bmThread_ = std::thread([this] {
        Calibration c = bmFor_.cal;
        bool ok = measureBaseMap(bmFor_, c, &bmProg_, 48000.0 * std::max(1, (int)bmFor_.oversample));
        if (ok) { bmResult_ = c; saveCachedBaseMap(bmFor_, c); }
        else if (bmProg_.cancel) return;   // (cancelled: stopBaseMap() ends the run)
        // done either way: a run that could not measure must not leave the UI "measuring..." for ever
        bmFailed_ = !ok;
        bmDone_ = true;
    });
}

void App::stopBaseMap() {
    if (bmThread_.joinable()) { bmProg_.cancel = true; bmThread_.join(); }
    bmRunning_ = false;
}

void App::pollBaseMap() {
    if (!bmRunning_ || !bmDone_) return;
    if (bmThread_.joinable()) bmThread_.join();
    bmRunning_ = false;
    if (bmFailed_) { baseMapInfo_ = "base map could not be measured for this engine (generic VE maps kept)"; return; }
    // only take it if the tuner did not edit the VE maps meanwhile and it is still the same engine
    bool same = airPathKey(bmFor_) == airPathKey(p_);
    if (same && std::memcmp(&p_.cal.veMap.v, &bmVeBefore_.v, sizeof(bmVeBefore_.v)) == 0 && !p_.cal.veMap.locked)
        std::memcpy(p_.cal.veMap.v, bmResult_.veMap.v, sizeof(bmResult_.veMap.v));
    if (same && std::memcmp(&p_.cal.veTps.v, &bmTpsBefore_.v, sizeof(bmTpsBefore_.v)) == 0 && !p_.cal.veTps.locked)
        std::memcpy(p_.cal.veTps.v, bmResult_.veTps.v, sizeof(bmResult_.veTps.v));
    if (same && calMeta_.name.empty()) calSaved_ = p_.cal;   // an unsaved base calibration is not "modified" by its own base map
    baseMapInfo_ = same ? "measured base map applied" : "base map discarded (engine changed meanwhile)";
    if (same) { bmKeyApplied_ = airPathKey(p_); bmVeApplied_ = p_.cal.veMap; bmTpsApplied_ = p_.cal.veTps; bmStale_ = false; }
}

// the hardware that decides the trapped air (ports, cams, pipes, displacement ...), not the calibration
uint64_t App::airPathKey(const EngineParams& in) {
    EngineParams p = in;
    std::memset(&p.cal, 0, sizeof(p.cal));
    return baseMapKey(p);
}

// engine hardware edited (e.g. rotary porting, cam, runners): once the edits have settled, measure a new base
// map - unless the tuner has worked on the VE maps since the last one (then only say that it is out of date)
void App::baseMapFollowEngine(float dt) {
    const uint64_t k = airPathKey(p_);
    if (k != bmKeySeen_) { bmKeySeen_ = k; bmKeyStable_ = 0; return; }
    bmKeyStable_ += dt;
    if (bmKeyApplied_ == 0 || bmKeyStable_ < 1.0f) return;
    if (k == bmKeyApplied_) { bmStale_ = false; return; }
    if (bmRunning_) {
        if (airPathKey(bmFor_) != k) startBaseMap();   // measuring an engine that is no longer this one
        return;
    }
    const bool untouched = std::memcmp(p_.cal.veMap.v, bmVeApplied_.v, sizeof(bmVeApplied_.v)) == 0 &&
                           std::memcmp(p_.cal.veTps.v, bmTpsApplied_.v, sizeof(bmTpsApplied_.v)) == 0;
    if (untouched) { baseMapInfo_ = "engine hardware changed: measuring a new base map"; startBaseMap(); }
    else bmStale_ = true;
}

// operating point trail (what the ECU looks up), sampled ~30 times a second
void App::recordOpTrail(float dt) {
    trailAcc_ += dt;
    if (trailAcc_ < 0.033f) return;
    trailAcc_ = 0;
    OpPoint a; a.x = tel_.ecuLoadX; a.y = tel_.ecuLoadY; a.valid = tel_.running && tel_.ecuLoadX > 0;
    OpPoint b = a; b.y = tel_.ecuTps;
    trailMap_.push_back(a); trailTps_.push_back(b);
    if (trailMap_.size() > 150) trailMap_.erase(trailMap_.begin());
    if (trailTps_.size() > 150) trailTps_.erase(trailTps_.begin());
}

static void label(const char* k, const char* v) {
    ImGui::TextDisabled("%s", k);
    ImGui::SameLine(ImGui::GetFontSize() * 15.0f);
    ImGui::TextUnformatted(v);
}

void App::sectionFuelTuning() {
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    OpPoint opMap; opMap.x = tel_.ecuLoadX; opMap.y = tel_.ecuLoadY; opMap.valid = tel_.running && efi;
    OpPoint opTps = opMap; opTps.y = tel_.ecuTps;
    if (isDiesel(p_)) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Diesel: these petrol fuel maps are not used - the diesel's fuel demand, injection timing, smoke limit and EGR maps are in Engine Control > Diesel.");
    else if (!efi) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "Carburettor: the fuel maps are not used (mixture comes from jets / choke). Switch to EFI in the Fuel system tab.");
    if (ImGui::BeginTabBar("fueltabs", ImGuiTabBarFlags_FittingPolicyScroll)) {
        if (ImGui::BeginTabItem("Setup")) {
            ImGui::PushItemWidth(-260);
            ImGui::SeparatorText("Load calculation");
            int lm = (int)c.loadMode;
            if (ImGui::Combo("Load model", &lm, "Speed density (MAP x VE)\0Alpha-N (throttle angle)\0MAF (air flow meter)\0Hybrid (speed density -> alpha-N)\0"))
                c.loadMode = (LoadMode)lm;
            Help("Speed density: air from manifold pressure x the VE map (most common).\n"
                 "Alpha-N: air from throttle angle and rpm (ITB / race engines with a weak MAP signal).\n"
                 "MAF: air measured by the flow meter, divided over the cylinder events (no VE map needed,\n"
                 "but the meter sees the throttle flow, not the cylinder filling - transients differ).\n"
                 "Hybrid: speed density at small throttle, blending into alpha-N as the throttle opens.");
            if (c.loadMode == LoadMode::Hybrid) {
                ImGui::SliderFloat("Blend from TPS", &c.hybridTpsLo, 0, 50, "%.0f %%");
                ImGui::SliderFloat("Blend to TPS", &c.hybridTpsHi, c.hybridTpsLo + 1, 100, "%.0f %%");
            }
            ImGui::SeparatorText("Feedback");
            bool at = c.autoTune != 0, cl = c.closedLoop != 0;
            if (ImGui::Checkbox("Auto-tune VE from the wideband", &at)) c.autoTune = at;
            Help("While the engine runs warm and steady, the ECU compares the wideband with the target lambda\n"
                 "(after the exhaust transport delay) and corrects the VE cells it was using. Locked maps are left alone.\n"
                 "With auto-tune on it also acts as the closed loop (the short-term trim stays at 0 %).");
            if (ImGui::Checkbox("Closed-loop trim (short-term)", &cl)) c.closedLoop = cl;
            Help("Short-term fuel trim on the wideband at part load (target leaner than AFR 13.5), warm engine.");
            char b[96];
            std::snprintf(b, sizeof(b), "%s   trim %+.1f %%", tel_.tuning ? "auto-tune LEARNING" : tel_.closedLoop ? "closed loop" : "open loop", (tel_.ecuTrim - 1) * 100);
            label("Now", b);
            ImGui::SeparatorText("Base map");
            ImGui::TextWrapped("A measured base map is a calibration run on a virtual motoring dyno: a copy of this engine is held "
                               "at fixed speeds while the throttle is swept, and the VE maps are set so the measured lambda hits the "
                               "target (like a calibrator with a wideband). It runs automatically when an engine is loaded.");
            if (bmRunning_) {
                ImGui::ProgressBar(bmProg_.fraction.load(), ImVec2(-80, 0), "measuring...");
                ImGui::SameLine();
                if (ImGui::Button("Cancel")) stopBaseMap();
            } else if (ImGui::Button("Measure base map (virtual dyno)")) startBaseMap(true);
            if (!baseMapInfo_.empty()) { ImGui::SameLine(); ImGui::TextDisabled("%s", baseMapInfo_.c_str()); }
            if (bmStale_) ImGui::TextColored(ImVec4(1, 0.75f, 0.35f, 1), "The engine hardware changed since this base map was measured, and the VE maps "
                                                                         "have been edited / learned since - press Measure to update them (replaces the VE maps).");
            if (ImGui::Button("Reset all maps to the generic base calibration")) {
                Calibration keep = c;
                generateCalibration(p_);
                p_.cal.autoTune = keep.autoTune; p_.cal.closedLoop = keep.closedLoop;
            }
            Help("Generic VE guess, lambda from the AFR targets below, ignition from the distributor curve in Ignition Tuning.");
            ImGui::SeparatorText("Lambda table generator");
            ImGui::SliderFloat("AFR cruise", &p_.afrCruise, 11, 17, "%.1f");
            ImGui::SliderFloat("AFR full load", &p_.afrWot, 10, 15, "%.1f");
            if (isBoosted(p_)) ImGui::SliderFloat("AFR at 1 bar boost", &p_.afrBoost, 10, 13.5f, "%.1f");
            if (ImGui::Button("Rebuild lambda table from these")) {
                EngineParams q = p_; generateCalibration(q);
                c.lambda = q.cal.lambda; c.powerLambda = q.cal.powerLambda;
            }
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("VE map")) {
            ImGui::TextDisabled("Volumetric efficiency %% over rpm x MAP. %s", c.loadMode == LoadMode::SpeedDensity || c.loadMode == LoadMode::Hybrid ? "Active." : "Not used by the current load model.");
            if (tabEd_[kCalVeMap].draw("vemap", c.veMap, calTableInfo(kCalVeMap), opMap, &trailMap_, ref ? &ref->veMap : nullptr, refName_.c_str())) {}
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("VE alpha-N")) {
            ImGui::TextDisabled("Cylinder filling %% of an atmospheric cylinder over rpm x throttle. %s",
                                c.loadMode == LoadMode::AlphaN || c.loadMode == LoadMode::Hybrid ? "Active." : "Used as limp-home map if the MAP sensor fails.");
            tabEd_[kCalVeTps].draw("vetps", c.veTps, calTableInfo(kCalVeTps), opTps, &trailTps_, ref ? &ref->veTps : nullptr, refName_.c_str());
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Lambda")) {
            ImGui::TextDisabled("Target lambda over rpm x MAP (AFR = lambda x 14.7). Leaner than 0.92 runs in closed loop.");
            tabEd_[kCalLambda].draw("lambda", c.lambda, calTableInfo(kCalLambda), opMap, &trailMap_, ref ? &ref->lambda : nullptr, refName_.c_str());
            ImGui::PushItemWidth(160);
            ImGui::SeparatorText("Power enrichment (by throttle angle)");
            ImGui::SliderFloat("from TPS", &c.powerTpsLo, 0, 100, "%.0f %%"); ImGui::SameLine();
            ImGui::SliderFloat("to TPS", &c.powerTpsHi, 0, 100, "%.0f %%"); ImGui::SameLine();
            ImGui::SliderFloat("lambda", &c.powerLambda, 0.65f, 1.0f, "%.3f");
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Enrichment")) {
            const float ect = tel_.snsValue[kSnsECT];
            ImGui::SeparatorText("Cranking AFR (by coolant)");
            curveEd_[kCalCrankAfr].drawCurve("crank", c.crankAfr, calCurveInfo(kCalCrankAfr), ect, true, ref ? &ref->crankAfr : nullptr);
            ImGui::SeparatorText("Warm-up enrichment (by coolant)");
            curveEd_[kCalWarmup].drawCurve("warm", c.warmup, calCurveInfo(kCalWarmup), ect, true, ref ? &ref->warmup : nullptr);
            ImGui::SeparatorText("Acceleration enrichment (by throttle opening rate)");
            curveEd_[kCalAccel].drawCurve("accel", c.accel, calCurveInfo(kCalAccel), 0, false, ref ? &ref->accel : nullptr);
            ImGui::PushItemWidth(170);
            ImGui::SliderFloat("AE decay", &c.accelDecay, 0.05f, 1.5f, "%.2f s"); ImGui::SameLine();
            ImGui::SliderFloat("AE cold multiplier", &c.accelColdMult, 1, 4, "x%.2f");
            ImGui::SeparatorText("After start / decel fuel cut exit / tip-in");
            ImGui::SliderFloat("After-start AFR", &c.afterStartAfr, 9, 14.7f, "%.1f"); ImGui::SameLine();
            ImGui::SliderFloat("fading over", &c.afterStartTime, 0.2f, 10, "%.1f s");
            ImGui::SliderFloat("DFCO exit enrichment", &c.dfcoExitEnrich, 0, 150, "%.0f %%"); ImGui::SameLine();
            ImGui::SliderFloat("decay", &c.dfcoExitDecay, 0.05f, 1.5f, "%.2f s");
            Help("Extra fuel when injection resumes after deceleration fuel cut: the port walls dried out during\nthe cut and soak up fuel again. Scaled by how long the cut lasted.");
            bool as = c.asyncTipIn != 0;
            if (ImGui::Checkbox("Asynchronous tip-in injection", &as)) c.asyncTipIn = as;
            Help("When the load jumps, ports that were already fuelled for the lower load (but have not inducted\nyet) get the missing fuel at once instead of firing lean on the first cycle.");
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Corrections")) {
            ImGui::SeparatorText("Intake air temperature fuel correction");
            curveEd_[kCalIatFuel].drawCurve("iatf", c.iatFuel, calCurveInfo(kCalIatFuel), tel_.snsValue[kSnsIAT], true, ref ? &ref->iatFuel : nullptr);
            ImGui::SeparatorText("Per-cylinder fuel trim");
            int n = isRotary(p_) ? p_.cylinders : p_.cylinders;
            ImGui::PushItemWidth(90);
            for (int i = 0; i < n; ++i) {
                char lb[24]; std::snprintf(lb, sizeof(lb), "%s %d##cf%d", isRotary(p_) ? "Rotor" : "Cyl", i + 1, i);
                ImGui::SliderFloat(lb, &c.cylFuel[i], -20, 20, "%+.1f%%");
                if ((i + 1) % 6 && i + 1 < n) ImGui::SameLine();
            }
            ImGui::PopItemWidth();
            ImGui::TextDisabled("Measured lambda per cylinder (last cycle):");
            for (int i = 0; i < std::min(tel_.cylinders, 16); ++i) { ImGui::SameLine(); ImGui::Text("%.2f", tel_.cylLambda[i]); }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Injectors")) {
            ImGui::PushItemWidth(-260);
            ImGui::SliderFloat("Injector flow (ECU calibration)", &c.injFlow, 60, 2400, "%.0f cc/min @ 3 bar");
            Help("What the ECU believes the injectors flow. Must match the fitted injectors (Engine Setup >\n"
                 "Components) - a mismatch shifts every cell by the same percentage.");
            char b[96]; std::snprintf(b, sizeof(b), "%.0f cc/min", p_.injector_ccmin);
            label("Fitted injectors", b);
            ImGui::SameLine();
            if (ImGui::SmallButton("Copy fitted value")) c.injFlow = p_.injector_ccmin;
            ImGui::PopItemWidth();
            ImGui::SeparatorText("Dead time over battery voltage");
            curveEd_[kCalDeadTime].drawCurve("dead", c.deadTime, calCurveInfo(kCalDeadTime), tel_.snsValue[kSnsBattery], true, ref ? &ref->deadTime : nullptr);
            std::snprintf(b, sizeof(b), "%.2f ms at 14 V", p_.comp.injDeadTime_ms);
            label("Fitted injector dead time", b);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Closed loop")) { tabFuelTrims(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Flex fuel")) {
            ImGui::PushItemWidth(-260);
            ImGui::TextDisabled("How the ECU adapts to the fuel: alcohol content -> stoich AFR, injector mass, spark, cold enrichment.");
            char b[160];
            std::snprintf(b, sizeof(b), "%.1f %% %s", tel_.ecuFlexPct, tel_.ecuFlexLive ? "(flex fuel sensor)" : p_.flexSensor ? "(sensor failed: fallback)" : "(no sensor: fallback)");
            label("Alcohol content used", b);
            std::snprintf(b, sizeof(b), "%.2f   (fuel in the line: %.2f)", tel_.ecuStoich, tel_.fuelStoich);
            label("Stoich AFR used", b);
            std::snprintf(b, sizeof(b), "%.3f   measured %.3f", tel_.ecuTargetLambda, tel_.snsValue[kSnsWideband]);
            label("Target lambda", b);
            std::snprintf(b, sizeof(b), "+%.0f %% when fully cold", tel_.ecuFlexCold * 100);
            label("Cold enrichment now", b);
            ImGui::SeparatorText("Fuel definition");
            ImGui::SliderFloat("Stoich AFR, gasoline", &c.stoichGasoline, 12, 15.5f, "%.2f");
            ImGui::SliderFloat("Stoich AFR, alcohol", &c.stoichAlcohol, 5, 10, "%.2f");
            Help("The two fuels the ECU blends by the measured alcohol content (by mass). Ethanol 9.0, methanol 6.45 -\n"
                 "a flex fuel sensor cannot tell them apart, so a methanol engine needs 6.45 here.");
            ImGui::SliderFloat("Fallback alcohol content", &c.flexFallbackPct, 0, 100, "%.0f %%");
            Help("Used without a flex fuel sensor, or when it fails (no signal). Must match the fuel in the tank.");
            ImGui::PopItemWidth();
            const float x = tel_.ecuFlexPct;
            ImGui::SeparatorText("Fuel trim over alcohol content");
            curveEd_[kCalFlexFuel].drawCurve("flexfuel", c.flexFuel, calCurveInfo(kCalFlexFuel), x, tel_.running, ref ? &ref->flexFuel : nullptr);
            ImGui::SeparatorText("Spark added over alcohol content");
            curveEd_[kCalFlexSpark].drawCurve("flexspark", c.flexSpark, calCurveInfo(kCalFlexSpark), x, tel_.running, ref ? &ref->flexSpark : nullptr);
            ImGui::SeparatorText("Cold enrichment over alcohol content (at full cold, fades as the coolant warms)");
            curveEd_[kCalFlexCold].drawCurve("flexcold", c.flexCold, calCurveInfo(kCalFlexCold), x, tel_.running, ref ? &ref->flexCold : nullptr);
            ImGui::SeparatorText("Knock threshold multiplier over alcohol content");
            curveEd_[kCalFlexKnock].drawCurve("flexknock", c.flexKnock, calCurveInfo(kCalFlexKnock), x, tel_.running, ref ? &ref->flexKnock : nullptr);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Fuel system", nullptr, tabJump("fuel_sys"))) { panelFuelIntake(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Measured")) { panelFuelMap(); ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }
}

void App::sectionIgnitionTuning() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    OpPoint opMap; opMap.x = tel_.ecuLoadX > 0 ? tel_.ecuLoadX : tel_.rpm; opMap.y = tel_.ecuMap; opMap.valid = tel_.running;
    if (ImGui::BeginTabBar("igntabs")) {
        if (ImGui::BeginTabItem("Ignition map")) {
            ImGui::TextDisabled("Spark advance (deg BTDC) over rpm x MAP. Idle spark control, anti-lag and per-cylinder trims are added on top.");
            tabEd_[kCalIgnition].draw("ign", c.ignition, calTableInfo(kCalIgnition), opMap, &trailMap_, ref ? &ref->ignition : nullptr, refName_.c_str());
            ImGui::Text("Advance now %.1f deg   knock index %.2f %s", tel_.advance, tel_.knock, tel_.knock > 1 ? "  <- KNOCKING: pull timing in this area" : "");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Corrections")) {
            ImGui::PushItemWidth(160);
            ImGui::SliderFloat("Cranking advance", &c.crankAdvance, -10, 25, "%.1f deg");
            ImGui::PopItemWidth();
            ImGui::SeparatorText("Intake air temperature spark correction");
            curveEd_[kCalIatSpark].drawCurve("iats", c.iatSpark, calCurveInfo(kCalIatSpark), tel_.snsValue[kSnsIAT], true, ref ? &ref->iatSpark : nullptr);
            ImGui::SeparatorText("Coolant temperature spark correction");
            curveEd_[kCalEctSpark].drawCurve("ects", c.ectSpark, calCurveInfo(kCalEctSpark), tel_.snsValue[kSnsECT], true, ref ? &ref->ectSpark : nullptr);
            ImGui::SeparatorText("Per-cylinder spark trim");
            ImGui::PushItemWidth(90);
            for (int i = 0; i < p_.cylinders; ++i) {
                char lb[24]; std::snprintf(lb, sizeof(lb), "%s %d##cs%d", isRotary(p_) ? "Rotor" : "Cyl", i + 1, i);
                ImGui::SliderFloat(lb, &c.cylSpark[i], -10, 10, "%+.1f");
                if ((i + 1) % 6 && i + 1 < p_.cylinders) ImGui::SameLine();
            }
            ImGui::PopItemWidth();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Knock control", nullptr, tabJump("knock"))) { tabKnock(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Limiter & base curve")) {
            panelIgnition();
            if (ImGui::Button("Rebuild ignition map from this curve")) {
                EngineParams q = p_; generateCalibration(q);
                c.ignition = q.cal.ignition; c.crankAdvance = q.cal.crankAdvance;
            }
            Help("The sliders above describe a classic distributor curve (base + centrifugal by rpm + vacuum by\nMAP - boost retard). They only feed this generator; the ECU runs on the ignition map.");
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Cylinder pressure")) { panelScope(); ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }
}

} // namespace es
