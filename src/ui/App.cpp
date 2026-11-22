#include "ui/App.h"
#include "imgui.h"
#include "implot.h"
#include "audio/ImpulseResponses.h"
#include <windows.h>
#include <commdlg.h>
#include <cstdio>
#include <cstring>

namespace es {

App::App(int preset) {
    presetIdx_ = clampv(preset, 0, presetCount() - 1);
    p_ = makePreset(presetIdx_);
    p_.ignition = 0;
    sent_ = p_;
    sim_ = std::make_unique<Simulator>(p_);
    sim_->startThread();
    audio_.start(sim_.get());
    startBaseMap();
    calOnPresetChanged();
}

App::~App() {
    stopBaseMap();
    audio_.stop();
    sim_->stopThread();
}

// Win32 open dialog -> load + resample the WAV -> hand it to the simulation thread
bool App::loadIrFile() {
    wchar_t file[MAX_PATH] = L"";
    OPENFILENAMEW ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = GetActiveWindow();
    ofn.lpstrFilter = L"WAV impulse responses (*.wav)\0*.wav\0All files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = L"Load exhaust impulse response";
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (!GetOpenFileNameW(&ofn)) return false;
    std::vector<float> ir;
    std::string err;
    if (!loadWavIR(file, Simulator::kOutRate, ir, err)) { irError_ = err; return false; }
    sim_->setCustomIR(ir);
    std::wstring w(file);
    size_t slash = w.find_last_of(L"\\/");
    std::wstring base = slash == std::wstring::npos ? w : w.substr(slash + 1);
    irFileName_.assign(base.begin(), base.end());   // display only (ASCII names)
    char info[64];
    std::snprintf(info, sizeof(info), "  (%.0f ms)", ir.size() * 1000.0 / Simulator::kOutRate);
    irFileName_ += info;
    irError_.clear();
    p_.irChoice = IrCustom;
    return true;
}

void App::resetPeak() {
    std::fill(std::begin(peakTq_), std::end(peakTq_), 0.0f);
    std::fill(std::begin(peakPw_), std::end(peakPw_), 0.0f);
    liveRpm_.clear(); liveTq_.clear(); livePw_.clear(); liveAge_.clear();
}

namespace ui { extern bool g_tableKeysCaptured; }

void App::keyboard(float dt) {
    ImGuiIO& io = ImGui::GetIO();
    const bool captured = ui::g_tableKeysCaptured;
    ui::g_tableKeysCaptured = false;   // the editors set it again while they have focus
    if (io.WantTextInput || captured) return;
    bool up = ImGui::IsKeyDown(ImGuiKey_W) || ImGui::IsKeyDown(ImGuiKey_UpArrow);
    bool wot = ImGui::IsKeyDown(ImGuiKey_Space);
    if (wot) { p_.throttle = 1; keyThrottle_ = true; }
    else if (up) { p_.throttle = std::min(1.0f, p_.throttle + dt * 2.5f); keyThrottle_ = true; }
    else if (keyThrottle_) {
        p_.throttle = std::max(0.0f, p_.throttle - dt * 4.0f);
        if (p_.throttle <= 0) keyThrottle_ = false;
    }
    // brake: B or Down (also lifts off the throttle)
    bool brakeKey = ImGui::IsKeyDown(ImGuiKey_B) || ImGui::IsKeyDown(ImGuiKey_DownArrow);
    if (brakeKey) { p_.throttle = 0; keyThrottle_ = false; p_.brake = std::min(1.0f, p_.brake + dt * 6.0f); brakeKey_ = true; }
    else if (brakeKey_) { p_.brake = std::max(0.0f, p_.brake - dt * 8.0f); if (p_.brake <= 0) brakeKey_ = false; }
    // clutch: hold C = pedal down at once; releasing lets it out over ~0.4 s
    if (ImGui::IsKeyDown(ImGuiKey_C)) { p_.clutchPedal = 1.0f; clutchKey_ = true; }
    else if (clutchKey_) { p_.clutchPedal = std::max(0.0f, p_.clutchPedal - dt * 2.5f); if (p_.clutchPedal <= 0) clutchKey_ = false; }
    // gears: E up, Q down (automatic: E = Drive, Q = Neutral), N neutral, V toggles driving mode
    if (ImGui::IsKeyPressed(ImGuiKey_V, false)) p_.vehicle = !p_.vehicle;
    if (p_.transMode == TransMode::Automatic) {
        if (ImGui::IsKeyPressed(ImGuiKey_E, false)) p_.gear = 1;
        if (ImGui::IsKeyPressed(ImGuiKey_Q, false) || ImGui::IsKeyPressed(ImGuiKey_N, false)) p_.gear = 0;
    } else {
        if (ImGui::IsKeyPressed(ImGuiKey_E, false)) p_.gear = std::min((int)p_.gears, (int)p_.gear + 1);
        if (ImGui::IsKeyPressed(ImGuiKey_Q, false)) p_.gear = std::max(0, (int)p_.gear - 1);
        if (ImGui::IsKeyPressed(ImGuiKey_N, false)) p_.gear = 0;
    }
    for (int k = 0; k <= 9; ++k)
        if (ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_0 + k), false)) { p_.throttle = k == 0 ? 1.0f : k * 0.1f; keyThrottle_ = false; }
    if (ImGui::IsKeyPressed(ImGuiKey_I, false)) p_.ignition = !p_.ignition;
    if (ImGui::IsKeyPressed(ImGuiKey_S, false)) { p_.ignition = 1; }
    static bool starterKey = false;
    bool sDown = ImGui::IsKeyDown(ImGuiKey_S);
    if (sDown) { p_.starter = 1; starterKey = true; }
    else if (starterKey) { p_.starter = 0; starterKey = false; }
    if (ImGui::IsKeyPressed(ImGuiKey_D, false)) { if (p_.dynoMode != DynoMode::Step) p_.dynoMode = DynoMode::Sweep; p_.sweepRequest++; }
    if (ImGui::IsKeyPressed(ImGuiKey_R, false)) { if (logRec_ >= 0) logStop(); else logStart(); }
    if (ImGui::IsKeyPressed(ImGuiKey_M, false)) logMarker("marker");
    if (ImGui::IsKeyPressed(ImGuiKey_L, false)) { p_.listener = (ListenerPos)(((int)p_.listener + 1) % 3); }
    // motorsport switches: T transbrake (hold), G rolling anti-lag button (hold), X nitrous arming switch (toggle)
    static bool tbKey = false, alsKey = false;
    if (ImGui::IsKeyDown(ImGuiKey_T)) { p_.transbrake = 1; tbKey = true; }
    else if (tbKey) { p_.transbrake = 0; tbKey = false; }
    if (ImGui::IsKeyDown(ImGuiKey_G)) { p_.rollingAlsButton = 1; alsKey = true; }
    else if (alsKey) { p_.rollingAlsButton = 0; alsKey = false; }
    if (ImGui::IsKeyPressed(ImGuiKey_X, false)) p_.n2oArm = !p_.n2oArm;
}

void App::pullDyno() {
    std::vector<DynoPoint> pts;
    int runId = 0;
    sim_->takeDynoPoints(pts, runId);
    if (pts.empty()) return;
    if (runId != lastRunId_ || runs_.empty()) {
        lastRunId_ = runId;
        static const float palette[][3] = {{0.95f, 0.45f, 0.2f}, {0.3f, 0.65f, 1.0f}, {0.4f, 0.85f, 0.4f}, {0.9f, 0.3f, 0.7f},
                                           {0.95f, 0.85f, 0.3f}, {0.6f, 0.5f, 1.0f}, {0.3f, 0.9f, 0.85f}, {1.0f, 0.5f, 0.5f}};
        DynoRun r;
        char buf[160];
        const char* ms[] = {"no muffler", "straight-through", "chambered", "turbo", "glasspack", "reverse-flow", "valved"};
        if (isRotary(p_))
            std::snprintf(buf, sizeof(buf), "#%d %s | %d-rotor %.1fL CR%.1f | IO %.0f IC %.0f | %s%s", (int)runs_.size() + 1, p_.name,
                          p_.cylinders, ratedDisplacementL(p_), p_.compression, p_.rIntakeOpen, p_.rIntakeClose,
                          p_.fuelSystem == FuelSystem::EFI ? "EFI " : "carb ", ms[(int)p_.muffler]);
        else
            std::snprintf(buf, sizeof(buf), "#%d %s | %.1fL CR%.1f | cam %.0f/%.0f | %s%s", (int)runs_.size() + 1, p_.name,
                          ratedDisplacementL(p_), p_.compression,
                          p_.intakeDuration, p_.exhaustDuration, p_.fuelSystem == FuelSystem::EFI ? "EFI " : "carb ",
                          ms[(int)p_.muffler]);
        if (isBoosted(p_)) {
            char b2[64];
            const char* sct[] = {"Roots", "screw", "centrifugal"};
            if (p_.induction == Induction::Turbo) std::snprintf(b2, sizeof(b2), " | turbo %.1f bar", p_.wastegateBar);
            else std::snprintf(b2, sizeof(b2), " | %s SC x%.1f", sct[(int)p_.scType], p_.scRatio);
            std::strncat(buf, b2, sizeof(buf) - std::strlen(buf) - 1);
        }
        r.label = buf;
        r.engine = p_.name;
        r.calName = calMeta_.name.empty() ? "(unsaved)" : calMeta_.name;
        r.calRev = calMeta_.revision;
        r.label += " | cal " + r.calName + " r" + std::to_string(r.calRev);
        if (p_.dynoMode == DynoMode::Step) r.label += " | step test";
        r.ambientC = p_.ambientTemp_C; r.baroKPa = (float)(ambientPressurePa(p_) * 1e-3);
        if (r.baroKPa < 100.0f) { char b3[48]; std::snprintf(b3, sizeof(b3), " | %.0f m", altitudeOfBaro(r.baroKPa)); r.label += b3; }
        const float* c = palette[runs_.size() % 8];
        r.color[0] = c[0]; r.color[1] = c[1]; r.color[2] = c[2]; r.color[3] = 1;
        runs_.push_back(r);
    }
    DynoRun& r = runs_.back();
    for (const DynoPoint& d : pts) {
        r.pts.push_back(d);
        float hp = d.powerKw * 1.341f;
        if (d.torque > r.peakTorque) { r.peakTorque = d.torque; r.peakTorqueRpm = d.rpm; }
        if (hp > r.peakPower) { r.peakPower = hp; r.peakPowerRpm = d.rpm; }
    }
}

void App::frame(float dt) {
    keyboard(dt);
    sim_->getTelemetry(tel_);
    // auto-tune learns inside the ECU: take its VE maps into our copy (and mark them as already sent)
    if (tel_.learnSeq != learnSeen_ && tel_.calVeMap.nx > 0) {
        learnSeen_ = tel_.learnSeq;
        if (std::memcmp(&p_.cal.veMap, &sent_.cal.veMap, sizeof(Table2D)) == 0) { p_.cal.veMap.v[0][0] = 0; std::memcpy(p_.cal.veMap.v, tel_.calVeMap.v, sizeof(tel_.calVeMap.v)); sent_.cal.veMap = p_.cal.veMap; }
        if (std::memcmp(&p_.cal.veTps, &sent_.cal.veTps, sizeof(Table2D)) == 0) { std::memcpy(p_.cal.veTps.v, tel_.calVeTps.v, sizeof(tel_.calVeTps.v)); sent_.cal.veTps = p_.cal.veTps; }
    }
    // learned long-term fuel trims, the same way
    if (tel_.ltftSeq != ltftSeen_ && tel_.calLtft.nx > 0) {
        ltftSeen_ = tel_.ltftSeq;
        if (std::memcmp(&p_.cal.ltft, &sent_.cal.ltft, sizeof(Table2D)) == 0 && tel_.calLtft.nx == p_.cal.ltft.nx && tel_.calLtft.ny == p_.cal.ltft.ny) {
            std::memcpy(p_.cal.ltft.v, tel_.calLtft.v, sizeof(tel_.calLtft.v)); sent_.cal.ltft = p_.cal.ltft;
        }
    }
    pullDyno();
    recordScope(dt);
    recordOpTrail(dt);
    pollBaseMap();
    baseMapFollowEngine(dt);
    logFrame();

    // auto start: crank until the engine runs on its own
    if (autoStart_) {
        autoStartTimer_ += dt;
        p_.ignition = 1;
        p_.starter = 1;
        caughtTimer_ = tel_.running && tel_.rpm > starterReleaseRpm(p_, tel_.snsValue[kSnsECT]) ? caughtTimer_ + dt : 0.0f;
        if (caughtTimer_ > 0.15f || autoStartTimer_ > 8.0f) { autoStart_ = false; p_.starter = 0; caughtTimer_ = 0; }
    }
    // slow-motion animation angle
    if (tel_.rpm < 120) animAngle_ = tel_.crankDeg;
    else animAngle_ = std::fmod(animAngle_ + tel_.rpm / 60.0f * 360.0f * dt * animSpeed_, (float)cycleDeg(p_));

    shell();
    newEngineUi();
    if (showDemo_) ImPlot::ShowDemoWindow(&showDemo_);

    // publish changed parameters to the simulation thread
    if (std::memcmp(&p_, &sent_, sizeof(EngineParams)) != 0) {
        sim_->setParams(p_);
        sent_ = p_;
    }
}

} // namespace es
