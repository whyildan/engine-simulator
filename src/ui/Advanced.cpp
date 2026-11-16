// Phase 6 UI: compressor / turbine maps (model choice, built-ins, CSV, map plot with efficiency islands),
// valvetrain dynamics (dynamic float speed, valve motion plot), altitude.
#include "ui/App.h"
#include "ui/Widgets.h"
#include "ui/FileDialog.h"
#include "imgui.h"
#include "implot.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace es {
using namespace ui;

namespace {
uint64_t fnv(const void* d, size_t n, uint64_t h = 1469598103934665603ull) {
    const unsigned char* b = static_cast<const unsigned char*>(d);
    for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
    return h;
}
double genericTurbineArea(const EngineParams& p) {
    return std::max(0.2f, p.turbineAR) * 1.0e-3 * std::sqrt(std::max(0.3, equivalentDisplacementL(p) / 2.0));
}
} // namespace

// ------------------------------------------------------------------ altitude (ambient pressure)
void App::altitudeUi(const char* label) {
    const float baro = p_.baro_kPa > 30 ? p_.baro_kPa : 101.325f;
    float alt = (float)std::max(0.0, altitudeOfBaro(baro));
    char fmt[48];
    std::snprintf(fmt, sizeof(fmt), "%%.0f m  (%.1f kPa)", baro);
    if (ImGui::SliderFloat(label, &alt, 0, 4500, fmt)) p_.baro_kPa = (float)baroAtAltitude(alt);
    Help("Air pressure falls ~1.2 % per 100 m (standard atmosphere): a naturally aspirated engine loses\n"
         "power with the air density, a turbo spins faster for the same boost and makes most of it back.\n"
         "The ECU reads the barometric pressure (alpha-N, boost gauge, injector pressure).");
}

// ------------------------------------------------------------------ turbo maps
void App::turboModelUi() {
    TurboMaps& tm = p_.turbo;
    const bool turbo = p_.induction == Induction::Turbo;
    int model = tm.useMaps ? 1 : 0;
    ImGui::RadioButton("Generic (size sliders)", &model, 0); ImGui::SameLine();
    ImGui::RadioButton("Compressor / turbine maps", &model, 1);
    Help("Generic: a simple compressor shape scaled by the size slider, constant turbine efficiency.\n"
         "Maps: speed lines with pressure ratio and efficiency (and a turbine flow / efficiency map),\n"
         "built in or loaded from CSV - surge, choke and efficiency islands come from the table.");
    if (model != (tm.useMaps ? 1 : 0)) {
        if (model == 1 && tm.comp.nLines < 2) {
            tm.comp = genericCompressorMap(p_.compFlow_gs, turbo ? 190000.0 : 60000.0);
            tm.compChoice = -1;
            std::snprintf(tm.compName, sizeof(tm.compName), "From size slider (%.0f g/s)", p_.compFlow_gs);
        }
        if (model == 1 && turbo && tm.turb.nFlow < 2) {
            tm.turb = genericTurbineMap(genericTurbineArea(p_));
            tm.turbChoice = -1;
            std::snprintf(tm.turbName, sizeof(tm.turbName), "From A/R %.2f", p_.turbineAR);
        }
        tm.useMaps = model;
    }
    if (!tm.useMaps) return;

    // ---- compressor
    {
        std::string items;
        for (int i = 0; i < builtinCompressorCount(); ++i) { items += builtinCompressorName(i); items.push_back('\0'); }
        items += tm.compChoice < 0 && tm.compName[0] ? tm.compName : "(custom)"; items.push_back('\0');
        int sel = tm.compChoice >= 0 ? tm.compChoice : builtinCompressorCount();
        if (ImGui::Combo("Compressor map", &sel, items.c_str()) && sel < builtinCompressorCount()) {
            tm.comp = builtinCompressor(sel); tm.compChoice = sel;
            std::snprintf(tm.compName, sizeof(tm.compName), "%s", builtinCompressorName(sel));
        }
        if (ImGui::SmallButton("Load CSV...##c")) {
            std::string path;
            if (fileDialog(false, "", path, L"Compressor map (*.csv)\0*.csv\0All files\0*.*\0", L"csv", L"Load compressor map")) {
                CompressorMap m; std::string name, err;
                if (loadCompressorCsv(path, m, name, err)) {
                    tm.comp = m; tm.compChoice = -1;
                    std::snprintf(tm.compName, sizeof(tm.compName), "%s", name.c_str());
                    turboMsg_ = "Loaded compressor map: " + name;
                } else turboMsg_ = "Compressor map not loaded: " + err;
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Save CSV...##c")) {
            std::string path;
            if (fileDialog(true, "compressor_map.csv", path, L"CSV (*.csv)\0*.csv\0", L"csv", L"Save compressor map"))
                turboMsg_ = saveCompressorCsv(path, tm.comp, tm.compName) ? "Compressor map saved." : "Could not write the file.";
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("From size slider")) {
            tm.comp = genericCompressorMap(p_.compFlow_gs, turbo ? 190000.0 : 60000.0);
            tm.compChoice = -1;
            std::snprintf(tm.compName, sizeof(tm.compName), "From size slider (%.0f g/s)", p_.compFlow_gs);
        }
        Help("CSV columns: speed (corrected rpm), flow (kg/s; or flow_gs / flow_lbmin), pr, eff (0..1 or %),\n"
             "one row per point, rows grouped by speed line, flow corrected to 298 K / 101.325 kPa.\n"
             "Up to 10 speed lines of 12 points are kept.");
        CompressorModel m;
        if (m.build(tm.comp))
            ImGui::TextDisabled("top line %.0fk rpm, choke %.0f g/s, peak efficiency %.0f %%", m.maxSpeed() / 1000, m.maxFlow() * 1000, m.peakEff() * 100);
        else ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "Compressor map unusable: the generic model is used.");
    }
    // ---- turbine
    if (turbo) {
        std::string items;
        for (int i = 0; i < builtinTurbineCount(); ++i) { items += builtinTurbineName(i); items.push_back('\0'); }
        items += tm.turbChoice < 0 && tm.turbName[0] ? tm.turbName : "(custom)"; items.push_back('\0');
        int sel = tm.turbChoice >= 0 ? tm.turbChoice : builtinTurbineCount();
        if (ImGui::Combo("Turbine map", &sel, items.c_str()) && sel < builtinTurbineCount()) {
            tm.turb = builtinTurbine(sel); tm.turbChoice = sel;
            std::snprintf(tm.turbName, sizeof(tm.turbName), "%s", builtinTurbineName(sel));
        }
        if (ImGui::SmallButton("Load CSV...##t")) {
            std::string path;
            if (fileDialog(false, "", path, L"Turbine map (*.csv)\0*.csv\0All files\0*.*\0", L"csv", L"Load turbine map")) {
                TurbineMap t; std::string name, err;
                if (loadTurbineCsv(path, t, name, err)) {
                    tm.turb = t; tm.turbChoice = -1;
                    std::snprintf(tm.turbName, sizeof(tm.turbName), "%s", name.c_str());
                    turboMsg_ = "Loaded turbine map: " + name;
                } else turboMsg_ = "Turbine map not loaded: " + err;
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Save CSV...##t")) {
            std::string path;
            if (fileDialog(true, "turbine_map.csv", path, L"CSV (*.csv)\0*.csv\0", L"csv", L"Save turbine map"))
                turboMsg_ = saveTurbineCsv(path, tm.turb, tm.turbName) ? "Turbine map saved." : "Could not write the file.";
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("From A/R slider")) {
            tm.turb = genericTurbineMap(genericTurbineArea(p_));
            tm.turbChoice = -1;
            std::snprintf(tm.turbName, sizeof(tm.turbName), "From A/R %.2f", p_.turbineAR);
        }
        Help("CSV: a section \"er,mfp\" (expansion ratio, reduced flow m*sqrt(T)/p in kg/s*sqrt(K)/kPa)\n"
             "and a section \"uc,eff\" (blade speed ratio U/C, efficiency); \"# wheel_dia_mm: 52\".");
        TurbineModel t;
        if (t.build(tm.turb))
            ImGui::TextDisabled("wheel %.0f mm, max reduced flow %.4f, peak efficiency %.0f %%", t.wheelDia(), t.maxMfp(), t.peakEff() * 100);
        else ImGui::TextColored(ImVec4(1, 0.5f, 0.3f, 1), "Turbine map unusable: the generic turbine is used.");
    }
    if (!turboMsg_.empty()) ImGui::TextDisabled("%s", turboMsg_.c_str());
}

// efficiency islands: contour lines of the efficiency over the (speed, beta) grid of the map, by marching squares
void App::rebuildMapView() {
    mapIsoX_.clear(); mapIsoY_.clear(); mapIsoLevel_.clear();
    mapLinesX_.clear(); mapLinesY_.clear(); mapLineSpeed_.clear(); mapSurgeX_.clear(); mapSurgeY_.clear();
    mapView_ = CompressorModel();
    if (!mapView_.build(p_.turbo.comp)) return;
    constexpr int B = CompressorModel::kB;
    // speed lines of the source map
    for (int k = 0; k < std::min<int>(p_.turbo.comp.nLines, kCompLines); ++k) {
        const double n = p_.turbo.comp.speed[k];
        double f[B], pr[B], e[B];
        mapView_.line(n, f, pr, e);
        std::vector<float> x(B), y(B);
        int ip = 0;
        for (int b = 0; b < B; ++b) { x[b] = (float)(f[b] * 1000); y[b] = (float)pr[b]; if (pr[b] > pr[ip]) ip = b; }
        mapLinesX_.push_back(x); mapLinesY_.push_back(y); mapLineSpeed_.push_back((float)n);
        mapSurgeX_.push_back(x[ip]); mapSurgeY_.push_back(y[ip]);
    }
    // fine grid for the islands
    constexpr int NS = 40;
    static double gf[NS][B], gp[NS][B], ge[NS][B];
    const double n0 = std::max(1.0, p_.turbo.comp.speed[0] * 1.0), n1 = mapView_.maxSpeed();
    for (int s = 0; s < NS; ++s) mapView_.line(n0 + (n1 - n0) * s / (NS - 1), gf[s], gp[s], ge[s]);
    const double pk = mapView_.peakEff();
    const double levels[] = {pk - 0.20, pk - 0.14, pk - 0.09, pk - 0.05, pk - 0.02};
    for (double lv : levels) {
        std::vector<float> xs, ys;
        for (int s = 0; s + 1 < NS; ++s)
            for (int b = 0; b + 1 < B; ++b) {
                // corners in order around the cell
                const int cs[4][2] = {{s, b}, {s + 1, b}, {s + 1, b + 1}, {s, b + 1}};
                float px[4], py[4]; int m = 0;
                for (int e = 0; e < 4; ++e) {
                    const int *a = cs[e], *c = cs[(e + 1) % 4];
                    const double va = ge[a[0]][a[1]] - lv, vc = ge[c[0]][c[1]] - lv;
                    if ((va < 0) != (vc < 0)) {
                        const double t = va / (va - vc);
                        px[m] = (float)(1000 * (gf[a[0]][a[1]] + t * (gf[c[0]][c[1]] - gf[a[0]][a[1]])));
                        py[m] = (float)(gp[a[0]][a[1]] + t * (gp[c[0]][c[1]] - gp[a[0]][a[1]]));
                        ++m;
                    }
                }
                if (m >= 2) { xs.push_back(px[0]); ys.push_back(py[0]); xs.push_back(px[1]); ys.push_back(py[1]); }
                if (m == 4) { xs.push_back(px[2]); ys.push_back(py[2]); xs.push_back(px[3]); ys.push_back(py[3]); }
            }
        mapIsoX_.push_back(xs); mapIsoY_.push_back(ys); mapIsoLevel_.push_back((float)lv);
    }
}

void App::turboMapPlot(float height) {
    const uint64_t key = fnv(&p_.turbo.comp, sizeof(p_.turbo.comp));
    if (key != mapViewKey_) { mapViewKey_ = key; rebuildMapView(); }
    if (!mapView_.valid()) return;
    const bool turbo = p_.induction == Induction::Turbo;
    const float opX = tel_.mapComp ? tel_.compFlowCorr : tel_.compFlow;
    mapTrailX_.push_back(opX); mapTrailY_.push_back(tel_.pr);
    if (mapTrailX_.size() > 400) { mapTrailX_.erase(mapTrailX_.begin()); mapTrailY_.erase(mapTrailY_.begin()); }
    const bool split = turbo && tel_.mapTurb;
    const float w = split ? ImGui::GetContentRegionAvail().x * 0.64f : -1.0f;
    float prTop = 1.5f;
    for (float y : mapSurgeY_) prTop = std::max(prTop, y);
    if (ImPlot::BeginPlot("Compressor map##tbl", ImVec2(w, height))) {
        ImPlot::SetupAxes("corrected flow (g/s)", "pressure ratio");
        ImPlot::SetupLegend(ImPlotLocation_SouthEast);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, mapView_.maxFlow() * 1000 * 1.1, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0.9, prTop * 1.1 + 0.1, ImPlotCond_Always);
        for (size_t k = 0; k < mapIsoX_.size(); ++k) {
            const float t = (float)k / std::max<size_t>(1, mapIsoX_.size() - 1);
            char id[32]; std::snprintf(id, sizeof(id), "%.0f%%##iso%zu", mapIsoLevel_[k] * 100, k);
            ImPlot::SetNextLineStyle(ImVec4(0.25f + 0.7f * t, 0.55f + 0.3f * t, 0.85f - 0.6f * t, 0.85f), 1.3f);
            ImPlot::PlotLine(id, mapIsoX_[k].data(), mapIsoY_[k].data(), (int)mapIsoX_[k].size(), ImPlotLineFlags_Segments);
        }
        for (size_t k = 0; k < mapLinesX_.size(); ++k) {
            char id[24]; std::snprintf(id, sizeof(id), "##sl%zu", k);
            ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.6f, 0.65f, 0.8f), 1);
            ImPlot::PlotLine(id, mapLinesX_[k].data(), mapLinesY_[k].data(), (int)mapLinesX_[k].size());
            // speed label just above the surge end of the line (the surge ends climb diagonally: no overlaps)
            char tx[16]; std::snprintf(tx, sizeof(tx), "%.0fk", mapLineSpeed_[k] / 1000);
            ImPlot::PlotText(tx, mapSurgeX_[k], mapSurgeY_[k], ImVec2(-14, -9));
        }
        ImPlot::SetNextLineStyle(ImVec4(1, 0.35f, 0.2f, 1), 2);
        ImPlot::PlotLine("surge line", mapSurgeX_.data(), mapSurgeY_.data(), (int)mapSurgeX_.size());
        ImPlot::SetNextLineStyle(ImVec4(0.95f, 0.8f, 0.3f, 0.6f), 1.5f);
        ImPlot::PlotLine("trail", mapTrailX_.data(), mapTrailY_.data(), (int)mapTrailX_.size());
        double ox[] = {opX}, oy[] = {tel_.pr};
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 7, tel_.surging ? ImVec4(1, 0.3f, 0.2f, 1) : ImVec4(1, 1, 1, 1));
        ImPlot::PlotScatter("operating point", ox, oy, 1);
        char txt[96];
        std::snprintf(txt, sizeof(txt), "eff %.0f %%   surge margin %.0f %%", tel_.compEff * 100, tel_.surgeMargin * 100);
        ImPlot::Annotation(ImPlot::GetPlotLimits().X.Max, ImPlot::GetPlotLimits().Y.Max, ImVec4(0.1f, 0.1f, 0.12f, 0.85f), ImVec2(-8, 8), true, "%s", txt);
        ImPlot::EndPlot();
    }
    if (split) {
        ImGui::SameLine();
        TurbineModel t;
        if (t.build(p_.turbo.turb) && ImPlot::BeginPlot("Turbine##tbl", ImVec2(-1, height))) {
            ImPlot::SetupAxes("blade speed ratio U/C", "efficiency");
            ImPlot::SetupLegend(ImPlotLocation_South);
            ImPlot::SetupAxisLimits(ImAxis_X1, 0, 1.3, ImPlotCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.9, ImPlotCond_Always);
            float x[60], y[60];
            for (int i = 0; i < 60; ++i) { x[i] = 1.3f * i / 59.0f; y[i] = (float)t.eff(x[i]); }
            ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.55f, 0.3f, 1), 2);
            ImPlot::PlotLine("efficiency", x, y, 60);
            double ox[] = {tel_.turbUc}, oy[] = {tel_.turbEff};
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 7, ImVec4(1, 1, 1, 1));
            ImPlot::PlotScatter("now", ox, oy, 1);
            char txt[96];
            std::snprintf(txt, sizeof(txt), "ER %.2f  flow %.4f", tel_.turbEr, tel_.turbMfp);
            ImPlot::Annotation(ImPlot::GetPlotLimits().X.Max, ImPlot::GetPlotLimits().Y.Max, ImVec4(0.1f, 0.1f, 0.12f, 0.85f), ImVec2(-8, 8), true, "%s", txt);
            ImPlot::EndPlot();
        }
    }
}

// ------------------------------------------------------------------ valvetrain dynamics
void App::valveDynamicsUi() {
    // the analysis follows the parts and the cams: recompute only when they change
    struct Key { ComponentSet c; float il, id, el, ed, vil, vid, vel, ved, lash; int32_t vtec; } k;
    std::memset(&k, 0, sizeof(k));
    k.c = p_.comp; k.c.repairCount = 0; k.c.allowFailure = 0;
    k.il = p_.intakeLift_mm; k.id = p_.intakeDuration; k.el = p_.exhaustLift_mm; k.ed = p_.exhaustDuration;
    k.vil = p_.vtecInLift_mm; k.vid = p_.vtecInDuration; k.vel = p_.vtecExLift_mm; k.ved = p_.vtecExDuration;
    k.lash = p_.valveLash_mm; k.vtec = p_.vtec;
    const uint64_t key = fnv(&k, sizeof(k));
    const bool hi = p_.vtec != 0;
    if (vtPreviewRpm_ <= 0) vtPreviewRpm_ = std::max(3000.0f, p_.revLimit);
    if (key != vtKey_) {
        vtKey_ = key;
        const ValvetrainSpec si = valvetrainSpec(p_, true), se = valvetrainSpec(p_, false);
        vtFloat_[0] = (float)dynamicFloatRpm(si, p_.intakeLift_mm * 1e-3, p_.intakeDuration);
        vtFloat_[1] = (float)dynamicFloatRpm(se, p_.exhaustLift_mm * 1e-3, p_.exhaustDuration);
        vtFloat_[2] = hi ? (float)dynamicFloatRpm(si, p_.vtecInLift_mm * 1e-3, p_.vtecInDuration) : 0.0f;
        vtFloat_[3] = hi ? (float)dynamicFloatRpm(se, p_.vtecExLift_mm * 1e-3, p_.vtecExDuration) : 0.0f;
        vtTraceRpm_ = -1;
    }
    char b[160];
    const float lim = p_.revLimit;
    float lo = std::min(vtFloat_[0], vtFloat_[1]);
    if (hi) {
        lo = std::min(lo, std::min(vtFloat_[2], vtFloat_[3]));
        std::snprintf(b, sizeof(b), "low cam %.0f / %.0f rpm, high cam %.0f / %.0f rpm (in / ex)", vtFloat_[0], vtFloat_[1], vtFloat_[2], vtFloat_[3]);
    } else std::snprintf(b, sizeof(b), "intake %.0f rpm, exhaust %.0f rpm", vtFloat_[0], vtFloat_[1]);
    ImGui::TextUnformatted("Dynamic float");
    ImGui::SameLine(ImGui::GetFontSize() * 11);
    ImGui::TextColored(lo > lim * 1.05f ? ImVec4(0.5f, 0.9f, 0.5f, 1) : lo > lim ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(1, 0.4f, 0.3f, 1), "%s", b);
    Help("Spring-mass valve driven through an elastic train (stiffness by lifter type) landing on an elastic seat,\n"
         "integrated over the cam event: the speed where the valve first visibly leaves the follower or bounces.\n"
         "The static estimate above compares spring force with mass x nose deceleration only.");
    if (tel_.rpm > 1500 && (tel_.loftIn > 0.01f || tel_.bounceIn > 0.01f || tel_.loftEx > 0.01f || tel_.bounceEx > 0.01f))
        ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "now: loft %.2f / %.2f mm, seat bounce %.2f / %.2f mm, closes %.0f / %.0f deg late (in / ex)",
                           tel_.loftIn, tel_.loftEx, tel_.bounceIn, tel_.bounceEx, tel_.lateCloseIn, tel_.lateCloseEx);

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14);
    ImGui::SliderFloat("Valve motion at", &vtPreviewRpm_, 1000, std::max(9000.0f, lim * 1.4f), "%.0f rpm");
    ImGui::SameLine();
    ImGui::Checkbox("high cam##vt", &vtShowHi_);
    if (!hi) vtShowHi_ = false;
    if (vtTraceRpm_ != vtPreviewRpm_ || vtTraceHi_ != vtShowHi_) {
        vtTraceRpm_ = vtPreviewRpm_; vtTraceHi_ = vtShowHi_;
        const bool h = vtShowHi_;
        vtPrevIn_.keepTrace = vtPrevEx_.keepTrace = true;
        vtPrevIn_.compute(valvetrainSpec(p_, true), (h ? p_.vtecInLift_mm : p_.intakeLift_mm) * 1e-3, h ? p_.vtecInDuration : p_.intakeDuration, vtPreviewRpm_);
        vtPrevEx_.compute(valvetrainSpec(p_, false), (h ? p_.vtecExLift_mm : p_.exhaustLift_mm) * 1e-3, h ? p_.vtecExDuration : p_.exhaustDuration, vtPreviewRpm_);
    }
    if (ImPlot::BeginPlot("##valvemotion", ImVec2(-1, ImGui::GetFontSize() * 12))) {
        ImPlot::SetupAxes("crank deg from the lobe centre", "lift (mm)");
        auto plot = [](const ValveDynamics& d, const char* nm, ImVec4 c) {
            const int n = (int)d.trD.size();
            if (n < 2) return;
            static std::vector<float> v, k;
            v.resize(n); k.resize(n);
            for (int i = 0; i < n; ++i) { v[i] = d.trValve[i] * 1000; k[i] = d.trCam[i] * 1000; }
            char id[40];
            std::snprintf(id, sizeof(id), "%s cam", nm);
            ImPlot::SetNextLineStyle(ImVec4(c.x, c.y, c.z, 0.45f), 1.2f);
            ImPlot::PlotLine(id, d.trD.data(), k.data(), n);
            std::snprintf(id, sizeof(id), "%s valve", nm);
            ImPlot::SetNextLineStyle(c, 2.0f);
            ImPlot::PlotLine(id, d.trD.data(), v.data(), n);
        };
        plot(vtPrevIn_, "intake", ImVec4(0.4f, 0.7f, 1.0f, 1));
        plot(vtPrevEx_, "exhaust", ImVec4(1.0f, 0.55f, 0.3f, 1));
        char txt[128];
        std::snprintf(txt, sizeof(txt), "loft %.2f / %.2f mm  bounce %.2f / %.2f mm", vtPrevIn_.loft * 1e3, vtPrevEx_.loft * 1e3,
                      vtPrevIn_.bounce * 1e3, vtPrevEx_.bounce * 1e3);
        ImPlot::Annotation(ImPlot::GetPlotLimits().X.Max, ImPlot::GetPlotLimits().Y.Max, ImVec4(0.1f, 0.1f, 0.12f, 0.85f), ImVec2(-8, 8), true, "%s", txt);
        ImPlot::EndPlot();
    }
}

// ------------------------------------------------------------------ ECU Setup: trigger, injection, ignition (Phase 7)
void App::ecuConfigUi() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    ImGui::SeparatorText("Trigger");
    ImGui::Combo("Crank trigger wheel", &c.triggerPattern, triggerNames());
    Help("The crank wheel gives the speed tooth by tooth; the angle is known once a missing-tooth gap has passed\n"
         "(a few teeth after it). More gaps (36-2-2-2) sync sooner; the 4-tooth wheel needs the cam tooth for its\n"
         "angle - up to two crank turns of cranking before the first spark.");
    {
        bool cam = c.camSensor != 0;
        if (ImGui::Checkbox("Cam phase sensor", &cam)) c.camSensor = cam ? 1 : 0;
        Help("With a cam tooth the ECU knows which revolution is the intake one: sequential injection.\n"
             "Without it (or before it is seen) the injectors fire every revolution with half the fuel.");
    }
    if (p_.fuelSystem == FuelSystem::EFI && !isRotary(p_)) {
        ImGui::SeparatorText("Injection");
        ImGui::Combo("Injection timing", &c.injMode, injModeNames());
        Help("Sequential: each injector once per cycle at its own angle (needs the cam phase).\n"
             "Semi-sequential: each injector at its own angle every revolution, half the fuel each time.\n"
             "Batch: all injectors together every revolution at one crank angle - some ports get their fuel\n"
             "while their intake valve is open, others wait a whole cycle on the port wall.");
    }
    ImGui::SeparatorText("Ignition");
    {
        int ct = (int)p_.comp.coilType;
        if (ImGui::Combo("Coil type", &ct, coilTypeNames())) p_.comp.coilType = (CoilType)ct;
    }
    Help("Distributor: one coil for all cylinders (little time to charge at high rpm). Wasted spark: one coil per\n"
         "pair, fires every revolution. Coil on plug: one per cylinder, most time to charge.");
    ImGui::SliderFloat("Max dwell duty", &c.dwellMaxDuty, 0.2f, 0.9f, "%.2f");
    Help("The coil may charge at most this share of the time between its sparks (it needs the rest to fire\n"
         "and cool). At high rpm this, not the dwell table, sets the dwell.");
    {
        const double full = 3.0 * std::max(0.3f, p_.comp.coilCharge_ms);
        char b[160];
        std::snprintf(b, sizeof(b), "%.2f ms now, spark %.0f of %.0f mJ (coil saturates in ~%.1f ms at 14 V)", tel_.dwellMs, tel_.sparkAvail,
                      p_.comp.coilEnergy_mJ, full / 3.0);
        ImGui::TextDisabled("%s", b);
        if (tel_.dwellMs > 2.5 * full) ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "Dwell far beyond saturation: the coil only heats up.");
    }
    curveEd_[kCalDwell].drawCurve("dwell", c.dwell, calCurveInfo(kCalDwell), tel_.snsValue[kSnsBattery], tel_.running, ref ? &ref->dwell : nullptr);
    Help("Dwell over battery voltage: a low voltage charges the coil more slowly, so it needs a longer dwell.\n"
         "Too short = a weak spark (misfires under boost / with a wide plug gap).");
    ImGui::SeparatorText("Hardware");
    ImGui::TextDisabled("ECU model: one generic standalone ECU (no per-model feature limits).");
    ImGui::TextDisabled("I/O assignment and CAN bus: not supported - every sensor and actuator is wired directly.");
}

// ------------------------------------------------------------------ Engine Control > Torque / gear (Phase 7)
void App::tabTorqueGear() {
    Calibration& c = p_.cal;
    const Calibration* ref = hasRef_ ? &refCal_ : nullptr;
    const bool dbw = p_.comp.throttleType == ThrottleType::DriveByWire;
    ImGui::PushItemWidth(-260);
    ImGui::SeparatorText("Torque management");
    {
        bool on = c.torqueMgmt != 0;
        if (ImGui::Checkbox("Torque limit by gear", &on)) c.torqueMgmt = on ? 1 : 0;
        Help("The ECU estimates the torque from the cylinder air charge (% of a full naturally aspirated charge;\n"
             "boost goes above 100 %) and holds it under the limit of the gear: gearbox / clutch protection,\n"
             "traction in the low gears. Drive-by-wire: the throttle is capped (clean); what is still above the\n"
             "limit is taken off with retard (first ~15 %) and then dropped sparks.");
    }
    ImGui::SliderFloat("Max retard##tq", &c.tqMaxRetard, 0, 25, "%.0f deg");
    ImGui::SliderFloat("Max spark cut##tq", &c.tqMaxCut, 0, 1, "%.2f");
    if (dbw) ImGui::SliderFloat("Throttle cap rate", &c.tqThrottleRate, 0.002f, 0.1f, "%.3f /s per %%");
    else ImGui::TextDisabled("Cable throttle: the limit works with retard and spark cut only.");
    char b[200];
    std::snprintf(b, sizeof(b), "gear %d   load %.0f %%   limit %s   reduction %.0f %%%s", tel_.gear, tel_.tqLoad,
                  tel_.tqLimit > 0 ? (std::to_string((int)tel_.tqLimit) + " %").c_str() : "-", tel_.tqReduction * 100,
                  dbw && tel_.tqThrCap < 0.999f ? ("   throttle cap " + std::to_string((int)(tel_.tqThrCap * 100)) + " %").c_str() : "");
    ImGui::TextDisabled("Now: %s", b);
    curveEd_[kCalTqGear].drawCurve("tqgear", c.tqLimitGear, calCurveInfo(kCalTqGear), (float)tel_.gear, tel_.vehicle, ref ? &ref->tqLimitGear : nullptr);
    ImGui::SeparatorText("Gear-based control");
    {
        bool on = c.gearControl != 0;
        if (ImGui::Checkbox("Rev limit and spark trim by gear", &on)) c.gearControl = on ? 1 : 0;
        Help("From the gear position sensor: a lower rev limit in the low gears (wheelspin, gearbox), spark trim\n"
             "per gear. Neutral uses the main limiter.");
    }
    ImGui::TextDisabled("Rev limit in effect: %.0f rpm", tel_.gearLimitRpm);
    curveEd_[kCalGearRev].drawCurve("gearrev", c.gearRevLimit, calCurveInfo(kCalGearRev), (float)tel_.gear, tel_.vehicle, ref ? &ref->gearRevLimit : nullptr);
    curveEd_[kCalGearSpark].drawCurve("gearspark", c.gearSpark, calCurveInfo(kCalGearSpark), (float)tel_.gear, tel_.vehicle, ref ? &ref->gearSpark : nullptr);
    ImGui::PopItemWidth();
}

} // namespace es
