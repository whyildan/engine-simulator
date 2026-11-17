#include "ecu/BaseMap.h"
#include "sim/Engine.h"
#include <algorithm>
#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

namespace es {

namespace {
struct Pt { double map, tps, ve, veAtm; bool fired; };   // fired: mixture-corrected (wideband) value

// piecewise-linear interpolation of y over sorted x (clamped at the ends)
double interp(const std::vector<std::pair<double, double>>& xy, double x) {
    if (xy.empty()) return 0;
    if (x <= xy.front().first) return xy.front().second;
    if (x >= xy.back().first) return xy.back().second;
    for (size_t i = 1; i < xy.size(); ++i)
        if (x <= xy[i].first) {
            double a = xy[i - 1].first, b = xy[i].first;
            double w = b > a ? (x - a) / (b - a) : 0.0;
            return xy[i - 1].second + (xy[i].second - xy[i - 1].second) * w;
        }
    return xy.back().second;
}
} // namespace

bool measureBaseMap(const EngineParams& pIn, Calibration& out, BaseMapProgress* prog, double fs) {
    EngineParams p = pIn;
    p.vehicle = 0; p.dynoMode = DynoMode::Off; p.ignition = 1; p.starter = 0; p.coldStart = 0;
    p.clutchPedal = 0; p.n2oArm = 0; p.transbrake = 0; p.rollingAlsButton = 0;   // no launch limiter / nitrous on the rig
    p.coolingFan_kmh = 50;    // the default cell fan: the engine bay / intercooler as it runs on the dyno
    p.cal.autoTune = 0; p.cal.closedLoop = 0;
    // the calibration rig measures the bare engine: no knock retard, trims, soft limiter or protection actions
    p.cal.knockControl = 0; p.cal.ltftLearn = 0; p.cal.softLimitBand = 0;
    // ... and no transient fuel: sensor noise would trigger acceleration enrichment at the steady test points
    for (int i = 0; i < p.cal.accel.n; ++i) p.cal.accel.v[i] = 0;
    p.cal.dfcoExitEnrich = 0; p.cal.asyncTipIn = 0;
    for (ProtRule& r : p.cal.prot) r.enabled = 0;
    // speed density with the VE of the point under test set directly (Ecu::veOverride), iterated until the
    // engine runs at the target mixture: measuring on a wrong VE would run it rich or lean, and the mixture
    // changes the exhaust temperature, the wave tuning in the pipes and so the very air being measured.
    // (MAF fuelling is no shortcut: on big-overlap engines much of the metered air blows straight through.)
    p.cal.loadMode = LoadMode::SpeedDensity;
    defaultSensorConfig(p.sensors);
    const double ambT = p.ambientTemp_C + 273.15;
    const double ecuT = ambT + 15.0;                    // the ECU's charge temperature (IAT + 15 K)
    const Table2D& ax = p.cal.veMap;
    // rpm samples: dense at the low end (closed-throttle air goes like 1/rpm there), then spread
    std::vector<int> pick = {0, 1, 2, 4, 6, 9, 12, 15};   // 500, 750, 1000, 1500, then spread
    // VTEC: also measure the columns either side of the cam crossover (low cam below, high cam above)
    if (p.vtec)
        for (int k = 0; k + 1 < ax.nx; ++k)
            if (ax.x[k] <= p.cal.vtecRpm && p.cal.vtecRpm < ax.x[k + 1]) { pick.push_back(k); pick.push_back(k + 1); break; }
    std::sort(pick.begin(), pick.end());
    pick.erase(std::unique(pick.begin(), pick.end()), pick.end());
    const int nR = (int)pick.size();
    static const double thr[] = {0.0, 0.02, 0.05, 0.1, 0.2, 0.35, 0.6, 1.0};
    const int nT = (int)(sizeof(thr) / sizeof(thr[0]));
    const double settle = 0.15, window = 0.08, dt = 1.0 / fs;
    std::vector<double> rpms(nR);
    for (int i = 0; i < nR; ++i) rpms[i] = ax.x[std::min(pick[i], (int)ax.nx - 1)];
    std::vector<std::vector<Pt>> pts(nR);
    Engine e(p, fs);
    e.ecu.holdIac = true;
    for (int ri = 0; ri < nR; ++ri) {
        const double omega = rpms[ri] * 2.0 * kPi / 60.0;
        for (int ti = 0; ti < nT; ++ti) {
            if (prog) {
                prog->fraction.store((float)(ri * nT + ti) / (nR * nT));
                if (prog->cancel.load()) return false;
            }
          double est = 0, ve = 0, map = 0, veAtm = 0;
          bool fired = false;
          const double tp = thr[ti];
          for (int pass = 0; pass < 4; ++pass) {
            e.ecu.veOverride = est * 100.0;            // 0 on the first pass: the generic table
            double sMap = 0, sVe = 0, sLam = 0, sVeEcu = 0, sTgt = 0; int n = 0, nl = 0;
            // settle until the point is steady: manifold pressure (plenums empty slowly at low rpm), trapped air
            // and mixture. After a big throttle step the port fuel film takes ~1 s to reach its new balance;
            // until then the mixture is off, which changes the exhaust and so the air being measured.
            const int chunk = (int)(0.1 * fs);
            double prevMap = -1, prevVe = -1, prevLam = -1;
            for (double t = 0; t < 3.0; t += 0.1) {
                double cMap = 0, cVe = 0, cLam = 0;
                for (int k = 0; k < chunk; ++k) {
                    e.setCrank(omega, e.theta()); e.step(dt, tp, 0.0);
                    double v = 0, l = 0;
                    for (int c = 0; c < e.cylinders(); ++c) { v += e.cyl(c).ve; l += std::min(3.0, e.cyl(c).lambda); }
                    cMap += e.intake.map(); cVe += v / e.cylinders(); cLam += l / e.cylinders();
                }
                cMap /= chunk; cVe /= chunk; cLam /= chunk;
                if (t >= settle && prevMap > 0 && std::fabs(cMap - prevMap) < 500.0 &&        // < 0.5 kPa per 0.1 s
                    std::fabs(cVe - prevVe) < 0.004 + 0.004 * cVe && std::fabs(cLam - prevLam) < 0.008) break;
                prevMap = cMap; prevVe = cVe; prevLam = cLam;
            }
            const int steps = (int)(window * fs);
            for (int k = 0; k < steps; ++k) {
                e.setCrank(omega, e.theta());           // motoring dyno: the speed is held
                e.step(dt, tp, 0.0);
                double ve = 0;
                for (int c = 0; c < e.cylinders(); ++c) ve += e.cyl(c).ve;
                sVe += ve / e.cylinders(); sMap += e.intake.map(); ++n;
                if (e.ecu.targetAfr > 0) {
                    double lam = 0;
                    for (int c = 0; c < e.cylinders(); ++c) lam += std::min(3.0, e.cyl(c).lambda);
                    sLam += lam / e.cylinders(); sVeEcu += e.ecu.veUsed; sTgt += e.ecu.targetAfr / 14.7; ++nl;
                }
            }
            map = sMap / n; veAtm = sVe / n;            // trapped air relative to an atmospheric cylinder
            ve = veAtm * (kPatm / map) * (ecuT / ambT);
            // like a calibrator with a wideband: scale the VE the ECU used by measured / target lambda, so the
            // map also covers fuel effects (port wetting, reversion standoff); air-based value if not firing
            fired = false;
            if (nl > n / 2) {
                double lamM = sLam / nl, lamT = sTgt / nl, veE = sVeEcu / nl * 0.01;
                if (lamM > 0.5 && lamM < 2.5 && lamT > 0.5) {
                    double veLam = veE * lamM / lamT;
                    if (veLam > 0.3 * ve && veLam < 2.5 * ve) { ve = veLam; fired = true; }
                }
            }
            if (std::getenv("BASEMAP_DBG")) std::fprintf(stderr, "basemap pass %d rpm %5.0f thr %.2f map %6.1f veAtm %6.3f ve %6.3f | iat %.1f egt %.0f lamM %.3f lamT %.3f veE %.3f nl %d\n", pass, rpms[ri], thr[ti], map / 1000.0, veAtm, ve, e.thermal.iatC, e.egtAvg, nl ? sLam / nl : 0.0, nl ? sTgt / nl : 0.0, nl ? sVeEcu / nl * 0.01 : 0.0, nl);
            const bool converged = est > 0 && std::fabs(ve - est) < 0.015 * est;
            est = ve;
            if (converged || nl <= n / 2) break;       // not firing: the air-based value needs no iteration
          }
          e.ecu.veOverride = 0;
            veAtm = ve * map / kPatm / (ecuT / ambT);
            pts[ri].push_back({map / 1000.0, tp * 100.0, ve * 100.0, veAtm * (ecuT / ambT) * 100.0, fired});
        }
    }
    // resample onto the table axes: rows by MAP (or TPS), columns by rpm
    for (int which = 0; which < 2; ++which) {
        Table2D& t = which == 0 ? out.veMap : out.veTps;
        std::vector<std::vector<double>> col(nR, std::vector<double>(t.ny));
        for (int ri = 0; ri < nR; ++ri) {
            std::vector<std::pair<double, double>> xy;
            // points where the engine did not fire (closed throttle at high rpm: deep vacuum, no mixture to read)
            // give only an air-based value that is unreliable at very low MAP - leave them out when the column has
            // enough firing points, the table then holds the lowest firing value below it
            int nFired = 0;
            for (const Pt& q : pts[ri]) nFired += q.fired;
            for (const Pt& q : pts[ri])
                if (q.fired || nFired < 2) xy.push_back({which == 0 ? q.map : q.tps, which == 0 ? q.ve : q.veAtm});
            std::sort(xy.begin(), xy.end());
            // MAP is not monotonic in rare cases (boost lag): keep the first sample per MAP band
            std::vector<std::pair<double, double>> clean;
            for (auto& v : xy) if (clean.empty() || v.first > clean.back().first + 0.5) clean.push_back(v);
            for (int r = 0; r < t.ny; ++r) col[ri][r] = interp(clean, t.y[r]);
        }
        for (int r = 0; r < t.ny; ++r) {
            std::vector<std::pair<double, double>> xy;
            // alpha-N: interpolate the air flow (VE x rpm), not VE - a fixed opening passes a fixed flow
            for (int ri = 0; ri < nR; ++ri) xy.push_back({rpms[ri], col[ri][r] * (which == 1 ? rpms[ri] : 1.0)});
            for (int k = 0; k < t.nx; ++k)
                t.v[r][k] = (float)clampv(interp(xy, t.x[k]) / (which == 1 ? std::max(1.0, (double)t.x[k]) : 1.0), 2.0, 160.0);
        }
    }
    if (prog) prog->fraction.store(1.0f);
    return true;
}

// ------------------------------------------------------------------ on-disk cache
// Keyed by everything that defines the engine (driver inputs, sound, dyno, vehicle and the VE maps
// themselves are left out). Bump kBaseMapVersion when the physics changes.
static constexpr uint32_t kBaseMapVersion = 24;   // 24: energy-neutral pipe / volume coupling (no self-excited ringing)   // 23: mass-weighted exhaust temperature (pipe wave speed)   // 22: reversion through the throttle keeps its fuel   // 21: trigger sync, dwell-limited spark (Phase 7)   // 20: valvetrain dynamics, turbo maps, intercooler core, cell fan (Phase 6)   // 17: VTEC crossover columns, afterfire needs oxygen   // 16: afterfire heat in the exhaust (Phase 5)   // 9: faster low-load burn, 10: idle-valve test points   // 2: burn-start jitter fix, 5: iterate VE to the target mixture + settle until steady, 6: drop non-firing points, 7: Phase 3
static std::string g_cacheDir;
void setBaseMapCacheDir(const std::string& dir) { g_cacheDir = dir; }

static uint64_t engineKey(const EngineParams& in);
uint64_t baseMapKey(const EngineParams& p) { return engineKey(p); }
static uint64_t engineKey(const EngineParams& in) {
    EngineParams p = in;
    std::memset(p.name, 0, sizeof(p.name));
    p.throttle = 0; p.ignition = 0; p.starter = 0; p.listener = ListenerPos::Tailpipe; p.volume = 0;
    p.dynoMode = DynoMode::Off; p.dynoHoldRpm = 0; p.sweepRequest = 0; p.sweepStartRpm = p.sweepEndRpm = p.sweepRate = 0;
    p.stepRpm = p.stepHold_s = p.coolingFan_kmh = 0;
    if (!p.turbo.useMaps) std::memset(&p.turbo, 0, sizeof(p.turbo));   // unused tables do not matter
    std::memset(p.turbo.compName, 0, sizeof(p.turbo.compName)); std::memset(p.turbo.turbName, 0, sizeof(p.turbo.turbName));
    p.turbo.compChoice = p.turbo.turbChoice = 0;
    p.vehicle = 0; p.gear = 0; p.clutchPedal = 0; p.brake = 0;
    p.nitrous = 0; p.n2oShotHp = 0; p.n2oWet = 0; p.n2oBottleKg = 0; p.n2oRefill = 0;
    p.n2oArm = 0; p.transbrake = 0; p.rollingAlsButton = 0;
    p.exhaustRasp = p.exhaustPunch = p.exhaustSteep = p.pressureMix = p.smoothHz = 0; p.soundStyle = 0;
    p.environment = Environment::OpenRoad; p.reverbMix = 0; p.irChoice = 0; p.irMix = 0;
    p.comp.repairCount = 0; p.comp.allowFailure = 0;
    p.coldStart = 0;
    std::memset(&p.cal.veMap.v, 0, sizeof(p.cal.veMap.v)); std::memset(&p.cal.veTps.v, 0, sizeof(p.cal.veTps.v));
    p.cal.veMap.locked = p.cal.veTps.locked = 0; p.cal.autoTune = 0; p.cal.closedLoop = 0;
    // Phase 3 control settings (trims, knock, idle, boost control, protection) do not change the measured air
    std::memset(reinterpret_cast<char*>(&p.cal) + offsetof(Calibration, stftGain), 0, sizeof(Calibration) - offsetof(Calibration, stftGain));
    uint64_t h = 1469598103934665603ull ^ kBaseMapVersion;
    const unsigned char* b = reinterpret_cast<const unsigned char*>(&p);
    for (size_t i = 0; i < sizeof(EngineParams); ++i) { h ^= b[i]; h *= 1099511628211ull; }
    return h;
}

static std::string cacheFile(const EngineParams& p) {
    std::string dir = g_cacheDir;
    if (dir.empty()) if (const char* e = std::getenv("BASEMAP_CACHE")) dir = e;
    if (dir.empty()) return {};
    char name[64];
    std::snprintf(name, sizeof(name), "/basemap_%016llx.bin", (unsigned long long)engineKey(p));
    return dir + name;
}

bool loadCachedBaseMap(const EngineParams& p, Calibration& out) {
    const std::string f = cacheFile(p);
    if (f.empty()) return false;
    Table2D t[2];
    FILE* fp = std::fopen(f.c_str(), "rb");
    if (!fp) return false;
    bool ok = std::fread(t, sizeof(t), 1, fp) == 1;
    std::fclose(fp);
    if (!ok) return false;
    std::memcpy(out.veMap.v, t[0].v, sizeof(t[0].v));
    std::memcpy(out.veTps.v, t[1].v, sizeof(t[1].v));
    return true;
}

void saveCachedBaseMap(const EngineParams& p, const Calibration& c) {
    const std::string f = cacheFile(p);
    if (f.empty()) return;
    Table2D t[2] = {c.veMap, c.veTps};
    if (FILE* fp = std::fopen(f.c_str(), "wb")) { std::fwrite(t, sizeof(t), 1, fp); std::fclose(fp); }
}

EngineParams makeCalibratedPreset(int idx) {
    static std::mutex mx;
    static std::map<int, Calibration> cache;
    EngineParams p = makePreset(idx);
    {
        std::lock_guard<std::mutex> lk(mx);
        auto it = cache.find(idx);
        if (it != cache.end()) { p.cal.veMap = it->second.veMap; p.cal.veTps = it->second.veTps; return p; }
    }
    Calibration c = p.cal;
    if (!loadCachedBaseMap(p, c)) {
        measureBaseMap(p, c);
        saveCachedBaseMap(p, c);
    }
    {
        std::lock_guard<std::mutex> lk(mx);
        cache[idx] = c;
    }
    p.cal.veMap = c.veMap; p.cal.veTps = c.veTps;
    return p;
}

} // namespace es
