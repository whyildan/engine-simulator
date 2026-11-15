#include "ecu/BaseMap.h"
#include "sim/Engine.h"
#include <algorithm>
#include <map>
#include <mutex>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace es {

namespace {
struct Pt { double map, tps, ve, veAtm; };

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
    p.cal.autoTune = 0; p.cal.closedLoop = 0;
    defaultSensorConfig(p.sensors);
    const double ambT = p.ambientTemp_C + 273.15;
    const double ecuT = ambT + 15.0;                    // the ECU's charge temperature (IAT + 15 K)
    const Table2D& ax = p.cal.veMap;
    // rpm samples: dense at the low end (closed-throttle air goes like 1/rpm there), then spread
    static const int pick[] = {0, 1, 2, 4, 7, 10, 13, 15};
    const int nR = 8;
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
            double sMap = 0, sVe = 0, sLam = 0, sVeEcu = 0, sTgt = 0; int n = 0, nl = 0;
            // settle until the manifold pressure has stopped moving (plenums empty slowly at low rpm)
            const int chunk = (int)(0.05 * fs);
            double prevMap = -1;
            for (double t = 0; t < 1.5; t += 0.05) {
                for (int k = 0; k < chunk; ++k) { e.setCrank(omega, e.theta()); e.step(dt, thr[ti], 0.0); }
                double m = e.intake.map();
                if (t >= settle && prevMap > 0 && std::fabs(m - prevMap) < 250.0) break;   // < 0.25 kPa per 50 ms
                prevMap = m;
            }
            const int steps = (int)(window * fs);
            for (int k = 0; k < steps; ++k) {
                e.setCrank(omega, e.theta());           // motoring dyno: the speed is held
                e.step(dt, thr[ti], 0.0);
                double ve = 0;
                for (int c = 0; c < e.cylinders(); ++c) ve += e.cyl(c).ve;
                sVe += ve / e.cylinders(); sMap += e.intake.map(); ++n;
                if (e.ecu.targetAfr > 0) {
                    double lam = 0;
                    for (int c = 0; c < e.cylinders(); ++c) lam += std::min(3.0, e.cyl(c).lambda);
                    sLam += lam / e.cylinders(); sVeEcu += e.ecu.veUsed; sTgt += e.ecu.targetAfr / 14.7; ++nl;
                }
            }
            double map = sMap / n, veAtm = sVe / n;     // trapped air relative to an atmospheric cylinder
            double ve = veAtm * (kPatm / map) * (ecuT / ambT);
            // like a calibrator with a wideband: scale the VE the ECU used by measured / target lambda, so the
            // map also covers fuel effects (port wetting, reversion standoff); air-based value if not firing
            if (nl > n / 2) {
                double lamM = sLam / nl, lamT = sTgt / nl, veE = sVeEcu / nl * 0.01;
                if (lamM > 0.5 && lamM < 2.5 && lamT > 0.5) {
                    double veLam = veE * lamM / lamT;
                    if (veLam > 0.3 * ve && veLam < 2.5 * ve) ve = veLam;
                }
            }
            veAtm = ve * map / kPatm / (ecuT / ambT);
            pts[ri].push_back({map / 1000.0, thr[ti] * 100.0, ve * 100.0, veAtm * (ecuT / ambT) * 100.0});
            if (std::getenv("BASEMAP_DBG")) std::fprintf(stderr, "basemap rpm %5.0f thr %.2f map %6.1f veAtm %6.3f ve %6.3f\n", rpms[ri], thr[ti], map / 1000.0, veAtm, ve);
        }
    }
    // resample onto the table axes: rows by MAP (or TPS), columns by rpm
    for (int which = 0; which < 2; ++which) {
        Table2D& t = which == 0 ? out.veMap : out.veTps;
        std::vector<std::vector<double>> col(nR, std::vector<double>(t.ny));
        for (int ri = 0; ri < nR; ++ri) {
            std::vector<std::pair<double, double>> xy;
            for (const Pt& q : pts[ri]) xy.push_back({which == 0 ? q.map : q.tps, which == 0 ? q.ve : q.veAtm});
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
static constexpr uint32_t kBaseMapVersion = 1;
static std::string g_cacheDir;
void setBaseMapCacheDir(const std::string& dir) { g_cacheDir = dir; }

static uint64_t engineKey(const EngineParams& in) {
    EngineParams p = in;
    std::memset(p.name, 0, sizeof(p.name));
    p.throttle = 0; p.ignition = 0; p.starter = 0; p.listener = ListenerPos::Tailpipe; p.volume = 0;
    p.dynoMode = DynoMode::Off; p.dynoHoldRpm = 0; p.sweepRequest = 0; p.sweepStartRpm = p.sweepEndRpm = p.sweepRate = 0;
    p.vehicle = 0; p.gear = 0; p.clutchPedal = 0; p.brake = 0;
    p.exhaustRasp = p.exhaustPunch = p.exhaustSteep = p.pressureMix = p.smoothHz = 0; p.soundStyle = 0;
    p.environment = Environment::OpenRoad; p.reverbMix = 0; p.irChoice = 0; p.irMix = 0;
    p.comp.repairCount = 0; p.comp.allowFailure = 0;
    p.coldStart = 0;
    std::memset(&p.cal.veMap.v, 0, sizeof(p.cal.veMap.v)); std::memset(&p.cal.veTps.v, 0, sizeof(p.cal.veTps.v));
    p.cal.veMap.locked = p.cal.veTps.locked = 0; p.cal.autoTune = 0; p.cal.closedLoop = 0;
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
