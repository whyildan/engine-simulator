#include "sim/TurboMap.h"
#include "sim/Gas.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

namespace es {

// ====================================================================== compressor
bool CompressorModel::build(const CompressorMap& m) {
    n_ = 0;
    struct Line { double sp; std::vector<double> f, p, e; };
    std::vector<Line> lines;
    double fMax = 0, pMax = 1;
    for (int k = 0; k < std::min<int>(m.nLines, kCompLines); ++k) {
        const int np = std::min<int>(m.nPts[k], kCompPts);
        if (np < 2 || !(m.speed[k] > 0)) continue;
        std::vector<std::pair<double, int>> ord;
        for (int j = 0; j < np; ++j)
            if (m.flow[k][j] > 0 && m.pr[k][j] > 0.5) ord.push_back({m.flow[k][j], j});
        if (ord.size() < 2) continue;
        std::sort(ord.begin(), ord.end());
        Line L; L.sp = m.speed[k];
        for (auto& o : ord) {
            L.f.push_back(m.flow[k][o.second]); L.p.push_back(m.pr[k][o.second]);
            L.e.push_back(clampv((double)m.eff[k][o.second], 0.05, 0.95));
            fMax = std::max(fMax, L.f.back()); pMax = std::max(pMax, L.p.back());
        }
        lines.push_back(L);
    }
    if (lines.size() < 2) return false;
    std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) { return a.sp < b.sp; });
    const double fs = 1.0 / std::max(1e-6, fMax), ps = 1.0 / std::max(1e-3, pMax - 1.0);
    for (const Line& L : lines) {
        if (n_ >= kCompLines) break;
        if (n_ > 0 && L.sp <= sp_[n_ - 1]) continue;
        // resample evenly along the line's length in normalised (flow, pr) space: the flat surge end and the
        // steep choke end both keep their detail
        const int np = (int)L.f.size();
        std::vector<double> s(np, 0.0);
        for (int j = 1; j < np; ++j) s[j] = s[j - 1] + std::hypot((L.f[j] - L.f[j - 1]) * fs, (L.p[j] - L.p[j - 1]) * ps);
        const double tot = std::max(1e-9, s[np - 1]);
        int j = 0;
        for (int b = 0; b < kB; ++b) {
            const double t = tot * b / (kB - 1);
            while (j < np - 2 && s[j + 1] < t) ++j;
            const double w = clampv((t - s[j]) / std::max(1e-12, s[j + 1] - s[j]), 0.0, 1.0);
            f_[n_][b] = L.f[j] + w * (L.f[j + 1] - L.f[j]);
            p_[n_][b] = L.p[j] + w * (L.p[j + 1] - L.p[j]);
            e_[n_][b] = L.e[j] + w * (L.e[j + 1] - L.e[j]);
        }
        sp_[n_++] = L.sp;
    }
    if (n_ < 2) { n_ = 0; return false; }
    return true;
}

void CompressorModel::lineAt(double n, double* f, double* p, double* e) const {
    int k0 = 0, k1 = 0;
    double w = 0, r = 1;
    if (n <= sp_[0]) { k0 = k1 = 0; r = n / sp_[0]; }
    else if (n >= sp_[n_ - 1]) { k0 = k1 = n_ - 1; r = std::min(1.2, n / sp_[n_ - 1]); }
    else {
        while (k1 < n_ - 1 && sp_[k1] < n) ++k1;
        k0 = k1 - 1;
        w = (n - sp_[k0]) / (sp_[k1] - sp_[k0]);
    }
    // outside the measured lines: fan laws (flow ~ N, pressure rise ~ N^2), efficiency tapering off
    const double effK = r < 1 ? 0.8 + 0.2 * r : std::max(0.5, 1.0 - 0.6 * (r - 1.0));
    for (int b = 0; b < kB; ++b) {
        const double fb = f_[k0][b] + w * (f_[k1][b] - f_[k0][b]);
        const double pb = p_[k0][b] + w * (p_[k1][b] - p_[k0][b]);
        const double eb = e_[k0][b] + w * (e_[k1][b] - e_[k0][b]);
        f[b] = fb * r;
        p[b] = 1.0 + (pb - 1.0) * r * r;
        e[b] = eb * effK;
    }
}

void CompressorModel::line(double n, double* flow, double* pr, double* eff) const {
    if (!valid()) return;
    lineAt(n, flow, pr, eff);
}

CompPoint CompressorModel::eval(double n, double pr) const {
    CompPoint c;
    if (!valid() || n < 0.02 * sp_[0]) return c;
    double f[kB], p[kB], e[kB];
    lineAt(n, f, p, e);
    int ip = 0;
    for (int b = 1; b < kB; ++b) if (p[b] > p[ip]) ip = b;
    c.surgePr = p[ip]; c.surgeFlow = f[ip]; c.chokeFlow = f[kB - 1];
    if (pr <= p[kB - 1]) {
        // choked: the flow cannot rise any further, the wheel just loses efficiency
        c.choke = true; c.flow = f[kB - 1];
        c.eff = std::max(0.3, e[kB - 1] - 0.8 * (p[kB - 1] - pr));
        return c;
    }
    for (int b = kB - 2; b >= ip; --b) {
        if (p[b] >= pr) {
            const double t = (pr - p[b + 1]) / std::max(1e-9, p[b] - p[b + 1]);
            c.flow = f[b + 1] + t * (f[b] - f[b + 1]);
            c.eff = e[b + 1] + t * (e[b] - e[b + 1]);
            return c;
        }
    }
    c.surge = true; c.flow = f[ip]; c.eff = e[ip] * 0.8;
    return c;
}

double CompressorModel::maxFlow() const { return n_ ? f_[n_ - 1][kB - 1] : 0; }
double CompressorModel::peakEff() const {
    double m = 0;
    for (int k = 0; k < n_; ++k) for (int b = 0; b < kB; ++b) m = std::max(m, e_[k][b]);
    return m;
}

// ====================================================================== turbine
bool TurbineModel::build(const TurbineMap& t) {
    nf_ = ne_ = 0;
    std::vector<std::pair<double, double>> a, b;
    for (int i = 0; i < std::min<int>(t.nFlow, kTurbPts); ++i) if (t.er[i] >= 1.0 && t.mfp[i] >= 0) a.push_back({t.er[i], t.mfp[i]});
    for (int i = 0; i < std::min<int>(t.nEff, kTurbPts); ++i) if (t.uc[i] >= 0 && t.eff[i] > 0) b.push_back({t.uc[i], t.eff[i]});
    std::sort(a.begin(), a.end()); std::sort(b.begin(), b.end());
    for (auto& v : a) { if (nf_ && v.first <= er_[nf_ - 1]) continue; er_[nf_] = v.first; mf_[nf_] = v.second; ++nf_; }
    for (auto& v : b) { if (ne_ && v.first <= uc_[ne_ - 1]) continue; uc_[ne_] = v.first; ef_[ne_] = clampv(v.second, 0.05, 0.95); ++ne_; }
    dia_ = t.wheelDia_mm > 5 ? t.wheelDia_mm : 50.0;
    if (nf_ < 2 || ne_ < 1) { nf_ = ne_ = 0; return false; }
    return true;
}

double TurbineModel::mfp(double er) const {
    if (!nf_) return 0;
    if (er <= er_[0]) {
        if (er_[0] <= 1.0) return mf_[0];
        return mf_[0] * clampv((er - 1.0) / (er_[0] - 1.0), 0.0, 1.0);
    }
    if (er >= er_[nf_ - 1]) return mf_[nf_ - 1];
    int i = 1;
    while (er_[i] < er) ++i;
    const double w = (er - er_[i - 1]) / (er_[i] - er_[i - 1]);
    return mf_[i - 1] + w * (mf_[i] - mf_[i - 1]);
}

double TurbineModel::eff(double uc) const {
    if (!ne_) return 0.6;
    if (ne_ == 1) return ef_[0];
    if (uc <= uc_[0]) return ef_[0] * clampv(uc / std::max(1e-3, uc_[0]), 0.2, 1.0);
    if (uc >= uc_[ne_ - 1]) return ef_[ne_ - 1] * std::max(0.2, 1.0 - (uc - uc_[ne_ - 1]));
    int i = 1;
    while (uc_[i] < uc) ++i;
    const double w = (uc - uc_[i - 1]) / (uc_[i] - uc_[i - 1]);
    return ef_[i - 1] + w * (ef_[i] - ef_[i - 1]);
}

double TurbineModel::peakEff() const {
    double m = 0;
    for (int i = 0; i < ne_; ++i) m = std::max(m, ef_[i]);
    return m;
}

// ====================================================================== generators
CompressorMap genericCompressorMap(double choke_gs, double maxRpm) {
    CompressorMap m;
    std::memset(&m, 0, sizeof(m));
    const double ch = std::max(1e-4, choke_gs * 1e-3);
    m.nLines = 10;
    for (int k = 0; k < 10; ++k) {
        const double n = 0.1 * (k + 1);
        const double pr0 = 1.0 + 2.6 * n * n, mch = ch * (0.12 + 0.88 * n);
        m.speed[k] = (float)(maxRpm * n);
        m.nPts[k] = kCompPts;
        for (int j = 0; j < kCompPts; ++j) {
            const double x = 0.98 * (1.0 - (double)j / (kCompPts - 1));
            const double f = std::sqrt(1.0 - x);
            m.flow[k][j] = (float)(mch * f);
            m.pr[k][j] = (float)(1.0 + (pr0 - 1.0) * x);
            m.eff[k][j] = (float)clampv(0.76 * (1.0 - 1.6 * (f - 0.62) * (f - 0.62)), 0.42, 0.76);
        }
    }
    return m;
}

TurbineMap genericTurbineMap(double area) {
    TurbineMap t;
    std::memset(&t, 0, sizeof(t));
    t.nFlow = kTurbPts;
    const double P = 2e5, T = 900;
    for (int i = 0; i < kTurbPts; ++i) {
        const double er = 1.0 + 3.0 * i / (kTurbPts - 1);
        t.er[i] = (float)er;
        t.mfp[i] = (float)(orificeFlow(area, P, T, P / er) * std::sqrt(T) / (P * 1e-3));
    }
    t.nEff = 2; t.uc[0] = 0.0f; t.eff[0] = 0.68f; t.uc[1] = 2.0f; t.eff[1] = 0.68f;
    t.wheelDia_mm = 50;
    return t;
}

namespace {
// a synthetic but realistically shaped compressor family: choke flow (kg/s) of the top line, surge pressure
// ratio of the top line, top speed, peak efficiency, surge flow / choke flow, steepness of the choke end
CompressorMap paramCompressor(double choke, double prTop, double topRpm, double effPeak, double surgeFrac, double steep) {
    CompressorMap m;
    std::memset(&m, 0, sizeof(m));
    static const double ns[] = {0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.93, 1.0};
    m.nLines = 8;
    for (int k = 0; k < 8; ++k) {
        const double n = ns[k];
        const double sf = surgeFrac * (0.85 + 0.3 * n);
        const double xs = 1.0 - std::pow(sf, 1.0 / steep);           // flow = mch (1 - x)^steep
        const double pr0 = 1.0 + (prTop - 1.0) / (1.0 - std::pow(surgeFrac * 1.15, 1.0 / steep)) * n * n;
        const double mch = choke * (0.2 + 0.8 * std::pow(n, 1.1));
        m.speed[k] = (float)(topRpm * n);
        m.nPts[k] = kCompPts;
        for (int j = 0; j < kCompPts; ++j) {
            const double u = (double)j / (kCompPts - 1);
            const double x = xs + (0.25 - xs) * u;                      // surge -> choke end of the map
            const double f = std::pow(1.0 - x, steep);
            m.flow[k][j] = (float)(mch * f);
            m.pr[k][j] = (float)(1.0 + (pr0 - 1.0) * x);
            const double fc = 0.74 - 0.08 * (n - 0.6);
            const double e = effPeak * (1.0 - 1.9 * (f - fc) * (f - fc) - 0.9 * (n - 0.68) * (n - 0.68));
            m.eff[k][j] = (float)clampv(e, 0.5, effPeak);
        }
    }
    return m;
}

TurbineMap paramTurbine(double mfpMax, double dia, double effPeak, double ucPeak) {
    TurbineMap t;
    std::memset(&t, 0, sizeof(t));
    t.nFlow = kTurbPts;
    for (int i = 0; i < kTurbPts; ++i) {
        const double er = 1.0 + 0.2 * i;
        t.er[i] = (float)er;
        t.mfp[i] = (float)(mfpMax * std::sqrt(clampv((1.0 - 1.0 / (er * er)) / (1.0 - 1.0 / 9.0), 0.0, 1.0)));
    }
    t.nEff = 10;
    for (int i = 0; i < 10; ++i) {
        const double uc = 0.2 + 0.1 * i;
        t.uc[i] = (float)uc;
        t.eff[i] = (float)std::max(0.25, effPeak * (1.0 - 2.4 * (uc - ucPeak) * (uc - ucPeak)));
    }
    t.wheelDia_mm = (float)dia;
    return t;
}

struct CompDef { const char* name; double choke, prTop, topRpm, eff, surge, steep; };
const CompDef kComps[] = {
    {"Small 44 mm (quick spool)", 0.19, 3.0, 210000, 0.76, 0.40, 0.38},
    {"Medium 52 mm", 0.30, 3.4, 165000, 0.78, 0.38, 0.36},
    {"Medium 58 mm (wide map)", 0.40, 3.5, 140000, 0.78, 0.32, 0.34},
    {"Large 67 mm", 0.58, 3.8, 120000, 0.79, 0.36, 0.34},
    {"Race 76 mm", 0.80, 4.2, 105000, 0.80, 0.38, 0.32},
    {"Centrifugal supercharger head", 0.45, 2.8, 60000, 0.76, 0.40, 0.40},
};
struct TurbDef { const char* name; double mfp, dia, eff, uc; };
const TurbDef kTurbs[] = {
    {"Small 47 mm, 0.48 A/R", 0.013, 47, 0.72, 0.66},
    {"Medium 53 mm, 0.64 A/R", 0.018, 53, 0.73, 0.68},
    {"Medium 60 mm, 0.82 A/R", 0.024, 60, 0.74, 0.68},
    {"Large 68 mm, 1.06 A/R", 0.032, 68, 0.75, 0.70},
};
} // namespace

int builtinCompressorCount() { return (int)(sizeof(kComps) / sizeof(kComps[0])); }
const char* builtinCompressorName(int i) { return kComps[clampv(i, 0, builtinCompressorCount() - 1)].name; }
CompressorMap builtinCompressor(int i) {
    const CompDef& d = kComps[clampv(i, 0, builtinCompressorCount() - 1)];
    return paramCompressor(d.choke, d.prTop, d.topRpm, d.eff, d.surge, d.steep);
}
int builtinTurbineCount() { return (int)(sizeof(kTurbs) / sizeof(kTurbs[0])); }
const char* builtinTurbineName(int i) { return kTurbs[clampv(i, 0, builtinTurbineCount() - 1)].name; }
TurbineMap builtinTurbine(int i) {
    const TurbDef& d = kTurbs[clampv(i, 0, builtinTurbineCount() - 1)];
    return paramTurbine(d.mfp, d.dia, d.eff, d.uc);
}

// ====================================================================== CSV
namespace {
std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}
std::string lower(std::string s) { for (char& c : s) c = (char)std::tolower((unsigned char)c); return s; }
std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == ',' || c == ';' || c == '\t') { out.push_back(trim(cur)); cur.clear(); }
        else cur += c;
    }
    out.push_back(trim(cur));
    return out;
}
bool num(const std::string& s, double& v) {
    if (s.empty()) return false;
    char* end = nullptr;
    v = std::strtod(s.c_str(), &end);
    return end && *end == 0;
}
// comment "# key: value"
bool commentKey(const std::string& line, const char* key, std::string& val) {
    std::string l = trim(line.substr(1));
    const size_t k = std::strlen(key);
    if (lower(l.substr(0, k)) != key) return false;
    l = trim(l.substr(k));
    if (!l.empty() && l[0] == ':') l = trim(l.substr(1));
    val = l;
    return true;
}
} // namespace

bool loadCompressorCsv(const std::string& path, CompressorMap& out, std::string& name, std::string& err) {
    std::ifstream f(path);
    if (!f) { err = "cannot open the file"; return false; }
    int cS = -1, cF = -1, cP = -1, cE = -1;
    double fUnit = 1.0;
    std::map<double, std::vector<std::array<double, 3>>> lines;
    std::string line;
    int row = 0;
    while (std::getline(f, line)) {
        ++row;
        line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') { std::string v; if (commentKey(line, "name", v)) name = v; continue; }
        std::vector<std::string> t = split(line);
        double v0;
        if (!num(t[0], v0)) {   // header
            cS = cF = cP = cE = -1;
            for (int i = 0; i < (int)t.size(); ++i) {
                const std::string h = lower(t[i]);
                if (h == "speed" || h == "rpm" || h == "speed_rpm" || h == "n") cS = i;
                else if (h == "flow" || h == "flow_kgs" || h == "flow_kg_s") { cF = i; fUnit = 1.0; }
                else if (h == "flow_gs" || h == "flow_g_s") { cF = i; fUnit = 1e-3; }
                else if (h == "flow_lbmin" || h == "flow_lb_min" || h == "lb/min") { cF = i; fUnit = 0.00755987; }
                else if (h == "pr" || h == "pressure_ratio") cP = i;
                else if (h == "eff" || h == "efficiency") cE = i;
            }
            continue;
        }
        if (cS < 0) { cS = 0; cF = 1; cP = 2; cE = 3; }   // no header: speed, flow (kg/s), pr, eff
        if (cF < 0 || cP < 0 || cE < 0) { err = "header needs speed, flow, pr and eff columns"; return false; }
        double s, fl, pr, ef;
        const int need = std::max(std::max(cS, cF), std::max(cP, cE));
        if ((int)t.size() <= need || !num(t[cS], s) || !num(t[cF], fl) || !num(t[cP], pr) || !num(t[cE], ef)) {
            err = "row " + std::to_string(row) + ": not a number"; return false;
        }
        if (ef > 1.5) ef *= 0.01;
        if (s <= 0 || fl <= 0 || pr <= 0.5 || ef <= 0) { err = "row " + std::to_string(row) + ": value out of range"; return false; }
        lines[s].push_back({fl * fUnit, pr, ef});
    }
    if (lines.size() < 2) { err = "need at least two speed lines"; return false; }
    std::memset(&out, 0, sizeof(out));
    std::vector<double> speeds;
    for (auto& kv : lines) speeds.push_back(kv.first);
    // more lines than fit: keep an even selection that includes the slowest and the fastest
    std::vector<int> pick;
    const int nl = (int)speeds.size();
    if (nl <= kCompLines) for (int i = 0; i < nl; ++i) pick.push_back(i);
    else for (int i = 0; i < kCompLines; ++i) pick.push_back((int)std::lround((double)i * (nl - 1) / (kCompLines - 1)));
    for (int k = 0; k < (int)pick.size(); ++k) {
        std::vector<std::array<double, 3>> pts = lines[speeds[pick[k]]];
        std::sort(pts.begin(), pts.end(), [](const std::array<double, 3>& a, const std::array<double, 3>& b) { return a[0] < b[0]; });
        if (pts.size() < 2) { err = "every speed line needs at least two points"; return false; }
        const int np = (int)pts.size(), use = std::min(np, kCompPts);
        out.speed[k] = (float)speeds[pick[k]];
        out.nPts[k] = use;
        for (int j = 0; j < use; ++j) {
            const int src = use == np ? j : (int)std::lround((double)j * (np - 1) / (use - 1));
            out.flow[k][j] = (float)pts[src][0]; out.pr[k][j] = (float)pts[src][1]; out.eff[k][j] = (float)pts[src][2];
        }
    }
    out.nLines = (int)pick.size();
    CompressorModel test;
    if (!test.build(out)) { err = "the map has no usable speed lines"; return false; }
    if (name.empty()) {
        size_t s = path.find_last_of("/\\");
        name = s == std::string::npos ? path : path.substr(s + 1);
    }
    return true;
}

bool saveCompressorCsv(const std::string& path, const CompressorMap& m, const std::string& name) {
    std::ofstream f(path);
    if (!f) return false;
    f << "# EngineSim compressor map\n# name: " << name << "\n";
    f << "# corrected to 298 K / 101.325 kPa inlet; flow kg/s, pressure ratio total-total, isentropic efficiency\n";
    f << "speed,flow,pr,eff\n";
    char b[128];
    for (int k = 0; k < std::min<int>(m.nLines, kCompLines); ++k)
        for (int j = 0; j < std::min<int>(m.nPts[k], kCompPts); ++j) {
            std::snprintf(b, sizeof(b), "%.0f,%.5f,%.4f,%.4f\n", m.speed[k], m.flow[k][j], m.pr[k][j], m.eff[k][j]);
            f << b;
        }
    return (bool)f;
}

bool loadTurbineCsv(const std::string& path, TurbineMap& out, std::string& name, std::string& err) {
    std::ifstream f(path);
    if (!f) { err = "cannot open the file"; return false; }
    std::memset(&out, 0, sizeof(out));
    out.wheelDia_mm = 50;
    int section = 0;   // 1 = flow, 2 = efficiency
    int cA = 0, cB = 1;
    std::string line;
    int row = 0;
    while (std::getline(f, line)) {
        ++row;
        line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') {
            std::string v; double d;
            if (commentKey(line, "name", v)) name = v;
            else if (commentKey(line, "wheel_dia_mm", v) && num(v, d)) out.wheelDia_mm = (float)d;
            continue;
        }
        std::vector<std::string> t = split(line);
        double a, b;
        if (!num(t[0], a)) {
            section = 0; cA = cB = -1;
            for (int i = 0; i < (int)t.size(); ++i) {
                const std::string h = lower(t[i]);
                if (h == "er" || h == "pr" || h == "expansion_ratio") { section = 1; cA = i; }
                else if (h == "mfp" || h == "flow" || h == "flow_param" || h == "reduced_flow") cB = i;
                else if (h == "uc" || h == "u/c" || h == "u_c" || h == "blade_speed_ratio") { section = 2; cA = i; }
                else if (h == "eff" || h == "efficiency") cB = i;
                else if (h == "wheel_dia_mm" && i + 1 < (int)t.size() && num(t[i + 1], b)) out.wheelDia_mm = (float)b;
            }
            if (section && (cA < 0 || cB < 0)) { err = "row " + std::to_string(row) + ": incomplete header"; return false; }
            continue;
        }
        if (!section) { err = "data before an \"er,mfp\" or \"uc,eff\" header"; return false; }
        if ((int)t.size() <= std::max(cA, cB) || !num(t[cA], a) || !num(t[cB], b)) { err = "row " + std::to_string(row) + ": not a number"; return false; }
        if (section == 1 && out.nFlow < kTurbPts) { out.er[out.nFlow] = (float)a; out.mfp[out.nFlow] = (float)b; ++out.nFlow; }
        if (section == 2 && out.nEff < kTurbPts) { out.uc[out.nEff] = (float)a; out.eff[out.nEff] = (float)(b > 1.5 ? b * 0.01 : b); ++out.nEff; }
    }
    TurbineModel test;
    if (!test.build(out)) { err = "need at least two flow points and one efficiency point"; return false; }
    if (name.empty()) {
        size_t s = path.find_last_of("/\\");
        name = s == std::string::npos ? path : path.substr(s + 1);
    }
    return true;
}

bool saveTurbineCsv(const std::string& path, const TurbineMap& t, const std::string& name) {
    std::ofstream f(path);
    if (!f) return false;
    char b[128];
    f << "# EngineSim turbine map\n# name: " << name << "\n";
    std::snprintf(b, sizeof(b), "# wheel_dia_mm: %.1f\n", t.wheelDia_mm); f << b;
    f << "# reduced flow = m * sqrt(T_in) / p_in  [kg/s * sqrt(K) / kPa]; er = p_in / p_out\n";
    f << "er,mfp\n";
    for (int i = 0; i < std::min<int>(t.nFlow, kTurbPts); ++i) { std::snprintf(b, sizeof(b), "%.3f,%.6f\n", t.er[i], t.mfp[i]); f << b; }
    f << "\nuc,eff\n";
    for (int i = 0; i < std::min<int>(t.nEff, kTurbPts); ++i) { std::snprintf(b, sizeof(b), "%.3f,%.4f\n", t.uc[i], t.eff[i]); f << b; }
    return (bool)f;
}

} // namespace es
