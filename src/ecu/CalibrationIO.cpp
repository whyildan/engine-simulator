#include "ecu/CalibrationIO.h"
#include "sim/EngineParams.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>

namespace fs = std::filesystem;

namespace es {

// ------------------------------------------------------------------ tiny JSON value / parser
namespace {
struct JVal {
    enum Kind { Null, Num, Str, Arr, Obj, Bool } kind = Null;
    double num = 0;
    bool b = false;
    std::string str;
    std::vector<JVal> arr;
    std::map<std::string, JVal> obj;
    const JVal* get(const char* k) const {
        if (kind != Obj) return nullptr;
        auto it = obj.find(k);
        return it == obj.end() ? nullptr : &it->second;
    }
};

struct Parser {
    const char* s; const char* e; std::string err;
    void ws() { while (s < e && (*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r')) ++s; }
    bool fail(const char* m) { if (err.empty()) { err = m; } return false; }
    bool str(std::string& out) {
        if (s >= e || *s != '"') return fail("expected string");
        ++s;
        while (s < e && *s != '"') {
            char c = *s++;
            if (c == '\\' && s < e) {
                char x = *s++;
                switch (x) {
                case 'n': out += '\n'; break; case 't': out += '\t'; break; case 'r': out += '\r'; break;
                case 'b': out += '\b'; break; case 'f': out += '\f'; break;
                case 'u': {
                    if (e - s < 4) return fail("bad escape");
                    unsigned cp = (unsigned)std::strtoul(std::string(s, s + 4).c_str(), nullptr, 16); s += 4;
                    if (cp < 0x80) out += (char)cp;
                    else if (cp < 0x800) { out += (char)(0xC0 | (cp >> 6)); out += (char)(0x80 | (cp & 0x3F)); }
                    else { out += (char)(0xE0 | (cp >> 12)); out += (char)(0x80 | ((cp >> 6) & 0x3F)); out += (char)(0x80 | (cp & 0x3F)); }
                    break;
                }
                default: out += x;
                }
            } else out += c;
        }
        if (s >= e) return fail("unterminated string");
        ++s;
        return true;
    }
    bool val(JVal& v, int depth = 0) {
        if (depth > 32) return fail("nesting too deep");
        ws();
        if (s >= e) return fail("unexpected end");
        char c = *s;
        if (c == '{') {
            v.kind = JVal::Obj; ++s; ws();
            if (s < e && *s == '}') { ++s; return true; }
            for (;;) {
                ws(); std::string k;
                if (!str(k)) return false;
                ws(); if (s >= e || *s != ':') return fail("expected ':'");
                ++s;
                if (!val(v.obj[k], depth + 1)) return false;
                ws();
                if (s < e && *s == ',') { ++s; continue; }
                if (s < e && *s == '}') { ++s; return true; }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            v.kind = JVal::Arr; ++s; ws();
            if (s < e && *s == ']') { ++s; return true; }
            for (;;) {
                v.arr.emplace_back();
                if (!val(v.arr.back(), depth + 1)) return false;
                ws();
                if (s < e && *s == ',') { ++s; continue; }
                if (s < e && *s == ']') { ++s; return true; }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') { v.kind = JVal::Str; return str(v.str); }
        if (e - s >= 4 && std::strncmp(s, "true", 4) == 0) { v.kind = JVal::Bool; v.b = true; s += 4; return true; }
        if (e - s >= 5 && std::strncmp(s, "false", 5) == 0) { v.kind = JVal::Bool; v.b = false; s += 5; return true; }
        if (e - s >= 4 && std::strncmp(s, "null", 4) == 0) { v.kind = JVal::Null; s += 4; return true; }
        char* end = nullptr;
        std::string tmp(s, std::min<size_t>(e - s, 64));
        double d = std::strtod(tmp.c_str(), &end);
        if (end == tmp.c_str()) return fail("unexpected character");
        v.kind = JVal::Num; v.num = d; s += end - tmp.c_str();
        return true;
    }
};

// ------------------------------------------------------------------ writer helpers
std::string esc(const std::string& in) {
    std::string o;
    for (char c : in) {
        switch (c) {
        case '"': o += "\\\""; break; case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break; case '\r': break; case '\t': o += "\\t"; break;
        default:
            if ((unsigned char)c < 0x20) { char b[8]; std::snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
            else o += c;
        }
    }
    return o;
}
std::string num(double v) {
    if (!std::isfinite(v)) v = 0;
    // shortest form that reads back to the same float (files stay readable and round-trip exactly)
    char b[32];
    for (int prec = 6; prec <= 9; ++prec) {
        std::snprintf(b, sizeof(b), "%.*g", prec, v);
        if ((float)std::strtod(b, nullptr) == (float)v) break;
    }
    return b;
}
std::string arr(const float* a, int n) {
    std::string o = "[";
    for (int i = 0; i < n; ++i) { if (i) o += ", "; o += num(a[i]); }
    return o + "]";
}
std::string tableJson(const Table2D& t, const char* ind) {
    const int nx = std::clamp((int)t.nx, 1, kTabMax), ny = std::clamp((int)t.ny, 1, kTabMax);
    std::string o = "{\n";
    o += std::string(ind) + "  \"x\": " + arr(t.x, nx) + ",\n";
    o += std::string(ind) + "  \"y\": " + arr(t.y, ny) + ",\n";
    o += std::string(ind) + "  \"locked\": " + (t.locked ? "true" : "false") + ",\n";
    o += std::string(ind) + "  \"v\": [\n";
    for (int r = 0; r < ny; ++r) o += std::string(ind) + "    " + arr(t.v[r], nx) + (r + 1 < ny ? ",\n" : "\n");
    o += std::string(ind) + "  ]\n" + ind + "}";
    return o;
}
std::string curveJson(const Curve1D& c) {
    const int n = std::clamp((int)c.n, 1, kCurveMax);
    return "{ \"x\": " + arr(c.x, n) + ", \"v\": " + arr(c.v, n) + ", \"locked\": " + (c.locked ? "true" : "false") + " }";
}

// scalar settings, one table drives write / read / compare
struct ScalarDef { const char* key; float Calibration::* f; int32_t Calibration::* i; };
const ScalarDef kScalars[] = {
    {"hybrid_tps_lo", &Calibration::hybridTpsLo, nullptr}, {"hybrid_tps_hi", &Calibration::hybridTpsHi, nullptr},
    {"power_tps_lo", &Calibration::powerTpsLo, nullptr},   {"power_tps_hi", &Calibration::powerTpsHi, nullptr},
    {"power_lambda", &Calibration::powerLambda, nullptr},  {"crank_advance", &Calibration::crankAdvance, nullptr},
    {"after_start_afr", &Calibration::afterStartAfr, nullptr}, {"after_start_time", &Calibration::afterStartTime, nullptr},
    {"accel_decay", &Calibration::accelDecay, nullptr},    {"accel_cold_mult", &Calibration::accelColdMult, nullptr},
    {"dfco_exit_enrich", &Calibration::dfcoExitEnrich, nullptr}, {"dfco_exit_decay", &Calibration::dfcoExitDecay, nullptr},
    {"injector_flow", &Calibration::injFlow, nullptr},
    {"async_tip_in", nullptr, &Calibration::asyncTipIn},   {"auto_tune", nullptr, &Calibration::autoTune},
    {"closed_loop", nullptr, &Calibration::closedLoop},
};
const char* kLoadModeNames[] = {"speed_density", "alpha_n", "maf", "hybrid"};

bool readArr(const JVal* j, float* out, int maxN, int& n) {
    if (!j || j->kind != JVal::Arr) return false;
    n = std::min((int)j->arr.size(), maxN);
    for (int i = 0; i < n; ++i) out[i] = (float)j->arr[i].num;
    return n > 0;
}
void readTable(const JVal* j, Table2D& t) {
    if (!j || j->kind != JVal::Obj) return;
    Table2D n = t;
    int nx = 0, ny = 0;
    if (!readArr(j->get("x"), n.x, kTabMax, nx) || !readArr(j->get("y"), n.y, kTabMax, ny)) return;
    const JVal* v = j->get("v");
    if (!v || v->kind != JVal::Arr || (int)v->arr.size() < ny) return;
    for (int r = 0; r < ny; ++r) { int k = 0; if (!readArr(&v->arr[r], n.v[r], kTabMax, k) || k < nx) return; }
    n.nx = nx; n.ny = ny;
    if (const JVal* l = j->get("locked")) n.locked = l->b || l->num != 0;
    t = n;
}
void readCurve(const JVal* j, Curve1D& c) {
    if (!j || j->kind != JVal::Obj) return;
    Curve1D n = c;
    int a = 0, b = 0;
    if (!readArr(j->get("x"), n.x, kCurveMax, a) || !readArr(j->get("v"), n.v, kCurveMax, b) || b < a) return;
    n.n = a;
    if (const JVal* l = j->get("locked")) n.locked = l->b || l->num != 0;
    c = n;
}
} // namespace

std::string calibrationToJson(const Calibration& c, const CalMeta& m) {
    std::string o = "{\n";
    o += "  \"format\": \"enginesim-calibration\",\n  \"format_version\": 1,\n";
    o += "  \"name\": \"" + esc(m.name) + "\",\n";
    o += "  \"engine\": \"" + esc(m.engine) + "\",\n";
    o += "  \"revision\": " + std::to_string(m.revision) + ",\n";
    o += "  \"saved\": \"" + esc(m.saved) + "\",\n";
    o += "  \"notes\": \"" + esc(m.notes) + "\",\n";
    o += "  \"load_mode\": \"" + std::string(kLoadModeNames[std::clamp((int)c.loadMode, 0, 3)]) + "\",\n";
    for (const ScalarDef& s : kScalars) o += "  \"" + std::string(s.key) + "\": " + num(s.f ? (double)(c.*(s.f)) : (double)(c.*(s.i))) + ",\n";
    o += "  \"cyl_fuel_trim\": " + arr(c.cylFuel, kCalMaxCyl) + ",\n";
    o += "  \"cyl_spark_trim\": " + arr(c.cylSpark, kCalMaxCyl) + ",\n";
    o += "  \"tables\": {\n";
    for (int i = 0; i < kCalTableCount; ++i)
        o += "    \"" + std::string(calTableInfo(i).key) + "\": " + tableJson(calTable(c, i), "    ") + (i + 1 < kCalTableCount ? ",\n" : "\n");
    o += "  },\n  \"curves\": {\n";
    for (int i = 0; i < kCalCurveCount; ++i)
        o += "    \"" + std::string(calCurveInfo(i).key) + "\": " + curveJson(calCurve(c, i)) + (i + 1 < kCalCurveCount ? ",\n" : "\n");
    o += "  }\n}\n";
    return o;
}

bool calibrationFromJson(const std::string& text, Calibration& c, CalMeta& m, std::string& err) {
    Parser ps{text.data(), text.data() + text.size(), {}};
    JVal root;
    if (!ps.val(root)) { err = "JSON error near byte " + std::to_string(ps.s - text.data()) + ": " + ps.err; return false; }
    if (root.kind != JVal::Obj) { err = "not a calibration file"; return false; }
    const JVal* f = root.get("format");
    if (!f || f->str != "enginesim-calibration") { err = "not an enginesim calibration (missing \"format\")"; return false; }
    Calibration n = c;
    if (const JVal* j = root.get("name")) m.name = j->str;
    if (const JVal* j = root.get("engine")) m.engine = j->str;
    if (const JVal* j = root.get("notes")) m.notes = j->str;
    if (const JVal* j = root.get("saved")) m.saved = j->str;
    if (const JVal* j = root.get("revision")) m.revision = (int)j->num;
    if (const JVal* j = root.get("load_mode"))
        for (int i = 0; i < 4; ++i) if (j->str == kLoadModeNames[i]) n.loadMode = (LoadMode)i;
    for (const ScalarDef& s : kScalars)
        if (const JVal* j = root.get(s.key)) {
            double v = j->kind == JVal::Bool ? (j->b ? 1.0 : 0.0) : j->num;
            if (!std::isfinite(v)) continue;
            if (s.f) n.*(s.f) = (float)v; else n.*(s.i) = (int32_t)v;
        }
    int k = 0;
    readArr(root.get("cyl_fuel_trim"), n.cylFuel, kCalMaxCyl, k);
    readArr(root.get("cyl_spark_trim"), n.cylSpark, kCalMaxCyl, k);
    if (const JVal* t = root.get("tables"))
        for (int i = 0; i < kCalTableCount; ++i) readTable(t->get(calTableInfo(i).key), calTable(n, i));
    if (const JVal* t = root.get("curves"))
        for (int i = 0; i < kCalCurveCount; ++i) readCurve(t->get(calCurveInfo(i).key), calCurve(n, i));
    c = n;
    return true;
}

// ------------------------------------------------------------------ validation
std::vector<CalIssue> validateCalibration(const Calibration& c, const EngineParams& p) {
    std::vector<CalIssue> out;
    auto add = [&](int sev, const char* fmt, auto... a) {
        char b[256]; std::snprintf(b, sizeof(b), fmt, a...); out.push_back({sev, b});
    };
    for (int i = 0; i < kCalTableCount; ++i) {
        const Table2D& t = calTable(c, i);
        const TableInfo& in = calTableInfo(i);
        if (t.nx < 2 || t.ny < 2 || t.nx > kTabMax || t.ny > kTabMax) { add(2, "%s: bad size %dx%d", in.name, t.nx, t.ny); continue; }
        for (int a = 1; a < t.nx; ++a) if (!(t.x[a] > t.x[a - 1])) { add(2, "%s: %s axis not increasing at column %d", in.name, in.xName, a + 1); break; }
        for (int a = 1; a < t.ny; ++a) if (!(t.y[a] > t.y[a - 1])) { add(2, "%s: %s axis not increasing at row %d", in.name, in.yName, a + 1); break; }
        int bad = 0, jumps = 0;
        float lo = 1e9f, hi = -1e9f;
        for (int r = 0; r < t.ny; ++r)
            for (int q = 0; q < t.nx; ++q) {
                float v = t.v[r][q];
                if (!std::isfinite(v) || v < in.lo || v > in.hi) ++bad;
                lo = std::min(lo, v); hi = std::max(hi, v);
                // a cell far from the average of its neighbours is probably a typo
                float s = 0; int nn = 0;
                if (q > 0) { s += t.v[r][q - 1]; ++nn; }
                if (q + 1 < t.nx) { s += t.v[r][q + 1]; ++nn; }
                if (r > 0) { s += t.v[r - 1][q]; ++nn; }
                if (r + 1 < t.ny) { s += t.v[r + 1][q]; ++nn; }
                if (nn && std::fabs(v - s / nn) > 0.25f * (in.hi - in.lo)) ++jumps;
            }
        if (bad) add(2, "%s: %d cell(s) outside %.4g .. %.4g %s", in.name, bad, in.lo, in.hi, in.unit);
        if (jumps) add(1, "%s: %d cell(s) differ strongly from their neighbours (typo?)", in.name, jumps);
        if (i == kCalLambda && hi > 1.05f && lo > 0.95f) add(1, "Target lambda: no rich region anywhere (max load runs lean of 1.05?)");
        if (i == kCalLambda) {
            float wot = (float)tableLookup(t, p.revLimit * 0.8, t.y[t.ny - 1]);
            if (wot > 0.95f) add(1, "Target lambda at full load %.2f - rich (~0.80-0.88) is safer for power / EGT", wot);
        }
        if (i == kCalIgnition) {
            if (hi > 50) add(1, "Ignition: up to %.1f deg advance - knock risk at load", hi);
            float last = t.x[t.nx - 1];
            if (last < p.revLimit) add(1, "Ignition: rpm axis ends at %.0f, below the rev limiter (%.0f) - last column is held", last, p.revLimit);
        }
        if ((i == kCalVeMap || i == kCalVeTps) && t.x[t.nx - 1] < p.revLimit)
            add(1, "%s: rpm axis ends at %.0f, below the rev limiter (%.0f)", in.name, t.x[t.nx - 1], p.revLimit);
    }
    for (int i = 0; i < kCalCurveCount; ++i) {
        const Curve1D& cv = calCurve(c, i);
        const TableInfo& in = calCurveInfo(i);
        if (cv.n < 2 || cv.n > kCurveMax) { add(2, "%s: bad size %d", in.name, cv.n); continue; }
        for (int a = 1; a < cv.n; ++a) if (!(cv.x[a] > cv.x[a - 1])) { add(2, "%s: axis not increasing at point %d", in.name, a + 1); break; }
        int bad = 0;
        for (int a = 0; a < cv.n; ++a) if (!std::isfinite(cv.v[a]) || cv.v[a] < in.lo || cv.v[a] > in.hi) ++bad;
        if (bad) add(2, "%s: %d point(s) outside %.4g .. %.4g %s", in.name, bad, in.lo, in.hi, in.unit);
    }
    if (!(c.injFlow > 20 && c.injFlow < 5000)) add(2, "Injector flow %.0f cc/min is implausible", c.injFlow);
    else if (std::fabs(c.injFlow / std::max(1.0f, p.injector_ccmin) - 1) > 0.03f)
        add(1, "Injector flow in the ECU (%.0f) differs from the fitted injector (%.0f cc/min) - fuelling scales by the ratio", c.injFlow, p.injector_ccmin);
    if (c.powerLambda < 0.65f || c.powerLambda > 1.0f) add(1, "Power enrichment lambda %.2f is unusual (0.78-0.9 typical)", c.powerLambda);
    if (c.hybridTpsHi <= c.hybridTpsLo && c.loadMode == LoadMode::Hybrid) add(2, "Hybrid load: TPS hi must be above TPS lo");
    if (c.powerTpsHi <= c.powerTpsLo) add(1, "Power enrichment: TPS hi must be above TPS lo");
    if (c.loadMode == LoadMode::Maf) add(0, "MAF load mode: fuel = measured air flow; VE maps are used only as a fallback");
    if (c.autoTune && (c.veMap.locked || c.veTps.locked)) add(0, "Auto-tune is on but a VE map is locked - the locked map is not learned");
    const int ncyl = std::clamp((int)p.cylinders, 1, kCalMaxCyl);
    for (int i = 0; i < ncyl; ++i) {
        if (std::fabs(c.cylFuel[i]) > 15) add(1, "Cylinder %d fuel trim %+.1f %% is large", i + 1, c.cylFuel[i]);
        if (std::fabs(c.cylSpark[i]) > 6) add(1, "Cylinder %d spark trim %+.1f deg is large", i + 1, c.cylSpark[i]);
    }
    if (out.empty()) add(0, "No problems found");
    return out;
}

// ------------------------------------------------------------------ compare
CalDiff diffTable(const Table2D& a, const Table2D& b) {
    CalDiff d;
    d.axesDiffer = a.nx != b.nx || a.ny != b.ny ||
                   std::memcmp(a.x, b.x, sizeof(float) * std::clamp((int)a.nx, 0, kTabMax)) != 0 ||
                   std::memcmp(a.y, b.y, sizeof(float) * std::clamp((int)a.ny, 0, kTabMax)) != 0;
    const int nx = std::clamp((int)b.nx, 1, kTabMax), ny = std::clamp((int)b.ny, 1, kTabMax);
    double s = 0;
    for (int r = 0; r < ny; ++r)
        for (int q = 0; q < nx; ++q) {
            // compare at b's breakpoints so different axes still give a sensible number
            float av = d.axesDiffer ? (float)tableLookup(a, b.x[q], b.y[r]) : a.v[r][q];
            float df = std::fabs(b.v[r][q] - av);
            if (df > 1e-4f) ++d.changed;
            d.maxAbs = std::max(d.maxAbs, df); s += df; ++d.cells;
        }
    d.meanAbs = d.cells ? (float)(s / d.cells) : 0;
    return d;
}

CalDiff diffCurve(const Curve1D& a, const Curve1D& b) {
    CalDiff d;
    d.axesDiffer = a.n != b.n || std::memcmp(a.x, b.x, sizeof(float) * std::clamp((int)a.n, 0, kCurveMax)) != 0;
    double s = 0;
    for (int i = 0; i < std::clamp((int)b.n, 1, kCurveMax); ++i) {
        float av = d.axesDiffer ? (float)curveLookup(a, b.x[i]) : a.v[i];
        float df = std::fabs(b.v[i] - av);
        if (df > 1e-4f) ++d.changed;
        d.maxAbs = std::max(d.maxAbs, df); s += df; ++d.cells;
    }
    d.meanAbs = d.cells ? (float)(s / d.cells) : 0;
    return d;
}

int diffScalars(const Calibration& a, const Calibration& b, std::vector<std::string>* names) {
    int n = 0;
    auto note = [&](const std::string& s) { ++n; if (names) names->push_back(s); };
    if (a.loadMode != b.loadMode) note("load_mode");
    for (const ScalarDef& s : kScalars) {
        double va = s.f ? (double)(a.*(s.f)) : (double)(a.*(s.i));
        double vb = s.f ? (double)(b.*(s.f)) : (double)(b.*(s.i));
        if (std::fabs(va - vb) > 1e-6) note(std::string(s.key) + ": " + num(va) + " -> " + num(vb));
    }
    for (int i = 0; i < kCalMaxCyl; ++i) {
        if (a.cylFuel[i] != b.cylFuel[i]) note("cyl " + std::to_string(i + 1) + " fuel trim: " + num(a.cylFuel[i]) + " -> " + num(b.cylFuel[i]));
        if (a.cylSpark[i] != b.cylSpark[i]) note("cyl " + std::to_string(i + 1) + " spark trim: " + num(a.cylSpark[i]) + " -> " + num(b.cylSpark[i]));
    }
    return n;
}

// ------------------------------------------------------------------ files
static std::string g_calDir = "calibrations";
void setCalibrationDir(const std::string& dir) { g_calDir = dir; }
std::string calibrationDir() { return g_calDir; }

std::string engineFolderName(const char* engineName) {
    std::string o;
    for (const char* c = engineName; c && *c; ++c) {
        unsigned char u = (unsigned char)*c;
        if (std::isalnum(u) || u == '-' || u == '.') o += (char)u;
        else if (!o.empty() && o.back() != '_') o += '_';
    }
    while (!o.empty() && (o.back() == '_' || o.back() == '.')) o.pop_back();
    return o.empty() ? "engine" : o;
}

static fs::path enginePath(const std::string& engine) { return fs::u8path(g_calDir) / fs::u8path(engineFolderName(engine.c_str())); }

bool writeTextFile(const std::string& path, const std::string& text) {
    std::error_code ec;
    fs::path p = fs::u8path(path);
    if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);
    fs::path tmp = p; tmp += ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f) return false;
        f << text;
        if (!f) return false;
    }
    fs::rename(tmp, p, ec);   // atomic replace: a crash never leaves a half-written calibration
    if (ec) { fs::remove(p, ec); fs::rename(tmp, p, ec); }
    return !ec;
}

bool readTextFile(const std::string& path, std::string& text) {
    std::ifstream f(fs::u8path(path), std::ios::binary);
    if (!f) return false;
    std::stringstream ss; ss << f.rdbuf();
    text = ss.str();
    return true;
}

static CalFileInfo infoOf(const fs::path& p) {
    CalFileInfo i;
    i.path = p.u8string(); i.name = p.stem().u8string();
    std::string t;
    if (readTextFile(i.path, t)) {
        Calibration c{}; CalMeta m; std::string err;
        if (calibrationFromJson(t, c, m, err)) { i.readable = true; i.saved = m.saved; i.revision = m.revision; i.notes = m.notes; if (!m.name.empty() && p.parent_path().extension() != ".history") i.name = m.name; }
    }
    return i;
}

std::vector<CalFileInfo> listCalibrations(const std::string& engine) {
    std::vector<CalFileInfo> v;
    std::error_code ec;
    for (auto& e : fs::directory_iterator(enginePath(engine), ec))
        if (e.is_regular_file() && e.path().extension() == ".json") { CalFileInfo i = infoOf(e.path()); i.name = e.path().stem().u8string(); v.push_back(i); }
    std::sort(v.begin(), v.end(), [](const CalFileInfo& a, const CalFileInfo& b) { return a.name < b.name; });
    return v;
}

std::vector<CalFileInfo> listHistory(const std::string& engine, const std::string& name) {
    std::vector<CalFileInfo> v;
    std::error_code ec;
    for (auto& e : fs::directory_iterator(enginePath(engine) / fs::u8path(name + ".history"), ec))
        if (e.is_regular_file() && e.path().extension() == ".json") v.push_back(infoOf(e.path()));
    std::sort(v.begin(), v.end(), [](const CalFileInfo& a, const CalFileInfo& b) { return a.revision > b.revision; });
    return v;
}

static bool validName(const std::string& n) {
    if (n.empty() || n.size() > 64) return false;
    for (char c : n) if (std::strchr("<>:\"/\\|?*", c) || (unsigned char)c < 0x20) return false;
    return n != "." && n != "..";
}

bool saveCalibrationFile(const std::string& engine, CalMeta& m, const Calibration& c, std::string& err) {
    if (!validName(m.name)) { err = "invalid name (no / \\ : * ? \" < > |)"; return false; }
    m.engine = engine;
    // next revision = one past the newest kept in history
    int rev = m.revision;
    for (const CalFileInfo& h : listHistory(engine, m.name)) rev = std::max(rev, h.revision);
    m.revision = rev + 1;
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char ts[32]; std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);
    m.saved = ts;
    const std::string text = calibrationToJson(c, m);
    const fs::path dir = enginePath(engine);
    char hist[32]; std::snprintf(hist, sizeof(hist), "r%04d.json", m.revision);
    if (!writeTextFile((dir / fs::u8path(m.name + ".json")).u8string(), text) ||
        !writeTextFile((dir / fs::u8path(m.name + ".history") / hist).u8string(), text)) {
        err = "could not write to " + dir.u8string();
        return false;
    }
    return true;
}

bool loadCalibrationFile(const std::string& path, Calibration& c, CalMeta& m, std::string& err) {
    std::string t;
    if (!readTextFile(path, t)) { err = "cannot read " + path; return false; }
    return calibrationFromJson(t, c, m, err);
}

bool deleteCalibrationFile(const std::string& engine, const std::string& name) {
    if (!validName(name)) return false;
    std::error_code ec;
    const fs::path dir = enginePath(engine);
    bool ok = fs::remove(dir / fs::u8path(name + ".json"), ec);
    fs::remove_all(dir / fs::u8path(name + ".history"), ec);
    return ok;
}

} // namespace es
