#include "ecu/CalibrationIO.h"
#include "ecu/EngineFile.h"
#include "sim/Fuel.h"
#include "core/MathUtil.h"
#include "sim/EngineParams.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstring>
#include <ctime>
#include <cstddef>
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
    // cam control
    {"vtec_rpm", &Calibration::vtecRpm, nullptr}, {"vtec_hyst", &Calibration::vtecHyst, nullptr},
    {"vtec_min_map", &Calibration::vtecMinMap, nullptr}, {"vtec_min_ect", &Calibration::vtecMinEct, nullptr},
    // closed-loop lambda
    {"stft_gain", &Calibration::stftGain, nullptr}, {"stft_limit", &Calibration::stftLimit, nullptr},
    {"ltft_learn", nullptr, &Calibration::ltftLearn}, {"ltft_rate", &Calibration::ltftRate, nullptr},
    {"ltft_limit", &Calibration::ltftLimit, nullptr}, {"cl_min_ect", &Calibration::clMinEct, nullptr},
    {"cl_delay", &Calibration::clDelay, nullptr}, {"cl_max_map", &Calibration::clMaxMap, nullptr},
    // knock control
    {"knock_control", nullptr, &Calibration::knockControl}, {"knock_threshold", &Calibration::knockThreshold, nullptr},
    {"knock_retard_step", &Calibration::knockRetardStep, nullptr}, {"knock_max_retard", &Calibration::knockMaxRetard, nullptr},
    {"knock_recover", &Calibration::knockRecover, nullptr}, {"knock_min_rpm", &Calibration::knockMinRpm, nullptr},
    // idle / throttle / limiter
    {"idle_kp", &Calibration::idleKp, nullptr}, {"idle_ki", &Calibration::idleKi, nullptr}, {"idle_kd", &Calibration::idleKd, nullptr},
    {"idle_spark_gain", &Calibration::idleSparkGain, nullptr}, {"idle_spark_min", &Calibration::idleSparkMin, nullptr},
    {"idle_spark_max", &Calibration::idleSparkMax, nullptr},
    {"soft_limit_band", &Calibration::softLimitBand, nullptr}, {"soft_limit_retard", &Calibration::softLimitRetard, nullptr},
    {"soft_limit_cut", &Calibration::softLimitCut, nullptr},
    // boost control
    {"boost_control", nullptr, &Calibration::boostControl}, {"boost_kp", &Calibration::boostKp, nullptr},
    {"boost_ki", &Calibration::boostKi, nullptr}, {"boost_kd", &Calibration::boostKd, nullptr},
    {"wg_duty_max", &Calibration::wgDutyMax, nullptr}, {"overboost_margin", &Calibration::overboostMargin, nullptr},
    {"overboost_time", &Calibration::overboostTime, nullptr}, {"boost_cut_bar", &Calibration::boostCutBar, nullptr},
    // protection actions
    {"limp_throttle", &Calibration::limpThrottle, nullptr}, {"limp_rpm", &Calibration::limpRpm, nullptr},
    {"prot_retard", &Calibration::protRetard, nullptr},
    // motorsport: launch / 3-step / flat shift / shift cut
    {"launch_control", nullptr, &Calibration::launchControl}, {"launch_rpm", &Calibration::launchRpm, nullptr},
    {"launch_timing", &Calibration::launchTiming, nullptr}, {"launch_min_tps", &Calibration::launchMinTps, nullptr},
    {"launch_max_speed", &Calibration::launchMaxSpeed, nullptr}, {"launch_cut_band", &Calibration::launchCutBand, nullptr},
    {"launch_cut_mode", nullptr, &Calibration::launchCutMode}, {"launch_enrich", &Calibration::launchEnrich, nullptr},
    {"clutch_moving_mode", nullptr, &Calibration::clutchMovingMode}, {"three_step_rpm", &Calibration::threeStepRpm, nullptr},
    {"flat_shift_margin", &Calibration::flatShiftMargin, nullptr}, {"flat_shift_retard", &Calibration::flatShiftRetard, nullptr},
    {"shift_cut", nullptr, &Calibration::shiftCut}, {"shift_cut_time", &Calibration::shiftCutTime, nullptr},
    {"shift_cut_min_tps", &Calibration::shiftCutMinTps, nullptr},
    // anti-lag
    {"als_timing", &Calibration::alsTiming, nullptr}, {"als_throttle", &Calibration::alsThrottle, nullptr},
    {"als_max_time", &Calibration::alsMaxTime, nullptr}, {"als_min_rpm", &Calibration::alsMinRpm, nullptr},
    {"rolling_als", nullptr, &Calibration::rollingAls}, {"rolling_als_min_speed", &Calibration::rollingAlsMinSpeed, nullptr},
    // traction control
    {"traction_control", nullptr, &Calibration::tractionControl}, {"tc_kp", &Calibration::tcKp, nullptr},
    {"tc_ki", &Calibration::tcKi, nullptr}, {"tc_max_retard", &Calibration::tcMaxRetard, nullptr},
    {"tc_max_cut", &Calibration::tcMaxCut, nullptr},
    // nitrous controller
    {"n2o_min_tps", &Calibration::n2oMinTps, nullptr}, {"n2o_min_rpm", &Calibration::n2oMinRpm, nullptr},
    {"n2o_max_rpm", &Calibration::n2oMaxRpm, nullptr}, {"n2o_min_ect", &Calibration::n2oMinEct, nullptr},
    {"n2o_min_speed", &Calibration::n2oMinSpeed, nullptr}, {"n2o_delay", &Calibration::n2oDelay, nullptr},
    {"n2o_ramp_time", &Calibration::n2oRampTime, nullptr}, {"n2o_start_pct", &Calibration::n2oStartPct, nullptr},
    {"n2o_retard", &Calibration::n2oRetard, nullptr}, {"n2o_flow_gs", &Calibration::n2oFlowGs, nullptr},
    {"n2o_lean_lambda", &Calibration::n2oLeanLambda, nullptr},
    {"dfco_resume_above", &Calibration::dfcoResumeAbove, nullptr}, {"dfco_anticipate", &Calibration::dfcoAnticipate, nullptr},
    {"dfco_delay", &Calibration::dfcoDelay, nullptr},
    // ECU configuration, torque management, gear-based control (Phase 7)
    {"trigger_pattern", nullptr, &Calibration::triggerPattern}, {"cam_sensor", nullptr, &Calibration::camSensor},
    {"injection_mode", nullptr, &Calibration::injMode}, {"dwell_max_duty", &Calibration::dwellMaxDuty, nullptr},
    {"torque_mgmt", nullptr, &Calibration::torqueMgmt}, {"tq_max_retard", &Calibration::tqMaxRetard, nullptr},
    {"tq_max_cut", &Calibration::tqMaxCut, nullptr}, {"tq_throttle_rate", &Calibration::tqThrottleRate, nullptr},
    {"gear_control", nullptr, &Calibration::gearControl},
    // fuel and flex fuel (Package 1)
    {"stoich_gasoline", &Calibration::stoichGasoline, nullptr}, {"stoich_alcohol", &Calibration::stoichAlcohol, nullptr},
    {"flex_fallback_pct", &Calibration::flexFallbackPct, nullptr},
    // diesel (Package 4)
    {"smoke_lambda", &Calibration::smokeLambda, nullptr}, {"diesel_idle_kp", &Calibration::dieselIdleKp, nullptr},
    {"diesel_idle_ki", &Calibration::dieselIdleKi, nullptr}, {"diesel_gov_band", &Calibration::dieselGovBand, nullptr},
    {"diesel_start_fuel", &Calibration::dieselStartFuel, nullptr},
    {"afterglow_time", &Calibration::afterglowTime, nullptr}, {"afterglow_ect", &Calibration::afterglowEct, nullptr},
    {"egr_min_ect", &Calibration::egrMinEct, nullptr},
};
const char* kActKeys[kActCount] = {"warn", "retard", "boost_reduce", "throttle_limit", "fuel_limit", "limp", "shutdown"};
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
    o += "  \"protection\": {\n";
    for (int i = 0; i < kProtCount; ++i) {
        const ProtRule& r = c.prot[i];
        o += "    \"" + std::string(protInfo(i).key) + "\": { \"enabled\": " + (r.enabled ? "true" : "false") +
             ", \"warn\": " + num(r.warn) + ", \"limit\": " + num(r.limit) + ", \"delay\": " + num(r.delay) +
             ", \"recover\": " + num(r.recover) + ", \"action\": \"" + kActKeys[std::clamp((int)r.action, 0, kActCount - 1)] + "\" }" +
             (i + 1 < kProtCount ? ",\n" : "\n");
    }
    o += "  },\n";
    o += "  \"tables\": {\n";
    for (int i = 0; i < kCalTableCount; ++i)
        o += "    \"" + std::string(calTableInfo(i).key) + "\": " + tableJson(calTable(c, i), "    ") + (i + 1 < kCalTableCount ? ",\n" : "\n");
    o += "  },\n  \"curves\": {\n";
    for (int i = 0; i < kCalCurveCount; ++i)
        o += "    \"" + std::string(calCurveInfo(i).key) + "\": " + curveJson(calCurve(c, i)) + (i + 1 < kCalCurveCount ? ",\n" : "\n");
    o += "  }\n}\n";
    return o;
}

static void calibrationFromRoot(const JVal& root, Calibration& c, CalMeta& m);
bool calibrationFromJson(const std::string& text, Calibration& c, CalMeta& m, std::string& err) {
    Parser ps{text.data(), text.data() + text.size(), {}};
    JVal root;
    if (!ps.val(root)) { err = "JSON error near byte " + std::to_string(ps.s - text.data()) + ": " + ps.err; return false; }
    if (root.kind != JVal::Obj) { err = "not a calibration file"; return false; }
    const JVal* f = root.get("format");
    if (!f || f->str != "enginesim-calibration") { err = "not an enginesim calibration (missing \"format\")"; return false; }
    calibrationFromRoot(root, c, m);
    return true;
}

static void calibrationFromRoot(const JVal& root, Calibration& c, CalMeta& m) {
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
    if (const JVal* pr = root.get("protection"))
        for (int i = 0; i < kProtCount; ++i) {
            const JVal* r = pr->get(protInfo(i).key);
            if (!r || r->kind != JVal::Obj) continue;
            ProtRule& d = n.prot[i];
            if (const JVal* v = r->get("enabled")) d.enabled = v->b || v->num != 0;
            if (const JVal* v = r->get("warn")) d.warn = (float)v->num;
            if (const JVal* v = r->get("limit")) d.limit = (float)v->num;
            if (const JVal* v = r->get("delay")) d.delay = (float)v->num;
            if (const JVal* v = r->get("recover")) d.recover = (float)v->num;
            if (const JVal* v = r->get("action"))
                for (int a = 0; a < kActCount; ++a) if (v->str == kActKeys[a]) d.action = a;
        }
    if (const JVal* t = root.get("tables"))
        for (int i = 0; i < kCalTableCount; ++i) readTable(t->get(calTableInfo(i).key), calTable(n, i));
    if (const JVal* t = root.get("curves"))
        for (int i = 0; i < kCalCurveCount; ++i) readCurve(t->get(calCurveInfo(i).key), calCurve(n, i));
    c = n;
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
    // the fuel the ECU assumes against the fuel the engine is filled with
    if (p.fuelSystem == FuelSystem::EFI) {
        const FuelProps f = fuelProps(fuelTypeBlend(p.fuelType));
        const double v = (p.flexSensor ? f.alcoholPct : (double)c.flexFallbackPct) * 0.01;
        const double mg = (1.0 - v) * 0.745, ma = v * 0.790, wa = ma / std::max(1e-6, mg + ma);
        const double st = (1.0 - wa) * c.stoichGasoline + wa * c.stoichAlcohol;
        if (std::fabs(st / f.stoich - 1.0) > 0.04)
            add(p.flexSensor ? 1 : 2, "%s: the ECU calculates with a stoich AFR of %.2f, the fuel needs %.2f - it runs %s by %.0f %%%s",
                fuelTypeName(p.fuelType), st, f.stoich, st > f.stoich ? "lean" : "rich", std::fabs(st / f.stoich - 1.0) * 100.0,
                p.flexSensor ? " (alcohol stoich setting)" : " (no flex fuel sensor: set the fallback alcohol %)");
    }
    if (c.powerLambda < 0.65f || c.powerLambda > 1.0f) add(1, "Power enrichment lambda %.2f is unusual (0.78-0.9 typical)", c.powerLambda);
    if (c.hybridTpsHi <= c.hybridTpsLo && c.loadMode == LoadMode::Hybrid) add(2, "Hybrid load: TPS hi must be above TPS lo");
    if (c.powerTpsHi <= c.powerTpsLo) add(1, "Power enrichment: TPS hi must be above TPS lo");
    if (c.stftLimit < 0 || c.stftLimit > 50 || c.ltftLimit < 0 || c.ltftLimit > 50) add(1, "Fuel trim limits beyond +-50 %% hide real problems");
    if (c.knockControl && c.knockThreshold <= 0.2f) add(1, "Knock threshold %.2f is below the sensor noise floor - constant false retard", c.knockThreshold);
    if (c.softLimitBand < 0 || c.softLimitBand > 2000) add(1, "Soft limiter band %.0f rpm is unusual (0-500 typical)", c.softLimitBand);
    if (c.boostControl) {
        if (p.induction != Induction::Turbo) add(1, "Boost control is on but the engine has no turbo - it does nothing");
        float hi = 0;
        for (int r = 0; r < c.boostTarget.ny; ++r) for (int q = 0; q < c.boostTarget.nx; ++q) hi = std::max(hi, c.boostTarget.v[r][q]);
        if (hi >= c.boostCutBar) add(2, "Boost target %.2f bar reaches the boost cut (%.2f bar)", hi, c.boostCutBar);
        if (p.induction == Induction::Turbo && hi < p.wastegateBar - 0.05f)
            add(0, "Boost target %.2f bar is below the wastegate spring (%.2f bar): the ECU cannot hold it lower", hi, p.wastegateBar);
    }
    for (int i = 0; i < kProtCount; ++i) {
        const ProtRule& r = c.prot[i];
        if (!r.enabled) continue;
        const ProtInfo& in = protInfo(i);
        if (in.high ? r.warn > r.limit : r.warn < r.limit) add(1, "Protection '%s': warning %.4g is past the limit %.4g", in.name, r.warn, r.limit);
        if (r.delay < 0 || r.recover < 0) add(2, "Protection '%s': negative delay / recovery", in.name);
    }
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
    for (int i = 0; i < kProtCount; ++i) {
        const ProtRule &x = a.prot[i], &y = b.prot[i];
        if (x.enabled != y.enabled || x.warn != y.warn || x.limit != y.limit || x.delay != y.delay || x.recover != y.recover || x.action != y.action)
            note(std::string("protection ") + protInfo(i).key + ": " + (x.enabled ? "" : "(off) ") + num(x.limit) + " " + protActionName(x.action) +
                 " -> " + (y.enabled ? "" : "(off) ") + num(y.limit) + " " + protActionName(y.action));
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

// ================================================================== engine files
namespace {
enum class EfType { F, I, FA, IA };
struct EngineField { const char* key; size_t off; EfType t; int n; };
#define EF(k, m) {k, offsetof(EngineParams, m), EfType::F, 1}
#define EI(k, m) {k, offsetof(EngineParams, m), EfType::I, 1}
#define EFA(k, m, n) {k, offsetof(EngineParams, m), EfType::FA, n}
#define EIA(k, m, n) {k, offsetof(EngineParams, m), EfType::IA, n}
#define CF(k, m) {k, offsetof(EngineParams, comp) + offsetof(ComponentSet, m), EfType::F, 1}
#define CI(k, m) {k, offsetof(EngineParams, comp) + offsetof(ComponentSet, m), EfType::I, 1}
// the engine and the car, not the test bench (throttle, key, dyno, ambient air, listener, sensor faults ...)
const EngineField kEngineFields[] = {
    // geometry / crank
    EI("engine_type", engineType), EI("cylinders", cylinders), EI("layout", layout), EF("bank_angle", bankAngle),
    EF("bore_mm", bore_mm), EF("stroke_mm", stroke_mm), EF("rod_mm", rod_mm), EF("compression", compression),
    EF("flywheel", flywheel), EIA("firing_order", firingOrder, kMaxCyl), EI("even_fire", evenFire),
    EFA("phase", phase, kMaxCyl), EIA("bank", bank, kMaxCyl), EF("imperfection", imperfection),
    // valvetrain
    EI("intake_valves", intakeValves), EI("exhaust_valves", exhaustValves),
    EF("intake_valve_dia_mm", intakeValveDia_mm), EF("exhaust_valve_dia_mm", exhaustValveDia_mm),
    EF("intake_duration", intakeDuration), EF("exhaust_duration", exhaustDuration),
    EF("intake_lift_mm", intakeLift_mm), EF("exhaust_lift_mm", exhaustLift_mm),
    EF("intake_centerline", intakeCenterline), EF("exhaust_centerline", exhaustCenterline),
    EF("intake_cam_advance", intakeCamAdvance), EF("exhaust_cam_advance", exhaustCamAdvance), EF("valve_lash_mm", valveLash_mm),
    EI("vtec", vtec), EF("vtec_in_duration", vtecInDuration), EF("vtec_in_lift_mm", vtecInLift_mm),
    EF("vtec_ex_duration", vtecExDuration), EF("vtec_ex_lift_mm", vtecExLift_mm), EI("vtc", vtc), EF("vtc_range", vtcRange),
    // ignition / limiter
    EF("base_advance", baseAdvance), EF("mech_advance", mechAdvance), EF("mech_advance_rpm", mechAdvanceRpm),
    EF("vacuum_advance", vacuumAdvance), EF("rev_limit", revLimit), EF("redline", redline), EF("friction_mult", frictionMult),
    EI("limiter", limiter),
    // intake
    EF("throttle_bore_mm", throttleBore_mm), EI("itb", itb), EF("plenum_l", plenum_L), EF("runner_len_mm", runnerLen_mm),
    EF("runner_dia_mm", runnerDia_mm), EF("air_filter", airFilter),
    // fuel
    EI("fuel_system", fuelSystem), EI("injection", injection), EF("injector_ccmin", injector_ccmin), EF("afr_cruise", afrCruise),
    EF("afr_wot", afrWot), EI("ve_learning", veLearning), EF("injection_angle", injectionAngle), EF("idle_target_rpm", idleTargetRpm),
    EI("carb_barrels", carbBarrels), EF("venturi_mm", venturi_mm), EF("main_jet", mainJet), EF("idle_mixture", idleMixture),
    EF("idle_screw", idleScrew), EF("accel_pump", accelPump), EF("choke", choke),
    EI("fuel_type", fuelType), EI("flex_sensor", flexSensor),
    EF("diesel_inj_rate", dieselInjRate), EF("diesel_max_fuel", dieselMaxFuel), EF("soi_base", soiBase), EF("soi_advance", soiAdvance),
    EI("glow_plugs", glowPlugs), EF("egr_valve_dia_mm", egrValveDia_mm), EI("vgt", vgt), EF("vgt_min_area", vgtMinArea),
    EI("emap_sensor", emapSensor), EI("wideband_pos", widebandPos), EI("egt_per_cyl", egtPerCyl), EI("egt_post_turbo", egtPostTurbo),
    // exhaust
    EI("header", header), EFA("primary_len_mm", primaryLen_mm, kMaxCyl), EF("primary_dia_mm", primaryDia_mm),
    EF("secondary_len_mm", secondaryLen_mm), EF("collector_len_mm", collectorLen_mm), EF("collector_dia_mm", collectorDia_mm),
    EI("crossover", crossover), EF("pipe_dia_mm", pipeDia_mm), EF("midpipe_len_mm", midpipeLen_mm), EI("cat", cat),
    EI("resonator", resonator), EI("muffler", muffler), EF("tailpipe_len_mm", tailpipeLen_mm),
    EF("resonator_hz", resonatorHz), EI("muffler_ext_tubes", mufflerExtTubes),
    // rotary
    EF("rotor_radius_mm", rotorRadius_mm), EF("eccentricity_mm", eccentricity_mm), EF("rotor_width_mm", rotorWidth_mm),
    EF("r_intake_open", rIntakeOpen), EF("r_intake_close", rIntakeClose), EF("r_intake_area_cm2", rIntakeArea_cm2),
    EF("r_exhaust_open", rExhaustOpen), EF("r_exhaust_close", rExhaustClose), EF("r_exhaust_area_cm2", rExhaustArea_cm2),
    EF("apex_leak", apexLeak),
    // forced induction
    EI("induction", induction), EI("sc_type", scType), EF("comp_flow_gs", compFlow_gs), EF("turbine_ar", turbineAR),
    EF("turbo_inertia", turboInertia), EF("wastegate_bar", wastegateBar), EI("intercooler", intercooler),
    EF("ic_efficiency", icEfficiency), EF("charge_volume_l", chargeVolume_L), EI("bov", bov), EI("anti_lag", antiLag),
    EF("boost_retard", boostRetard), EF("afr_boost", afrBoost), EF("sc_displacement_l", scDisplacement_L),
    EF("sc_ratio", scRatio), EI("sc_bypass", scBypass),
    // vehicle
    EI("trans_mode", transMode), EI("gears", gears), EFA("gear_ratio", gearRatio, kMaxGears), EF("final_drive", finalDrive),
    EF("mass_kg", mass_kg), EF("cda", cdA), EF("crr", crr), EF("tire_radius_mm", tireRadius_mm), EF("grip", grip),
    EF("drive_frac", driveFrac), EF("clutch_torque", clutchTorque), EF("driveline_eff", drivelineEff),
    // nitrous hardware
    EI("nitrous", nitrous), EF("n2o_shot_hp", n2oShotHp), EI("n2o_wet", n2oWet), EF("n2o_bottle_kg", n2oBottleKg),
    // sound character
    EF("exhaust_rasp", exhaustRasp), EF("exhaust_punch", exhaustPunch), EF("exhaust_steep", exhaustSteep),
    EI("sound_style", soundStyle), EF("pressure_mix", pressureMix), EF("smooth_hz", smoothHz), EI("ir_choice", irChoice),
    EF("ir_mix", irMix),
    // ECU hardware
    EF("ecu_rate_hz", ecuRateHz),
    // components
    CI("piston_type", pistonType), CF("piston_mass_g", pistonMass_g), CF("ring_tension", ringTension), CI("rod_type", rodType),
    CF("rod_mass_g", rodMass_g), CI("crank_type", crankType), CI("lifter", lifter), CF("spring_seat_n", springSeat_N),
    CF("spring_open_n", springOpen_N), CI("throttle_type", throttleType), CF("dbw_response_ms", dbwResponse_ms),
    CF("inj_dead_time_ms", injDeadTime_ms), CF("fuel_pressure_bar", fuelPressure_bar), CI("fuel_regulator", fuelRegulator),
    CF("fuel_pump_lph", fuelPumpLph), CI("coil_type", coilType), CF("coil_energy_mj", coilEnergy_mJ),
    CF("coil_charge_ms", coilCharge_ms), CI("plug_heat_range", plugHeatRange), CF("plug_gap_mm", plugGap_mm),
    CF("oil_pump_capacity", oilPumpCapacity), CF("oil_relief_bar", oilRelief_bar), CF("water_pump_capacity", waterPumpCapacity),
};
#undef EF
#undef EI
#undef EFA
#undef EIA
#undef CF
#undef CI
static_assert(sizeof(Layout) == 4 && sizeof(EngineType) == 4 && sizeof(Induction) == 4 && sizeof(PistonType) == 4 &&
              sizeof(int) == 4, "engine file fields assume 32-bit enums");

std::string g_engineDir = "engines";

std::string arrI(const int32_t* a, int n) {
    std::string o = "[";
    for (int i = 0; i < n; ++i) { if (i) o += ", "; o += std::to_string(a[i]); }
    return o + "]";
}
std::string turboJson(const TurboMaps& t) {
    std::string o = "{\n";
    o += "    \"use_maps\": " + std::to_string(t.useMaps) + ",\n";
    o += "    \"compressor_name\": \"" + esc(t.compName) + "\",\n";
    o += "    \"turbine_name\": \"" + esc(t.turbName) + "\",\n";
    o += "    \"compressor_choice\": " + std::to_string(t.compChoice) + ", \"turbine_choice\": " + std::to_string(t.turbChoice) + ",\n";
    const CompressorMap& c = t.comp;
    const int nl = std::clamp((int)c.nLines, 0, kCompLines);
    o += "    \"compressor\": { \"speed\": " + arr(c.speed, nl) + ", \"points\": " + arrI(c.nPts, nl) + ",\n";
    std::string fl = "[", pr = "[", ef = "[";
    for (int k = 0; k < nl; ++k) {
        const int np = std::clamp((int)c.nPts[k], 0, kCompPts);
        if (k) { fl += ", "; pr += ", "; ef += ", "; }
        fl += arr(c.flow[k], np); pr += arr(c.pr[k], np); ef += arr(c.eff[k], np);
    }
    o += "      \"flow\": " + fl + "],\n      \"pr\": " + pr + "],\n      \"eff\": " + ef + "] },\n";
    const TurbineMap& b = t.turb;
    const int nf = std::clamp((int)b.nFlow, 0, kTurbPts), ne = std::clamp((int)b.nEff, 0, kTurbPts);
    o += "    \"turbine\": { \"er\": " + arr(b.er, nf) + ", \"mfp\": " + arr(b.mfp, nf) + ", \"uc\": " + arr(b.uc, ne) +
         ", \"eff\": " + arr(b.eff, ne) + ", \"wheel_dia_mm\": " + num(b.wheelDia_mm) + " }\n  }";
    return o;
}
void readTurbo(const JVal* j, TurboMaps& t) {
    if (!j || j->kind != JVal::Obj) return;
    if (const JVal* v = j->get("use_maps")) t.useMaps = (int32_t)v->num;
    if (const JVal* v = j->get("compressor_name")) std::snprintf(t.compName, sizeof(t.compName), "%s", v->str.c_str());
    if (const JVal* v = j->get("turbine_name")) std::snprintf(t.turbName, sizeof(t.turbName), "%s", v->str.c_str());
    if (const JVal* v = j->get("compressor_choice")) t.compChoice = (int32_t)v->num;
    if (const JVal* v = j->get("turbine_choice")) t.turbChoice = (int32_t)v->num;
    if (const JVal* c = j->get("compressor"); c && c->kind == JVal::Obj) {
        CompressorMap m; std::memset(&m, 0, sizeof(m));
        int nl = 0, np = 0;
        float pts[kCompLines] = {};
        readArr(c->get("speed"), m.speed, kCompLines, nl);
        readArr(c->get("points"), pts, kCompLines, np);
        const JVal *fl = c->get("flow"), *pr = c->get("pr"), *ef = c->get("eff");
        bool ok = nl >= 2 && np == nl && fl && pr && ef && fl->kind == JVal::Arr && pr->kind == JVal::Arr && ef->kind == JVal::Arr &&
                  (int)fl->arr.size() >= nl && (int)pr->arr.size() >= nl && (int)ef->arr.size() >= nl;
        for (int k = 0; ok && k < nl; ++k) {
            int a = 0, b = 0, d = 0;
            readArr(&fl->arr[k], m.flow[k], kCompPts, a); readArr(&pr->arr[k], m.pr[k], kCompPts, b); readArr(&ef->arr[k], m.eff[k], kCompPts, d);
            m.nPts[k] = std::min(std::min(a, b), std::min(d, (int)pts[k]));
        }
        if (ok) { m.nLines = nl; t.comp = m; }
    }
    if (const JVal* b = j->get("turbine"); b && b->kind == JVal::Obj) {
        TurbineMap m; std::memset(&m, 0, sizeof(m));
        int a = 0, c = 0, d = 0, e = 0;
        readArr(b->get("er"), m.er, kTurbPts, a); readArr(b->get("mfp"), m.mfp, kTurbPts, c);
        readArr(b->get("uc"), m.uc, kTurbPts, d); readArr(b->get("eff"), m.eff, kTurbPts, e);
        m.nFlow = std::min(a, c); m.nEff = std::min(d, e);
        if (const JVal* v = b->get("wheel_dia_mm")) m.wheelDia_mm = (float)v->num;
        if (m.nFlow >= 2 && m.nEff >= 1) t.turb = m;
    }
}
std::string nowStamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char ts[32]; std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);
    return ts;
}
} // namespace

std::string engineToJson(const EngineParams& p, const EngineMeta& m, bool withCalibration) {
    const char* base = reinterpret_cast<const char*>(&p);
    std::string o = "{\n";
    o += "  \"format\": \"enginesim-engine\",\n  \"format_version\": 1,\n";
    o += "  \"name\": \"" + esc(p.name) + "\",\n";
    o += "  \"author\": \"" + esc(m.author) + "\",\n";
    o += "  \"notes\": \"" + esc(m.notes) + "\",\n";
    o += "  \"saved\": \"" + esc(m.saved) + "\",\n";
    o += "  \"app\": \"" + esc(m.app) + "\",\n";
    o += "  \"engine\": {\n";
    for (const EngineField& f : kEngineFields) {
        o += "    \"" + std::string(f.key) + "\": ";
        switch (f.t) {
        case EfType::F: o += num(*reinterpret_cast<const float*>(base + f.off)); break;
        case EfType::I: o += std::to_string(*reinterpret_cast<const int32_t*>(base + f.off)); break;
        case EfType::FA: o += arr(reinterpret_cast<const float*>(base + f.off), f.n); break;
        case EfType::IA: o += arrI(reinterpret_cast<const int32_t*>(base + f.off), f.n); break;
        }
        o += ",\n";
    }
    o += "    \"turbo_maps\": " + turboJson(p.turbo) + "\n  }";
    if (withCalibration) {
        CalMeta cm; cm.name = "engine file"; cm.engine = p.name;
        std::string cal = calibrationToJson(p.cal, cm);
        while (!cal.empty() && (cal.back() == '\n' || cal.back() == ' ')) cal.pop_back();
        // indent the embedded object by two spaces
        std::string ind;
        for (char ch : cal) { ind += ch; if (ch == '\n') ind += "  "; }
        o += ",\n  \"calibration\": " + ind;
    }
    o += "\n}\n";
    return o;
}

bool engineFromJson(const std::string& text, EngineParams& p, EngineMeta& m, std::string& err) {
    Parser ps{text.data(), text.data() + text.size(), {}};
    JVal root;
    if (!ps.val(root)) { err = "JSON error near byte " + std::to_string(ps.s - text.data()) + ": " + ps.err; return false; }
    if (root.kind != JVal::Obj) { err = "not an engine file"; return false; }
    const JVal* f = root.get("format");
    if (!f || f->str != "enginesim-engine") { err = "not an enginesim engine file (missing \"format\")"; return false; }
    const JVal* e = root.get("engine");
    if (!e || e->kind != JVal::Obj) { err = "the file has no \"engine\" section"; return false; }
    EngineParams n = p;
    char* base = reinterpret_cast<char*>(&n);
    for (const EngineField& fd : kEngineFields) {
        const JVal* j = e->get(fd.key);
        if (!j) continue;
        switch (fd.t) {
        case EfType::F: if (j->kind == JVal::Num && std::isfinite(j->num)) *reinterpret_cast<float*>(base + fd.off) = (float)j->num; break;
        case EfType::I:
            if (j->kind == JVal::Num) *reinterpret_cast<int32_t*>(base + fd.off) = (int32_t)j->num;
            else if (j->kind == JVal::Bool) *reinterpret_cast<int32_t*>(base + fd.off) = j->b ? 1 : 0;
            break;
        case EfType::FA: { int k = 0; readArr(j, reinterpret_cast<float*>(base + fd.off), fd.n, k); break; }
        case EfType::IA: {
            float tmp[kMaxCyl] = {}; int k = 0;
            if (readArr(j, tmp, std::min(fd.n, kMaxCyl), k))
                for (int i = 0; i < k; ++i) reinterpret_cast<int32_t*>(base + fd.off)[i] = (int32_t)tmp[i];
            break;
        }
        }
    }
    readTurbo(e->get("turbo_maps"), n.turbo);
    if (const JVal* j = root.get("name")) std::snprintf(n.name, sizeof(n.name), "%s", j->str.c_str());
    m.name = n.name;
    if (const JVal* j = root.get("author")) m.author = j->str;
    if (const JVal* j = root.get("notes")) m.notes = j->str;
    if (const JVal* j = root.get("saved")) m.saved = j->str;
    if (const JVal* j = root.get("app")) m.app = j->str;
    // keep the numbers in their working ranges (a hand-edited file must not crash the simulation)
    n.cylinders = clampv((int)n.cylinders, 1, isRotary(n) ? 4 : kMaxCyl);
    n.gears = clampv((int)n.gears, 1, kMaxGears);
    n.bore_mm = clampv(n.bore_mm, 20.0f, 200.0f); n.stroke_mm = clampv(n.stroke_mm, 20.0f, 200.0f);
    n.compression = clampv(n.compression, 4.0f, 25.0f); n.revLimit = clampv(n.revLimit, 1000.0f, 25000.0f);
    if (validateFiring(n)) applyLayoutDefaults(n);   // broken firing order: the layout's default one
    n.fuelType = clampv((int)n.fuelType, 0, kFuelTypeCount - 1);
    n.resonator = clampv((int)n.resonator, 0, 2); n.resonatorHz = clampv(n.resonatorHz, 0.0f, 400.0f);
    n.mufflerExtTubes = n.mufflerExtTubes ? 1 : 0;
    // the calibration: from the file, or a base calibration made for this engine
    generateCalibration(n);
    m.hasCalibration = false;
    if (const JVal* c = root.get("calibration"); c && c->kind == JVal::Obj) {
        const JVal* cf = c->get("format");
        if (cf && cf->str == "enginesim-calibration") {
            CalMeta cm;
            calibrationFromRoot(*c, n.cal, cm);
            m.hasCalibration = true;
        }
    }
    p = n;
    return true;
}

void setEngineDir(const std::string& dir) { g_engineDir = dir; }
std::string engineDir() { return g_engineDir; }

std::vector<EngineFileInfo> listEngineFiles() {
    std::vector<EngineFileInfo> v;
    std::error_code ec;
    for (auto& e : fs::directory_iterator(fs::u8path(g_engineDir), ec)) {
        if (!e.is_regular_file()) continue;
        const std::string fn = e.path().filename().u8string();
        if (fn.size() < 12 || fn.substr(fn.size() - 12) != ".engine.json") continue;
        EngineFileInfo i;
        i.path = e.path().u8string();
        i.name = fn.substr(0, fn.size() - 12);
        std::string text;
        if (readTextFile(i.path, text)) {
            Parser ps{text.data(), text.data() + text.size(), {}};
            JVal root;
            if (ps.val(root) && root.kind == JVal::Obj && root.get("format") && root.get("format")->str == "enginesim-engine") {
                i.readable = true;
                if (const JVal* j = root.get("name")) i.name = j->str;
                if (const JVal* j = root.get("author")) i.author = j->str;
                if (const JVal* j = root.get("notes")) i.notes = j->str;
                if (const JVal* j = root.get("saved")) i.saved = j->str;
            }
        }
        v.push_back(i);
    }
    std::sort(v.begin(), v.end(), [](const EngineFileInfo& a, const EngineFileInfo& b) { return a.name < b.name; });
    return v;
}

bool exportEngineFile(const std::string& path, const EngineParams& p, EngineMeta& m, std::string& err) {
    m.saved = nowStamp();
    m.name = p.name;
    if (!writeTextFile(path, engineToJson(p, m, true))) { err = "could not write " + path; return false; }
    return true;
}

bool saveEngineFile(const EngineParams& p, EngineMeta& m, std::string& err, std::string* pathOut) {
    std::string stem = engineFolderName(p.name);
    if (stem.empty()) stem = "engine";
    const std::string path = (fs::u8path(g_engineDir) / fs::u8path(stem + ".engine.json")).u8string();
    if (!exportEngineFile(path, p, m, err)) return false;
    if (pathOut) *pathOut = path;
    return true;
}

bool loadEngineFile(const std::string& path, EngineParams& p, EngineMeta& m, std::string& err) {
    std::string text;
    if (!readTextFile(path, text)) { err = "could not read " + path; return false; }
    return engineFromJson(text, p, m, err);
}

bool deleteEngineFile(const std::string& path) {
    std::error_code ec;
    return fs::remove(fs::u8path(path), ec);
}

} // namespace es
