#include "sim/DataLog.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace es {

namespace {
float knockRetardMax(const Telemetry& t) {
    float m = 0;
    for (int i = 0; i < kMaxCyl; ++i) m = std::max(m, t.knockRetard[i]);
    return m;
}
float protActive(const Telemetry& t) {
    int n = 0;
    for (int i = 0; i < kProtCount; ++i) n += t.protState[i] == 2 ? 1 : 0;
    return (float)n;
}
const LogChannel kChannels[] = {
    {"rpm",          "Engine speed",        "rpm",   "Engine",  [](const Telemetry& t) { return t.rpm; }},
    {"torque",       "Torque",              "Nm",    "Engine",  [](const Telemetry& t) { return t.torque; }},
    {"power",        "Power",               "kW",    "Engine",  [](const Telemetry& t) { return t.powerKw; }},
    {"throttle",     "Throttle position",   "%",     "Engine",  [](const Telemetry& t) { return t.throttle * 100.0f; }},
    {"pedal",        "Pedal position",      "%",     "Engine",  [](const Telemetry& t) { return t.snsValue[kSnsAPP]; }},
    {"map",          "Manifold pressure",   "kPa",   "Engine",  [](const Telemetry& t) { return t.mapKPa; }},
    {"airflow",      "Air flow",            "g/s",   "Engine",  [](const Telemetry& t) { return t.airFlow; }},
    {"ve",           "Volumetric efficiency", "%",   "Engine",  [](const Telemetry& t) { return t.ve * 100.0f; }},
    {"misfires",     "Misfires (count)",    "",      "Engine",  [](const Telemetry& t) { return (float)t.misfires; }},
    {"afr",          "AFR",                 "",      "Fuel",    [](const Telemetry& t) { return t.afr; }},
    {"afr_target",   "Target AFR",          "",      "Fuel",    [](const Telemetry& t) { return t.targetAfr; }},
    {"lambda",       "Wideband lambda",     "",      "Fuel",    [](const Telemetry& t) { return t.snsValue[kSnsWideband]; }},
    {"inj_duty",     "Injector duty",       "%",     "Fuel",    [](const Telemetry& t) { return t.injDuty * 100.0f; }},
    {"fuelflow",     "Fuel flow",           "g/s",   "Fuel",    [](const Telemetry& t) { return t.fuelFlow; }},
    {"ecu_ve",       "ECU VE (map)",        "%",     "Fuel",    [](const Telemetry& t) { return t.ecuVe; }},
    {"stft",         "Short-term trim",     "%",     "Fuel",    [](const Telemetry& t) { return t.ecuStft; }},
    {"ltft",         "Long-term trim",      "%",     "Fuel",    [](const Telemetry& t) { return t.ecuLtft; }},
    {"accel_enr",    "Accel enrichment",    "%",     "Fuel",    [](const Telemetry& t) { return t.ecuAccel * 100.0f; }},
    {"fuel_press",   "Fuel pressure",       "bar",   "Fuel",    [](const Telemetry& t) { return t.snsValue[kSnsFuelP]; }},
    {"advance",      "Ignition advance",    "deg",   "Ignition", [](const Telemetry& t) { return t.advance; }},
    {"knock",        "Knock index",         "",      "Ignition", [](const Telemetry& t) { return t.knock; }},
    {"knock_retard", "Knock retard (max)",  "deg",   "Ignition", knockRetardMax},
    {"soft_cut",     "Soft limiter cut",    "%",     "Ignition", [](const Telemetry& t) { return t.cutFraction * 100.0f; }},
    {"limiter",      "Rev limiter",         "on",    "Ignition", [](const Telemetry& t) { return t.limiter ? 1.0f : 0.0f; }},
    {"iac",          "Idle valve",          "%",     "Control", [](const Telemetry& t) { return t.iac; }},
    {"idle_target",  "Idle target",         "rpm",   "Control", [](const Telemetry& t) { return t.idleTarget; }},
    {"boost",        "Boost",               "bar",   "Boost",   [](const Telemetry& t) { return t.mapKPa / 101.325f - 1.0f; }},
    {"boost_target", "Boost target",        "bar",   "Boost",   [](const Telemetry& t) { return t.boostTargetBar; }},
    {"wg_duty",      "Wastegate duty",      "%",     "Boost",   [](const Telemetry& t) { return std::max(0.0f, t.wgDuty) * 100.0f; }},
    {"wastegate",    "Wastegate position",  "%",     "Boost",   [](const Telemetry& t) { return t.wastegate * 100.0f; }},
    {"turbo_rpm",    "Turbo / blower speed", "krpm", "Boost",   [](const Telemetry& t) { return t.shaftRpm / 1000.0f; }},
    {"ect",          "Coolant temperature", "C",     "Sensors", [](const Telemetry& t) { return t.snsValue[kSnsECT]; }},
    {"iat",          "Intake air temp",     "C",     "Sensors", [](const Telemetry& t) { return t.snsValue[kSnsIAT]; }},
    {"oil_temp",     "Oil temperature",     "C",     "Sensors", [](const Telemetry& t) { return t.snsValue[kSnsOilT]; }},
    {"oil_press",    "Oil pressure",        "bar",   "Sensors", [](const Telemetry& t) { return t.snsValue[kSnsOilP]; }},
    {"egt",          "Exhaust gas temp",    "C",     "Sensors", [](const Telemetry& t) { return t.snsValue[kSnsEGT]; }},
    {"battery",      "Battery voltage",     "V",     "Sensors", [](const Telemetry& t) { return t.snsValue[kSnsBattery]; }},
    {"speed",        "Vehicle speed",       "km/h",  "Vehicle", [](const Telemetry& t) { return t.speedKmh; }},
    {"gear",         "Gear",                "",      "Vehicle", [](const Telemetry& t) { return (float)t.gear; }},
    {"wheel_slip",   "Wheel slip",          "%",     "Vehicle", [](const Telemetry& t) { return t.wheelSlip * 100.0f; }},
    {"clutch_tq",    "Clutch torque",       "Nm",    "Vehicle", [](const Telemetry& t) { return t.clutchTq; }},
    {"limp",         "Limp mode",           "on",    "Protection", [](const Telemetry& t) { return t.limp ? 1.0f : 0.0f; }},
    {"prot_active",  "Protections active",  "",      "Protection", protActive},
    {"launch",       "Launch / shift limiter", "state", "Motorsport", [](const Telemetry& t) { return (float)t.launchState; }},
    {"tc_slip",      "Wheel slip (TC)",     "%",     "Motorsport", [](const Telemetry& t) { return t.tcSlip; }},
    {"tc_reduction", "TC torque reduction", "%",     "Motorsport", [](const Telemetry& t) { return t.tcReduction * 100.0f; }},
    {"n2o_duty",     "Nitrous duty",        "%",     "Motorsport", [](const Telemetry& t) { return t.n2oDuty * 100.0f; }},
    {"n2o_bottle",   "Nitrous bottle",      "kg",    "Motorsport", [](const Telemetry& t) { return t.n2oBottleKg; }},
};
constexpr int kCount = (int)(sizeof(kChannels) / sizeof(kChannels[0]));
} // namespace

int logChannelCount() { return kCount; }
const LogChannel& logChannel(int i) { return kChannels[std::clamp(i, 0, kCount - 1)]; }
int logChannelIndex(const char* key) {
    for (int i = 0; i < kCount; ++i) if (std::strcmp(kChannels[i].key, key) == 0) return i;
    return -1;
}

void sampleChannels(const Telemetry& t, float* out) {
    for (int i = 0; i < kCount; ++i) out[i] = kChannels[i].get(t);
}

void LogSession::append(double time, const float* values) {
    if ((int)ch.size() != kCount) ch.resize(kCount);
    t.push_back(time);
    for (int i = 0; i < kCount; ++i) ch[i].push_back(values[i]);
}

// ------------------------------------------------------------------ CSV
static std::string clean(const std::string& s) {   // one line, no separators that would break the header
    std::string o;
    for (char c : s) o += (c == '\n' || c == '\r') ? ' ' : c;
    return o;
}

std::string logCsv(const LogSession& s, const std::vector<int>& chans) {
    std::vector<int> cols = chans;
    if (cols.empty()) for (int i = 0; i < kCount; ++i) cols.push_back(i);
    std::ostringstream o;
    o << "# enginesim log\n";
    o << "# name: " << clean(s.name) << "\n";
    o << "# engine: " << clean(s.engine) << "\n";
    o << "# calibration: " << clean(s.calName) << "\n";
    o << "# calibration_revision: " << s.calRev << "\n";
    o << "# started: " << clean(s.started) << "\n";
    o << "# rate_hz: " << s.rateHz << "\n";
    if (!s.notes.empty()) o << "# notes: " << clean(s.notes) << "\n";
    for (const LogMarker& m : s.markers) o << "# marker: " << m.t << " " << clean(m.label) << "\n";
    o << "time_s";
    for (int c : cols) o << "," << kChannels[c].key << (*kChannels[c].unit ? std::string(" [") + kChannels[c].unit + "]" : std::string());
    o << "\n";
    char b[32];
    for (size_t k = 0; k < s.t.size(); ++k) {
        std::snprintf(b, sizeof(b), "%.4f", s.t[k]);
        o << b;
        for (int c : cols) {
            const float v = c < (int)s.ch.size() && k < s.ch[c].size() ? s.ch[c][k] : 0.0f;
            std::snprintf(b, sizeof(b), ",%.6g", std::isfinite(v) ? v : 0.0f);
            o << b;
        }
        o << "\n";
    }
    return o.str();
}

bool writeLogCsv(const std::string& path, const LogSession& s, const std::vector<int>& chans, std::string& err) {
    std::ofstream f(std::filesystem::u8path(path), std::ios::binary);   // UTF-8 path (file dialog)
    if (!f) { err = "cannot write " + path; return false; }
    f << logCsv(s, chans);
    if (!f) { err = "write failed: " + path; return false; }
    return true;
}

bool parseLogCsv(const std::string& text, LogSession& s, std::string& err) {
    s = LogSession();
    s.ch.assign(kCount, {});
    std::istringstream in(text);
    std::string line;
    std::vector<int> map;   // CSV column -> channel (-1 unknown / not stored)
    bool header = false;
    size_t lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line[0] == '#') {
            auto val = [&](const char* key) -> const char* {
                const size_t n = std::strlen(key);
                return line.compare(2, n, key) == 0 && line.size() > 2 + n + 1 ? line.c_str() + 2 + n + 1 : nullptr;
            };
            if (const char* v = val("name:")) s.name = v;
            else if (const char* v = val("engine:")) s.engine = v;
            else if (const char* v = val("calibration:")) s.calName = v;
            else if (const char* v = val("calibration_revision:")) s.calRev = std::atoi(v);
            else if (const char* v = val("started:")) s.started = v;
            else if (const char* v = val("rate_hz:")) s.rateHz = (float)std::atof(v);
            else if (const char* v = val("notes:")) s.notes = v;
            else if (const char* v = val("marker:")) {
                char* end = nullptr;
                const double t = std::strtod(v, &end);
                s.markers.push_back({t, end && *end == ' ' ? std::string(end + 1) : std::string()});
            }
            continue;
        }
        std::vector<std::string> cells;
        std::stringstream ls(line);
        std::string cell;
        while (std::getline(ls, cell, ',')) cells.push_back(cell);
        if (!header) {
            if (cells.empty() || cells[0].rfind("time", 0) != 0) { err = "not an enginesim log (no time column)"; return false; }
            for (size_t c = 1; c < cells.size(); ++c) {
                std::string key = cells[c].substr(0, cells[c].find(' '));
                map.push_back(logChannelIndex(key.c_str()));
            }
            header = true;
            continue;
        }
        if (cells.size() != map.size() + 1) { err = "bad row at line " + std::to_string(lineNo); return false; }
        s.t.push_back(std::atof(cells[0].c_str()));
        for (int i = 0; i < kCount; ++i) s.ch[i].push_back(0.0f);
        for (size_t c = 0; c < map.size(); ++c)
            if (map[c] >= 0) s.ch[map[c]].back() = (float)std::atof(cells[c + 1].c_str());
    }
    if (!header) { err = "empty log"; return false; }
    return true;
}

bool readLogCsv(const std::string& path, LogSession& s, std::string& err) {
    std::ifstream f(std::filesystem::u8path(path), std::ios::binary);
    if (!f) { err = "cannot read " + path; return false; }
    std::stringstream ss; ss << f.rdbuf();
    return parseLogCsv(ss.str(), s, err);
}

} // namespace es
