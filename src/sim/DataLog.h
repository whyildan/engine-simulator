#pragma once
// Data logging: the channel table (what can be logged, from the telemetry the simulator publishes), recorded
// sessions with markers and the calibration they ran on, CSV export / import.
#include "sim/Simulator.h"
#include <string>
#include <vector>

namespace es {

struct LogChannel {
    const char* key;    // CSV column / file name
    const char* name;   // UI
    const char* unit;
    const char* group;  // UI grouping
    float (*get)(const Telemetry&);
};
int logChannelCount();
const LogChannel& logChannel(int i);
int logChannelIndex(const char* key);      // -1 if unknown

struct LogMarker { double t; std::string label; };

struct LogSession {
    std::string name, engine, calName, started, notes;
    int calRev = 0;
    float rateHz = 50;
    std::vector<double> t;                      // s since the start of the recording
    std::vector<std::vector<float>> ch;         // ch[channel][sample]
    std::vector<LogMarker> markers;
    void clear() { t.clear(); ch.assign(logChannelCount(), {}); markers.clear(); }
    double duration() const { return t.empty() ? 0.0 : t.back() - t.front(); }
    size_t size() const { return t.size(); }
    // append one sample of every channel (as produced by sampleChannels)
    void append(double time, const float* values);
};

// all channel values of one telemetry snapshot, in channel order
void sampleChannels(const Telemetry& t, float* out);

// CSV: comment header (# key: value) with the session data, a column header, one row per sample; markers are
// written as "# marker: <t> <label>". `channels` empty = all.
bool writeLogCsv(const std::string& path, const LogSession& s, const std::vector<int>& channels, std::string& err);
std::string logCsv(const LogSession& s, const std::vector<int>& channels);
bool readLogCsv(const std::string& path, LogSession& s, std::string& err);
bool parseLogCsv(const std::string& text, LogSession& s, std::string& err);

} // namespace es
