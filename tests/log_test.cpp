// Phase 4: data logging - sampling rate in sim time, channel table, CSV round trip with metadata and markers.
// usage: log_test
#include "sim/DataLog.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

int main() {
    std::printf("[channels] %d log channels\n", logChannelCount());
    {
        std::set<std::string> keys;
        bool unique = true;
        for (int i = 0; i < logChannelCount(); ++i) unique = unique && keys.insert(logChannel(i).key).second;
        CHECK(unique, "channel keys are unique");
        CHECK(logChannelIndex("rpm") == 0 && logChannelIndex("nope") == -1, "lookup by key");
    }
    std::printf("[sampling] idle with logging at 50 Hz for 3 s of sim time\n");
    LogSession s;
    s.clear();
    {
        EngineParams p = makeCalibratedPreset(0);
        p.ignition = 1; p.starter = 1;
        Simulator sim(p);
        std::vector<float> buf(256 * 2);
        for (int k = 0; k < 300; ++k) sim.render(buf.data(), 256);
        p.starter = 0; sim.setParams(p);
        sim.setLogging(true, 50.0);
        const double frames = 3.0 * Simulator::kOutRate;
        for (double f = 0; f < frames; f += 256) sim.render(buf.data(), 256);
        sim.setLogging(false, 50.0);
        std::vector<double> times; std::vector<float> vals;
        sim.takeLogSamples(times, vals);
        const int n = logChannelCount();
        for (size_t k = 0; k < times.size(); ++k) s.append(times[k] - times.front(), &vals[k * n]);
        std::printf("  %zu samples, span %.2f s\n", s.size(), s.duration());
        CHECK(s.size() >= 145 && s.size() <= 155, "about 150 samples (50 Hz x 3 s)");
        double maxGap = 0;
        for (size_t k = 1; k < s.t.size(); ++k) maxGap = std::max(maxGap, s.t[k] - s.t[k - 1]);
        CHECK(maxGap < 0.03, "evenly spaced (largest gap %.4f s)", maxGap);
        const int rpm = logChannelIndex("rpm");
        double mean = 0; for (float v : s.ch[rpm]) mean += v; mean /= s.size();
        CHECK(mean > 500 && mean < 1500, "rpm channel carries the idle speed (%.0f rpm)", mean);
    }
    std::printf("[csv] round trip with metadata and markers\n");
    {
        s.name = "idle test"; s.engine = "I4 2.0"; s.calName = "street, v2"; s.calRev = 7; s.rateHz = 50; s.started = "2026-11-16 10:00:00";
        s.markers.push_back({1.25, "blip"});
        s.markers.push_back({2.5, "protection: coolant"});
        const std::string text = logCsv(s, {});
        LogSession r; std::string err;
        const bool ok = parseLogCsv(text, r, err);
        CHECK(ok, "parses (%s)", err.c_str());
        CHECK(r.name == s.name && r.engine == s.engine && r.calName == s.calName && r.calRev == 7 && r.rateHz == 50, "metadata kept");
        CHECK(r.markers.size() == 2 && std::fabs(r.markers[1].t - 2.5) < 1e-9 && r.markers[1].label == "protection: coolant", "markers kept");
        bool same = r.size() == s.size();
        for (int c = 0; same && c < logChannelCount(); ++c)
            for (size_t k = 0; k < s.size(); ++k)
                if (std::fabs(r.ch[c][k] - s.ch[c][k]) > 1e-4f * (1.0f + std::fabs(s.ch[c][k]))) { same = false; break; }
        CHECK(same, "all samples kept (%zu rows)", r.size());
        // a subset of channels: the others come back as zero, the chosen ones intact
        const int rpm = logChannelIndex("rpm"), afr = logChannelIndex("afr");
        LogSession q;
        CHECK(parseLogCsv(logCsv(s, {rpm, afr}), q, err) && std::fabs(q.ch[afr][3] - s.ch[afr][3]) < 1e-3f && q.ch[logChannelIndex("map")][3] == 0.0f,
              "channel subset export");
        CHECK(!parseLogCsv("hello,world\n1,2\n", q, err), "foreign CSV rejected");
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}
