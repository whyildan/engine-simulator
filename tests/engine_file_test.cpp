// Engine files (save / export / import of whole engines): round trip of every preset, partial files, bad files,
// out-of-range values, and that an imported engine runs.
// usage: engine_file_test
#include "ecu/EngineFile.h"
#include "ecu/BaseMap.h"
#include "sim/Simulator.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

int main() {
    const std::string tmp = std::getenv("TEMP") ? std::getenv("TEMP") : ".";
    {
        std::printf("[round trip] every preset: export, import into another engine, export again\n");
        int same = 0;
        for (int i = 0; i < presetCount(); ++i) {
            EngineParams p = makePreset(i);
            p.cal.cylFuel[1] = 2.5f; p.cal.veMap.v[3][4] = 77.25f;   // a "tuned" calibration
            EngineMeta m; m.author = "tester"; m.notes = "line one\nline \"two\"";
            const std::string a = engineToJson(p, m, true);
            EngineParams q = makePreset((i + 7) % presetCount());   // load into a different engine
            EngineMeta mq; std::string err;
            const bool ok = engineFromJson(a, q, mq, err);
            const std::string b = engineToJson(q, mq, true);
            if (ok && a == b && mq.author == "tester" && mq.notes == m.notes && mq.hasCalibration &&
                q.cal.cylFuel[1] == 2.5f && q.cal.veMap.v[3][4] == 77.25f && std::strcmp(q.name, p.name) == 0 &&
                baseMapKey(q) == baseMapKey(p)) ++same;   // (the key hashes every engine field: nothing left out)
            else std::printf("    differs: %s (%s)\n", p.name, ok ? "content" : err.c_str());
        }
        CHECK(same == presetCount(), "%d of %d presets come back identical (hardware, turbo maps, calibration)", same, presetCount());
    }
    {
        std::printf("[turbo maps] a table compressor / turbine map survives\n");
        EngineParams p = makePreset(16);
        p.turbo.useMaps = 1; p.turbo.comp = builtinCompressor(2); p.turbo.turb = builtinTurbine(1);
        std::snprintf(p.turbo.compName, sizeof(p.turbo.compName), "Medium 58");
        EngineParams q = makePreset(0); EngineMeta m; std::string err;
        engineFromJson(engineToJson(p, m), q, m, err);
        CHECK(q.turbo.useMaps == 1 && std::memcmp(&q.turbo.comp, &p.turbo.comp, sizeof(p.turbo.comp)) == 0 &&
              std::memcmp(&q.turbo.turb, &p.turbo.turb, sizeof(p.turbo.turb)) == 0 && std::strcmp(q.turbo.compName, "Medium 58") == 0,
              "maps and their names identical");
    }
    {
        std::printf("[partial / bad files]\n");
        EngineParams base = makePreset(3);
        EngineParams q = base; EngineMeta m; std::string err;
        const char* partial = "{ \"format\": \"enginesim-engine\", \"name\": \"Shared V6\", \"engine\": { \"bore_mm\": 95.5, \"cylinders\": 6 } }";
        const bool ok = engineFromJson(partial, q, m, err);
        CHECK(ok && q.bore_mm == 95.5f && q.stroke_mm == base.stroke_mm && !m.hasCalibration && std::strcmp(q.name, "Shared V6") == 0,
              "missing fields keep their values, the calibration is generated");
        q = base;
        CHECK(!engineFromJson("{ \"format\": \"enginesim-calibration\" }", q, m, err), "a calibration file is not an engine (%s)", err.c_str());
        CHECK(!engineFromJson("{ \"format\": \"enginesim-engine\", \"engine\": { \"bore_mm\": 9", q, m, err), "broken JSON is rejected (%s)", err.c_str());
        q = base;
        engineFromJson("{ \"format\": \"enginesim-engine\", \"engine\": { \"cylinders\": 99, \"compression\": 80, \"rev_limit\": -5, "
                       "\"firing_order\": [0, 0, 0, 0, 0, 0] } }", q, m, err);
        CHECK(q.cylinders == kMaxCyl && q.compression == 25.0f && q.revLimit == 1000.0f && validateFiring(q) == nullptr,
              "out-of-range values are clamped, a broken firing order replaced (cyl %d, CR %.0f, firing %s)", q.cylinders, q.compression,
              validateFiring(q) ? validateFiring(q) : "ok");
    }
    {
        std::printf("[files] save into the engines folder, list, load, export, delete\n");
        setEngineDir(tmp + "\\es_engines_test");
        EngineParams p = makePreset(2);
        std::snprintf(p.name, sizeof(p.name), "My I6 / test: v2");
        p.bore_mm = 84.25f;
        EngineMeta m; m.author = "me"; std::string err, path;
        CHECK(saveEngineFile(p, m, err, &path), "saved (%s)", path.c_str());
        auto list = listEngineFiles();
        bool found = false;
        for (auto& f : list) found = found || (f.name == "My I6 / test: v2" && f.readable && f.author == "me");
        CHECK(found, "listed with its name and author (%zu files)", list.size());
        EngineParams q = makePreset(0); EngineMeta mq;
        CHECK(loadEngineFile(path, q, mq, err) && q.bore_mm == 84.25f, "loaded back");
        const std::string ex = tmp + "\\es_export.engine.json";
        CHECK(exportEngineFile(ex, p, m, err) && loadEngineFile(ex, q, mq, err), "exported and imported from another folder");
        CHECK(deleteEngineFile(path), "deleted");
    }
    {
        std::printf("[run] an imported engine starts and idles\n");
        EngineParams p = makePreset(5);
        EngineParams q = makePreset(0); EngineMeta m; std::string err;
        engineFromJson(engineToJson(p, m), q, m, err);
        Calibration c = q.cal;
        if (!loadCachedBaseMap(q, c)) { measureBaseMap(q, c); saveCachedBaseMap(q, c); }
        q.cal.veMap = c.veMap; q.cal.veTps = c.veTps;
        q.ignition = 1; q.starter = 1;
        Simulator sim(q);
        std::vector<float> buf(960);
        for (int k = 0; k < 150; ++k) sim.render(buf.data(), 480);
        q.starter = 0; sim.setParams(q);
        for (int k = 0; k < 300; ++k) sim.render(buf.data(), 480);
        Telemetry t; sim.getTelemetry(t);
        CHECK(t.running && t.rpm > 600, "%s idles at %.0f rpm", q.name, t.rpm);
    }
    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}
