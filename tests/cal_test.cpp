// Calibration files: JSON round trip for every preset, tolerant import, validation and diff.
#include "ecu/CalibrationIO.h"
#include "sim/EngineParams.h"
#include <cstdio>
#include <cstring>
#include <filesystem>

using namespace es;

int main(int argc, char** argv) {
    int fails = 0;
    auto check = [&](bool ok, const char* what) { if (!ok) { std::printf("FAIL: %s\n", what); ++fails; } };
    for (int i = 0; i < presetCount(); ++i) {
        EngineParams p = makePreset(i);
        p.cal.veMap.v[3][4] = 87.123456f; p.cal.cylFuel[1] = -2.5f; p.cal.ignition.locked = 1;
        CalMeta m; m.name = "test"; m.engine = p.name; m.notes = "line 1\nline \"2\" \\ tab\t"; m.revision = 7;
        std::string js = calibrationToJson(p.cal, m);
        Calibration c{}; CalMeta m2; std::string err;
        // start from a zeroed calibration: everything must come from the file
        bool ok = calibrationFromJson(js, c, m2, err);
        check(ok, err.c_str());
        // compare field by field (padding-free POD)
        bool same = std::memcmp(&c, &p.cal, sizeof(Calibration)) == 0;
        if (!same) {
            for (int t = 0; t < kCalTableCount; ++t) { CalDiff d = diffTable(calTable(p.cal, t), calTable(c, t)); if (d.changed || d.axesDiffer) std::printf("  %s differs\n", calTableInfo(t).name); }
            std::vector<std::string> n; diffScalars(p.cal, c, &n); for (auto& s : n) std::printf("  %s\n", s.c_str());
        }
        char w[96]; std::snprintf(w, sizeof(w), "round trip preset %d (%s)", i, p.name);
        check(same, w);
        check(m2.notes == m.notes && m2.revision == 7 && m2.engine == p.name, "meta round trip");
        auto issues = validateCalibration(p.cal, p);
        int errs = 0; for (auto& is : issues) { if (is.severity == 2) { ++errs; std::printf("  preset %d: %s\n", i, is.text.c_str()); } }
        std::snprintf(w, sizeof(w), "generated calibration of preset %d validates without errors", i);
        check(errs == 0, w);
    }
    // tolerant import: missing fields keep the current values, garbage is rejected
    {
        EngineParams p = makePreset(0);
        Calibration c = p.cal; CalMeta m; std::string err;
        check(calibrationFromJson("{\"format\":\"enginesim-calibration\",\"power_lambda\":0.8}", c, m, err), "partial file");
        check(c.powerLambda == 0.8f && std::memcmp(&c.veMap, &p.cal.veMap, sizeof(Table2D)) == 0, "partial file keeps the rest");
        check(!calibrationFromJson("{\"format\":\"enginesim-calibration\",", c, m, err), "truncated file rejected");
        check(!calibrationFromJson("{\"a\":1}", c, m, err), "foreign json rejected");
        Calibration bad = p.cal; bad.lambda.v[2][2] = 3.0f; bad.ignition.x[3] = bad.ignition.x[2];
        int errs = 0; for (auto& is : validateCalibration(bad, p)) errs += is.severity == 2;
        check(errs >= 2, "validation finds out-of-range cell and non-increasing axis");
        Calibration d = p.cal; d.veMap.v[1][1] += 5; d.veMap.v[2][2] -= 3;
        CalDiff df = diffTable(p.cal.veMap, d.veMap);
        check(df.changed == 2 && df.maxAbs > 4.99f && df.maxAbs < 5.01f, "diff counts changed cells");
    }
    // files: save twice -> two revisions in history, load back
    {
        namespace fs = std::filesystem;
        std::string dir = (fs::temp_directory_path() / "enginesim_cal_test").u8string();
        fs::remove_all(dir);
        setCalibrationDir(dir);
        EngineParams p = makePreset(1);
        CalMeta m; m.name = "street";
        std::string err;
        check(saveCalibrationFile(p.name, m, p.cal, err), "save 1");
        p.cal.veMap.v[0][0] += 1;
        check(saveCalibrationFile(p.name, m, p.cal, err), "save 2");
        check(m.revision == 2, "revision bumps");
        check(listHistory(p.name, "street").size() == 2, "history has 2 entries");
        check(listCalibrations(p.name).size() == 1, "one calibration listed");
        Calibration c{}; CalMeta m2;
        check(loadCalibrationFile(listCalibrations(p.name)[0].path, c, m2, err) && std::memcmp(&c, &p.cal, sizeof(c)) == 0, "load latest");
        CalMeta bad; bad.name = "a/b";
        check(!saveCalibrationFile(p.name, bad, p.cal, err), "invalid name rejected");
        check(deleteCalibrationFile(p.name, "street") && listCalibrations(p.name).empty(), "delete");
        fs::remove_all(dir);
    }
    std::printf(fails ? "cal_test: %d FAILED\n" : "cal_test: all passed\n", fails);
    return fails ? 1 : 0;
}
