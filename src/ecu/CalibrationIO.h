#pragma once
// Calibration files: JSON (de)serialisation, validation and comparison. The files live in
// calibrations/<engine>/<name>.json; every save also keeps a numbered copy in <name>.history/.
#include "ecu/Calibration.h"
#include <string>
#include <vector>

namespace es {

struct EngineParams;

struct CalMeta {
    std::string name;      // calibration name (file stem)
    std::string engine;    // engine (preset) it was made for
    std::string notes;
    std::string saved;     // local time stamp of the save
    int revision = 0;
};

std::string calibrationToJson(const Calibration& c, const CalMeta& m);
// fields missing in the file keep the value already in `c`; false + err on a syntax error
bool calibrationFromJson(const std::string& text, Calibration& c, CalMeta& m, std::string& err);

struct CalIssue { int severity; std::string text; };   // 0 info, 1 warning, 2 error
std::vector<CalIssue> validateCalibration(const Calibration& c, const EngineParams& p);

// difference summary of one table / curve (b - a)
struct CalDiff { int changed = 0, cells = 0; float maxAbs = 0, meanAbs = 0; bool axesDiffer = false; };
CalDiff diffTable(const Table2D& a, const Table2D& b);
CalDiff diffCurve(const Curve1D& a, const Curve1D& b);
// number of scalar settings that differ
int diffScalars(const Calibration& a, const Calibration& b, std::vector<std::string>* names = nullptr);

// ---- files
void setCalibrationDir(const std::string& dir);      // root folder ("calibrations")
std::string calibrationDir();
std::string engineFolderName(const char* engineName); // sanitised
struct CalFileInfo { std::string name, path, saved, notes; int revision = 0; bool readable = false; };
std::vector<CalFileInfo> listCalibrations(const std::string& engine);
std::vector<CalFileInfo> listHistory(const std::string& engine, const std::string& name);
bool saveCalibrationFile(const std::string& engine, CalMeta& m, const Calibration& c, std::string& err);  // bumps m.revision
bool loadCalibrationFile(const std::string& path, Calibration& c, CalMeta& m, std::string& err);
bool deleteCalibrationFile(const std::string& engine, const std::string& name);   // file + history
bool writeTextFile(const std::string& path, const std::string& text);
bool readTextFile(const std::string& path, std::string& text);

} // namespace es
