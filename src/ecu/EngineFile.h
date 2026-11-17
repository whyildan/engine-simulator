#pragma once
// Engine files: a whole engine (hardware, vehicle, sound character and its ECU calibration) as JSON, so it can be
// saved as your own preset and shared. Fields are written by name: a file from an older or newer version loads,
// what it lacks keeps the default. Test-bench state (throttle, dyno, ambient air, sensor faults) is not part of it.
// Your own engines live in engines/<name>.engine.json next to the exe.
#include "sim/EngineParams.h"
#include <string>
#include <vector>

namespace es {

struct EngineMeta {
    std::string name;      // engine name (also the file stem)
    std::string author;
    std::string notes;
    std::string saved;     // local time stamp
    std::string app;       // app version that wrote it
    bool hasCalibration = false;
};

std::string engineToJson(const EngineParams& p, const EngineMeta& m, bool withCalibration = true);
// p: the engine to load into (its fields are the defaults for anything the file lacks); without a calibration in the
// file a base calibration is generated for the engine
bool engineFromJson(const std::string& text, EngineParams& p, EngineMeta& m, std::string& err);

void setEngineDir(const std::string& dir);   // root folder ("engines")
std::string engineDir();
struct EngineFileInfo { std::string name, path, author, notes, saved; bool readable = false; };
std::vector<EngineFileInfo> listEngineFiles();
// writes engines/<name>.engine.json (replacing a file of the same name); path of the file in *pathOut
bool saveEngineFile(const EngineParams& p, EngineMeta& m, std::string& err, std::string* pathOut = nullptr);
bool loadEngineFile(const std::string& path, EngineParams& p, EngineMeta& m, std::string& err);
bool exportEngineFile(const std::string& path, const EngineParams& p, EngineMeta& m, std::string& err);
bool deleteEngineFile(const std::string& path);

} // namespace es
