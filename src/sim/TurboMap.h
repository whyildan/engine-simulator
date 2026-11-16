#pragma once
// Compressor and turbine maps as tables (Phase 6).
//
// Compressor map: speed lines of corrected wheel speed, each a list of points from the surge line to choke with
// corrected mass flow, total pressure ratio and isentropic efficiency - the same data a manufacturer map shows.
// Corrected to 298 K / 101.325 kPa inlet:  flow_corr = flow * sqrt(T / 298) / (p / 101.325 kPa),
// speed_corr = N * sqrt(298 / T).
//
// Turbine map: reduced mass flow  m * sqrt(T_in) / p_in  [kg/s * sqrt(K) / kPa] against the expansion ratio
// p_in / p_out, and efficiency against the blade speed ratio U / C (tip speed over the isentropic spouting
// velocity), plus the wheel diameter that turns shaft speed into U.
//
// The tables live inside EngineParams (plain data), the evaluator classes resample them once.
#include <cstdint>
#include <string>

namespace es {

constexpr int kCompLines = 10, kCompPts = 12, kTurbPts = 16;
constexpr double kMapTref = 298.0, kMapPref = 101325.0;

struct CompressorMap {
    int32_t nLines;                        // speed lines (ascending speed)
    float speed[kCompLines];               // corrected wheel speed (rpm)
    int32_t nPts[kCompLines];              // points on each line, surge first, choke last
    float flow[kCompLines][kCompPts];      // corrected mass flow (kg/s)
    float pr[kCompLines][kCompPts];        // total pressure ratio
    float eff[kCompLines][kCompPts];       // isentropic efficiency 0..1
};

struct TurbineMap {
    int32_t nFlow;                         // reduced flow vs expansion ratio (ascending ratio)
    float er[kTurbPts];
    float mfp[kTurbPts];                   // kg/s * sqrt(K) / kPa
    int32_t nEff;                          // efficiency vs blade speed ratio (ascending U/C)
    float uc[kTurbPts];
    float eff[kTurbPts];
    float wheelDia_mm;                     // turbine wheel (inducer) diameter
};

struct TurboMaps {
    int32_t useMaps;                       // 0 = generic model from the size sliders, 1 = these tables
    int32_t compChoice, turbChoice;        // built-in index; -1 = loaded from a file / converted
    char compName[40], turbName[40];
    CompressorMap comp;
    TurbineMap turb;
};

// ---------------------------------------------------------------- evaluation
struct CompPoint {
    double flow = 0;      // corrected kg/s (left of the surge line: the flow at the surge point)
    double eff = 0.4;
    double surgePr = 1;   // highest pressure ratio this speed can hold
    double surgeFlow = 0; // flow at that point
    double chokeFlow = 0; // corrected kg/s at the choke end of the line
    bool surge = false, choke = false;
};

class CompressorModel {
public:
    static constexpr int kB = 25;          // resampled points per line (surge -> choke)
    bool build(const CompressorMap& m);    // false if the map is unusable
    bool valid() const { return n_ >= 2; }
    CompPoint eval(double nCorr, double pr) const;
    double maxSpeed() const { return n_ ? sp_[n_ - 1] : 0; }
    double maxFlow() const;                // choke flow of the top line (kg/s, corrected)
    double peakEff() const;
    // a line at speed nCorr, resampled (flow, pr, eff) - for plots
    void line(double nCorr, double* flow, double* pr, double* eff) const;
private:
    void lineAt(double nCorr, double* f, double* p, double* e) const;
    int n_ = 0;
    double sp_[kCompLines] = {};
    double f_[kCompLines][kB] = {}, p_[kCompLines][kB] = {}, e_[kCompLines][kB] = {};
};

class TurbineModel {
public:
    bool build(const TurbineMap& t);
    bool valid() const { return nf_ >= 2; }
    double mfp(double er) const;           // reduced flow at this expansion ratio
    double eff(double uc) const;           // efficiency at this blade speed ratio
    double wheelDia() const { return dia_; }
    double maxMfp() const { return nf_ ? mf_[nf_ - 1] : 0; }
    double peakEff() const;
private:
    int nf_ = 0, ne_ = 0;
    double er_[kTurbPts] = {}, mf_[kTurbPts] = {}, uc_[kTurbPts] = {}, ef_[kTurbPts] = {};
    double dia_ = 50;
};

// ---------------------------------------------------------------- generators
// the analytic model of the size sliders as a table (choke flow at max speed in g/s, max wheel speed rpm)
CompressorMap genericCompressorMap(double chokeFlow_gs, double maxRpm);
// a nozzle-like turbine of this effective area (m^2) with the generic flat efficiency
TurbineMap genericTurbineMap(double area_m2);
int builtinCompressorCount();
const char* builtinCompressorName(int i);
CompressorMap builtinCompressor(int i);
int builtinTurbineCount();
const char* builtinTurbineName(int i);
TurbineMap builtinTurbine(int i);

// ---------------------------------------------------------------- files (CSV)
// compressor: columns speed (rpm), flow (kg/s; or flow_lbmin, flow_gs), pr, eff (0..1 or %), rows grouped by
// speed line. turbine: section "er,mfp" then section "uc,eff"; "# wheel_dia_mm: 52". Lines starting with # are
// comments, "# name: ..." names the map.
bool loadCompressorCsv(const std::string& path, CompressorMap& out, std::string& name, std::string& err);
bool saveCompressorCsv(const std::string& path, const CompressorMap& m, const std::string& name);
bool loadTurbineCsv(const std::string& path, TurbineMap& out, std::string& name, std::string& err);
bool saveTurbineCsv(const std::string& path, const TurbineMap& t, const std::string& name);

} // namespace es
