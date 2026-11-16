#pragma once
#include "sim/Simulator.h"
#include "audio/AudioOutput.h"
#include "ecu/BaseMap.h"
#include "ecu/CalibrationIO.h"
#include "sim/DataLog.h"
#include "ui/TableEditor.h"
#include "ui/EngineView3D.h"
#include <atomic>
#include <thread>
#include <memory>
#include <string>
#include <vector>

struct ImVec4;

namespace es {

// one widget of the custom dashboard
struct DashWidget {
    int ch = 0;                 // log channel
    int type = 2;               // 0 analogue gauge, 1 digital, 2 bar
    float lo = 0, hi = 100;     // display range
    int useLo = 0;
    float warnLo = 0, alarmLo = 0;   // (kept next to each other: edited as a pair)
    int useHi = 0;
    float warnHi = 0, alarmHi = 0;
};

struct DynoRun {
    std::string label;
    std::vector<DynoPoint> pts;
    float color[4];
    bool visible = true;
    float peakTorque = 0, peakTorqueRpm = 0, peakPower = 0, peakPowerRpm = 0;
    std::string engine, calName;   // what the run was made with (before / after tuning)
    int calRev = 0;
};

class App {
public:
    // navigation sections (ECU-software style)
    enum Section { kEngineSetup, kEcuSetup, kFuelTuning, kIgnitionTuning, kBoostControl, kEngineControl, kSensorConfig,
                   kEngineProtection, kLiveDashboard, kDataLogging, kVirtualDyno, kCalibrationManager, kSectionCount };

    explicit App(int preset = 0);
    ~App();
    void frame(float dt);                 // called once per UI frame
    void setSection(int s) { section_ = s; }

private:
    // application shell (Layout.cpp)
    void shell();
    void headerBar();
    void navBar();
    void workArea();
    bool sectionImplemented(int s) const;
    void plannedSection(const char* phase, const char* const* items, int n, const char* existing);
    void sectionEcuSetup();
    void sectionFuelTuning();
    void sectionIgnitionTuning();
    // Phase 3 (Control.cpp)
    void tabFuelTrims();
    void tabKnock();
    void tabBoostControl();
    // Phase 5: motorsport
    void tabLaunch();
    void tabTraction();
    void tabAntiLag();
    void tabNitrous();
    void sectionEngineControl();
    void sectionProtection();
    // Phase 4: data logging (Logging.cpp)
    void sectionDataLogging();
    // dyno extras (Phase 4)
    bool dynoKw_ = false, imperial_ = false;
    int dynoBase_ = -1;
    void dynoExportCsv();
    // custom dashboard (Dashboard.cpp)
    void panelCustomDash();
    void dashDefaults();
    void dashSave();
    void dashLoad();
    std::vector<DashWidget> dash_;
    int dashCols_ = 4;
    bool dashEdit_ = false, dashLoaded_ = false;
    void logFrame();
    void logStart();
    void logStop();
    void logMarker(const std::string& label);
    std::vector<LogSession> logSessions_;
    std::vector<std::vector<int>> logPanels_;
    std::vector<char> logExport_;
    int logRec_ = -1, logView_ = -1, logCmp_ = -1, logRateIdx_ = 2, logCounter_ = 0;
    int logPrevLaunch_ = 0;
    bool logPrevN2o_ = false;
    bool logXRpm_ = false, logPrevLimp_ = false, logFitReq_ = true, logFitRpm_ = false;
    int logFitView_ = -2;
    int logPrevProt_[kProtCount] = {}, logPrevGear_ = 0;
    double logT0_ = -1;
    char logMarkText_[64] = {};
    std::string logMsg_;
    // calibration manager (CalManager.cpp)
    void sectionCalibrationManager();
    void calRefresh();
    void calOnPresetChanged();
    void calApplyLoaded(const Calibration& c, const CalMeta& m, const char* what);
    bool calSave(const char* name);
    void calMessage(int severity, const std::string& s);
    CalMeta calMeta_;
    Calibration calSaved_{};
    std::vector<CalFileInfo> calList_, calHist_;
    std::vector<CalIssue> calIssues_;
    std::string calListEngine_, calMsg_;
    int calMsgSev_ = 0, calSel_ = -1, calTab_ = 0;
    bool calListDirty_ = true, calTabForce_ = false;
    char calName_[64] = {}, calNotes_[2048] = {};
    // calibration editing (Tuning.cpp)
    ui::TableEditor tabEd_[kCalTableCount], curveEd_[kCalCurveCount];
    std::vector<ui::OpPoint> trailMap_, trailTps_;
    float trailAcc_ = 0;
    void recordOpTrail(float dt);
    Calibration refCal_{};
    bool hasRef_ = false;
    std::string refName_;
    // measured base map (virtual dyno calibration in a background thread)
    void startBaseMap(bool force = false);
    void stopBaseMap();
    void pollBaseMap();
    std::thread bmThread_;
    BaseMapProgress bmProg_;
    std::atomic<bool> bmDone_{false};
    bool bmRunning_ = false;
    EngineParams bmFor_{};
    Calibration bmResult_{};
    Table2D bmVeBefore_{}, bmTpsBefore_{};
    // the engine the current VE maps were measured for, and the maps as measured (untouched by the tuner?)
    uint64_t bmKeyApplied_ = 0, bmKeySeen_ = 0;
    Table2D bmVeApplied_{}, bmTpsApplied_{};
    float bmKeyStable_ = 0;
    bool bmStale_ = false;
    void baseMapFollowEngine(float dt);
    static uint64_t airPathKey(const EngineParams& p);
    std::string baseMapInfo_;
    void sectionSensors();
    int section_ = kLiveDashboard;
    // panels (Panels.cpp)
    void panelDashboard();
    void panelEngine();
    void panelValvetrain();
    void panelRotary();
    void panelPorts();
    void panelBoost();
    void panelVehicle();
    void panelIgnition();
    void panelFuelIntake();
    void panelExhaust();
    void panelDyno();
    void panelScope();
    void panelEngineView();
    void panelAudio();
    void panelComponents();
    void keyboard(float dt);
    void pullDyno();
    void loadPreset(int i);

    EngineParams p_;
    EngineParams sent_;
    std::unique_ptr<Simulator> sim_;
    AudioOutput audio_;
    Telemetry tel_;
    std::vector<DynoRun> runs_;
    int lastRunId_ = -1;
    int presetIdx_ = 0;
    bool keyThrottle_ = false, brakeKey_ = false, clutchKey_ = false;
    float animAngle_ = 0, animSpeed_ = 0.02f;
    ui::ViewCam engCam_;
    bool autoStart_ = false;
    float autoStartTimer_ = 0, caughtTimer_ = 0;
    bool firstFrame_ = true;
    bool equalPrimaries_ = true;
    bool showDemo_ = false;
    int learnSeen_ = -1, ltftSeen_ = -2;
    float scopeZoom_ = 1;
    std::vector<float> fftMag_;
    std::vector<float> mapTrailX_, mapTrailY_;
    // scope history (boost graph) and measured maps (fuel map)
    struct History { std::vector<float> t, boost, target, shaft, wg, thr, afr; float now = 0; };
    History hist_;
    float histAccum_ = 0;
    static constexpr int kMapR = 21, kMapM = 32;           // 500 rpm x 10 kPa cells
    float afrMap_[kMapR * kMapM] = {}, dutyMap_[kMapR * kMapM] = {}, advMap_[kMapR * kMapM] = {};
    unsigned char hitMap_[kMapR * kMapM] = {};
    int fuelMapView_ = 0;
    std::vector<float> opTrailX_, opTrailY_;
    void recordScope(float dt);
    // live dyno: recent torque / power vs rpm while revving, plus a peak-hold envelope per rpm bin
    std::vector<float> liveRpm_, liveTq_, livePw_, liveAge_;
    static constexpr int kPeakBins = 400;                 // 50 rpm bins up to 20 000 rpm
    float peakTq_[kPeakBins] = {}, peakPw_[kPeakBins] = {};
    bool showLive_ = true, showPeak_ = true;
    void resetPeak();
    // custom impulse response
    bool loadIrFile();
    std::string irFileName_, irError_;
    void panelBoostGraph();
    void panelFuelMap();
};

} // namespace es
