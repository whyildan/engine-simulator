#pragma once
#include "sim/EngineParams.h"
#include "imgui.h"

namespace es::ui {

// round analogue gauge (tachometer style)
void Gauge(const char* id, float value, float vmin, float vmax, float redFrom, const char* label,
           const char* fmt, float size, int majorTicks = 10, float tickScale = 1.0f);
// horizontal bar meter with numeric readout
void Meter(const char* label, float value, float vmin, float vmax, const char* fmt, ImU32 color, float width = -1);
// round status lamp
void Lamp(const char* label, bool on, ImU32 color);
// animated cross-section of the engine (pistons, rods, crank, valves, spark)
void EngineAnimation(const EngineParams& p, const float phases[kMaxCyl], float crankDeg, ImVec2 size);
// animated Wankel rotary: epitrochoid housing, rotor, eccentric shaft, ports, plugs
void RotaryAnimation(const EngineParams& p, const float phases[kMaxCyl], float shaftDeg, ImVec2 size);
// block diagram of the exhaust system
void ExhaustSchematic(const EngineParams& p, ImVec2 size);
// help marker "(?)" with tooltip
void Help(const char* text);

} // namespace es::ui
