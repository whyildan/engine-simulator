<div align="center">

# EngineSim

**A real-time internal-combustion engine simulator — with sound that comes out of the physics and a full ECU tuning platform.**

[![Release](https://img.shields.io/github/v/release/whyildan/engine-simulator?sort=semver)](https://github.com/whyildan/engine-simulator/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Platform](https://img.shields.io/badge/platform-Windows%20x64-lightgrey)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C)

![Virtual dyno: WOT sweep of the I6 3.0 preset](docs/images/dyno.png)

</div>

EngineSim simulates every cylinder (or every rotor face) of an engine in real time: gas exchange, combustion,
heat, friction and the pressure waves in the intake and exhaust. The sound you hear is **not sampled** — it is
the exhaust flow of the simulated engine, radiated from the tailpipe. Change the firing order, the header
lengths or the rotary porting and the engine *sounds* different because it *is* different.

On top sits an ECU that only sees virtual sensors and runs on editable calibration maps, like real tuning
software: VE / lambda / ignition tables, closed-loop fuel trims, knock control, boost control, engine
protection, a virtual dyno and a calibration manager. Everything is changed **live, while the engine runs**.

## Highlights

| | |
|---|---|
| 🔊 **Physics-based sound** | Waveguide intake / exhaust network, per-pipe gas temperature, impulse-response convolution, mechanical noise, afterfire pops. |
| ⚙️ **Any layout** | Inline, V, boxer, 1–16 cylinders, odd-fire / big-bang, custom firing orders — and **Wankel rotaries** with 1–4 rotors and four porting styles. |
| 🌀 **The rotary "brap"** | A bridge-ported 13B idles lumpy because exhaust floods back through the port overlap and the diluted charges misfire — modelled, not faked. |
| 🔧 **ECU tuning platform** | Table editor with live operating point and 3D view, speed density / alpha-N / MAF / hybrid load, enrichments, STFT / LTFT, per-cylinder knock control, ECU boost control; trigger wheels (60-2, 36-1, 36-2-2-2 …) with crank / cam sync, sequential / semi-sequential / batch injection, coil dwell table. |
| 🛡️ **Engine protection** | Ten configurable rules (overboost, oil pressure, coolant, lean, knock, EGT, sensor failure …) with limp mode and shutdown. |
| 🌬️ **Forced induction** | Turbos with lag, wastegate, BOV flutter and anti-lag; compressor / turbine maps (built in or loaded from CSV) with efficiency islands, surge and choke; intercooler heat soak; Roots, twin-screw and centrifugal superchargers. |
| 📈 **Virtual dyno** | WOT sweeps, steady-state step tests and holds, SAE J1349 / DIN 70020 / ISO 1585 / SAE J607 correction, altitude, cell fan, BSFC / EGT / boost per point, before / after comparison, CSV export. |
| 🔩 **Valvetrain dynamics** | Spring-mass valves on an elastic train: valve float, loft, late closing and seat bounce past the float speed, from the springs, moving mass and lifter type. |
| 🏁 **Motorsport** | Launch control (2-step) with pops and bangs, 3-step / flat shift, shift cut, traction control, transbrake, rolling anti-lag, progressive wet / dry nitrous, torque management and rev limit / timing by gear. |
| 🗂️ **Data logging** | 55 channels at 10–100 Hz, markers, stacked graphs over time or rpm, CSV export / import, session compare tied to the calibration version. |
| 🎛️ **Custom dashboard** | Your own grid of gauges, digital readouts and bars with warning / alarm thresholds and a sensor status strip. |
| 🚗 **Drive it** | Car or motorcycle as the load: manual / auto-clutch / automatic gearbox, clutch, tyre slip, 0–100 and 1/4-mile timers. |
| 🧪 **Virtual sensors** | Lag, noise, 10-bit ADC, transport delay, faults (open / short / stuck / noisy) and the ECU's fallbacks. |
| 💾 **Calibration manager** | JSON calibrations per engine, version history, compare, locks, validation. |
| 📤 **Share engines** | Save the whole engine (hardware, car, sound and its calibration) as your own preset under *My engines*, export it as one `.engine.json` file and import someone else's. |

| | |
|---|---|
| ![VE map editor with 3D surface](docs/images/ve-map-3d.png) | ![Live dashboard](docs/images/dashboard.png) |
| *Fuel Tuning — VE map with operating point and 3D surface* | *Live Dashboard — gauges, readouts, rotatable 2.5D engine view* |
| ![Engine setup](docs/images/engine-setup.png) | ![Engine protection](docs/images/protection.png) |
| *Engine Setup — geometry, firing order, engine view (K20A i-VTEC)* | *Engine Protection — rules, live state, actions* |

## Getting started

**Download:** grab `EngineSim-<version>-win64.zip` from the [Releases](https://github.com/whyildan/engine-simulator/releases)
page, unzip it anywhere and run `enginesim.exe` (Windows 10/11 x64, any DirectX 11 GPU). The release page also
has short audio samples.

Then: pick a preset in the header, press **START ENGINE** (or hold `S`), and use `W` / `Space` for throttle.
The first start of each engine measures its base fuel map on a virtual dyno in the background (a few seconds,
cached afterwards in `basemaps/` next to the exe).

```bash
enginesim.exe --preset 23 --section 11   # start with the bridge-port rotary, in the Virtual Dyno section
```

### Controls

| Key | Action |
|---|---|
| `S` (hold) | starter (switches the ignition on) |
| `I` | ignition on / off |
| `W` / `↑` (hold) | throttle (opens gradually) · `Space` (hold) full throttle |
| `1`…`9`, `0` | fixed throttle 10 %…90 %, 100 % · `↓` release |
| `D` | dyno sweep |
| `R` | data log: start / stop recording · `M` marker |
| `C` + full throttle, standing | 2-step launch limiter (when launch control is on) |
| `T` (hold) | transbrake · `G` (hold) rolling anti-lag · `X` nitrous arming switch |
| `L` | listener: tailpipe / engine bay / cabin |
| `V` | driving mode (car as the load) |
| `E` / `Q` | shift up / down · `N` neutral · `C` (hold) clutch · `B` brake |

### Presets

| # | Engine | # | Engine |
|---|---|---|---|
| 0 | **I4 2.0 K20A-style (i-VTEC, 8600 rpm)** | 12 | Rotary 13B 2-rotor |
| 1 | I3 1.0 EFI | 13 | Rotary 20B 3-rotor (street port) |
| 2 | I6 3.0 EFI | 14 | Rotary 26B 4-rotor (peripheral port race) |
| 3 | V6 3.5 EFI | 15 | Rotary 1-rotor carb |
| 4 | V8 5.7 crossplane carb | 16 | I4 2.0 turbo (hot hatch) |
| 5 | V8 4.5 flatplane EFI | 17 | V8 6.2 Roots blower |
| 6 | V10 5.2 EFI | 18 | Rotary 13B turbo |
| 7 | V12 6.0 EFI | 19 | I6 3.0 big single turbo + anti-lag |
| 8 | Flat-4 2.5 boxer (unequal headers) | 20 | V6 3.5 centrifugal supercharger |
| 9 | V-twin 1.6 45° carb | 21 | I5 2.5 EFI |
| 10 | Single 0.5 thumper carb | 22 | Flat-6 3.6 boxer |
| 11 | I4 1.6 ITB race | 23 | **Rotary 13B bridge port (brap idle)** |

## Building from source

Requirements: Windows, Visual Studio 2019 or newer (MSVC, C++17), CMake ≥ 3.16. Dependencies — Dear ImGui
1.90.9-docking, ImPlot 0.16, ImPlot3D 0.2, miniaudio 0.11.21 — are fetched automatically on the first configure.

```bash
cmake -S . -B build -G "Visual Studio 16 2019" -A x64
```

```bash
cmake --build build --config Release
```

If CMake is not on your PATH, use the one bundled with Visual Studio
(`…/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe`). The app is `build/Release/enginesim.exe`.

## The application

A fixed header (ignition, START ENGINE, throttle, preset, RPM / MAP / AFR / timing / boost / EGT readouts and
warning lamps) and a 12-section navigation in the style of ECU software:

1. **Engine Setup** — geometry, valvetrain / rotary ports, exhaust, vehicle, sound, components, and the
   **engine view**: flat cutaways of every cylinder / rotor laid out along the crankshaft in a rotatable 3D
   scene (drag to orbit, wheel to zoom; live valve lift, VTEC cam, spark and gas by stroke)
2. **ECU Setup** — loop rate, trigger wheel and cam sensor, injection timing, coil type and dwell, cold start, simulation pause / reset
3. **Fuel Tuning** — VE (speed density / alpha-N), lambda, enrichments, corrections, injectors, closed loop, base map
4. **Ignition Tuning** — ignition map, corrections, knock control, per-cylinder trims
5. **Boost Control** — turbo / supercharger hardware (generic or compressor / turbine maps), ECU boost control, anti-lag, nitrous
6. **Engine Control** — idle control, drive-by-wire pedal map, soft / hard rev limiter, cam control (VTEC / VTC), launch / shift, traction control, torque limit and rev limit / spark trim by gear
7. **Sensor Configuration** — true vs measured values, noise, lag, fault injection
8. **Engine Protection** — protection rules, limp mode, shutdown
9. **Live Dashboard** — gauges, readouts, scopes (pressure, P-V, boost, fuel map, sound) and a configurable
   *Custom* dashboard (layout saved to `dashboard.cfg` next to the exe)
10. **Data Logging** — record channels, markers, graphs vs time / rpm, CSV export / import, session compare
11. **Virtual Dyno** — sweeps, step tests, holds, correction standards, cell fan, before / after comparison, Nm / kW or lb-ft / hp, CSV export
12. **Calibration Manager** — files, versions, compare, validation

<details>
<summary><b>Everything you can configure (all live)</b></summary>

- **Engine**: piston / rotary; inline / V / boxer; 1–16 cylinders, bank angle, bore, stroke, rod, compression,
  flywheel, **firing order**, even firing or **free per-cylinder phasing** (odd-fire V-twin, big-bang), exhaust
  bank split.
- **Valvetrain**: valves and diameters, cam duration, lift, centerline / LSA, cam advance (VVT), lash, with a
  valve lift / overlap graph; **VTEC-style two-stage cams** (high-cam profile, ECU switch-over by rpm / load /
  coolant with hysteresis) and a **VTC-style intake cam phaser** (advance map over rpm × MAP).
- **Fuel & intake**: EFI (port / throttle body, injector size and dead time) or a carburettor (barrels, venturi,
  jets, idle screw, accelerator pump, choke); throttle body or ITBs, plenum, runners, air filter.
- **Exhaust**: log manifold / N-into-1 / tri-Y headers, per-cylinder primary lengths ("boxer rumble"),
  collector, true dual / H / X / Y crossover, cat, resonator, muffler type, tailpipe.
- **Sound**: *Smooth* (engine-sim style: exhaust pulse pressure + band limit + air noise + impulse response) or
  *Raw* (pure tailpipe flow radiation); pulse pressure mix, rasp (jet noise), punch, optional wave steepening,
  environment reverb (open road, garage, dyno cell, tunnel), impulse responses (built-in or **your own WAV**),
  listener position (tailpipe / engine bay / cabin), *imperfection* (cylinder-to-cylinder and cycle-to-cycle
  spread).
- **Components** — every part has a physical effect: piston / rod / crank material and mass (inertia, safe rpm),
  ring tension (friction), lifters and springs (**valve float** from a spring-mass valvetrain model, with a valve
  motion plot at any rpm), throttle type, injectors, coils and plugs
  (misfire, pre-ignition, fouling), oil and water pumps. Optional **mechanical failure** when parts are
  overloaded, the engine knocks hard or valves hit pistons.
- **Separate ECU + virtual sensors**: TPS, pedal, MAP, MAF, IAT, ECT, narrowband O2, wideband, CKP, CMP, knock,
  EGT, oil pressure / temperature, fuel pressure, speed, gear, battery — with lag, noise, ADC, faults and
  out-of-range fallbacks (MAP failure → alpha-N limp, CMP lost → batch fire, CKP lost → engine stops).
- **Ambient**: temperature and altitude (barometric pressure from the standard atmosphere; the ECU reads it).
- **Thermal, fuel supply, electrics**: coolant / oil / intake temperatures, thermostat, radiator and fan;
  rail pressure from pump + regulator; battery and alternator (injector dead time follows the voltage).
- **ECU calibration**: VE maps (speed density and alpha-N), target lambda, ignition, cranking / warm-up /
  after-start / acceleration / DFCO-exit enrichments, IAT / ECT corrections, injector data, per-cylinder trims,
  load mode SD / alpha-N / MAF / hybrid. The base VE map is **measured on a virtual motoring dyno** for every
  engine (and again when you change its air path); wideband auto-tune refines it.
- **ECU control loops**: STFT + learned LTFT table (learning conditions, limits), per-cylinder knock control via
  a crank-angle knock window, idle target over coolant temperature with PID and idle spark control,
  drive-by-wire pedal map, soft limiter (retard + partial spark cut), ECU boost control (target over rpm ×
  throttle × gear, feed-forward + PID to a wastegate solenoid, overboost cut, boost cut).
- **Motorsport**: launch control (2-step: rpm, timing, spark or fuel cut, enrichment), clutch-in limiter while
  moving (3-step at a fixed rpm, or flat shift holding the clutch-in rpm), shift cut on the gear-change signal,
  traction control (slip target over speed, PI → retard then spark cut), transbrake (automatic / auto-clutch:
  holds the car, the engine loads against it like a converter at stall), anti-lag on lift-off and rolling
  anti-lag on a button, nitrous (shot size, wet or dry kit, bottle, arming switch, rpm / throttle / coolant
  window, delay, progressive ramp, timing retard, lean shut-off).
- **Data logging & dashboard**: pick channels and a rate (10 / 20 / 50 / 100 Hz of simulated time), add markers
  (automatic ones on protection actions, limp mode, gear changes, launch and nitrous), up to 6 stacked graph panels with linked
  zoom / pan over time or rpm, overlay a second session, export the channels you choose to CSV (with engine,
  calibration name and revision in the header) and load CSV logs back. Custom dashboard widgets show any log
  channel as a gauge, digital readout or bar with low / high warning and alarm thresholds.
- **Engine protection**: over-rev, overboost, coolant / oil temperature, oil / fuel pressure, lean at load,
  excessive knock, EGT, sensor failure — each with warning, limit, delay, recovery and an action (warning,
  retard, boost reduction, throttle limit, fuel limit, limp mode, shutdown).
- **Rotaries**: 1–4 rotors, R / e / B, compression, apex seal wear, port timing and area, porting presets
  (stock side port, street port, **bridge port**, peripheral port).
- **Forced induction**: turbo (compressor size, turbine A/R, inertia, wastegate, BOV, anti-lag) or supercharger
  (Roots, twin-screw, centrifugal; size, pulley, bypass), intercooler, live compressor map.
- **Vehicle**: gears and ratios, final drive, clutch, mass, Cd·A, rolling resistance, tyres and grip, gradient.

Pipe length / diameter changes glide smoothly; topology changes (cylinders, engine type, header type …)
rebuild the engine with a short sound crossfade, without stopping it.

</details>

<details>
<summary><b>How it works — the physics</b></summary>

- **Cylinders**: control volumes (mass, energy, fuel and burned-gas fractions), slider-crank kinematics,
  compressible orifice flow through the valves, **Wiebe** combustion whose duration depends on rpm, λ and
  residual gas, cycle-to-cycle variation, misfire (lean, rich, dilution, weak spark, fouled plug), Woschni heat
  transfer, end-gas knock index. The exhaust temperature is mass-weighted over each exhaust event; the gas loses
  heat to the water-cooled port (Dittus–Boelter convection, pulsating-flow factor) and the EGT gauge is a
  thermocouple in the header (convection from the gas, radiation to the slowly warming pipe wall) — so idle reads
  ~450 C, full load ~850–950 C, and a lift-off barely moves it. Fuel that burns in the manifold (overrun pops, a spark-cut 2-step) heats the
  exhaust gas further — that is what spools a turbo on the 2-step.
- **Afterfire**: unburned fuel only pops where it meets oxygen in a flammable mixture — a misfired or cut charge,
  or the overrun. The exhaust of a rich burn has no oxygen; mixture that slips through a firing cylinder
  (overlap scavenging, late burning) keeps burning as a continuous flame (heat, no bang); the trace of fuel the
  port walls give off during a fuel cut is far too lean to ignite. Fuel-cut cycles do not count as misfires.
- **Nitrous**: N₂O is fed to the ports as air-equivalent charge (its extra oxygen plus its decomposition heat),
  with a wet kit's own fuel and charge cooling from the liquid flashing to gas; a 100 hp jet flows ~32 g/s.
- **Intake & exhaust**: a **digital waveguide network** — bidirectional delay lines with frequency-dependent
  losses, N-way scattering junctions, gas composition carried along each pipe, and open ends that radiate. Each
  pipe has its own gas temperature following the EGT, so resonances move with load.
- **Sound**: the tailpipe flow radiates as a monopole; the *Smooth* chain adds the exhaust pulse pressure and
  band-limits it, then an impulse response supplies the muffler / body timbre. Structure-borne noise comes from
  block and head resonance modes excited by combustion, valve seating and pops; the cabin listener hears it
  through a low-pass and cabin booms.
- **Rotary**: three chambers per rotor, 1080° cycle, `V = Vc + Vd/2·(1 − cos(2θ/3))`, `Vd = 3√3·R·e·B`, ports
  opened by the rotor flanks, slow long-chamber combustion, apex seal leakage. A big port overlap lets exhaust
  revert into the intake at idle; the diluted charges misfire, the next one fires on fresh charge and the
  unburned mixture pops in the exhaust — the "brap brap" of a ported rotary.
- **Forced induction**: centrifugal compressor maps with surge and choke, turbine power from the exhaust energy,
  shaft dynamics (natural lag), positive-displacement blowers with leakage. With map tables the compressor runs
  on corrected speed and flow (speed lines resampled along their length, interpolated between lines, fan laws
  outside them) and the turbine on its reduced flow vs expansion ratio and efficiency vs blade speed ratio.
  The intercooler is a heat exchanger against a core that soaks heat and sheds it to the air blown through it.
- **Valvetrain**: one valve event integrated per rpm: mass on its spring, driven through an elastic train
  (stiffness by lifter type), landing on an elastic seat. The deviation from the cam lift (loft, late closing,
  seat bounce) goes straight into the valve flow area.
- **ECU**: a separate controller at its own loop rate (100–5000 Hz) that only reads the sensor layer;
  injection by crank angle, X-tau wall-film compensation, asynchronous tip-in fuel.
- **Vehicle**: friction clutch with stick / slip, gearbox, wheel inertia, Pacejka-like tyre slip curve,
  aerodynamic drag, gradient, brakes — clutch torque feeds back into the crank (engine braking, push starts).

More detail on the ECU platform and its roadmap: [docs/ECU_PLAN.md](docs/ECU_PLAN.md).

</details>

<details>
<summary><b>Headless test & analysis tools</b></summary>

```bash
build/Release/sim_tests.exe survey   # every preset: start, idle, blip, return to idle
```

```bash
build/Release/sim_tests.exe fuzz 300 # random live parameter changes, checks for NaN / runaway states
```

```bash
build/Release/control_test.exe      # fuel trims, knock control, soft limiter, idle / DBW, boost control, protection
```

```bash
build/Release/ecu_test.exe          # sensors, ECU fallbacks, closed loop, thermal, fuel pump, ECU rate
```

```bash
build/Release/cal_test.exe          # calibration files: JSON round trip, validation, diff, version history
```

```bash
build/Release/engine_file_test.exe  # engine files: every preset round trips, partial / bad files, save / export / import
```

```bash
build/Release/parts_test.exe        # physical effect of each component
```

```bash
build/Release/log_test.exe          # data logging: sampling, markers, CSV round trip
```

```bash
build/Release/misfire_probe.exe 0   # misfires (by cause) and pops per phase: idle, cruise, blips, WOT, decel
```

```bash
build/Release/dyno_sweep.exe 0      # WOT sweep: torque, power, AFR, spark, VTEC / VTC, VE per rpm
```

```bash
build/Release/motorsport_test.exe   # 2-step, flat shift, shift cut, traction control, transbrake, rolling ALS, nitrous
```

```bash
build/Release/phase6_test.exe       # turbo maps + CSV, valve float, altitude, intercooler heat soak, dyno step test / correction
```

```bash
build/Release/valve_probe.exe 4     # valvetrain dynamics of a preset: float speed, loft / bounce over rpm
```

```bash
build/Release/phase7_test.exe       # trigger wheels / sync, injection modes, dwell, torque management, gear control
```

```bash
build/Release/basemap_warm.exe 14   # measure every preset's base map in parallel (fills the cache)
```

```bash
build/Release/egt_probe.exe         # EGT / spark / AFR per preset at idle, part load and WOT
```

```bash
build/Release/drive_test.exe 0       # driving: automatic, manual + auto clutch, manual, clutch dump
```

```bash
build/Release/idle_hold.exe 30       # every preset idles 30 s on its own - reports any stall
```

```bash
build/Release/idle_probe.exe 0       # idle quality: rpm spread, idle valve jumps, cycle-to-cycle IMEP (CoV), thumps
```

```bash
build/Release/egt_trace.exe 0        # EGT at idle and WOT, and a lift-off from the redline traced every 20 ms
```

```bash
build/Release/silence_probe.exe      # every preset must be silent with the engine off
```

```bash
build/Release/stop_probe.exe 0      # an engine parked at any crank angle / after key off must be silent
```

```bash
build/Release/op_trace.exe 11       # idle, 3000 rpm part load, WOT: operating point, lambda per cylinder, protection
```

```bash
build/Release/render_wav.exe 23 brap.wav idle=8   # record 8 s of warm idle to WAV
```

```bash
python tools/analyze_wav.py brap.wav 0 8 900       # level + spectral peaks (engine orders)
```

</details>

## Roadmap

- [x] Phase 1 — ECU-style UI, component catalogue, separate ECU with virtual sensors, thermal model
- [x] Phase 2 — calibration tables, table editor, measured base maps, calibration manager
- [x] Phase 3 — closed-loop trims, knock control, idle / DBW, ECU boost control, engine protection
- [x] Phase 4 — configurable dashboard, data logging with CSV export, dyno export
- [x] Phase 5 — launch control, 2-step / 3-step, flat shift, traction control, transbrake, nitrous
- [x] Phase 6 — compressor / turbine maps, valvetrain dynamics, altitude, intercooler heat soak, dyno step test and correction
- [x] Phase 7 — trigger wheels and sync, sequential / semi-sequential / batch injection, coil dwell, torque management, gear-based control

## Acknowledgements

The *Smooth* sound chain (pulse pressure + air noise + impulse response) is inspired by
[engine-sim](https://github.com/ange-yaghi/engine-sim) by Ange Yaghi; the per-pipe temperature zones,
structure-borne modes and cabin model by [ICEsim](https://github.com/Agent00PED/ice-simulator). Both MIT.
Built with [Dear ImGui](https://github.com/ocornut/imgui), [ImPlot](https://github.com/epezent/implot),
[ImPlot3D](https://github.com/brenocq/implot3d) and [miniaudio](https://github.com/mackron/miniaudio).

## License

MIT — see [LICENSE](LICENSE). Third-party libraries keep their own licenses (Dear ImGui, ImPlot, ImPlot3D: MIT;
miniaudio: public domain / MIT-0).
