# EngineSim — real-time engine simulator (C++)

An internal-combustion engine simulator with per-cylinder physics, sound **generated from the physics**
(pressure waves in the exhaust and intake), a dyno, and an ECU tuning platform. Everything is configured
**live inside the application while the engine is running** (no config files). Both **piston** and
**Wankel rotary** engines are supported.

## Build (Windows, MSVC 2019 + CMake)

If CMake is not on your PATH, use the one bundled with Visual Studio:

```bash
"/c/Program Files (x86)/Microsoft Visual Studio/2019/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" -S . -B build -G "Visual Studio 16 2019" -A x64
```

```bash
"/c/Program Files (x86)/Microsoft Visual Studio/2019/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" --build build --config Release
```

Dependencies (Dear ImGui 1.90.9-docking, ImPlot 0.16, ImPlot3D 0.2, miniaudio 0.11.21) are downloaded
automatically through FetchContent on the first configure.

Run `build/Release/enginesim.exe`, or start directly with a given preset:
`build/Release/enginesim.exe --preset 12` (13B rotary).

## Controls

| Key | Action |
|---|---|
| `S` (hold) | starter / crank (switches the ignition on automatically) |
| `I` | ignition on/off |
| `W` / `↑` (hold) | throttle (opens gradually, release = closes) |
| `Space` (hold) | full throttle (WOT) |
| `1`…`9`, `0` | fixed throttle 10 %…90 %, 100 % |
| `↓` | release throttle |
| `D` | run a dyno sweep |
| `L` | change listener position (tailpipe / engine bay / cabin) |
| `V` | driving mode on/off (car as the load) |
| `E` / `Q` | shift up / down (automatic: `E` = D, `Q` = N) |
| `N` | neutral |
| `C` (hold) | clutch pedal down (released slowly, ~0.4 s) |
| `B` / `↓` (hold) | brake |

The **START ENGINE** button cranks until the engine runs on its own.

### Layout

The header at the top is always visible: ignition, START ENGINE, crank, throttle, preset selection, and the
main readouts (RPM, MAP, AFR, timing, boost, EGT, torque, gear). On the left is a 12-section navigation in the
style of ECU software: Engine Setup, ECU Setup, Fuel Tuning, Ignition Tuning, Boost Control, Engine Control,
Sensor Configuration, Engine Protection, Live Dashboard, Data Logging, Virtual Dyno, Calibration Manager.
Dimmed sections are not built yet and only describe what is planned (see [docs/ECU_PLAN.md](docs/ECU_PLAN.md)).
`build/Release/enginesim.exe --section 11` opens a given section directly (1–12).

## What can be configured (all in real time)

- **Engine**: Piston / Rotary engine type; Inline / V / Boxer layout, 1–16 cylinders, bank angle, bore, stroke,
  conrod length, compression, flywheel inertia, **firing order** (reorder with arrows), even firing or
  **free per-cylinder phasing** (odd-fire V-twin, big-bang), and the left/right exhaust bank split.
- **Valvetrain**: number and diameter of valves, cam duration, lift, centerline/LSA, cam advance (VVT),
  valve lash, plus a valve lift + overlap graph.
- **Ignition**: base / mechanical / vacuum advance, rev limiter (ignition cut = pops, fuel cut).
- **Fuel & Intake**: **EFI** (port / throttle-body, injector size, target AFR, tuned vs untuned ECU, idle target)
  or a **carburettor** (barrels, venturi, main jet, idle mixture, idle screw, accelerator pump, choke);
  throttle body, ITBs, plenum volume, runner length/diameter, air filter.
- **Exhaust**: log manifold / N-into-1 header / tri-Y 4-2-1, primary length (equal or different per cylinder —
  "boxer rumble"), diameter, collector, crossover (true dual / H-pipe / X-pipe / Y), catalytic converter,
  resonator, muffler (straight / glasspack / chambered / turbo), tailpipe.
- **Sound style** (Audio tab): *Smooth* (default, engine-sim style: the main signal is the exhaust pulse
  pressure, band-limited by a 4th-order *Smoothing* low-pass, with subtle air noise, then shaped by an impulse
  response) or *Raw* (the original model: tailpipe flow radiation only, no extra layers, no IR, identical
  cylinders). *Pulse pressure mix* sets the share of pulse pressure (round, full) vs flow radiation (brighter).
- **Audio**: listener position, volume, physics oversampling, *Rasp* (jet turbulence noise at the tailpipe,
  strength ∝ gas velocity², frequency following Strouhal 0.3·v/D), *Punch* (low-frequency pulse pressure
  component), *Wave steepening* (optional, off by default: high-pressure pulses travel faster and sharpen —
  adds bite but also roughness), and *Environment* (open road / garage / dyno cell / tunnel, feedback-delay-network
  reverb) + reverb amount. The air-noise idea and the pressure/derivative mix are inspired by
  [engine-sim](https://github.com/ange-yaghi/engine-sim), but derived from the simulated flow.
- **Impulse response (convolution)**: the exhaust sound is passed through an IR with a partitioned FFT
  convolution (256-sample latency). Choices: *Auto* (matches the muffler / bike), *Raw pipe*, *Sport*,
  *Stock muffler*, *Race*, *Motorcycle* (procedural: dense early reflections, a decaying tail, a muffler body
  resonance), or **Load WAV...** for your own IR (PCM 16/24/32-bit / float, resampled automatically).
  IRs from engine-sim's `ir_lib` (MIT license) can be used — keep their license notice.
- **Imperfection** (Engine tab): small differences between cylinders (port flow, injector, timing, combustion,
  primary length; zero on average) + cycle-to-cycle variation, so the sound is not sterile like a perfect engine.
- **Components (Engine Setup → Components)**: every part has a physical effect.
  - *Piston / connecting rod / crankshaft*: material and mass set the extra inertia and the **safe rpm limit**
    (inertia forces vs the weakest part's strength); piston ring tension adds friction.
  - *Lifters + valve springs*: moving mass vs spring force sets the **valve float** rpm. Above it the valves
    close late and bounce: torque drops, misfires. Stiffer springs = more friction.
  - *Throttle body*: cable or drive-by-wire (motor response time); *injectors* (flow rate + dead time).
  - *Coil* (distributor / wasted spark / coil-on-plug, energy, charge time) and *spark plugs* (heat range, gap):
    too little spark energy = misfire. Plug too hot = pre-ignition/knock, too cold + rich = fouling.
  - *Oil / water pump*: oil pressure vs rpm (Oil lamp in the header) and drive power (friction).
  - **Allow mechanical failure** option: overloaded parts, heavy knock, or valves hitting pistons accumulate
    damage until the engine fails and seizes (*Repair engine* button).
  - *Size parts to this engine* picks parts that are safe for the current rev limiter. Raising the limiter
    without changing parts can cause valve float or a failure.
- **Separate ECU + virtual sensors**: the ECU (*ECU Setup*) only reads sensors (TPS, pedal, MAP, MAF, IAT, ECT,
  narrowband O2, wideband, CKP, CMP, knock, EGT, oil pressure & temperature, fuel pressure, speed, gear,
  battery). The ECU loop rate is adjustable (100–5000 Hz). *Sensor Configuration* shows the true value vs what
  the ECU reads, signal voltage and status, and lets you set noise, lag and faults (open, short, stuck, noisy).
  The ECU detects out-of-range signals and uses fallbacks (MAP failure → estimate from throttle opening, CKP lost
  → engine stops, CMP lost → batch fire). A *stuck* sensor goes undetected, just like the real thing.
- **Thermal model**: coolant, oil and intake air temperatures; thermostat, radiator (vehicle speed + fan), water
  pump. Cold oil = more friction; cold port walls = more fuel film and some fuel not evaporating. **Cold start**
  option (ECU Setup): the ECU uses cranking/warm-up enrichment and fast idle; carburettors use an automatic choke
  + fast-idle cam.
- **Fuel supply & electrics**: rail pressure from pump + regulator (manifold-referenced or returnless); an
  undersized pump = falling pressure = lean mixture; battery voltage (drops while cranking, alternator when
  running) affects injector dead time.
- **ECU calibration (tables)**: *Fuel Tuning* and *Ignition Tuning* hold the VE maps (speed density and
  alpha-N), target lambda, ignition, enrichment / correction curves, injector data, per-cylinder trims, and the
  load mode (SD / alpha-N / MAF / hybrid). Table editor: colour-coded grid with operating point + trail,
  keyboard (arrows, digits, +/−, Ctrl+C/V/Z/Y), interpolation, smoothing, lock, diff, and a 3D surface. The base
  VE map is measured automatically on a virtual dyno (in the background, cached in `basemaps/`); wideband
  auto-tune can refine it.
- **Calibration Manager**: save / load / duplicate / delete / import / export calibrations (JSON in
  `calibrations/<engine>/`), version history + restore, notes, compare with other versions, map locks, validation.
- **Pause / Resume / Reset** in the header (reset = a new engine at rest, cold or warm).
- **Rotary engines (Wankel, Mazda style)**: choose *Engine type → Rotary (Wankel)*. Set the number of rotors
  (1–4), generating radius R, eccentricity e, rotor width B, compression, and apex seal wear. The **Ports** tab
  (replacing Valvetrain) sets intake & exhaust port open/close (eccentric shaft degrees), port area, and porting
  presets: *stock side port*, *street port*, *bridge port* (brap-brap idle), *peripheral port* (racing).
  Presets: 13B 2-rotor, 20B 3-rotor, 26B 4-rotor peripheral port, 1-rotor carburetted.

- **Forced induction (Boost tab)**: *Turbocharger* — compressor size, turbine A/R (lag vs top end), rotor
  inertia, wastegate (target boost), blow-off valve (no BOV = "stu-tu-tu" surge flutter), anti-lag ALS (bangs
  off throttle). *Supercharger* — Roots, twin-screw (instant boost + characteristic whine), or centrifugal
  (boost rising with rpm), blower size, pulley ratio, bypass valve. Shared: intercooler + efficiency, charge pipe
  volume, ignition retard per bar, AFR under boost, injector size. A compressor map shows the real-time operating
  point. Presets: I4 2.0 turbo, V8 6.2 Roots blower, 13B rotary turbo, I6 big single turbo + anti-lag, V6
  centrifugal supercharger.

- **Vehicle & gearbox (Vehicle tab)**: a driving mode in which a car (or a motorcycle for small 1–2 cylinder
  engines) is the engine's load. Transmission: *manual* (you work the clutch — dropping it at idle stalls the
  engine, shifting without the clutch = a jolt), *manual with automatic clutch*, or *automatic* (shift points rise
  with throttle, kickdown). Set 1–8 gears and their ratios, final drive, clutch capacity, mass, Cd×A, rolling
  resistance, tyre radius & grip, weight on the driven wheels, road gradient. Includes a speedometer, gear
  indicator, clutch slip / wheelspin lamps, and automatic 0–100 km/h and 1/4 mile timers.

- **Scope**: besides cylinder pressure, P-V, per-cylinder and sound, there is a **Boost** tab (20-second graph of
  boost/MAP, wastegate target, turbo/blower speed, throttle, wastegate, AFR) and a **Fuel map** tab (rpm × MAP
  heatmap in the style of ECU tuning software: the VE table learned by the ECU, target AFR, plus measured AFR,
  injector duty and ignition timing filled in as the engine passes through each cell; with operating point
  and trail).

Changes to pipe length/diameter "glide" smoothly; topology changes (number of cylinders/rotors, engine type,
header type, muffler, etc.) rebuild the engine with a ~60 ms sound crossfade without stopping the engine.

## Physics model (summary)

- Each cylinder = a control volume (mass, energy, fuel & burned-gas fractions), slider-crank kinematics,
  compressible orifice valve flow, **Wiebe** combustion with a duration depending on rpm/λ/residuals + cycle-to-cycle
  variation, misfire, Woschni heat transfer, knock index.
- Intake & exhaust = a **digital waveguide network** (bidirectional delay lines, frequency-dependent losses,
  N-way scattering junctions, open ends that radiate sound). The exhaust sound = the derivative of volume flow
  at the tailpipe exit (monopole), so firing order, banks, header length and muffler change the sound directly.
- The gas in each pipe has its own temperature (primaries hottest, cooling towards the tailpipe) that follows
  EGT: speed of sound and impedance change with it, so resonances shift with load. Large-amplitude waves can be
  made to steepen (optional, travel time ~ 1 − k·p, shifted slowly so it does not buzz).
- Structure-borne sound: resonance modes of the oil pan, block and cylinder head (frequencies scale with bore)
  excited by combustion pressure rise, pops and valve seating. Cabin: 4th-order Butterworth low-pass (~800 Hz)
  and cabin-mode boom at 85 Hz / 176 Hz. These ideas are inspired by ICEsim (Agent00PED, MIT).
- Crank: inertia, FMEP friction, starter, idle air control + dashpot (EFI), DFCO, afterfire/pops.
- Dyno: PI absorber (hold rpm) or WOT sweep recording torque & power curves (comparable between runs).

### Rotary model

Each rotor has 3 chambers; one 4-stroke cycle per chamber = 1080° of eccentric shaft, with volume
`V = Vc + Vd/2·(1 − cos(2θ/3))` and `Vd = 3√3·R·e·B`. Each rotor fires once per shaft revolution (rotors phased
360/N° apart). Ports are opened/closed by the rotor flanks (no valves); the long, thin combustion chamber makes
combustion slower and heat loss larger; worn apex seals leak gas into the neighbouring chamber. Displacement is
quoted Mazda style (13B = 1.3 L), but the engine "breathes" like a 2.6 L four-stroke.

### Forced induction model

- Centrifugal compressor (turbo & centrifugal supercharger): a simple map per speed line (PR falling from the
  surge limit to choke), efficiency depending on the operating point, outlet temperature from adiabatic
  compression then the intercooler. Flow reverses during surge.
- Turbine: an orifice between the exhaust manifold node and the downpipe, power from the expansion of hot gas
  (EGT); the turbo shaft is integrated as (turbine power − compressor power) / inertia → natural turbo lag.
- Roots / twin-screw: a fixed volume pump per revolution minus leakage, driven by the crank (torque load).
- ECU: AFR enrichment and timing reduction with boost; knock becomes easier at high pressure.

### Vehicle model

The clutch = a friction element with a *locked* state (torsional spring + damping, so there is driveline shunt)
and a *slipping* state (torque = capacity × engagement). The gearbox multiplies wheel speed to the input shaft;
the driven wheels have their own inertia and the tyres use a Pacejka-like slip curve (peak at ~18 % slip,
~75 % grip when spinning), so wheelspin happens naturally. Car body: mass, rolling resistance, aerodynamic drag
½ρCdAv², gradient, brakes. Clutch torque is fed back to the crank (it can also drive the engine: engine braking,
push starts).

## License

MIT — see [LICENSE](LICENSE). Third-party libraries fetched at build time keep their own licenses
(Dear ImGui, ImPlot, ImPlot3D: MIT; miniaudio: public domain / MIT-0).

## Headless test tools

```bash
build/Release/sim_tests.exe          # sanity: masses, WOT torque (I4/V8/single), idle
```

```bash
build/Release/sim_tests.exe survey   # all presets: start, idle, blip, return to idle
```

```bash
build/Release/sim_tests.exe fuzz 120 # random live parameter changes (incl. piston<->rotary), checks for NaN
```

```bash
build/Release/sim_tests.exe 12       # WOT torque table for preset 12 (13B)
```

```bash
build/Release/ecu_test.exe          # sensors (lag, ADC, faults), ECU fallbacks, closed loop, thermal, fuel pump, ECU rate
```

```bash
build/Release/cal_test.exe          # calibration files: JSON round trip for all presets, validation, diff, version history
```

```bash
build/Release/parts_test.exe        # physical effect of each component (valve float, friction, coil, plugs, DBW, dead time, damage, oil)
```

```bash
build/Release/sim_tests.exe parts    # default part characteristics per preset (safe rpm, valve float, spark energy)
```

```bash
build/Release/drive_test.exe 0       # driving: automatic, manual + auto clutch, manual, clutch dump
```

```bash
build/Release/silence_probe.exe      # every preset must be silent with the engine off (checks for false hum)
```

```bash
build/Release/render_wav.exe 12 13b.wav   # render a preset's driving cycle to WAV
```

```bash
python tools/analyze_wav.py 13b.wav 2.6 3.4 900   # level + spectral peaks (engine orders)
```
