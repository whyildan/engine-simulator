# EngineSim — simulator mesin real-time (C++)

Simulator mesin pembakaran dalam dengan fisika per-silinder, suara yang **dihasilkan dari fisika**
(gelombang tekanan di knalpot & intake), dyno, dan semua konfigurasi diubah **langsung di dalam
aplikasi saat mesin hidup** (tanpa file config). Mendukung mesin **piston** dan **rotari Wankel**.

## Build (Windows, MSVC 2019 + CMake)

CMake tidak ada di PATH; pakai CMake bawaan Visual Studio:

```bash
"/c/Program Files (x86)/Microsoft Visual Studio/2019/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" -S . -B build -G "Visual Studio 16 2019" -A x64
```

```bash
"/c/Program Files (x86)/Microsoft Visual Studio/2019/BuildTools/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" --build build --config Release
```

Dependensi (Dear ImGui 1.90.9-docking, ImPlot 0.16, miniaudio 0.11.21) diunduh otomatis lewat
FetchContent saat configure pertama.

Jalankan: `build/Release/enginesim.exe` — atau langsung dengan preset tertentu:
`build/Release/enginesim.exe --preset 12` (rotari 13B).

## Kontrol

| Tombol | Fungsi |
|---|---|
| `S` (tahan) | starter / crank (otomatis menyalakan ignition) |
| `I` | ignition on/off |
| `W` / `↑` (tahan) | gas (naik perlahan, lepas = turun) |
| `Space` (tahan) | gas penuh (WOT) |
| `1`…`9`, `0` | throttle tetap 10 %…90 %, 100 % |
| `↓` | lepas gas |
| `D` | jalankan dyno sweep |
| `L` | ganti posisi pendengar (knalpot / ruang mesin / kabin) |
| `V` | mode berkendara on/off (mobil sebagai beban) |
| `E` / `Q` | naik / turun gigi (otomatis: `E` = D, `Q` = N) |
| `N` | netral |
| `C` (tahan) | kopling diinjak (dilepas pelan ~0.4 dtk) |
| `B` / `↓` (tahan) | rem |

Tombol **START ENGINE** men-crank sampai mesin hidup sendiri.

### Tata letak

Header di atas selalu terlihat: ignition, START ENGINE, crank, throttle, pilihan preset, dan angka utama
(RPM, MAP, AFR, timing, boost, EGT, torsi, gigi). Di kiri ada navigasi 12 bagian gaya software ECU:
Engine Setup, ECU Setup, Fuel Tuning, Ignition Tuning, Boost Control, Engine Control, Sensor Configuration,
Engine Protection, Live Dashboard, Data Logging, Virtual Dyno, Calibration Manager. Bagian yang diredupkan
belum dibuat dan hanya menjelaskan rencananya (lihat [docs/ECU_PLAN.md](docs/ECU_PLAN.md)).
`build/Release/enginesim.exe --section 11` membuka langsung bagian tertentu (1–12).

## Apa yang bisa dikonfigurasi (semua realtime)

- **Engine**: tipe mesin Piston / Rotary; layout Inline / V / Boxer, 1–16 silinder, sudut bank,
  bore, stroke, panjang conrod, kompresi, inersia flywheel, **firing order** (urut ulang dengan
  panah), firing genap atau **fase per silinder bebas** (odd-fire V-twin, big-bang), dan pembagian
  bank knalpot kiri/kanan.
- **Valvetrain**: jumlah & diameter katup, durasi cam, lift, centerline/LSA, cam advance (VVT),
  valve lash, plus grafik lift katup + overlap.
- **Ignition**: base / mekanis / vakum advance, rev limiter (ignition cut = letupan, fuel cut).
- **Fuel & Intake**: **EFI** (port / throttle-body, ukuran injector, target AFR, ECU tuned
  vs tidak, idle target) atau **karburator** (barrel, venturi, main jet, idle mixture, idle screw,
  pompa akselerasi, choke); throttle body, ITB, volume plenum, panjang/diameter runner, filter udara.
- **Exhaust**: log manifold / header N-into-1 / tri-Y 4-2-1, panjang primary (sama atau berbeda
  per silinder — "boxer rumble"), diameter, collector, crossover (true dual / H-pipe / X-pipe / Y),
  catalytic converter, resonator, muffler (lurus / glasspack / chambered / turbo), tailpipe.
- **Sound style** (tab Audio): *Smooth* (default, gaya engine-sim: sinyal utamanya tekanan pulsa gas buang,
  dibatasi frekuensinya oleh low-pass orde 4 *Smoothing*, diberi air noise halus, lalu dibentuk impulse response)
  atau *Raw* (model asli: hanya radiasi aliran tailpipe, tanpa lapisan tambahan, tanpa IR, silinder seragam).
  *Pulse pressure mix* mengatur porsi tekanan pulsa (bulat, penuh) vs radiasi aliran (lebih cerah).
- **Audio**: posisi pendengar, volume, oversampling fisika, *Rasp* (noise turbulensi jet gas di
  tailpipe, kuatnya ∝ kecepatan gas², frekuensinya mengikuti Strouhal 0.3·v/D), *Punch* (komponen
  tekanan pulsa frekuensi rendah), *Wave steepening* (opsional, default mati: pulsa bertekanan tinggi merambat lebih cepat sehingga menajam — menambah gigitan tapi juga kekasaran), dan *Environment* (jalan terbuka / garasi / ruang dyno / terowongan,
  reverb feedback-delay-network) + jumlah reverb. Ide air-noise dan campuran tekanan/turunan terinspirasi
  dari [engine-sim](https://github.com/ange-yaghi/engine-sim), tapi diturunkan dari aliran yang disimulasikan.
- **Impulse response (konvolusi)**: suara knalpot dilewatkan ke IR lewat konvolusi FFT terpartisi
  (latensi 256 sampel). Pilihan: *Auto* (sesuai muffler/motor), *Raw pipe*, *Sport*, *Stock muffler*,
  *Race*, *Motorcycle* (dibuat prosedural: pantulan awal rapat, ekor yang meredup, resonansi bodi
  knalpot), atau **Load WAV...** untuk IR sendiri (PCM 16/24/32-bit / float, otomatis di-resample).
  IR dari `ir_lib` engine-sim (lisensi MIT) bisa dipakai — cantumkan lisensinya.
- **Imperfection** (tab Engine): selisih kecil antar silinder (aliran port, injector, timing, pembakaran,
  panjang primary; rata-rata nol) + variasi antar siklus, supaya suara tidak steril seperti mesin
  sempurna.
- **Komponen (Engine Setup → Components)**: setiap part punya efek fisik.
  - *Piston / connecting rod / crankshaft*: bahan dan massa menentukan inersia tambahan dan **batas rpm aman**
    (gaya inersia vs kekuatan part terlemah); tension ring piston menambah friksi.
  - *Lifter + pegas katup*: massa yang bergerak vs gaya pegas menentukan rpm **valve float**. Di atasnya katup
    terlambat menutup dan memantul: torsi turun, misfire. Pegas lebih keras = friksi lebih besar.
  - *Throttle body* kabel atau drive-by-wire (waktu respons motor), *injector* (laju alir + dead time).
  - *Coil* (distributor / wasted spark / coil-on-plug, energi, waktu charge) dan *busi* (heat range, gap):
    energi percikan kurang = misfire. Busi terlalu panas = pre-ignition/knock, terlalu dingin + kaya = fouling.
  - *Pompa oli / air*: tekanan oli vs rpm (lampu Oil di header) dan daya penggerak (friksi).
  - Opsi **Allow mechanical failure**: part yang kelebihan beban, knock berat, atau katup menabrak piston
    menumpuk kerusakan sampai mesin rusak dan terkunci (tombol *Repair engine*).
  - *Size parts to this engine* memilih part yang aman untuk rev limiter saat ini. Kalau limiter dinaikkan
    tanpa mengganti part, mesin bisa valve float atau rusak.
- **ECU terpisah + sensor virtual**: ECU (tab *ECU Setup*) hanya membaca sensor (TPS, pedal, MAP, MAF, IAT, ECT,
  O2 narrowband, wideband, CKP, CMP, knock, EGT, tekanan & suhu oli, tekanan bahan bakar, kecepatan, gigi, aki).
  Laju loop ECU bisa diatur (100–5000 Hz). Tab *Sensor Configuration* menampilkan nilai sebenarnya vs yang dibaca
  ECU, tegangan sinyal, status, dan bisa mengatur noise, lag, serta kerusakan (putus, korslet, macet, noisy).
  ECU mendeteksi sinyal di luar rentang dan memakai fallback (MAP rusak → estimasi dari bukaan gas, CKP putus →
  mesin mati, CMP hilang → batch fire). Sensor yang *macet* tidak terdeteksi, persis seperti aslinya.
- **Model termal**: suhu coolant, oli, dan udara masuk; thermostat, radiator (kecepatan mobil + kipas), pompa air.
  Oli dingin = friksi lebih besar; dinding port dingin = film bensin lebih banyak dan sebagian bensin tidak
  menguap. Opsi **Cold start** (ECU Setup): ECU memakai pengayaan cranking/warm-up dan fast idle; karburator
  memakai choke otomatis + fast-idle cam.
- **Suplai bahan bakar & listrik**: tekanan rail dari pompa + regulator (berreferensi manifold atau returnless),
  pompa kekecilan = tekanan turun = campuran kurus; tegangan aki (turun saat cranking, alternator saat jalan)
  memengaruhi dead time injektor.
- **Kalibrasi ECU (tabel)**: *Fuel Tuning* dan *Ignition Tuning* berisi peta VE (speed density dan alpha-N),
  target lambda, pengapian, kurva enrichment / koreksi, data injektor, trim per silinder, dan mode beban
  (SD / alpha-N / MAF / hybrid). Editor tabel: grid berwarna dengan titik operasi + jejak, keyboard (panah,
  angka, +/−, Ctrl+C/V/Z/Y), interpolasi, smoothing, lock, diff, dan permukaan 3D. Base map VE diukur otomatis
  di dyno virtual (latar belakang, di-cache di `basemaps/`); auto-tune wideband bisa menyempurnakannya.
- **Calibration Manager**: simpan / muat / duplikat / hapus / import / export kalibrasi (JSON di
  `calibrations/<mesin>/`), riwayat versi + restore, catatan, compare dengan versi lain, lock peta, validasi.
- **Pause / Resume / Reset** di header (reset = mesin baru dalam keadaan diam, dingin atau panas).
- **Mesin rotari (Wankel, gaya Mazda)**: pilih *Engine type → Rotary (Wankel)*. Atur jumlah rotor
  (1–4), generating radius R, eksentrisitas e, lebar rotor B, kompresi, dan keausan apex seal.
  Tab **Ports** (pengganti Valvetrain) mengatur buka/tutup port intake & exhaust (derajat poros
  eksentrik), luas port, dan preset porting: *stock side port*, *street port*, *bridge port*
  (idle brap-brap), *peripheral port* (balap). Preset: 13B 2-rotor, 20B 3-rotor,
  26B 4-rotor peripheral port, 1-rotor karburator.

- **Forced induction (tab Boost)**: *Turbocharger* — ukuran kompresor, A/R turbin (lag vs top end),
  inersia rotor, wastegate (target boost), blow-off valve (tanpa BOV = flutter surge "stu-tu-tu"),
  anti-lag ALS (letupan saat lepas gas). *Supercharger* — Roots, twin-screw (boost instan + whine
  khas), atau sentrifugal (boost naik dengan rpm), ukuran blower, rasio pulley, bypass valve.
  Bersama: intercooler + efisiensi, volume pipa charge, ignition retard per bar, AFR saat boost,
  ukuran injector. Ada peta kompresor dengan titik operasi realtime. Preset: I4 2.0 turbo,
  V8 6.2 Roots blower, rotari 13B turbo, I6 big single turbo + anti-lag, V6 supercharger sentrifugal.

- **Kendaraan & gearbox (tab Vehicle)**: mode berkendara di mana mobil (atau motor untuk mesin
  1–2 silinder kecil) menjadi beban mesin. Transmisi *manual* (kopling diinjak sendiri — lepas
  mendadak saat idle = mesin mati, ganti gigi tanpa kopling = hentakan), *manual dengan kopling
  otomatis*, atau *otomatis* (titik pindah gigi naik dengan bukaan gas, kickdown). Atur 1–8 gigi dan
  rasionya, final drive, kapasitas kopling, massa, Cd×A, rolling resistance, radius & grip ban, bobot
  di roda penggerak, tanjakan jalan. Ada speedometer, indikator gigi, lampu clutch slip / wheelspin,
  dan pencatat 0–100 km/h serta 1/4 mil otomatis.

- **Scope**: selain tekanan silinder, P-V, per-silinder, dan suara, ada tab **Boost** (grafik 20 detik
  boost/MAP, target wastegate, putaran turbo/blower, bukaan gas, wastegate, AFR) dan tab **Fuel map**
  (heatmap rpm × MAP gaya software tuning ECU: tabel VE yang dipelajari ECU, target AFR, serta AFR,
  injector duty dan timing pengapian terukur yang terisi saat mesin melewati sel itu; ada titik operasi
  dan jejaknya).

Perubahan panjang/diameter pipa "meluncur" halus; perubahan topologi (jumlah silinder/rotor, tipe
mesin, tipe header, muffler, dsb.) membangun ulang mesin dengan crossfade suara ~60 ms tanpa
mematikan mesin.

## Model fisika (ringkas)

- Tiap silinder = volume kontrol (massa, energi, fraksi bahan bakar & gas buang), kinematika
  slider-crank, aliran katup orifice kompresibel, pembakaran **Wiebe** dengan durasi tergantung
  rpm/λ/residual + variasi siklus-ke-siklus, misfire, perpindahan panas Woschni, indeks knock.
- Intake & exhaust = **jaringan waveguide digital** (delay line dua arah, rugi frekuensi,
  junction hamburan N-arah, ujung terbuka yang meradiasikan suara). Suara knalpot = turunan aliran
  volume di ujung tailpipe (monopol), jadi firing order, bank, panjang header, dan muffler langsung
  mengubah suara.
- Gas di tiap pipa punya suhu sendiri (primary paling panas, makin dingin ke tailpipe) yang mengikuti
  EGT: kecepatan suara dan impedansi ikut berubah, jadi resonansi bergeser dengan beban. Gelombang
  amplitudo besar bisa dibuat menajam (opsional, waktu tempuh ~ 1 − k·p, digeser pelan supaya tidak
  berdengung).
- Bunyi struktur: mode resonansi bak oli, blok, dan kepala silinder (frekuensi ikut ukuran bore) digetarkan
  kenaikan tekanan pembakaran, letupan, dan dudukan katup. Kabin: low-pass Butterworth orde 4 (~800 Hz)
  dan dengung mode kabin di 85 Hz / 176 Hz. Ide-ide ini terinspirasi dari ICEsim (Agent00PED, MIT).
- Crank: inersia, friksi FMEP, starter, idle air control + dashpot (EFI), DFCO, afterfire/letupan.
- Dyno: absorber PI (hold rpm) atau sweep WOT yang merekam kurva torsi & tenaga (bisa dibandingkan
  antar run).

### Model rotari

Tiap rotor punya 3 ruang bakar; satu siklus 4 langkah per ruang = 1080° poros eksentrik, dengan
volume `V = Vc + Vd/2·(1 − cos(2θ/3))` dan `Vd = 3√3·R·e·B`. Tiap rotor menyala sekali per putaran
poros (rotor dipasang 360/N° terpisah). Port dibuka/ditutup oleh sisi rotor (bukan katup); ruang
bakar yang panjang dan tipis membuat pembakaran lebih lambat dan kehilangan panas lebih besar; apex
seal yang aus membocorkan gas ke ruang tetangga. Displacement ditulis gaya Mazda (13B = 1.3 L),
tapi mesinnya "bernapas" seperti 2.6 L empat-tak.

### Model forced induction

- Kompresor sentrifugal (turbo & supercharger sentrifugal): peta sederhana per garis kecepatan
  (PR turun dari batas surge ke choke), efisiensi tergantung titik operasi, suhu keluar dari kompresi
  adiabatik lalu intercooler. Aliran membalik saat surge.
- Turbin: orifice antara node manifold buang dan downpipe, daya dari ekspansi gas panas (EGT);
  poros turbo diintegrasikan (daya turbin − daya kompresor) / inersia → turbo lag alami.
- Roots / twin-screw: pompa volume tetap per putaran dikurangi kebocoran, digerakkan crank (beban torsi).
- ECU: pengayaan AFR dan pengurangan timing sesuai boost; knock makin mudah pada tekanan tinggi.

### Model kendaraan

Kopling = elemen gesek dengan status *lengket* (pegas torsi + redaman, jadi ada hentakan driveline)
dan *slip* (torsi = kapasitas × bukaan). Gearbox mengalikan putaran roda ke poros input; roda
penggerak punya inersia sendiri dan ban memakai kurva slip mirip Pacejka (puncak ~18 % slip,
~75 % grip saat spin), jadi wheelspin muncul alami. Bodi mobil: massa, rolling resistance, drag
aerodinamis ½ρCdAv², tanjakan, rem. Torsi kopling dikembalikan ke crank (bisa juga mendorong
mesin: engine braking, dorong-start).

## Alat uji headless

```bash
build/Release/sim_tests.exe          # sanity: massa, torsi WOT (I4/V8/single), idle
```

```bash
build/Release/sim_tests.exe survey   # semua preset: start, idle, blip, kembali idle
```

```bash
build/Release/sim_tests.exe fuzz 120 # ubah parameter acak live (termasuk piston<->rotari), cek NaN
```

```bash
build/Release/sim_tests.exe 12       # tabel torsi WOT untuk preset 12 (13B)
```

```bash
build/Release/ecu_test.exe          # sensor (lag, ADC, fault), fallback ECU, closed loop, termal, pompa bensin, laju ECU
```

```bash
build/Release/cal_test.exe          # file kalibrasi: JSON round trip semua preset, validasi, diff, riwayat versi
```

```bash
build/Release/parts_test.exe        # efek fisik tiap komponen (valve float, friksi, coil, busi, DBW, dead time, kerusakan, oli)
```

```bash
build/Release/sim_tests.exe parts    # karakteristik part default per preset (rpm aman, valve float, energi percikan)
```

```bash
build/Release/drive_test.exe 0       # berkendara: otomatis, manual+auto clutch, manual, clutch dump
```

```bash
build/Release/silence_probe.exe      # semua preset harus senyap saat mesin mati (cek dengung palsu)
```

```bash
build/Release/render_wav.exe 12 13b.wav   # render siklus berkendara preset ke WAV
```

```bash
python tools/analyze_wav.py 13b.wav 2.6 3.4 900   # level + puncak spektrum (engine order)
```
