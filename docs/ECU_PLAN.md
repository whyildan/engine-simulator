# EngineSim → simulasi mesin + platform tuning ECU: analisis dan rencana

Dokumen ini menjawab `tuning-update.md` bagian 19, langkah 1–6: memeriksa kode yang sudah ada, memetakan
fitur yang sudah ada dan yang belum, lalu membagi pekerjaan per fase. Pekerjaan dilakukan satu fase per
satu, dengan tes di akhir tiap fase.

Status: **rencana sudah direvisi dan keputusan sudah disepakati (lihat bagian 5); Fase 1 (1a, 1b, 1c) selesai; berikutnya Fase 2.**

---

## 1. Kondisi kode sekarang (ringkas)

| Area | Yang sudah ada | Lokasi |
|---|---|---|
| Fisika mesin | Termodinamika 0D per silinder, aliran katup kompresibel, pembakaran Wiebe, Woschni, indeks knock, friksi FMEP, inersia crank | `src/sim/Cylinder.*`, `src/sim/Engine.*` |
| Konfigurasi | Inline/V/boxer 1–16 silinder (I3/I4/I5/I6/V6/V8/V10/V12/flat-4/flat-6 bisa dibentuk), rotari Wankel, bore/stroke/rod/CR, firing order, fase bebas, cam/valve, VVT | `EngineParams.h`, `Presets.cpp` |
| Intake / bahan bakar | Throttle, plenum, ITB, runner waveguide, EFI port/TB, karburator lengkap, wall film X-tau, DFCO | `src/sim/Intake.*` |
| "ECU" | **Tertanam di fisika**: target AFR hanya 3 skalar (cruise/WOT/boost), tabel VE yang dipelajari otomatis (21×32, belum bisa diedit), satu trim closed-loop, IAC PI + dashpot, pengayaan cranking/after-start, timing pengapian gaya distributor (base + mekanis + vakum − retard boost), rev limiter keras (pemotongan pengapian atau bahan bakar) | `Intake.cpp`, `Engine.cpp` |
| Boost | Turbo (peta kompresor, poros, wastegate pneumatik PI), supercharger Roots/screw/sentrifugal, BOV, anti-lag | `src/sim/Boost.*` |
| Drivetrain | Kopling, gearbox 1–8 gigi (manual, kopling otomatis, transmisi otomatis), ban slip, aero | `src/sim/Vehicle.*` |
| Dyno | Free / hold rpm / sweep, overlay run, jejak live + peak hold | `src/sim/Dyno.*`, `Panels.cpp` |
| Monitoring | Gauge rpm/MAP/AFR/boost, grafik scope 20 dtk, heatmap fuel map (hanya baca) | `Widgets.cpp`, `Panels.cpp` |
| Arsitektur | UI → `EngineParams` (POD, mutex + versi) → thread simulasi (96 kHz) → ring audio, telemetri | `Simulator.*` |
| Tes | `sim_tests` (sanity / survey / fuzz), `drive_test`, `silence_probe`, `ir_test`, `boost_trace`, `freerev_check` | `tests/` |

**Celah arsitektur utama:**
- **ECU membaca state fisika langsung.** MAP, rpm, dan lambda diambil sebagai nilai sebenarnya, dan logika ECU tercampur dengan model intake dan pengapian. Spesifikasi (bagian 7 dan 14) meminta ECU membaca sensor virtual dan berjalan sebagai modul terpisah.
- **Belum ada model termal.** Suhu coolant, oli, dan IAT dinamis belum ada; yang ada hanya suhu ambient dan EGT. Pengayaan warm-up, koreksi ECT/IAT, dan proteksi suhu butuh model ini.
- **Belum ada kalibrasi yang bisa disimpan atau dimuat.**
- **Belum ada data logging.**

---

## 2. Pemetaan spesifikasi → status

Keterangan: ✅ sudah ada · 🟡 sebagian (perlu dirombak atau diperluas) · ❌ belum ada

| § | Fitur | Status | Catatan |
|---|---|---|---|
| 1 | Arsitektur I3–V12, flat-4/6, rotari | ✅ | Preset I5 dan flat-6 belum ada (tinggal ditambah) |
| 1 | Bore/stroke/rod/CR/firing/limiter/inersia | ✅ | |
| 1 | Redline terpisah, friksi yang bisa diatur | 🟡 | Friksi masih rumus tetap |
| 1 | Komponen (piston, rod, crank, cam, pegas katup, manifold, throttle body, injector, coil, busi, pompa oli/air) | ❌ | Katalog lengkap, dan tiap komponen punya efek fisik (lihat Fase 1b) |
| 2 | Mode injeksi sequential/batch/semi-seq | 🟡 | Saat ini hanya sequential |
| 2 | Trigger crank/cam, CAN, I/O | ❌ | Trigger bisa disimulasikan (sinkronisasi saat start); CAN dan I/O ditandai "not supported" |
| 2 | Speed density / Alpha-N / MAF / hybrid | 🟡 | Hanya mirip speed density; sensor MAF belum ada |
| 3 | Tabel VE, target AFR/λ, injector + dead time, tekanan bahan bakar | 🟡 | Tabel VE ada tapi tidak bisa diedit; dead time dan tekanan bahan bakar belum ada |
| 3 | Pengayaan cranking/warm-up/akselerasi, DFCO, after-start, trim | 🟡 | Ada, tapi hardcoded; warm-up butuh model termal |
| 3 | Editor tabel (sel, interpolasi, smoothing, copy/paste, undo/redo, sel aktif, compare, per silinder) | ❌ | Antarmuka utama yang paling penting |
| 3 | Permukaan 3D | ❌ | Butuh pustaka ImPlot3D (dependensi baru via FetchContent) |
| 4 | Tabel pengapian, cranking, dwell, koreksi per silinder, koreksi suhu/beban | 🟡 | Kurva distributor ada; tabel belum ada |
| 4 | Knock detection + retard | 🟡 | Indeks knock fisik ada; kontrolnya belum ada |
| 5 | Turbo, wastegate, inersia, efisiensi | ✅ | |
| 5 | Tabel boost (rpm/gigi/TPS), duty wastegate, PID, overboost, boost cut | 🟡 | Saat ini wastegate pneumatik dengan satu target |
| 6 | Idle, rev limiter, anti-lag, VVT | 🟡 | Ada, tapi belum berupa tabel |
| 6 | Soft/hard limiter, launch, 2-step/3-step, flat shift, DBW, VTEC, torque mgmt | ❌ | |
| 7 | Sensor virtual (tegangan, noise, delay, gagal, fallback) | ❌ | Fondasi fase 1 |
| 8 | Lambda STFT/LTFT, batas, kondisi learning, Auto Tune | 🟡 | Satu trim dan VE learning otomatis tersembunyi |
| 8 | Kontrol knock per silinder, PID boost | ❌ | |
| 9 | Proteksi dan limp mode | ❌ | Butuh model termal dan tekanan oli |
| 10 | Fitur balap (launch, TC, nitrous, transbrake, …) | ❌ | Kendaraan dan slip ban sudah ada, jadi bisa dibangun di atasnya |
| 11 | Dashboard live yang bisa diatur | 🟡 | Gauge ada; tata letak belum bisa diatur, sebagian channel belum ada |
| 12 | Data logging, grafik, CSV, perbandingan sesi | ❌ | |
| 13 | Dyno steady/sweep, perbandingan, ekspor | 🟡 | Ekspor dan satuan imperial belum ada |
| 14 | Pemisahan fisika / sensor / ECU, frekuensi bisa diatur | ❌ | Inti fase 1 |
| 15 | Manajemen kalibrasi (simpan, muat, versi, compare, lock, validasi) | ❌ | |
| 16 | UI gelap padat, navigasi 12 bagian | 🟡 | Saat ini dock panel ImGui; perlu dirombak ke navigasi baru |
| 17 | Pause/resume/reset, sesi deterministik | 🟡 | Simulasi deterministik (seed tetap); pause/reset belum ada |

---

## 3. Arsitektur target

```
EngineParams (konfigurasi mesin, POD)     Calibration (tabel ECU, POD, disimpan/dimuat)
        │                                          │
        ▼                                          ▼
 Fisika (Engine, Intake, Exhaust, Boost,    ┌── EcuController (src/ecu/) ──────────────┐
 Vehicle, Thermal*)  ── state sebenarnya ─► │ SensorLayer → ECU loop (laju diatur)     │
        ▲                                   │  fuel · ignition · idle/DBW · boost ·    │
        │        ActuatorCommands           │  limiter · knock · lambda · protection   │
        └───────────────────────────────────┴──────────────────────────────────────────┘
```

- **`src/ecu/Sensors.*`**
  - Mengubah state sebenarnya menjadi bacaan sensor: kalibrasi tegangan, noise, lag orde 1, resolusi, mode gagal (putus/terhubung singkat/macet).
  - Mendeteksi nilai di luar rentang dan menyediakan fallback.
  - ECU **hanya** melihat struct `SensorReadings`.
- **`src/ecu/Tables.*`**
  - Tabel 1D/2D bertipe dengan sumbu yang bisa diedit dan interpolasi bilinear.
  - Validasi rentang dan flag lock.
- **`src/ecu/Calibration.*`**
  - Semua tabel dan konfigurasi ECU dalam satu POD, dipublikasikan ke thread simulasi dengan cara yang sama seperti `EngineParams`.
  - Bisa disimpan dan dimuat sebagai JSON. Tiap konfigurasi mesin punya profilnya sendiri.
- **`src/ecu/Ecu.*`**
  - Event per silinder (injeksi dan spark tetap tepat sudut) ditambah loop tetap, default 1 kHz, bisa diatur, untuk idle, boost PID, lambda, knock, dan proteksi.
  - Keluarannya `ActuatorCommands`: pulse width per silinder, sudut spark per silinder, posisi DBW/IAC, duty wastegate, dan flag cut.
- **Fisika hanya menerima aktuator.**
  - `IntakeSystem::injectEvent` menerima pulse width; massa bahan bakar = (PW − dead time) × laju alir dari tekanan bahan bakar.
  - `Engine` menerima sudut spark.
  - `Boost` menerima duty solenoid wastegate (pegas + duty).
- **`src/sim/Thermal.*` (baru)**
  - Massa termal blok, coolant, dan oli.
  - Panas masuk dari pembakaran dan friksi; panas keluar lewat radiator (tergantung kecepatan kendaraan dan kipas) dan termostat.
  - Tekanan oli dihitung dari rpm, viskositas (suhu oli), dan pompa.
  - IAT berubah dengan heat soak dan intercooler.
  - Dampaknya ke fisika: friksi berubah dengan suhu oli, penguapan bahan bakar (wall film) berubah dengan suhu, dan kecenderungan knock berubah dengan suhu dinding.

**Prinsip yang dipegang:**
- Modul yang sudah jalan (waveguide, silinder, boost, kendaraan, audio) tidak ditulis ulang.
- Logika ECU yang sudah ada dipindahkan ke `src/ecu/`. Fasenya diterima kalau perilakunya sama atau lebih baik (survey/fuzz/drive tidak mundur).

---

## 4. Fase

### Fase 1 – Fondasi

Fase 1 cukup besar, jadi dibagi menjadi 1a, 1b, dan 1c. Tiap bagian ditutup dengan tes dan build sendiri.

#### 1a – Navigasi UI baru ✅ selesai
- Ganti tab lama dengan navigasi 12 bagian:
  1. Engine Setup
  2. ECU Setup
  3. Fuel Tuning
  4. Ignition Tuning
  5. Boost Control
  6. Engine Control
  7. Sensor Configuration
  8. Engine Protection
  9. Live Dashboard
  10. Data Logging
  11. Virtual Dyno
  12. Calibration Manager
- Bentuknya sidebar kiri ditambah area kerja, dengan tema gelap yang padat.
- Panel yang sudah ada dipindahkan ke bagian yang sesuai:
  - Engine Setup: mesin, valvetrain/port, exhaust, intake fisik, kendaraan, audio;
  - Fuel Tuning: fuel;
  - Ignition Tuning: ignition;
  - Boost Control: boost;
  - Live Dashboard: dashboard dan scope;
  - Virtual Dyno: dyno.
- Bagian yang belum punya isi menampilkan status "planned (Phase N)". Tidak ada kontrol palsu.
- Kontrol keyboard, START ENGINE, dan gauge utama tetap selalu terlihat di header.

#### 1b – Katalog komponen (Engine Setup → Components) ✅ selesai
Tiap komponen dipilih dari katalog (plus opsi "custom") dan punya efek fisik yang nyata:

| Komponen | Parameter | Efek di simulasi |
|---|---|---|
| Piston | massa, tipe (cast / hypereutectic / forged), ring | massa bolak-balik → batas rpm aman, gaya inersia, friksi ring; forged lebih tahan knock |
| Connecting rod | massa, material, panjang | massa bolak-balik/berputar, rasio rod/stroke (sudah ada), batas rpm |
| Crankshaft | inersia, tipe (cast / forged / billet), crossplane / flatplane | inersia crank (flywheel), firing / throw (sudah ada), batas rpm |
| Camshaft | profil (durasi, lift, LSA, sudah ada), material lobe | lift vs pegas → valve float |
| Pegas katup | tekanan seat / open, massa retainer | **valve float** di atas rpm kritis: katup tidak menutup penuh → kebocoran, torsi turun, misfire |
| Intake manifold | plenum, runner (sudah ada), tipe (single / dual / ITB) | gelombang intake (sudah ada) |
| Exhaust manifold | header (sudah ada) | waveguide (sudah ada) |
| Throttle body | diameter (sudah ada), tipe (kabel / DBW) | aliran udara; DBW → kontrol ECU |
| Injector | laju alir, dead time vs tegangan, tekanan referensi | massa bahan bakar = (PW − dead time) × laju(ΔP) |
| Coil | waktu dwell minimum, energi | energi percikan rendah di rpm tinggi atau boost → misfire |
| Busi | heat range, gap | heat range terlalu panas → pre-ignition / knock lebih mudah; gap besar + boost → misfire |
| Pompa oli | kapasitas, relief | tekanan oli vs rpm dan suhu oli → proteksi |
| Pompa air | kapasitas | laju pendinginan → model termal |

- Efek yang butuh model termal atau tekanan oli baru aktif setelah 1c.
- Massa dan inersia langsung masuk ke dinamika crank dan friksi.

#### 1c – ECU terpisah, sensor, termal ✅ selesai
1. Buat `src/ecu/` (Sensors, Ecu, ActuatorCommands). Pindahkan logika ECU dari `Intake.cpp` dan `Engine.cpp` **tanpa mengubah perilaku**:
   - fuel: target AFR, VE, X-tau, trim, cranking/after-start, DFCO;
   - spark;
   - IAC/dashpot;
   - limiter.
2. Lapisan sensor virtual: TPS, MAP, MAF, IAT, ECT, O2 narrowband, wideband, CKP/CMP (rpm dan sinkronisasi), knock, EGT, tekanan dan suhu oli, tekanan bahan bakar, kecepatan kendaraan, posisi gigi, tegangan baterai. Tiap sensor punya noise, delay, dan kegagalan yang bisa disimulasikan.
3. Model termal (ECT, oli, IAT, tekanan oli) beserta efeknya ke fisika.
4. Injector dead time (vs tegangan baterai) dan tekanan bahan bakar (regulator, referensi MAP).
5. Preset I5 dan flat-6.
6. Pause, resume, reset simulasi; frekuensi loop ECU bisa diatur.
7. Tes:
   - unit test untuk tabel/interpolasi, sensor (kalibrasi, fallback), dan termal (energi);
   - regresi survey/fuzz/drive/silence dengan hasil yang setara.

**Catatan hasil Fase 1c:**
- ECU (`src/ecu/Ecu.*`) hanya membaca `SensorReadings`. Isinya: idle, spark, limiter, anti-lag, DFCO, film X-tau
  (model ECU sendiri), closed loop wideband (cadangan narrowband), warm-up / cranking / after-start enrichment,
  kompensasi dead time (tegangan aki) dan aliran injektor (tekanan bahan bakar). Fallback: MAP → alpha-N,
  CKP hilang → mesin mati, CMP hilang → batch fire, sensor lain → nilai pengganti.
- Masih tersisa dari model lama: peta VE "tuned ECU" dikalibrasi otomatis dari udara yang terperangkap
  (`Ecu::calibrationFeedback`). Ini sama dengan peta VE yang sudah pas dengan mesin, dan akan diganti tabel
  VE yang bisa diedit + auto-tune wideband di Fase 2/3.
- Wastegate turbo masih pneumatik (pegas + aktuator), belum dikendalikan ECU (boost control di Fase 3).
- Preset I5 2.5 dan flat-6 3.6 ditambahkan (indeks 21, 22).
- Diketahui: preset rotari 4-rotor balap (26B) tidak bisa hidup pada start dingin −10 °C.

### Fase 2 – Kalibrasi ECU ✅ selesai
- **Tabel default dibuat otomatis dari preset**, jadi preset lama langsung bisa jalan:
  - VE (rpm × MAP);
  - target λ/AFR (rpm × MAP);
  - pengapian utama (rpm × MAP);
  - pengapian cranking;
  - pengayaan cranking, warm-up (ECT), after-start, dan akselerasi (ΔTPS / ΔMAP vs ECT);
  - koreksi IAT dan ECT untuk fuel dan spark;
  - trim fuel dan spark per silinder;
  - dead time injector vs tegangan.
- **Mode beban:** speed density, alpha-N, MAF, dan hybrid (blend menurut TPS).
- **Editor tabel**, antarmuka utama:
  - grid seperti spreadsheet dengan warna;
  - sel aktif dan jejak titik operasi;
  - navigasi keyboard dan edit multi-sel (set, +/−, ×%);
  - interpolasi baris/kolom, smoothing, copy/paste, undo/redo;
  - compare dengan versi lain (selisih berwarna);
  - lock tabel;
  - tampilan permukaan 3D (ImPlot3D).
- **Manajemen kalibrasi:** baru, simpan, muat, duplikat, import/export (JSON), riwayat versi dan restore, catatan, validasi.
- **UI:** editor tabel mengisi bagian Fuel / Ignition Tuning dan Calibration Manager di navigasi dari Fase 1a.

**Catatan hasil Fase 2:**
- `src/ecu/Calibration.*`: tabel 2D (maks 16×16) dan kurva 1D (maks 10 titik), interpolasi bilinear, generator
  kalibrasi dari parameter preset. ECU (`src/ecu/Ecu.*`) sekarang sepenuhnya digerakkan tabel: VE SD / VE alpha-N,
  target λ, pengapian, power enrichment (TPS), cranking AFR, warm-up, after-start, akselerasi (laju TPS, dikali
  saat dingin), pengayaan keluar DFCO, injeksi asinkron saat tip-in, koreksi IAT/ECT, dead time, trim per silinder.
- Mode beban SD / alpha-N / MAF / hybrid. Preset ITB otomatis alpha-N.
- **Base map terukur**: dyno motoring virtual (`src/ecu/BaseMap.*`) mengukur udara yang terperangkap pada 8 rpm × 8
  bukaan gas dan menulis peta VE. Berjalan di thread latar; hasilnya di-cache di `basemaps/` di samping exe
  (kunci = hash definisi mesin), jadi preset yang sama langsung pakai peta terukur pada pemakaian berikutnya.
- Closed loop trim dan auto-tune wideband (dengan buffer transport delay). Trim dimatikan saat auto-tune aktif.
- Editor tabel (`src/ui/TableEditor.*`): grid berwarna, titik operasi + jejak, keyboard, set / tambah / ×%,
  interpolasi H/V, smoothing, copy/paste, undo/redo, edit sumbu, lock, diff terhadap referensi, permukaan 3D.
- **Calibration Manager** (`src/ui/CalManager.cpp`, `src/ecu/CalibrationIO.*`): file JSON di
  `calibrations/<mesin>/<nama>.json`, tiap simpan menambah revisi di `<nama>.history/rNNNN.json`; muat, duplikat,
  hapus, import/export, catatan, restore revisi, compare (tabel per tabel + daftar setelan yang beda; juga
  dipakai tombol Diff di editor), lock semua peta, validasi (sumbu, rentang, sel loncat, cakupan rpm, injektor,
  trim). Import toleran: field yang tidak ada mempertahankan nilai sekarang. Tes: `cal_test`.
- Perbaikan setelah Fase 2:
  - jitter waktu mulai bakar kadang "membungkus" ke 720° sehingga bahan bakar terbakar seketika saat spark
    (torsi per siklus naik-turun ±24 Nm, tenaga hilang ~10%); sekarang sudut bakar dihitung bertanda;
  - base map: VE tiap titik diiterasi (ECU memakai nilai hasil ukur lewat `Ecu::veOverride`) sampai campuran
    kena target, dan tiap titik ditunggu sampai MAP, udara terperangkap, dan λ stabil. Titik gas tertutup yang
    tidak menyala tidak dipakai. Sebelumnya VE di rpm rendah gas penuh terukur ~20% terlalu rendah. Mode MAF
    tidak dipakai karena pada mesin overlap besar banyak udara terukur yang lolos langsung ke knalpot;
  - DFCO hanya sekali per deselerasi (aktif lagi setelah gas dibuka) dan pengayaan keluar-DFCO mengikuti
    "kekeringan" dinding port yang kontinu (meluruh 1 s): setelah potong bahan bakar knalpot berisi udara yang
    tersedot balik di beban rendah;
  - pengayaan dari laju kenaikan MAP saat gas tertutup (katup idle membuka menangkap rpm yang jatuh);
  - di mode berkendara, torsi di grafik dyno = torsi kopling dirata-rata per siklus.
- Diketahui: start dingin (20 °C / −10 °C) pada beberapa preset boost dan 26B masih bisa mati saat kembali dari
  blip; bisa dituning lewat tabel enrichment (akselerasi dingin, keluar DFCO, warm-up).

### Fase 3 – Kontrol lanjutan ✅ selesai
- Lambda closed-loop: STFT dan LTFT (sel LTFT per wilayah beban), batas koreksi, kondisi learning (ECT, delay setelah transien, rentang beban), dan **Auto Tune VE** dari wideband.
- Kontrol knock: threshold dan window sensor knock, retard per silinder, laju pemulihan.
- Idle: target rpm vs ECT, PID idle, idle ignition control. DBW: peta pedal ke throttle.
- Boost: target per rpm/gigi/TPS, PID ke duty solenoid, overboost cut, boost cut.
- Proteksi dan limp mode: semua kondisi di §9, dengan threshold, delay, pemulihan, dan tingkat keparahan.
- Soft limiter (pemotongan sebagian) dan hard limiter.

**Catatan hasil Fase 3:**
- **Fuel trim** (`Ecu::fuelTrimStep`): STFT mengintegrasikan error wideband (gain, batas ±%), hanya saat syarat
  terpenuhi (ECT minimum, jeda setelah start/transien, MAP maksimum, target lebih kurus dari AFR 13.5), dan meluruh
  ke 0 di loop terbuka. LTFT = tabel 8 × 8 (rpm × MAP) yang mengambil alih koreksi STFT per sel, berlaku juga di
  loop terbuka, bisa dikunci/dihapus, ikut tersimpan di file kalibrasi. Auto-tune VE tetap opsi tersendiri.
- **Knock control** (`Ecu::knockStep`): window sudut engkol membagi sinyal knock per silinder (per rotor untuk
  rotari) per siklus pembakaran. Silinder di atas threshold di-retard satu langkah, semua pulih dengan laju
  pemulihan. Sensor knock rusak → retard pengaman setengah dari maksimum.
- **Idle / DBW**: target idle = kurva vs ECT, gain PID katup idle dan kontrol timing idle bisa diatur (default =
  kontroler lama). Drive-by-wire: peta pedal → throttle; sensor pedal rusak → throttle idle.
- **Soft limiter**: pada pita sebelum hard limit, timing di-retard dan sebagian percikan dipotong (acak).
- **Boost control** (`Ecu::boostStep`): target bar dari rpm × TPS × kurva per gigi; feed-forward dari tekanan
  pegas wastegate + PID pada sensor MAP → duty solenoid. Fisik: solenoid membuang tekanan aktuator, sehingga
  aktuator hanya melihat boost × (1 − 0.9·duty). Overboost cut (melewati target + margin terlalu lama) dan boost
  cut (batas absolut) berupa fuel cut.
- **Proteksi** (`Ecu::protectionStep`): 10 aturan (over-rev, overboost, coolant, oli, tekanan oli diskalakan
  dengan rpm, tekanan bahan bakar, kurus saat beban, knock berlebih, EGT, kegagalan sensor), masing-masing
  dengan warn, limit, delay, recovery, dan aksi: peringatan, retard, kurangi boost, batas throttle (DBW; untuk
  throttle kabel menjadi batas rpm), batas bahan bakar (batas rpm), limp mode, shutdown (terkunci sampai mesin
  berhenti). Lampu CEL dan teks LIMP/SHUTDOWN di header.
- UI: Fuel Tuning › Closed loop, Ignition Tuning › Knock control, Engine Control (Idle, Throttle, Rev limiter),
  Boost Control › ECU boost control, Engine Protection. Tes: `control_test`.
- Ikut diperbaiki: pengayaan laju-MAP kini memakai laju MAP yang difilter lambat (0.15 s), dan laju cepat hanya
  saat rpm sedang jatuh. Sebelumnya denyut MAP saat idle memicunya terus sehingga idle selalu sedikit kaya dan
  closed loop terganggu. Transfer STFT → LTFT kini mempertahankan koreksi penuh di titik operasi. Preset 26B
  memakai AFR cruise 12.5 (peripheral port dengan residu besar hanya idle bersih saat kaya).
- Rig base map kini tanpa pengayaan transien (akselerasi, keluar-DFCO, asinkron, laju-MAP) — derau sensor
  memicunya di titik ukur yang stabil dan membiaskan VE. Base map diukur ulang otomatis saat jalur udara mesin
  berubah (mis. porting rotari), selama peta VE belum diedit.
- Rotari: preset baru 13B bridge port (idle "brap brap"). Brap muncul dari fisika (overlap besar → gas buang
  balik → muatan encer misfire → muatan segar berikutnya meledak keras → letupan di knalpot); preset rotari
  memakai mix suara lebih serak (pulse pressure 0.1, punch 0.15) agar denyut tak rata itu tidak tertutup
  dentum rendah. 13B stock idle 1000 rpm (950 bisa mati setelah idle lama). Tes `idle_hold`.

### Fase 4 – Monitoring ✅ selesai
- Dashboard yang bisa diatur: gauge analog/digital per channel, tata letak tersimpan, alarm dan indikator status sensor.
- Data logging:
  - pilih channel dan laju, start/stop, marker event;
  - grafik terhadap waktu dan rpm, zoom/pan;
  - ekspor CSV;
  - bandingkan sesi, ditautkan ke versi kalibrasi.
- Dyno: ekspor CSV, satuan imperial (HP = lb-ft × rpm / 5252), perbandingan sebelum dan sesudah tuning.

Implementasi:
- **Channel log** (`src/sim/DataLog.*`): 43 channel (48 sejak Fase 5) (key, nama, satuan, grup, getter dari telemetri). Sampel
  diambil di thread simulator per blok render menurut waktu simulasi (10/20/50/100 Hz), jadi laju log tidak
  bergantung pada frame rate UI. `LogSession` menyimpan mesin, nama + revisi kalibrasi, laju, dan marker.
- **CSV**: baris header `# key: value` (sesi, mesin, kalibrasi, revisi, laju), baris `# marker: t, label`, lalu
  kolom `time` + channel yang dipilih. Impor CSV menolak file yang bukan log EngineSim. Tes: `log_test`.
- **UI Data Logging** (`src/ui/Logging.cpp`): Record `R`, marker `M`, marker otomatis saat aksi proteksi, limp
  mode, dan ganti gigi; tabel sesi; overlay sesi pembanding; 1–6 panel grafik (ImPlot subplot, sumbu X
  tertaut) terhadap waktu atau rpm; saat merekam grafik mengikuti 30 s terakhir, tombol Fit menampilkan sesi
  penuh.
- **Dashboard Custom** (`src/ui/Dashboard.cpp`): grid widget (gauge analog, digital, bar) untuk channel log
  mana pun, ambang warning/alarm bawah dan atas (warna dan kedip), daftar alarm aktif, strip status sensor
  (merah = ECU memakai nilai fallback). Tata letak disimpan ke `dashboard.cfg` di samping exe.
- **Dyno**: Nm/kW atau lb-ft/hp, ekspor CSV per run, run baseline dengan grafik delta dan delta puncak;
  label run memuat mesin dan revisi kalibrasi.
- **EGT** (diperbaiki sebelum fase ini): sensor EGT kini membaca gas setelah port buang yang melepas panas ke
  dinding port (Dittus–Boelter, faktor aliran berdenyut 1.8, panjang port 4·D, dinding = coolant + 120 K).
  Idle turun dari 800–959 °C ke ±560–740 °C. Sisa selisih ke mesin asli (±300–550 °C) berasal dari pembakaran
  idle yang lambat (CA50 ±30° ATDC); mempercepatnya membuat idle beberapa preset tidak stabil, jadi ditunda.

### Fase 5 – Motorsport ✅ selesai
- Launch control dan 2-step/3-step (pemotongan pengapian/bahan bakar + retard), flat shift (cut saat ganti gigi tanpa angkat gas).
- Traction control dari slip roda yang sudah ada di `Vehicle`.
- Rolling anti-lag (memakai ALS yang sudah ada).
- Boost by gear dan transbrake.
- Nitrous: injeksi N₂O dengan tambahan oksigen dan efek pendinginan muatan, plus pengayaan bahan bakar.

Implementasi:
- **Input digital** (`DigitalInputs` di `src/ecu/Sensors.h`): saklar kopling (pedal > 50 %), transbrake, saklar
  arming nitrous, tombol rolling anti-lag, sinyal ganti gigi, dan kecepatan roda penggerak. ECU membacanya
  apa adanya (tanpa model lag/noise).
- **`Ecu::motorsportStep`**: satu limiter aktif sekaligus, urutan 2-step → 3-step → flat shift → shift cut →
  rolling ALS. Limiter memotong spark (atau bahan bakar) dengan porsi yang naik dari 0 di limit − band ke 100 %
  di limit + band. 2-step/3-step/rolling ALS memakai timing absolut (launch timing / ALS timing); flat shift
  menahan rpm saat kopling diinjak (+ margin) dengan retard. Bahan bakar dari silinder yang dipotong terbakar
  di manifold (letupan), dan panasnya kini ikut menaikkan EGT dan memutar turbin — itulah yang membangun
  boost di 2-step. Base map diukur ulang (versi 16).
- **Traction control**: slip = (roda penggerak − kecepatan mobil) / max(kecepatan, 10 km/h); target slip berupa
  kurva atas kecepatan; PI dengan anti-windup; 40 % pertama pengurangan berupa retard, sisanya spark cut
  (maks 90 %).
- **Transbrake** (gearbox otomatis / auto-clutch): roda dikunci, mesin dibebani seperti torque converter saat
  stall (beban ∝ rpm², stall ±45 % rev limit); tombol ikut meng-arm 2-step. Lepas tombol → kopling menggigit.
- **Anti-lag**: parameter yang dulu tetap (−30°, throttle 10 %, 2 s, > 2500 rpm) kini kalibrasi.
- **Nitrous**: N₂O masuk ke port sebagai muatan setara udara ×2.2 (oksigen 1.57× udara + panas dekomposisi),
  0.32 g/s per hp shot, pendinginan muatan, kit wet (bahan bakar sendiri, ±AFR 12.5 untuk porsi N₂O) atau dry
  (ECU menambah bahan bakar menurut nilai aliran N₂O di kalibrasinya). Kontroler: arming, TPS, jendela rpm,
  ECT, kecepatan, delay, ramp progresif, retard, mati saat lean. Botol berkurang; aliran turun di 15 % terakhir.
- **UI**: Engine Control › Launch / shift dan Traction control; Boost Control › Anti-lag dan Nitrous; tombol
  `T` (transbrake), `G` (rolling ALS), `X` (arming nitrous); status aktif di kanan baris judul section; 5
  channel log baru dan marker otomatis "launch" / "nitrous on".
- **Hasil `motorsport_test`**: 2-step I4 tertahan ±4100 rpm dengan letupan; turbo besar di 2-step dengan
  timing −15° vs +15°: turbo 69k vs 59k rpm, EGT 666 vs 547 °C; flat shift: maks 5332 rpm (tanpa: 6983);
  shift cut memangkas lonjakan rpm saat ganti gigi; TC di grip 0.6: slip rata-rata 151 % → 17 % dan mobil
  lebih cepat; transbrake menahan mobil di ±2700 rpm dengan boost 1.3 bar lalu meluncur; rolling ALS menahan
  rpm dan membangun boost; nitrous 100 hp di dyno: +99 hp, AFR 12.4 (wet) / 12.3 (dry), lean → mati.
- Ikut diperbaiki: pengayaan laju-MAP (ECU menangkap idle yang turun) tidak lagi dihitung sebagai transien.
  Sebelumnya setiap turunan idle me-reset closed loop, sehingga idle yang naik-turun hampir tidak pernah masuk
  closed loop (tes baterai macet: closed loop 40 % waktu, λ 0.875 → sekarang 100 %, λ 0.999).
- Belum: kontrol VVT/VTEC oleh ECU, torque management, kontrol berbasis gigi selain boost.

### Fase 6 – Simulasi lanjutan
- Peta efisiensi kompresor dan turbin yang bisa dimuat.
- Model valve float dari pegas katup dan massa komponen.
- Perbaikan model termal dan dyno.

Sebagian besar isi fase ini (simulasi per silinder, dinamika turbo, drivetrain) sudah ada.

---

## 5. Keputusan (sudah disepakati)

1. **Urutan fase:** sesuai daftar di atas (1 Fondasi → 2 Kalibrasi → 3 Kontrol → 4 Monitoring → 5 Motorsport → 6 Lanjutan).
2. **Isi Fase 1:** seperti yang ditulis, ditambah navigasi UI baru (1a) dan katalog komponen lengkap (1b).
3. **Komponen:** katalog lengkap, dan tiap komponen punya efek fisik.
4. **UI:** navigasi 12 bagian langsung dipasang di Fase 1a.
5. **File kalibrasi:** disimpan sebagai JSON di folder `calibrations/`, satu set profil per konfigurasi mesin. Semua pengaturan tetap diubah langsung di aplikasi.
6. **Dependensi baru:** ImPlot3D (FetchContent) untuk permukaan 3D tabel, di Fase 2.
7. **Gaya UI:** gelap dan padat, terinspirasi software tuning profesional, tanpa nama, logo, atau tampilan merek tertentu.
8. **Satuan:** metrik default, dengan tombol imperial (Fase 4).
