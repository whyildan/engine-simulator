# EngineSim → simulasi mesin + platform tuning ECU: analisis dan rencana

Dokumen ini menjawab `tuning-update.md` bagian 19, langkah 1–6: memeriksa kode yang sudah ada, memetakan
fitur yang sudah ada dan yang belum, lalu membagi pekerjaan per fase. Pekerjaan dilakukan satu fase per
satu, dengan tes di akhir tiap fase.

Status: **Fase 1–7 selesai (lihat bagian 4). Semua fitur spesifikasi di tabel bagian 2 sudah ✅, kecuali model ECU /
I/O / CAN yang sengaja tidak didukung (➖); diesel ditunda (lihat "Sisa pekerjaan").**

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

Keterangan: ✅ sudah ada · 🟡 sebagian (perlu dirombak atau diperluas) · ❌ belum ada · ➖ sengaja tidak didukung

Tabel ini menggambarkan kondisi setelah Fase 7 (diperbarui terhadap kode, bukan kondisi awal).

| § | Fitur | Status | Catatan |
|---|---|---|---|
| 1 | Arsitektur I3–V12, flat-4/6, rotari | ✅ | Termasuk preset I5 dan flat-6 |
| 1 | Bore/stroke/rod/CR/firing/limiter/inersia | ✅ | |
| 1 | Redline terpisah, friksi mekanis yang bisa diatur | ✅ | Fase 7: redline (zona merah tach + lampu shift) dan pengali friksi |
| 1 | Komponen (piston, rod, crank, cam, pegas katup, manifold, throttle body, injector, coil, busi, pompa oli/air) | ✅ | Fase 1b; valve float dinamis di Fase 6 |
| 2 | Pemilihan model ECU | ➖ | Ditandai "not supported" di ECU Setup: satu profil ECU generik |
| 2 | Pola trigger crank, sinkronisasi cam | ✅ | Fase 7: 60-2, 36-1, 36-2-2-2, 24-1, 12-1, 4 gigi + cam; sinkron setelah celah lewat, rpm per gigi |
| 2 | Mode injeksi sequential / batch / semi-sequential | ✅ | Fase 7, bisa dipilih; tanpa fase cam → semi-sequential |
| 2 | Konfigurasi pengapian | ✅ | Fase 7: tipe coil di ECU Setup |
| 2 | Assignment I/O, CAN bus | ➖ | Ditandai "not supported" di ECU Setup |
| 2 | Speed density / Alpha-N / MAF / hybrid, open / closed loop | ✅ | Fase 2–3 |
| 3 | Tabel VE, target AFR/λ, injector + dead time, tekanan bahan bakar | ✅ | Fase 2 |
| 3 | Pengayaan cranking/warm-up/akselerasi, DFCO, after-start, trim | ✅ | Fase 2–3 |
| 3 | Editor tabel (sel, interpolasi, smoothing, copy/paste, undo/redo, sel aktif, compare, per silinder) | ✅ | Fase 2 |
| 3 | Permukaan 3D | ✅ | ImPlot3D |
| 4 | Tabel pengapian, cranking, koreksi per silinder, koreksi suhu/beban | ✅ | Fase 2 |
| 4 | Dwell time | ✅ | Fase 7: tabel dwell vs tegangan aki + batas duty; energi percikan mengikutinya |
| 4 | Knock detection + retard | ✅ | Fase 3, per silinder |
| 5 | Turbo, wastegate, inersia, efisiensi, peta kompresor / turbin | ✅ | Peta tabel + CSV di Fase 6 |
| 5 | Tabel boost (rpm/gigi/TPS), duty wastegate, PID, overboost, boost cut | ✅ | Fase 3 |
| 6 | Idle, DBW, soft/hard limiter, launch, 2-step/3-step, flat shift, anti-lag, VVT, VTEC | ✅ | Fase 3, 5, dan setelah Fase 5 |
| 6 | Torque management | ✅ | Fase 7: batas torsi per gigi lewat throttle DBW, retard, spark cut |
| 6 | Kontrol berbasis gigi (selain boost) | ✅ | Fase 7: limiter dan trim timing per gigi |
| 7 | Sensor virtual (tegangan, noise, delay, gagal, fallback) | ✅ | Fase 1c |
| 8 | Lambda STFT/LTFT, batas, kondisi learning, Auto Tune | ✅ | Fase 3 |
| 8 | Kontrol knock per silinder, PID boost | ✅ | Fase 3 |
| 9 | Proteksi dan limp mode | ✅ | Fase 3, 10 aturan |
| 10 | Launch, TC, rolling anti-lag, flat shift, 2/3-step, boost by gear, nitrous, transbrake | ✅ | Fase 5 |
| 10 | Torque management (balap) | ✅ | Sama dengan §6 |
| 11 | Dashboard live yang bisa diatur | ✅ | Fase 4 |
| 12 | Data logging, grafik, CSV, perbandingan sesi | ✅ | Fase 4, 55 channel |
| 13 | Dyno steady/sweep, perbandingan, ekspor, satuan | ✅ | Fase 4 + step test dan koreksi di Fase 6 |
| 14 | Pemisahan fisika / sensor / ECU, frekuensi bisa diatur | ✅ | Fase 1c |
| 15 | Manajemen kalibrasi (simpan, muat, versi, compare, lock, validasi) | ✅ | Fase 2 |
| 16 | UI gelap padat, navigasi 12 bagian | ✅ | Fase 1a |
| 17 | Pause/resume/reset, sesi deterministik | ✅ | Fase 1c |

**Sisa pekerjaan:**
- Pemilihan model ECU, assignment I/O, CAN bus (§2) — sengaja tidak didukung (ditandai di ECU Setup).
- Diesel (ditunda; fitur ECU yang menjadi syaratnya sudah selesai di Fase 7) — Paket 4 di bawah.

### Rencana update berikutnya (urutan prioritas)

Bobot: **S** kecil (beberapa jam) · **M** sedang (±1 sesi) · **L** besar (beberapa sesi) · **XL** sangat besar.

| Urutan | Paket | Isi | Bobot | Butuh dulu | Kenapa di urutan ini |
|---|---|---|---|---|---|
| 1 | **Bahan bakar dan flex fuel** | pilihan bahan bakar (regular / premium / race / E10 / E20 / E85 / E100 / metanol) dengan AFR, LHV, oktan, panas penguapan, kecepatan api; sensor flex fuel + koreksi ECU otomatis | **L** | – | Fondasi: AFR stoik dan LHV sekarang konstanta tetap di ±20 tempat. Wideband (paket 2) dan diesel (paket 4) butuh sifat bahan bakar yang bisa berganti. Efeknya juga paling terasa (knock, tenaga, start dingin). |
| 2 | **Sensor knalpot** | sensor EMAP (backpressure, rasio drive pressure, proteksi); wideband membaca gas di pipa; posisi EGT (per silinder, sebelum / sesudah turbin) | **M** | paket 1 (λ dihitung dari AFR stoik bahan bakar yang dipakai) | Pekerjaan kecil-menengah dengan manfaat tuning besar; posisi sensor ini dipakai paket 3. |
| 3 | **Peta aliran dan kesehatan mesin** | skema jalur gas dari konfigurasi, partikel aliran (kecepatan = aliran, warna = suhu, aliran balik terlihat), warna komponen dari keadaan simulasi, panel kesehatan, hover nilai, mode diagnosis | **L** | paket 2 (sensor digambar di posisinya, EMAP di skema turbo) | Murni tampilan di atas data yang sudah ada; paling bagus setelah sensor berada di posisi nyata. |
| 4 | **Diesel** | pembakaran kompresi (tanpa busi), injeksi langsung + timing injeksi, tanpa throttle (beban = jumlah bahan bakar), busi pijar, turbo VGT, smoke limit, EGR | **XL** | paket 1 (bahan bakar sebagai data), paket 2 (EMAP penting untuk turbo diesel) | Terbesar dan paling berbeda dari mesin bensin; dikerjakan terakhir setelah fondasinya siap. |

Tiap paket ditutup dengan tes sendiri dan regresi penuh, seperti fase-fase sebelumnya.

**Paket 1 – Bahan bakar dan flex fuel (detail):**
- Pilihan bahan bakar di Engine Setup, masing-masing dengan sifat fisiknya. Nilai di bawah adalah kisaran dari
  literatur dan masih perlu dicek ulang saat implementasi:

  | Bahan bakar | RON / MON | AFR stoik | LHV MJ/kg | Panas penguapan kJ/kg | Catatan |
  |---|---|---|---|---|---|
  | Regular (Pertalite RON 90, US "87" AKI) | 90 / ±82 | 14.6–14.7 | ±43 | ±350 | rasio kompresi ±9–10:1 |
  | Premium (Pertamax 92, RON 95, Pertamax Turbo 98) | 92–98 / 84–88 | 14.6–14.7 | ±43 | ±350 | rasio kompresi ±10–13:1 |
  | Race fuel (tanpa timbal) | ±100–110 / 98–105 | ±14.0–14.7 | ±43–44 | ±350 | knock resistance tinggi |
  | E10 | +2–3 RON di atas basisnya | ±14.0 | ±41.5 | ±400 | O₂ ±3.5 % berat |
  | E20 | ±+5 RON | ±13.4 | ±40 | ±450 | |
  | E85 | ±105–108 / ±88–90 | ±9.8 | ±29 | ±800 | injektor ±+35 % aliran; susah start dingin |
  | Etanol murni (E100) | 108–110 / ±90–92 | 9.0 | 26.9 | ±900 | |
  | Metanol (M100) | 108–109 / ±87–89 | 6.45 | 20.0 | ±1100 | injektor ±2× aliran; api hampir tak terlihat |

- Efeknya di simulasi (semua harus nyata, bukan sekadar label):
  - **AFR stoikiometri dan LHV per bahan bakar.** Sekarang `kStoich` 14.7 dan `kLHV` 44 MJ/kg adalah konstanta
    tetap di ±20 tempat. Keduanya menentukan massa bahan bakar, energi pembakaran, dan tampilan AFR/λ.
  - **Knock dari oktan.** Ambang indeks knock mengikuti RON/MON (sensitivitas RON − MON berpengaruh di beban
    tinggi), sehingga bensin regular di mesin rasio kompresi tinggi knock dan knock control me-retard.
  - **Pendinginan muatan dari panas penguapan.** Alkohol mendinginkan muatan (muatan lebih padat, ikut menahan
    knock).
  - **Kecepatan api.** Alkohol terbakar sedikit lebih cepat, sehingga durasi pembakaran lebih pendek.
  - **Start dingin.** Alkohol sulit menguap saat dingin, jadi butuh pengayaan cranking/warm-up lebih besar dan bisa
    gagal start di suhu rendah.
  - **Injektor dan pompa.** Duty injektor naik sesuai kebutuhan massa bahan bakar.
  - **Sensor flex fuel (wajib, bukan opsional).** Sensor kadar etanol di saluran bahan bakar (frekuensi 50–150 Hz
    = E0–E100, ditambah suhu bahan bakar) sebagai sensor virtual baru, lengkap dengan noise, jeda, dan kegagalan.
    ECU memakainya untuk mengoreksi otomatis:
    - AFR stoikiometri dan pulse injektor;
    - timing (lebih maju dengan etanol tinggi);
    - pengayaan cranking / warm-up (alkohol susah menguap saat dingin);
    - ambang knock.
    Kadar etanol berubah bertahap saat mengisi bahan bakar campuran (tangki + saluran). Tanpa sensor atau saat sensor
    rusak: ECU memakai nilai cadangan dari kalibrasi, sehingga ganti bahan bakar = kalibrasi harus disesuaikan.
- Rujukan:
  - [IEA AMF – sifat etanol](https://iea-amf.org/content/fuel_information/ethanol/fuel_properties)
  - [IEA AMF – sifat metanol](https://iea-amf.org/content/fuel_information/methanol/fuel_properties)
  - [NREL 63091 – heat of vaporization campuran etanol](https://www.nrel.gov/docs/fy15osti/63091.pdf)
  - [SAE 2010-01-2112 – AFR stoikiometri bensin beretanol](https://saemobilus.sae.org/content/2010-01-2112)
  - [ORNL – knock limit E85](https://bioenergykdf.ornl.gov/system/files/The%20Impact%20of%20Low%20Octane%20Hydrocarbon%20Blending%20Streams%20on%20the%20Knock%20Limit%20of%20E85_Szybist.pdf)
  - [ESDM – jenis BBM Indonesia (RON 90 / 92 / 98)](https://www.esdm.go.id/id/media-center/news-archives/yuk-cerdas-pilih-bbm-berkualitas)

**Paket 2 – Sensor knalpot (detail):**
- **Sensor EMAP (backpressure)** untuk mesin turbo: tekanan manifold sebelum turbin (node `turbineIn` sudah
  dihitung fisika), rasio drive pressure (EMAP ÷ boost), channel log, aturan proteksi backpressure tinggi.
- **Wideband membaca gas di pipa**: λ dihitung dari komposisi gas di collector / downpipe (bahan bakar, gas
  terbakar, udara) — misfire dan campuran yang lolos saat overlap ikut terbaca seperti sensor sungguhan. Sekarang
  λ = rata-rata silinder + jeda transport. Turbo: wideband setelah turbin.
- **Posisi EGT**: opsional EGT per silinder (header), dan EGT sebelum / sesudah turbin pada mesin turbo. Sekarang
  satu termokopel di collector (= sebelum turbin pada turbo).

**Paket 3 – Peta aliran dan kesehatan mesin, Live Dashboard › Flow map (detail):**
- **Skema jalur gas yang dibangun dari konfigurasi mesin**, bukan gambar tetap:
  - jalurnya: air filter → kompresor / supercharger → intercooler → throttle (atau ITB / karburator) → plenum /
    runner → head dan katup → header → turbin / wastegate → katalis → muffler;
  - mesin NA tanpa turbo dan intercooler; rotari dengan port;
  - **gaya (sudah diputuskan):** tema aplikasi sendiri — latar gelap dengan aksen oranye, warna status hijau /
    kuning / merah hanya untuk keadaan komponen — bukan hijau neon seperti gambar referensi, supaya orisinal dan
    menyatu dengan UI yang sekarang.
- **Aliran hidup:**
  - partikel bergerak di pipa, kecepatannya mengikuti aliran massa dan warnanya mengikuti suhu gas (biru dingin →
    oranye panas);
  - tebal garis mengikuti tekanan;
  - aliran balik (reversion, standoff di depan throttle, surge kompresor) terlihat sebagai partikel yang mundur.
- **Warna komponen dari keadaan simulasi** (hijau = normal, kuning = peringatan, merah = batas):
  - turbo: overspeed, surge, EGT sebelum turbin;
  - intercooler: heat soak;
  - throttle / DBW: dibatasi proteksi;
  - head: knock, valve float;
  - header: EGT;
  - busi: fouling, energi percikan.
  Ambang batasnya diambil dari aturan proteksi dan batas komponen yang sudah ada.
- **Panel kesehatan di bawah skema:** check engine, knock, busi, sensor, katup, rod, crank, piston, perbaikan.
  - Data dari ComponentModel (stres, damage, kegagalan), valve float, fouling, status sensor, dan aturan proteksi.
  - Klik ikon → lompat ke panel terkait (Components, Knock control, Sensor Configuration, …).
- **Hover / klik node** → nilai langsung: tekanan, suhu, aliran, efisiensi kompresor / turbin, rasio drive pressure
  (bersama sensor EMAP).
- **Mode diagnosis:** sensor yang diberi gangguan (Sensor Configuration) berkedip di posisinya pada skema; lambda dan
  EGT digambar di posisi pemasangannya (bersama update posisi sensor knalpot).
- Tidak termasuk: keausan jangka panjang (jarak tempuh, servis) — sudah diputuskan tidak dibuat.

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
- **Channel log** (`src/sim/DataLog.*`): 43 channel (50 sejak Fase 5 + cam) (key, nama, satuan, grup, getter dari telemetri). Sampel
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

### Setelah Fase 5 – K20A, VTEC / VTC, misfire dan letupan
- **Preset 0 kini K20A-style (i-VTEC)**: 86 × 86, CR 11.5, 4 katup 35/30 mm, cam low 236°/9.8 mm (overlap kecil:
  idle bersih), cam high 272°/12.0 mm mulai 5800 rpm di beban, VTC 50°, limiter 8600 rpm, flywheel 0.12, runner
  260 mm, header 4-2-1 primer 380 mm, transmisi rasio rapat DC5. Dyno: 238 Nm @ 6000, **220 PS @ 7000–7500**.
  Preset 16 (I4 turbo) tetap memakai I4 generik lama.
- **VTEC**: profil cam kedua (durasi/lift intake & exhaust). ECU memindah ke cam high di atas rpm switch-over saat
  MAP dan coolant cukup, dengan histeresis; spool valve butuh tekanan oli dan ±80 ms. Sumbu rpm tabel VE punya
  titik tepat di kiri-kanan switch-over dan rig base map mengukur keduanya (base map versi 17).
- **VTC**: phaser intake kontinu dari peta advance (rpm × MAP, tabel kalibrasi baru), ±200°/s, butuh oli.
  UI: Engine Setup › Valvetrain (hardware + grafik profil low/high) dan Engine Control › Cam control.
- **Misfire**: siklus yang di-cut (DFCO/limiter, termasuk muatan yang masuk saat cut) tidak dihitung misfire;
  penyebab misfire dicatat (lean, spark, fouled, rich, dilution) — alat `misfire_probe`.
- **Letupan (afterfire)**: perlu oksigen (gas buang campuran kaya tidak punya), perlu campuran dalam batas nyala
  (λ < ±1.7: jejak bahan bakar saat fuel cut terlalu tipis), dan muatan yang lolos dari silinder yang menyala
  (scavenging overlap, pembakaran terlambat) terbakar kontinu di port (panas, tanpa letupan). Letupan tetap ada
  dari muatan yang misfire / di-cut (2-step, rotari brap) dan saat overrun.
- **DFCO**: delay 0.25 s sebelum fuel cut (blip/angkat gas singkat tidak memotong), pengayaan keluar-DFCO 150 %
  yang meluruh 0.25 s — setelah cut, gas sisa di ruang bakar adalah udara, pada vakum tinggi hampir sebanyak
  muatan segar, sehingga siklus pertama sangat kurus. Titik resume dan antisipasinya kini kalibrasi.
- **VE generik** memperhitungkan overlap cam (cam overlap kecil menangkap lebih banyak udara di beban rendah /
  putaran rendah) — K20A tidak lagi mati sebelum base map pertama selesai diukur.
- **Hasil K20A** (`misfire_probe`): idle, cruise, WOT dan deselerasi 0–2 misfire dan 0–1 letupan; idle 860 rpm
  (simpangan ±50). Sebelumnya I4 lama: ratusan misfire saat blip/deselerasi dan letupan saat WOT dan idle.
- Preset 20 (V6 supercharger sentrifugal): gain idle lebih lembut (volume intake supercharger besar).

### Engine view 2.5D
- `src/ui/EngineView3D.*` menggantikan animasi 2D lama: setiap part (crank berbobot imbang, rod, piston dengan
  alur ring, dinding liner, head dengan port, katup bentuk T, follower, lobe cam yang berputar setengah putaran
  engkol; rotari: housing epitrochoid, rotor dengan roda gigi, poros eksentrik, port, busi) berupa bentuk datar
  cembung yang diberi ketebalan, ditempatkan per silinder sepanjang poros engkol. Proyeksi perspektif software di
  ImGui draw list, culling sisi belakang, diurutkan per irisan sepanjang poros lalu per lapisan.
- Kamera: drag = putar, klik kanan/tengah = geser, roda = zoom, klik ganda = reset; tombol Front / Side / 3/4 dan
  slider tebal potongan. Menampilkan lift katup, VTEC (lobe cam tinggi) dan VTC langsung dari telemetri.

### Fase 6 – Simulasi lanjutan ✅ selesai
- Peta efisiensi kompresor dan turbin yang bisa dimuat.
- Model valve float dari pegas katup dan massa komponen.
- Perbaikan model termal dan dyno.

Implementasi:
- **Peta turbo** (`src/sim/TurboMap.*`): peta kompresor = garis kecepatan (maks 10 × 12 titik) berisi aliran
  terkoreksi (298 K / 101.325 kPa), rasio tekanan, dan efisiensi; peta turbin = aliran tereduksi
  m·√T/p terhadap rasio ekspansi + efisiensi terhadap U/C + diameter roda. Tiap garis di-resample sepanjang
  panjangnya, diinterpolasi antar garis, dan di luar garis memakai fan law. Titik di atas puncak garis = surge,
  di bawah ujung garis = choke. Turbin dari peta dijadikan luas nosel ekuivalen (stabil seperti model lama).
  Bawaan: 6 kompresor (44–76 mm + kepala supercharger sentrifugal) dan 4 turbin (sintetis, bentuk realistis).
  Muat / simpan CSV (`speed,flow,pr,eff`; flow juga `flow_gs` / `flow_lbmin`; turbin `er,mfp` + `uc,eff`).
  Model generik (slider ukuran) tetap default; peta dipilih di Boost Control. UI: peta dengan pulau efisiensi
  (marching squares), garis surge, jejak titik operasi, kurva efisiensi turbin.
- **Valvetrain dinamis** (`src/sim/Valvetrain.*`): katup = massa pada pegas, digerakkan lewat train elastis
  (kekakuan per tipe lifter: pushrod ±5 kN/mm, bucket 12 kN/mm), mendarat di dudukan elastis (restitusi ±0.45).
  Satu event katup diintegrasi per rpm (dihitung ulang tiap rpm berubah 1.5 %), hasilnya tabel deviasi dari
  lift cam yang langsung masuk ke luas aliran katup. Penggeraknya profil pengganti (1 − x²)³(1 + 1.15x²) dengan
  kelengkungan hidung sama: profil asli tidak punya ramp sehingga train ikut bergetar. Telemetri: loft,
  bounce, terlambat menutup. Pegas default pushrod diberi margin lebih (train elastis float lebih awal).
  K20A: float ±9300 rpm (limiter 8600); pegas 60 % → float ±7400 rpm dan torsi di 7800 rpm hilang.
- **Ambient**: tekanan udara dari ketinggian (atmosfer standar), per thread simulasi (`kPatm`), dibaca ECU
  sebagai sensor baro (alpha-N, boost gauge, tekanan injektor, overboost). Base map diukur di tekanan itu.
- **Intercooler**: efektivitas terhadap suhu inti; inti menyerap panas kompresor dan membuangnya ke udara yang
  ditiup (kecepatan kendaraan atau kipas sel dyno) → heat soak saat diam / dyno tanpa kipas.
- **Dyno**: mode step test (tahan tiap rpm sampai stabil lalu dirata-rata), kipas sel (radiator + intercooler),
  faktor koreksi SAE J1349 / DIN 70020 / ISO 1585 / SAE J607 dari suhu dan tekanan saat run, data per titik
  bertambah (boost, EGT, timing, IAT, BSFC), ekspor CSV memuat semuanya. Channel log baru: efisiensi kompresor,
  suhu inti intercooler, suhu charge, valve float, loft.
- Ikut diperbaiki: lift VTEC memakai dinamika cam tinggi saja (cam rendah hanya kinematis saat terkunci).
  Sebelumnya, cam rendah yang float ikut terbawa walaupun rocker sudah dikunci ke cam tinggi. ECU kini punya
  kompensasi beban "drive": katup idle dibuka lebih dulu saat transmisi otomatis masuk gigi, dan lebih lebar
  saat transbrake ditahan. Tanpa itu idle melawan transbrake berayun 400–900 rpm dan bisa mati. Rig base map
  memakai kipas sel default (50 km/h), sama dengan dyno (base map versi 20).
- Tes: `phase6_test`, alat `valve_probe`.

### Fase 7 – Konfigurasi ECU, torque management, kontrol per gigi ✅ selesai
- **Trigger** (`Engine::triggerStep`, kalibrasi `trigger_pattern`, `cam_sensor`): roda 60-2, 36-1, 36-2-2-2,
  24-1, 12-1, atau 4 gigi + gigi cam. Kecepatan CKP diukur per gigi (roda bergigi sedikit = pembacaan lebih
  jarang). Sudut engkol baru diketahui setelah celah gigi lewat (90° sebelum TMA silinder 1, + 2 gigi untuk
  mengenali celah); sebelum itu tidak ada percikan / bahan bakar. Fase siklus diketahui setelah gigi cam lewat;
  sebelum itu (atau tanpa sensor cam) injeksi semi-sequential. K20A: 36-2-2-2 sinkron dalam 50°, 60-2 / 36-1
  ±290°, roda 4 gigi + cam 780°; semua hidup dalam 0.5–0.7 s.
- **Injeksi**: sequential, semi-sequential (tiap port dua kali per siklus, setengah), batch (semua injektor
  bersamaan di satu sudut silinder 1). Rotari tetap sequential.
- **Pengapian**: tipe coil di ECU Setup; tabel dwell vs tegangan aki (default 1.3 × waktu isi coil × 14/V) dan
  batas duty dwell. Fisika: energi = E·(1 − e^(−3·dwell/τ)), τ = waktu isi × 14/V, dwell maks = selang antar
  percikan coil − 1.2 ms. Dwell 0.25 ms pada I4 turbo: 18 mJ, 266 misfire spark, torsi 434 → 236 Nm.
- **Torque management** (`Ecu::torqueStep`): estimasi torsi = muatan udara silinder / muatan penuh NA (%),
  batas per gigi. DBW: batas throttle diintegrasikan ke batas (bersih); sisa di atas batas: retard (±15 %
  pertama) lalu spark cut. K20A gigi 1 batas 60 %: DBW torsi kopling 190 → 63 Nm (beban 58 %), throttle kabel
  179 → 117 Nm.
- **Kontrol per gigi**: limiter dan trim timing per gigi dari sensor posisi gigi (gigi 1 dibatasi 5000 rpm →
  maks 5100 rpm).
- **Mesin**: pengali friksi (×1.5 → torsi 6000 rpm 237 → 219 Nm) dan redline (zona merah tach, lampu Shift;
  default limiter − 300).
- UI: ECU Setup (Trigger, Injection, Ignition + tabel dwell, Hardware: model ECU / I/O / CAN "not supported"),
  Engine Control › Torque / gear, Engine Setup › Engine (redline, friksi). Base map versi 21. Tes:
  `phase7_test`.
- Ikut diperbaiki di tes: severity float berayun antar siklus begitu katup float (tes memakai nilai terburuk),
  auto-tune VE pada idle yang naik-turun diberi waktu belajar 20 s.
- Diketahui saat itu: tip-in dari idle kadang jatuh ke misfire kurus beruntun dan mati — penyebabnya ditemukan dan
  diperbaiki setelah Fase 7 (bahan bakar yang keluar mundur lewat throttle dihapus, lihat di bawah).

### Setelah Fase 7 – Idle lebih halus
- Gejala: idle naik-turun dan bunyi "dung" — katup idle melompat dari ±10–15 % ke 30–56 % sesaat, siklus
  berikutnya terlalu terisi dan meledak keras, rpm terlempar lalu turun lagi.
- Penyebab: kontrol idle (timing dan katup) memakai rpm mentah, yang ikut riak di antara dua pembakaran. Timing
  idle (0.04°/rpm) berganti tiap siklus mengikuti riak itu → pembakaran tidak rata; error katup ikut berkedut.
- Perbaikan: kontrol idle memakai rpm yang dirata-rata sekitar satu selang pembakaran (K20A ±35 ms, V10 ±14 ms) —
  riak hilang, penurunan rpm yang sebenarnya tidak tertunda. Pendekatan lain (katup dibuat lambat / dibatasi lajunya)
  diuji dan ditolak: mesin cam besar (V10, V12, flatplane, flat-6) butuh katup yang cepat dan bisa mati tanpa itu.
- Hasil (`idle_probe`, 12 s idle): K20A ±49 → ±24 rpm, CoV IMEP 32 → 9 %, katup maks 56 → 25 %, "dung" 1 → 0;
  I3 / I6 / V6 / I5 / flat-4 / 13B CoV 22–29 % → 10–15 %. Mesin cam besar tetap lumpy (karakter overlap).
- **Bug bahan bakar hilang saat tip-in (diperbaiki)**: di WOT rpm rendah dengan cam overlap / IVC lambat, muatan
  yang sudah masuk silinder terdorong balik ke port, runner, plenum, lalu keluar mundur lewat throttle. Gas yang
  keluar mundur itu **dihapus** (NA: dianggap hilang ke atmosfer; boost: tidak masuk ke pipa charge, dan aliran
  maju dari charge pun hanya membawa udara). Neraca bahan bakar saat blip V8 Roots: ±150–220 mg per 50 ms (60–70 %
  dari yang disemprot) hilang di throttle → λ silinder 1.5–3 → misfire kurus beruntun → mati. Inilah stall
  "acak" di survey sejak Fase 6 (V8 Roots / K20A, imperfection 0 / 1).
  Sekarang: NA — gas yang keluar mundur ditahan sebagai "standoff" di depan throttle (maks ±volume inlet) dan
  terhisap kembali lebih dulu (air box menahan semuanya, velocity stack terbuka hanya ±40 % — sisanya lepas ke
  udara; tanpa itu 26B peripheral port banjir saat cranking); boost — masuk ke pipa charge, aliran maju membawa komposisinya; perubahan suhu
  plenum mempertahankan komposisi. Karburator hanya menakar bahan bakar dari udara segar (puff standoff sudah
  membawa bahan bakarnya; menakar ulang membuat AFR ±2.7 dan mati). Base map versi 22.

### Setelah Fase 7 – EGT
- Gejala: EGT idle terlalu tinggi (single thumper 730–750 °C, mesin nyata ±350–500 °C), EGT naik tajam saat gas
  dilepas dari putaran tinggi (K20A +360 K, single carb sampai 1370 °C), padahal EGT WOT justru terlalu rendah
  (K20A ±550 °C, mesin nyata ±850–950 °C).
- Penyebab dan perbaikan:
  1. Suhu gas keluar silinder dirata-rata dengan bobot q / massa yang tersisa di silinder: gas terakhir (overlap,
     dingin di WOT; residu panas di idle) paling menentukan. Sekarang rata-rata berbobot massa per event buang.
  2. Campuran yang lolos saat overlap menyala di port bila gas blowdown (suhu saat katup buang membuka) panas, bukan
     menurut rata-rata gas yang sudah tercampur campuran dingin.
  3. Panas afterfire dibagi aliran gas buang yang sebenarnya keluar dari silinder (bukan aliran throttle yang
     langsung berhenti saat gas dilepas), dirata-rata sama; kenaikan suhu dibatasi 700 K.
  4. Sensor EGT = termokopel di header: konveksi dari gas (kuat di aliran cepat, lemah di idle / saat gas dilepas),
     radiasi ke dinding pipa yang memanas pelan. Di idle membaca jauh di bawah suhu gas, di WOT hampir sama.
  5. Port buang (didinginkan air) mengambil lebih banyak panas dari gas idle yang lambat (batas Nusselt laminar).
- Hasil (`egt_trace`): idle K20A 641 → 445 °C, single thumper 744 → 522 °C, I4 turbo 704 → 485 °C; WOT K20A
  544 → 825 °C, I4 turbo 976 °C (sebelum turbin), 13B 895 °C. Lepas gas dari redline: K20A +64 K lalu turun,
  single carb +56 K. Suhu gas di WOT ikut menentukan kecepatan gelombang di pipa → base map versi 23.
- Ikut menyesuaikan: gas WOT yang lebih panas memberi turbin lebih banyak energi — wastegate kini bisa membuang
  2.5× luas turbin (sebelumnya 1.3×: boost merayap 0.4 bar di atas pegas). Transbrake: beban converter naik dalam
  ±0.3 s (paket kopling terisi tekanan oli) dan feed-forward katup idle saat transbrake 35 %.

### Setelah Fase 7 – Bunyi "ngiing" saat mesin diam
- Gejala: di 0 rpm terdengar nada tinggi (K20A ±1.4 kHz dari intake, ±1.2 kHz dari exhaust), paling keras saat
  katup intake sebuah silinder terbuka. Nadanya tidak meredam, malah naik lalu bertahan.
- Penyebab: volume kecil (port, plenum, kolektor) disambung ke pipa gelombang memakai tekanan dari sebelum
  pertukaran massa langkah itu. Telat satu sampel pada sebuah compliance bekerja seperti resistansi negatif, jadi
  resonansi runner + port mendapat energi terus. Saat mesin jalan tertutup rugi aliran; saat diam tidak ada yang
  meredam.
- Perbaikan (`WaveNetwork::step`): tekanan node dihitung trapezoidal (tengah pertukaran) - tidak menambah atau
  membuang energi. Backward Euler juga menghentikan bunyi tetapi meredam gelombang (daya puncak K20A −4 %);
  trapezoidal −2 % (sebagian tuning gelombang sebelumnya ikut digelembungkan artefak yang sama). Base map versi 24.
- Uji: `stop_probe` (crank diparkir tiap 30°, dan mesin dimatikan dengan kunci) - semua preset diam setelah kunci
  dimatikan; yang tersisa hanya gerak pelan crank yang diparkir paksa di tengah kompresi (±50 Hz, meredam).

### Setelah Fase 7 – Simpan / bagikan mesin
- **Engine Setup → Engine → Save / share this engine**: simpan mesin sendiri ke *My engines* (muncul di daftar
  preset, header dan tab Engine), **Export file...** ke satu file `.engine.json` untuk dibagikan, **Import file...**
  memuat file orang lain dan menyimpan salinannya di *My engines*, **Delete** dengan konfirmasi. Author dan notes ikut
  tersimpan (tooltip di daftar).
- Isi file (`src/ecu/EngineFile.h`, implementasi di `CalibrationIO.cpp`): format `enginesim-engine` versi 1, semua
  parameter hardware ditulis per nama (geometri, valvetrain, VTEC/VTC, intake, bahan bakar, exhaust, rotari, forced
  induction termasuk peta turbo, kendaraan, nitrous, karakter suara, komponen) plus kalibrasi ECU yang sedang
  dipakai. Kondisi bench (gas, kunci kontak, dyno, udara sekitar, sensor rusak, listener) tidak ikut, jadi tetap
  saat ganti mesin.
- Field yang tidak ada di file memakai nilai mesin yang sedang aktif; nilai di luar batas di-clamp, firing order
  rusak diganti default. File tanpa kalibrasi → kalibrasi dasar dibuat dan base map diukur seperti preset.
- Disimpan di folder `engines/` di samping exe. Uji: `engine_file_test` (24 preset round trip identik, peta turbo,
  file sebagian / rusak, clamp, simpan / daftar / muat / export / hapus, mesin hasil import bisa idle).

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
