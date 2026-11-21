# EngineSim → simulasi mesin + platform tuning ECU: analisis dan rencana

Dokumen ini menjawab `tuning-update.md` bagian 19, langkah 1–6: memeriksa kode yang sudah ada, memetakan
fitur yang sudah ada dan yang belum, lalu membagi pekerjaan per fase. Pekerjaan dilakukan satu fase per
satu, dengan tes di akhir tiap fase.

Status: **Fase 1–7 selesai (lihat bagian 4). Semua fitur spesifikasi di tabel bagian 2 sudah ✅, kecuali model ECU /
I/O / CAN yang sengaja tidak didukung (➖). Paket update 1–3 selesai; diesel (paket 4) berikutnya (lihat "Rencana update
berikutnya").**

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
- Mesin 2-tak (belum didukung: tipe mesin hanya piston 4-tak dan rotari) — Paket 5 di bawah.

### Rencana update berikutnya (urutan prioritas)

Bobot: **S** kecil (beberapa jam) · **M** sedang (±1 sesi) · **L** besar (beberapa sesi) · **XL** sangat besar.

| Urutan | Paket | Isi | Bobot | Butuh dulu | Kenapa di urutan ini |
|---|---|---|---|---|---|
| 1 | **Bahan bakar dan flex fuel** ✅ | pilihan bahan bakar (regular / premium / race / E10 / E20 / E85 / E100 / metanol) dengan AFR, LHV, oktan, panas penguapan, kecepatan api; sensor flex fuel + koreksi ECU otomatis | **L** | – | Fondasi: AFR stoik dan LHV sekarang konstanta tetap di ±20 tempat. Wideband (paket 2) dan diesel (paket 4) butuh sifat bahan bakar yang bisa berganti. Efeknya juga paling terasa (knock, tenaga, start dingin). |
| 2 | **Sensor knalpot** ✅ | sensor EMAP (backpressure, rasio drive pressure, proteksi); wideband membaca gas di pipa; posisi EGT (per silinder, sebelum / sesudah turbin) | **M** | paket 1 (λ dihitung dari AFR stoik bahan bakar yang dipakai) | Pekerjaan kecil-menengah dengan manfaat tuning besar; posisi sensor ini dipakai paket 3. |
| 3 | **Peta aliran dan kesehatan mesin** ✅ | skema jalur gas dari konfigurasi, partikel aliran (kecepatan = aliran, warna = suhu, aliran balik terlihat), warna komponen dari keadaan simulasi, panel kesehatan, hover nilai, mode diagnosis | **L** | paket 2 (sensor digambar di posisinya, EMAP di skema turbo) | Murni tampilan di atas data yang sudah ada; paling bagus setelah sensor berada di posisi nyata. |
| 4 | **Diesel** | pembakaran kompresi (tanpa busi), injeksi langsung + timing injeksi, tanpa throttle (beban = jumlah bahan bakar), busi pijar, turbo VGT, smoke limit, EGR | **XL** | paket 1 (bahan bakar sebagai data), paket 2 (EMAP penting untuk turbo diesel) | Terbesar dan paling berbeda dari mesin bensin; dikerjakan terakhir setelah fondasinya siap. |
| 5 | **Mesin 2-tak** | tipe mesin baru: kompresi karter (reed / port piston), port transfer & buang yang dibuka piston, scavenging, oli campur, knalpot expansion chamber (tuned pipe), four-stroking | **XL** | Kelompok A (gas, Woschni, pembakaran), model misfire umpan balik residu (sesudah Kelompok B), akustik Kelompok B | Diminta 2026-11-20. Sama besar dengan diesel dan sama-sama tipe mesin baru; urutan terhadap diesel bisa ditukar. |

Tiap paket ditutup dengan tes sendiri dan regresi penuh, seperti fase-fase sebelumnya.

**Paket 1 – Bahan bakar dan flex fuel (detail):** ✅ selesai — lihat "Paket 1 – implementasi" di bagian 4.
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

**Paket 2 – Sensor knalpot (detail):** ✅ selesai — lihat "Paket 2 – implementasi" di bagian 4.
- **Sensor EMAP (backpressure)** untuk mesin turbo: tekanan manifold sebelum turbin (node `turbineIn` sudah
  dihitung fisika), rasio drive pressure (EMAP ÷ boost), channel log, aturan proteksi backpressure tinggi.
- **Wideband membaca gas di pipa**: λ dihitung dari komposisi gas di collector / downpipe (bahan bakar, gas
  terbakar, udara) — misfire dan campuran yang lolos saat overlap ikut terbaca seperti sensor sungguhan. Sekarang
  λ = rata-rata silinder + jeda transport. Turbo: wideband setelah turbin.
- **Posisi EGT**: opsional EGT per silinder (header), dan EGT sebelum / sesudah turbin pada mesin turbo. Sekarang
  satu termokopel di collector (= sebelum turbin pada turbo).

**Paket 3 – Peta aliran dan kesehatan mesin, Live Dashboard › Flow map (detail):** ✅ selesai — lihat "Paket 3 –
implementasi" di bagian 4.
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

**Catatan base map dan cache (dicek 2026-11-20; diperbaiki 2026-11-21, versi 34):**
- **Karburator tidak lagi diukur** ✅ (2026-11-21): `baseMapApplies()` (BaseMap.h) — diesel dan karburator dilewati
  (app: pesan "carburettor: no base map needed"; `makeCalibratedPreset` mengembalikan peta generik). ECU karburator
  hanya modul pengapian: peta VE tidak dipakai untuk bahan bakar (`Ecu::injectionPulse` hanya untuk EFI; karburator
  menakar dari aliran venturi, Intake.cpp); satu-satunya pemakai lain adalah estimasi torsi torque management
  (`airEstimate`) — di mesin karburator memakai peta generik. Ganti ke EFI → kunci berubah → diukur.
- **Kolom rpm diukur paralel** ✅ (2026-11-21): tiap kolom mesin sendiri (kPatm per thread), thread = inti − 2 (sisakan
  untuk simulasi / audio dan UI), pemanasan 1 s gas penuh per kolom (keadaan termal seperti urutan lama). Di CPU 16
  thread: preset 1 22 → 6 s, preset 0 25 → 5 s, preset 17 47 → 17 s. Beda peta VE rata-rata 0.7–1.5 %; selisih
  terbesar (preset 0, 9100 rpm 37 kPa, +21 %) justru kesalahan cara lama: titik gas tertutup langsung setelah gas
  penuh kolom sebelumnya belum tenang (MAP 32.7 kPa > 15.4 kPa pada gas 2 %; kini 12.7 kPa, monoton).
- **Pipa tala 2-tak (Duct)**: dulu ±0.6 s per detik simulasi; kini ±0.1 s (limiter MC, sel 40 mm, ±48 kHz — lihat
  Paket 5, "Kecepatan"). Preset 25 (karburator) tidak memakai base map.
- **Cache** (`basemaps/` di samping exe; tes tidak memakai cache kecuali `BASEMAP_CACHE` di-set):
  - kunci = hash FNV-1a 64-bit atas seluruh `EngineParams` (bagian yang tidak menentukan udara dinolkan) XOR
    `kBaseMapVersion`. Parameter baru (mis. `resonator_hz`, `muffler_ext_tubes`, jenis bahan bakar) otomatis ikut;
  - kalibrasi sebelum `stftGain` (target λ, peta pengapian, ...) ikut kunci: hasil ukur memang bergantung padanya, jadi
    mengedit peta tersebut berarti cache tidak dipakai lagi (pengukuran ulang saat mesin dimuat berikutnya);
  - file terpotong (aplikasi mati saat menulis) ditolak oleh cek ukuran `fread`; file lama tidak pernah dihapus (±2 KB
    per mesin);
  - **versi wajib dinaikkan tiap fisika yang ikut diukur berubah** — terlewat sesudah Kelompok B dan perubahan misfire
    (peta v29 tetap dipakai); diperbaiki: versi 30.

**Paket 4 – Diesel (detail):** tahap 4a, 4b dan 4c selesai, plus tunda Assanis dan injeksi pilot (lihat bagian 4). Rujukan dicek identitas dan abstraknya; persamaan dan konstanta dibaca
dari sumber aslinya saat implementasi (seperti Kelompok A).
- **Bahan bakar:** solar sebagai data bahan bakar (paket 1): AFR stoik, LHV, densitas, angka setana (bukan oktan).
- **Penyalaan:** tanpa busi; tunda penyalaan (ignition delay) dari kondisi silinder dan angka setana — korelasi
  Hardenberg & Hase (SAE 790493: rumus empiris dari banyak mesin DI, masukan kecepatan piston, rpm, angka setana; ada
  kritik bahwa daya prediksinya terbatas, jadi dikalibrasi seperti D–E di A3). Integral tunda (Livengood–Wu) dari A3
  bisa dipakai ulang.
- **Pembakaran:** dua fase — premixed (cepat, sesudah tunda) dan difusi (mengikuti laju injeksi) — sebagai jumlah dua
  fungsi Wiebe (Watson, Pilley & Marzouk, SAE 800029; Ghojel SAE 821034; Miyamoto dkk. SAE 850107). Modul pembakaran
  "bisa diganti" dari A4 menjadi tempatnya. Opsi lanjutan: model spray multizone Hiroyasu (Bull. JSME 26, 1983;
  pengembangan SAE 1999-01-2926) untuk jelaga dan NOx — besar, nanti.
- **Injeksi langsung:** pulse / laju injeksi, timing injeksi (pengganti timing busi di ECU), pilot / main / post, tekanan
  rail (common rail). Beban = jumlah bahan bakar (tanpa throttle; throttle hanya untuk EGR / mematikan mesin).
- **Turbo VGT + EGR:** model mean-value Wahlström & Eriksson (Linköping; Proc. IMechE Part D 2011,
  DOI 10.1177/0954407011398177) — VGT, EGR, smoke limiter (bahan bakar maksimum dari aliran udara dan λ minimum) dan
  pembatas kecepatan turbo; kontrol PID EGR/VGT (IFAC 2005). EMAP dari paket 2 dipakai.
- **Busi pijar:** start dingin (pemanasan, waktu tunggu); proteksi dan diagnosis.
- **Suara:** "klotok" diesel dari laju kenaikan tekanan pembakaran premixed — indikator CPI dan laju kenaikan tekanan
  orde dua (studi regresi 2017), kualitas suara dari komponen tekanan silinder (Meccanica 2009), metode numerik sumber
  bunyi pembakaran (Torregrosa, Broatch & Gil, Applied Acoustics 2018). Model getaran blok yang sudah ada digerakkan oleh
  dp/dθ pembakaran.
- **Tes:** tunda penyalaan vs angka setana, bentuk laju panas dua fase, smoke limit membatasi λ saat akselerasi, VGT
  menahan boost, start dingin dengan / tanpa busi pijar, regresi penuh.

**Paket 5 – Mesin 2-tak (detail):** tahap 5a + 5b + pengisian selesai (lihat bagian 4, "Paket 5"); diminta 2026-11-20. Rujukan dicek identitas / abstraknya.
- **Siklus dan port:** satu langkah kerja per putaran; port buang, transfer dan intake dibuka-tutup piston (luas port
  dari posisi piston), intake lewat reed valve atau port piston ke karter.
- **Kompresi karter:** karter sebagai volume gas yang dipompa piston (rasio kompresi karter), mengisi silinder lewat
  port transfer (model simulasi mesin 2-tak kompresi-karter: Benson dkk. 1975, dikutip di literatur scavenging).
- **Scavenging:** model dua zona Benson–Brandham — fase displacement, mixing dan short-circuit dengan dua parameter
  semi-empiris; dinilai layak bersama model Dang/Wallace dan Maekawa (SAE 970358; tinjauan Brunel). Hasil: efisiensi
  scavenging / trapping, campuran yang lolos langsung ke knalpot (HC, letupan).
- **Knalpot expansion chamber (tuned pipe):** kerucut divergen, perut, kerucut konvergen, stinger — dari waveguide yang
  sudah ada (kerucut sebagai deret potongan; validasi Kelompok B berlaku). Gelombang balik dari kerucut konvergen
  "menyumbat" port buang dan mendorong campuran kembali (MOC + monopol: laporan NASA 1982; perbandingan syarat batas
  SAE 950985; model quasi-1D dikalibrasi SAE 2011-01-1145).
- **Four-stroking / pembakaran tak teratur di beban ringan:** residu berlebih (SAE 830091) — model misfire umpan balik
  residu yang dibuat untuk "brap" rotari dipakai langsung.
- **Pelumasan campur (premix):** rasio oli dalam bahan bakar; efek kecil ke AFR / LHV; asap.
- **Suara:** orde pembakaran 1 per silinder per putaran, "ring-ding" dari tuned pipe; suara ditentukan hampir
  seluruhnya oleh aksi gelombang di pipa (tesis Adelaide tentang knalpot 2-tak).
- **Tes:** trapping efficiency vs rasio delivery terhadap model scavenging, efek tuned pipe pada torsi (pita rpm), four-
  stroking muncul di beban ringan, regresi penuh.

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

### Setelah Fase 7 – Preset ITB (alpha-N)
- Gejala: idle AFR ±11 (target 14.7) dengan misfire terus, mesin mati saat gas dibuka ke 30 % dari idle, dan di WOT
  rpm tinggi proteksi knock memotong throttle (sweep dyno 6500 rpm: AFR 33, torsi anjlok).
- Penyebab dan perbaikan:
  1. Pengayaan laju MAP (menangkap idle yang turun pada speed density) ikut aktif pada ITB: MAP berdenyut tiap
     langkah isap dan idle cam besar turun-naik, jadi bahan bakar +40 % hampir terus-menerus (λ 0.74). Tip-in
     menambah pengayaan akselerasi di atasnya → λ 0.4, mesin tenggelam. Sekarang hanya bila MAP memang sinyal
     beban (bukan alpha-N, bukan hybrid di TPS tinggi).
  2. Auto-tune VE mensyaratkan MAP stabil, padahal pada alpha-N MAP tidak dipakai - VE ITB tidak pernah belajar.
     Syarat MAP stabil kini hanya bila MAP ikut menentukan beban.
  3. Spark WOT 36° pada CR 12.5: knock control kehabisan retard (> 9°) → proteksi. `mechAdvance` 24 → 18 (30°
     WOT, seperti K20A); daya puncak sama (169 PS) karena sebelumnya knock control sudah memundurkan timing.
- Hasil (`egt_probe 11`): idle 554 °C AFR 14.70, 3000 rpm 30 % AFR 12.90, WOT 908 °C AFR 12.81, tanpa proteksi.
  Alat baru `op_trace <preset>`: urutan egt_probe dengan titik operasi, λ per silinder dan status proteksi.

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

### Paket 1 – implementasi bahan bakar dan flex fuel ✅
- **Data bahan bakar** (`src/sim/Fuel.*`): bensin (AFR 14.7, LHV 43.5 MJ/kg, HoV 350 kJ/kg, 0.745 kg/L), etanol
  (9.0, 26.9, 920, 0.789, RON 108.5 / MON 90), metanol (6.45, 20.0, 1100, 0.792, 108.7 / 88.6). Campuran dihitung
  dari fraksi volume: AFR, LHV dan HoV menurut massa; oktan alkohol non-linear (E10 +3 RON, E20 +6, E85 ±108);
  penguapan dingin alkohol memburuk kuadratik dengan kadar alkohol. Daftar di pompa: Regular 90, Premium 92 / 95 /
  98, Race 104, E10, E20, E85, E100, M100. LHV bensin turun 44 → 43.5 MJ/kg (nilai literatur); base map versi 25.
- **Tangki dan saluran**: 50 L, mulai 30 L. Pilih bahan bakar = tangki dikuras dan diisi; tombol Refuel menambah
  liter bahan bakar lain (campur menurut volume). Saluran + rail (±1 L) disirkulasi pompa lewat return line
  (120 L/jam), jadi bahan bakar baru sampai ke injektor dalam ±30 detik.
- **Efek fisika** (konstanta `kStoich` / `kLHV` dihapus): AFR stoik dan LHV dari bahan bakar di saluran (λ, energi,
  afterfire, beban termal), densitas (injektor dan jet karburator mengalirkan volume — massa ikut densitas;
  karburator: massa ∝ √densitas), pendinginan muatan saat menguap (40 % panas laten dari udara: bensin ±9 K, E85
  ±25 K, metanol ±60 K → VE naik, knock turun), kecepatan api (alkohol lebih cepat), penguapan dingin di film port
  dan di silinder.
- **Knock dari oktan**: ambang suhu end-gas = 940 K + 10 K per angka oktan indeks di atas premium 95
  (±1.2° spark knock-limited per ON). OI = RON − K·(RON − MON), K dari +0.35 (NA) turun ke −0.35 di boost tinggi.
  Premium 95 tetap persis seperti kalibrasi sebelumnya.
- **Sensor flex fuel** (`kSnsFlex`, frekuensi, lag 0.8 s, bisa diberi gangguan): kadar alkohol di saluran. ECU:
  AFR stoik dari campuran massa (kalibrasi `stoich_gasoline` / `stoich_alcohol` — metanol perlu 6.45 karena sensor
  tidak bisa membedakan), densitas untuk massa injektor, dan empat kurva atas kadar alkohol: trim bahan bakar,
  tambahan spark, pengayaan dingin, pengali ambang knock. Tanpa sensor / sensor rusak: `flex_fallback_pct`.
  Validasi kalibrasi memberi peringatan bila AFR stoik yang dipakai ECU tidak cocok dengan bahan bakar di tangki.
- **UI**: Fuel system (Engine Setup dan Fuel Tuning): pilihan bahan bakar, tangki, isi ulang, sensor flex dipasang;
  tab Fuel Tuning › Flex fuel. Channel log baru: target lambda, flex fuel %, AFR stoik ECU. File mesin menyimpan
  `fuel_type` dan `flex_sensor`.
- **Hasil** (`fuel_test`, K20A dengan kalibrasi premium):
  - regular 90 knock lebih banyak (5500 rpm: retard 5.9° vs 2.9°), race 104 tanpa retard;
  - E85 + sensor: λ tetap di target, torsi +4 %; E85 tanpa sensor: λ 1.27 (ECU yang belajar menarik ke 0.91);
  - metanol: dengan stoik alkohol 6.45 dan injektor 2.3× torsi +11 %, VE +5.6 % (pendinginan muatan); dengan
    injektor standar duty 126 % → lean;
  - isi 20 L E85 di atas 30 L premium → E34, sensor naik bertahap (4.6 % setelah 5 s, 33.5 % setelah 2 menit);
  - start dingin −10 °C: E85 / E100 tanpa pengayaan dingin flex tidak hidup, E85 dengan pengayaan hidup;
    pengayaan penuh justru membanjiri (film alkohol terus menyuplai setelah start), jadi default ±50 % untuk E85;
  - karburator V8 dengan jet bensin pada E85: λ 1.61.

### Paket 2 – implementasi sensor knalpot ✅
- **Wideband dari gas di pipa** (`Engine::lambdaPipe`): setiap aliran keluar katup buang dicatat komposisinya —
  udara bebas, bahan bakar tak terbakar, dan produk pembakaran yang membawa λ muatan siklus itu (campuran kaya
  meninggalkan CO / H₂ yang masih kekurangan oksigen; campuran lean: kelebihan udara jadi udara bebas). Penanda
  terpisah untuk produk kaya sempat dicoba, tetapi hilang di gas buang yang balik ke intake saat idle (EGR internal)
  sehingga sensor membaca terlalu lean. Gas itu bercampur di kolektor (atau
  rumah turbin + downpipe) dengan waktu tinggal = volume / aliran volume, lalu sensor menambah jeda transport dan
  lag-nya sendiri. λ = oksigen ÷ (bahan bakar × AFR stoik). Bahan bakar tak terbakar dihitung 0.6× (berdifusi lebih
  lambat dari O₂ melalui penghalang sensor) → misfire terbaca lean. Hasil (`exhaust_sensor_test`, K20A): beban
  parsial dan WOT kaya sama dengan silinder (±0.01), 50 % spark dipotong: sensor 0.97 sementara silinder 0.81.
  Port injection: campuran yang lolos saat overlap membawa bahan bakarnya sendiri, jadi terbaca seperti muatan.
- **Posisi wideband** (turbo): kolektor sebelum turbin atau downpipe sesudahnya (default) — jeda 22 vs 53 ms di
  3500 rpm, volume campur lebih besar. ECU tahu posisi sensornya: estimasi jeda auto-tune ikut lebih panjang.
- **ECU menahan adaptasi saat misfire**: deteksi misfire dari kecepatan crank (input `misfireCount`); 0.5 s setelah
  misfire STFT dan auto-tune VE tidak belajar (bahan bakar tak terbakar membuat sensor membaca lean).
- **Fouling busi diperlambat** (0.02 → 0.0035 per detik per 50 K di bawah 450 °C): karbon menumpuk dalam hitungan
  menit, bukan detik. Sebelumnya I4 turbo dengan idle sedikit kaya (λ 0.93 di detik-detik awal) mengotori busi
  dalam ±20 s, lalu misfire → sensor lean → ECU memperkaya → mati (idle hold 30 s).
- **EMAP** (`kSnsEMAP`, 10–710 kPa): tekanan manifold sebelum turbin (NA: rata-rata port buang). Rasio drive
  pressure = EMAP ÷ MAP (telemetri, log, UI). Aturan proteksi baru **High drive pressure** (peringatan 2.0, batas
  2.6, 1 s, aksi boost reduction; hanya turbo, sensor terpasang dan di bawah boost).
- **EGT**: termokopel per primary (opsional, `egt_per_cyl`) dengan model probe yang sama (konveksi + radiasi ke
  dinding); yang terpanas ikut aturan High EGT. Termokopel sesudah turbin (`kSnsEGT2`): gas turun sebesar kerja
  turbin (daya rata-rata ÷ aliran rata-rata × cp, aliran wastegate tercampur) — I4 turbo 5000 rpm: 71 K.
- **File / UI / log**: `emap_sensor`, `wideband_pos`, `egt_per_cyl`, `egt_post_turbo` di file mesin; bagian
  Exhaust sensors di Engine Setup › Exhaust (pemasangan + bacaan, bar EGT per silinder); channel log baru: EMAP,
  rasio drive pressure, EGT sesudah turbin, EGT silinder terpanas.
- **Catatan model (belum diubah)**: EGT naik untuk campuran kaya di model (silinder +30 % bahan bakar ±20 K lebih
  panas), padahal di mesin nyata campuran kaya mendinginkan gas buang. Penyebab: panas pembakaran kaya dihitung
  seperti semua udara terbakar sempurna (tanpa CO / disosiasi) dan durasi bakar memanjang. Tes per silinder memakai
  spark yang dimundurkan (silinder itu +47 K).
- **Catatan regresi**: start 26B (rotari 4-rotor race) peka terhadap urutan noise sensor - sensor baru yang menarik
  angka acak dari generator yang sama sempat membuatnya gagal start pada satu survey (IMP 0). Noise EGT per silinder
  kini hanya diambil bila sensornya terpasang; start 26B rapuh (cenderung banjir) sudah ada sebelumnya.

### Paket 3 – implementasi peta aliran dan kesehatan mesin ✅
- **Data aliran dari fisika** (`src/sim/FlowStats.h`, `Engine::flowStep`): untuk tiap bagian jalur gas — inlet
  (kompresor pada mesin boost), blow-off / bypass, throttle, katup intake, katup buang, turbin, wastegate, knalpot —
  aliran maju dan aliran mundur dicatat **terpisah** setiap langkah simulasi lalu dirata-rata ±0.15 s. Aliran yang
  bolak-balik dalam satu siklus jadi terlihat dua arah: reversion ke port, standoff di depan throttle, gas buang yang
  tertarik balik saat overlap, surge kompresor. Ditambah tekanan dan suhu tiap volume (udara luar, keluaran kompresor
  sebelum / sesudah intercooler, plenum, port intake, port buang, gas keluar silinder, kolektor, sebelum / sesudah
  turbin). Hanya pencatatan: tidak ada state fisika yang diubah.
- **Kesehatan** (`src/sim/Health.*`, tanpa UI sehingga bisa dites): tiap komponen dan bagian dalam mesin diberi level
  hijau / kuning / merah dengan alasan yang bisa dibaca. Ambangnya:
  - dari aturan proteksi kalibrasi (peringatan / batas, ditambah status aturan dari ECU): EGT (header, turbin),
    overboost (plenum), drive pressure (turbin, hanya di bawah boost), knock;
  - dari batas komponen (`ComponentModel`): beban inersia per part (piston, rod, crank — `partSafeRpm` baru; kuning
    85 %, merah 100 %), valve float (kuning 5 %, merah 50 % = ambang kerusakan);
  - dari fisika langsung: knock retard terhadap otoritas knock control (kuning 40 %, merah 90 % = kehabisan retard),
    busi (pre-ignition dari 850 °C, fouling kuning 15 % / merah 40 %, energi percikan < 1× kebutuhan), turbo (kuning
    95 % / merah 100 % kecepatan puncak, surge), intercooler heat soak (inti 20 / 40 K di atas udara luar), throttle
    dibatasi proteksi (merah) atau torque management (kuning).
  - Panel: check engine, knock, busi, sensor (rusak terdeteksi = merah; gangguan yang tidak terdeteksi ECU, mis.
    sensor macet = kuning), katup, rod, crank, piston, perbaikan.
- **Live Dashboard › Flow map** (`src/ui/FlowMap.cpp`):
  - skema dibangun dari konfigurasi: filter → kompresor turbo / supercharger (Roots / screw / sentrifugal, dengan belt
    ke crank) → intercooler → blow-off / bypass → throttle, karburator atau ITB per runner → plenum / air box → runner
    + fuel rail → silinder (rotari: rotor) → primary → kolektor → turbin + wastegate (poros ke kompresor) → katalis →
    muffler → ujung knalpot. Komponen yang tidak dipasang tidak digambar.
  - partikel: kecepatan mengikuti kecepatan gas (aliran massa ÷ densitas ÷ luas pipa dari diameter yang dikonfigurasi),
    warna mengikuti suhu gas (biru dingin → oranye panas), partikel berongga = gas yang bergerak mundur. Tebal pipa
    mengikuti tekanan. Silinder diwarnai knock retard-nya, titik busi dan tanda katup diwarnai keadaannya.
  - hover node / pipa → tekanan, suhu, aliran maju / mundur, kecepatan gas; kompresor: efisiensi, aliran terkoreksi,
    surge margin; turbin: rasio ekspansi, efisiensi, U/C, drive pressure. Klik → panel terkait (Boost Control ›
    Hardware, Engine Control › Throttle, Ignition Tuning › Knock control, Engine Setup › Exhaust / Components,
    Fuel Tuning › Fuel system, Sensor Configuration, Engine Protection). Tile Repair memperbaiki mesin yang rusak.
  - sensor knalpot (lambda, EGT, EGT2, EMAP, EGT per primary) selalu digambar di posisi pasangnya; **mode diagnosis**
    menggambar semua sensor di posisinya dengan bacaan ECU, yang diberi gangguan atau ditolak ECU berkedip merah;
    sensor di luar jalur gas (VSS, gigi, aki) didaftar di bawah peta.
  - gaya: tema aplikasi (gelap, aksen oranye); hijau / kuning / merah hanya untuk keadaan komponen.
  - opsi baris perintah baru `--tab KEY` (mis. `--section 9 --tab flowmap`) dan `--start` (menyalakan mesin), dipakai
    untuk tangkapan layar otomatis.
- **Hasil `flow_test`**:
  - K20A 3000 rpm gas 10 %: throttle 17.9 g/s + bahan bakar 1.2 = katup intake 19.3 g/s, katup buang 19.0, plenum 38 kPa;
    WOT 5500 rpm: 109.1 + 8.4 = 117.5 g/s;
  - ITB 2500 rpm WOT (overlap besar): reversion 6.7 dari 44.8 g/s, standoff 22.9 g/s mundur lewat stack (58.0 maju),
    gas buang tertarik balik 4.4 g/s;
  - I4 turbo 4500 rpm WOT (1.10 bar): kompresor = throttle 183 g/s, keluar kompresor 128 °C → sesudah intercooler 62 °C,
    EMAP 222 → downpipe 104 kPa, gas 1026 → 956 °C melewati turbin; lepas gas: blow-off membuang 86 g/s; tanpa
    blow-off: surge 20 g/s mundur dan kompresor merah;
  - level kesehatan per ambang (knock, busi, EGT, sensor, proteksi, throttle, rod / piston / crank, intercooler, turbo,
    drive pressure, rotari tanpa katup / rod): semua lolos.
- **Catatan model (belum diubah)**:
  - pipa knalpot adalah waveguide akustik: fluks massa rata-ratanya mengikuti densitas pipa masing-masing, jadi tidak
    terbawa persis antar pipa dengan suhu berbeda — pada I4 turbo turbin + wastegate melihat ±13 % lebih banyak gas
    daripada yang keluar katup buang (224 vs 198 g/s). Peta aliran menampilkan apa adanya;
  - K20A sempat knock di beban ringan (sudah ada sebelum paket 1–3, dicek pada commit 7355803) — diperbaiki, lihat di bawah.

### Setelah Paket 3 – Knock di beban ringan
- Gejala: K20A dengan kalibrasi bawaan knock di beban ringan — 3000 rpm MAP 34 kPa indeks 1.0–1.4 dan retard 2–6°;
  4500 / 6000 rpm gas 15 % sampai retard penuh 10°. Lampu Knock di header dan panel kesehatan ikut menyala saat cruise.
- Penyebab: kriteria knock hanya membandingkan suhu end gas (pemampatan isentropik dari kondisi IVC) dengan ambang
  ±940 K; tekanan baru diperhitungkan di atas 90 bar (boost). Di beban ringan residu panas dan advance besar membuat end
  gas justru paling panas di seluruh peta (1060–1130 K), tetapi hanya pada 12–24 bar. Di mesin nyata waktu tunda
  autoignition naik tajam saat tekanan turun (Douaud–Eyzat, SAE 780080: τ ∝ p^-1.7 · exp(3800 / T)), jadi beban ringan
  tidak knock.
- Perbaikan (`knockEquivalentK`, `src/sim/Fuel.h`): di bawah tekanan end gas beban penuh mesin NA (60 bar — K20A WOT
  58–75 bar), end gas dinilai dengan suhu yang memberi waktu tunda sama pada 60 bar: 1/T_eq = 1/T + (1.7 / 3800) ·
  ln(60 / p). Di atas 60 bar tidak berubah (kalibrasi beban penuh dan suku tekanan boost tetap). Base map versi 26.
  Referensi 50 bar dicoba dulu: siklus gas 15 % dengan advance 30–42° masih mencapai 47–52 bar dan tetap knock.
- Hasil (`knock_probe 0`, alat baru: suhu / tekanan end gas, indeks dan retard per titik operasi):
  - MAP sampai 58 kPa: tanpa retard di 2000–6000 rpm (sebelumnya 1.5–10°);
  - WOT: retard 1.8–4.4° (sebelumnya 2.6–4.0°); 6000 rpm MAP 71 kPa: 5.4° (sebelumnya 7–9°);
  - I4 turbo: end gas 670–900 K di bawah boost, tidak knock (sama seperti sebelumnya);
  - `fuel_test` beban penuh tidak berubah (5500 rpm: regular 5.9° vs premium 2.9°), `control_test`: peta +12° tetap
    knock (indeks 1.87) dan knock control menurunkannya (1.03, retard 6.3°). `flow_test` kini juga memeriksa knock /
    kepala silinder hijau saat cruise. Regresi penuh lolos.
- Diganti oleh integral Livengood–Wu di Kelompok A3 (di bawah); `knockEquivalentK` dihapus. Hasil beban ringan tetap.

### Setelah Paket 3 – Presisi fisika, Kelompok A (A1–A4)
Rujukan dicek ke sumbernya sebelum dipakai; konstanta yang tidak bisa diverifikasi tidak dipakai.
- **A1 – Sifat gas tergantung suhu dan komposisi** (`src/sim/Thermo.*`):
  - data spesies N2, O2, CO2, H2O, CO, H2: polinomial NASA-7 GRI-Mech 3.0 (salinan Cantera `data/gri30.yaml`). Dicek:
    entalpi pembentukan CO2 −393.51, H2O −241.82, CO −110.53 kJ/mol; cp kontinu di 1000 K; LHV CO 10.10 MJ/kg dan H2
    120.0 MJ/kg; γ udara 1.399 (300 K) → 1.334 (1000 K);
  - isi silinder = udara (+ uap bahan bakar, dihitung sebagai udara) dan gas terbakar dengan komposisi yang dilacak
    (residu bercampur produk baru). Energi dalam sensibel u(T), suhu dicari dari U (tabel 2 K, monoton);
  - pembakaran: semua bahan bakar yang menguap ikut terbakar. λ ≥ 1 sempurna (udara lebih tetap udara); λ < 1 seluruh
    oksigen terpakai dan sisa CO / H2 mengikuti water-gas shift K = [CO][H2O]/([CO2][H2]) = 3.5 (beku ±1740 K; dari data
    GRI dihitung 3.58 di 1740 K). Energi kimia CO / H2 tidak dilepas, jadi campuran kaya lebih dingin. Disosiasi belum;
  - komposisi atom bahan bakar: bensin CH2.0 (AFR stoik 14.70, sama dengan model), etanol C2H6O, metanol CH4O;
  - efisiensi pembakaran 0.92 → 0.97: 0.92 lama ikut menanggung kerugian campuran kaya yang kini dihitung sendiri; Heywood
    memberi 95–98 % untuk λ ≥ 1 (sisa: HC dari crevice / quench);
  - aliran entalpi di katup, kebocoran apex rotari, dan inisialisasi silinder memakai h(T) / u(T) yang sama.
- **A2 – Woschni lengkap**: w = C1·Sp + C2·(Vd·Tr)/(pr·Vr)·(p − pm); C1 6.18 saat katup terbuka, 2.28 saat tertutup;
  C2 3.24·10⁻³ m/(s·K) dari pembakaran sampai katup buang membuka; pm tekanan motoring dari IVC (n = 1.32).
  Sebelumnya: w = 2.28·Sp + 3 m/s saat terbakar.
- **A3 – Knock Livengood–Wu**: τ = s·17.68 ms·(ON/100)^3.402·(p/atm)^−1.7·exp(3800/T) (Douaud–Eyzat, SAE 780080),
  knock saat ∫dt/τ = 1. ON = indeks oktan Kalghatgi seperti sebelumnya. End gas dimampatkan isentropik dari IVC dengan
  cp variabel (entropi udara NASA), ditambah suku busi panas dan coolant panas yang lama. Indeks ke sensor / ECU:
  1 + ln(I)/0.377 (1 = mulai knock; skala sama dengan indeks suhu lama). Integral dibuka di IVC dan ditutup saat 90 %
  terbakar (atau katup buang membuka bila tidak terbakar). Sebelumnya indeks dihitung tepat di TMA, sehingga end gas
  sesudah TMA (puncak tekanan) tidak pernah dinilai. Suku tekanan boost ad hoc (1.3e-5·(p − 90 bar)) dihapus: D–E sudah
  memuat p^−1.7.
  - Prefaktor dikalibrasi ke model (praktik umum: konstanta D–E disetel ulang per mesin): s = 3.2. Dengan 17.68 ms
    murni, K20A kalibrasi bawaan bensin 95 knock berat di seluruh WOT (indeks ±3, retard 10°) karena end gas model
    isentropik (tanpa rugi panas ke dinding, tanpa suku pengenceran residu — D–E diketahui gagal dengan EGR; suku EGR
    Hoepke dkk. SAE 2012-01-0707 tidak bisa diverifikasi, jadi tidak dipakai). s = 3.2 menaruhnya kembali di batas
    knock, seperti kalibrasi sebelumnya.
- **Hasil** (`knock_probe 0`, `dyno_sweep`, `egt_probe`, `fuel_test`):
  - K20A beban parsial (MAP ≤ 60 kPa): tanpa retard di 2000–6000 rpm; WOT retard 3.4–5.5°;
  - oktan: retard regular / premium / race 7.3 / 4.3 / 0° (3000 rpm) dan 5.8 / 1.5 / 0° (5500 rpm);
  - torsi puncak K20A 240 → 187 Nm, I4 turbo 487 → 373 Nm. Efisiensi brake K20A WOT dulu ±37 % (BMEP 15.1 bar pada
    λ 0.86 — terlalu tinggi; K20A asli 206 Nm / BMEP 12.9 bar, ±30 %) karena γ tetap 1.35 untuk gas panas. Preset belum
    dikalibrasi ulang (menunggu A4);
  - EGT WOT K20A 834 → 858 °C, I4 turbo (λ 0.80) 960 → 914 °C: campuran kaya kini mendinginkan;
  - base map versi 27 (A1), 28 (A2, A3). Regresi penuh lolos.
- **A4 – Pembakaran "hybrid"** (`src/sim/Flame.h`, `Engine::burnDuration`): bentuk Wiebe tetap, durasinya dari fisika
  nyala. Model entrainment penuh (Blizard–Keck / Tabaczynski, persamaan dari model LUSIE, Int. J. Engine Research 2018)
  tidak dipakai: ia butuh data turbulensi dari CFD dan konstanta yang menurut paper itu sendiri harus disetel per mesin.
  - kecepatan nyala laminar Metghalchi & Keck 1982 (Combustion and Flame 48, Tabel 3A, dibaca dari teks paper):
    S_u = S_u0 (Tu/298)^α (p/atm)^β (1 − 2.1 f), S_u0 = B_M + B_2 (φ − φ_M)², α = 2.18 − 0.8(φ − 1),
    β = −0.16 + 0.22(φ − 1); indolene (bensin): φ_M 1.13, B_M 27.58, B_2 −78.34 cm/s. Masukan ditahan di rentang yang
    diukur (φ 0.8–1.5, f ≤ 0.2, Tu ≤ 700 K, p 0.4–50 atm): tidak diekstrapolasi. Di luar itu batas kurus dan
    pengenceran tetap urusan model misfire. (Ekstrapolasi sempat dicoba: V8 karburator dan rotari bridge-port tidak
    hidup, 6 cek regresi gagal.)
  - rasio antar bahan bakar dari satu studi (Gülder 1982: puncak 0.455 / 0.47 / 0.50 m/s iso-oktana / etanol / metanol);
    data M–K sendiri memberi metanol 1.34× bensin, tidak sejalan dengan Gülder dan data modern. Menggantikan faktor
    nyala paket 1 (etanol 0.90, metanol 0.86 — perkiraan);
  - durasi = dasar(rpm, skala turbulensi lama × 1.12) × [0.28 r + 0.72 √r (α/α_ref)^¼], r = S_L,ref / S_L. Kernel
    (0–10 % Wiebe) tumbuh laminar ∝ 1/S_L; nyala turbulen Zimont u_t ∝ u′ Da^¼ ∝ S_L^½ α^−¼ (tebal nyala δ = α/S_L;
    difusivitas α ∝ T^1.75 / p). Referensi: K20A 4500 rpm WOT saat spark (752 K, 18.3 bar). Varian dengan ν juga di
    kernel (λ_T ∝ ν^½) dicoba: beban ringan / WOT ±2× (terlalu kuat; mesin nyata ±1.5×); tanpa α: ±1.15× (terlalu lemah).
    Faktor buatan lama dihapus: λ kuadrat, residu ×(1 + 2.2 f), beban rendah ×1.45, faktor nyala bahan bakar;
  - hasil K20A: durasi WOT 53–73°, beban ringan 74–101° (±1.4–1.5×); beban parsial tetap tanpa knock; torsi puncak 184 Nm.
- **Tuning idle sesudah A4**: pembakaran idle kini lebih cepat dari faktor empiris lama (CA50 ±20–29° ATDC, dulu 36–44°),
  sehingga mesin overlap besar menyala lebih keras di antara misfire pengenceran dan idle lebih kasar (CoV rata-rata
  35 → 51 %). Seperti kalibrator: rpm idle sedikit naik (MAP naik, residu turun) dan / atau advance idle turun (base
  advance −X, mekanis +X: timing WOT putaran atas tetap). Dipilih dari grid idle (+0 / 100 / 200 rpm × 0 / 4 / 8°)
  sebagai perubahan terkecil yang idle setidaknya sehalus sebelumnya (tabel `kRetune` di `Presets.cpp`):
  I3, flat-4 −4°; V8 flatplane +100 rpm −4°; V10, V12, flat-6, I6 turbo besar +100 rpm −8°; V8 Roots +200 rpm; 20B
  +200 rpm −8°; I4 turbo +100 rpm. Thumper: pompa akselerator 1.8 → 1.3 (satu tekanan penuh = ±3× bahan bakar satu
  siklus WOT: λ ±0.5, misfire kaya dan mati saat tip-in).
  - `idle_probe` (CoV IMEP, model lama → sekarang): V12 34 → 13 %, V10 52 → 46 %, 20B 104 → 99 %, V8 Roots 33 → 28 %,
    I6 8 → 5 %, V6 8 → 5 %, I5 8 → 5 %, K20A 3.1 → 2.7 %; rotari bridge-port yang dulu mati kini idle ±1260 rpm.
    ITB race (±100 %) dan 26B (135 → 186 %) tidak membaik dengan rpm / timing idle — 26B sudah tercatat rapuh.
  - base map versi 29.

### Setelah Paket 3 – Akustik knalpot, Kelompok B
Rumus dicek ke sumbernya; tiap bagian diuji terhadap teorinya di `sound_test`.
- **B5 – Ujung pipa terbuka** (`OpenEnd`, `Waveguide.*`): dulu −0.985 × lowpass 3500 Hz yang sama untuk semua diameter,
  tanpa koreksi panjang. Sekarang model kausal Silva, Guillemain, Kergomard, Mallaroni & Norris (arXiv 0811.3625, eq. 16,
  tabel 1) untuk pipa tanpa flange: R = −(1 + n1 τs)/(1 + d1 τs + d2 τ²s²), τ = a/c, n1 0.167, d1 1.393, d2 0.457 —
  |R| = 1 − (ka)²/2 dan koreksi ujung 0.6133 a (Levine & Schwinger) di frekuensi rendah, pipa besar memancarkan lebih
  banyak nada tinggi. Biquad per ujung (bilinear), dihitung ulang saat jari-jari / suhu gas berubah. Faktor 0.985
  tetap: rugi aliran rata-rata (vortex shedding) yang tidak ada di teori tanpa aliran.
  Uji (pipa 60 mm, gas 700 K): |R| dalam 3.5 % dari Levine & Schwinger sampai ka = 2, koreksi ujung dalam 8.4 % (paper:
  ≤ 8 % untuk model kausal ini).
- **B6 – Muffler dan resonator**:
  - ruang ekspansi yang sudah ada diuji terhadap TL = 10 log[1 + ¼(m − 1/m)² sin² kL]: selisih 0.07 dB (m = 6.5) —
    model muffler lama sudah benar untuk gelombang bidang;
  - **resonator Helmholtz** (pilihan Resonator: None / Inline chamber / Helmholtz): leher (½ luas pipa, 0.25 m) ke volume
    tertutup di samping mid pipe, disetel ke `resonator_hz` (0 = otomatis: frekuensi pembakaran di 2000 rpm, "drone"
    cruise). Uji: resonansi 149 Hz untuk target 150 Hz, TL dalam 0.96 dB dari teori lumped (0.5–2 f_r); di mesin
    berjalan (K20A, 2000 rpm, gas 25 %) orde pembakaran turun 27 dB (resonator inline lama −0.2 dB);
  - **tabung extended** pada muffler chambered (`muffler_ext_tubes`): tabung masuk L/2 dan keluar L/4 di dalam tiap
    ruang, ruang anulus di sekitarnya = resonator seperempat gelombang tertutup (Munjal). Uji: cabang samping tertutup
    sesuai TL = 10 log[1 + (S_b/2S)² tan² kl] (0.04 dB); lembah TL ruang di kL = π dan 2π naik dari 0 ke 42 / 37 dB;
  - UI Engine Setup › Exhaust, file mesin, skema knalpot ("HELM").
- **B7 – Alat pembanding** `tools/compare_wav.py a.wav b.wav t0 t1 rpm cyl`: level, pita 1/3 oktaf dan level per orde
  mesin dari dua render. K20A sebelum / sesudah Kelompok B: idle berubah ≤ 1.2 dB (sedikit turun di 300 Hz–1.3 kHz).
- **Temuan (belum diubah)**: volume gas model (port, plenum, kolektor, turbin) isotermal — p = mRT/V dengan T tetap —
  jadi kekakuan akustiknya RT, bukan γRT: resonansi Helmholtz plenum / port ±14 % (√γ) terlalu rendah. Resonator baru
  diukur untuk kekakuan yang sebenarnya dipakai node sehingga beresonansi tepat di frekuensi tuning.
- **Regresi**: semua lolos kecuali `idle_hold` untuk dua rotari paling ekstrem — 26B (tidak hidup) dan bridge port (mati
  setelah ±3 s). Keduanya sudah rapuh sebelumnya (bridge port: ±55 % pembakaran misfire acak saat idle, residu 0.42–0.59);
  perubahan gelombang balik dari ujung pipa B5 cukup untuk menjatuhkannya. Dibereskan dengan misfire dilusi
  deterministik (bagian berikut).
  `ecu_test`: cek auto-tune kini memakai λ rata-rata 2 s (sama seperti cek closed loop di atasnya) — satu sampel di idle
  yang naik-turun sempat memberi 1.054; rata-ratanya 1.020 (toleransi 4 %).

### Setelah Kelompok B – "Brap" rotari: misfire dilusi deterministik
- **Masalah**: bridge port idle dengan ±55 % misfire *acak* (residu 0.42–0.59). Misfire karena dilusi dulu peluang yang naik
  linear dari residu 0.35 ke 0.60 dan diundi per siklus — hasilnya batuk acak, bukan irama "brap".
- **Model sekarang** (`Engine.cpp`, blok busi): ambang dengan sebaran kecil — misfire bila residu + 0.03·N(0,1) > 0.475
  (titik tengah ramp lama, jadi toleransi dilusi rata-rata tidak berubah). Siklus saling terkait lewat residu: misfire
  meninggalkan campuran segar (siklus berikutnya terbakar kuat), pembakaran kuat meninggalkan residu panas (berikutnya
  misfire). Umpan balik deterministik dengan sedikit noise ini mekanisme variasi siklus di dekat batas dilusi (Daw, Finney
  dkk., ORNL) dan sama dengan "four-stroking" mesin 2-tak (SAE 830091).
- **Hasil**: `idle_hold` 0 mati mesin (26B dan bridge port kembali hidup); CoV idle mesin cam besar turun banyak (V10 46 →
  9 %, flat-6 51 → 10 %, V8 flatplane 58 → 17 %); I3 / I6 / flat-4 naik 5 → 12–16 % tanpa misfire sama sekali (urutan
  noise lain). Metrik irama `tools/lope_wav.py` hanya naik sedikit (konsentrasi modulasi bridge port 0.20 → 0.24, 26B 0.22
  → 0.28) — iramanya lebih teratur, tapi belum "brap" yang tegas. Target idle bridge port tetap 1450 rpm (1050 mati).
  Base map versi 30.
- **Regresi**: dua cek sempat gagal — `control_test` (knock control) dan `phase7_test` (batas torsi dengan throttle kabel).
  Sapuan seed RNG (salinan sementara) menunjukkan keduanya noise pengukuran, bukan perubahan fisika: perubahan ini
  memakai RNG mesin di tiap busi, jadi urutan acak lainnya bergeser.
  - knock: rata-rata 2 s tepat setelah hold menangkap kontrol knock yang masih konvergen (1.38–1.77 antar seed, batas
    1.69) → rata-rata 6 s: 1.35–1.47 untuk tujuh seed (tanpa kontrol 2.12);
  - batas torsi: torsi kopling sesaat setelah kopling mengunci di gigi 1 berayun 20–180 Nm (shuffle driveline), dan
    rata-rata 0.6 s pemotongan busi acak terlalu sedikit sampelnya (72–98 Nm antar seed) → torsi mesin rata-rata 1.5 s:
    rasio 0.51–0.68 untuk tujuh seed (batas 0.8).
  Regresi penuh lolos.

### Setelah Kelompok B – "Brap" tahap 2: letupan kantong misfire, pembakaran parsial (Daw)
- **Diagnosis** (bridge port idle, 8 s, per kejadian buang): 60 % misfire, datang berderet (±4 Hz). Saat port buang
  terbuka: menyala 2.6 bar / 1090 K, misfire 0.9 bar / 363 K. Tapi RMS suara misfire tetap ±60 % siklus menyala
  (aliran yang didorong rotor, bukan sisa dengung: tetap sama 7 kejadian setelah nyala terakhir); kompresor di Listener
  bukan penyebabnya (rasio puncak 2.17 vs 2.10 tanpa kompresor). Campuran dari deretan misfire terbakar **diam-diam** di
  port begitu siklus menyala lagi (cabang "nyala terus-menerus" afterfire) — hanya 22 letupan dalam 8 s.
- **Letupan kantong** (`Cylinder::pocketFuel/pocketAir`, blok afterfire): campuran yang keluar dari muatan misfire / cut
  dicatat sebagai kantong premix. Blowdown panas berikutnya ke port yang sama (rotari: tiga muka rotor berbagi port)
  menyalakannya sekaligus → letupan, memakai rumus letupan yang sudah ada (syarat bisa terbakar λ < 1.7). Slip
  scavenging tetap terbakar sebagai nyala terus-menerus. Ini penalaran fisika; paper khusus afterfire rotari tidak
  ditemukan. Bridge port: 22 → ±200 letupan per 8 s.
- **Efisiensi pembakaran Daw–Finney — dicoba, lalu dikeluarkan** (keputusan 2026-11-20: "pops only" setelah membandingkan
  WAV sebelum / letupan saja / letupan + Daw). Yang dicoba (SAE 962086, teks lengkap OSTI 390645, pers. 7–9): misfire dilusi biner diganti
  C = 1/(1 + 100^((d − 0.475)/w)) dengan d = dilusi + 0.03·N(0,1); C < 0.1 dihitung misfire, di atasnya pembakaran
  parsial (bahan bakar terbakar = C × muatan; sisanya tinggal di muatan dan ikut residu — pers. 8–9 paper, di model ini
  otomatis lewat neraca massa). Paper memakai rasio ekuivalen sebagai masukan; dilusi sebagai masukan adalah perluasan
  model ini. w = 0.02 dipilih (4 % dari d_m; tepi paper 1–12 % dari φm), bukan diukur.
- **Hasil, jujur**: per ruang bakar siklus lemah diikuti siklus kuat — autokorelasi lag-6 (muka yang sama, siklus
  berikut) −0.64, sesuai Daw dkk. (−0.65 s/d −0.7). Tapi modulasi pelan 2–6 Hz (irama "brap") **tidak menguat**: dengan
  bobot A, kedalaman pita 2–6 Hz bridge port 0.43 → 0.29, 26B 0.26 → 0.18 (seed bawaan); antar seed hasilnya tersebar
  0.23–0.40, lebih besar dari beda antar pilihan w (0 / 0.02 / 0.05). Deretan misfire panjang berkurang (misfire setelah
  misfire 78 vs ±110), rpm lebih jarang berayun, dan modulasi pindah ke pola per kejadian yang teratur (±8 Hz).
  Kesimpulan: "brap" pelan yang lama sebagian besar adalah ayunan rpm (hunting) akibat deretan misfire panjang.
  Keputusan akhir 2026-11-20: setelah mendengar ketiga WAV, suara sebelum tahap 2 (`bp_new`) yang paling disukai —
  letupan kantong dan pembakaran parsial **dua-duanya dikeluarkan**; kode kembali ke tahap 1 (render idle bridge port
  identik byte-per-byte dengan `bp_new.wav`). Base map versi 33.
  Bila nanti irama "brap" mau dipertegas lagi, tuasnya ada di deretan misfire panjang dan ayunan rpm (kontrol idle),
  bukan di bentuk kurva pembakaran.
- Alat ukur scratch: probe per kejadian (RMS per transisi, autokorelasi bagian terbakar), analisis pita envelope dengan
  bobot A. Base map versi 31 (letupan + Daw), 32 (Daw dikeluarkan), 33 (letupan juga dikeluarkan).

### Target "brap" dari rekaman asli (2-rotor RX-7, idle; rekaman dari user, 67 s)
Diukur dengan cara yang sama untuk rekaman dan render (envelope RMS 1 ms, loudness maksimum per kejadian):
| | rekaman | sim bridge port (`bp_new`) |
|---|---|---|
| laju kejadian | 56–57 Hz (≈1700 rpm) | 41 Hz (≈1240 rpm) |
| irama envelope | 4.5–4.7 Hz, 78–85 % energi modulasi di 2–6 Hz | 40–46 % di 2–6 Hz |
| keras / pelan (q90/q10) | 3.8–6.8× | 2.4× |
| autokorelasi lag 1 / lag 12 | +0.64…+0.74 / +0.79…+0.85 | +0.01 / +0.05 |
| bentuk satu siklus (12 kejadian) | 6 keras rata (0 dB), lalu memudar −4, −6, −7, −9, −9, −8 dB, lalu langsung keras lagi | datar (−2 s/d −5 dB) |
Artinya: siklus relaksasi 12 kejadian — semua muka kedua rotor menyala 3 putaran poros, lalu melemah dan misfire 3
putaran; per ruang bakar nyala / misfire bergantian, dan keenam ruang bakar sefase. Sefase berarti ada keadaan
*bersama* yang menumpuk saat menyala dan pulih saat misfire. Temuan di model (probe per kejadian): pipa memang
membawa komposisinya sendiri (`cm/cmf/cmb` per pipa, `Waveguide.cpp`; `burned_` / `inflowSource` tidak dipakai — dugaan
awal saya salah). Keadaan bersama itu ada: runner intake tiap rotor berisi 30–41 % gas terbakar hasil reversi lewat
overlap, naik saat deretan nyala dan turun saat deretan misfire, rpm ikut berayun 980–1310. Yang merusak pengelompokan
"6 nyala / 6 misfire" adalah kopling antar-muka lewat node port buang kecil (40 cm³ = gas kejadian terakhir): sesudah
nyala, muka berikutnya menarik gas hampir 100 % terbakar (0.87–0.94) dan misfire; sesudah misfire menarik campuran segar
(0.50–0.58) dan menyala — pola `X.X` per rotor, sedangkan di rekaman muka berurutan sefase.
Eksperimen (salinan scratch, belum dipasang): volume port buang ×3 dan sebaran misfire 0.01 memunculkan siklus pelan
10 kejadian (±4.2 Hz, pita 2–6 Hz sampai 0.80 seperti rekaman), tapi bistabil — tergantung seed jatuh ke siklus pelan
atau ke selang-seling cepat ±17 Hz. Idle tidak bisa naik ke ±1700 rpm (MAP idle sudah 88–91 kPa).

**Dibuat robust (2026-11-20).** Log 1500–2500 kejadian per run (rotor, muka, nyala, dilusi, komposisi port / runner /
primary, rpm, MAP) + regresi dilusi tiap kejadian terhadap nyala 1–12 kejadian sebelumnya (R² 0.90–0.97):
- per rotor, nyala menaikkan dilusi muka ke-2/3/4 berikutnya (+0.07 / +0.09 / +0.04) plus keadaan runner bersama;
  urutan nyala/misfire per rotor ternyata **sudah stabil** (siklus 10 kejadian di semua seed, autokorelasi 0.83–0.94).
  Yang berubah antar seed adalah **fase antar rotor**: kedua rotor berlawanan fase (lag 1 −0.4…−0.6), sehingga nyala
  tersebar rata (`X..X.X..X.`) dan envelope bergetar ±17 Hz — bukan brap. Di rekaman kedua rotor sefase (lag 1 +0.68).
- yang tidak membuat sefase: batas dilusi 0.49–0.52 (porsi nyala terkunci 40 %), flywheel, tipe header, panjang
  primary 350–1600 mm, volume port intake / buang, plenum 1–9 L, ITB (mati).
- yang membuat sefase: **intake pendek bersama** — plenum 0.3–0.5 L dengan runner 50–100 mm. Gas yang di-revert satu
  rotor saat idle sampai ke rotor lain di langkah hisap berikutnya, jadi kedua rotor berbagi dilusi yang menumpuk
  pelan dan menyala / misfire bersama. Semua seed: lag 1 +0.35…+0.65, siklus 11–12 kejadian, nyala 45–47 %.
- **Dipasang**: preset bridge port (23) `plenum_L` 3.0 → 0.3, `runnerLen_mm` → 50 (satu throttle, ruang kecil bersama).
  Fisika tidak diubah (ambang dilusi 0.475 dan sebaran 0.03 tetap — dengan geometri ini 0.03 pun robust).
  Hasil (render project, bandingkan dengan rekaman; keduanya high-pass 80 Hz karena rekaman tidak punya frekuensi
  rendah):
  | | rekaman | sebelum (`bp_new`) | sekarang |
  |---|---|---|---|
  | pita 2–6 Hz | 0.85 | 0.58 | 0.83 |
  | keras / pelan | 5.1× | 4.8× | 3.9× |
  | lag 1 / siklus terkuat | +0.67 / 12 (+0.86) | +0.12 / 10 (+0.27) | +0.62 / 11 (+0.67) |
  | bentuk siklus | 6 keras, −4…−9 dB | datar | 5 keras, −2…−6 dB |
  | irama | 4.7 Hz | 3.7 Hz | 3.7–4.0 Hz |
- Sisa selisih: idle ±1250–1300 rpm (rekaman ±1700), jadi irama sedikit lebih pelan; katup idle sudah 100 % dan target
  1450 tidak tercapai. Dicoba "throttle stop" EFI (parameter baru, prototipe di scratch): 4 % → ±1600 rpm, irama
  4.7–4.8 Hz, tapi kontras turun ke 2.9× (misfire dengan muatan lebih besar lebih berisik). Belum dipasang.
- **Keputusan 2026-11-20**: setelah mendengar `brap_final` / `bp_new` / opsi idle tinggi — versi intake pendek
  bersama dipertahankan; throttle stop tidak dipasang.
(Spektrum rekaman puncak di 100–125 Hz dan turun tajam di atas 400 Hz, tapi rantai rekam ponsel / kamera memotong
frekuensi rendah — belum dipakai sebagai target.)

### Paket 4 – Diesel, tahap 4a: bahan bakar, injeksi langsung, tunda penyalaan, pembakaran, ECU ✅
Konstanta dicek ke sumber yang dibuka (teks lengkap / tabel), bukan dari ingatan:
- **Bahan bakar** (`Fuel.h/.cpp`): solar EN 590 (cetane 51) dan solar cetane 40 (minimum ASTM D975-24, kelas 1-D/2-D).
  LHV 42.79 MJ/kg dan 0.837 kg/L (tabel GREET 1.8d.1, U.S. conventional diesel); AFR 14.5 (MIT 2.61 kuliah 15;
  Colorado State C12.5H22.2 AFR 14.5 — komposisi itu dipakai untuk produk pembakaran); kalor penguapan 250 kJ/kg
  (rentang IEA-AMF 225–280); densitas EN 590 0.820–0.845. Solar tidak dicampur dengan bensin / alkohol.
- **Tunda penyalaan**: Hardenberg & Hase (SAE 790493) seperti Heywood pers. 10.37 (dicek di slide MIT 2.61 kuliah 16):
  τ (°CA) = (0.36 + 0.22 Sp) exp[E_A (1/(R T) − 1/17190) (21.2/(p − 12.4))^0.63], E_A = 618840/(CN + 25).
  Diintegralkan dt/τ atas keadaan muatan dari awal injeksi (seperti integral knock A3); di bawah 12.4 bar tidak
  menyala. Uji: 3.721° untuk 10 m/s, 900 K, 60 bar, CN 50 (hitungan tangan 3.72).
- **Pembakaran** (`Engine::dieselStep`): bentuk Watson tipe WAVE (Clean Energy 8(5):73, 2024, pers. 1–7, dicek oleh
  agen riset): premixed 1−[1−(0.75t)²]^5000, difusi 1−[1−(cd3 t)^1.75]^5000, ekor 1−[1−(ct3 t)^2.5]^5000,
  t = Δθ/125°, cd3 = 0.055/(1+0.5φ), ct3 = 3.7cd3/(1+1.12φ), porsi ekor 0.6φ² (φ ≤ 0.85). Faktor (rpm/BRPM)^0.3 tidak
  dipakai (BRPM tidak terverifikasi). Porsi premixed **bukan** korelasi Watson (SAE 800029 tidak bisa dibuka): bahan
  bakar yang sudah disuntikkan selama tunda (gambaran Heywood). Pembakaran tidak boleh mendahului injeksi. Muatan yang
  belum menyala saat katup buang terbuka keluar tidak terbakar (misfire).
- **Injeksi langsung**: kuantitas ECU (mg/langkah) dari awal injeksi (SOI) dengan laju injektor (mg/ms); penguapan
  mengambil kalor laten dari muatan. Tanpa throttle (intake selalu terbuka).
- **ECU diesel**: peta permintaan bahan bakar (rpm × pedal), governor idle (PI), governor kecepatan maksimum (bahan
  bakar → 0 di pita sebelum batas putaran), **pembatas asap** λ 1.38 (MIT 2.61 kuliah 16: "smoke-limit A/F ~ 20"),
  bahan bakar start selama starter jalan, peta SOI (rpm × bahan bakar; dibuat dari timing statis + speed advance
  seperti pompa mekanis). Mode beban MAF (diesel mengukur udaranya). Batas putaran = potong bahan bakar.
- **Starter**: torsi starter diskalakan dengan rasio kompresi (tekanan kompresi ~CR^1.3): starter diesel jauh lebih
  kuat — tanpa itu mesin 19.5:1 tidak bisa diputar.
- **Preset 24 "I4 1.9 turbo diesel (TDI-style)"**: 79.5 × 95.5 mm, CR 19.5, turbo wastegate 0.8 bar, 45 mg/langkah,
  SOI 2° + 5° speed advance. Timing / boost / bahan bakar adalah pilihan kalibrasi (bukan konstanta bersumber), diatur
  agar tekanan puncak ±105–150 bar: ±230 Nm di 2000 rpm, ±210 Nm di 4000 rpm, idle ±900 rpm, λ asap 1.31–1.46.
- **UI**: sistem bahan bakar "Diesel (direct injection)", slider laju injeksi / bahan bakar penuh / timing / speed
  advance, rasio kompresi sampai 25:1 untuk diesel; tab Engine Control › Diesel (pembatas asap, governor, peta
  permintaan dan SOI).
- **Tes** `diesel_test`: korelasi vs hitungan tangan, start hangat, idle dipegang governor, MAP idle ≈ ambien (tanpa
  throttle), kunci OFF = mati, torsi mengikuti kuantitas bahan bakar, pembatas asap aktif, cetane 40 menyala lebih
  lambat di idle (efek cetane di HH mengecil pada muatan panas: di beban penuh hanya beberapa persen).
- **Catatan / batas**:
  - pembatas asap mendarat di λ ±1.30–1.34 pada sebagian titik (target 1.38): estimasi udara ECU sedikit optimis;
  - HH tetap memprediksi penyalaan pada CR rendah (mis. 9.5:1 dengan ±20 bar di TDC) — batas korelasinya;
  - bensin / alkohol di mesin diesel di luar cakupan model (cetane 0 di luar rentang korelasi) — hanya diberi
    peringatan di UI;
  - **suara**: idle diesel masih terlalu "bersih" (orde pembakaran dominan, di atas 300 Hz 50–70 dB lebih rendah,
    tanpa klotok). Getaran blok ada, tapi dari belakang mobil dicampur ×0.05 lewat lowpass, dan kenaikan tekanan
    premixed di idle (±2.8°) hampir tidak berisi energi di atas 2 kHz. Dikerjakan di tahap suara (4c) bersama sumber
    combustion noise (ringing intensity Eng; kurva atenuasi struktur).
- Sisa Paket 4: **4b** turbo VGT + EGR (model Wahlström & Eriksson, persamaan sudah dicek di LiTH-ISY-R-2882),
  **4c** busi pijar / start dingin dan suara klotok diesel.

### Paket 4 – Diesel, tahap 4c: suara klotok dan busi pijar ✅
- **Diagnosis**: laju kenaikan tekanan puncak diesel 7.5 bar/° di idle, 14–18 bar/° beban penuh (K20A 0.4 / 1–2.6);
  eksitasinya ada, tapi resonator blok bereaksi lemah ke ramp ±0.5 ms dan ringing ruang bakar tidak ada di model 0-D.
  `Cylinder::dpMaxCycle` baru; puncak tekanan / laju kini diambil di jendela yang berpusat di TDC pembakaran (diesel
  bisa memuncak sebelum TDC).
- **Ringing** (`CombustionNoise.h`): amplitudo puncak-ke-puncak = β (dP/dt)maks, β = 0.05 ms (Eng SAE 2002-01-2859,
  seperti dipakai Dernotte, Dec & Ji, OSTI 1123537 pers. 2 — dibaca dari halaman yang dirender; ditetapkan untuk HCCI,
  di sini dipakai untuk pembakaran premixed diesel). Frekuensi mode tangensial pertama f = 1.841 c/(πB) (Draper; via
  US 10443535; contoh paten 4.7 kHz, model 4.78 kHz dengan γ 1.3). Dieksitasi saat laju kenaikan tekanan tertinggi
  sejak awal pembakaran turun ke separuhnya; peluruhan 1 ms **dipilih** (tidak bersumber). Hanya untuk suara.
- **Atenuasi struktur**: kurva AVL generik (OSTI 1123537 Gbr. 2, dibaca dari plot ±2 dB: −150 dB 200 Hz … −90 dB 2 kHz
  … −132 dB 10 kHz), difit 3× high-pass 600 Hz + 3× low-pass 4800 Hz (Q 0.85) + −93.25 dB: 1.1 dB rms (uji: selisih
  terbesar 2.3 dB). Tekanan semua silinder (+ ringing) → filter → tekanan bunyi (Pa) → kanal mekanis. **Hanya diesel**:
  mesin bensin tetap memakai jalur resonator blok lama (suaranya tidak berubah).
- **Hasil** (idle preset 24): di ruang mesin diesel 30–44 dB di atas K20A pada 500 Hz–1.3 kHz dan +15 dB di ±5 kHz
  (ringing), crest factor 6.4 (impulsif); dari belakang mobil +17…+27 dB (500–1260 Hz) dan +23 dB di 5 kHz dibanding
  4a, tapi level total tetap didominasi orde pembakaran knalpot (campuran mekanis dari belakang ×0.05 lewat lowpass
  2.5 kHz — tidak diubah).
- **Busi pijar**: tipe None / Metal / Ceramic (`glow_plugs`). Ujung: orde satu ke 1150 °C (metal) / 1300 °C
  (keramik) dengan konstanta waktu dari brosur Bosch (850 °C < 5 s, 1000 °C < 2 s). ECU: pre-heat sejak kunci ON
  menurut suhu pendingin (BorgWarner/BERU: 0 °C 2 s, −5 °C ±5 s, −10 °C ±7 s; lebih dingin tetap 7 s — tidak di sumber;
  2 s sampai 20 °C, 0 s mesin hangat), lampu glow menyala selama pre-heat (lampu "Glow" di bar atas menggantikan
  "Knock" untuk diesel), memanas selama cranking, post-heat sampai pendingin 70 °C atau maks 180 s (BERU).
- **Start dingin**: korelasi HH hampir tidak peka suhu (E_A/R ±980 K — tunda 1.5° bahkan pada 600 K), dan model tidak
  punya blow-by ring (paten Bosch US 7900599 menyebut blow-by + rugi panas di kecepatan starter sebagai penyebab
  kompresi dingin rendah): suhu akhir kompresi saat cranking −20 °C masih 807 K. Karena itu batas penyalaan tanpa
  bantuan **dikalibrasi ke perilaku yang bersumber**, bukan diukur: integral tunda hanya berjalan bila muatan ≥ 815 K
  atau ujung busi pijar ≥ 850 °C (Bosch). 815 K dipilih agar preset start tanpa busi sampai −7.5 °C dan gagal di
  −10 °C (Bosch: DI start tanpa busi di atas −5 °C — brosur; di atas −10 °C — paten).
- **Torsi starter** kini ikut rasio kompresi (sudah di 4a); flag diesel juga di-set di konstruktor Engine.
- **Tes** (`diesel_test`): filter vs kurva AVL (≤3 dB), frekuensi resonansi vs contoh paten, amplitudo ringing di
  idle vs relasi Eng (0.90 vs 1.01 bar), +10 °C tanpa busi start, −15 °C tanpa busi tidak start, −15 °C dengan busi
  start setelah lampu padam, pre-heat −5 °C ±5 s. (Perbaikan tes: bagian beban kini memakai salinan parameter — hold
  dyno sebelumnya terbawa ke bagian cetane / ringing.)
- **Batas**: β dari HCCI; peluruhan ringing dipilih; batas start dingin dikalibrasi; post-heat belum memendekkan tunda
  (BERU: post-heat mengurangi asap putih dan ketukan start dingin — di model hanya menjamin penyalaan).

### Paket 4 – Diesel, tahap 4b: turbo VGT dan EGR ✅
Bentuk persamaan dari Wahlström & Eriksson (LiTH-ISY-R-2882, laporan teknis yang diperluas di Proc. IMechE D 2011;
persamaan dicek oleh agen riset); konstanta hasil fit mereka milik mesin mereka (Scania) — tidak dipakai.
- **VGT** (`vgt`, `vgtMinArea`; `Boost.cpp`): tanpa wastegate; luas aliran turbin = luas terbuka (dari A/R) × f_vgt,
  f_vgt = c_f2 + c_f1 √(1 − ((u − c_vgt2)/c_vgt1)²) (busur elips, pers. 53–55) dipusatkan di ujung terbuka
  (c_vgt1 = c_vgt2 = 1), dari `vgtMinArea` (tertutup) sampai 1. Aktuator orde satu 0.15 s (dipilih). Efisiensi turbin
  tetap model yang ada (bukan BSR pers. 48 — belum).
- **ECU VGT**: PID boost (gain kalibrasi wastegate) dari titik setengah tertutup, integral tanpa wind-up; duty =
  menutup vane; batas duty 1.0 (vane boleh menutup penuh). VGT selalu dikontrol ECU. **Diesel: target boost dibaca
  di atas pedal** (tanpa throttle), naik dengan beban, penuh dari ±1800 rpm (titik awal, bukan bersumber).
- **EGR** (`egr_valve_dia_mm`; `Engine.cpp`): katup dari manifold buang (sebelum turbin) ke plenum. Aliran orifis
  kompresibel model yang sama dengan port lain; satu arah (W&E pers. 31–34 menjepit rasio tekanan di 1 → tanpa aliran
  balik); gas membawa fraksi terbakarnya (mengencerkan muatan, menggantikan udara segar yang tidak terlihat MAF).
  Tanpa pendingin: plenum menuju campuran berbobot aliran antara muatan segar dan gas pada suhu buang. Aktuator 0.1 s.
  ECU: peta EGR (% atas rpm × bahan bakar) — terbuka di idle / beban ringan, tertutup ke beban penuh dan putaran
  tinggi (titik awal, bukan bersumber); hanya mesin hangat (≥60 °C), tidak di limp mode.
- **Preset 24**: VGT (A/R terbuka 0.8, tertutup 12 %), katup EGR 25 mm, target boost maks 0.8 bar.
  Hasil (dyno, pedal penuh): boost 0.31 / 0.67 / 0.80 / 0.79 / 0.83 bar di 1000 / 1500 / 2000 / 3000 / 4000 rpm
  (semua di target), torsi 156 / 217 / 234 / 230 / 220 Nm — kurva datar khas diesel VGT (versi wastegate 4a: 120 /
  164 / 233 / 220 / 212); tekanan puncak ≤150 bar. Pedal 30 %: EGR 28–39 % muatan di 1500 rpm, ±10 % di 2500 rpm,
  λ turun (3.00 → 1.83), suhu intake naik (tanpa pendingin, ±144 °C); boost sedikit di bawah target karena EGR
  mengambil energi buang dari turbin (kopling VGT–EGR).
- **Perbaikan sampingan**: aliran EGR untuk telemetri dirata-rata (0.1 s) — aliran per langkah nol di antara pulsa.
- **UI**: Boost hardware › "Variable geometry (VGT)", luas vane tertutup, persentase bukaan vane; Intake / fuel ›
  katup EGR (mm) + aliran / persentase; tab Diesel › peta EGR dan batas suhu pendingin.
- **Tes** (`diesel_test`): VGT memegang target di 1500 rpm (0.63 vs 0.63) dan 3500 rpm (0.80 vs 0.80); turbin yang
  sama dengan vane terbuka hanya 0.04 bar di 1500; EGR beban sebagian > 10 % muatan, udara segar dan λ turun, intake
  lebih panas; beban penuh katup tertutup.
- **Batas**: konstanta f_vgt dan peta EGR / target boost adalah kalibrasi; efisiensi turbin tidak bergantung posisi
  vane; tanpa pendingin EGR (suhu intake tinggi); NOx / jelaga tidak dimodelkan (EGR di sini hanya mengencerkan dan
  memanaskan).

### Paket 4 lanjutan – tunda Assanis dan injeksi pilot ✅
- **Tunda penyalaan**: korelasi Assanis et al. (J. Eng. Gas Turbines Power 125, 2003) τ = 2.4 φ^−0.2 p^−1.02
  exp(2100/T) ms menggantikan HH di mesin (HH tetap ada sebagai fungsi dan di tes). Assanis peka suhu dan tekanan
  muatan (HH hampir tidak peka suhu — lihat 4c); efek cetane tidak ada di Assanis, jadi faktor cetane HH (E_A =
  618840/(CN+25)) dipakai **relatif terhadap CN 51** di atas 12.9 bar. φ = bahan bakar siklus itu / udara.
- **Injeksi pilot**: Busch et al. (Sandia, SAND2015-2001 / SAE 2015-01-0796): pilot 1.5 mg di 1500 rpm, IMEPg 9 bar,
  bising pembakaran turun ±6 dB untuk dwell > ±200 µs. ECU: `pilot_mg` 1.5, dwell 300 µs dari 1500 rpm, **1000 µs di
  idle** (dipilih: tunda pilot sendiri ±0.5 ms di idle — dengan 300 µs pilot belum menyala saat main mulai), sampai
  3000 rpm dengan histeresis 200 rpm. Kuantitas main = permintaan − pilot.
- **Mesin**: pilot dan main punya jam pembakaran sendiri. Pilot yang sudah menyala saat main mulai: main mendapat
  tunda sendiri (integral yang sama, dari keadaan muatan yang sudah dipanaskan pilot), **dibagi `kPilotMainDelayDiv` =
  2.2** — kalibrasi: efek suhu saja hanya memberi −2.8 dB (dan +2…3 dB di beban lebih tinggi); dengan 2.2 hasilnya
  ±6 dB seperti Busch. Bahan bakar main yang disuntikkan selama tundanya → premixed main; sisanya difusi + ekor dari
  saat main menyala. Pilot yang belum menyala saat main mulai: satu pembakaran biasa (premixed = semua yang sudah
  disuntikkan).
- **Perbaikan**: (1) pilot menyala / mati bolak-balik di batas rpm (3000) — siklus yang pilotnya dinyalakan ECU
  setelah sudut pilot lewat tidak pernah mulai injeksi: torsi WOT 3000 jatuh ke 162 Nm. Kini histeresis 200 rpm, dan
  siklus yang belum mulai injeksi mulai di sudut main tanpa pilot. (2) **Pangsa EGR** (telemetri dan campuran suhu
  plenum) memakai aliran EGR rata-rata 0.1 s dibagi aliran udara **sesaat** — di antara pulsa intake udara berhenti,
  pangsa terbaca 100 %: tes melaporkan 63 % dan plenum terlalu panas. Kini udara dirata-rata dengan cara yang sama:
  37 % di 1500 rpm pedal 30 %, intake ±135 °C (sebelumnya ±155 °C).
- **Hasil** (preset 24): bising pembakaran (level teratenuasi struktur) 1500 rpm pedal 50 % 102.8 dB dengan pilot vs
  108.4 dB tanpa (−5.5 dB), idle −6.1 dB. Pedal penuh 149 / 210 / 233 / 230 / 219 Nm di 1000 / 1500 / 2000 / 3000 /
  4000 rpm (4b: 156 / 217 / 234 / 230 / 220), tekanan puncak 100–150 bar.
- **Tes** (`diesel_test`): Assanis vs hitungan tangan, efek cetane, pilot −6 dB di 1500 rpm beban menengah (±1.5 dB)
  dan lebih pelan di idle. Bising diukur dari RMS laju penuh (`combNoiseRms`) — sampling 1 ms ter-alias dengan ledakan
  pembakaran.
- **Batas**: pembagi tunda main 2.2 dan dwell idle adalah kalibrasi; satu pilot (tanpa post-injection); laju injeksi
  konstan (tanpa bentuk laju / tekanan rail).

### Paket 5 – Mesin 2-tak (tahap 5a + 5b + pengisian): port, karter, reed, scavenging, expansion chamber ✅
Rujukan dicek (teks dibaca kecuali disebut lain); yang tidak terverifikasi disebut "dipilih". Rujukan utama pengisian:
Blair, *Design and Simulation of Two-Stroke Engines* (SAE R-161, 1996) — dibaca dari salinan pindaian (teks OCR +
gambar halaman; angka dari grafik ±3–5 %).
- **Tipe mesin** `EngineType::TwoStroke`: siklus 360°, satu langkah kerja per putaran (`revsPerFiring` = 1 seperti
  rotari: ECU, injeksi sekuensial, koil, frekuensi pengapian, fase silinder 360/N). Tanpa katup / noken (tanpa float,
  tanpa sync cam).
- **Port** (`CylGeom::portArea`): luas = lebar × tinggi yang sudah dibuka mahkota piston (kinematika engkol), simetris
  terhadap BDC, penuh di BDC. Parameter: buang terbuka (°ATDC), transfer terbuka, lebar buang / transfer (× bore).
  **Cd port buang terukur** (Blair App. A2.3, Gbr. A2.4–A2.7, port buang mesin GP 125 cc, Cd "aktual" untuk simulasi):
  fungsi rasio luas port / pipa k dan rasio tekanan — keluar 0.82–0.96 (k 0.13) … 0.59–0.64 (k 0.82), balik 0.64–0.81.
  Cd transfer 0.75 (koefisien inflow terukur yang sama pada k ±0.44, dipinjam — saluran transfer tidak dimodelkan).
- **Karter** (`Engine::crankcaseStep`): volume di bawah piston, terbesar di TDC; volume minimum dari rasio kompresi
  primer (Vmin + Vd) / Vmin. Energi eksplisit (U = m cv T): kerja kompresi, entalpi masuk / keluar, kalor ke dinding
  (h 100 W/m²K, dipilih). Tekanan karter bekerja di bawah piston (kerja pompa karter ada di torsi).
- **Reed valve**: pegas-massa-redaman (Dave dkk., SAE 2004-01-1455: memadai terhadap CFD — abstrak), mode pertama
  150 Hz (Collins, MPhil Oxford Brookes 1994, mesin uji 125 cc: 120–183 Hz; petal glass-fibre mesin GP 125 Blair: 160 Hz,
  dihitung dari E 21.5 GPa, ρ 1850 — cocok), gaya ∝ Δp × (15 − angkat mm)/15 (Collins: 1/3 pada 10 mm), stopper 10 mm
  (dipilih), satu arah. Kekakuan statis (12 kPa untuk angkat penuh) dan redaman 0.3 **dikalibrasi**: angkat puncak
  6.9 mm di 5500 rpm gas penuh versi 125 cc (Collins: 6.3–8.2 mm di 5000 rpm). Uji: kekakuan petal Blair (±5 kPa),
  luas 2× atau redaman ½ hampir tidak mengubah delivery ratio — reed bukan pembatas.
- **Scavenging: model Blair** (`sim/Scavenge.h`, sec. 3.3), menggantikan Benson–Brandham (BB tidak bisa meniru silinder
  loop yang bagus: kemiringan ln(1−SE) vs SR terukur −1…−2, BB hanya 0…−1 — Blair Gbr. 3.14 / SAE 881265). Kurva
  SE–SR terukur 10 silinder nyata dari rig scavenging QUB (Gbr. 3.16: ln(1−SEv) = K0 + K1 SRv + K2 SRv²); kemurnian gas
  di port buang = kemiringan kurva (pers. 3.3.12); suhu udara / gas buang di silinder dari Ctemp = 0.65 Kse Kcp
  (pers. 3.3.13–3.3.15, fit Blair); SE massa → volume (3.3.16), suhu keluar (3.3.18). Parameter `ts_scavenge` = silinder
  (preset: YAM14, loop 5 port yang bagus). Telemetri per siklus: delivery ratio dan trapping efficiency **atas udara**
  (definisi Blair / MIT), scavenging efficiency, charging efficiency (udara segar terperangkap / Vd pada densitas
  ambien).
- **Expansion chamber — desain empiris Blair** (`tunedPipeGeometry`, sec. 6.2.5): panjang tala port → tail pipe
  LT = 83.3 a0 θep / rpm (a0 pada 500 °C enduro … 600 °C GP), L1…L7 = 0.1 / 0.275 / 0.183 / 0.092 / 0.11 / 0.24 / 0.24 LT,
  d1 = k1 d0 (1.125 → 1.05), d4 = k2 d0 (2.125 → 3.25), tail pipe 0.7 / 0.65 / 0.6 d0, diffuser horn kh 1.5 (d2, d3 dari
  rumus horn — diperiksa: contoh buku 56 / 89 mm, hasil 56.4 / 88.7). `ts_pipe_class` 0 enduro … 0.5 motocross … 1
  road race: konstanta di antara ujung-ujung Blair diinterpolasi (dipilih). Tab Ports menampilkan semua dimensi.
- **Dinamika gas 1D di pipa tala** (`sim/Duct.h`): gelombang di expansion chamber berukuran ±tekanan ambien (Blair Gbr.
  5.34: pulsa ±2 atm, hisapan menarik silinder ke 0.7 atm, plugging menaikkan 1 → 2+ atm) — akustik kecil-sinyal
  (waveguide) hanya memberi ±⅓ hisapan / plugging (linear: pulsa 0.4 ms memantul dari horn 0.46 m dengan ±−0.22 —
  persis yang terukur di model lama; kecepatan gelombang tergantung amplitudo tidak menolong). Kini: persamaan Euler
  kuasi-1D (luas berubah), volume hingga ±20 mm, MUSCL-Hancock + HLLC (Toro), dua skalar (bahan bakar, gas terbakar)
  ikut aliran — muatan segar yang lolos ke pipa didorong kembali oleh plugging. Dinding: gesek Cf 0.005 dan kalor
  analogi Reynolds (dipilih); suhu dinding mengikuti gas (dipilih agar ±350 °C asumsi Blair). Ujung port: orifice + karakteristik
  dari pipa (diselesaikan implisit tiap langkah); ujung belakang: karakteristik keluar ↔ waveguide silencer (suara
  tetap dari waveguide). Port 2-tak tanpa pipa tala: port langsung ke pipa akustik (tanpa volume port), sama implisit.
- **Perbaikan lain yang ditemukan**: (1) muatan segar yang short-circuit dihitung sebagai afterfire yang terbakar di pipa
  (+700 K) — pada 2-tak bersilinder-karter ia keluar tak terbakar (emisi HC tinggi); kini hanya muatan misfire;
  (2) gesekan 2-tak kini Blair pers. 5.4.2 (bantalan gelinding, sepeda motor): fmep = 105 Lst N Pa — tanpa aksesori;
  (3) DR / TE atas udara.
- **Validasi — mesin balap 125 Blair** (sec. 5.5.2: 56 × 50.6, CR terperangkap 9, karter 1.35, EPO 81° / TPO 115°,
  pipa tala 11 750 rpm; terukur Gbr. 5.29): TE 0.45 / 0.66 (terukur 0.50 / 0.69) di 8780 / 11 740 rpm — sesuai;
  CE 0.36 / 0.58 (0.63 / 0.79); DR 0.81 / 0.89 (1.26 / 1.15). Sebelum paket pengisian: DR 0.66, TE 0.55, CE 0.37 di
  11 740 dan tanpa puncak tala. Dengan pembakaran 50° (uji): pmax 80 bar (terukur ±87), bmep 6.8 bar (10.5).
- **Inersia saluran transfer** (2026-11-21, `Engine::transferFlow`, `ts_transfer_len`): aliran transfer bukan lagi
  orifice kuasi-tunak — kolom gas di saluran (panjang L = 1.5 × langkah, dipilih; luas = luas port transfer penuh)
  dm/dt = (A/L)(Δp − m|m|/(2ρ CdA²)), implisit, dibatasi aliran tercekik. Momentum kolom tetap mendorong muatan saat
  tekanan silinder naik lagi (plugging) — Blair memodelkan saluran sebagai pipa. Cek 125 Blair: DR 0.86–0.91 tanpa
  inersia → 1.10 / 1.13 / 1.03 di 8780 / 9600 / 11 740 rpm (terukur 1.26 / 1.20 / 1.15; simulasi Blair sendiri ±1.1),
  CE 0.45 / 0.57 / 0.65 (0.63 / 0.65 / 0.79), TE 0.41 / 0.51 / 0.64 (0.50 / 0.54 / 0.69). Panjang 1 / 2 × langkah
  memberi DR 1.01 / 1.04 di 11 740 — sensitif, jadi parameter (tab Ports).
- **Kecepatan (2026-11-21)**: sim load di rig (render penuh dengan suara) preset 25 1 / 2 / 3 silinder: 77 / 141 / 207 %
  (commit 2732411) → 31–33 / 54–55 / 75–90 % (diukur berulang; app idle preset 25: 27 %). Caranya: limiter MC (lebih tajam dari minmod: 40 mm ≈ minmod 20 mm), sel ±40 mm,
  duct melangkah ±48 kHz (aliran port dirata-rata antar langkah — massa / energi tetap kekal), geometri dihitung sekali,
  solver batas mulai dari solusi sebelumnya, loop digabung. Harga: CE 125 Blair 0.65 vs 0.67 pada sel 20–30 mm (−3 %).
  Bug yang ditemukan sesudahnya: langkah duct adaptif (menunggu sampai batas Courant, di 0 rpm sampai 4 langkah mesin)
  membuat kopling batas terlambat → pipa berdengung sendiri ±530 Hz saat mesin diam. Kini ritme tetap ±48 kHz dan ujung
  keluar diselesaikan tiap langkah mesin; tes `[rest]` di twostroke_test (rms < 1e-3 saat diam / sesudah dimatikan).
- **Pembakaran 2-tak — Blair** (sec. 4.3.6; `Engine::burnDuration`): tunda nyala lalu kurva Vibe
  x = 1 − exp(−a (θ/b)^(m+1)) dengan a 5.25, m 1.25 (fit Blair untuk mesin GP 125 terukur, Cartwright). Data terukur:
  GP 125 10 350 rpm SE 0.96 — tunda 12°, b 41°; LS400 3000 rpm WOT — tunda 10°, b 60°; Reid 3000 rpm gas 100 / 25 /
  10 % (SE 0.80 / 0.75 / 0.65) — tunda 13 / 14 / 23°, b 49 / 43 / 45°; chainsaw 9600 rpm — b 64° (campuran "sedikit
  berpengaruh"). Blair: b "jarang < 45° atau > 65°, tak tergantung putaran dan beban" → **b = parameter ruang bakar**
  `ts_burn_deg` (squish), tanpa suku rpm / beban / campuran; preset 25: 41° (ruang balap seperti GP 125), default 50°.
  Pengenceran dan tekanan rendah beban ringan masuk ke **tunda**: mengikuti faktor kernel api (∝ 1/S_L, Flame.h),
  diskalakan agar GP 125 di rig-nya memberi 12° (faktor kernel 1.00 di 10 350 rpm — kalibrasi satu titik). Cek mesin
  ukuran LS400 (85 × 70, CR terperangkap 6.6) 3000 rpm: tunda 10.3° gas penuh / 22.4° gas ringan (SE 0.66, DR 0.23)
  vs Reid 13 / 23° (SR 0.24) dan Kee 10° — cocok. Hasil GP 125: 50 % terbakar 8.7° ATDC (terukur 9°), pmax 72 bar pada
  CE 0.59 (terukur ±87 bar pada CE 0.79). Model durasi 4-tak (A4) tidak lagi dipakai untuk 2-tak. Batas: hanya satu
  titik kalibrasi untuk tunda; data b tersebar 41–64° antar mesin (squish / bentuk ruang bakar belum dimodelkan).
- **Start / idle**: karter awal berisi campuran λ 0.8 (sisa bahan bakar dari jalan terakhir, dipilih) — dekoder
  trigger memotong percikan ±2 putaran pertama. Start hangat 0.44 s, idle ±1700 rpm (udara idle 1.0, four-stroking).
- **Four-stroking**: muncul sendiri dari model misfire umpan balik residu (Daw/ORNL, sama dengan "brap" rotari): di
  3000 rpm gas 4.5 % ±46–55 % siklus misfire (2 %: 81 %), hampir semua karena residu; batasnya bergeser karena scavenging Blair
  menyisakan lebih sedikit residu — SAE 830091 (Tsuchiya dkk., Yamaha; abstrak), Lu & Lee SAE 960746 (abstrak).
  Gas penuh: 0 % misfire.
- **Preset 25 "1-cyl 250 two-stroke (MX-style)"**: bore × stroke dan CR dari Honda CR250R 2002 (factbook Honda:
  66.4 × 72, 8.5:1 — diambil sebagai CR terperangkap → geometris 16), buang 82° / transfer 116° ATDC, lebar 0.70 / 1.10
  × bore, karter 1.40:1, reed 20 cm², karburator 38 mm, YAM14, expansion chamber motocross tala 8000 rpm. Timing port /
  lebar MX **tidak terverifikasi** (sumber: Yamaha MT-110 EPO 104°, TPO 128° — Zhang 1995; Piaggio 50 EPO 105°,
  TPO 123° — snippet). Pengapian 14° idle naik ke 22° pada 6000 rpm: terbaik di rig dengan pembakaran Blair (seperti
  20° mesin GP terukur; sebelumnya butuh 32–38° karena pembakaran lambat).
- **UI**: Engine › tipe mesin "Two-stroke"; tab Ports: port, karter, reed, silinder scavenging (Blair), DR / TE / SE /
  tekanan karter / bukaan reed langsung, expansion chamber (tala rpm, desain enduro…road race, dimensi L1–L7 / d0–d7).
- **Tes** (`twostroke_test`): geometri port; karter dipompa, reed tertutup saat tekanan karter puncak, angkat reed vs
  Collins; kemurnian keluar Blair (gas buang dulu, segar kemudian), YAM14 > YAM12 (TE, SE); validasi 125 Blair (DR ±0.2 dan TE ±0.1
  dari terukur di dua putaran, tala menaikkan TE / CE, CE > 0.6); pembakaran (tunda GP 125 ±1.5° dari 12°, 50 %
  terbakar 5–11° ATDC, tunda bertambah > 6° di beban ringan seperti Reid); pipa tala > pipa biasa, puncak torsi pindah
  dengan tala; four-stroking di gas 4.5 %; start dan idle.
- **Batas (penting)**: preset 25 kini ±31 N·m di 8000–9000 rpm (awal paket: ±12–15) vs CR250R 51 N·m @ 8000
  (spesifikasi Honda) / YZ250 41.5 N·m (dyno MXA). Sisa: (a) DR masih ±0.1–0.15 di bawah terukur dan TE di putaran
  rendah lebih kecil (0.41 vs 0.50): intake (karburator + reed) belum gas-dinamika 1D, panjang saluran transfer
  dipilih; (b) pipa tala sedikit terlalu "tinggi" (tala 8000 → puncak 9000); (c) b per mesin (squish) dipilih dari
  data Blair. Preset 25 (karburator) tidak memakai base map. Belum: oli campur (premix), port intake piston / rotary
  valve, power valve, tampilan 3D 2-tak.

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
