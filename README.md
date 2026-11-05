# Temperature Monitoring ICN-2 (PlatformIO)

Monitor suhu insinerator/pirolisis **ICN-2** Depok: 5 termokopel (MAX31856),
HMI Nextion, panel relay/reject, flow meter LFS, alarm limit suhu (EEPROM),
upload data ke server, dan fleet MQTT + OTA. Dulu berupa sketch Arduino
`biodigester_esp32_v4.4.ino` (masih ada di riwayat git), sekarang project
PlatformIO dengan pola yang sama seperti `ICN-Control-MQTT` (ICN-1).

```
5x MAX31856 (SPI, CS 15/2/4/5/13) ─┐
HMI Nextion   ── UART1 (RX32/TX33) ─┤
Panel relay   ── UART2 (RX16/TX17) ─┤  ESP32 ── WiFi ─> HTTPS POST /api/tempMonitor2
LFS meter 1-6 ── UART0 (Serial)    ─┘        └─> MQTT TLS :8883  depok_icn2/<id>/...
```

## Yang TIDAK berubah

- **Pin**: CS termokopel 15/2/4/5/13, HMI RX32/TX33, panel RX16/TX17, UART0 untuk LFS.
- **Protokol serial**: perintah HMI (`*IBsn`, `*IR1n`, `*IP0n`, `*IL00`+limit),
  panel (`*I11n`, `*I30n`/`*I31n`, output `*O...`), poll LFS (`#ID:n-GET`, `#ID:n-RESET`).
- **EEPROM**: 5 x int32 + CRC32 di alamat 0..23. Limit yang sudah tersimpan
  tetap terbaca setelah update.
- **Data ke server**: kolom `temp_monitor2` yang sama (`Tmp_CmbJ1`, `Tmp_PrlJ1_1..4`,
  `Tmp_IcnJ1`/`Tmp_DctJ1` = 0), sensor fault/offline dikirim 0. Interval tetap 60 detik.
- Filter noise (median + mean, tolak spike) dan alarm (limit, TC1 over-heat 100 °C).

## Yang berubah

| Lama (v4.4) | Sekarang (5.0.0) |
|---|---|
| `https://api-rmc.vercel.app/api/tempMonitor2` | `https://monitoring.xampah.com/api/tempMonitor2` |
| `setInsecure()` (sertifikat tidak dicek) | diverifikasi dengan ISRG Root X1 (Let's Encrypt) |
| API key hardcoded di sketch | dari `secrets.ini` (gitignored) |
| Tidak ada remote monitoring | Fleet MQTT site `depok_icn2`: status, log, OTA, rollback otomatis |

## Bug yang diperbaiki

Ditandai `[FIX]` di `src/main.cpp`.

1. **Limit tersimpan 0 di EEPROM**: frame `*IL00` yang terpotong menulis limit 0,
   sehingga alarm menyala terus. Sekarang frame dibaca biner 20 byte dan
   divalidasi (1..1370 °C). Limit tidak valid ditolak, dan HMI kembali ke nilai tersimpan.
2. **Limit tertentu tidak bisa disimpan**: limit yang byte-nya berisi 0x0A/0x0D/0xFF
   (10, 13, 255, 266, 269, 511, 522, 525, 767, …) memotong frame HMI. Sekarang aman.
3. **Heap habis / crash**: buffer serial HMI, panel, dan LFS memakai `String` tanpa
   batas. Sekarang buffer tetap 64 byte.
4. **Data LFS menumpuk saat halaman Setting**: buffer tidak pernah dikosongkan. Sekarang dibuang.
5. **Dua perintah panel dalam satu paket digabung** (`*I110*I111`) sehingga dua-duanya hilang.
6. **Sensor macet tanpa tanda**: bila MAX31856 tidak menjawab, state machine
   menunggu selamanya dan nilai lama terus di-upload. Sekarang timeout 1 s → fault.
7. **Warna merah tidak kembali** setelah fault sensor hilang.
8. **HTTP**: semua kode > 0 dianggap sukses (401/422/500 juga), dan `http.end()`
   dipanggil dua kali. Sekarang hanya 2xx yang sukses, dan 4xx tidak di-retry.
9. **UART HMI jenuh**: nilai dikirim ~30x/detik sehingga `loop()` tertahan menunggu UART.
   Sekarang dikirim hanya saat berubah, plus refresh tiap 5 detik.
10. **Output panel tidak sinkron setelah reset**: lampu alarm/WiFi bisa tertinggal
    ON dari sebelum reset. Status sebenarnya sekarang dikirim sekali setelah boot.
11. **Baris LFS pendek** (mis. balasan `#ID:n-RESET`) dikirim ke HMI sebagai `X_VolLFn.val=` kosong.
12. **WiFi**: `disconnect(true)` mematikan radio sementara auto-reconnect juga berjalan.
13. **Debug di bus LFS**: UART0 dipakai bersama meter LFS. Di build `release` tidak
    ada teks debug di UART0; log tetap terlihat di dashboard (live log MQTT).

## Kredensial

Di VPS (repo `API_Integration_IoT-Laravel12`):

```bash
bash scripts/provision-device.sh depok-icn2-01 --site depok_icn2
```

Salin `~/rmc-api-secrets/devices/depok-icn2-01.secrets.ini` ke repo ini sebagai
`secrets.ini` (sudah gitignored; contoh di `secrets.ini.example`).

> API key lama (`sk_x8dT...`) pernah ter-commit di sketch v4.4 dan masih ada di
> riwayat git. Anggap bocor: jangan dipakai lagi. Provisioning di atas membuat key baru.

## Build & flash

```bash
pio run -e release -t upload   # flash PERTAMA wajib lewat USB (firmware lama belum punya OTA)
pio device monitor             # 9600 (log debug hanya di env esp32dev)
pio run -e release             # image OTA -> .pio/build/release/firmware.bin
pio test -e native             # unit test logika murni (butuh g++/MinGW di PC)
```

- Selalu build OTA dengan env `release`. Env ini menolak compile tanpa API key /
  login MQTT, dan menolak bila `DP_MQTT_USER` != `DP_DEVICE_ID`.
- Env `esp32dev` (tanpa `secrets.ini`) untuk uji di meja. Debug serial menyala,
  MQTT nonaktif, dan upload dijawab 401. Jangan dipasang di lapangan: debug ikut
  terkirim ke bus LFS.
- Versi firmware ada di `Config::FIRMWARE_VERSION` (`include/config.h`). Naikkan
  setiap rilis, lalu upload release di dashboard dengan versi yang **sama persis**.

## Fleet: status, log, OTA

Site **`depok_icn2`**. Sengaja terpisah dari `depok_icn` (ICN-1): image ICN-1
memakai pin dan HMI yang berbeda, jadi server tidak pernah menawarkannya ke
board ini, dan firmware ini juga menolaknya. Task MQTT berjalan di core 0;
`loop()` (sensor, HMI, panel, alarm) tetap berjalan di core 1, termasuk selama download OTA.

| Topik | Arah | Isi |
|---|---|---|
| `depok_icn2/<id>/status` | publish (retained, LWT) | fw, 5 suhu + limit + status sensor, alarm, LFS, hasil upload |
| `depok_icn2/<id>/log` | publish | log debug + progres OTA |
| `depok_icn2/<id>/cmd` | subscribe | `PING` / `STATUS` / `RESTART` / `OTA` |

- **OTA**: dashboard Depok → Fleet (OTA) → tab **ICN-2 (insinerator)** → Firmware / Rollout.
  Image diverifikasi SHA-256. Kalau firmware baru gagal online 3x boot berturut-turut,
  controller kembali ke versi sebelumnya sendiri.
- Upload ke server ditahan selama OTA berjalan (menghemat heap untuk sesi TLS).

## Struktur

```
include/config.h          identitas, URL, MQTT, flag debug
src/main.cpp              logika kontrol (sensor, HMI, panel, LFS, EEPROM, upload)
src/mqtt_client.cpp       status / log / perintah fleet
src/ota_manager.cpp       OTA + rollback (sama dengan ICN-1)
lib/IcnCore/              logika murni + unit test: filter, framing serial, JSON, OTA rollback
```
