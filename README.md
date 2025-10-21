# SISTEM _MONITORING_ SUHU DAN KELEMBAPAN BERBASIS SENSOR SHT-20 DENGAN _EDGE GATEWAY_ ESP32-S3 DAN AKTUATOR POMPA DAN FAN BERBASIS _RELAY LOW_
Proyek ini merupakan implementasi sistem  tertanam _(embedded system)_ berbasis rust yang di jalankan pada ESP 32 S3. Tujuan utama adanya proyek ini adalah untuk mengembangkan arsitektur pemrograman yang efisien, aman, dan _low-level_ menggunkan _embedded rust_, _framework_, sekaligus menguji komunikasi dasar seperti GPIO, sensor, dan jaringan

## Anggota Kelompok
1. Karina Lailatul Maghfiroh Maharani (2042231035)
2. Fortunia Putri Syahari (2042231078)
3. Ahmad Radhy, S.Si., M.Si. (Dosen Pengampu)

Teknik Instrumentasi — Institut Teknologi Sepuluh Nopember (ITS)

## 🚀 Fitur Utama
1. Sensor Monitoring (SHT 20)
   Sistem membaca data suhu dan kelembapan menggunakan sensor SHT20 melalui protokol Modbus RTU. Sensor ini memiliki akurasi tinggi (±0.3 °C
   untuk suhu dan ±3 % RH untuk kelembapan), antarmuka digital I²C, serta konsumsi daya rendah sehingga ideal untuk akuisisi data berkala.
3. Edge Gateway ESP32 S3
   ESP32-S3 berperan sebagai edge gateway yang menangani akuisisi data dari sensor dan pengendalian aktuator (relay untuk pompa dan fan).
   Firmware dikembangkan menggunakan Embedded Rust, yang memberikan keandalan tinggi dengan error handling aman dan efisien di sisi mikrokontroler.
5. Backend Data Distributor
   Program backend Rust menerima data dari ESP32-S3 melalui serial, menyimpan ke InfluxDB dalam format Line Protocol, dan mendistribusikannya ke ThingsBoard Cloud menggunakan protokol MQTT.
   Arsitektur asynchronous (Tokio) digunakan agar sistem mampu menangani data real-time secara efisien.
7. Time Seris Storage
   Semua data sensor dan simulasi disimpan dalam InfluxDB, sebuah time-series database yang dioptimalkan untuk data yang memiliki timestamp.
   Database ini memungkinkan analisis tren suhu dan kelembapan dalam periode tertentu.
9. Cloud Visualization (ThingsBoard Dashboard)
   Data dari InfluxDB dikirim ke ThingsBoard Cloud untuk divisualisasikan dalam dashboard real-time. Platform ini menampilkan timeseries chart suhu dan kelembapan serta mendukung pemantauan jarak jauh.
10. Kontrol Aktuator Pompa & Fan (_Relay Low Active_)
   Pompa air dan fan dikontrol otomatis oleh ESP32-S3 berdasarkan nilai ambang suhu dan kelembapan. Ketika suhu meningkat, fan menyala; ketika kelembapan menurun, pompa aktif.
   Karena sistem tidak menggunakan logika biner 1/0 untuk logging, status aktuator tidak direkam ke InfluxDB melainkan dijalankan langsung oleh edge gateway.
   
## 🧩 Instalasi dan Setup Lingkungan
## 📸 Hasil Tampilan Sistem
### Tampilan Data Explorer InfluxDB
![influx](https://github.com/user-attachments/assets/11161768-aebb-4686-b66c-657c703e1538)

Gambar menunjukkan antarmuka InfluxDB Data Explorer yang digunakan untuk membuat query dan memvisualisasikan data time-series. 
Data diambil dari bucket bernama sktfix dengan measurement sensor_data dan tag device = ESP32. 
Grafik memperlihatkan tren suhu dan kelembapan yang berhasil direkam dan stabil di akhir periode pengamatan.
Namun, hasil aktuator tidak muncul pada InfluxDB karena sistem tidak menggunakan logika biner (1 untuk ON dan 0 untuk OFF) sebagai indikator status relay. 
Akibatnya, aksi pompa dan fan hanya dijalankan secara langsung oleh mikrokontroler tanpa proses pencatatan pada database.

### Tampilan Dashboard ThingsBoard

Tampilan Dashboard ThingsBoard menampilkan grafik Timeseries Line Chart dari dua parameter utama:
Humidity (Kelembapan) – Garis biru stabil di sekitar 66.23 %.
Temperature (Suhu) – Garis hijau stabil di sekitar 31.9 °C.
Kursor di sisi kanan grafik menunjukkan pembacaan terakhir dari kedua variabel. 
Tampilan ini membuktikan bahwa data sensor SHT20 yang dikirim dari ESP32-S3 dan diolah oleh backend Rust berhasil ditampilkan secara real-time di Cloud Platform ThingsBoard.
Status aktuator tetap tidak terekam di database karena kontrol relay aktif dilakukan langsung oleh ESP32-S3 tanpa proses logging ke InfluxDB.
### Tampilan Terminal VSCode
endela terminal menampilkan hasil eksekusi backend program berbasis Rust. Data mentah yang diterima dari serial, misalnya T = 25.4°C dan RH = 49.8%, berhasil diproses dan dikirim ke InfluxDB. 
Log terminal menunjukkan pesan seperti:
📥 Data mentah diterima: T=25.4, RH=49.8
✅ Data suhu & kelembaban terkirim ke InfluxDB
Hal ini menunjukkan fungsionalitas penuh dari program backend — mulai dari pembacaan data serial, konversi format, hingga pengiriman ke sistem penyimpanan time-series.

## 🧱 Struktur Direktori
esp/
└── hello/
    └── hello-rust/
        ├── src/
        │   ├── lib.rs              # Pustaka utama untuk fungsi inti
        │   └── bin/
        │       └── main.rs         # Program utama untuk ESP32-S3
        ├── changes/
        │   ├── main_1.rs           # Versi awal kode
        │   └── main_2.rs           # Versi revisi atau tambahan fitur
        ├── Cargo.toml               # File konfigurasi proyek Rust
        ├── build.rs                 # Skrip build untuk target ESP32
        ├── .cargo/config.toml       # Pengaturan toolchain dan target MCU
        ├── rust-toolchain.toml      # Spesifikasi versi Rust yang digunakan
        ├── export-esp.ps1           # Skrip PowerShell untuk ekspor/kompilasi
        └── .gitignore
