#![no_std]
#![no_main]

use panic_halt as _;
use esp_hal::{
    Config,
    uart::{Uart, Config as UartConfig},
    gpio::{Output, Level, OutputConfig},
    time::{Instant, Duration},
};
use esp_println::{println, print};

esp_bootloader_esp_idf::esp_app_desc!();

// ================== KONFIGURASI ==================
const BAUD: u32 = 9_600;
const SID:  u8  = 1;
const FC:   u8  = 0x04;
const REG_START: u16 = 0x0001;
const REG_QTY:   u16 = 2;

const TURNAROUND_SPINS: u32 = 8_000;
const TIMEOUT_SPINS:    u32 = 200_000;

// === Ambang batas kontrol dengan HYSTERESIS ===
// POMPA (Humidifier - menambah kelembaban)
const RH_ON:  f32 = 40.0;  // Pompa ON jika RH < 40% (kelembaban rendah)
const RH_OFF: f32 = 50.0;  // Pompa OFF jika RH > 50% (kelembaban cukup)

// KIPAS (Pendingin - menurunkan suhu)
const T_ON:   f32 = 35.0;  // Kipas ON jika T > 35°C (suhu tinggi)
const T_OFF:  f32 = 30.0;  // Kipas OFF jika T < 30°C (suhu normal)

const DELAY_MS: u32 = 2000; // Delay antar pembacaan

// Minimum delay untuk switching relay (mencegah chattering)
const MIN_SWITCH_DELAY_MS: u32 = 5000; // 5 detik
// =================================================

// State tracking
static mut PUMP_STATE: bool = false;
static mut FAN_STATE: bool = false;
static mut PUMP_LAST_SWITCH: Option<Instant> = None;
static mut FAN_LAST_SWITCH: Option<Instant> = None;
static mut LOOP_COUNT: u32 = 0;

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(Config::default());

    // ===== MATIKAN LED PUTIH BOARD =====
    let mut led8  = Output::new(p.GPIO8, Level::Low, OutputConfig::default());
    let mut led38 = Output::new(p.GPIO38, Level::Low, OutputConfig::default());
    let mut led48 = Output::new(p.GPIO48, Level::Low, OutputConfig::default());
    led8.set_low();
    led38.set_low();
    led48.set_low();
    // ====================================

    // UART1 TX=GPIO17, RX=GPIO18
    let mut uart = Uart::new(p.UART1, UartConfig::default().with_baudrate(BAUD))
        .expect("UART1 init failed")
        .with_tx(p.GPIO17)
        .with_rx(p.GPIO18);

    // RS485 DE → HIGH = TX, LOW = RX
    let mut de = Output::new(p.GPIO16, Level::Low, OutputConfig::default());

    // === Relay aktif-HIGH (HIGH = ON, LOW = OFF) ===
    let mut pump = Output::new(p.GPIO15, Level::Low, OutputConfig::default()); // Pompa
    let mut fan  = Output::new(p.GPIO12, Level::Low, OutputConfig::default()); // Kipas
    // ===============================================

    println!("\n╔═══════════════════════════════════════════════════════════╗");
    println!("║  SHT20 RS485 + Relay Control v2.0 (Hysteresis + Logger) ║");
    println!("╚═══════════════════════════════════════════════════════════╝");
    println!("📌 Pin Config:");
    println!("   UART: TX=GPIO17, RX=GPIO18, DE=GPIO16");
    println!("   Relay: Pump=GPIO15, Fan=GPIO12");
    println!("\n⚙️  Control Parameters:");
    println!("   💧 Pompa (Humidifier): ON < {:.1}%, OFF > {:.1}%", RH_ON, RH_OFF);
    println!("   🌬️  Kipas (Cooling): ON > {:.1}°C, OFF < {:.1}°C", T_ON, T_OFF);
    println!("   ⏱️  Min Switch Delay: {}s", MIN_SWITCH_DELAY_MS/1000);
    println!("   🔄 Read Interval: {}ms", DELAY_MS);
    println!("\n🚀 Starting monitoring...\n");

    // Siapkan frame Modbus
    let mut req = [0u8; 8];
    build_req(&mut req, SID, FC, REG_START, REG_QTY);
    
    // Log frame Modbus yang akan dikirim
    println!("📤 Modbus Request Frame:");
    print!("   HEX: ");
    for (i, &byte) in req.iter().enumerate() {
        print!("{:02X}", byte);
        if i < req.len() - 1 { print!(" "); }
    }
    println!();
    println!("   SID={}, FC={}, Reg={:#06X}, Qty={}\n", SID, FC, REG_START, REG_QTY);

    let mut error_count = 0u32;
    let mut total_reads = 0u32;
    let mut successful_reads = 0u32;

    loop {
        unsafe { LOOP_COUNT += 1; }
        total_reads += 1;

        println!("─────────────────────────────────────────────────────────");
        println!("🔁 Loop #{} | Time: {}ms", unsafe { LOOP_COUNT }, Instant::now().elapsed().as_millis());
        
        // Bersihkan RX buffer
        {
            let mut b = [0u8; 1];
            let mut cleared = 0;
            while uart.read(&mut b).is_ok() { cleared += 1; }
            if cleared > 0 {
                println!("🧹 Cleared {} bytes from RX buffer", cleared);
            }
        }

        // --- KIRIM FRAME ---
        println!("📤 Sending Modbus request...");
        de.set_high();
        let _ = uart.write(&req);
        let _ = uart.flush();
        short_spin(TURNAROUND_SPINS);
        de.set_low();
        println!("   ✓ Request sent, switching to RX mode");

        // --- BACA RESPON ---
        let mut rx = [0u8; 16];
        let mut n = 0usize;
        let mut spins = 0u32;
        let start_read = Instant::now();

        while n < 9 && spins < TIMEOUT_SPINS {
            let mut b = [0u8; 1];
            match uart.read(&mut b) {
                Ok(1) => { 
                    rx[n] = b[0]; 
                    n += 1;
                }
                _ => short_spin(400),
            }
            spins += 1;
        }

        let read_time = start_read.elapsed().as_millis();
        println!("📥 Response received: {} bytes in {}ms", n, read_time);

        // Log raw response
        if n > 0 {
            print!("   RAW HEX: ");
            for i in 0..n {
                print!("{:02X}", rx[i]);
                if i < n - 1 { print!(" "); }
            }
            println!();
        }

        // --- PARSE DATA & KONTROL RELAY ---
        if n == 9 && valid_rsp(&rx[..n], SID, FC, 4) {
            successful_reads += 1;
            let t_raw  = u16::from_be_bytes([rx[3], rx[4]]);
            let rh_raw = u16::from_be_bytes([rx[5], rx[6]]);
            let t  = t_raw as f32 / 10.0;
            let rh = rh_raw as f32 / 10.0;
            
            println!("✅ Valid response:");
            println!("   Temperature: {:.1}°C (raw: {})", t, t_raw);
            println!("   Humidity: {:.1}% (raw: {})", rh, rh_raw);
            
            error_count = 0; // Reset error counter
            
            unsafe {
                let pump_status = if PUMP_STATE { "ON " } else { "OFF" };
                let fan_status = if FAN_STATE { "ON " } else { "OFF" };
                
                println!("\n📊 Current Status:");
                println!("   💧 Pompa: {} | 🌬️  Kipas: {}", pump_status, fan_status);
                println!("   Success Rate: {}/{} ({:.1}%)", 
                    successful_reads, total_reads, 
                    (successful_reads as f32 / total_reads as f32) * 100.0
                );

                // JSON Output
                println!(r#"   JSON: {{"t":{:.1},"rh":{:.1},"pump":"{}","fan":"{}"}}"#, 
                    t, rh, pump_status.trim(), fan_status.trim()
                );

                let now = Instant::now();

                // === Logika Pompa dengan Hysteresis ===
                println!("\n🔍 Pump Logic Check:");
                println!("   Current RH: {:.1}% | Thresholds: ON<{:.1}% OFF>{:.1}%", rh, RH_ON, RH_OFF);
                println!("   Current State: {}", if PUMP_STATE { "ON" } else { "OFF" });
                
                let can_switch_pump = match PUMP_LAST_SWITCH {
                    None => {
                        println!("   Can switch: YES (first run)");
                        true
                    }
                    Some(last) => {
                        let elapsed = now.elapsed().as_millis().saturating_sub(last.elapsed().as_millis());
                        let can = elapsed >= MIN_SWITCH_DELAY_MS as u64;
                        println!("   Time since last switch: {}ms / {}ms | Can switch: {}", 
                            elapsed, MIN_SWITCH_DELAY_MS, if can { "YES" } else { "NO" });
                        can
                    }
                };

                if can_switch_pump {
                    if !PUMP_STATE && rh < RH_ON {
                        // Kelembaban rendah → Nyalakan pompa
                        pump.set_high();
                        PUMP_STATE = true;
                        PUMP_LAST_SWITCH = Some(now);
                        println!("   ⚡ ACTION: Pompa ON (RH {:.1}% < {:.1}%)", rh, RH_ON);
                        println!("   💧 Humidifier activated to increase humidity");
                    } else if PUMP_STATE && rh > RH_OFF {
                        // Kelembaban cukup → Matikan pompa
                        pump.set_low();
                        PUMP_STATE = false;
                        PUMP_LAST_SWITCH = Some(now);
                        println!("   ⚡ ACTION: Pompa OFF (RH {:.1}% > {:.1}%)", rh, RH_OFF);
                        println!("   💧 Humidity sufficient, humidifier stopped");
                    } else {
                        println!("   ⏸️  No action needed (in hysteresis zone {:.1}%-{:.1}%)", RH_ON, RH_OFF);
                    }
                } else {
                    println!("   ⏸️  Switch delayed (waiting for minimum delay)");
                }

                // === Logika Kipas dengan Hysteresis ===
                println!("\n🔍 Fan Logic Check:");
                println!("   Current T: {:.1}°C | Thresholds: ON>{:.1}°C OFF<{:.1}°C", t, T_ON, T_OFF);
                println!("   Current State: {}", if FAN_STATE { "ON" } else { "OFF" });
                
                let can_switch_fan = match FAN_LAST_SWITCH {
                    None => {
                        println!("   Can switch: YES (first run)");
                        true
                    }
                    Some(last) => {
                        let elapsed = now.elapsed().as_millis().saturating_sub(last.elapsed().as_millis());
                        let can = elapsed >= MIN_SWITCH_DELAY_MS as u64;
                        println!("   Time since last switch: {}ms / {}ms | Can switch: {}", 
                            elapsed, MIN_SWITCH_DELAY_MS, if can { "YES" } else { "NO" });
                        can
                    }
                };

                if can_switch_fan {
                    if !FAN_STATE && t > T_ON {
                        // Suhu tinggi → Nyalakan kipas
                        fan.set_high();
                        FAN_STATE = true;
                        FAN_LAST_SWITCH = Some(now);
                        println!("   ⚡ ACTION: Kipas ON (T {:.1}°C > {:.1}°C)", t, T_ON);
                        println!("   🌬️  Cooling fan activated to reduce temperature");
                    } else if FAN_STATE && t < T_OFF {
                        // Suhu normal → Matikan kipas
                        fan.set_low();
                        FAN_STATE = false;
                        FAN_LAST_SWITCH = Some(now);
                        println!("   ⚡ ACTION: Kipas OFF (T {:.1}°C < {:.1}°C)", t, T_OFF);
                        println!("   🌬️  Temperature normal, cooling fan stopped");
                    } else {
                        println!("   ⏸️  No action needed (in hysteresis zone {:.1}°C-{:.1}°C)", T_OFF, T_ON);
                    }
                } else {
                    println!("   ⏸️  Switch delayed (waiting for minimum delay)");
                }
            }
        } else {
            error_count += 1;
            println!("❌ Invalid/Timeout response:");
            println!("   Bytes received: {}/9", n);
            println!("   Error count: {}/3", error_count);
            
            if n > 0 {
                if !valid_rsp(&rx[..n], SID, FC, 4) {
                    println!("   Issue: CRC or format validation failed");
                    if n >= 2 && (rx[1] & 0x80) != 0 {
                        println!("   ⚠️  Modbus Exception: Error code {:#04X}", rx[2]);
                    }
                }
            } else {
                println!("   Issue: No response (timeout)");
            }

            println!(r#"   JSON: {{"t":null,"rh":null,"error_count":{}}}"#, error_count);

            // 🔒 Fallback aman setelah 3 kali error berturut-turut
            if error_count >= 3 {
                unsafe {
                    pump.set_low();
                    fan.set_low();
                    PUMP_STATE = false;
                    FAN_STATE = false;
                }
                println!("\n🚨 SAFETY MODE ACTIVATED!");
                println!("   All relays turned OFF due to communication errors");
            }
        }

        println!("\n⏳ Waiting {}ms before next read...", DELAY_MS);
        delay_ms(DELAY_MS);
    }
}

// ================== UTILITAS ==================
fn short_spin(iter: u32) {
    for _ in 0..iter {
        core::hint::spin_loop();
    }
}

fn delay_ms(ms: u32) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(ms as u64) {
        core::hint::spin_loop();
    }
}

fn build_req(buf: &mut [u8; 8], sid: u8, fc: u8, start: u16, qty: u16) {
    buf[0] = sid;
    buf[1] = fc;
    buf[2..4].copy_from_slice(&start.to_be_bytes());
    buf[4..6].copy_from_slice(&qty.to_be_bytes());
    let crc = crc16(&buf[..6]);
    buf[6] = (crc & 0xFF) as u8;
    buf[7] = (crc >> 8) as u8;
}

fn valid_rsp(frame: &[u8], sid: u8, fc: u8, byte_count: u8) -> bool {
    if frame.len() < 5 { return false; }
    frame[0] == sid &&
    (frame[1] & 0x80) == 0 &&
    frame[1] == fc &&
    frame[2] == byte_count &&
    check_crc(frame)
}

fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            let lsb = crc & 1;
            crc >>= 1;
            if lsb != 0 { crc ^= 0xA001; }
        }
    }
    crc
}

fn check_crc(frame: &[u8]) -> bool {
    if frame.len() < 3 { return false; }
    let calc = crc16(&frame[..frame.len() - 2]);
    frame[frame.len() - 2] == (calc & 0xFF) as u8 &&
    frame[frame.len() - 1] == (calc >> 8) as u8
}