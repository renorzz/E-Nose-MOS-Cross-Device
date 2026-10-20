#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::{println, print};
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Flex, InputConfig, OutputConfig, Pull},
    i2c::master::{Config, I2c},
    uart::Uart,
    time::{Rate, Instant},
};
use dht_sensor::{dht22, DhtReading};

// =========================================================
// ELECTRONIC NOSE - ESP32 BARE-METAL (NO_STD) VERSION
// =========================================================
// Konfigurasi Hardware Terbaru:
// - ADS Kanan (0x48): A0 = MQ6, A1 = MQ135, A2 = MQ7, A3 = MQ3
// - ADS Kiri  (0x49): A0 = TGS2602, A1 = TGS2620, A2 = TGS2611, A3 = TGS2600
// - I2C Bus: SDA = GPIO 8 (LV1 -> HV1), SCL = GPIO 9 (LV2 -> HV2)
// - Motor Driver: Ditiadakan (Tanpa Motor Driver)
// =========================================================

// Tambahkan macro descriptor agar kompatibel dengan bootloader ESP-IDF
esp_bootloader_esp_idf::esp_app_desc!();

const ADS1_ADDR: u8 = 0x48; // ADS Kanan
const ADS2_ADDR: u8 = 0x49; // ADS Kiri
const ADS_REG_CONVERT: u8 = 0x00;
const ADS_REG_CONFIG: u8 = 0x01;

// =========================================================
// WRAPPER UNTUK SATISFY EMBEDDED-HAL 0.2 TRAITS
// =========================================================

struct OldDelay(Delay);

impl embedded_hal::blocking::delay::DelayUs<u8> for OldDelay {
    fn delay_us(&mut self, us: u8) {
        self.0.delay_micros(us as u32);
    }
}

impl embedded_hal::blocking::delay::DelayUs<u16> for OldDelay {
    fn delay_us(&mut self, us: u16) {
        self.0.delay_micros(us as u32);
    }
}

impl embedded_hal::blocking::delay::DelayUs<u32> for OldDelay {
    fn delay_us(&mut self, us: u32) {
        self.0.delay_micros(us);
    }
}

impl embedded_hal::blocking::delay::DelayMs<u8> for OldDelay {
    fn delay_ms(&mut self, ms: u8) {
        self.0.delay_millis(ms as u32);
    }
}

impl embedded_hal::blocking::delay::DelayMs<u16> for OldDelay {
    fn delay_ms(&mut self, ms: u16) {
        self.0.delay_millis(ms as u32);
    }
}

impl embedded_hal::blocking::delay::DelayMs<u32> for OldDelay {
    fn delay_ms(&mut self, ms: u32) {
        self.0.delay_millis(ms);
    }
}

struct OldPin<'a>(Flex<'a>);

impl<'a> embedded_hal::digital::v2::InputPin for OldPin<'a> {
    type Error = core::convert::Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }
}

impl<'a> embedded_hal::digital::v2::OutputPin for OldPin<'a> {
    type Error = core::convert::Infallible;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }
}

// =========================================================
// STRUKTUR DATA & STATE
// =========================================================

#[derive(Clone, Default)]
struct SensorData {
    mq6: f32, mq135: f32, mq3: f32, mq7: f32,
    tgs2611: f32, tgs2602: f32, tgs2600: f32, tgs2620: f32,
    dht1_temp: f32, dht1_hum: f32, dht2_temp: f32, dht2_hum: f32,
}

struct SystemState {
    running: bool,
    driver1_active: bool,
    driver2_active: bool,
    cycle_start_ms: u64,
    elapsed_ticks: u64,
    last_sensor_read_ms: u64,
    sensors: SensorData,
}

impl SystemState {
    fn new() -> Self {
        Self {
            running: false,
            driver1_active: false,
            driver2_active: false,
            cycle_start_ms: 0,
            elapsed_ticks: 0,
            last_sensor_read_ms: 0,
            sensors: SensorData::default(),
        }
    }
}

// =========================================================
// HELPER FUNCTIONS
// =========================================================

fn millis() -> u64 {
    Instant::now().duration_since_epoch().as_millis()
}

fn get_elapsed_time(state: &SystemState) -> u64 {
    state.elapsed_ticks
}

fn get_inlet_time(state: &SystemState) -> u64 {
    let e = state.elapsed_ticks;
    if e >= 71 && e <= 170 { e - 70 } else if e > 170 { 100 } else { 0 }
}

fn get_outlet_time(state: &SystemState) -> u64 {
    let e = state.elapsed_ticks;
    if e >= 231 && e <= 360 { e - 230 } else if e > 360 { 130 } else { 0 }
}

// Fungsi cetak float hemat RAM Stack & aman untuk bare-metal
fn print_sensor_float(val: f32) {
    let int_part = val as i32;
    let frac_int = (((val - int_part as f32) * 100.0).abs() + 0.5) as i32;
    let (display_int, display_frac) = if frac_int >= 100 {
        (int_part + 1, 0)
    } else {
        (int_part, frac_int)
    };
    print!("{}.{:02}", display_int, display_frac);
}

fn send_json(state: &SystemState) {
    let s = &state.sensors;
    print!("{{\"sensors\":{{\"mq6\":");
    print_sensor_float(s.mq6);
    print!(",\"mq135\":");
    print_sensor_float(s.mq135);
    print!(",\"mq3\":");
    print_sensor_float(s.mq3);
    print!(",\"mq7\":");
    print_sensor_float(s.mq7);
    print!(",\"tgs2611\":");
    print_sensor_float(s.tgs2611);
    print!(",\"tgs2602\":");
    print_sensor_float(s.tgs2602);
    print!(",\"tgs2600\":");
    print_sensor_float(s.tgs2600);
    print!(",\"tgs2620\":");
    print_sensor_float(s.tgs2620);
    print!(",\"dht1_temp\":");
    print_sensor_float(s.dht1_temp);
    print!(",\"dht1_hum\":");
    print_sensor_float(s.dht1_hum);
    print!(",\"dht2_temp\":");
    print_sensor_float(s.dht2_temp);
    print!(",\"dht2_hum\":");
    print_sensor_float(s.dht2_hum);
    print!("}},\"status\":{{\"inlet_pump\":{},\"inlet_valve\":{},", state.driver1_active, state.driver1_active);
    print!("\"outlet_pump\":{},\"outlet_valve\":{},", state.driver2_active, state.driver2_active);
    print!("\"inlet_time\":{},\"outlet_time\":{},\"elapsed_time\":{}}},", get_inlet_time(state), get_outlet_time(state), get_elapsed_time(state));
    println!("\"timestamp\":{}}}", millis());
}

fn ads_config(channel: u8) -> [u8; 3] {
    let mux: u16 = match channel {
        0 => 0b100,
        1 => 0b101,
        2 => 0b110,
        3 => 0b111,
        _ => 0b100,
    };
    let high: u8 = (0b1_000_000_1u8) | ((mux as u8) << 4);
    let low: u8 = 0b1000_0011;
    [ADS_REG_CONFIG, high, low]
}

fn ads_read_single(
    i2c: &mut I2c<'_, esp_hal::Blocking>,
    addr: u8,
    channel: u8,
    delay: &Delay,
) -> Result<i16, esp_hal::i2c::master::Error> {
    let cfg = ads_config(channel);
    i2c.write(addr, &cfg)?;
    delay.delay_millis(12);
    i2c.write(addr, &[ADS_REG_CONVERT])?;
    let mut buf = [0u8; 2];
    i2c.read(addr, &mut buf)?;
    Ok(i16::from_be_bytes(buf))
}

fn raw_to_mv(raw: i16) -> f32 {
    (raw as f32) * 6144.0 / 32767.0
}

static mut LAST_I2C_ERR_MS: u64 = 0;

fn read_sensors(state: &mut SystemState, i2c: &mut I2c<'_, esp_hal::Blocking>, delay: &Delay) {
    let mut read_ch = |addr: u8, ch: u8, name: &'static str| -> f32 {
        match ads_read_single(i2c, addr, ch, delay) {
            Ok(raw) => raw_to_mv(raw),
            Err(e) => {
                let now = millis();
                unsafe {
                    if now.saturating_sub(LAST_I2C_ERR_MS) > 3000 {
                        println!(">>> [I2C ERROR] {} (addr 0x{:02X}, ch {}): {:?}", name, addr, ch, e);
                        LAST_I2C_ERR_MS = now;
                    }
                }
                0.0
            }
        }
    };

    // ADS Kanan (0x48): A0 = MQ6, A1 = MQ135, A2 = MQ7, A3 = MQ3
    let mq6   = read_ch(ADS1_ADDR, 0, "MQ6");
    let mq135 = read_ch(ADS1_ADDR, 1, "MQ135");
    let mq7   = read_ch(ADS1_ADDR, 2, "MQ7");
    let mq3   = read_ch(ADS1_ADDR, 3, "MQ3");

    // ADS Kiri (0x49): A0 = 2602, A1 = 2620, A2 = 2611, A3 = 2600
    let tgs2602 = read_ch(ADS2_ADDR, 0, "TGS2602");
    let tgs2620 = read_ch(ADS2_ADDR, 1, "TGS2620");
    let tgs2611 = read_ch(ADS2_ADDR, 2, "TGS2611");
    let tgs2600 = read_ch(ADS2_ADDR, 3, "TGS2600");

    state.sensors = SensorData {
        mq6,
        mq135,
        mq3,
        mq7,
        tgs2611,
        tgs2602,
        tgs2600,
        tgs2620,
        ..state.sensors
    };
}

// =========================================================
// ENTRY POINT
// =========================================================

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    // 1. Inisialisasi UART0 pada pin standard TX=GPIO43, RX=GPIO44 untuk komunikasi GUI (COM8)
    let mut uart = Uart::new(peripherals.UART0, esp_hal::uart::Config::default())
        .expect("Failed to initialize UART0")
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    // 2. Inisialisasi I2C (100 kHz untuk kestabilan sinyal level shifter)
    // SDA: ESP32 GPIO 8 -> Level Shifter LV1 -> HV1 (SDA ADS Kanan & Kiri)
    // SCL: ESP32 GPIO 9 -> Level Shifter LV2 -> HV2 (SCL ADS Kanan & Kiri)
    let i2c_config = Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Failed to initialize I2C")
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9);

    println!(">>> System ready (no_std - Tanpa Motor Driver)");

    // Test koneksi ADS Kanan (0x48) & ADS Kiri (0x49)
    let ads1_ok = i2c.write(ADS1_ADDR, &[ADS_REG_CONVERT]).is_ok();
    let ads2_ok = i2c.write(ADS2_ADDR, &[ADS_REG_CONVERT]).is_ok();
    println!(">>> [I2C STATUS] ADS Kanan (0x48): {}", if ads1_ok { "TERHUBUNG (OK)" } else { "TIDAK TERDETEKSI" });
    println!(">>> [I2C STATUS] ADS Kiri  (0x49): {}", if ads2_ok { "TERHUBUNG (OK)" } else { "TIDAK TERDETEKSI" });

    // 3. Inisialisasi DHT22 pins menggunakan Open Drain & Flex Driver
    let mut dht1_flex = Flex::new(peripherals.GPIO10);
    dht1_flex.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
    dht1_flex.apply_output_config(&OutputConfig::default().with_drive_mode(DriveMode::OpenDrain));
    dht1_flex.set_input_enable(true);
    dht1_flex.set_output_enable(true);
    let mut dht1_pin = OldPin(dht1_flex);

    let mut dht2_flex = Flex::new(peripherals.GPIO11);
    dht2_flex.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
    dht2_flex.apply_output_config(&OutputConfig::default().with_drive_mode(DriveMode::OpenDrain));
    dht2_flex.set_input_enable(true);
    dht2_flex.set_output_enable(true);
    let mut dht2_pin = OldPin(dht2_flex);

    let mut state = SystemState::new();
    let mut cmd_buf = [0u8; 64];
    let mut cmd_len = 0;

    loop {
        let now = millis();
        // 4. BACA COMMAND DARI UART0 SECARA NON-BLOCKING (COM8)
        let mut rx_buf = [0u8; 1];
        while let Ok(1) = uart.read_buffered(&mut rx_buf) {
            let b = rx_buf[0];
            if b == b'\n' || b == b'\r' {
                if cmd_len > 0 {
                    // Konversi buffer ke string slice
                    if let Ok(cmd_str) = core::str::from_utf8(&cmd_buf[..cmd_len]) {
                        let command = cmd_str.trim();

                        // Perintah START dari GUI
                        if (command == "START" || command == "start") && !state.running {
                            println!(">>> START");
                            state.running = true;
                            state.cycle_start_ms = millis();
                            state.elapsed_ticks = 0;
                            state.driver1_active = false;
                            state.driver2_active = false;

                            println!(">>> Fase 1: Baseline (1-70s)");
                            read_sensors(&mut state, &mut i2c, &delay);
                            send_json(&state);
                            state.last_sensor_read_ms = millis();
                        }
                        // Perintah STOP dari GUI
                        else if (command == "STOP" || command == "stop") && state.running {
                            println!(">>> STOP");
                            state.running = false;
                            state.driver1_active = false;
                            state.driver2_active = false;
                            send_json(&state);
                        }
                    }
                    cmd_len = 0;
                }
            } else if cmd_len < cmd_buf.len() {
                cmd_buf[cmd_len] = b;
                cmd_len += 1;
            }
        }

        // 5. BACA SENSOR SETIAP 1 DETIK
        if now.saturating_sub(state.last_sensor_read_ms) >= 1000 {
            if state.running {
                state.elapsed_ticks += 1;
            }
            let mut old_delay = OldDelay(Delay::new());
            
            // Baca DHT22 Sensor 1
            if let Ok(reading) = dht22::Reading::read(&mut old_delay, &mut dht1_pin) {
                state.sensors.dht1_temp = reading.temperature;
                state.sensors.dht1_hum = reading.relative_humidity;
            }
            
            // Baca DHT22 Sensor 2
            if let Ok(reading) = dht22::Reading::read(&mut old_delay, &mut dht2_pin) {
                state.sensors.dht2_temp = reading.temperature;
                state.sensors.dht2_hum = reading.relative_humidity;
            }

            read_sensors(&mut state, &mut i2c, &delay);
            send_json(&state);

            // Log status siklus secara human-readable untuk pemantauan serial terminal
            println!(">>> STATUS SIKLUS - Inlet: {} | Outlet: {} (Elapsed: {}s)",
                if state.driver1_active { "ON" } else { "OFF" },
                if state.driver2_active { "ON" } else { "OFF" },
                get_elapsed_time(&state)
            );

            state.last_sensor_read_ms = now;
        }

        // 6. LOGIKA STATE SIKLUS (Virtual Timeline untuk GUI)
        if state.running {
            let elapsed = get_elapsed_time(&state);

            if elapsed >= 1 && elapsed <= 70 {
                state.driver1_active = false;
                state.driver2_active = false;
            } else if elapsed >= 71 && elapsed <= 170 {
                if !state.driver1_active {
                    state.driver1_active = true;
                    state.driver2_active = false;
                    println!(">>> Fase 2: Inlet ON (71-170s) - 100 detik");
                }
            } else if elapsed >= 171 && elapsed <= 230 {
                if state.driver1_active || state.driver2_active {
                    println!(">>> Fase Sensing: Semua diam (171-230s) - 60 detik");
                }
                state.driver1_active = false;
                state.driver2_active = false;
            } else if elapsed >= 231 && elapsed <= 360 {
                if !state.driver2_active {
                    println!(">>> Fase 3: Outlet ON (231-360s) - 130 detik");
                }
                state.driver1_active = false;
                state.driver2_active = true;
            } else if elapsed > 360 {
                println!(">>> Cycle complete! Restarting cycle...");
                state.elapsed_ticks = 0;
                state.driver1_active = false;
                state.driver2_active = false;
                println!(">>> Fase 1: Baseline (1-70s)");
                send_json(&state);
            }
        }

        delay.delay_millis(1);
    }
}
