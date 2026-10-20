// =========================================================
// ELECTRONIC NOSE - RUST VERSION
// ✅ Single click START - No delay
// ✅ Database starts from time_seconds = 1
// ✅ Immediate graph and data recording
// ✅ FIXED: PWM control for L298N motor driver
// ✅ 3 FASE: OFF 70s -> INLET 100s -> OUTLET 130s
// =========================================================

use esp_idf_hal::{
    delay::FreeRtos,
    gpio::*,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::*, // Menggantikan prelude untuk kHz()
};
use esp_idf_svc::systime::EspSystemTime;

use std::sync::{Arc, Mutex};

use dht_sensor::{dht22, DhtReading};

// ─── ADS1115 I2C addresses ───────────────────────────────
const ADS1_ADDR: u8 = 0x48;
const ADS2_ADDR: u8 = 0x49;

// ─── ADS1115 register / config constants ─────────────────
const ADS_REG_CONVERT: u8 = 0x00;
const ADS_REG_CONFIG: u8 = 0x01;

/// Build ADS1115 config word (gain = TWOTHIRDS ±6.144 V, single-shot)
///  OS=1 (start), MUX = single channel, PGA = 000 (6.144V),
///  MODE = 1 (single shot), DR = 100 (128 SPS), COMP disabled
fn ads_config(channel: u8) -> [u8; 3] {
    let mux: u16 = match channel {
        0 => 0b100, // AIN0 vs GND
        1 => 0b101,
        2 => 0b110,
        3 => 0b111,
        _ => 0b100,
    };
    // MSB: OS=1, MUX[2:0], PGA=000 (6.144V), MODE=1
    let high: u8 = (0b1_000_000_1u8) | ((mux as u8) << 4);
    // LSB: DR=100 (128SPS), COMP_MODE=0, COMP_POL=0, COMP_LAT=0, COMP_QUE=11
    let low: u8 = 0b1000_0011;
    [ADS_REG_CONFIG, high, low]
}

/// Read a single-ended channel from ADS1115 via I2C, returns raw i16
fn ads_read_single(
    i2c: &mut I2cDriver,
    addr: u8,
    channel: u8,
) -> Result<i16, esp_idf_hal::i2c::I2cError> {
    // 1. Write config register to start conversion
    let cfg = ads_config(channel);
    i2c.write(addr, &cfg, 50)?;

    // 2. Wait for conversion (single-shot ~8ms @ 128SPS)
    FreeRtos::delay_ms(10);

    // 3. Point to conversion register
    i2c.write(addr, &[ADS_REG_CONVERT], 50)?;

    // 4. Read 2 bytes
    let mut buf = [0u8; 2];
    i2c.read(addr, &mut buf, 50)?;

    Ok(i16::from_be_bytes(buf))
}

/// Convert raw i16 to millivolts (GAIN_TWOTHIRDS = 6.144V FSR, 32767 counts)
fn raw_to_mv(raw: i16) -> f32 {
    (raw as f32) * 6144.0 / 32767.0
}

// ─── Sensor data ─────────────────────────────────────────
#[derive(Clone, Default)]
struct SensorData {
    mq6: f32,
    mq135: f32,
    mq3: f32,
    mq7: f32,
    tgs2611: f32,
    tgs2602: f32,
    tgs2600: f32,
    tgs2620: f32,
    dht1_temp: f32,
    dht1_hum: f32,
    dht2_temp: f32,
    dht2_hum: f32,
}

// ─── System state ─────────────────────────────────────────
struct SystemState {
    running: bool,
    driver1_active: bool, // Inlet  (Driver A)
    driver2_active: bool, // Outlet (Driver B)
    cycle_start_ms: u64,
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
            last_sensor_read_ms: 0,
            sensors: SensorData::default(),
        }
    }
}

// ─── Timing helpers ───────────────────────────────────────
fn millis() -> u64 {
    EspSystemTime {}.now().as_millis() as u64
}

fn get_elapsed_time(state: &SystemState) -> u64 {
    if !state.running {
        return 0;
    }
    let elapsed_ms = millis().saturating_sub(state.cycle_start_ms);
    let elapsed_sec = elapsed_ms / 1000;
    if elapsed_sec == 0 { 1 } else { elapsed_sec }
}

fn get_inlet_time(state: &SystemState) -> u64 {
    let elapsed = get_elapsed_time(state);
    if elapsed >= 71 && elapsed <= 170 {
        elapsed - 70
    } else if elapsed > 170 {
        100
    } else {
        0
    }
}

fn get_outlet_time(state: &SystemState) -> u64 {
    let elapsed = get_elapsed_time(state);
    if elapsed >= 171 && elapsed <= 300 {
        elapsed - 170
    } else if elapsed > 300 {
        130
    } else {
        0
    }
}

// ─── JSON output (via UART0 / stdout) ────────────────────
fn send_json(state: &SystemState) {
    let s = &state.sensors;
    println!(
        "{{\"sensors\":{{\"mq6\":{:.2},\"mq135\":{:.2},\"mq3\":{:.2},\"mq7\":{:.2},\
         \"tgs2611\":{:.2},\"tgs2602\":{:.2},\"tgs2600\":{:.2},\"tgs2620\":{:.2},\
         \"dht1_temp\":{:.2},\"dht1_hum\":{:.2},\"dht2_temp\":{:.2},\"dht2_hum\":{:.2}}},\
         \"status\":{{\"inlet_pump\":{},\"inlet_valve\":{},\
         \"outlet_pump\":{},\"outlet_valve\":{},\
         \"inlet_time\":{},\"outlet_time\":{},\"elapsed_time\":{}}},\
         \"timestamp\":{}}}",
        s.mq6, s.mq135, s.mq3, s.mq7,
        s.tgs2611, s.tgs2602, s.tgs2600, s.tgs2620,
        s.dht1_temp, s.dht1_hum, s.dht2_temp, s.dht2_hum,
        state.driver1_active, state.driver1_active,
        state.driver2_active, state.driver2_active,
        get_inlet_time(state),
        get_outlet_time(state),
        get_elapsed_time(state),
        millis(),
    );
}

// ─── Motor driver helpers ─────────────────────────────
macro_rules! driver_start {
    ($in1:expr, $in2:expr, $in3:expr, $in4:expr) => {{
        $in1.set_high().ok();
        $in2.set_low().ok();
        $in3.set_high().ok();
        $in4.set_low().ok();
    }};
}

macro_rules! driver_stop {
    ($in1:expr, $in2:expr, $in3:expr, $in4:expr) => {{
        $in1.set_low().ok();
        $in2.set_low().ok();
        $in3.set_low().ok();
        $in4.set_low().ok();
    }};
}

// ─── Main ─────────────────────────────────────────────────
fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;

    // ── I2C (SDA=8, SCL=9) ───────────────────────────────
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        pins.gpio8,  // SDA
        pins.gpio9,  // SCL
        &I2cConfig::new().baudrate(400_u32.kHz().into()), // FIX: tambah _u32
    )?;
    let i2c = Arc::new(Mutex::new(i2c));

    // ── Driver A – Inlet ────
    let mut in1_a = PinDriver::output(pins.gpio5)?;
    let mut in2_a = PinDriver::output(pins.gpio17)?;
    let mut in3_a = PinDriver::output(pins.gpio14)?;
    let mut in4_a = PinDriver::output(pins.gpio19)?;

    // ── Driver B – Outlet ───
    let mut in1_b = PinDriver::output(pins.gpio16)?;
    let mut in2_b = PinDriver::output(pins.gpio4)?;
    let mut in3_b = PinDriver::output(pins.gpio45)?;
    let mut in4_b = PinDriver::output(pins.gpio20)?;

    driver_stop!(in1_a, in2_a, in3_a, in4_a);
    driver_stop!(in1_b, in2_b, in3_b, in4_b);

    // ── DHT22 sensors (GPIO 10 and 11) ────────────────────
    // FIX: Pull parameter ditambahkan langsung ke constructor
    let mut dht1_pin = PinDriver::input_output_od(pins.gpio10, Pull::Up)?;
    let mut dht2_pin = PinDriver::input_output_od(pins.gpio11, Pull::Up)?;

    println!(">>> System ready");
unsafe {
        esp_idf_sys::uart_driver_install(
            esp_idf_sys::uart_port_t_UART_NUM_0,
            256, // Ukuran buffer RX (Penerima)
            0,   // Ukuran buffer TX (0 karena kita pakai println! bawaan)
            0,
            core::ptr::null_mut(),
            0,
        );
    }
    
    let mut state = SystemState::new();
    let mut cmd_buf = String::new();

    loop {
        let now = millis();

        {
            let ch = unsafe {
                let mut byte: u8 = 0;
                let n = esp_idf_sys::uart_read_bytes(
                    esp_idf_sys::uart_port_t_UART_NUM_0,
                    &mut byte as *mut u8 as *mut core::ffi::c_void,
                    1,
                    0,
                );
                if n > 0 { Some(byte) } else { None }
            };

            if let Some(b) = ch {
                if b == b'\n' || b == b'\r' {
                    let command = cmd_buf.trim().to_uppercase();

                    if command == "START" && !state.running {
                        println!(">>> START");
                        state.running = true;
                        state.cycle_start_ms = millis();

                        driver_stop!(in1_a, in2_a, in3_a, in4_a);
                        driver_stop!(in1_b, in2_b, in3_b, in4_b);
                        state.driver1_active = false;
                        state.driver2_active = false;

                        println!(">>> Fase 1: Semua mati (1-70s)");

                        read_sensors(&mut state, &i2c);
                        send_json(&state);
                        state.last_sensor_read_ms = millis();
                        println!(">>> First data sent!");

                    } else if command == "STOP" && state.running {
                        println!(">>> STOP");
                        state.running = false;

                        driver_stop!(in1_a, in2_a, in3_a, in4_a);
                        driver_stop!(in1_b, in2_b, in3_b, in4_b);
                        state.driver1_active = false;
                        state.driver2_active = false;

                        send_json(&state);
                    }

                    cmd_buf.clear();
                } else {
                    cmd_buf.push(b as char);
                }
            }
        }

        if now.saturating_sub(state.last_sensor_read_ms) >= 5000 {
            let mut delay = esp_idf_hal::delay::Ets;
            if let Ok(reading) = dht22::Reading::read(&mut delay, &mut dht1_pin) {
                state.sensors.dht1_temp = reading.temperature;
                state.sensors.dht1_hum = reading.relative_humidity; // FIX: relative_humidity
            }
            if let Ok(reading) = dht22::Reading::read(&mut delay, &mut dht2_pin) {
                state.sensors.dht2_temp = reading.temperature;
                state.sensors.dht2_hum = reading.relative_humidity; // FIX: relative_humidity
            }

            read_sensors(&mut state, &i2c);
            send_json(&state);
            state.last_sensor_read_ms = now;
        }

        if state.running {
            let elapsed = get_elapsed_time(&state);

            if elapsed >= 1 && elapsed <= 70 {
                if state.driver1_active {
                    driver_stop!(in1_a, in2_a, in3_a, in4_a);
                    state.driver1_active = false;
                }
                if state.driver2_active {
                    driver_stop!(in1_b, in2_b, in3_b, in4_b);
                    state.driver2_active = false;
                }

            } else if elapsed >= 71 && elapsed <= 170 {
                if !state.driver1_active {
                    driver_start!(in1_a, in2_a, in3_a, in4_a);
                    state.driver1_active = true;
                    state.driver2_active = false;
                    println!(">>> Fase 2: Inlet ON (71-170s) - 100 detik");
                }

            } else if elapsed >= 171 && elapsed <= 300 {
                if state.driver1_active {
                    driver_stop!(in1_a, in2_a, in3_a, in4_a);
                    state.driver1_active = false;
                }
                if !state.driver2_active {
                    driver_start!(in1_b, in2_b, in3_b, in4_b);
                    state.driver2_active = true;
                    println!(">>> Fase 3: Outlet ON (171-300s) - 130 detik");
                }

            } else if elapsed > 300 {
                println!(">>> Cycle complete!");
                state.running = false;

                driver_stop!(in1_a, in2_a, in3_a, in4_a);
                driver_stop!(in1_b, in2_b, in3_b, in4_b);
                state.driver1_active = false;
                state.driver2_active = false;

                send_json(&state);
            }
        }

        FreeRtos::delay_ms(1);
    }
}

// ─── Sensor reading ──────────────────────────────────────
fn read_sensors(state: &mut SystemState, i2c: &Arc<Mutex<I2cDriver>>) {
    let mut bus = i2c.lock().unwrap();

    // ADS Kanan (0x48): A0 = MQ6, A1 = MQ135, A2 = MQ7, A3 = MQ3
    let raw_mq6    = ads_read_single(&mut bus, ADS1_ADDR, 0).unwrap_or(0);
    let raw_mq135  = ads_read_single(&mut bus, ADS1_ADDR, 1).unwrap_or(0);
    let raw_mq7    = ads_read_single(&mut bus, ADS1_ADDR, 2).unwrap_or(0);
    let raw_mq3    = ads_read_single(&mut bus, ADS1_ADDR, 3).unwrap_or(0);

    // ADS Kiri (0x49): A0 = 2602, A1 = 2620, A2 = 2611, A3 = 2600
    let raw_tgs2602 = ads_read_single(&mut bus, ADS2_ADDR, 0).unwrap_or(0);
    let raw_tgs2620 = ads_read_single(&mut bus, ADS2_ADDR, 1).unwrap_or(0);
    let raw_tgs2611 = ads_read_single(&mut bus, ADS2_ADDR, 2).unwrap_or(0);
    let raw_tgs2600 = ads_read_single(&mut bus, ADS2_ADDR, 3).unwrap_or(0);

    // FIX: Gunakan ..state.sensors agar nilai DHT yang dibaca terpisah tidak tertimpa nol
    state.sensors = SensorData {
        mq6:     raw_to_mv(raw_mq6),
        mq135:   raw_to_mv(raw_mq135),
        mq3:     raw_to_mv(raw_mq3),
        mq7:     raw_to_mv(raw_mq7),
        tgs2611: raw_to_mv(raw_tgs2611),
        tgs2602: raw_to_mv(raw_tgs2602),
        tgs2600: raw_to_mv(raw_tgs2600),
        tgs2620: raw_to_mv(raw_tgs2620),
        ..state.sensors
    };
}