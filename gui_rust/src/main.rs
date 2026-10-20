use crossbeam_channel::{unbounded, Receiver, Sender};
use eframe::egui;
use egui_plot::{HLine, Legend, Line, LineStyle, Plot, PlotPoints};
use serde::Deserialize;
use serialport::SerialPortType;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Deserialize, Clone)]
struct Sensors {
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

#[derive(Debug, Deserialize, Clone)]
struct Status {
    #[allow(dead_code)]
    inlet_pump: bool,
    #[allow(dead_code)]
    inlet_valve: bool,
    #[allow(dead_code)]
    outlet_pump: bool,
    #[allow(dead_code)]
    outlet_valve: bool,
    #[allow(dead_code)]
    inlet_time: u64,
    #[allow(dead_code)]
    outlet_time: u64,
    elapsed_time: u64,
}

#[derive(Debug, Deserialize, Clone)]
struct Payload {
    sensors: Sensors,
    status: Status,
    timestamp: u64,
}

#[derive(Clone)]
struct ChartData {
    mq6: Vec<[f64; 2]>,
    mq135: Vec<[f64; 2]>,
    mq3: Vec<[f64; 2]>,
    mq7: Vec<[f64; 2]>,
    tgs2611: Vec<[f64; 2]>,
    tgs2602: Vec<[f64; 2]>,
    tgs2600: Vec<[f64; 2]>,
    tgs2620: Vec<[f64; 2]>,
}

impl ChartData {
    fn new() -> Self {
        Self {
            mq6: Vec::new(),
            mq135: Vec::new(),
            mq3: Vec::new(),
            mq7: Vec::new(),
            tgs2611: Vec::new(),
            tgs2602: Vec::new(),
            tgs2600: Vec::new(),
            tgs2620: Vec::new(),
        }
    }

    fn push(&mut self, time: f64, sensors: &Sensors) {
        let push_limit = |vec: &mut Vec<[f64; 2]>, val: f32| {
            vec.push([time, val as f64]);
        };

        push_limit(&mut self.mq6, sensors.mq6);
        push_limit(&mut self.mq135, sensors.mq135);
        push_limit(&mut self.mq3, sensors.mq3);
        push_limit(&mut self.mq7, sensors.mq7);
        push_limit(&mut self.tgs2611, sensors.tgs2611);
        push_limit(&mut self.tgs2602, sensors.tgs2602);
        push_limit(&mut self.tgs2600, sensors.tgs2600);
        push_limit(&mut self.tgs2620, sensors.tgs2620);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NormMethod {
    RsR0,      // Rs / R0 (Resistansi Sensor - Turun alami saat ada gas)
    R0Rs,      // R0 / Rs = Gs / G0 (Konduktansi Sensor - Puncak Naik)
    VsV0,      // Vs / V0 (Rasio Tegangan - Puncak Naik)
    DeltaV,    // (Vs - V0) / V0 (Respon Relatif ΔV/V0)
}

impl NormMethod {
    fn label(&self) -> &'static str {
        match self {
            NormMethod::RsR0 => "Rs/R0 (Resistansi - Turun Alami)",
            NormMethod::R0Rs => "R0/Rs (Konduktansi - Puncak Naik)",
            NormMethod::VsV0 => "Vs/V0 (Rasio Tegangan - Puncak Naik)",
            NormMethod::DeltaV => "(Vs - V0)/V0 (Respon Relatif)",
        }
    }

    fn y_axis_label(&self) -> &'static str {
        match self {
            NormMethod::RsR0 => "Normalized Resistance (Rs / R0)",
            NormMethod::R0Rs => "Normalized Conductance (R0 / Rs)",
            NormMethod::VsV0 => "Rasio Tegangan (Vs / V0)",
            NormMethod::DeltaV => "Respon Relatif (ΔV / V0)",
        }
    }

    fn calculate(&self, vs: f64, v0: f64, vcc: f64) -> f64 {
        if v0 <= 0.0 {
            return match self {
                NormMethod::DeltaV => 0.0,
                _ => 1.0,
            };
        }
        match self {
            NormMethod::RsR0 => {
                let vcc = vcc.max(1000.0);
                let vs_c = vs.clamp(1.0, vcc - 1.0);
                let v0_c = v0.clamp(1.0, vcc - 1.0);
                let rs = (vcc - vs_c) / vs_c;
                let r0 = (vcc - v0_c) / v0_c;
                if r0.abs() < 1e-6 {
                    1.0
                } else {
                    rs / r0
                }
            }
            NormMethod::R0Rs => {
                let vcc = vcc.max(1000.0);
                let vs_c = vs.clamp(1.0, vcc - 1.0);
                let v0_c = v0.clamp(1.0, vcc - 1.0);
                let rs = (vcc - vs_c) / vs_c;
                let r0 = (vcc - v0_c) / v0_c;
                if rs.abs() < 1e-6 {
                    1.0
                } else {
                    r0 / rs
                }
            }
            NormMethod::VsV0 => {
                vs / v0.max(1.0)
            }
            NormMethod::DeltaV => {
                (vs - v0) / v0.max(1.0)
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    Normalized, // Grafik Rs/R0 (Satu Sumbu Y)
    Voltage,    // Grafik Tegangan Mentah (mV)
    Both,       // Split Keduanya
}

#[derive(Clone, Debug)]
pub struct Baseline {
    pub sum_mq6: f64,
    pub sum_mq135: f64,
    pub sum_mq3: f64,
    pub sum_mq7: f64,
    pub sum_tgs2611: f64,
    pub sum_tgs2602: f64,
    pub sum_tgs2600: f64,
    pub sum_tgs2620: f64,
    pub count: usize,

    pub v0_mq6: f64,
    pub v0_mq135: f64,
    pub v0_mq3: f64,
    pub v0_mq7: f64,
    pub v0_tgs2611: f64,
    pub v0_tgs2602: f64,
    pub v0_tgs2600: f64,
    pub v0_tgs2620: f64,

    pub is_manual: bool,
    pub locked: bool,
}

impl Baseline {
    fn new() -> Self {
        Self {
            sum_mq6: 0.0,
            sum_mq135: 0.0,
            sum_mq3: 0.0,
            sum_mq7: 0.0,
            sum_tgs2611: 0.0,
            sum_tgs2602: 0.0,
            sum_tgs2600: 0.0,
            sum_tgs2620: 0.0,
            count: 0,

            v0_mq6: 0.0,
            v0_mq135: 0.0,
            v0_mq3: 0.0,
            v0_mq7: 0.0,
            v0_tgs2611: 0.0,
            v0_tgs2602: 0.0,
            v0_tgs2600: 0.0,
            v0_tgs2620: 0.0,

            is_manual: false,
            locked: false,
        }
    }

    fn reset_auto(&mut self) {
        self.sum_mq6 = 0.0;
        self.sum_mq135 = 0.0;
        self.sum_mq3 = 0.0;
        self.sum_mq7 = 0.0;
        self.sum_tgs2611 = 0.0;
        self.sum_tgs2602 = 0.0;
        self.sum_tgs2600 = 0.0;
        self.sum_tgs2620 = 0.0;
        self.count = 0;
        self.locked = false;
        self.is_manual = false;
    }

    fn set_manual_from_sensors(&mut self, s: &Sensors) {
        self.v0_mq6 = s.mq6 as f64;
        self.v0_mq135 = s.mq135 as f64;
        self.v0_mq3 = s.mq3 as f64;
        self.v0_mq7 = s.mq7 as f64;
        self.v0_tgs2611 = s.tgs2611 as f64;
        self.v0_tgs2602 = s.tgs2602 as f64;
        self.v0_tgs2600 = s.tgs2600 as f64;
        self.v0_tgs2620 = s.tgs2620 as f64;
        self.is_manual = true;
        self.locked = true;
    }

    fn update(&mut self, s: &Sensors, elapsed_sec: u64) {
        if self.is_manual {
            return;
        }

        // Akumulasi data baseline selama Fase 1 (1 - 70 detik)
        if (1..=70).contains(&elapsed_sec) {
            self.sum_mq6 += s.mq6 as f64;
            self.sum_mq135 += s.mq135 as f64;
            self.sum_mq3 += s.mq3 as f64;
            self.sum_mq7 += s.mq7 as f64;
            self.sum_tgs2611 += s.tgs2611 as f64;
            self.sum_tgs2602 += s.tgs2602 as f64;
            self.sum_tgs2600 += s.tgs2600 as f64;
            self.sum_tgs2620 += s.tgs2620 as f64;
            self.count += 1;

            let c = self.count as f64;
            self.v0_mq6 = self.sum_mq6 / c;
            self.v0_mq135 = self.sum_mq135 / c;
            self.v0_mq3 = self.sum_mq3 / c;
            self.v0_mq7 = self.sum_mq7 / c;
            self.v0_tgs2611 = self.sum_tgs2611 / c;
            self.v0_tgs2602 = self.sum_tgs2602 / c;
            self.v0_tgs2600 = self.sum_tgs2600 / c;
            self.v0_tgs2620 = self.sum_tgs2620 / c;
        } else if elapsed_sec > 70 {
            self.locked = true;
        } else if self.count == 0 {
            // Inisialisasi awal saat idle sebelum start
            self.v0_mq6 = s.mq6 as f64;
            self.v0_mq135 = s.mq135 as f64;
            self.v0_mq3 = s.mq3 as f64;
            self.v0_mq7 = s.mq7 as f64;
            self.v0_tgs2611 = s.tgs2611 as f64;
            self.v0_tgs2602 = s.tgs2602 as f64;
            self.v0_tgs2600 = s.tgs2600 as f64;
            self.v0_tgs2620 = s.tgs2620 as f64;
        }
    }
}

enum AppMsg {
    Data(Payload),
    Error(String),
}

struct ElectronicNoseApp {
    available_ports: Vec<String>,
    selected_port: String,
    connected: bool,
    baud_rate: u32,
    
    // Communication
    receiver: Option<Receiver<AppMsg>>,
    tx_command: Option<Sender<String>>,
    
    // State
    latest_payload: Option<Payload>,
    chart_data: ChartData,
    start_time: Instant,
    export_history: Vec<Payload>,
    connection_error: Option<String>,
    auto_scale: bool,

    // Normalization & View Settings
    baseline: Baseline,
    vcc_mv: f64,
    norm_method: NormMethod,
    view_mode: ViewMode,
}

impl ElectronicNoseApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Setup dark mode
        cc.egui_ctx.set_visuals(egui::Visuals::dark());

        let ports = Self::scan_ports();
        let mut app = Self {
            available_ports: ports.clone(),
            selected_port: String::new(),
            connected: false,
            baud_rate: 115200,
            receiver: None,
            tx_command: None,
            latest_payload: None,
            chart_data: ChartData::new(),
            start_time: Instant::now(),
            export_history: Vec::new(),
            connection_error: None,
            auto_scale: true,

            baseline: Baseline::new(),
            vcc_mv: 5000.0,
            norm_method: NormMethod::RsR0,
            view_mode: ViewMode::Normalized, // Default ke grafik normalized Rs/R0
        };
        
        // Prioritaskan port ESP32 (misal COM17 atau port USB pertama)
        if let Some(target) = ports.iter().find(|p| p.as_str() == "COM17") {
            app.selected_port = target.clone();
        } else if !ports.is_empty() && ports[0] != "No ports found" {
            app.selected_port = ports[0].clone();
        }
        
        app
    }

    fn scan_ports() -> Vec<String> {
        let mut usb_ports = Vec::new();
        let mut other_ports = Vec::new();
        if let Ok(available) = serialport::available_ports() {
            for port in available {
                match port.port_type {
                    SerialPortType::UsbPort(_) => {
                        usb_ports.push(port.port_name);
                    }
                    _ => {
                        other_ports.push(port.port_name);
                    }
                }
            }
        }
        usb_ports.sort();
        other_ports.sort();

        let mut ports = usb_ports;
        ports.extend(other_ports);

        if ports.is_empty() {
            ports.push("No ports found".to_string());
        }
        ports
    }

    fn connect(&mut self, ctx: egui::Context) {
        if self.selected_port.is_empty() || self.selected_port == "No ports found" {
            return;
        }

        let port_name = self.selected_port.clone();
        let baud_rate = self.baud_rate;

        match serialport::new(&port_name, baud_rate)
            .timeout(Duration::from_millis(100))
            .open()
        {
            Ok(mut port) => {
                // Pastikan DTR & RTS tidak menarik ESP32 ke bootloader
                let _ = port.write_data_terminal_ready(false);
                let _ = port.write_request_to_send(false);

                self.connection_error = None;
                let (tx, rx) = unbounded();
                let (tx_cmd, rx_cmd) = unbounded::<String>();
                self.receiver = Some(rx);
                self.tx_command = Some(tx_cmd);
                self.connected = true;
                self.start_time = Instant::now();
                self.chart_data = ChartData::new();
                self.export_history.clear();
                if !self.baseline.is_manual {
                    self.baseline.reset_auto();
                }

                // Start reader thread (langsung gunakan port tanpa try_clone)
                thread::spawn(move || {
                    let mut reader_port = port;
                    let mut line = String::new();
                    let mut buf = [0u8; 128];

                    loop {
                        match reader_port.read(&mut buf) {
                            Ok(n) if n > 0 => {
                                if let Ok(s) = std::str::from_utf8(&buf[..n]) {
                                    line.push_str(s);
                                    while let Some(pos) = line.find('\n') {
                                        let trimmed = line[..pos].trim();
                                        if !trimmed.is_empty() {
                                            println!("RAW SERIAL: {}", trimmed);
                                        }
                                        // Cari kurung kurawal pembuka dan penutup untuk payload JSON
                                        if let (Some(start), Some(end)) = (trimmed.find('{'), trimmed.rfind('}')) {
                                            if start < end {
                                                let json_slice = &trimmed[start..=end];
                                                match serde_json::from_str::<Payload>(json_slice) {
                                                    Ok(payload) => {
                                                        if tx.send(AppMsg::Data(payload)).is_err() {
                                                            break;
                                                        }
                                                        ctx.request_repaint(); // Wake up UI
                                                    }
                                                    Err(e) => {
                                                        println!("⚠️ JSON PARSE ERROR: {:?}\nINPUT: {}", e, json_slice);
                                                    }
                                                }
                                            }
                                        }
                                        line = line[pos + 1..].to_string();
                                    }
                                }
                            }
                            Ok(_) => {
                                thread::sleep(Duration::from_millis(10));
                            }
                            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                                // Timeout is expected
                            }
                            Err(e) => {
                                let _ = tx.send(AppMsg::Error(e.to_string()));
                                break;
                            }
                        }

                        // Check for commands to send
                        if let Ok(cmd) = rx_cmd.try_recv() {
                            let _ = reader_port.write_all(cmd.as_bytes());
                            let _ = reader_port.flush();
                        }
                    }
                });
            }
            Err(e) => {
                self.connection_error = Some(format!("{}: {}", port_name, e));
                println!("Failed to open port '{}': {}", port_name, e);
            }
        }
    }

    fn disconnect(&mut self) {
        self.connected = false;
        self.receiver = None;
        self.tx_command = None;
        self.connection_error = None;
    }

    fn get_current_phase(&self) -> String {
        if let Some(payload) = &self.latest_payload {
            let elapsed = payload.status.elapsed_time;
            match elapsed {
                1..=70 => "Fase 1: Baseline (1-70s)".to_string(),
                71..=170 => "Fase 2: Sampling / Inlet (71-170s)".to_string(),
                171..=230 => "Fase Sensing (171-230s)".to_string(),
                231..=360 => "Fase 3: Purging / Outlet (231-360s)".to_string(),
                _ => "Standby / Idle".to_string(),
            }
        } else {
            "Menunggu data...".to_string()
        }
    }

    fn export_to_csv(&self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("CSV Excel", &["csv"])
            .set_file_name("data_sensor.csv")
            .save_file()
        {
            if let Ok(mut wtr) = csv::WriterBuilder::new().delimiter(b';').from_path(path) {
                let _ = wtr.write_record(&[
                    "Timestamp (ms)", "Elapsed Time (s)", 
                    "MQ6 (mV)", "MQ135 (mV)", "MQ3 (mV)", "MQ7 (mV)", 
                    "TGS2611 (mV)", "TGS2602 (mV)", "TGS2600 (mV)", "TGS2620 (mV)", 
                    "MQ6_RsR0", "MQ135_RsR0", "MQ3_RsR0", "MQ7_RsR0", 
                    "TGS2611_RsR0", "TGS2602_RsR0", "TGS2600_RsR0", "TGS2620_RsR0", 
                    "DHT1 Temp", "DHT1 Hum", "DHT2 Temp", "DHT2 Hum"
                ]);
                for p in &self.export_history {
                    let s = &p.sensors;
                    let norm = |v: f32, v0: f64| -> String {
                        let val = self.norm_method.calculate(v as f64, v0, self.vcc_mv);
                        format!("{:.4}", val).replace('.', ",")
                    };
                    let _ = wtr.write_record(&[
                        p.timestamp.to_string(),
                        p.status.elapsed_time.to_string(),
                        format!("{:.2}", s.mq6).replace('.', ","),
                        format!("{:.2}", s.mq135).replace('.', ","),
                        format!("{:.2}", s.mq3).replace('.', ","),
                        format!("{:.2}", s.mq7).replace('.', ","),
                        format!("{:.2}", s.tgs2611).replace('.', ","),
                        format!("{:.2}", s.tgs2602).replace('.', ","),
                        format!("{:.2}", s.tgs2600).replace('.', ","),
                        format!("{:.2}", s.tgs2620).replace('.', ","),
                        norm(s.mq6, self.baseline.v0_mq6),
                        norm(s.mq135, self.baseline.v0_mq135),
                        norm(s.mq3, self.baseline.v0_mq3),
                        norm(s.mq7, self.baseline.v0_mq7),
                        norm(s.tgs2611, self.baseline.v0_tgs2611),
                        norm(s.tgs2602, self.baseline.v0_tgs2602),
                        norm(s.tgs2600, self.baseline.v0_tgs2600),
                        norm(s.tgs2620, self.baseline.v0_tgs2620),
                        format!("{:.1}", s.dht1_temp).replace('.', ","),
                        format!("{:.1}", s.dht1_hum).replace('.', ","),
                        format!("{:.1}", s.dht2_temp).replace('.', ","),
                        format!("{:.1}", s.dht2_hum).replace('.', ","),
                    ]);
                }
                let _ = wtr.flush();
            }
        }
    }

    fn render_normalized_plot(&self, ui: &mut egui::Ui, height: Option<f32>) {
        let mut plot = Plot::new("sensor_norm_plot")
            .legend(Legend::default())
            .x_axis_label("Waktu (detik)")
            .y_axis_label(self.norm_method.y_axis_label())
            .include_y(0.0)
            .include_y(1.2);

        if let Some(h) = height {
            plot = plot.height(h);
        }

        if self.auto_scale {
            plot = plot.auto_bounds(egui::Vec2b::new(true, true));
        }

        plot.show(ui, |plot_ui| {
            if self.chart_data.mq6.is_empty() {
                plot_ui.text(
                    egui_plot::Text::new(
                        egui_plot::PlotPoint::new(5.0, 1.0),
                        "📊 Menunggu data sensor untuk grafik Normalized Rs/R0...\n1. Pilih Port di sebelah kiri (misal: COM17)\n2. Klik Connect\n3. Klik START SIKLUS (Fase 1: 1-70s kalibrasi baseline R0 otomatis)",
                    )
                    .color(egui::Color32::from_rgb(180, 180, 180)),
                );
                return;
            }

            // Garis referensi baseline (Rs/R0 = 1.0 atau ΔV/V0 = 0.0)
            let baseline_y = if self.norm_method == NormMethod::DeltaV { 0.0 } else { 1.0 };
            plot_ui.hline(
                HLine::new(baseline_y)
                    .color(egui::Color32::from_rgb(140, 140, 140))
                    .style(LineStyle::Dashed { length: 8.0 })
                    .width(1.2),
            );

            let method = self.norm_method;
            let vcc = self.vcc_mv;
            let b = &self.baseline;

            let pts_mq6: PlotPoints = self.chart_data.mq6.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_mq6, vcc)])
                .collect();
            let pts_mq135: PlotPoints = self.chart_data.mq135.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_mq135, vcc)])
                .collect();
            let pts_mq7: PlotPoints = self.chart_data.mq7.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_mq7, vcc)])
                .collect();
            let pts_mq3: PlotPoints = self.chart_data.mq3.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_mq3, vcc)])
                .collect();
            let pts_tgs2602: PlotPoints = self.chart_data.tgs2602.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_tgs2602, vcc)])
                .collect();
            let pts_tgs2620: PlotPoints = self.chart_data.tgs2620.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_tgs2620, vcc)])
                .collect();
            let pts_tgs2611: PlotPoints = self.chart_data.tgs2611.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_tgs2611, vcc)])
                .collect();
            let pts_tgs2600: PlotPoints = self.chart_data.tgs2600.iter()
                .map(|&[t, v]| [t, method.calculate(v, b.v0_tgs2600, vcc)])
                .collect();

            // Semua 8 sensor digabung ke dalam SATU sumbu Y
            plot_ui.line(
                Line::new(pts_mq6)
                    .name("MQ-6 (A0 kanan)")
                    .color(egui::Color32::from_rgb(255, 99, 71))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_mq135)
                    .name("MQ-135 (A1 kanan)")
                    .color(egui::Color32::from_rgb(30, 144, 255))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_mq7)
                    .name("MQ-7 (A2 kanan)")
                    .color(egui::Color32::from_rgb(255, 193, 7))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_mq3)
                    .name("MQ-3 (A3 kanan)")
                    .color(egui::Color32::from_rgb(186, 85, 211))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_tgs2602)
                    .name("TGS-2602 (A0 kiri)")
                    .color(egui::Color32::from_rgb(46, 204, 113))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_tgs2620)
                    .name("TGS-2620 (A1 kiri)")
                    .color(egui::Color32::from_rgb(0, 206, 209))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_tgs2611)
                    .name("TGS-2611 (A2 kiri)")
                    .color(egui::Color32::from_rgb(243, 156, 18))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(pts_tgs2600)
                    .name("TGS-2600 (A3 kiri)")
                    .color(egui::Color32::from_rgb(233, 30, 99))
                    .width(2.0),
            );
        });
    }

    fn render_voltage_plot(&self, ui: &mut egui::Ui, height: Option<f32>) {
        let mut plot = Plot::new("sensor_voltage_plot")
            .legend(Legend::default())
            .x_axis_label("Waktu (detik)")
            .y_axis_label("Tegangan (mV)")
            .include_y(0.0)
            .include_y(1000.0);

        if let Some(h) = height {
            plot = plot.height(h);
        }

        if self.auto_scale {
            plot = plot.auto_bounds(egui::Vec2b::new(true, true));
        }

        plot.show(ui, |plot_ui| {
            if self.chart_data.mq6.is_empty() {
                plot_ui.text(
                    egui_plot::Text::new(
                        egui_plot::PlotPoint::new(5.0, 500.0),
                        "📊 Menunggu data sensor...\n1. Pilih Port di sebelah kiri (misal: COM17)\n2. Klik Connect\n3. Klik START SIKLUS",
                    )
                    .color(egui::Color32::from_rgb(180, 180, 180)),
                );
                return;
            }

            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.mq6.clone()))
                    .name("MQ-6 (A0 kanan)")
                    .color(egui::Color32::from_rgb(255, 99, 71))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.mq135.clone()))
                    .name("MQ-135 (A1 kanan)")
                    .color(egui::Color32::from_rgb(30, 144, 255))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.mq7.clone()))
                    .name("MQ-7 (A2 kanan)")
                    .color(egui::Color32::from_rgb(255, 193, 7))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.mq3.clone()))
                    .name("MQ-3 (A3 kanan)")
                    .color(egui::Color32::from_rgb(186, 85, 211))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.tgs2602.clone()))
                    .name("TGS-2602 (A0 kiri)")
                    .color(egui::Color32::from_rgb(46, 204, 113))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.tgs2620.clone()))
                    .name("TGS-2620 (A1 kiri)")
                    .color(egui::Color32::from_rgb(0, 206, 209))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.tgs2611.clone()))
                    .name("TGS-2611 (A2 kiri)")
                    .color(egui::Color32::from_rgb(243, 156, 18))
                    .width(2.0),
            );
            plot_ui.line(
                Line::new(PlotPoints::from(self.chart_data.tgs2600.clone()))
                    .name("TGS-2600 (A3 kiri)")
                    .color(egui::Color32::from_rgb(233, 30, 99))
                    .width(2.0),
            );
        });
    }
}

impl eframe::App for ElectronicNoseApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Handle incoming messages
        let mut should_disconnect = false;
        if let Some(rx) = &self.receiver {
            while let Ok(msg) = rx.try_recv() {
                match msg {
                    AppMsg::Data(payload) => {
                        let time = self.start_time.elapsed().as_secs_f64();
                        self.chart_data.push(time, &payload.sensors);
                        self.baseline.update(&payload.sensors, payload.status.elapsed_time);
                        self.export_history.push(payload.clone());
                        self.latest_payload = Some(payload);
                    }
                    AppMsg::Error(err) => {
                        self.connection_error = Some(err);
                        should_disconnect = true;
                    }
                }
            }
        }
        
        if should_disconnect {
            self.disconnect();
        }

        // --- Side Panel (Controls & Status) ---
        egui::SidePanel::left("control_panel")
            .resizable(true)
            .default_width(320.0)
            .show(ctx, |ui| {
                ui.heading("Electronic Nose");
                ui.label(egui::RichText::new("Konfigurasi: 8 Gas Sensors | Tanpa Motor Driver").small().color(egui::Color32::GRAY));
                ui.separator();

                ui.add_space(8.0);
                ui.label(egui::RichText::new("Koneksi Serial:").strong());
                
                egui::ComboBox::from_label("Port")
                    .selected_text(&self.selected_port)
                    .show_ui(ui, |ui| {
                        for p in &self.available_ports {
                            ui.selectable_value(&mut self.selected_port, p.clone(), p);
                        }
                    });

                ui.horizontal(|ui| {
                    if ui.button("🔄 Refresh").clicked() {
                        self.available_ports = Self::scan_ports();
                        if !self.available_ports.is_empty() {
                            self.selected_port = self.available_ports[0].clone();
                        }
                    }

                    if !self.connected {
                        if ui.button("⚡ Connect").clicked() {
                            self.connect(ctx.clone());
                        }
                    } else {
                        if ui.button("❌ Disconnect").clicked() {
                            self.disconnect();
                        }
                    }
                });

                if let Some(err) = &self.connection_error {
                    ui.add_space(5.0);
                    ui.colored_label(egui::Color32::from_rgb(255, 50, 50), format!("⚠️ {}", err));
                }

                ui.add_space(10.0);
                if self.connected {
                    ui.horizontal(|ui| {
                        let start_btn = ui.add_sized([130.0, 32.0], egui::Button::new("▶ START SIKLUS"));
                        if start_btn.clicked() {
                            if let Some(tx) = &self.tx_command {
                                let _ = tx.send("START\n".to_string());
                            }
                            if !self.baseline.is_manual {
                                self.baseline.reset_auto();
                            }
                        }
                        
                        let stop_btn = ui.add_sized([130.0, 32.0], egui::Button::new("⏹ STOP SIKLUS"));
                        if stop_btn.clicked() {
                            if let Some(tx) = &self.tx_command {
                                let _ = tx.send("STOP\n".to_string());
                            }
                        }
                    });
                }

                ui.add_space(12.0);
                ui.separator();
                ui.label(egui::RichText::new("Opsi Tampilan & Ekspor:").strong());
                ui.add_space(4.0);

                let export_btn = ui.add_enabled(
                    !self.export_history.is_empty(), 
                    egui::Button::new("💾 Export ke Excel (CSV)")
                );
                if export_btn.clicked() {
                    self.export_to_csv();
                }

                ui.checkbox(&mut self.auto_scale, "💡 Auto Scale Grafik");
                
                let clear_btn = ui.add_enabled(
                    !self.chart_data.mq6.is_empty(),
                    egui::Button::new("🗑 Hapus Grafik")
                );
                if clear_btn.clicked() {
                    self.chart_data = ChartData::new();
                    self.export_history.clear();
                    if !self.baseline.is_manual {
                        self.baseline.reset_auto();
                    }
                }

                ui.add_space(8.0);
                ui.separator();
                ui.label(egui::RichText::new("Mode Tampilan & Baseline:").strong());
                ui.add_space(4.0);

                ui.label("Pilih Tampilan Grafik:");
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.view_mode, ViewMode::Normalized, "📈 Rs/R0");
                    ui.selectable_value(&mut self.view_mode, ViewMode::Voltage, "⚡ mV");
                    ui.selectable_value(&mut self.view_mode, ViewMode::Both, "📑 Split");
                });

                ui.add_space(4.0);
                egui::ComboBox::from_label("Rumus")
                    .selected_text(self.norm_method.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.norm_method, NormMethod::RsR0, NormMethod::RsR0.label());
                        ui.selectable_value(&mut self.norm_method, NormMethod::R0Rs, NormMethod::R0Rs.label());
                        ui.selectable_value(&mut self.norm_method, NormMethod::VsV0, NormMethod::VsV0.label());
                        ui.selectable_value(&mut self.norm_method, NormMethod::DeltaV, NormMethod::DeltaV.label());
                    });

                if self.norm_method == NormMethod::RsR0 || self.norm_method == NormMethod::R0Rs {
                    ui.horizontal(|ui| {
                        ui.label("Tegangan Vcc:");
                        ui.add(egui::DragValue::new(&mut self.vcc_mv).suffix(" mV").clamp_range(3000.0..=6000.0).speed(10.0));
                    });
                }

                ui.add_space(4.0);
                let baseline_status_str = if self.baseline.is_manual {
                    "🔒 Manual".to_string()
                } else if self.baseline.locked {
                    format!("🔒 Terkunci (Fase 1, n={})", self.baseline.count)
                } else if self.baseline.count > 0 {
                    format!("⏳ Mengukur Fase 1 (n={})", self.baseline.count)
                } else {
                    "⏸ Standby (R0 awal)".to_string()
                };
                ui.label(format!("Status R0: {}", baseline_status_str));

                ui.horizontal(|ui| {
                    if ui.button("🎯 Set R0 Sekarang").clicked() {
                        if let Some(payload) = &self.latest_payload {
                            self.baseline.set_manual_from_sensors(&payload.sensors);
                        }
                    }
                    if ui.button("🔄 Auto R0 (Fase 1)").clicked() {
                        self.baseline.reset_auto();
                    }
                });

                ui.add_space(12.0);
                ui.separator();
                ui.label(egui::RichText::new("Live Status Siklus:").strong());
                ui.add_space(5.0);

                if let Some(payload) = &self.latest_payload {
                    let phase_text = self.get_current_phase();
                    let phase_color = match payload.status.elapsed_time {
                        1..=70 => egui::Color32::from_rgb(180, 180, 180),
                        71..=170 => egui::Color32::from_rgb(0, 180, 255),
                        171..=230 => egui::Color32::from_rgb(255, 170, 0),
                        231..=360 => egui::Color32::from_rgb(0, 220, 120),
                        _ => egui::Color32::WHITE,
                    };
                    ui.label("Status Fase:");
                    ui.colored_label(phase_color, egui::RichText::new(phase_text).strong());

                    ui.add_space(5.0);
                    ui.label(format!("⏱ Waktu Berjalan: {} detik", payload.status.elapsed_time));
                    ui.label(format!("📈 Jumlah Sampel: {}", self.export_history.len()));

                    ui.add_space(8.0);
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Respon Sensor Gas:").strong());
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new("(mV & Rs/R0)").small().color(egui::Color32::GRAY));
                        });
                    });
                    ui.add_space(4.0);

                    let calc_r = |v: f32, v0: f64| -> f64 {
                        self.norm_method.calculate(v as f64, v0, self.vcc_mv)
                    };

                    egui::Grid::new("sensors_voltage_grid").num_columns(3).spacing([12.0, 4.0]).show(ui, |ui| {
                        ui.label(egui::RichText::new("Sensor").underline());
                        ui.label(egui::RichText::new("Tegangan").underline());
                        ui.label(egui::RichText::new("Rs / R0").underline());
                        ui.end_row();

                        let s = &payload.sensors;
                        let b = &self.baseline;

                        ui.colored_label(egui::Color32::from_rgb(255, 99, 71), "MQ-6 (A0 kanan):");
                        ui.label(format!("{:.1} mV", s.mq6));
                        ui.colored_label(egui::Color32::from_rgb(255, 99, 71), format!("{:.3}", calc_r(s.mq6, b.v0_mq6)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(30, 144, 255), "MQ-135 (A1 kanan):");
                        ui.label(format!("{:.1} mV", s.mq135));
                        ui.colored_label(egui::Color32::from_rgb(30, 144, 255), format!("{:.3}", calc_r(s.mq135, b.v0_mq135)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(255, 193, 7), "MQ-7 (A2 kanan):");
                        ui.label(format!("{:.1} mV", s.mq7));
                        ui.colored_label(egui::Color32::from_rgb(255, 193, 7), format!("{:.3}", calc_r(s.mq7, b.v0_mq7)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(186, 85, 211), "MQ-3 (A3 kanan):");
                        ui.label(format!("{:.1} mV", s.mq3));
                        ui.colored_label(egui::Color32::from_rgb(186, 85, 211), format!("{:.3}", calc_r(s.mq3, b.v0_mq3)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(46, 204, 113), "TGS-2602 (A0 kiri):");
                        ui.label(format!("{:.1} mV", s.tgs2602));
                        ui.colored_label(egui::Color32::from_rgb(46, 204, 113), format!("{:.3}", calc_r(s.tgs2602, b.v0_tgs2602)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(0, 206, 209), "TGS-2620 (A1 kiri):");
                        ui.label(format!("{:.1} mV", s.tgs2620));
                        ui.colored_label(egui::Color32::from_rgb(0, 206, 209), format!("{:.3}", calc_r(s.tgs2620, b.v0_tgs2620)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(243, 156, 18), "TGS-2611 (A2 kiri):");
                        ui.label(format!("{:.1} mV", s.tgs2611));
                        ui.colored_label(egui::Color32::from_rgb(243, 156, 18), format!("{:.3}", calc_r(s.tgs2611, b.v0_tgs2611)));
                        ui.end_row();

                        ui.colored_label(egui::Color32::from_rgb(233, 30, 99), "TGS-2600 (A3 kiri):");
                        ui.label(format!("{:.1} mV", s.tgs2600));
                        ui.colored_label(egui::Color32::from_rgb(233, 30, 99), format!("{:.3}", calc_r(s.tgs2600, b.v0_tgs2600)));
                        ui.end_row();
                    });

                    ui.add_space(8.0);
                    ui.separator();
                    ui.label(egui::RichText::new("Suhu & Kelembaban:").strong());
                    ui.add_space(4.0);

                    egui::Grid::new("dht_grid").num_columns(2).spacing([20.0, 4.0]).show(ui, |ui| {
                        ui.label("DHT1 Chamber:");
                        ui.label(format!("{:.1} °C | {:.1} %", payload.sensors.dht1_temp, payload.sensors.dht1_hum));
                        ui.end_row();

                        ui.label("DHT2 Ambien:");
                        ui.label(format!("{:.1} °C | {:.1} %", payload.sensors.dht2_temp, payload.sensors.dht2_hum));
                        ui.end_row();
                    });
                } else {
                    ui.colored_label(egui::Color32::GRAY, "Menunggu data serial dari ESP32...");
                }
            });

        // --- Central Panel (Realtime Chart) ---
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Electronic Nose Dashboard");
                ui.separator();

                // Pilihan Tab / Tampilan Grafik
                ui.selectable_value(&mut self.view_mode, ViewMode::Normalized, "📈 Normalized Rs/R0 (Satu Sumbu Y)");
                ui.selectable_value(&mut self.view_mode, ViewMode::Voltage, "⚡ Tegangan (mV)");
                ui.selectable_value(&mut self.view_mode, ViewMode::Both, "📑 Split (Keduanya)");

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.connected {
                        ui.colored_label(egui::Color32::from_rgb(0, 255, 100), format!("● Terhubung ({})", self.selected_port));
                    } else {
                        ui.colored_label(egui::Color32::GRAY, "○ Terputus");
                    }
                });
            });
            ui.add_space(6.0);

            match self.view_mode {
                ViewMode::Normalized => {
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new("📊 Grafik Normalized Respon (Rs/R0) — Seluruh 8 Sensor dalam Satu Sumbu Y")
                                .strong()
                                .color(egui::Color32::from_rgb(100, 200, 255)),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(
                                egui::RichText::new("Garis Putus-putus = Baseline R0 (1.0)")
                                    .small()
                                    .color(egui::Color32::from_rgb(160, 160, 160)),
                            );
                        });
                    });
                    self.render_normalized_plot(ui, None);
                }
                ViewMode::Voltage => {
                    ui.label(
                        egui::RichText::new("⚡ Grafik Realtime Tegangan Sensor Gas (mV)")
                            .strong()
                            .color(egui::Color32::from_rgb(255, 200, 100)),
                    );
                    self.render_voltage_plot(ui, None);
                }
                ViewMode::Both => {
                    let total_height = ui.available_height() - 50.0;
                    let half_height = (total_height * 0.5).max(180.0);

                    ui.label(
                        egui::RichText::new("⚡ 1. Grafik Tegangan Sensor Gas (mV)")
                            .strong()
                            .small()
                            .color(egui::Color32::from_rgb(255, 200, 100)),
                    );
                    self.render_voltage_plot(ui, Some(half_height - 20.0));

                    ui.separator();

                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new("📈 2. Grafik Normalized Respon (Rs/R0) — Satu Sumbu Y")
                                .strong()
                                .small()
                                .color(egui::Color32::from_rgb(100, 200, 255)),
                        );
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(
                                egui::RichText::new("Garis Putus-putus = Baseline R0 (1.0)")
                                    .small()
                                    .color(egui::Color32::from_rgb(160, 160, 160)),
                            );
                        });
                    });
                    self.render_normalized_plot(ui, Some(half_height - 20.0));
                }
            }
        });
        
        if self.connected {
            ctx.request_repaint_after(Duration::from_millis(100));
        }
    }
}

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 680.0])
            .with_min_inner_size([850.0, 520.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Electronic Nose Dashboard",
        options,
        Box::new(|cc| Box::new(ElectronicNoseApp::new(cc))),
    )
}
