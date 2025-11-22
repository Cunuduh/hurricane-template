use core::time::Duration;
extern crate alloc;
use slint::SharedString;
use vexide::{
    devices::{
        battery,
        controller::Controller,
        smart::imu::{InertialError, InertialSensor},
    },
    prelude::*,
    time::Instant,
};
use core::f64::consts::PI;
use crate::{
    VexSelector,
    mcl::{Beam, Mcl},
    odometry::{Odometry, Pose, TrackingWheelConfig},
    pid::Pid,
    triggers::{IntakeCommand, PneumaticTarget, TriggerAction, TriggerDefinition, TriggerManager},
    utils::normalize_angle,
};

const USE_MCL_LOCALIZATION: bool = false;

#[derive(Copy, Clone, Debug)]
pub struct PoseSettings {
    pub is_reversed: bool,
    pub max_voltage: f64,
}
impl Default for PoseSettings {
    fn default() -> Self {
        Self {
            is_reversed: false,
            max_voltage: 12.0,
        }
    }
}
pub struct ChassisConfig {
    pub initial_pose: Pose,
    pub wheel_diameter: f64,
    pub ext_gear_ratio: f64,
    pub track_width: f64,
    pub max_volts: f64,
    pub ff_ks: f64,
    pub ff_kv: f64,
    pub ff_ka: f64,
    pub ff_kv_ang: f64,
    pub ff_ka_ang: f64,
    pub dt: Duration,
    pub turn_pid: Pid,
    pub heading_pid: Pid,
    pub drive_pid: Pid,

    pub small_drive_exit_error: f64,
    pub small_drive_settle_time: Duration,
    pub big_drive_exit_error: f64,
    pub big_drive_settle_time: Duration,

    pub small_turn_exit_error: f64,
    pub small_turn_settle_time: Duration,
    pub big_turn_exit_error: f64,
    pub big_turn_settle_time: Duration,

    pub stall_current_threshold: f64,
    pub stall_velocity_threshold: f64,
    pub stall_time: Duration,

    pub t_accel: f64,

    pub tw_config: TrackingWheelConfig,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IntakeMode {
    Idle,
    Intake,
    Outtake,
    OuttakeMiddle,
    Reverse,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DetectedColour {
    Red,
    Blue,
    Unknown,
}
pub struct BlockCounter {
    pub block_count: usize,
    detect_threshold: f64,
    release_threshold: f64,
    block_detected: bool,
    last_count_time: Instant,
    min_time_between_blocks: Duration,
    pub activation_delay: Duration,
}
impl BlockCounter {
    pub fn new() -> Self {
        Self {
            block_count: 0,
            detect_threshold: 0.35,
            release_threshold: 0.25,
            block_detected: false,
            last_count_time: Instant::now(),
            min_time_between_blocks: Duration::from_millis(50),
            activation_delay: Duration::from_millis(250),
        }
    }
    pub fn update(&mut self, proximity: f64) -> bool {
        let mut new_block = false;
        if !self.block_detected && proximity > self.detect_threshold {
            new_block = true;
            if Instant::now().duration_since(self.last_count_time) > self.min_time_between_blocks {
                self.block_count += 1;
                self.last_count_time = Instant::now();
            }
            self.block_detected = true;
        } else if self.block_detected && proximity < self.release_threshold {
            self.block_detected = false;
        }
        new_block
    }
}
pub struct Chassis<const L: usize, const R: usize, const I: usize> {
    pub left_motors: [Motor; L],
    pub right_motors: [Motor; R],
    pub parallel_wheel: RotationSensor,
    pub perpendicular_wheel: RotationSensor,
    pub intake_motors: [Motor; I],
    pub config: ChassisConfig,
    pub imu: InertialSensor,
    pub controller: Controller,
    pub odometry: Odometry,
    pub motor_free_rpm: f64,
    pub stall_timer: Option<Instant>,
    pub prev_turn: f64,
    pub prev_throttle: f64,
    pub quick_stop_accumulator: f64,
    pub neg_inertia_accumulator: f64,
    pub triggers: TriggerManager,
    pub intake_mode: IntakeMode,
    pub block_counter: BlockCounter,
    pub outtake_jam_reverse_until: Option<Instant>,
    pub outtake_initial_reverse_until: Option<Instant>,
    pub outtake_middle_jam_reverse_until: Option<Instant>,
    pub outtake_middle_initial_reverse_until: Option<Instant>,
    pub scraper: AdiDigitalOut,
    // true when scraper is up (solenoid low)
    pub scraper_up: bool,
    pub hood: AdiDigitalOut,
    pub wings: AdiDigitalOut,
    pub block_park: AdiDigitalOut,
    pub colour_sort: AdiDigitalOut,
    pub colour_sort_enabled: bool,
    pub alt_colour_sort_enabled: bool,
    pub alt_colour_sort_run_until: Option<Instant>,
    pub alt_colour_last_enemy: bool,
    pub last_detected_colour: DetectedColour,
    pub colour_sort_activation_time: Option<Instant>,
    pub colour_sort_run_until: Option<Instant>,
    pub optical_sensor: OpticalSensor,
    pub dist_front: DistanceSensor,
    pub dist_right: DistanceSensor,
    pub dist_back: DistanceSensor,
    pub dist_left: DistanceSensor,
    pub indexer_run_until: Option<Instant>,
    pub indexer_pending_activation_until: Option<Instant>,
    pub ui: slint::Weak<VexSelector>,
    pub last_diagnostics_update: Instant,
    pub mcl: Mcl,
    pub last_mcl_pose: Pose,
    pub team_is_red: bool,
    // block park macro state
    pub block_park_macro_active: bool,
    pub block_park_macro_detected: bool,
    pub block_park_macro_post_delay_until: Option<Instant>,
    pub block_park_macro_last_prox_high: bool,
}
pub struct ChassisArgs<const L: usize, const R: usize, const I: usize> {
    pub left_motors: [Motor; L],
    pub right_motors: [Motor; R],
    pub parallel_wheel: RotationSensor,
    pub perpendicular_wheel: RotationSensor,
    pub intake_motors: [Motor; I],
    pub imu: InertialSensor,
    pub optical_sensor: OpticalSensor,
    pub dist_front: DistanceSensor,
    pub dist_right: DistanceSensor,
    pub dist_back: DistanceSensor,
    pub dist_left: DistanceSensor,
    pub scraper: AdiDigitalOut,
    pub hood: AdiDigitalOut,
    pub wings: AdiDigitalOut,
    pub block_park: AdiDigitalOut,
    pub colour_sort: AdiDigitalOut,
    pub controller: Controller,
    pub config: ChassisConfig,
    pub triggers: &'static [TriggerDefinition],
    pub ui: slint::Weak<VexSelector>,
}
impl<const L: usize, const R: usize, const I: usize> Chassis<L, R, I> {
    pub async fn new(mut args: ChassisArgs<L, R, I>) -> Self {
        let ui = args.ui.clone();

        if let Some(handle) = ui.upgrade() {
            handle.set_show_progress(true);
            handle.set_progress_value(0.0);
            handle.set_progress_text(SharedString::from("Calibrating IMU..."));
        }
        {
            let ui_anim = ui.clone();
            vexide::task::spawn(async move {
                let start = Instant::now();
                let total_ms = 3000.0_f64;
                loop {
                    sleep(Duration::from_millis(50)).await;
                    if let Some(h) = ui_anim.upgrade() {
                        if !h.get_show_progress() {
                            break;
                        }
                        let elapsed_ms = start.elapsed().as_millis() as f64;
                        let pct = (elapsed_ms / total_ms * 100.0).clamp(0.0, 100.0) as f32;
                        h.set_progress_value(pct);
                    } else {
                        break;
                    }
                }
            })
            .detach();
        }
        match args.imu.calibrate().await {
            Ok(_) => {
                println!("IMU calibration successful");
                if let Some(handle) = ui.upgrade() {
                    handle.set_progress_text(SharedString::from("IMU calibration successful"));
                    handle.set_progress_value(100.0);
                }
            }
            Err(e) => {
                let msg = match e {
                    InertialError::CalibrationTimedOut => "IMU calibration timed out",
                    InertialError::Port { .. } => "IMU not detected on the configured port",
                    InertialError::BadStatus => "IMU failed to report status",
                    _ => "IMU calibration error",
                };
                println!("{}: {:?}", msg, e);
                if let Some(handle) = ui.upgrade() {
                    handle.set_progress_text(SharedString::from(msg));
                }
                sleep(Duration::from_millis(750)).await;
            }
        }
        if let Some(handle) = ui.upgrade() {
            handle.set_show_progress(false);
        }

        let _ = args.imu.reset_heading();
        let _ = args.imu.reset_rotation();
        let _ = args
            .imu
            .set_rotation(-args.config.initial_pose.heading.to_degrees());

        for m in args.left_motors.iter_mut() {
            let _ = m.reset_position();
        }
        for m in args.right_motors.iter_mut() {
            let _ = m.reset_position();
        }
        for m in args.intake_motors.iter_mut() {
            let _ = m.reset_position();
        }
        let _ = args.parallel_wheel.reset_position();
        let _ = args.perpendicular_wheel.reset_position();
        let initial_pose = args.config.initial_pose;
        let odometry = Odometry::new(initial_pose, args.config.tw_config);
        // initial variance: 2 inches for X/Y, 5 degrees for theta
        let initial_variance = (2.0, 2.0, (5.0_f32).to_radians());
        let mcl = Mcl::new(initial_pose, initial_variance);
        let motor_free_rpm = match args.left_motors[0].gearset() {
            Ok(Gearset::Blue) => 600.0,
            Ok(Gearset::Green) => 200.0,
            Ok(Gearset::Red) => 100.0,
            Err(_) => 600.0,
        };
        let _ = args.scraper.set_low();
        // scraper is up when low
        let _ = args.hood.set_high();
        let _ = args.colour_sort.set_low();
        let _ = args.wings.set_low();
        let _ = args.block_park.set_low();
        let _ = args.optical_sensor.set_led_brightness(1.0);
        let _ = args
            .optical_sensor
            .set_integration_time(Duration::from_millis(40));
        let initial_team_is_red = ui.upgrade().is_none_or(|h| h.get_is_red_alliance());
        let _ = args.controller.screen.clear_screen().await;
        Self {
            left_motors: args.left_motors,
            right_motors: args.right_motors,
            parallel_wheel: args.parallel_wheel,
            perpendicular_wheel: args.perpendicular_wheel,
            intake_motors: args.intake_motors,
            config: args.config,
            imu: args.imu,
            controller: args.controller,
            odometry,
            motor_free_rpm,
            stall_timer: None,
            prev_turn: 0.0,
            prev_throttle: 0.0,
            quick_stop_accumulator: 0.0,
            neg_inertia_accumulator: 0.0,
            triggers: TriggerManager::new(args.triggers),
            intake_mode: IntakeMode::Idle,
            block_counter: BlockCounter::new(),
            outtake_jam_reverse_until: None,
            outtake_initial_reverse_until: None,
            outtake_middle_jam_reverse_until: None,
            outtake_middle_initial_reverse_until: None,
            scraper: args.scraper,
            scraper_up: true,
            hood: args.hood,
            wings: args.wings,
            block_park: args.block_park,
            colour_sort: args.colour_sort,
            colour_sort_enabled: false,
            alt_colour_sort_enabled: false,
            alt_colour_sort_run_until: None,
            alt_colour_last_enemy: false,
            last_detected_colour: DetectedColour::Unknown,
            colour_sort_activation_time: None,
            colour_sort_run_until: None,
            optical_sensor: args.optical_sensor,
            dist_front: args.dist_front,
            dist_right: args.dist_right,
            dist_back: args.dist_back,
            dist_left: args.dist_left,
            indexer_run_until: None,
            indexer_pending_activation_until: None,
            ui,
            last_diagnostics_update: Instant::now(),
            mcl,
            last_mcl_pose: initial_pose,
            // read initial value from UI if present, default to red
            team_is_red: initial_team_is_red,
            block_park_macro_active: false,
            block_park_macro_detected: false,
            block_park_macro_post_delay_until: None,
            block_park_macro_last_prox_high: false,
        }
    }

    pub fn update_state(&mut self) {
        let parallel_pos = self.parallel_wheel.position().unwrap_or_default();
        let perpendicular_pos = self.perpendicular_wheel.position().unwrap_or_default();
        let imu_heading_rad = -self.imu.rotation().unwrap_or_default().to_radians();

        let prev_pose = self.odometry.pose();
        self.odometry
            .update(parallel_pos, perpendicular_pos, imu_heading_rad);
        let curr_pose = self.odometry.pose();
        let dx = curr_pose.x - prev_pose.x;
        let dy = curr_pose.y - prev_pose.y;

        if !USE_MCL_LOCALIZATION {
            self.last_mcl_pose = curr_pose;
            self.update_diagnostics();
            return;
        }

        let mut beams: alloc::vec::Vec<Beam> = alloc::vec::Vec::with_capacity(4);
        // ignore front distance when scraper is up (low)
        if !self.scraper_up
            && let Ok(o) = self.dist_front.object()
            && let Some(obj) = o
        {
            let inches = (obj.distance as f64) / 25.4;
            beams.push(Beam {
                angle: 0.0,
                distance: inches as f32,
                offset_x: 3.25,
                offset_y: 5.25,
            });
        }
        if let Ok(o) = self.dist_right.object()
            && let Some(obj) = o
        {
            let inches = (obj.distance as f64) / 25.4;
            beams.push(Beam {
                angle: -core::f32::consts::FRAC_PI_2,
                distance: inches as f32,
                offset_x: 5.5,
                offset_y: 1.625,
            });
        }
        if let Ok(o) = self.dist_back.object()
            && let Some(obj) = o
        {
            let inches = (obj.distance as f64) / 25.4;
            beams.push(Beam {
                angle: core::f32::consts::PI,
                distance: inches as f32,
                offset_x: -5.0,
                offset_y: -1.25,
            });
        }
        if let Ok(o) = self.dist_left.object()
            && let Some(obj) = o
        {
            let inches = (obj.distance as f64) / 25.4;
            beams.push(Beam {
                angle: core::f32::consts::FRAC_PI_2,
                distance: inches as f32,
                offset_x: -5.5,
                offset_y: 1.625,
            });
        }

        let est = self.mcl.run(&beams, (dx, dy), imu_heading_rad);
        self.last_mcl_pose = est;
        self.update_diagnostics();
    }

    #[inline]
    pub fn mcl_pose(&self) -> Pose {
        self.last_mcl_pose
    }

    pub fn reset_xy(&mut self) {
        let pose = self.odometry.pose();
        let heading = pose.heading;

        let mut x_sum = 0.0;
        let mut x_count = 0;
        let mut y_sum = 0.0;
        let mut y_count = 0;

        let mut process = |dist: f64, offset_x: f64, offset_y: f64, angle_offset: f64| {
            let sensor_heading = normalize_angle(heading + angle_offset);
            let h_deg = sensor_heading.to_degrees();

            if h_deg.abs() < 20.0 {
                let term = offset_x * heading.cos() - offset_y * heading.sin()
                    + dist * sensor_heading.cos();
                x_sum += 72.0 - term;
                x_count += 1;
            }
            else if (h_deg.abs() - 180.0).abs() < 20.0 {
                let term = offset_x * heading.cos() - offset_y * heading.sin()
                    + dist * sensor_heading.cos();
                x_sum += -72.0 - term;
                x_count += 1;
            }
            else if (h_deg - 90.0).abs() < 20.0 {
                let term = offset_x * heading.sin() + offset_y * heading.cos()
                    + dist * sensor_heading.sin();
                y_sum += 72.0 - term;
                y_count += 1;
            }
            else if (h_deg + 90.0).abs() < 20.0 {
                let term = offset_x * heading.sin() + offset_y * heading.cos()
                    + dist * sensor_heading.sin();
                y_sum += -72.0 - term;
                y_count += 1;
            }
        };

        if !self.scraper_up {
            if let Ok(Some(obj)) = self.dist_front.object() {
                process(obj.distance as f64 / 25.4, 3.25, 5.25, 0.0);
            }
        }
        if let Ok(Some(obj)) = self.dist_right.object() {
            process(obj.distance as f64 / 25.4, 5.5, 1.625, -PI / 2.0);
        }
        if let Ok(Some(obj)) = self.dist_back.object() {
            process(obj.distance as f64 / 25.4, -5.0, -1.25, PI);
        }
        if let Ok(Some(obj)) = self.dist_left.object() {
            process(obj.distance as f64 / 25.4, -5.5, 1.625, PI / 2.0);
        }

        let mut new_pose = pose;
        if x_count > 0 {
            new_pose.x = x_sum / x_count as f64;
        }
        if y_count > 0 {
            new_pose.y = y_sum / y_count as f64;
        }

        if x_count > 0 || y_count > 0 {
            self.set_pose(new_pose);
        }
    }

    pub fn apply_trigger_actions(&mut self, actions: &[TriggerAction]) {
        for &action in actions {
            self.apply_trigger_action(action);
        }
        self.service_autonomous_intake();
    }

    fn apply_trigger_action(&mut self, action: TriggerAction) {
        match action {
            TriggerAction::Intake(command) => {
                self.indexer_run_until = None;
                self.indexer_pending_activation_until = None;
                match command {
                    IntakeCommand::Intake => {
                        self.intake_mode = IntakeMode::Intake;
                        self.outtake_initial_reverse_until = None;
                        self.outtake_jam_reverse_until = None;
                        self.outtake_middle_initial_reverse_until = None;
                        self.outtake_middle_jam_reverse_until = None;
                    }
                    IntakeCommand::Outtake => {
                        self.intake_mode = IntakeMode::Outtake;
                        self.block_counter.block_count = 0;
                        let now = Instant::now();
                        self.outtake_initial_reverse_until = Some(now + Duration::from_millis(100));
                        self.outtake_jam_reverse_until = None;
                        self.outtake_middle_initial_reverse_until = None;
                        self.outtake_middle_jam_reverse_until = None;
                    }
                    IntakeCommand::OuttakeMiddle => {
                        self.intake_mode = IntakeMode::OuttakeMiddle;
                        self.block_counter.block_count = 0;
                        let now = Instant::now();
                        self.outtake_middle_initial_reverse_until =
                            Some(now + Duration::from_millis(100));
                        self.outtake_middle_jam_reverse_until = None;
                        self.outtake_initial_reverse_until = None;
                        self.outtake_jam_reverse_until = None;
                    }
                    IntakeCommand::Reverse => {
                        self.intake_mode = IntakeMode::Reverse;
                        self.outtake_initial_reverse_until = None;
                        self.outtake_jam_reverse_until = None;
                        self.outtake_middle_initial_reverse_until = None;
                        self.outtake_middle_jam_reverse_until = None;
                    }
                    IntakeCommand::Stop => {
                        self.intake_mode = IntakeMode::Idle;
                        self.outtake_initial_reverse_until = None;
                        self.outtake_jam_reverse_until = None;
                        self.outtake_middle_initial_reverse_until = None;
                        self.outtake_middle_jam_reverse_until = None;
                        for m in self.intake_motors.iter_mut() {
                            let _ = m.set_voltage(0.0);
                        }
                    }
                }
            }
            TriggerAction::SetPneumatic { target, state } => {
                self.set_pneumatic_state(target, state);
            }
            TriggerAction::TogglePneumatic(target) => {
                self.toggle_pneumatic(target);
            }
            TriggerAction::SetColourSortEnabled(enabled) => {
                self.colour_sort_enabled = enabled;
                if !enabled {
                    self.colour_sort_activation_time = None;
                    self.colour_sort_run_until = None;
                    let _ = self.colour_sort.set_low();
                }
            }
            TriggerAction::SetAltColourSortEnabled(enabled) => {
                self.alt_colour_sort_enabled = enabled;
                // ensure solenoid is off when switching alt mode
                if enabled {
                    let _ = self.colour_sort.set_low();
                    self.colour_sort_activation_time = None;
                    self.colour_sort_run_until = None;
                    self.alt_colour_last_enemy = false;
                } else {
                    self.alt_colour_sort_run_until = None;
                    self.alt_colour_last_enemy = false;
                }
            }
            TriggerAction::ResetXY => {
                self.reset_xy();
            }
        }
    }

    fn set_pneumatic_state(&mut self, target: PneumaticTarget, state: bool) {
        match target {
            PneumaticTarget::Scraper => {
                if state {
                    let _ = self.scraper.set_high();
                    self.scraper_up = false;
                } else {
                    let _ = self.scraper.set_low();
                    self.scraper_up = true;
                }
            }
            PneumaticTarget::Hood => {
                if state {
                    let _ = self.hood.set_high();
                } else {
                    let _ = self.hood.set_low();
                }
            }
            PneumaticTarget::Wings => {
                if state {
                    let _ = self.wings.set_high();
                } else {
                    let _ = self.wings.set_low();
                }
            }
            PneumaticTarget::BlockPark => {
                if state {
                    let _ = self.block_park.set_high();
                } else {
                    let _ = self.block_park.set_low();
                }
            }
            PneumaticTarget::ColourSort => {
                self.colour_sort_activation_time = None;
                self.colour_sort_run_until = None;
                if state {
                    let _ = self.colour_sort.set_high();
                } else {
                    let _ = self.colour_sort.set_low();
                }
            }
        }
    }

    fn toggle_pneumatic(&mut self, target: PneumaticTarget) {
        match target {
            PneumaticTarget::Scraper => {
                let _ = self.scraper.toggle();
                self.scraper_up = !self.scraper_up;
            }
            PneumaticTarget::Hood => {
                let _ = self.hood.toggle();
            }
            PneumaticTarget::Wings => {
                let _ = self.wings.toggle();
            }
            PneumaticTarget::BlockPark => {
                let _ = self.block_park.toggle();
            }
            PneumaticTarget::ColourSort => {
                self.colour_sort_activation_time = None;
                self.colour_sort_run_until = None;
                let _ = self.colour_sort.toggle();
            }
        }
    }

    pub fn check_stall(&mut self) -> bool {
        let mut total_current = 0.0;
        let mut total_velocity = 0.0;
        let mut num_motors = 0;

        for motor in self.left_motors.iter().chain(self.right_motors.iter()) {
            total_current += motor.current().unwrap_or_default().abs();
            total_velocity += motor.velocity().unwrap_or_default().abs();
            num_motors += 1;
        }

        let avg_current = if num_motors > 0 {
            total_current / num_motors as f64
        } else {
            0.0
        };
        let avg_velocity = if num_motors > 0 {
            total_velocity / num_motors as f64
        } else {
            0.0
        };

        if avg_current >= self.config.stall_current_threshold
            && avg_velocity <= self.config.stall_velocity_threshold
        {
            if self.stall_timer.is_none() {
                self.stall_timer = Some(Instant::now());
            }
            if let Some(timer) = self.stall_timer
                && timer.elapsed() >= self.config.stall_time
            {
                return true;
            }
        } else {
            self.stall_timer = None;
        }
        false
    }

    fn update_diagnostics(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_diagnostics_update) < Duration::from_millis(100) {
            return;
        }
        self.last_diagnostics_update = now;

        if let Some(handle) = self.ui.upgrade() {
            let ui_is_red = handle.get_is_red_alliance();
            self.team_is_red = ui_is_red;
            let mcl = self.last_mcl_pose;
            let odom = self.odometry.pose();
            handle.set_pose_x(mcl.x as f32);
            handle.set_pose_y(mcl.y as f32);
            handle.set_mcl_x(mcl.x as f32);
            handle.set_mcl_y(mcl.y as f32);
            handle.set_odom_x(odom.x as f32);
            handle.set_odom_y(odom.y as f32);
            let heading_deg = -self.imu.heading().unwrap_or_default() as f32;
            handle.set_heading(heading_deg);

            let lm1 = self.left_motors[0].temperature().unwrap_or_default() as f32;
            let lm2 = self.left_motors[1].temperature().unwrap_or_default() as f32;
            let lm3 = self.left_motors[2].temperature().unwrap_or_default() as f32;
            let rm1 = self.right_motors[0].temperature().unwrap_or_default() as f32;
            let rm2 = self.right_motors[1].temperature().unwrap_or_default() as f32;
            let rm3 = self.right_motors[2].temperature().unwrap_or_default() as f32;
            let im1 = self.intake_motors[0].temperature().unwrap_or_default() as f32;
            let im2 = self.intake_motors[1].temperature().unwrap_or_default() as f32;
            let im3 = self.intake_motors[2].temperature().unwrap_or_default() as f32;

            handle.set_left_motor_temp_1(lm1);
            handle.set_left_motor_temp_2(lm2);
            handle.set_left_motor_temp_3(lm3);
            handle.set_right_motor_temp_1(rm1);
            handle.set_right_motor_temp_2(rm2);
            handle.set_right_motor_temp_3(rm3);
            handle.set_intake_motor_temp_1(im1);
            handle.set_intake_motor_temp_2(im2);
            handle.set_intake_motor_temp_3(im3);

            let battery_voltage = battery::voltage() as f32;
            let battery_percentage = (battery::capacity() * 100.0) as f32;
            handle.set_battery_voltage(battery_voltage);
            handle.set_battery_percentage(battery_percentage);
        }
    }

    pub fn set_pose(&mut self, pose: Pose) {
        self.odometry.reset(pose);
        // when resetting pose manually, we assume high confidence, so small variance
        let variance = (1.0, 1.0, (2.0_f32).to_radians());
        self.mcl = Mcl::new(pose, variance);
        self.last_mcl_pose = pose;
        let _ = self.imu.set_rotation(-pose.heading.to_degrees());
        self.update_diagnostics();
    }
}
