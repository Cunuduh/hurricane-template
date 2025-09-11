use core::time::Duration;
extern crate alloc;
use slint::SharedString;
use alloc::vec;
use vexide::{
    devices::{
        battery,
        controller::Controller,
        smart::imu::{InertialError, InertialSensor},
    },
    prelude::*,
    time::Instant,
};

use crate::{
    VexSelector,
    odometry::{Odometry, Pose, TrackingWheelConfig},
    pid::Pid,
    triggers::TriggerManager,
};

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
    pub dt: Duration,
    pub drive_pid: Pid,
    pub turn_pid: Pid,
    pub heading_pid: Pid,

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

    pub accel_t: f64,

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
pub struct BlockCounter {
    pub block_count: usize,
    detect_threshold: f64,
    release_threshold: f64,
    block_detected: bool,
    last_count_time: Instant,
    min_time_between_blocks: Duration,
}
impl BlockCounter {
    pub fn new() -> Self {
        Self {
            block_count: 0,
            detect_threshold: 0.35,
            release_threshold: 0.15,
            block_detected: false,
            last_count_time: Instant::now(),
            min_time_between_blocks: Duration::from_millis(40),
        }
    }
    pub fn update(&mut self, proximity: f64) -> bool {
        let mut new_block = false;
        if !self.block_detected && proximity > self.detect_threshold {
            // only count if enough time has passed since last block
            if Instant::now().duration_since(self.last_count_time) > self.min_time_between_blocks {
                self.block_count += 1;
                self.last_count_time = Instant::now();
                new_block = true;
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
    pub hood: AdiDigitalOut,
    pub colour_sort: AdiDigitalOut,
    pub optical_sensor: OpticalSensor,
    pub indexer_run_until: Option<Instant>,
    pub ui: slint::Weak<VexSelector>,
    pub last_diagnostics_update: Instant,
}
pub struct ChassisArgs<const L: usize, const R: usize, const I: usize> {
    pub left_motors: [Motor; L],
    pub right_motors: [Motor; R],
    pub parallel_wheel: RotationSensor,
    pub perpendicular_wheel: RotationSensor,
    pub intake_motors: [Motor; I],
    pub imu: InertialSensor,
    pub optical_sensor: OpticalSensor,
    pub scraper: AdiDigitalOut,
    pub hood: AdiDigitalOut,
    pub colour_sort: AdiDigitalOut,
    pub controller: Controller,
    pub config: ChassisConfig,
    pub triggers: &'static [(&'static str, fn())],
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
                let total_ms = 4000.0_f64;
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
            .set_rotation(args.config.initial_pose.heading.to_degrees());

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
        let odometry = Odometry::new(args.config.initial_pose, args.config.tw_config);
        let motor_free_rpm = match args.left_motors[0].gearset() {
            Ok(Gearset::Blue) => 600.0,
            Ok(Gearset::Green) => 200.0,
            Ok(Gearset::Red) => 100.0,
            Err(_) => 600.0,
        };
        let _ = args.scraper.set_low();
        let _ = args.hood.set_low();
        let _ = args.colour_sort.set_low();
        let _ = args.optical_sensor.set_led_brightness(1.0);
        let _ = args
            .optical_sensor
            .set_integration_time(Duration::from_millis(40));
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
            hood: args.hood,
            colour_sort: args.colour_sort,
            optical_sensor: args.optical_sensor,
            indexer_run_until: None,
            ui,
            last_diagnostics_update: Instant::now(),
        }
    }

    pub fn update_state(&mut self) {
        let parallel_pos = self.parallel_wheel.position().unwrap_or_default();
        let perpendicular_pos = self.perpendicular_wheel.position().unwrap_or_default();
        let imu_heading_rad = self.imu.rotation().unwrap_or_default().to_radians();
        self.odometry
            .update(parallel_pos, perpendicular_pos, imu_heading_rad);
        self.update_diagnostics();
    }

    pub fn check_stall(&mut self) -> bool {
        let mut total_current = 0.0;
        let mut total_velocity = 0.0;
        let mut num_motors = 0;

        for motor in self.left_motors.iter().chain(self.right_motors.iter()) {
            total_current += motor.current().unwrap_or_default();
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
            let p = self.odometry.pose();
            handle.set_pose_x(p.x as f32);
            handle.set_pose_y(p.y as f32);
            let heading_deg = self.imu.heading().unwrap_or_default() as f32;
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
}
