use core::{f64::consts::PI, time::Duration};
extern crate alloc;
use vexide::{
    devices::{controller::Controller, smart::imu::{InertialSensor, InertialError}},
    prelude::*,
    time::Instant,
};
use slint::SharedString;

use crate::{VexSelector, odometry::{Odometry, Pose, TrackingWheelConfig}};
use crate::pid::Pid;
use crate::triggers::TriggerManager;

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

    pub tw_config: Option<TrackingWheelConfig>,
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
    pub intake_state: i8,
    pub prev_l1: bool,
    pub outtake_state: i8,
    pub prev_r1: bool,
    pub outtake_jam_reverse_until: Option<Instant>,
    pub outtake_initial_reverse_until: Option<Instant>,
    pub outtake_middle_state: i8,
    pub prev_r2: bool,
    pub outtake_middle_jam_reverse_until: Option<Instant>,
    pub outtake_middle_initial_reverse_until: Option<Instant>,
}
pub struct ChassisArgs<const L: usize, const R: usize, const I: usize> {
    pub left_motors: [Motor; L],
    pub right_motors: [Motor; R],
    pub parallel_wheel: RotationSensor,
    pub perpendicular_wheel: RotationSensor,
    pub intake_motors: [Motor; I],
    pub imu: InertialSensor,
    pub controller: Controller,
    pub config: ChassisConfig,
    pub triggers: &'static [(&'static str, fn())],
    pub ui: slint::Weak<VexSelector>,
}
impl<const L: usize, const R: usize, const I: usize> Chassis<L, R, I> {
    pub async fn new(
        mut args: ChassisArgs<L, R, I>,
    ) -> Self {
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
                        if !h.get_show_progress() { break; }
                        let elapsed_ms = start.elapsed().as_millis() as f64;
                        let pct = (elapsed_ms / total_ms * 100.0).clamp(0.0, 100.0) as f32;
                        h.set_progress_value(pct);
                    } else {
                        break;
                    }
                }
            }).detach();
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
        let _ = args.imu.set_rotation(args.config.initial_pose.heading.to_degrees());


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
        let odometry = Odometry::new(
            args.config.initial_pose,
            args.config.wheel_diameter,
            args.config.track_width,
            args.config.ext_gear_ratio,
            args.config.tw_config,
        );
        let motor_free_rpm = match args.left_motors[0].gearset() {
            Ok(Gearset::Blue) => 600.0,
            Ok(Gearset::Green) => 200.0,
            Ok(Gearset::Red) => 100.0,
            Err(_) => 600.0,
        };

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
            intake_state: 0,
            prev_l1: false,
            outtake_state: 0,
            prev_r1: false,
            outtake_jam_reverse_until: None,
            outtake_initial_reverse_until: None,
            outtake_middle_state: 0,
            prev_r2: false,
            outtake_middle_jam_reverse_until: None,
            outtake_middle_initial_reverse_until: None,
        }
    }
    pub fn normalize_angle(&self, mut angle: f64) -> f64 {
        while angle > PI {
            angle -= 2.0 * PI;
        }
        while angle < -PI {
            angle += 2.0 * PI;
        }
        angle
    }

    pub fn update_odometry(&mut self) {
        let parallel_pos = self.parallel_wheel.position().unwrap_or_default();
        let perpendicular_pos = self.perpendicular_wheel.position().unwrap_or_default();
        let imu_heading_rad = self.imu.rotation().unwrap_or_default().to_radians();
        self.odometry
            .update(parallel_pos, perpendicular_pos, imu_heading_rad);
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
                && timer.elapsed() >= self.config.stall_time {
                    return true;
                }
        } else {
            self.stall_timer = None;
        }
        false
    }
}
