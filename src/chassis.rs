use core::time::Duration;
extern crate alloc;
use serde::{Deserialize, Serialize};
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
use crate::{
    VexSelector,
    mcl::{DistanceSensorBeam, Mcl},
    odometry::{Odometry, Pose, TrackingWheelConfig},
    pid::Pid,
    triggers::{IntakeCommand, PneumaticTarget, ResetAxis, TriggerAction, TriggerDefinition, TriggerManager},
    utils::normalize_angle,
};

const USE_MCL_LOCALIZATION: bool = false;

#[derive(Copy, Clone, Debug, Default, Serialize, Deserialize)]
pub struct PoseSettings {
    #[serde(default)]
    pub is_reversed: bool,
    #[serde(default = "PoseSettings::default_voltage")]
    pub max_voltage: f64,
    #[serde(default)]
    pub fast: bool,
}
impl PoseSettings {
    fn default_voltage() -> f64 {
        6.0
    }
}
pub struct ChassisAutonConfig {
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

    pub t_accel_ramsete: f64,
    pub t_accel_drive: f64,
    pub t_accel_turn: f64,
    pub t_decel_ramsete: f64,
    pub t_decel_drive: f64,
    pub t_decel_turn: f64,

    pub centripetal_accel_limit: f64,
    pub velocity_exit_threshold: f64,

    pub tw_config: TrackingWheelConfig,
    pub dist_sensor_config: crate::mcl::DistanceSensorConfig,
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
    pub config: ChassisAutonConfig,
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
    pub prev_pose: Pose,
    pub last_update_time: Instant,
    pub pose_velocity: f64,
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
    pub auton_config: ChassisAutonConfig,
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
            .set_rotation(-args.auton_config.initial_pose.heading.to_degrees());

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
        let initial_pose = args.auton_config.initial_pose;
        let odometry = Odometry::new(initial_pose, args.auton_config.tw_config);
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
            config: args.auton_config,
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
            prev_pose: initial_pose,
            last_update_time: Instant::now(),
            pose_velocity: 0.0,
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
            .update(parallel_pos, Position::from_revolutions(0.0), imu_heading_rad);
        let curr_pose = self.odometry.pose();
        let dx = curr_pose.x - prev_pose.x;
        let dy = curr_pose.y - prev_pose.y;

        let now = Instant::now();
        let dt = now.duration_since(self.last_update_time).as_secs_f64();
        if dt > 1e-6 {
            let dist = dx.hypot(dy);
            self.pose_velocity = dist / dt;
        }
        self.prev_pose = prev_pose;
        self.last_update_time = now;

        if !USE_MCL_LOCALIZATION {
            self.last_mcl_pose = curr_pose;
            self.update_diagnostics();
            return;
        }

        let dist_cfg = self.config.dist_sensor_config;
        let mut beams: alloc::vec::Vec<DistanceSensorBeam> = alloc::vec::Vec::with_capacity(4);
        if !self.scraper_up
            && let Ok(Some(obj)) = self.dist_front.object()
        {
            let mut b = dist_cfg.front;
            b.distance = (obj.distance as f32) / 25.4;
            beams.push(b);
        }
        if let Ok(Some(obj)) = self.dist_right.object() {
            let mut b = dist_cfg.right;
            b.distance = (obj.distance as f32) / 25.4;
            beams.push(b);
        }
        if let Ok(Some(obj)) = self.dist_back.object() {
            let mut b = dist_cfg.back;
            b.distance = (obj.distance as f32) / 25.4;
            beams.push(b);
        }
        if let Ok(Some(obj)) = self.dist_left.object() {
            let mut b = dist_cfg.left;
            b.distance = (obj.distance as f32) / 25.4;
            beams.push(b);
        }

        let est = self.mcl.run(&beams, (dx, dy), imu_heading_rad);
        self.last_mcl_pose = est;
        self.update_diagnostics();
    }

    #[inline]
    pub fn mcl_pose(&self) -> Pose {
        self.last_mcl_pose
    }

    pub fn reset_pos(&mut self, axis: ResetAxis) {
        const FIELD_HALF: f64 = 72.0;
        const ANGLE_TOLERANCE: f64 = 20.0;

        let pose = self.odometry.pose();
        let heading = pose.heading;
        let dist_cfg = self.config.dist_sensor_config;

        let reset_x = matches!(axis, ResetAxis::X | ResetAxis::XY);
        let reset_y = matches!(axis, ResetAxis::Y | ResetAxis::XY);

        let mut x_sum = 0.0;
        let mut x_count = 0;
        let mut y_sum = 0.0;
        let mut y_count = 0;

        fn process_beam(
            beam: DistanceSensorBeam,
            dist: f64,
            heading: f64,
            x_sum: &mut f64,
            x_count: &mut i32,
            y_sum: &mut f64,
            y_count: &mut i32,
            reset_x: bool,
            reset_y: bool,
        ) {
            let global_angle = normalize_angle(heading + beam.angle as f64);
            let c = global_angle.cos();
            let s = global_angle.sin();
            let ct = heading.cos();
            let st = heading.sin();
            let ox = beam.offset_x as f64;
            let oy = beam.offset_y as f64;
            let sx = ox * ct - oy * st;
            let sy = ox * st + oy * ct;

            let angle_deg = global_angle.to_degrees();
            if reset_x {
                if angle_deg.abs() < ANGLE_TOLERANCE {
                    *x_sum += FIELD_HALF - (sx + dist * c);
                    *x_count += 1;
                } else if (angle_deg.abs() - 180.0).abs() < ANGLE_TOLERANCE {
                    *x_sum += -FIELD_HALF - (sx + dist * c);
                    *x_count += 1;
                }
            }
            if reset_y {
                if (angle_deg - 90.0).abs() < ANGLE_TOLERANCE {
                    *y_sum += FIELD_HALF - (sy + dist * s);
                    *y_count += 1;
                } else if (angle_deg + 90.0).abs() < ANGLE_TOLERANCE {
                    *y_sum += -FIELD_HALF - (sy + dist * s);
                    *y_count += 1;
                }
            }
        }

        if !self.scraper_up {
            if let Ok(Some(obj)) = self.dist_front.object() {
                process_beam(
                    dist_cfg.front,
                    obj.distance as f64 / 25.4,
                    heading,
                    &mut x_sum,
                    &mut x_count,
                    &mut y_sum,
                    &mut y_count,
                    reset_x,
                    reset_y,
                );
            }
        }
        if let Ok(Some(obj)) = self.dist_back.object() {
            process_beam(
                dist_cfg.back,
                obj.distance as f64 / 25.4,
                heading,
                &mut x_sum,
                &mut x_count,
                &mut y_sum,
                &mut y_count,
                reset_x,
                reset_y,
            );
        }
        if let Ok(Some(obj)) = self.dist_left.object() {
            process_beam(
                dist_cfg.left,
                obj.distance as f64 / 25.4,
                heading,
                &mut x_sum,
                &mut x_count,
                &mut y_sum,
                &mut y_count,
                reset_x,
                reset_y,
            );
        }
        if let Ok(Some(obj)) = self.dist_right.object() {
            process_beam(
                dist_cfg.right,
                obj.distance as f64 / 25.4,
                heading,
                &mut x_sum,
                &mut x_count,
                &mut y_sum,
                &mut y_count,
                reset_x,
                reset_y,
            );
        }

        let mut new_pose = pose;
        if x_count > 0 {
            new_pose.x = x_sum / x_count as f64;
            println!("reset_pos: x {:.2} -> {:.2}", pose.x, new_pose.x);
        }
        if y_count > 0 {
            new_pose.y = y_sum / y_count as f64;
            println!("reset_pos: y {:.2} -> {:.2}", pose.y, new_pose.y);
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
            TriggerAction::ResetPos(axis) => {
                self.reset_pos(axis);
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

    pub fn check_stall(&mut self, fast: bool) -> bool {
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

        let stalled = avg_current >= self.config.stall_current_threshold
            && avg_velocity <= self.config.stall_velocity_threshold;

        if stalled {
            if fast {
                return true;
            }
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
        let parallel_revs = self.parallel_wheel.position().unwrap_or_default().as_revolutions();
        let perpendicular_revs = self.perpendicular_wheel.position().unwrap_or_default().as_revolutions();
        self.odometry.reset(pose, parallel_revs, perpendicular_revs);
        let variance = (1.0, 1.0, (2.0_f32).to_radians());
        self.mcl = Mcl::new(pose, variance);
        self.last_mcl_pose = pose;
        let _ = self.imu.set_rotation(-pose.heading.to_degrees());
        self.update_diagnostics();
    }
}
