use core::{f64::consts::PI, time::Duration};
extern crate alloc;
use alloc::{boxed::Box, collections::btree_map::BTreeMap, string::String, vec::Vec};
use vexide::{
    devices::{controller::Controller, smart::imu::InertialSensor},
    prelude::*,
    time::Instant,
};

use crate::odometry::{Odometry, Pose, TrackingWheelConfig};
use crate::pid::Pid;
use crate::utils::path::{interpolate_catmull_rom, interpolate_linear};
use crate::triggers::TriggerManager;

struct Segment {
    delta_h: f64,
    t: f64,
    w_d: f64,
}

struct Profile {
    profile_points: Vec<ProfilePoint>,
    times: Vec<f64>,
    total_time: f64,
}

pub trait Pos2Like {
    fn x(&self) -> f64;
    fn y(&self) -> f64;
    fn distance(&self, other: &Self) -> f64 {
        let dx = self.x() - other.x();
        let dy = self.y() - other.y();
        dx.hypot(dy)
    }
    fn lerp(&self, other: &Self, t: f64) -> Self;
}
#[derive(Copy, Clone, Debug)]
struct ProfilePoint {
    point: Pose,
    distance: f64,
    curvature: f64,
    velocity: f64,
    is_reversed: bool,
    max_speed: f64,
}
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

pub struct Chassis<const L: usize, const R: usize> {
    pub left_motors: [Motor; L],
    pub right_motors: [Motor; R],
    pub parallel_wheel: RotationSensor,
    pub perpendicular_wheel: RotationSensor,
    pub config: ChassisConfig,
    pub imu: InertialSensor,
    pub controller: Controller,
    pub odometry: Odometry,
    pub motor_free_rpm: f64,
    stall_timer: Option<Instant>,
    prev_turn: f64,
    prev_throttle: f64,
    quick_stop_accumulator: f64,
    neg_inertia_accumulator: f64,
    pub triggers: TriggerManager,
}

impl<const L: usize, const R: usize> Chassis<L, R> {
    pub async fn new(
        controller: Controller,
        mut left_motors: [Motor; L],
        mut right_motors: [Motor; R],
        mut parallel_wheel: RotationSensor,
        mut perpendicular_wheel: RotationSensor,
        mut imu: InertialSensor,
        config: ChassisConfig,
        triggers: BTreeMap<String, Box<dyn FnMut()>>,
    ) -> Self {
        let _ = imu.calibrate().await;
        
        let _ = imu.reset_heading();
        let _ = imu.reset_rotation();
        let _ = imu.set_rotation(config.initial_pose.heading.to_degrees());


        for m in left_motors.iter_mut() {
            let _ = m.reset_position();
        }
        for m in right_motors.iter_mut() {
            let _ = m.reset_position();
        }
        let _ = parallel_wheel.reset_position();
        let _ = perpendicular_wheel.reset_position();
        let odometry = Odometry::new(
            config.initial_pose, 
            config.wheel_diameter,
            config.track_width,
            config.ext_gear_ratio,
            config.tw_config,
        );
        let motor_free_rpm = match left_motors[0].gearset() {
            Ok(Gearset::Blue) => 600.0,
            Ok(Gearset::Green) => 200.0,
            Ok(Gearset::Red) => 100.0,
            Err(_) => 600.0,
        };

        Self {
            left_motors,
            right_motors,
            parallel_wheel,
            perpendicular_wheel,
            config,
            imu,
            controller,
            odometry,
            motor_free_rpm,
            stall_timer: None,
            prev_turn: 0.0,
            prev_throttle: 0.0,
            quick_stop_accumulator: 0.0,
            neg_inertia_accumulator: 0.0,
            triggers: TriggerManager::new(triggers),
        }
    }
    fn normalize_angle(&self, mut angle: f64) -> f64 {
        while angle > PI {
            angle -= 2.0 * PI;
        }
        while angle < -PI {
            angle += 2.0 * PI;
        }
        angle
    }
    // https://www.desmos.com/calculator/7oyvwwpmed
    fn drive_curve(&self, x: f64, t: f64) -> f64 {
        ((-t / 10.0).exp() + ((x.abs() - 127.0) / 10.0).exp() * (1.0 - (-t / 10.0).exp())) * x
    }

    fn update_odometry(&mut self) {
        let left_pos = self.left_motors[0].position().unwrap_or_default();
        let right_pos = self.right_motors[0].position().unwrap_or_default();
        let parallel_pos = self.parallel_wheel.position().unwrap_or_default();
        let perpendicular_pos = self.perpendicular_wheel.position().unwrap_or_default();
        let imu_heading_rad = self.imu.rotation().unwrap_or_default().to_radians();
        self.odometry
            .update(left_pos, right_pos, Some(parallel_pos), Some(perpendicular_pos), imu_heading_rad);
    }

    fn check_stall(&mut self) -> bool {
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
            || avg_velocity <= self.config.stall_velocity_threshold
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

    // https://wiki.purduesigbots.com/software/robotics-basics/curvature-cheesy-drive
    fn cheesy_turn_remap(&self, iturn: f64) -> f64 {
        const CD_TURN_NONLINEARITY: f64 = 0.65;
        let denom = (PI / 2.0 * CD_TURN_NONLINEARITY).sin();
        let first = (PI / 2.0 * CD_TURN_NONLINEARITY * iturn).sin() / denom;
        (PI / 2.0 * CD_TURN_NONLINEARITY * first).sin() / denom
    }
    fn update_cheesy_accumulators(&mut self) {
        if self.neg_inertia_accumulator > 1.0 {
            self.neg_inertia_accumulator -= 1.0;
        } else if self.neg_inertia_accumulator < -1.0 {
            self.neg_inertia_accumulator += 1.0;
        } else {
            self.neg_inertia_accumulator = 0.0;
        }
        if self.quick_stop_accumulator > 1.0 {
            self.quick_stop_accumulator -= 1.0;
        } else if self.quick_stop_accumulator < -1.0 {
            self.quick_stop_accumulator += 1.0;
        } else {
            self.quick_stop_accumulator = 0.0;
        }
    }
    pub fn cheesy_control(&mut self) {
        self.update_odometry();
        let c_state = self.controller.state().unwrap_or_default();
        let throttle = c_state.left_stick.y();
        let turn = c_state.right_stick.x();
        // taken from BLRS repo
        const DRIVE_DEADBAND: f64 = 0.05;
        const DRIVE_SLEW: f64 = 0.02;
        const CD_NEG_INERTIA_SCALAR: f64 = 4.0;
        const CD_SENSITIVITY: f64 = 1.0;
        let mut turn_in_place = false;
        let mut linear_cmd = throttle;
        if throttle.abs() < DRIVE_DEADBAND && turn.abs() > DRIVE_DEADBAND {
            linear_cmd = 0.0;
            turn_in_place = true;
        } else if throttle - self.prev_throttle > DRIVE_SLEW {
            linear_cmd = self.prev_throttle + DRIVE_SLEW;
        } else if throttle - self.prev_throttle < -(DRIVE_SLEW * 2.0) {
            linear_cmd = self.prev_throttle - (DRIVE_SLEW * 2.0);
        }
        let remapped_turn = self.cheesy_turn_remap(turn);
        let (left, right) = if turn_in_place {
            let l = remapped_turn * remapped_turn.abs();
            let r = -remapped_turn * remapped_turn.abs();
            (l, r)
        } else {
            let neg_inertia_power = (turn - self.prev_turn) * CD_NEG_INERTIA_SCALAR;
            self.neg_inertia_accumulator += neg_inertia_power;
            let angular_cmd = linear_cmd.abs() * (remapped_turn + self.neg_inertia_accumulator) * CD_SENSITIVITY - self.quick_stop_accumulator;
            let mut l = linear_cmd;
            let mut r = linear_cmd;
            l += angular_cmd;
            r -= angular_cmd;
            self.update_cheesy_accumulators();
            (l, r)
        };
        self.prev_turn = turn;
        self.prev_throttle = throttle;

        let curved_left = self.drive_curve(left * 127.0, 5.0) / 127.0;
        let curved_right = self.drive_curve(right * 127.0, 5.0) / 127.0;

        let max_volts = self.config.max_volts;
        let vl = (curved_left * max_volts).clamp(-max_volts, max_volts);
        let vr = (curved_right * max_volts).clamp(-max_volts, max_volts);
        for m in self.left_motors.iter_mut() {
            let _ = m.set_voltage(vl);
        }
        for m in self.right_motors.iter_mut() {
            let _ = m.set_voltage(vr);
        }
    }
    pub async fn drive_ptp(
        &mut self,
        points: &[(Pose, PoseSettings)],
    ) {
        for (i, (pose, settings)) in points.iter().enumerate() {
            self.drive_to_point(pose.x, pose.y, *settings)
                .await;
            self.triggers.check_index(i);
        }
    }

    fn create_motion_profile(&self, interpolated_path: Vec<(Pose, PoseSettings)>, raw_waypoints: &[(Pose, PoseSettings)]) -> Profile {
        let mut path_points = interpolated_path.iter().map(|p| p.0).collect::<Vec<_>>();
        let m = path_points.len();
        let mut profile_points = Vec::with_capacity(m);

        if m == 0 {
            return Profile {
                profile_points,
                times: Vec::new(),
                total_time: 0.0,
            };
        }

        for i in 0..m {
            if i < m - 1 {
                path_points[i].heading = (path_points[i+1].y - path_points[i].y).atan2(path_points[i+1].x - path_points[i].x);
            } else if m > 1 {
                path_points[i].heading = path_points[i-1].heading;
            } else {
                path_points[i].heading = 0.0;
            }
        }
        
        let mut cum_d = 0.0;
        profile_points.push(ProfilePoint { 
            point: path_points[0], 
            distance: 0.0, 
            curvature: 0.0, 
            velocity: 0.0, 
            is_reversed: raw_waypoints[0].1.is_reversed, 
            max_speed: raw_waypoints[0].1.max_voltage 
        });
        
        for i in 1..m {
            let d = (path_points[i].x - path_points[i-1].x).hypot(path_points[i].y - path_points[i-1].y);
            cum_d += d;
            let raw_idx_for_props = ((i * (raw_waypoints.len() - 1)) / (m - 1)).min(raw_waypoints.len() - 1);
            profile_points.push(ProfilePoint { 
                point: path_points[i], 
                distance: cum_d, 
                curvature: 0.0, 
                velocity: 0.0, 
                is_reversed: raw_waypoints[raw_idx_for_props].1.is_reversed, 
                max_speed: raw_waypoints[raw_idx_for_props].1.max_voltage 
            });
        }
        
        profile_points.iter_mut().filter(|p| p.is_reversed).for_each(|p| {
            p.point.heading += PI;
            p.point.heading = self.normalize_angle(p.point.heading);
        });
        
        for i in 1..(m-1) {
            let a = &profile_points[i-1].point;
            let b = &profile_points[i].point;
            let c = &profile_points[i+1].point;

            let ab_len = (b.x - a.x).hypot(b.y - a.y);
            let bc_len = (c.x - b.x).hypot(c.y - b.y);
            let ac_len = (c.x - a.x).hypot(c.y - a.y);

            let area = 0.5 * ((a.x * (b.y - c.y)) + (b.x * (c.y - a.y)) + (c.x * (a.y - b.y)));
            let denom = ab_len * bc_len * ac_len;
            if denom > 1e-9 {
                profile_points[i].curvature = 4.0 * area.abs() / denom;
            } else {
                profile_points[i].curvature = 0.0;
            }
        }
        
        let w_circ = self.config.wheel_diameter * PI;
        let rpm = self.motor_free_rpm / self.config.ext_gear_ratio;
        let max_v = (rpm / 60.0) * w_circ;
        let max_a = max_v / self.config.accel_t;
        let k = 2.0;
        
        for p in profile_points.iter_mut() {
            let point_max_v = (p.max_speed / self.config.max_volts) * max_v;
            let mut v = point_max_v;
            if p.curvature > 1e-6 {
                let v_k_curv_limit = k / p.curvature;
                v = v.min(v_k_curv_limit);
            }
            p.velocity = v;
        }
        
        for i in 0..(m - 1) {
            let d = profile_points[i+1].distance - profile_points[i].distance;
            if d < 1e-9 {
                profile_points[i+1].velocity = profile_points[i+1].velocity.min(profile_points[i].velocity);
                continue;
            }
            let vf = profile_points[i].velocity.powi(2) + 2.0 * max_a * d;
            if vf >= 0.0 {
                profile_points[i+1].velocity = profile_points[i+1].velocity.min(vf.sqrt());
            } else {
                profile_points[i+1].velocity = profile_points[i+1].velocity.min(0.0);
            }
        }
        profile_points[0].velocity = 0.0;
        profile_points[m-1].velocity = 0.0;
        
        for i in (0..(m-1)).rev() {
            let d = profile_points[i+1].distance - profile_points[i].distance;
            if d < 1e-9 {
                profile_points[i].velocity = profile_points[i].velocity.min(profile_points[i+1].velocity);
                continue;
            }
            let vf_old = profile_points[i+1].velocity.powi(2) + 2.0 * max_a * d;
            if vf_old >= 0.0 {
                 profile_points[i].velocity = profile_points[i].velocity.min(vf_old.sqrt());
            } else {
                 profile_points[i].velocity = profile_points[i].velocity.min(0.0);
            }
        }

        let mut times = Vec::with_capacity(m);
        times.push(0.0);
        for i in 1..m {
            let d = profile_points[i].distance - profile_points[i - 1].distance;
            let v = profile_points[i].velocity;
            times.push(times[i - 1] + d / v);
        }
        let total_time = *times.last().unwrap_or(&0.0);

        Profile { profile_points, times, total_time }
    }
    pub async fn drive_ramsete_catmull_rom(
        &mut self,
        path_waypoints: &[(Pose, PoseSettings)],
        steps: usize,
        b: f64,
        zeta: f64,
    ) {
        let interpolated_path = interpolate_catmull_rom(path_waypoints, steps);
        let initialized_profile = self.create_motion_profile(interpolated_path, path_waypoints);
        let profile = initialized_profile.profile_points;
        let times = initialized_profile.times;
        let total_time = initialized_profile.total_time;
        
        self.execute_ramsete_profile(profile, times, total_time, b, zeta, steps).await;
    }

    async fn execute_ramsete_profile(
        &mut self,
        profile: Vec<ProfilePoint>,
        times: Vec<f64>,
        total_time: f64,
        b: f64,
        zeta: f64,
        steps: usize,
    ) {
        let m = profile.len();
        if m < 2 {
            println!("ramsete: too few points");
            return;
        }
        let mut segments = Vec::<Segment>::with_capacity(m - 1);
        for i in 0..(m - 1) {
            let p0_h = profile[i].point.heading;
            let p1_h = profile[i+1].point.heading;
            
            let current_delta_h = self.normalize_angle(p1_h - p0_h);
            let delta_t = times[i+1] - times[i];
            
            let w_d = if delta_t.abs() < 1e-9 {
                if current_delta_h.abs() < 1e-9 { 0.0 } else { current_delta_h * 1e9 }
            } else {
                current_delta_h / delta_t
            };
            segments.push(Segment {
                delta_h: current_delta_h,
                t: delta_t,
                w_d,
            });
        }

        let wp_count = if steps > 0 { (m - 1) / steps + 1 } else { 1 };
        let mut last_wp = None;

        let mut last_time = Instant::now();
        let start_time = last_time;
        let mut seg_i = 0;
        let mut small_timer: Option<Instant> = None;
        let mut big_timer: Option<Instant> = None;
        let mut vel_timer: Option<Instant> = None;

        loop {
            let now = Instant::now();
            let dt_loop = now.duration_since(last_time).as_secs_f64();
            if dt_loop < self.config.dt.as_secs_f64() {
                sleep(self.config.dt - now.duration_since(last_time)).await;
                continue;
            }
            last_time = now;
            let elapsed = now.duration_since(start_time).as_secs_f64();
            self.update_odometry();
            
            let mut total_velocity = 0.0;
            let mut num_motors = 0;
            for motor in self.left_motors.iter().chain(self.right_motors.iter()) {
                total_velocity += motor.velocity().unwrap_or_default().abs();
                num_motors += 1;
            }
            let avg_velocity = if num_motors > 0 { total_velocity / num_motors as f64 } else { 0.0 };
            
            let current = self.odometry.pose();
            let goal = profile.last().unwrap().point;
            let dx_end = goal.x - current.x;
            let dy_end = goal.y - current.y;
            let dist_err = dx_end.hypot(dy_end);
            
            if dist_err <= self.config.small_drive_exit_error {
                big_timer = None;
                vel_timer = None;
                if small_timer.is_none() {
                    small_timer = Some(Instant::now());
                }
                if let Some(t) = small_timer
                    && t.elapsed() >= self.config.small_drive_settle_time {
                        println!("ramsete: done (small error)");
                        break;
                    }
            } else {
                small_timer = None;
                if dist_err <= self.config.big_drive_exit_error {
                    vel_timer = None;
                    if big_timer.is_none() {
                        big_timer = Some(Instant::now());
                    }
                    if let Some(t) = big_timer
                        && t.elapsed() >= self.config.big_drive_settle_time {
                            println!("ramsete: done (big error)");
                            break;
                        }
                } else {
                    big_timer = None;
                    if avg_velocity <= self.config.stall_velocity_threshold {
                        if vel_timer.is_none() {
                            vel_timer = Some(Instant::now());
                        }
                        if let Some(t) = vel_timer
                            && t.elapsed() >= self.config.stall_time {
                                println!("ramsete: done (low velocity detected)");
                                break;
                            }
                    } else {
                        vel_timer = None;
                    }
                }
            }
            
            if elapsed >= total_time {
                println!("ramsete: done (timeout)");
                break;
            }

            while seg_i < m - 2 && elapsed >= times[seg_i + 1] {
                seg_i += 1;
            }
            let i = seg_i;

            if wp_count > 1 {
                let wp_idx = if i >= (wp_count - 1) * steps {
                    wp_count - 1
                } else {
                    i / steps
                };
                
                if Some(wp_idx) != last_wp && wp_idx > 0 {
                    self.triggers.check_index(wp_idx);
                    last_wp = Some(wp_idx);
                }
            }
            let seg_data = &segments[i];

            let p0 = &profile[i].point;
            let p1 = &profile[i + 1].point;
            let p0_heading = p0.heading;

            let (tau, w_d) = if seg_data.t.abs() < 1e-9 {
                (0.0, seg_data.w_d)
            } else {
                let t0 = times[i];
                (((elapsed - t0) / seg_data.t).clamp(0.0, 1.0), seg_data.w_d)
            };

            let traveled = profile[i].distance + (profile[i+1].distance - profile[i].distance) * tau;
            self.triggers.check_distance(traveled);

            let x_d = p0.x + (p1.x - p0.x) * tau;
            let y_d = p0.y + (p1.y - p0.y) * tau;
            
            let mut heading_d = p0_heading + seg_data.delta_h * tau;
            heading_d = self.normalize_angle(heading_d);
            
            let v0 = profile[i].velocity;
            let v1 = profile[i + 1].velocity;
            let reversed = if profile[i].is_reversed {
                -1.0
            } else {
                1.0
            };
            let v_d = v0 + (v1 - v0) * tau * reversed;
            
            self.update_odometry();
            let pose = self.odometry.pose();
            let dx = x_d - pose.x;
            let dy = y_d - pose.y;
            let e_x = dx * pose.heading.cos() + dy * pose.heading.sin();
            let e_y = -dx * pose.heading.sin() + dy * pose.heading.cos();
            let e_theta = self.normalize_angle(heading_d - pose.heading);

            let k = 2.0 * zeta * (w_d * w_d + b * v_d * v_d).sqrt();
            let v = v_d * e_theta.cos() + k * e_x;
            let w = if e_theta.abs() < 1e-9 {
                w_d + k * e_theta
            } else {
                w_d + k * e_theta + (b * v_d * e_theta.sin() * e_y) / e_theta
            };
            
            let wheel_vel_l = v + w * self.config.track_width / 2.0;
            let wheel_vel_r = v - w * self.config.track_width / 2.0;
            let max_linear_vel = (self.motor_free_rpm / self.config.ext_gear_ratio / 60.0) * (self.config.wheel_diameter * PI);
            let vl = (wheel_vel_l / max_linear_vel * self.config.max_volts).clamp(-self.config.max_volts, self.config.max_volts);
            let vr = (wheel_vel_r / max_linear_vel * self.config.max_volts).clamp(-self.config.max_volts, self.config.max_volts);
            for m in self.left_motors.iter_mut() { let _ = m.set_voltage(vl); }
            for m in self.right_motors.iter_mut() { let _ = m.set_voltage(vr); }
        }
        for m in self.left_motors.iter_mut().chain(self.right_motors.iter_mut()) {
            let _ = m.brake(BrakeMode::Hold);
        }
    }

    pub async fn drive_to_point(
        &mut self,
        target_x: f64,
        target_y: f64,
        settings: PoseSettings,
    ) {
        self.update_odometry();
        let pose_before_turn = self.odometry.pose();
        let angle_to_target =
            (target_y - pose_before_turn.y).atan2(target_x - pose_before_turn.x);

        let target_orientation = if settings.is_reversed {
            self.normalize_angle(angle_to_target + PI)
        } else {
            angle_to_target
        };

        self.turn_to_angle(target_orientation.to_degrees(), settings.max_voltage)
            .await;

        self.update_odometry();
        let pose_after_turn = self.odometry.pose();
        let dx = target_x - pose_after_turn.x;
        let dy = target_y - pose_after_turn.y;
        let dist_to_target = dx.hypot(dy);

        let drive_dist = if settings.is_reversed {
            -dist_to_target
        } else {
            dist_to_target
        };

        if drive_dist.abs() > 0.01 {
            self.drive_straight(drive_dist, settings.max_voltage)
                .await;
        }
    }

    pub async fn turn_to_point(&mut self, x: f64, y: f64, v: f64) {
        self.update_odometry();
        let p = self.odometry.pose();
        let a = (y - p.y).atan2(x - p.x);
        self.turn_to_angle(a.to_degrees(), v).await;
    }

    pub async fn turn_to_angle(&mut self, a_deg: f64, v: f64) {
        let mut pid = self.config.turn_pid;
        pid.reset();
        self.stall_timer = None;
        let a = self.normalize_angle(a_deg.to_radians());
        let initial_heading = self.odometry.pose().heading;
        let mut t0 = Instant::now();
        let mut ts: Option<Instant> = None;
        let mut tb: Option<Instant> = None;

        loop {
            let t = Instant::now();
            let dt = t.duration_since(t0).as_secs_f64();

            if dt < self.config.dt.as_secs_f64() {
                sleep(self.config.dt - t.duration_since(t0)).await;
                continue;
            }
            t0 = t;

            self.update_odometry();
            let h = self.odometry.pose().heading;
            let dh = self.normalize_angle(h - initial_heading);
            self.triggers.check_angle(dh.to_degrees());
            let e = self.normalize_angle(a - h);

            if self.check_stall() {
                println!("turn_to_angle: stall detected");
                break;
            }
            
            let e2 = e.to_degrees().abs();
            if e2 <= self.config.small_turn_exit_error {
                tb = None;
                if ts.is_none() {
                    ts = Some(Instant::now());
                }
                if let Some(t) = ts
                    && t.elapsed() >= self.config.small_turn_settle_time {
                        println!("turn_to_angle: done (small error)");
                        break;
                    }
            } else {
                ts = None;
                if e2 <= self.config.big_turn_exit_error {
                    if tb.is_none() {
                        tb = Some(Instant::now());
                    }
                    if let Some(t) = tb
                        && t.elapsed() >= self.config.big_turn_settle_time {
                            println!("turn_to_angle: done (big error)");
                            break;
                        }
                } else {
                    tb = None;
                }
            }

            let u = pid.next(e, dt).clamp(-v, v);
            for m in self.left_motors.iter_mut() { let _ = m.set_voltage(u); }
            for m in self.right_motors.iter_mut() { let _ = m.set_voltage(-u); }
        }

        for m in self.left_motors.iter_mut().chain(self.right_motors.iter_mut()) { let _ = m.brake(BrakeMode::Hold); }
    }

    pub async fn drive_straight(&mut self, target_dist: f64, max_voltage: f64) {
        let mut drive_pid = self.config.drive_pid;
        let mut heading_pid = self.config.heading_pid;
        drive_pid.reset();
        heading_pid.reset();
        self.stall_timer = None;

        let initial_pose = self.odometry.pose();
        let initial_heading = initial_pose.heading;
        let mut last_time = Instant::now();
        let mut small_error_timer: Option<Instant> = None;
        let mut big_error_timer: Option<Instant> = None;

        loop {
            let now = Instant::now();
            let dt = now.duration_since(last_time).as_secs_f64();
            if dt < self.config.dt.as_secs_f64() {
                sleep(self.config.dt - now.duration_since(last_time)).await;
                continue;
            }
            last_time = now;

            self.update_odometry();
            if self.check_stall() {
                println!("drive_straight: stall detected");
                break;
            }

            let current = self.odometry.pose();
            let dx = current.x - initial_pose.x;
            let dy = current.y - initial_pose.y;
            let traveled = (dx.hypot(dy))
                * (dx * initial_heading.cos() + dy * initial_heading.sin()).signum();
            self.triggers.check_distance(traveled.abs());

            let err = target_dist - traveled;
            if err.abs() <= self.config.small_drive_exit_error {
                big_error_timer = None;
                if small_error_timer.is_none() {
                    small_error_timer = Some(Instant::now());
                }
                if let Some(t) = small_error_timer
                    && t.elapsed() >= self.config.small_drive_settle_time {
                        println!("drive_straight: done (small error)");
                        break;
                    }
            } else {
                small_error_timer = None;
                if err.abs() <= self.config.big_drive_exit_error {
                    if big_error_timer.is_none() {
                        big_error_timer = Some(Instant::now());
                    }
                    if let Some(t) = big_error_timer
                        && t.elapsed() >= self.config.big_drive_settle_time {
                            println!("drive_straight: done (big error)");
                            break;
                        }
                } else {
                    big_error_timer = None;
                }
            }

            let u = drive_pid.next(err, dt);
            let heading_error = self.normalize_angle(initial_heading - current.heading);
            let tc = heading_pid.next(heading_error, dt)
                .clamp(-self.config.max_volts, self.config.max_volts);

            let vl = (u + tc).clamp(-max_voltage, max_voltage);
            let vr = (u - tc).clamp(-max_voltage, max_voltage);
            for m in self.left_motors.iter_mut() { let _ = m.set_voltage(vl); }
            for m in self.right_motors.iter_mut() { let _ = m.set_voltage(vr); }
        }

        for m in self.left_motors.iter_mut().chain(self.right_motors.iter_mut()) {
            let _ = m.brake(BrakeMode::Hold);
        }
    }
}
