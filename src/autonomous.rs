use core::{f64::consts::PI, time::Duration};

use super::chassis::Chassis;
extern crate alloc;
use alloc::vec::Vec;

use vexide::{prelude::*, time::Instant};

use crate::{
    chassis::PoseSettings,
    motion_controller::{MotionProfile2DPoint, compute_motion_profile, TrapezoidalProfile1D},
    odometry::Pose,
    utils::{normalize_angle, path::interpolate_curve},
};

struct Segment {
    delta_h: f64,
    t: f64,
}

impl<const L: usize, const R: usize, const I: usize> Chassis<L, R, I> {
    pub async fn drive_spline(
        &mut self,
        path_waypoints: &[(Pose, PoseSettings)],
        step_size: f64,
        b: f64,
        zeta: f64,
        fast: bool,
    ) {
        // rebase path to start at current pose by prepending current pose as first waypoint
        // inherit settings from the original first waypoint if present
        let mut rebased: Vec<(Pose, PoseSettings)> = Vec::with_capacity(path_waypoints.len() + 1);
        let start_pose = self.mcl_pose();
        let first_settings = path_waypoints.first().map(|p| p.1).unwrap_or_default();
        rebased.push((start_pose, first_settings));
        rebased.extend_from_slice(path_waypoints);

        let interpolated_path = interpolate_curve(&rebased, step_size);
        let initialized_profile =
            compute_motion_profile::<L, R, I>(self, interpolated_path, &rebased);
        let profile = initialized_profile.profile_points;
        let times = initialized_profile.times;
        let total_time = initialized_profile.total_time;
        // 1550.0 is approx 39.37^2, converting b (m^-2) to in^-2
        let b_inches = b / 1550.0;
        self.execute_ramsete_profile(profile, times, total_time, b_inches, zeta, fast)
            .await;
    }

    async fn execute_ramsete_profile(
        &mut self,
        profile: Vec<MotionProfile2DPoint>,
        times: Vec<f64>,
        total_time: f64,
        b: f64,
        zeta: f64,
        fast: bool,
    ) {
        let m = profile.len();
        if m < 2 {
            println!("ramsete: too few points");
            return;
        }
        let mut segments = Vec::<Segment>::with_capacity(m - 1);
        for i in 0..(m - 1) {
            let p0_h = profile[i].point.heading;
            let p1_h = profile[i + 1].point.heading;
            let current_delta_h = normalize_angle(p1_h - p0_h);
            let delta_t = times[i + 1] - times[i];
            segments.push(Segment {
                delta_h: current_delta_h,
                t: delta_t,
            });
        }

        let mut last_wp = None;

        let mut last_time = Instant::now();
        let start_time = last_time;
        let mut seg_i = 0;
        let mut small_timer: Option<Instant> = None;
        let mut big_timer: Option<Instant> = None;
        let safety_timeout =
            Duration::from_millis(((total_time * 1000.0) as u64).saturating_add(100));

        let mut settle_drive_pid = self.config.drive_pid;
        let mut settle_turn_pid = self.config.turn_pid;
        settle_drive_pid.reset();
        settle_turn_pid.reset();

        loop {
            let now = Instant::now();
            let dt_loop = now.duration_since(last_time).as_secs_f64();
            if dt_loop < self.config.dt.as_secs_f64() {
                sleep(self.config.dt - now.duration_since(last_time)).await;
                continue;
            }
            last_time = now;
            let elapsed = now.duration_since(start_time).as_secs_f64();
            self.update_state();
            self.service_autonomous_intake();
            if now.duration_since(start_time) >= safety_timeout {
                println!("ramsete: timeout");
                break;
            }

            let current = self.mcl_pose();
            let goal = profile.last().unwrap().point;
            let dx_end = goal.x - current.x;
            let dy_end = goal.y - current.y;
            let dist_err = dx_end.hypot(dy_end);
            let heading_err = normalize_angle(goal.heading - current.heading)
                .abs()
                .to_degrees();

            let position_satisfied = dist_err <= self.config.small_drive_exit_error;
            let heading_satisfied = heading_err <= self.config.small_turn_exit_error;

            if position_satisfied && heading_satisfied {
                big_timer = None;
                if small_timer.is_none() {
                    small_timer = Some(Instant::now());
                }
                if let Some(t) = small_timer
                    && t.elapsed() >= self.config.small_drive_settle_time
                {
                    println!("ramsete: done (small error)");
                    break;
                }
            } else {
                small_timer = None;
                let big_position = dist_err <= self.config.big_drive_exit_error;
                let big_heading = heading_err <= self.config.big_turn_exit_error;
                if big_position && big_heading {
                    if fast {
                        println!("ramsete: done (fast)");
                        break;
                    }
                    if big_timer.is_none() {
                        big_timer = Some(Instant::now());
                    }
                    if let Some(t) = big_timer
                        && t.elapsed() >= self.config.big_drive_settle_time
                    {
                        println!("ramsete: done (big error)");
                        break;
                    }
                } else {
                    big_timer = None;
                }
            }

            let mut target_vl;
            let mut target_vr;
            // fall back to PID settling at the end of the profile
            if elapsed >= total_time {
                let final_pose = profile.last().unwrap().point;
                let current = self.mcl_pose();

                let dx = final_pose.x - current.x;
                let dy = final_pose.y - current.y;
                let dist_error = dx * final_pose.heading.cos() + dy * final_pose.heading.sin();

                let heading_error = normalize_angle(final_pose.heading - current.heading);

                let mut drive_output = settle_drive_pid.next(dist_error, dt_loop);
                let turn_output = settle_turn_pid.next(heading_error, dt_loop);

                const TURN_BIAS: f64 = 0.8;
                let turn_bias_scale =
                    (1.0 - ((1.0 - heading_error.cos()) / TURN_BIAS)).clamp(0.0, 1.0);
                drive_output *= turn_bias_scale;

                target_vl = drive_output - turn_output;
                target_vr = drive_output + turn_output;
            } else {
                while seg_i < m - 2 && elapsed >= times[seg_i + 1] {
                    seg_i += 1;
                }
                let i = seg_i;

                let wp_idx = profile[i].waypoint_index;
                if Some(wp_idx) != last_wp && wp_idx > 0 {
                    let actions = self.triggers.check_index(wp_idx);
                    self.apply_trigger_actions(&actions);
                    last_wp = Some(wp_idx);
                }
                let seg_data = &segments[i];

                let p0 = &profile[i].point;
                let p1 = &profile[i + 1].point;
                let p0_heading = p0.heading;

                let (tau, w_r) = if seg_data.t.abs() < 1e-9 {
                    (0.0, 0.0)
                } else {
                    let t0 = times[i];
                    let tau_local = ((elapsed - t0) / seg_data.t).clamp(0.0, 1.0);
                    // interpolate reference angular velocity
                    let w0 = profile[i].angular_velocity;
                    let w1 = profile[i + 1].angular_velocity;
                    let w_interp = w0 + (w1 - w0) * tau_local;
                    (tau_local, w_interp)
                };

                let max_voltage_0 = profile[i].max_voltage.min(self.config.max_volts);
                let max_voltage_1 = profile[i + 1].max_voltage.min(self.config.max_volts);
                let seg_max_voltage = if max_voltage_0 <= 0.0 && max_voltage_1 <= 0.0 {
                    0.0
                } else {
                    let interpolated = max_voltage_0 + (max_voltage_1 - max_voltage_0) * tau;
                    interpolated.clamp(0.0, self.config.max_volts)
                };

                let traveled =
                    profile[i].distance + (profile[i + 1].distance - profile[i].distance) * tau;
                let actions = self.triggers.check_distance(traveled);
                self.apply_trigger_actions(&actions);

                let x_d = p0.x + (p1.x - p0.x) * tau;
                let y_d = p0.y + (p1.y - p0.y) * tau;

                let mut heading_d = p0_heading + seg_data.delta_h * tau;
                heading_d = normalize_angle(heading_d);

                let v0 = profile[i].velocity;
                let v1 = profile[i + 1].velocity;
                let reversed = if profile[i].is_reversed { -1.0 } else { 1.0 };
                let v_d = (v0 + (v1 - v0) * tau) * reversed;
                let w_r = w_r * reversed;

                let pose = self.mcl_pose();
                let dx = x_d - pose.x;
                let dy = y_d - pose.y;
                let e_x = dx * pose.heading.cos() + dy * pose.heading.sin();
                let e_y = -dx * pose.heading.sin() + dy * pose.heading.cos();
                let e_theta = normalize_angle(heading_d - pose.heading);

                #[inline]
                fn sinc(x: f64) -> f64 {
                    if x.abs() < 1e-4 {
                        1.0 - x * x / 6.0
                    } else {
                        x.sin() / x
                    }
                }
                let k = 2.0 * zeta * (w_r * w_r + b * v_d * v_d).sqrt();
                let v = v_d * e_theta.cos() + k * e_x;
                let w = w_r + k * e_theta + b * v_d * sinc(e_theta) * e_y;

                let a0 = profile[i].acceleration;
                let a1 = profile[i + 1].acceleration;
                let a_profile = a0 + (a1 - a0) * tau;
                let a_vr = a_profile * reversed;

                let alpha0 = profile[i].angular_acceleration;
                let alpha1 = profile[i + 1].angular_acceleration;
                let a_wr = (alpha0 + (alpha1 - alpha0) * tau) * reversed;

                let ks = self.config.ff_ks;
                let kv_lin = self.config.ff_kv;
                let ka_lin = self.config.ff_ka;
                let kv_ang = self.config.ff_kv_ang;
                let ka_ang = self.config.ff_ka_ang;

                let u_lin = kv_lin * v + ka_lin * a_vr;
                let u_ang = kv_ang * w + ka_ang * a_wr;

                target_vl = u_lin - u_ang;
                target_vr = u_lin + u_ang;

                target_vl += ks * target_vl.signum();
                target_vr += ks * target_vr.signum();

                if seg_max_voltage <= 0.0 {
                    target_vl = 0.0;
                    target_vr = 0.0;
                } else {
                    let peak = target_vl.abs().max(target_vr.abs());
                    if peak > seg_max_voltage {
                        let linear_component = u_lin + ks * u_lin.signum();
                        let angular_component_l = -u_ang;
                        let angular_component_r = u_ang;

                        let mut scaled_linear = linear_component;
                        if linear_component.abs() > seg_max_voltage {
                            scaled_linear = seg_max_voltage * linear_component.signum();
                        }

                        target_vl = scaled_linear + angular_component_l;
                        target_vr = scaled_linear + angular_component_r;

                        let final_peak = target_vl.abs().max(target_vr.abs());
                        if final_peak > self.config.max_volts {
                            let scale = self.config.max_volts / final_peak;
                            target_vl *= scale;
                            target_vr *= scale;
                        }
                    }
                }

            }

            let vl = target_vl.clamp(-self.config.max_volts, self.config.max_volts);
            let vr = target_vr.clamp(-self.config.max_volts, self.config.max_volts);
            if vl.abs().max(vr.abs()) >= 3.0 && self.check_stall() {
                println!("ramsete: stall detected");
                break;
            }
            for m in self.left_motors.iter_mut() {
                let _ = m.set_voltage(vl);
            }
            for m in self.right_motors.iter_mut() {
                let _ = m.set_voltage(vr);
            }
        }
        for m in self
            .left_motors
            .iter_mut()
            .chain(self.right_motors.iter_mut())
        {
            let _ = m.brake(BrakeMode::Brake);
        }
    }

    pub async fn drive_linear(&mut self, points: &[(Pose, PoseSettings)]) {
        if points.is_empty() {
            return;
        }
        let settings = points.last().unwrap().1;
        let target = points.last().unwrap().0;
        let mut heading_pid = self.config.heading_pid;
        let mut drive_pid = self.config.drive_pid;
        heading_pid.reset();
        drive_pid.reset();
        self.stall_timer = None;

        self.update_state();
        let start_pose = self.odometry.pose();
        let initial_heading = start_pose.heading;

        let drive_direction = if settings.is_reversed { -1.0 } else { 1.0 };

        let dx = target.x - start_pose.x;
        let dy = target.y - start_pose.y;
        let total_dist = dx.hypot(dy);
        let (t_dir_x, t_dir_y) = if total_dist > 1e-6 {
            (dx / total_dist, dy / total_dist)
        } else {
            (initial_heading.cos(), initial_heading.sin())
        };
        if total_dist < self.config.small_drive_exit_error {
            return;
        }

        let mut last_time = Instant::now();
        let start_time = last_time;
        let max_linear_vel = (self.motor_free_rpm / self.config.ext_gear_ratio / 60.0)
            * (self.config.wheel_diameter * PI);
        
        let max_v = max_linear_vel * (settings.max_voltage / self.config.max_volts);
        let max_a = max_linear_vel / self.config.t_accel_drive;
        let max_d = max_linear_vel / self.config.t_decel_drive;
        let profile = TrapezoidalProfile1D::new(total_dist, max_v, max_a, max_d);
        
        let safety_timeout =
            Duration::from_millis(((profile.total_time * 1000.0) as u64).saturating_add(50));

        let mut small_timer: Option<Instant> = None;
        let mut big_timer: Option<Instant> = None;

        loop {
            let now = Instant::now();
            let dt = now.duration_since(last_time).as_secs_f64();
            if dt < self.config.dt.as_secs_f64() {
                sleep(self.config.dt - now.duration_since(last_time)).await;
                continue;
            }
            last_time = now;
            let elapsed = now.duration_since(start_time).as_secs_f64();

            self.update_state();
            self.service_autonomous_intake();
            if now.duration_since(start_time) >= safety_timeout {
                println!("drive_linear: timeout");
                break;
            }

            let pose = self.odometry.pose();
            let cx = pose.x - start_pose.x;
            let cy = pose.y - start_pose.y;
            let traveled_along = cx * t_dir_x + cy * t_dir_y;
            let error = total_dist - traveled_along; // signed: < 0 when overshot
            let error_abs = error.abs();

            let progress = traveled_along.clamp(0.0, total_dist);
            let actions = self.triggers.check_distance(progress);
            self.apply_trigger_actions(&actions);

            if error_abs <= self.config.small_drive_exit_error {
                big_timer = None;
                if small_timer.is_none() {
                    small_timer = Some(Instant::now());
                }
                if let Some(t) = small_timer
                    && t.elapsed() >= self.config.small_drive_settle_time
                {
                    println!("drive_linear: done (small error)");
                    break;
                }
            } else {
                small_timer = None;
                if error_abs <= self.config.big_drive_exit_error {
                    if settings.fast {
                        println!("drive_linear: done (fast)");
                        break;
                    }
                    if big_timer.is_none() {
                        big_timer = Some(Instant::now());
                    }
                    if let Some(t) = big_timer
                        && t.elapsed() >= self.config.big_drive_settle_time
                    {
                        println!("drive_linear: done (big error)");
                        break;
                    }
                } else {
                    big_timer = None;
                }
            }

            if self.check_stall() {
                println!("drive_linear: stall detected");
                break;
            }

            // try to maintain the initial heading for straight-line driving with small corrections
            let heading_error = normalize_angle(initial_heading - pose.heading);
            let heading_correction = heading_pid
                .next(heading_error, dt)
                .clamp(-settings.max_voltage, settings.max_voltage);

            let (p_ref, v_ref, a_ref) = profile.sample(elapsed);
            let error = p_ref - traveled_along;

            let kv = self.config.ff_kv;
            let ka = self.config.ff_ka;
            let ks = self.config.ff_ks;

            let u_ff = kv * v_ref + ka * a_ref;

            let mut drive_output = (u_ff + drive_pid.next(error, dt))
                .clamp(-settings.max_voltage, settings.max_voltage);
            // scale down linear speed when there is a large heading error
            const TURN_BIAS: f64 = 0.8;
            let turn_bias_scale = (1.0 - ((1.0 - heading_error.cos()) / TURN_BIAS)).clamp(0.0, 1.0);
            drive_output *= turn_bias_scale;
            drive_output *= drive_direction;

            // compute motor voltages combining forward drive and heading correction
            let mut left_v = drive_output - heading_correction;
            let mut right_v = drive_output + heading_correction;

            // add static friction feedforward
            if left_v.abs() > 1e-3 {
                left_v += ks * left_v.signum();
            }
            if right_v.abs() > 1e-3 {
                right_v += ks * right_v.signum();
            }

            left_v = left_v.clamp(-self.config.max_volts, self.config.max_volts);
            right_v = right_v.clamp(-self.config.max_volts, self.config.max_volts);

            for m in self.left_motors.iter_mut() {
                let _ = m.set_voltage(left_v);
            }
            for m in self.right_motors.iter_mut() {
                let _ = m.set_voltage(right_v);
            }
        }

        for m in self
            .left_motors
            .iter_mut()
            .chain(self.right_motors.iter_mut())
        {
            let _ = m.brake(BrakeMode::Brake);
        }
    }

    pub async fn drive_ptp(&mut self, points: &[(Pose, PoseSettings)]) {
        for (pose, settings) in points {
            self.drive_to_point(pose.x, pose.y, *settings).await;
        }
    }

    pub async fn drive_to_point(&mut self, target_x: f64, target_y: f64, settings: PoseSettings) {
        self.update_state();
        let pose = self.odometry.pose();
        let mut turn_heading = (target_y - pose.y).atan2(target_x - pose.x);
        if settings.is_reversed {
            turn_heading = normalize_angle(turn_heading + PI);
        }
        let turn_v = settings.max_voltage.min(self.config.max_volts).max(1.0);
        let heading_delta = normalize_angle(turn_heading - self.odometry.pose().heading);
        if heading_delta.abs().to_degrees() > self.config.small_turn_exit_error {
            self.turn_to_angle(turn_heading.to_degrees(), turn_v, false, settings.fast)
                .await;
        }

        self.update_state();
        let start = self.odometry.pose();
        let start_pose = Pose {
            x: start.x,
            y: start.y,
            heading: start.heading,
        };
        let end_pose = Pose {
            x: target_x,
            y: target_y,
            heading: start.heading,
        };
        let path: [(Pose, PoseSettings); 2] = [(start_pose, settings), (end_pose, settings)];
        self.drive_linear(&path).await;
    }

    pub async fn turn_to_point(&mut self, x: f64, y: f64, v: f64, reverse: bool, fast: bool) {
        self.update_state();
        let p = self.mcl_pose();
        let a = (y - p.y).atan2(x - p.x);
        self.turn_to_angle(a.to_degrees(), v, reverse, fast).await;
    }

    pub async fn turn_to_angle(&mut self, a_deg: f64, v: f64, reverse: bool, fast: bool) {
        let mut pid = self.config.turn_pid;
        pid.reset();
        self.stall_timer = None;
        let mut a = normalize_angle(a_deg.to_radians());
        let initial_heading = self.mcl_pose().heading;

        if reverse {
            let shortest_error = normalize_angle(a - initial_heading);
            if shortest_error > 0.0 {
                a -= 2.0 * PI;
            } else {
                a += 2.0 * PI;
            }
        }
        let mut t0 = Instant::now();
        let mut ts: Option<Instant> = None;
        let mut tb: Option<Instant> = None;
        let start_time = Instant::now();
        let diff = if reverse {
            a - initial_heading
        } else {
            normalize_angle(a - initial_heading)
        };
        let direction = diff.signum();
        let planned_delta = diff.abs();
        let max_linear_vel = (self.motor_free_rpm / self.config.ext_gear_ratio / 60.0)
            * (self.config.wheel_diameter * PI);
        let max_angular_vel = 2.0 * max_linear_vel / self.config.track_width;

        let max_v = max_angular_vel * (v.min(self.config.max_volts) / self.config.max_volts);
        let max_a = max_angular_vel / self.config.t_accel_turn;
        let max_d = max_angular_vel / self.config.t_decel_turn;

        let profile = TrapezoidalProfile1D::new(planned_delta, max_v, max_a, max_d);

        let safety_timeout =
            Duration::from_millis(((profile.total_time * 1000.0) as u64).saturating_add(50));

        let small_exit_deg = self.config.small_turn_exit_error;
        let big_exit_deg = self.config.big_turn_exit_error;

        let mut e_prev: Option<f64> = None;

        loop {
            let t = Instant::now();
            let dt = t.duration_since(t0).as_secs_f64();

            if dt < self.config.dt.as_secs_f64() {
                sleep(self.config.dt - t.duration_since(t0)).await;
                continue;
            }
            t0 = t;

            self.update_state();
            self.service_autonomous_intake();
            if t.duration_since(start_time) >= safety_timeout {
                println!("turn_to_angle: timeout");
                break;
            }
            let h = self.mcl_pose().heading;
            let dh = normalize_angle(h - initial_heading);
            let actions = self.triggers.check_angle(dh.to_degrees());
            self.apply_trigger_actions(&actions);
            let mut e = normalize_angle(a - h);

            if let Some(prev) = e_prev {
                let delta = e - prev;
                if delta > PI {
                    e -= 2.0 * PI;
                } else if delta < -PI {
                    e += 2.0 * PI;
                }
            }
            e_prev = Some(e);

            if self.check_stall() {
                println!("turn_to_angle: stall detected");
                break;
            }

            let e2 = e.to_degrees().abs();
            if e2 <= small_exit_deg {
                tb = None;
                if ts.is_none() {
                    ts = Some(Instant::now());
                }
                if let Some(t) = ts
                    && t.elapsed() >= self.config.small_turn_settle_time
                {
                    println!("turn_to_angle: done (small error)");
                    break;
                }
            } else {
                ts = None;
                if e2 <= big_exit_deg {
                    if fast {
                        println!("turn_to_angle: done (fast)");
                        break;
                    }
                    if tb.is_none() {
                        tb = Some(Instant::now());
                    }
                    if let Some(t) = tb
                        && t.elapsed() >= self.config.big_turn_settle_time
                    {
                        println!("turn_to_angle: done (big error)");
                        break;
                    }
                } else {
                    tb = None;
                }
            }
            let elapsed = t.duration_since(start_time).as_secs_f64();
            let (p_ref, v_ref, a_ref) = profile.sample(elapsed);

            let kv = self.config.ff_kv_ang;
            let ka = self.config.ff_ka_ang;
            let ks = self.config.ff_ks;

            let u_ff = (kv * v_ref + ka * a_ref) * direction;

            let target_h = initial_heading + direction * p_ref;
            let error_h = normalize_angle(target_h - h);

            let u_fb = pid.next(error_h, dt);

            let u = (u_ff + u_fb).clamp(-v, v);

            let mut left_v = -u;
            let mut right_v = u;

            if left_v.abs() > 1e-3 {
                left_v += ks * left_v.signum();
            }
            if right_v.abs() > 1e-3 {
                right_v += ks * right_v.signum();
            }

            left_v = left_v.clamp(-self.config.max_volts, self.config.max_volts);
            right_v = right_v.clamp(-self.config.max_volts, self.config.max_volts);

            for m in self.left_motors.iter_mut() {
                let _ = m.set_voltage(left_v);
            }
            for m in self.right_motors.iter_mut() {
                let _ = m.set_voltage(right_v);
            }
        }

        for m in self
            .left_motors
            .iter_mut()
            .chain(self.right_motors.iter_mut())
        {
            let _ = m.brake(BrakeMode::Brake);
        }
    }

    pub async fn calibrate_tracking_wheels(&mut self) {
        println!("starting tracking wheel offset calibration...");

        const ITERATIONS: usize = 10;

        let mut parallel_offset_sum = 0.0;
        let mut perpendicular_offset_sum = 0.0;

        self.odometry.reset(Pose::default());
        let _ = self.imu.reset_heading();

        for i in 0..ITERATIONS {
            let _ = self.imu.reset_heading();
            let _ = self.parallel_wheel.reset_position();
            let _ = self.perpendicular_wheel.reset_position();

            let target_deg = if i.is_multiple_of(2) { 90.0 } else { 270.0 };

            self.turn_to_angle(target_deg, 6.0, false, false).await;
            sleep(Duration::from_millis(250)).await;

            let imu_start = 0.0;
            let imu_end = -self.imu.rotation().unwrap_or_default().to_radians();
            let t_delta = normalize_angle(imu_end - imu_start);

            let parallel_pos = self.parallel_wheel.position().unwrap_or_default();
            let perpendicular_pos = self.perpendicular_wheel.position().unwrap_or_default();

            let parallel_delta = parallel_pos.as_revolutions();
            let perpendicular_delta = perpendicular_pos.as_revolutions();

            let wheel_circumference = self.config.tw_config.wheel_diameter * core::f64::consts::PI;

            let parallel_distance = parallel_delta * wheel_circumference;
            let perpendicular_distance = perpendicular_delta * wheel_circumference;

            if t_delta.abs() > 1e-6 {
                parallel_offset_sum += parallel_distance / t_delta;
                perpendicular_offset_sum += perpendicular_distance / t_delta;
            }
        }

        let parallel_offset = parallel_offset_sum / ITERATIONS as f64;
        let perpendicular_offset = perpendicular_offset_sum / ITERATIONS as f64;

        println!("parallel offset: {:.3} inches", parallel_offset);
        println!("perpendicular offset: {:.3} inches", perpendicular_offset);
    }

    pub async fn drive_straight(&mut self, target_dist: f64, max_voltage: f64, reverse: bool, fast: bool) {
        self.update_state();
        let start = self.odometry.pose();
        let effective_reverse = if reverse {
            true
        } else {
            target_dist.is_sign_negative()
        };
        let magnitude = target_dist.abs();
        let signed_dist = if effective_reverse {
            -magnitude
        } else {
            magnitude
        };
        let dx = start.heading.cos() * signed_dist;
        let dy = start.heading.sin() * signed_dist;
        let start_pose = Pose {
            x: start.x,
            y: start.y,
            heading: start.heading,
        };
        let end_pose = Pose {
            x: start.x + dx,
            y: start.y + dy,
            heading: start.heading,
        };
        let settings = PoseSettings {
            is_reversed: effective_reverse,
            max_voltage,
            fast,
        };
        let path: [(Pose, PoseSettings); 2] = [(start_pose, settings), (end_pose, settings)];
        self.drive_linear(&path).await;
    }

    pub async fn drive_for(&mut self, ms: u64, voltage: f64, reverse: bool) {
        let mut v = voltage;
        if reverse {
            v = -v;
        }
        let v_clamped = v.clamp(-self.config.max_volts, self.config.max_volts);
        let total = Duration::from_millis(ms);
        let mut elapsed = Duration::from_millis(0);
        let dt = self.config.dt;
        while elapsed < total {
            self.update_state();
            self.service_autonomous_intake();
            for m in self.left_motors.iter_mut() {
                let _ = m.set_voltage(v_clamped);
            }
            for m in self.right_motors.iter_mut() {
                let _ = m.set_voltage(v_clamped);
            }
            let step = if total - elapsed > dt {
                dt
            } else {
                total - elapsed
            };
            sleep(step).await;
            elapsed += step;
        }
        for m in self
            .left_motors
            .iter_mut()
            .chain(self.right_motors.iter_mut())
        {
            let _ = m.brake(BrakeMode::Brake);
        }
    }
}
