use core::{f64::consts::PI, time::Duration};

use super::chassis::Chassis;
extern crate alloc;
use alloc::vec::Vec;

use vexide::{prelude::*, time::Instant};

use crate::{
    chassis::PoseSettings,
    motion_controller::{ProfilePoint, compute_motion_profile},
    odometry::Pose,
    utils::{
        normalize_angle,
        path::interpolate_curve,
    },
};

struct Segment {
    delta_h: f64,
    t: f64,
    w_d: f64,
}

impl<const L: usize, const R: usize, const I: usize> Chassis<L, R, I> {
    pub async fn drive_spline(
        &mut self,
        path_waypoints: &[(Pose, PoseSettings)],
        steps: usize,
        b: f64,
        zeta: f64,
    ) {
        // rebase path to start at current pose by prepending current pose as first waypoint
        // inherit settings from the original first waypoint if present
        let mut rebased: Vec<(Pose, PoseSettings)> = Vec::with_capacity(path_waypoints.len() + 1);
        let mut start_pose = self.mcl_pose();
        // do not force tangent direction at start unless caller explicitly sets it in the first waypoint
        start_pose.heading = f64::NAN;
        let first_settings = path_waypoints
            .first()
            .map(|p| p.1)
            .unwrap_or_default();
        rebased.push((start_pose, first_settings));
        rebased.extend_from_slice(path_waypoints);

        let interpolated_path = interpolate_curve(&rebased, steps);
        let initialized_profile =
            compute_motion_profile::<L, R, I>(self, interpolated_path, &rebased);
        let profile = initialized_profile.profile_points;
        let times = initialized_profile.times;
        let total_time = initialized_profile.total_time;

        self.execute_ramsete_profile(profile, times, total_time, b, zeta, steps)
            .await;
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
            let p1_h = profile[i + 1].point.heading;

            let current_delta_h = normalize_angle(p1_h - p0_h);
            let delta_t = times[i + 1] - times[i];

            let w_d = current_delta_h / delta_t;
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
        let mut last_w: Option<f64> = None;
        let safety_timeout =
            Duration::from_millis(((total_time * 1000.0) as u64).saturating_add(1500));

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

            if dist_err <= self.config.small_drive_exit_error {
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
                if dist_err <= self.config.big_drive_exit_error {
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
                    let actions = self.triggers.check_index(wp_idx);
                    self.apply_trigger_actions(&actions);
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

            let pose = self.mcl_pose();
            let dx = x_d - pose.x;
            let dy = y_d - pose.y;
            let e_x = dx * pose.heading.cos() + dy * pose.heading.sin();
            let e_y = -dx * pose.heading.sin() + dy * pose.heading.cos();
            let e_theta = normalize_angle(heading_d - pose.heading);

            #[inline]
            fn sinc(x: f64) -> f64 {
                if x.abs() < 1e-4 {
                    // use second term of taylor series of sin(x)/x as a decent approximation
                    1.0 - x * x / 6.0
                } else {
                    x.sin() / x
                }
            }
            let k = 2.0 * zeta * (w_d * w_d + b * v_d * v_d).sqrt();
            let v = v_d * e_theta.cos() + k * e_x;
            let w = w_d + k * e_theta + b * v_d * sinc(e_theta) * e_y;

            let a_d = if seg_data.t.abs() > 1e-9 {
                ((v1 - v0) / seg_data.t) * reversed
            } else {
                0.0
            };
            let ks = self.config.ff_ks;
            let kv_lin = self.config.ff_kv;
            let ka_lin = self.config.ff_ka;
            let kv_ang = self.config.ff_kv_ang;
            let ka_ang = self.config.ff_ka_ang;

            // compute angular acceleration from discrete derivative over loop dt
            let alpha_d = if let Some(prev_w) = last_w {
                if dt_loop > 1e-6 {
                    (w - prev_w) / dt_loop
                } else {
                    0.0
                }
            } else {
                0.0
            };
            last_w = Some(w);

            let u_lin = kv_lin * v + ka_lin * a_d;
            let u_ang = kv_ang * w + ka_ang * alpha_d;

            let mut target_vl = u_lin - u_ang;
            let mut target_vr = u_lin + u_ang;

            // add static friction per side based on sign
            target_vl += ks * target_vl.signum();
            target_vr += ks * target_vr.signum();

            if seg_max_voltage <= 0.0 {
                target_vl = 0.0;
                target_vr = 0.0;
            } else {
                let peak = target_vl.abs().max(target_vr.abs());
                if peak > seg_max_voltage {
                    let scale = seg_max_voltage / peak;
                    target_vl *= scale;
                    target_vr *= scale;
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
            let _ = m.brake(BrakeMode::Hold);
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
        let estimated_vel = max_linear_vel * (settings.max_voltage / self.config.max_volts) * 0.8;
        let estimated_time = if estimated_vel > 0.1 {
            total_dist / estimated_vel
        } else {
            total_dist / 0.1
        };
        let safety_timeout =
            Duration::from_millis(((estimated_time * 1000.0) as u64).saturating_add(250));

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

            let mut drive_output = drive_pid
                .next(error, dt)
                .clamp(-settings.max_voltage, settings.max_voltage);
            // scale down linear speed when there is a large heading error
            const TURN_BIAS: f64 = 0.8;
            let turn_bias_scale = (1.0 - ((1.0 - heading_error.cos()) / TURN_BIAS)).clamp(0.0, 1.0);
            drive_output *= turn_bias_scale;
            drive_output *= drive_direction;

            // compute motor voltages combining forward drive and heading correction
            let left_v = (drive_output - heading_correction)
                .clamp(-self.config.max_volts, self.config.max_volts);
            let right_v = (drive_output + heading_correction)
                .clamp(-self.config.max_volts, self.config.max_volts);

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
            let _ = m.brake(BrakeMode::Hold);
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
            self.turn_to_angle(turn_heading.to_degrees(), turn_v, false).await;
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

    pub async fn turn_to_point(&mut self, x: f64, y: f64, v: f64) {
        self.update_state();
        let p = self.mcl_pose();
        let a = (y - p.y).atan2(x - p.x);
        self.turn_to_angle(a.to_degrees(), v, false).await;
    }

    pub async fn turn_to_angle(&mut self, a_deg: f64, v: f64, reverse: bool) {
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
        // estimate timeout similar to drive_linear but for angular motion
        let planned_delta = if reverse {
            (a - initial_heading).abs()
        } else {
            normalize_angle(a - initial_heading).abs()
        };
        let max_linear_vel = (self.motor_free_rpm / self.config.ext_gear_ratio / 60.0)
            * (self.config.wheel_diameter * PI);
        let voltage_scale = (v.min(self.config.max_volts) / self.config.max_volts).clamp(0.0, 1.0);
        let estimated_w = (2.0 * max_linear_vel / self.config.track_width) * voltage_scale * 0.8;
        let estimated_time = if estimated_w > 0.1 { planned_delta / estimated_w } else { planned_delta / 0.1 };
        let safety_timeout = Duration::from_millis(((estimated_time * 1000.0) as u64).saturating_add(250));

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
            let u = pid.next(e, dt).clamp(-v, v);
            for m in self.left_motors.iter_mut() {
                let _ = m.set_voltage(-u);
            }
            for m in self.right_motors.iter_mut() {
                let _ = m.set_voltage(u);
            }
        }

        for m in self
            .left_motors
            .iter_mut()
            .chain(self.right_motors.iter_mut())
        {
            let _ = m.brake(BrakeMode::Hold);
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

            self.turn_to_angle(target_deg, 6.0, false).await;
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
                parallel_offset_sum += perpendicular_distance / t_delta;
                perpendicular_offset_sum += parallel_distance / t_delta;
            }
        }

        let parallel_offset = parallel_offset_sum / ITERATIONS as f64;
        let perpendicular_offset = perpendicular_offset_sum / ITERATIONS as f64;

        println!("parallel offset: {:.3} inches", parallel_offset);
        println!("perpendicular offset: {:.3} inches", perpendicular_offset);
    }

    pub async fn drive_straight(&mut self, target_dist: f64, max_voltage: f64, reverse: bool) {
        self.update_state();
        let start = self.odometry.pose();
        let effective_reverse = if reverse { true } else { target_dist.is_sign_negative() };
        let magnitude = target_dist.abs();
        let signed_dist = if effective_reverse { -magnitude } else { magnitude };
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
            let step = if total - elapsed > dt { dt } else { total - elapsed };
            sleep(step).await;
            elapsed += step;
        }
        for m in self
            .left_motors
            .iter_mut()
            .chain(self.right_motors.iter_mut())
        {
            let _ = m.brake(BrakeMode::Hold);
        }
    }
}
