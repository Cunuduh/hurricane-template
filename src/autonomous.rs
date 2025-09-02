use core::{f64::consts::PI, time::Duration};

use super::chassis::Chassis;
extern crate alloc;
use alloc::vec::Vec;

use vexide::{prelude::*, time::Instant};

use crate::{
    chassis::PoseSettings,
    motion_controller::{ProfilePoint, compute_motion_profile},
    odometry::Pose,
    utils::{normalize_angle, path::interpolate_catmull_rom},
};

struct Segment {
    delta_h: f64,
    t: f64,
    w_d: f64,
}

impl<const L: usize, const R: usize, const I: usize> Chassis<L, R, I> {
    pub async fn drive_ptp(&mut self, points: &[(Pose, PoseSettings)]) {
        for (i, (pose, settings)) in points.iter().enumerate() {
            self.drive_to_point(pose.x, pose.y, *settings).await;
            self.triggers.check_index(i);
        }
    }
    pub async fn drive_ramsete_catmull_rom(
        &mut self,
        path_waypoints: &[(Pose, PoseSettings)],
        steps: usize,
        b: f64,
        zeta: f64,
    ) {
        let interpolated_path = interpolate_catmull_rom(path_waypoints, steps);
        let initialized_profile =
            compute_motion_profile::<L, R, I>(self, interpolated_path, path_waypoints);
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
            self.update_odometry();
            if now.duration_since(start_time) >= safety_timeout {
                println!("ramsete: timeout");
                break;
            }

            let current = self.odometry.pose();
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

            let traveled =
                profile[i].distance + (profile[i + 1].distance - profile[i].distance) * tau;
            self.triggers.check_distance(traveled);

            let x_d = p0.x + (p1.x - p0.x) * tau;
            let y_d = p0.y + (p1.y - p0.y) * tau;

            let mut heading_d = p0_heading + seg_data.delta_h * tau;
            heading_d = normalize_angle(heading_d);

            let v0 = profile[i].velocity;
            let v1 = profile[i + 1].velocity;
            let reversed = if profile[i].is_reversed { -1.0 } else { 1.0 };
            let v_d = (v0 + (v1 - v0) * tau) * reversed;

            let pose = self.odometry.pose();
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

            let wheel_vel_l = v + w * self.config.track_width / 2.0;
            let wheel_vel_r = v - w * self.config.track_width / 2.0;
            let max_linear_vel = (self.motor_free_rpm / self.config.ext_gear_ratio / 60.0)
                * (self.config.wheel_diameter * PI);
            let vl = (wheel_vel_l / max_linear_vel * self.config.max_volts)
                .clamp(-self.config.max_volts, self.config.max_volts);
            let vr = (wheel_vel_r / max_linear_vel * self.config.max_volts)
                .clamp(-self.config.max_volts, self.config.max_volts);
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

    pub async fn drive_to_point(&mut self, target_x: f64, target_y: f64, settings: PoseSettings) {
        self.update_odometry();
        let pose_before_turn = self.odometry.pose();
        let angle_to_target = (target_y - pose_before_turn.y).atan2(target_x - pose_before_turn.x);

        let target_orientation = if settings.is_reversed {
            normalize_angle(angle_to_target + PI)
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
            self.drive_straight(drive_dist, settings.max_voltage).await;
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
        let a = normalize_angle(a_deg.to_radians());
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
            let dh = normalize_angle(h - initial_heading);
            self.triggers.check_angle(dh.to_degrees());
            let e = normalize_angle(a - h);

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
                    && t.elapsed() >= self.config.small_turn_settle_time
                {
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
                let _ = m.set_voltage(u);
            }
            for m in self.right_motors.iter_mut() {
                let _ = m.set_voltage(-u);
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

            self.turn_to_angle(target_deg, 6.0).await;
            sleep(Duration::from_millis(250)).await;

            let imu_start = 0.0;
            let imu_end = self.imu.rotation().unwrap_or_default().to_radians();
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
            let traveled =
                (dx.hypot(dy)) * (dx * initial_heading.cos() + dy * initial_heading.sin()).signum();
            self.triggers.check_distance(traveled.abs());

            let err = target_dist - traveled;
            if err.abs() <= self.config.small_drive_exit_error {
                big_error_timer = None;
                if small_error_timer.is_none() {
                    small_error_timer = Some(Instant::now());
                }
                if let Some(t) = small_error_timer
                    && t.elapsed() >= self.config.small_drive_settle_time
                {
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
                        && t.elapsed() >= self.config.big_drive_settle_time
                    {
                        println!("drive_straight: done (big error)");
                        break;
                    }
                } else {
                    big_error_timer = None;
                }
            }

            let u = drive_pid.next(err, dt);
            let heading_error = normalize_angle(initial_heading - current.heading);
            let tc = heading_pid
                .next(heading_error, dt)
                .clamp(-self.config.max_volts, self.config.max_volts);

            let vl = (u + tc).clamp(-max_voltage, max_voltage);
            let vr = (u - tc).clamp(-max_voltage, max_voltage);
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
}
