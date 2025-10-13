use core::f64::consts::PI;
extern crate alloc;
use alloc::vec::Vec;

use vexide::prelude::*;

use crate::{
    chassis::{Chassis, PoseSettings},
    odometry::Pose,
    utils::normalize_angle,
};

pub struct Profile {
    pub profile_points: Vec<ProfilePoint>,
    pub times: Vec<f64>,
    pub total_time: f64,
}

#[derive(Copy, Clone, Debug)]
pub struct ProfilePoint {
    pub point: Pose,
    pub distance: f64,
    pub curvature: f64,
    pub velocity: f64,
    pub is_reversed: bool,
    pub max_voltage: f64,
}

pub fn compute_motion_profile<const L: usize, const R: usize, const I: usize>(
    chassis: &Chassis<L, R, I>,
    interpolated_path: Vec<(Pose, PoseSettings)>,
    raw_waypoints: &[(Pose, PoseSettings)],
) -> Profile {
    let path_points = interpolated_path.iter().map(|p| p.0).collect::<Vec<_>>();
    let m = path_points.len();
    let mut profile_points = Vec::with_capacity(m);

    if m == 0 {
        return Profile {
            profile_points,
            times: Vec::new(),
            total_time: 0.0,
        };
    }

    let mut cum_d = 0.0;
    profile_points.push(ProfilePoint {
        point: path_points[0],
        distance: 0.0,
        curvature: 0.0,
        velocity: 0.0,
        is_reversed: raw_waypoints[0].1.is_reversed,
        max_voltage: raw_waypoints[0].1.max_voltage,
    });

    for i in 1..m {
        let d = (path_points[i].x - path_points[i - 1].x)
            .hypot(path_points[i].y - path_points[i - 1].y);
        cum_d += d;
        let raw_idx_for_props =
            ((i * (raw_waypoints.len() - 1)) / (m - 1)).min(raw_waypoints.len() - 1);
        profile_points.push(ProfilePoint {
            point: path_points[i],
            distance: cum_d,
            curvature: 0.0,
            velocity: 0.0,
            is_reversed: raw_waypoints[raw_idx_for_props].1.is_reversed,
            max_voltage: raw_waypoints[raw_idx_for_props].1.max_voltage,
        });
    }

    profile_points
        .iter_mut()
        .filter(|p| p.is_reversed)
        .for_each(|p| {
            p.point.heading += PI;
            p.point.heading = normalize_angle(p.point.heading);
        });

    // compute curvature as |Δθ|/Δs using centered difference
    for i in 1..(m - 1) {
        let h_prev = profile_points[i - 1].point.heading;
        let h_next = profile_points[i + 1].point.heading;
        let d_prev = profile_points[i].distance - profile_points[i - 1].distance;
        let d_next = profile_points[i + 1].distance - profile_points[i].distance;
        let ds = d_prev + d_next;
        if d_prev > 1e-9 && d_next > 1e-9 && ds > 1e-9 {
            let dtheta = normalize_angle(h_next - h_prev).abs();
            profile_points[i].curvature = dtheta / ds;
        }
    }

    let w_circ = chassis.config.wheel_diameter * PI;
    let rpm = chassis.motor_free_rpm / chassis.config.ext_gear_ratio;
    let max_v = (rpm / 60.0) * w_circ;
    let max_a = max_v / chassis.config.accel_t;
    let k = 2.0;
    // first compute the "speed limit" of every point given the curvature
    for p in profile_points.iter_mut() {
        let point_max_velocity = (p.max_voltage / chassis.config.max_volts) * max_v;
        let mut v = point_max_velocity;
        if p.curvature > 1e-6 {
            let v_k_curv_limit = k / p.curvature;
            v = v.min(v_k_curv_limit);
        }
        p.velocity = v;
    }
    // the forward pass to make sure acceleration is realistic given distance between points and our computed speed limit
    for i in 0..(m - 1) {
        let d = profile_points[i + 1].distance - profile_points[i].distance;
        if d < 1e-9 {
            profile_points[i + 1].velocity = profile_points[i + 1]
                .velocity
                .min(profile_points[i].velocity);
            continue;
        }
        let vf = profile_points[i].velocity.powi(2) + 2.0 * max_a * d;
        if vf >= 0.0 {
            profile_points[i + 1].velocity = profile_points[i + 1].velocity.min(vf.sqrt());
        } else {
            profile_points[i + 1].velocity = profile_points[i + 1].velocity.min(0.0);
        }
    }
    profile_points[0].velocity = 0.0;
    profile_points[m - 1].velocity = 0.0;
    // same logic as forward pass but in reverse for deceleration
    for i in (0..(m - 1)).rev() {
        let d = profile_points[i + 1].distance - profile_points[i].distance;
        if d < 1e-9 {
            profile_points[i].velocity = profile_points[i]
                .velocity
                .min(profile_points[i + 1].velocity);
            continue;
        }
        let vf_old = profile_points[i + 1].velocity.powi(2) + 2.0 * max_a * d;
        if vf_old >= 0.0 {
            profile_points[i].velocity = profile_points[i].velocity.min(vf_old.sqrt());
        } else {
            profile_points[i].velocity = profile_points[i].velocity.min(0.0);
        }
    }
    // finally, compute the e.t.a. for each point
    let mut times = Vec::with_capacity(m);
    times.push(0.0);
    for i in 1..m {
        let d = profile_points[i].distance - profile_points[i - 1].distance;
        let v_prev = profile_points[i - 1].velocity.max(1e-6);
        let v_curr = profile_points[i].velocity.max(1e-6);
        let v_avg = 0.5 * (v_prev + v_curr);
        times.push(times[i - 1] + d / v_avg);
    }
    let total_time = *times.last().unwrap_or(&0.0);

    Profile {
        profile_points,
        times,
        total_time,
    }
}
