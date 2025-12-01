use core::f64::consts::PI;
extern crate alloc;
use alloc::vec::Vec;

use vexide::prelude::*;

use crate::{
    chassis::{Chassis, PoseSettings},
    odometry::Pose,
    utils::normalize_angle,
};

pub struct MotionProfile2D {
    pub profile_points: Vec<MotionProfile2DPoint>,
    pub times: Vec<f64>,
    pub total_time: f64,
}

#[derive(Copy, Clone, Debug)]
pub struct MotionProfile2DPoint {
    pub point: Pose,
    pub distance: f64,
    pub curvature: f64,
    pub velocity: f64,
    pub acceleration: f64,
    pub angular_velocity: f64,
    pub angular_acceleration: f64,
    pub is_reversed: bool,
    pub max_voltage: f64,
    pub waypoint_index: usize,
}

pub fn compute_motion_profile<const L: usize, const R: usize, const I: usize>(
    chassis: &Chassis<L, R, I>,
    interpolated_path: Vec<(Pose, PoseSettings, usize)>,
    _raw_waypoints: &[(Pose, PoseSettings)],
) -> MotionProfile2D {
    let path_points = interpolated_path.iter().map(|p| p.0).collect::<Vec<_>>();
    let m = path_points.len();
    let mut profile_points = Vec::with_capacity(m);

    if m == 0 {
        return MotionProfile2D {
            profile_points,
            times: Vec::new(),
            total_time: 0.0,
        };
    }

    let mut cum_d = 0.0;
    let mut p0 = path_points[0];
    if interpolated_path[0].1.is_reversed {
        p0.heading = normalize_angle(p0.heading + PI);
    }
    profile_points.push(MotionProfile2DPoint {
        point: p0,
        distance: 0.0,
        curvature: 0.0,
        velocity: 0.0,
        acceleration: 0.0,
        angular_velocity: 0.0,
        angular_acceleration: 0.0,
        is_reversed: interpolated_path[0].1.is_reversed,
        max_voltage: interpolated_path[0].1.max_voltage,
        waypoint_index: interpolated_path[0].2,
    });

    for i in 1..m {
        let d = (path_points[i].x - path_points[i - 1].x)
            .hypot(path_points[i].y - path_points[i - 1].y);
        cum_d += d;
        
        let is_reversed = interpolated_path[i].1.is_reversed;
        let max_voltage = interpolated_path[i].1.max_voltage;
        let waypoint_index = interpolated_path[i].2;

        let mut p = path_points[i];
        if is_reversed {
            p.heading = normalize_angle(p.heading + PI);
        }

        profile_points.push(MotionProfile2DPoint {
            point: p,
            distance: cum_d,
            curvature: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
            angular_velocity: 0.0,
            angular_acceleration: 0.0,
            is_reversed,
            max_voltage,
            waypoint_index,
        });
    }

    // compute curvature as |Δθ|/Δs using centred difference
    for i in 1..(m - 1) {
        let h_prev = profile_points[i - 1].point.heading;
        let h_next = profile_points[i + 1].point.heading;
        let d_prev = profile_points[i].distance - profile_points[i - 1].distance;
        let d_next = profile_points[i + 1].distance - profile_points[i].distance;
        let ds = d_prev + d_next;
        if d_prev > 1e-9 && d_next > 1e-9 && ds > 1e-9 {
            let dtheta = normalize_angle(h_next - h_prev);
            profile_points[i].curvature = dtheta / ds;
        } else {
            profile_points[i].curvature = profile_points[i - 1].curvature;
        }
    }
    if m > 1 {
        profile_points[0].curvature = profile_points[1].curvature;
        profile_points[m - 1].curvature = profile_points[m - 2].curvature;
    }
    let w_circ = chassis.config.wheel_diameter * PI;
    let rpm = chassis.motor_free_rpm / chassis.config.ext_gear_ratio;
    let v_max = (rpm / 60.0) * w_circ;
    let a_max = v_max / chassis.config.t_accel_ramsete;
    let d_max = v_max / chassis.config.t_decel_ramsete;
    let a_cmax = 0.67 * 39.37;
    // first compute the "speed limit" of every point given the curvature
    for p in profile_points.iter_mut() {
        let point_max_velocity = (p.max_voltage / chassis.config.max_volts) * v_max;
        let mut v = point_max_velocity;
        let curvature = p.curvature.abs();
        if curvature > 1e-6 {
            // constrain velocity based on centripetal acceleration limit of robot on the foam tiles so it doesn't tip over while turning
            let v_c = (a_cmax / curvature).sqrt();
            // constrain velocity based on max speed the outer wheel can go given track width
            let v_t = v_max / (1.0 + (curvature * chassis.config.track_width / 2.0));
            v = v.min(v_c).min(v_t);
        }
        p.velocity = v;
    }
    profile_points[0].velocity = 0.0;
    profile_points[m - 1].velocity = 0.0;
    
    // forward pass: enforce acceleration limits
    for i in 0..(m - 1) {
        let d = profile_points[i + 1].distance - profile_points[i].distance;
        if d < 1e-9 {
            profile_points[i + 1].velocity = profile_points[i + 1]
                .velocity
                .min(profile_points[i].velocity);
            profile_points[i + 1].acceleration = 0.0;
            continue;
        }
        
        let v_i = profile_points[i].velocity;
        let v_limit = profile_points[i + 1].velocity;
        let v_f = (v_i.powi(2) + 2.0 * a_max * d).sqrt().min(v_limit);
        
        profile_points[i + 1].velocity = v_f;
        profile_points[i + 1].acceleration = (v_f.powi(2) - v_i.powi(2)) / (2.0 * d);
    }
    
    // backward pass: enforce deceleration limits
    for i in (0..(m - 1)).rev() {
        let d = profile_points[i + 1].distance - profile_points[i].distance;
        if d < 1e-9 {
            profile_points[i].velocity = profile_points[i]
                .velocity
                .min(profile_points[i + 1].velocity);
            continue;
        }
        
        let v_f = profile_points[i + 1].velocity;
        let v_limit = profile_points[i].velocity;
        let v_i = (v_f.powi(2) + 2.0 * d_max * d).sqrt().min(v_limit);
        
        profile_points[i].velocity = v_i;
        profile_points[i].acceleration = (v_f.powi(2) - v_i.powi(2)) / (2.0 * d);
    }
    
    // apply product rule
    // α = dω/dt = (dv/dt)·κ + v·(dκ/dt)
    // α = a·κ + v·(dκ/ds)·(ds/dt)
    // α = a·κ + v²·(dκ/ds)
    for i in 1..(m - 1) {
        let k_prev = profile_points[i - 1].curvature;
        let k_next = profile_points[i + 1].curvature;
        let d_prev = profile_points[i].distance - profile_points[i - 1].distance;
        let d_next = profile_points[i + 1].distance - profile_points[i].distance;
        let ds = d_prev + d_next;
        
        let dk_ds = if ds > 1e-9 {
            (k_next - k_prev) / ds
        } else {
            0.0
        };
        
        let v = profile_points[i].velocity;
        let a = profile_points[i].acceleration;
        let k = profile_points[i].curvature;
        
        profile_points[i].angular_velocity = v * k;
        profile_points[i].angular_acceleration = a * k + v * v * dk_ds;
    }
    
    profile_points[0].angular_velocity = profile_points[0].velocity * profile_points[0].curvature;
    profile_points[0].angular_acceleration = 
        profile_points[0].acceleration * profile_points[0].curvature;
    
    profile_points[m - 1].angular_velocity = profile_points[m - 1].velocity * profile_points[m - 1].curvature;
    profile_points[m - 1].angular_acceleration = 
        profile_points[m - 1].acceleration * profile_points[m - 1].curvature;

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


    MotionProfile2D {
        profile_points,
        times,
        total_time,
    }
}

pub struct TrapezoidalProfile1D {
    pub total_time: f64,
    pub t1: f64,
    pub t2: f64,
    pub distance: f64,
    pub max_v: f64,
    pub max_a: f64,
    pub max_d: f64,
}

impl TrapezoidalProfile1D {
    pub fn new(distance: f64, max_v: f64, max_a: f64, max_d: f64) -> Self {
        let mut v_peak = max_v;
        let d_accel = 0.5 * v_peak * v_peak / max_a;
        let d_decel = 0.5 * v_peak * v_peak / max_d;

        let (t1, t2, total_time);

        if d_accel + d_decel > distance {
            v_peak = (2.0 * distance / (1.0 / max_a + 1.0 / max_d)).sqrt();
            let t_accel = v_peak / max_a;
            let t_decel = v_peak / max_d;
            t1 = t_accel;
            t2 = t_accel;
            total_time = t_accel + t_decel;
        } else {
            let t_accel = v_peak / max_a;
            let d_cruise = distance - d_accel - d_decel;
            let t_cruise = d_cruise / v_peak;
            let t_decel = v_peak / max_d;
            t1 = t_accel;
            t2 = t_accel + t_cruise;
            total_time = t2 + t_decel;
        }

        Self {
            total_time,
            t1,
            t2,
            distance,
            max_v: v_peak,
            max_a,
            max_d,
        }
    }

    pub fn sample(&self, t: f64) -> (f64, f64, f64) {
        if t < 0.0 {
            return (0.0, 0.0, 0.0);
        }
        if t >= self.total_time {
            return (self.distance, 0.0, 0.0);
        }

        if t < self.t1 {
            // accelerating
            let a = self.max_a;
            let v = a * t;
            let p = 0.5 * a * t * t;
            (p, v, a)
        } else if t < self.t2 {
            // cruising
            let dt = t - self.t1;
            let p_accel = 0.5 * self.max_a * self.t1 * self.t1;
            let v = self.max_v;
            let p = p_accel + v * dt;
            (p, v, 0.0)
        } else {
            // decelerating
            let dt = t - self.t2;
            let a = -self.max_d;
            let v = self.max_v + a * dt;
            let p_accel = 0.5 * self.max_a * self.t1 * self.t1;
            let p_cruise = p_accel + self.max_v * (self.t2 - self.t1);
            let p = p_cruise + self.max_v * dt + 0.5 * a * dt * dt;
            (p, v, a)
        }
    }
}
