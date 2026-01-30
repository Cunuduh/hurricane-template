use core::f64::consts::PI;

use serde::{Deserialize, Serialize};
use vexide::prelude::{Float, Position};
use crate::utils::normalize_angle;

fn default_nan() -> f64 {
    f64::NAN
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

#[derive(Default, Copy, Clone, Debug, Serialize, Deserialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    #[serde(default = "default_nan")]
    pub heading: f64,
}
impl Pose {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y, heading: 0.0 }
    }
    pub fn with_heading(x: f64, y: f64, heading: f64) -> Self {
        Self { x, y, heading }
    }
}
impl Pos2Like for Pose {
    fn x(&self) -> f64 {
        self.x
    }
    fn y(&self) -> f64 {
        self.y
    }
    fn distance(&self, other: &Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }
    fn lerp(&self, other: &Self, t: f64) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            heading: self.heading + (other.heading - self.heading) * t,
        }
    }
}
#[derive(Debug, Clone, Copy)]
pub struct TrackingWheelConfig {
    pub parallel_offset: f64,
    pub perpendicular_offset: f64,
    pub wheel_diameter: f64,
}

pub struct Odometry {
    pose: Pose,
    prev_parallel_revs: f64,
    prev_perpendicular_revs: f64,
    tracking_wheels: TrackingWheelConfig,
}

impl Odometry {
    pub fn new(initial_pose: Pose, tracking_wheels: TrackingWheelConfig) -> Self {
        Self {
            pose: initial_pose,
            prev_parallel_revs: 0.0,
            prev_perpendicular_revs: 0.0,
            tracking_wheels,
        }
    }

    pub fn update(
        &mut self,
        parallel_revs: Position,
        perpendicular_revs: Position,
        imu_heading_rad: f64,
    ) {
        let par = parallel_revs;
        let perp = perpendicular_revs;
        let p = par.as_revolutions();
        let q = perp.as_revolutions();

        let dp = (p - self.prev_parallel_revs) * self.tracking_wheels.wheel_diameter * PI;
        let ds = (q - self.prev_perpendicular_revs) * self.tracking_wheels.wheel_diameter * PI;

        self.prev_parallel_revs = p;
        self.prev_perpendicular_revs = q;

        let theta = normalize_angle(imu_heading_rad);
        let dtheta = normalize_angle(theta - self.pose.heading);
        let dx_robot = dp - dtheta * self.tracking_wheels.parallel_offset;
        let dy_robot = ds - dtheta * self.tracking_wheels.perpendicular_offset;

        let avg_heading = self.pose.heading + dtheta / 2.0;

        self.pose.x += dx_robot * avg_heading.cos() - dy_robot * avg_heading.sin();
        self.pose.y += dx_robot * avg_heading.sin() + dy_robot * avg_heading.cos();
        self.pose.heading = theta;
    }

    pub fn pose(&self) -> Pose {
        self.pose
    }
    pub fn reset(&mut self, pose: Pose, parallel_revs: f64, perpendicular_revs: f64) {
        self.pose = pose;
        self.prev_parallel_revs = parallel_revs;
        self.prev_perpendicular_revs = perpendicular_revs;
    }
}
