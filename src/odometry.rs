use core::f64::consts::PI;
use vexide::prelude::Float;
use vexide::prelude::Position;
use crate::control::Pos2Like;

#[derive(Default, Copy, Clone, Debug)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
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
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
    fn lerp (&self, other: &Self, t: f64) -> Self {
        Self {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            heading: self.heading + (other.heading - self.heading) * t,
        }
    }
}
pub struct Odometry {
    pose: Pose,
    prev_lrevs: f64,
    prev_rrevs: f64,
    wheel_circumference: f64,
    track_width: f64,
    ext_gear_ratio: f64,
}

impl Odometry {
    pub fn new(
        initial_pose: Pose,
        wheel_diameter: f64,
        track_width: f64,
        ext_gear_ratio: f64,
    ) -> Self {
        Self {
            pose: initial_pose,
            prev_lrevs: 0.0,
            prev_rrevs: 0.0,
            wheel_circumference: wheel_diameter * PI,
            track_width,
            ext_gear_ratio,
        }
    }

    pub fn update(
        &mut self,
        left_revs: Position,
        right_revs: Position,
        imu_heading_rad: f64,
    ) {
        let lrevs = left_revs.as_revolutions();
        let rrevs = right_revs.as_revolutions();

        let delta_lrevs = lrevs - self.prev_lrevs;
        let delta_rrevs = rrevs - self.prev_rrevs;
        self.prev_lrevs = lrevs;
        self.prev_rrevs = rrevs;

        let delta_wheel_lrevs = delta_lrevs / self.ext_gear_ratio;
        let delta_wheel_rrevs = delta_rrevs / self.ext_gear_ratio;
        let delta_ldist = delta_wheel_lrevs * self.wheel_circumference;
        let delta_rdist = delta_wheel_rrevs * self.wheel_circumference;
        // average distance traveled
        let avg_fwd_dist = (delta_ldist + delta_rdist) / 2.0;

        // encoder-based delta heading
        let delta_heading_enc = (delta_rdist - delta_ldist) / self.track_width;
        let enc_heading = self.pose.heading + delta_heading_enc;
        // also use imu heading
        let imu_heading = self.normalize_angle(imu_heading_rad);
        // fuse encoder and imu heading with complementary filter
        // offer all weight to imu (less drift, no problem of motor slipping or scrub) until i get them odom pods
        const ALPHA: f64 = 1.00;
        let fused = self.normalize_angle((1.0 - ALPHA) * enc_heading + ALPHA * imu_heading);

        // update pose using fused heading
        self.pose.x += avg_fwd_dist * fused.cos();
        self.pose.y += avg_fwd_dist * fused.sin();
        self.pose.heading = fused;
    }

    pub fn pose(&self) -> Pose {
        self.pose
    }
    pub fn reset(&mut self, pose: Pose) {
        self.pose = pose;
        self.prev_lrevs = 0.0;
        self.prev_rrevs = 0.0;
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
}
