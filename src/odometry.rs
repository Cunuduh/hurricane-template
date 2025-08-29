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
        (self.x - other.x).hypot(self.y - other.y)
    }
    fn lerp (&self, other: &Self, t: f64) -> Self {
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
    prev_lrevs: f64,
    prev_rrevs: f64,
    prev_parallel_revs: Option<f64>,
    prev_perpendicular_revs: Option<f64>,
    wheel_circumference: f64,
    track_width: f64,
    ext_gear_ratio: f64,
    tracking_wheels: Option<TrackingWheelConfig>,
}

impl Odometry {
    pub fn new(
        initial_pose: Pose,
        wheel_diameter: f64,
        track_width: f64,
        ext_gear_ratio: f64,
        tracking_wheels: Option<TrackingWheelConfig>,
    ) -> Self {
        Self {
            pose: initial_pose,
            prev_lrevs: 0.0,
            prev_rrevs: 0.0,
            prev_parallel_revs: None,
            prev_perpendicular_revs: None,
            wheel_circumference: wheel_diameter * PI,
            track_width,
            ext_gear_ratio,
            tracking_wheels,
        }
    }

    pub fn update(
        &mut self,
        parallel_revs: Position,
        perpendicular_revs: Position,
        imu_heading_rad: f64,
    ) {
        if let Some(tw) = &self.tracking_wheels {
            let par = parallel_revs;
            let perp = perpendicular_revs;
            let p = par.as_revolutions();
            let q = perp.as_revolutions();

            let dp = (p - self.prev_parallel_revs.unwrap_or(p)) * tw.wheel_diameter * PI;
            let ds = (q - self.prev_perpendicular_revs.unwrap_or(q)) * tw.wheel_diameter * PI;

            self.prev_parallel_revs = Some(p);
            self.prev_perpendicular_revs = Some(q);

            let theta = self.normalize_angle(imu_heading_rad);
            let dtheta = self.normalize_angle(theta - self.pose.heading);
            // compensate rotation-induced wheel travel
            let dx_robot = dp - dtheta * tw.perpendicular_offset;
            let dy_robot = ds + dtheta * tw.parallel_offset;

            self.pose.x += dx_robot * self.pose.heading.cos()
                - dy_robot * self.pose.heading.sin();
            self.pose.y += dx_robot * self.pose.heading.sin()
                + dy_robot * self.pose.heading.cos();
            self.pose.heading = theta;
        }
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
