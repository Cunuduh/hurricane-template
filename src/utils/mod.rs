use core::f64::consts::PI;
use vexide::prelude::Float;
pub mod path;

pub fn normalize_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

pub fn reflected_angle(angle: f64) -> f64 {
    normalize_angle(PI - angle)
}