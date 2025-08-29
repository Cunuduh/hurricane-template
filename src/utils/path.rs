use alloc::vec;
use alloc::vec::Vec;
use vexide::prelude::Float;
use crate::{chassis::PoseSettings, odometry::{Pose, Pos2Like}};

fn catmull_rom<T: Pos2Like + Clone>(
    p0: &T,
    p1: &T,
    p2: &T,
    p3: &T,
    t: f64,
    alpha: f64,
) -> (T, f64, f64) {
    let epsilon = 1e-7;
    let d01 = (p1.distance(p0) + epsilon).powf(alpha).max(epsilon);
    let d12 = (p2.distance(p1) + epsilon).powf(alpha).max(epsilon);
    let d23 = (p3.distance(p2) + epsilon).powf(alpha).max(epsilon);

    let t0 = 0.0;
    let t1 = t0 + d01;
    let t2 = t1 + d12;
    let t3 = t2 + d23;

    let t_ = t1 + (t2 - t1) * t;
    let dt_scale = t2 - t1; // derivative scaling factor

    let dt01 = (t1 - t0).max(epsilon);
    let dt12 = (t2 - t1).max(epsilon);
    let dt23 = (t3 - t2).max(epsilon);
    let dt02 = (t2 - t0).max(epsilon);
    let dt13 = (t3 - t1).max(epsilon);

    // first derivatives of intermediate points
    let da1_dt = ((p1.x() - p0.x()) / dt01, (p1.y() - p0.y()) / dt01);
    let da2_dt = ((p2.x() - p1.x()) / dt12, (p2.y() - p1.y()) / dt12);
    let da3_dt = ((p3.x() - p2.x()) / dt23, (p3.y() - p2.y()) / dt23);

    let a1 = p0.lerp(p1, (t_ - t0) / dt01);
    let a2 = p1.lerp(p2, (t_ - t1) / dt12);
    let a3 = p2.lerp(p3, (t_ - t2) / dt23);

    // derivatives of b1, b2
    let db1_dt = (
        da1_dt.0 + (a2.x() - a1.x()) / dt02 + ((da2_dt.0 - da1_dt.0) * (t_ - t0)) / dt02,
        da1_dt.1 + (a2.y() - a1.y()) / dt02 + ((da2_dt.1 - da1_dt.1) * (t_ - t0)) / dt02
    );
    let db2_dt = (
        da2_dt.0 + (a3.x() - a2.x()) / dt13 + ((da3_dt.0 - da2_dt.0) * (t_ - t1)) / dt13,
        da2_dt.1 + (a3.y() - a2.y()) / dt13 + ((da3_dt.1 - da2_dt.1) * (t_ - t1)) / dt13
    );

    let b1 = a1.lerp(&a2, (t_ - t0) / dt02);
    let b2 = a2.lerp(&a3, (t_ - t1) / dt13);

    let pt = b1.lerp(&b2, (t_ - t1) / dt12);
    // final derivative
    let dp_dt = (
        db1_dt.0 + (b2.x() - b1.x()) / dt12 + ((db2_dt.0 - db1_dt.0) * (t_ - t1)) / dt12,
        db1_dt.1 + (b2.y() - b1.y()) / dt12 + ((db2_dt.1 - db1_dt.1) * (t_ - t1)) / dt12
    );

    (pt, dp_dt.0 / dt_scale, dp_dt.1 / dt_scale)
}
pub fn interpolate_catmull_rom(
    points: &[(Pose, PoseSettings)], 
    steps: usize,
) -> Vec<(Pose, PoseSettings)> {
    const ALPHA: f64 = 0.5;
    let n = points.len();
    if n == 0 { return Vec::new(); }
    if n == 1 { return vec![(points[0].0, points[0].1)]; }
    
    let mut result = Vec::with_capacity((n - 1) * steps + 1);
    
    for i in 0..(n - 1) {
        let p0 = if i == 0 { points[0].0 } else { points[i - 1].0 };
        let p1 = points[i].0;
        let p2 = points[i + 1].0;
        let p3 = if i + 2 >= n { points[n - 1].0 } else { points[i + 2].0 };
        
        for step in 0..steps {
            let t = step as f64 / steps as f64;
            let (mut pt, dx, dy) = catmull_rom(&p0, &p1, &p2, &p3, t, ALPHA);
            // heading is tangent of the spline; reverse handling is applied later
            pt.heading = dy.atan2(dx);
            result.push((pt, points[i].1));
        }
    }
    
    result.push(points[n - 1]);
    result
}
