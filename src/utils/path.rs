extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;
use vexide::prelude::Float;
use crate::{control::{Pos2Like, PoseSettings}, odometry::Pose};

fn catmull_rom<T: Pos2Like + Clone>(
    p0: &T,
    p1: &T,
    p2: &T,
    p3: &T,
    t: f64,
    alpha: f64,
) -> T {
    let epsilon = 1e-7;

    let d01 = (p1.distance(p0) + epsilon).powf(alpha).max(epsilon);
    let d12 = (p2.distance(p1) + epsilon).powf(alpha).max(epsilon);
    let d23 = (p3.distance(p2) + epsilon).powf(alpha).max(epsilon);

    let t0 = 0.0;
    let t1 = t0 + d01;
    let t2 = t1 + d12;
    let t3 = t2 + d23;

    let t_ = t1 + (t2 - t1) * t;

    let dt01 = (t1 - t0).max(epsilon);
    let dt12 = (t2 - t1).max(epsilon);
    let dt23 = (t3 - t2).max(epsilon);
    let dt02 = (t2 - t0).max(epsilon);
    let dt13 = (t3 - t1).max(epsilon);

    let a1 = p0.lerp(p1, (t_ - t0) / dt01);
    let a2 = p1.lerp(p2, (t_ - t1) / dt12);
    let a3 = p2.lerp(p3, (t_ - t2) / dt23);

    let b1 = a1.lerp(&a2, (t_ - t0) / dt02);
    let b2 = a2.lerp(&a3, (t_ - t1) / dt13);

    b1.lerp(&b2, (t_ - t1) / dt12)
}

pub fn interpolate_catmull_rom(
    points: &[(Pose, PoseSettings)],
    steps: usize,
) -> Vec<(Pose, PoseSettings)> {
    const ALPHA: f64 = 0.5;
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(points[0].0, points[0].1)];
    }
    let mut result = Vec::with_capacity((n - 1) * steps + 1);
    for i in 0..(n - 1) {
        let p0 = if i == 0 { points[0].0 } else { points[i - 1].0 };
        let p1 = points[i].0;
        let p2 = points[i + 1].0;
        let p3 = if i + 2 >= n { points[n - 1].0 } else { points[i + 2].0 };
        for step in 0..steps {
            let t = step as f64 / steps as f64;
            let pt = catmull_rom(&p0, &p1, &p2, &p3, t, ALPHA);
            result.push((pt, points[i].1));
        }
    }
    result.push(points[n - 1]);
    result
}

pub fn interpolate_linear(
    points: &[(Pose, PoseSettings)],
    steps: usize,
) -> Vec<(Pose, PoseSettings)> {
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(points[0].0, points[0].1)];
    }
    
    let mut result = Vec::with_capacity((n - 1) * steps + 1);
    for i in 0..(n - 1) {
        let p1 = points[i].0;
        let p2 = points[i + 1].0;
        for step in 0..steps {
            let t = step as f64 / steps as f64;
            let pt = p1.lerp(&p2, t);
            result.push((pt, points[i].1));
        }
    }
    result.push(points[n - 1]);
    result
}
