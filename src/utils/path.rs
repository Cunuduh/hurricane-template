use vexide::prelude::Float;
use crate::{control::{Pos2Like, PoseSettings}, odometry::Pose};

pub const fn calculate_interpolated_path_size<const N: usize, const STEPS: usize>() -> usize {
    if N == 0 {
        0
    } else if N == 1 {
        1
    } else {
        (N - 1) * STEPS + 1
    }
}
// https://en.wikipedia.org/wiki/Centripetal_Catmull-Rom_spline
// we prefer centripetal catmull-rom for paths because it's c3 continuous
// meaning it has continuous third derivatives (acceleration), important for smooth acceleration
pub fn catmull_rom<T: Pos2Like + Clone>(
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

pub fn interpolate_catmull_rom<const N: usize, const STEPS: usize>(
    points: &[(Pose, PoseSettings); N],
) -> [(Pose, PoseSettings); calculate_interpolated_path_size::<N, STEPS>()] {
    let mut result = [(Pose::default(), PoseSettings::default()); calculate_interpolated_path_size::<N, STEPS>()];
    // 0.0 = uniform catmull-rom
    // 0.5 = centripetal catmull-rom
    // 1.0 = chordal catmull-rom
    // we opt for 0.5 because it works better around sharp bends, guaranteed to not cross over itself
    const ALPHA: f64 = 0.5;
    
    if N == 0 {
        return result;
    }
    if N == 1 {
        if calculate_interpolated_path_size::<N, STEPS>() > 0 {
             result[0] = (points[0].0, points[0].1);
        }
        return result;
    }

    let mut idx = 0;
    for i in 0..(N - 1) {
        let p0 = if i == 0 { points[0].0 } else { points[i - 1].0 };
        let p1 = points[i].0;
        let p2 = points[i + 1].0;
        let p3 = if i + 2 >= N { points[N - 1].0 } else { points[i + 2].0 };

        for step in 0..STEPS {
            let t = step as f64 / STEPS as f64;
            if idx < calculate_interpolated_path_size::<N, STEPS>() {
                result[idx].0 = catmull_rom(&p0, &p1, &p2, &p3, t, ALPHA);
                result[idx].1.is_reversed = points[i].1.is_reversed;
                result[idx].1.max_voltage = points[i].1.max_voltage;
                idx += 1;
            }
        }
    }
    if idx < calculate_interpolated_path_size::<N, STEPS>() {
        result[idx] = points[N - 1];
    } else if N > 0 && calculate_interpolated_path_size::<N, STEPS>() > 0 && idx == 0 && STEPS == 0 {
        result[0] = points[N-1];
    }

    result
}
