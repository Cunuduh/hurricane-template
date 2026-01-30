use alloc::{vec, vec::Vec};

use vexide::prelude::Float;

use crate::{
    chassis::PoseSettings,
    odometry::{Pos2Like, Pose},
};

pub fn interpolate_curve(
    points: &[(Pose, PoseSettings)],
    step_size: f64,
) -> Vec<(Pose, PoseSettings, usize)> {
    // mixed catmull-rom/hermite: use centripetal catmull-rom tangents by default, but if a waypoint
    // has a finite heading, override the tangent direction there with that heading (magnitude preserved)
    const ALPHA: f64 = 0.5;
    let n = points.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(points[0].0, points[0].1, 0)];
    }

    // parameterization t_i using centripetal distances
    let mut t: Vec<f64> = Vec::with_capacity(n);
    t.push(0.0);
    for i in 1..n {
        let d = points[i].0.distance(&points[i - 1].0).max(1e-7);
        let dt = d.powf(ALPHA);
        t.push(t[i - 1] + dt);
    }

    // compute default catmull-rom tangent vectors m_i (derivative wrt global parameter)
    let mut m: Vec<(f64, f64)> = Vec::with_capacity(n);
    for i in 0..n {
        let (mi_x, mi_y) = if i == 0 {
            let dt = (t[1] - t[0]).max(1e-7);
            let dx = (points[1].0.x() - points[0].0.x()) / dt;
            let dy = (points[1].0.y() - points[0].0.y()) / dt;
            (dx, dy)
        } else if i == n - 1 {
            let dt = (t[n - 1] - t[n - 2]).max(1e-7);
            let dx = (points[n - 1].0.x() - points[n - 2].0.x()) / dt;
            let dy = (points[n - 1].0.y() - points[n - 2].0.y()) / dt;
            (dx, dy)
        } else {
            let dt = (t[i + 1] - t[i - 1]).max(1e-7);
            let dx = (points[i + 1].0.x() - points[i - 1].0.x()) / dt;
            let dy = (points[i + 1].0.y() - points[i - 1].0.y()) / dt;
            (dx, dy)
        };

        // if heading specified at this waypoint, override tangent direction but keep magnitude
        let h = points[i].0.heading;
        let (mi_x, mi_y) = if h.is_finite() {
            let mag = (mi_x * mi_x + mi_y * mi_y).sqrt();
            let mag = if mag.is_finite() && mag > 1e-7 { mag } else {
                // fallback magnitude: chord length to nearest neighbor in parameter units
                let neighbor = if i + 1 < n { i + 1 } else { i - 1 };
                let dt = (t[neighbor] - t[i]).abs().max(1e-7);
                (points[neighbor].0.x() - points[i].0.x()).hypot(points[neighbor].0.y() - points[i].0.y()) / dt
            };
            (h.cos() * mag, h.sin() * mag)
        } else {
            (mi_x, mi_y)
        };
        m.push((mi_x, mi_y));
    }

    // sample each segment using cubic hermite with s in [0,1]
    let step_size = step_size.max(0.1);
    let mut result = Vec::new();

    for i in 0..(n - 1) {
        let p0 = points[i].0;
        let p1 = points[i + 1].0;
        let dt = (t[i + 1] - t[i]).max(1e-7);
        // scale tangents to segment parameter s
        let m0 = (m[i].0 * dt, m[i].1 * dt);
        let m1 = (m[i + 1].0 * dt, m[i + 1].1 * dt);

        // estimate segment length using chord length
        let segment_dist = p0.distance(&p1);
        let steps = (segment_dist / step_size).ceil().max(1.0) as usize;

        for step in 0..steps {
            let s = step as f64 / steps as f64;
            let s2 = s * s;
            let s3 = s2 * s;

            // hermite basis
            let h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
            let h10 = s3 - 2.0 * s2 + s;
            let h01 = -2.0 * s3 + 3.0 * s2;
            let h11 = s3 - s2;

            let x = h00 * p0.x + h10 * m0.0 + h01 * p1.x + h11 * m1.0;
            let y = h00 * p0.y + h10 * m0.1 + h01 * p1.y + h11 * m1.1;

            // derivative wrt s
            let dh00 = 6.0 * s2 - 6.0 * s;
            let dh10 = 3.0 * s2 - 4.0 * s + 1.0;
            let dh01 = -6.0 * s2 + 6.0 * s;
            let dh11 = 3.0 * s2 - 2.0 * s;

            let dx = dh00 * p0.x + dh10 * m0.0 + dh01 * p1.x + dh11 * m1.0;
            let dy = dh00 * p0.y + dh10 * m0.1 + dh01 * p1.y + dh11 * m1.1;
            let heading = dy.atan2(dx);

            let pose = Pose { x, y, heading };
            
            // interpolate voltage
            let v0 = points[i].1.max_voltage;
            let v1 = points[i + 1].1.max_voltage;
            let v_interp = v0 + (v1 - v0) * s;
            let mut settings = points[i].1;
            settings.max_voltage = v_interp;

            result.push((pose, settings, i));
        }
    }

    // push final waypoint with a sensible heading
    let last = points[n - 1].0;
    let last_heading = if last.heading.is_finite() {
        last.heading
    } else if n >= 2 {
        let dx = points[n - 1].0.x() - points[n - 2].0.x();
        let dy = points[n - 1].0.y() - points[n - 2].0.y();
        dy.atan2(dx)
    } else {
        0.0
    };
    result.push((Pose { x: last.x, y: last.y, heading: last_heading }, points[n - 1].1, n - 1));

    result
}
