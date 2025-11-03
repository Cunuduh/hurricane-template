#![allow(dead_code)]

extern crate alloc;

use alloc::vec::Vec;
use core::{
    f32::consts::PI,
    mem,
    simd::{f32x4, num::SimdFloat},
};

use rand::RngCore;
use veranda::SystemRng;
use vexide::{io::println, prelude::Float};

use crate::odometry::Pose;

const N: usize = 256;
const INV_N: f32 = (N as f32).recip();
const GAUSSIAN_STDDEV: f32 = 3.5; // inches
const GAUSSIAN_FACTOR: f32 = 1.0;
const Z_HIT: f32 = 0.9;
const Z_RAND: f32 = 0.1;
const XY_NOISE: f32 = 0.1; // inches per update step
const REINIT_SPREAD: f32 = 8.0;
const UNIFORM_PARTICLE_RATIO: f32 = 0.2;
const FIELD_HALF: f32 = 72.0; // +/- 72 inches for VRC field

const MIN_BEAMS_FOR_UPDATE: usize = 2;
const MAX_DIST_SINCE_UPDATE: f64 = 2.0; // inches
const DYN_STDDEV_SCALE: f32 = 0.05; // extra stddev per inch of measured distance
const RESAMPLE_JITTER: f32 = 0.05; // small uniform jitter after resample

const SIMD_WIDTH: usize = 4;

fn sum_particle_components(particles: &Particles) -> (f32, f32, f32) {
    let mut sx_simd = f32x4::splat(0.0);
    let mut sy_simd = f32x4::splat(0.0);
    let mut st_simd = f32x4::splat(0.0);
    let mut idx = 0;
    while idx + SIMD_WIDTH <= N {
        sx_simd += f32x4::from_slice(&particles.x[idx..idx + SIMD_WIDTH]);
        sy_simd += f32x4::from_slice(&particles.y[idx..idx + SIMD_WIDTH]);
        st_simd += f32x4::from_slice(&particles.theta[idx..idx + SIMD_WIDTH]);
        idx += SIMD_WIDTH;
    }
    let mut sx = sx_simd.reduce_sum();
    let mut sy = sy_simd.reduce_sum();
    let mut st = st_simd.reduce_sum();
    for i in idx..N {
        sx += particles.x[i];
        sy += particles.y[i];
        st += particles.theta[i];
    }
    (sx, sy, st)
}

fn clamp_weight(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

fn prefix_sum(src: &[f32], dst: &mut [f32]) {
    assert_eq!(src.len(), dst.len());

    let mut offset = 0.0_f32;
    let mut idx = 0;
    while idx + SIMD_WIDTH <= src.len() {
        let x = f32x4::from_slice(&src[idx..idx + SIMD_WIDTH]);
        let y1 = f32x4::from_array([0.0, x[0], x[1], x[2]]);
        let s1 = x + y1;
        let y2 = f32x4::from_array([0.0, 0.0, s1[0], s1[1]]);
        let s2 = s1 + y2;
        let out = s2 + f32x4::splat(offset);
        out.copy_to_slice(&mut dst[idx..idx + SIMD_WIDTH]);
        offset = out[SIMD_WIDTH - 1];
        idx += SIMD_WIDTH;
    }
    while idx < src.len() {
        offset += src[idx];
        dst[idx] = offset;
        idx += 1;
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Beam {
    pub angle: f32,
    pub distance: f32,
    pub offset_x: f32,
    pub offset_y: f32,
}

#[derive(Clone, Debug)]
struct Particles {
    x: Vec<f32>,
    y: Vec<f32>,
    theta: Vec<f32>,
    weight: Vec<f32>,
}

impl Particles {
    fn new(initial_pose: Pose, n: usize) -> Self {
        let mut x = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        let mut theta = Vec::with_capacity(n);
        let mut weight = Vec::with_capacity(n);
        for _ in 0..n {
            x.push(initial_pose.x as f32);
            y.push(initial_pose.y as f32);
            theta.push(initial_pose.heading as f32);
            weight.push(-1.0_f32);
        }
        Self {
            x,
            y,
            theta,
            weight,
        }
    }

    fn repair_out_of_field(&mut self, rng: &mut SystemRng, field_half: f32) {
        for i in 0..self.len() {
            let mut xi = self.x[i];
            let mut yi = self.y[i];
            if xi < -field_half || xi > field_half {
                let u = next_f32(rng) * 2.0 - 1.0;
                xi = u * field_half;
            }
            if yi < -field_half || yi > field_half {
                let u = next_f32(rng) * 2.0 - 1.0;
                yi = u * field_half;
            }
            self.x[i] = xi;
            self.y[i] = yi;
        }
    }

    fn with_capacity(n: usize) -> Self {
        Self {
            x: Vec::with_capacity(n),
            y: Vec::with_capacity(n),
            theta: Vec::with_capacity(n),
            weight: Vec::with_capacity(n),
        }
    }

    fn clear(&mut self) {
        self.x.clear();
        self.y.clear();
        self.theta.clear();
        self.weight.clear();
    }

    fn len(&self) -> usize {
        self.x.len()
    }

    fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    fn update_delta_noise(
        &mut self,
        dx: f32,
        dy: f32,
        imu_heading: f32,
        rng: &mut SystemRng,
        xy_noise: f32,
    ) {
        for i in 0..self.len() {
            let nx = (next_f32(rng) * 2.0_f32) - 1.0_f32;
            let ny = (next_f32(rng) * 2.0_f32) - 1.0_f32;
            self.x[i] += dx + xy_noise * nx;
            self.y[i] += dy + xy_noise * ny;
            self.theta[i] = imu_heading;
        }
    }

    fn expected_point(&self, index: usize, beam: &Beam) -> (f32, f32, f32) {
        let theta = self.theta[index];
        let x = self.x[index];
        let y = self.y[index];
        let global_theta = theta + beam.angle;
        let ct = theta.cos();
        let st = theta.sin();
        let sx = x + beam.offset_x * ct - beam.offset_y * st;
        let sy = y + beam.offset_x * st + beam.offset_y * ct;
        (sx, sy, global_theta)
    }

    fn distance_to_wall(point: (f32, f32, f32), field_half: f32) -> f32 {
        let (px, py, ang) = point;
        let c = ang.cos();
        let s = ang.sin();

        let mut t = f32::INFINITY;
        if c > 1e-6 {
            t = t.min((field_half - px) / c);
        } else if c < -1e-6 {
            t = t.min((-field_half - px) / c);
        }
        if s > 1e-6 {
            t = t.min((field_half - py) / s);
        } else if s < -1e-6 {
            t = t.min((-field_half - py) / s);
        }
        if t.is_finite() { t.max(0.0) } else { 0.0 }
    }

    fn gaussian(x: f32, stddev: f32, factor: f32) -> f32 {
        if stddev <= 0.0 {
            return 0.0;
        }
        let z = x / stddev;
        let denom = stddev * (2.0_f32 * PI).sqrt();
        factor * (-0.5 * z * z).exp() / denom
    }

    fn update_weight(&mut self, beams: &[Beam], field_half: f32, stddev: f32, factor: f32) {
        if beams.is_empty() {
            for i in 0..self.len() {
                self.weight[i] = INV_N;
            }
            return;
        }

        let mut max_log_w = f32::NEG_INFINITY;
        let mut log_ws: Vec<f32> = Vec::with_capacity(self.len());
        for i in 0..self.len() {
            let mut log_w = 0.0_f32;
            for b in beams {
                let origin = self.expected_point(i, b);
                let expected_dist = Self::distance_to_wall(origin, field_half);
                let error = (b.distance - expected_dist).abs();
                let stddev_dyn = stddev + DYN_STDDEV_SCALE * b.distance;
                let p_hit = Self::gaussian(error, stddev_dyn, factor);
                let p = Z_HIT * p_hit + Z_RAND;
                let lp = p.max(1e-12).ln();
                log_w += lp;
            }
            if !log_w.is_finite() {
                log_w = f32::NEG_INFINITY;
            }
            if log_w > max_log_w {
                max_log_w = log_w;
            }
            log_ws.push(log_w);
        }

        // exponentiate after max subtraction to avoid underflow
        for (i, &lw) in log_ws.iter().enumerate() {
            let w = (lw - max_log_w).exp();
            self.weight[i] = if w.is_finite() { w } else { 0.0 };
        }
    }

    fn push_copy_from(&mut self, other: &Self, index: usize) {
        self.x.push(other.x[index]);
        self.y.push(other.y[index]);
        self.theta.push(other.theta[index]);
        self.weight.push(other.weight[index]);
    }
}

fn next_f32(rng: &mut impl RngCore) -> f32 {
    let bits = rng.next_u32() >> 8; // keep top 24 bits
    const SCALE: f32 = (1u32 << 24) as f32;
    (bits as f32) / SCALE
}

pub struct Mcl {
    particles: Particles,
    resampled_particles: Particles,
    weight_buffer: Vec<f32>,
    cumulative_weights: Vec<f32>,
    average_pose: Pose,
    last_valid_pose: Pose,
    rng: SystemRng,
    dist_since_update: f64,
}
// based on https://www.aadishv.dev/mcl/, but I decided IMU headings are good enough to not need to estimate them
impl Mcl {
    pub fn new(initial_pose: Pose) -> Self {
        let particles = Particles::new(initial_pose, N);
        let resampled_particles = Particles::with_capacity(N);
        let weight_buffer = Vec::with_capacity(N);
        let cumulative_weights = Vec::with_capacity(N);
        Self {
            particles,
            resampled_particles,
            weight_buffer,
            cumulative_weights,
            average_pose: initial_pose,
            last_valid_pose: initial_pose,
            rng: SystemRng::new(),
            dist_since_update: 0.0,
        }
    }

    pub fn run(&mut self, beams: &[Beam], delta_xy: (f64, f64), imu_heading: f64) -> Pose {
        self.update_step(delta_xy, imu_heading);
        // accumulate traveled distance for gated updates
        self.dist_since_update += (delta_xy.0.hypot(delta_xy.1)).abs();
        if beams.len() >= MIN_BEAMS_FOR_UPDATE && self.dist_since_update >= MAX_DIST_SINCE_UPDATE {
            self.resample_step(beams);
            self.dist_since_update = 0.0;
        }
        let mut p = self.average_pose;
        p.heading = imu_heading;
        p
    }

    pub fn pose(&self) -> Pose {
        self.average_pose
    }

    fn update_step(&mut self, delta_xy: (f64, f64), imu_heading: f64) {
        let dx = delta_xy.0 as f32;
        let dy = delta_xy.1 as f32;
        self.particles
            .update_delta_noise(dx, dy, imu_heading as f32, &mut self.rng, XY_NOISE);
    }

    fn clamp_weights(&mut self) -> f32 {
        self.weight_buffer.clear();
        let mut sum = 0.0_f32;
        for i in 0..N {
            let w = clamp_weight(self.particles.weight[i]);
            self.particles.weight[i] = w;
            self.weight_buffer.push(w);
            sum += w;
        }
        sum
    }

    fn resample_step(&mut self, beams: &[Beam]) {
        // keep particles within field before computing weights
        self.particles.repair_out_of_field(&mut self.rng, FIELD_HALF);
        self.particles
            .update_weight(beams, FIELD_HALF, GAUSSIAN_STDDEV, GAUSSIAN_FACTOR);

        let total_weight = self.clamp_weights();

        if total_weight <= 0.0 || !total_weight.is_finite() {
            println!(
                "MCL warning: total weight invalid ({}), skipping resample",
                total_weight
            );
            return;
        }
        if total_weight.is_finite() {
            self.last_valid_pose = self.average_pose;
        }
        self.resampled_particles.clear();
        let weight_len = self.weight_buffer.len();
        self.cumulative_weights.resize(weight_len, 0.0);
        self.cumulative_weights.fill(0.0);
        prefix_sum(&self.weight_buffer, &mut self.cumulative_weights);

        let u0: f32 = next_f32(&mut self.rng) * INV_N;
        let mut i = 0;
        // stochastic universal sampling
        for j in 0..N {
            let target = (u0 + (j as f32) * INV_N) * total_weight;
            while i + 1 < self.cumulative_weights.len() && self.cumulative_weights[i] < target {
                i += 1;
            }
            let idx = i.min(N - 1);
            self.resampled_particles
                .push_copy_from(&self.particles, idx);
        }
        mem::swap(&mut self.particles, &mut self.resampled_particles);

        // small jitter to reduce particle impoverishment
        if RESAMPLE_JITTER > 0.0 {
            for i in 0..N {
                let jx = (next_f32(&mut self.rng) * 2.0 - 1.0) * RESAMPLE_JITTER;
                let jy = (next_f32(&mut self.rng) * 2.0 - 1.0) * RESAMPLE_JITTER;
                self.particles.x[i] += jx;
                self.particles.y[i] += jy;
            }
        }

        let (sx, sy, st) = sum_particle_components(&self.particles);
        self.average_pose = Pose {
            x: (sx * INV_N) as f64,
            y: (sy * INV_N) as f64,
            heading: (st * INV_N) as f64,
        };
    }
}
