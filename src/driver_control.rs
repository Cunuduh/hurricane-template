use super::chassis::Chassis;
use core::{f64::consts::PI, time::Duration};
extern crate alloc;
use vexide::{
    prelude::*,
    time::Instant,
};

impl<const L: usize, const R: usize, const I: usize> Chassis<L, R, I> {
    // https://www.desmos.com/calculator/7oyvwwpmed
    fn drive_curve(&self, x: f64, t: f64) -> f64 {
        ((-t / 10.0).exp() + ((x.abs() - 127.0) / 10.0).exp() * (1.0 - (-t / 10.0).exp())) * x
    }
    // https://wiki.purduesigbots.com/software/robotics-basics/curvature-cheesy-drive
    fn cheesy_turn_remap(&self, iturn: f64) -> f64 {
        const CD_TURN_NONLINEARITY: f64 = 0.65;
        let denom = (PI / 2.0 * CD_TURN_NONLINEARITY).sin();
        let first = (PI / 2.0 * CD_TURN_NONLINEARITY * iturn).sin() / denom;
        (PI / 2.0 * CD_TURN_NONLINEARITY * first).sin() / denom
    }
    fn update_cheesy_accumulators(&mut self) {
        if self.neg_inertia_accumulator > 1.0 {
            self.neg_inertia_accumulator -= 1.0;
        } else if self.neg_inertia_accumulator < -1.0 {
            self.neg_inertia_accumulator += 1.0;
        } else {
            self.neg_inertia_accumulator = 0.0;
        }
        if self.quick_stop_accumulator > 1.0 {
            self.quick_stop_accumulator -= 1.0;
        } else if self.quick_stop_accumulator < -1.0 {
            self.quick_stop_accumulator += 1.0;
        } else {
            self.quick_stop_accumulator = 0.0;
        }
    }
    pub fn cheesy_control(&mut self) {
        self.update_odometry();
        let c_state = self.controller.state().unwrap_or_default();
        let throttle = c_state.left_stick.y();
        let turn = c_state.right_stick.x();
        // taken from BLRS repo
        const DRIVE_DEADBAND: f64 = 0.05;
        const DRIVE_SLEW: f64 = 0.02;
        const CD_NEG_INERTIA_SCALAR: f64 = 4.0;
        const CD_SENSITIVITY: f64 = 1.0;
        let mut turn_in_place = false;
        let mut linear_cmd = throttle;
        if throttle.abs() < DRIVE_DEADBAND && turn.abs() > DRIVE_DEADBAND {
            linear_cmd = 0.0;
            turn_in_place = true;
        } else if throttle - self.prev_throttle > DRIVE_SLEW {
            linear_cmd = self.prev_throttle + DRIVE_SLEW;
        } else if throttle - self.prev_throttle < -(DRIVE_SLEW * 2.0) {
            linear_cmd = self.prev_throttle - (DRIVE_SLEW * 2.0);
        }
        let remapped_turn = self.cheesy_turn_remap(turn);
        let (left, right) = if turn_in_place {
            let l = remapped_turn * remapped_turn.abs();
            let r = -remapped_turn * remapped_turn.abs();
            (l, r)
        } else {
            let neg_inertia_power = (turn - self.prev_turn) * CD_NEG_INERTIA_SCALAR;
            self.neg_inertia_accumulator += neg_inertia_power;
            let angular_cmd = linear_cmd.abs() * (remapped_turn + self.neg_inertia_accumulator) * CD_SENSITIVITY - self.quick_stop_accumulator;
            let mut l = linear_cmd;
            let mut r = linear_cmd;
            l += angular_cmd;
            r -= angular_cmd;
            self.update_cheesy_accumulators();
            (l, r)
        };
        self.prev_turn = turn;
        self.prev_throttle = throttle;

        let curved_left = self.drive_curve(left * 127.0, 5.0) / 127.0;
        let curved_right = self.drive_curve(right * 127.0, 5.0) / 127.0;

        let max_volts = self.config.max_volts;
        let vl = (curved_left * max_volts).clamp(-max_volts, max_volts);
        let vr = (curved_right * max_volts).clamp(-max_volts, max_volts);
        for m in self.left_motors.iter_mut() {
            let _ = m.set_voltage(vl);
        }
        for m in self.right_motors.iter_mut() {
            let _ = m.set_voltage(vr);
        }
    }

    pub fn handle_intake_outtake_controls(&mut self) {
        let c_state = self.controller.state().unwrap_or_default();
        let l1 = c_state.button_l1.is_pressed();
        let l2 = c_state.button_l2.is_pressed();
        let r1 = c_state.button_r1.is_pressed();
        let r2 = c_state.button_r2.is_pressed();

        let l1_edge = l1 && !self.prev_l1;
        let r1_edge = r1 && !self.prev_r1;
        let r2_edge = r2 && !self.prev_r2;

        let any_running = self.intake_state != 0
            || self.outtake_state != 0
            || self.outtake_middle_state != 0
            || self.outtake_initial_reverse_until.is_some()
            || self.outtake_jam_reverse_until.is_some()
            || self.outtake_middle_initial_reverse_until.is_some()
            || self.outtake_middle_jam_reverse_until.is_some();

        if (l1_edge || r1_edge || r2_edge) && any_running {
            self.intake_state = 0;
            self.outtake_state = 0;
            self.outtake_middle_state = 0;
            self.outtake_initial_reverse_until = None;
            self.outtake_jam_reverse_until = None;
            self.outtake_middle_initial_reverse_until = None;
            self.outtake_middle_jam_reverse_until = None;
            for m in self.intake_motors.iter_mut() { let _ = m.set_voltage(0.0); }
            self.prev_l1 = l1;
            self.prev_r1 = r1;
            self.prev_r2 = r2;
            return;
        }

        if !(self.intake_state != 0
            || self.outtake_state != 0
            || self.outtake_middle_state != 0
            || self.outtake_initial_reverse_until.is_some()
            || self.outtake_jam_reverse_until.is_some()
            || self.outtake_middle_initial_reverse_until.is_some()
            || self.outtake_middle_jam_reverse_until.is_some()) {
            if l1_edge {
                self.intake_state = if self.intake_state == 1 { 0 } else { 1 };
            } else if r1_edge {
                if self.outtake_state == 1 {
                    self.outtake_state = 0;
                    self.outtake_jam_reverse_until = None;
                    self.outtake_initial_reverse_until = None;
                } else {
                    self.outtake_state = 1;
                    self.outtake_initial_reverse_until = Some(Instant::now() + Duration::from_millis(175));
                }
            } else if r2_edge {
                if self.outtake_middle_state == 1 {
                    self.outtake_middle_state = 0;
                    self.outtake_middle_jam_reverse_until = None;
                    self.outtake_middle_initial_reverse_until = None;
                } else {
                    self.outtake_middle_state = 1;
                    self.outtake_middle_initial_reverse_until = Some(Instant::now() + Duration::from_millis(175));
                }
            }
        }

        self.prev_l1 = l1;
        self.prev_r1 = r1;
        self.prev_r2 = r2;
        
        if l2 {
            for m in self.intake_motors.iter_mut() { let _ = m.set_voltage(-12.0); }
            return;
        }

        let stalled = self.intake_stalled();
        Self::poll_stall_and_reverse_for(
            self.outtake_state,
            stalled,
            &mut self.outtake_initial_reverse_until,
            &mut self.outtake_jam_reverse_until,
        );
        Self::poll_stall_and_reverse_for(
            self.outtake_middle_state,
            stalled,
            &mut self.outtake_middle_initial_reverse_until,
            &mut self.outtake_middle_jam_reverse_until,
        );

        let reversing_outtake = self.outtake_initial_reverse_until.is_some() || self.outtake_jam_reverse_until.is_some();
        let reversing_outtake_middle = self.outtake_middle_initial_reverse_until.is_some() || self.outtake_middle_jam_reverse_until.is_some();

        let outtake_middle_active = self.outtake_middle_state != 0 || reversing_outtake_middle;
        if outtake_middle_active {
            let dir = if l2 || reversing_outtake_middle { -1.0 } else { self.outtake_middle_state as f64 };
            if dir != 0.0 {
                for (i, m) in self.intake_motors.iter_mut().enumerate() {
                    let applied = if i == 2 { -dir } else { dir };
                    let _ = m.set_voltage(12.0 * applied);
                }
                return;
            }
        }

        let outtake_active = self.outtake_state != 0 || reversing_outtake;
        if outtake_active {
            let dir = if l2 || reversing_outtake { -1.0 } else { self.outtake_state as f64 };
            if dir != 0.0 {
                for m in self.intake_motors.iter_mut() { let _ = m.set_voltage(12.0 * dir); }
                return;
            }
        }

        let dir = if l2 { -1.0 } else { self.intake_state as f64 };
        if dir != 0.0 {
            for m in self.intake_motors.iter_mut() { let _ = m.set_voltage(12.0 * dir); }
        }
    }

    fn poll_stall_and_reverse_for(
        state: i8,
        stalled: bool,
        initial_reverse_until: &mut Option<Instant>,
        jam_reverse_until: &mut Option<Instant>,
    ) {
        let now = Instant::now();

        if let Some(until) = *initial_reverse_until
            && now >= until {
                *initial_reverse_until = None;
            }
        if let Some(until) = *jam_reverse_until
            && now >= until {
                *jam_reverse_until = None;
            }

        if initial_reverse_until.is_some() || jam_reverse_until.is_some() {
            return;
        }

        if state == 1 && stalled {
            *jam_reverse_until = Some(now + Duration::from_millis(150));
        }
    }

    pub fn intake_stalled(&self) -> bool {
        let current_thresh = self.config.stall_current_threshold;
        let vel_thresh = self.config.stall_velocity_threshold;

        let mut any_stalled = false;
        for m in self.intake_motors.iter() {
            let current = m.current().unwrap_or_default();
            let velocity = m.velocity().unwrap_or_default().abs();
            if current >= current_thresh && velocity <= vel_thresh {
                let port = m.port_number();
                println!(
                    "intake_stalled: motor port {} current={:.3}A velocity={:.3} (thresh curr={:.3}, vel={:.3})",
                    port, current, velocity, current_thresh, vel_thresh
                );
                any_stalled = true;
            }
        }
        any_stalled
    }
}