use core::{f64::consts::PI, time::Duration};

use super::chassis::{Chassis, DetectedColour, IntakeMode};
extern crate alloc;
use vexide::{prelude::*, time::Instant};

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
    pub fn cheesy_control(&mut self, c_state: &vexide::devices::controller::ControllerState) {
        self.update_state();
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
            let angular_cmd =
                linear_cmd.abs() * (remapped_turn + self.neg_inertia_accumulator) * CD_SENSITIVITY
                    - self.quick_stop_accumulator;
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
    pub fn toggle_scraper(&mut self) {
        let _ = self.scraper.toggle();
        self.scraper_up = !self.scraper_up;
    }
    pub fn toggle_wings(&mut self) {
        let _ = self.wings.toggle();
    }
    pub fn toggle_block_park(&mut self) {
        let _ = self.block_park.toggle();
    }
    pub fn block_park_macro_is_active(&self) -> bool {
        self.block_park_macro_active
    }
    pub fn start_block_park_macro(&mut self) {
        self.block_park_macro_active = true;
        self.block_park_macro_detected = false;
        self.block_park_macro_post_delay_until = None;
        let prox = self.optical_sensor.proximity().unwrap_or_default();
        self.block_park_macro_last_prox_high = prox > 0.35;
        let _ = self.block_park.set_low();
        self.intake_mode = IntakeMode::Idle;
        self.outtake_initial_reverse_until = None;
        self.outtake_jam_reverse_until = None;
        self.outtake_middle_initial_reverse_until = None;
        self.outtake_middle_jam_reverse_until = None;
    }
    pub fn cancel_block_park_macro(&mut self) {
        self.block_park_macro_active = false;
        self.block_park_macro_detected = false;
        self.block_park_macro_post_delay_until = None;
        self.block_park_macro_last_prox_high = false;
        for m in self.intake_motors.iter_mut() {
            let _ = m.set_voltage(0.0);
        }
        let _ = self.block_park.set_low();
    }
    pub fn service_block_park_macro(&mut self) {
        if !self.block_park_macro_active {
            return;
        }

        let now = Instant::now();
        let prox = self.optical_sensor.proximity().unwrap_or_default();

        if !self.block_park_macro_detected {
            // hysteresis around thresholds similar to block counter
            let detect_threshold = 0.35;
            let release_threshold = 0.15;
            let was_high = self.block_park_macro_last_prox_high;
            let is_high = if was_high {
                prox > release_threshold
            } else {
                prox > detect_threshold
            };
            if is_high && !was_high {
                self.block_park_macro_detected = true;
                self.block_park_macro_post_delay_until = Some(now + Duration::from_millis(214));
            }
            self.block_park_macro_last_prox_high = is_high;
        }

        // keep reversing until post-delay elapses after first detection
        if !self.block_park_macro_detected
            || self
                .block_park_macro_post_delay_until
                .is_some_and(|until| now < until)
        {
            for m in self.intake_motors.iter_mut() {
                let _ = m.set_voltage(-9.0);
            }
            return;
        }

        // done: stop intake and activate block park
        for m in self.intake_motors.iter_mut() {
            let _ = m.set_voltage(0.0);
        }
        let _ = self.block_park.set_high();
        self.block_park_macro_active = false;
        self.block_park_macro_detected = false;
        self.block_park_macro_post_delay_until = None;
        self.block_park_macro_last_prox_high = false;
    }
    pub fn toggle_colour_sort(&mut self) {
        self.colour_sort_enabled = !self.colour_sort_enabled;
        if !self.colour_sort_enabled {
            self.colour_sort_activation_time = None;
            self.colour_sort_run_until = None;
            self.set_colour_sort_state(false);
        }
    }

    fn hue_is_red(hue: f64) -> bool {
        let h = hue.rem_euclid(360.0);
        !(60.0..300.0).contains(&h)
    }
    fn hue_is_blue(hue: f64) -> bool {
        let h = hue.rem_euclid(360.0);
        (180.0..300.0).contains(&h)
    }
    fn set_colour_sort_state(&mut self, high: bool) {
        if self.alt_colour_sort_enabled {
            let _ = self.colour_sort.set_low();
            return;
        }
        if high {
            let _ = self.colour_sort.set_high();
        } else {
            let _ = self.colour_sort.set_low();
        }
    }
    fn update_colour_sort(&mut self, detected_colour: DetectedColour, now: Instant) {
        self.last_detected_colour = detected_colour;

        if !self.colour_sort_enabled {
            self.colour_sort_activation_time = None;
            self.colour_sort_run_until = None;
            self.set_colour_sort_state(false);
            return;
        }

        let enemy_detected = match detected_colour {
            DetectedColour::Red => !self.team_is_red,
            DetectedColour::Blue => self.team_is_red,
            DetectedColour::Unknown => false,
        };

        if enemy_detected {
            const ACTIVATION_DELAY_MS: u64 = 75;
            const PULSE_DURATION_MS: u64 = 25;
            let activation_delay = Duration::from_millis(ACTIVATION_DELAY_MS);
            let pulse_duration = Duration::from_millis(PULSE_DURATION_MS);

            let mut activation_time_opt = self.colour_sort_activation_time;
            let currently_running = self.colour_sort_activation_time.is_none()
                && matches!(self.colour_sort_run_until, Some(until) if now < until);

            let activation_time = if currently_running {
                activation_time_opt = None;
                now
            } else {
                match activation_time_opt {
                    Some(existing) => existing,
                    None => {
                        let activation = now + activation_delay;
                        activation_time_opt = Some(activation);
                        activation
                    }
                }
            };

            let base_end = self
                .colour_sort_run_until
                .unwrap_or(activation_time.max(now));
            let new_run_end = base_end.max(activation_time) + pulse_duration;
            self.colour_sort_run_until = Some(new_run_end);
            self.colour_sort_activation_time = activation_time_opt;
        }

        if let Some(run_until) = self.colour_sort_run_until {
            if now >= run_until {
                self.colour_sort_run_until = None;
                self.colour_sort_activation_time = None;
                self.set_colour_sort_state(false);
            } else if let Some(activation_time) = self.colour_sort_activation_time {
                if now >= activation_time {
                    self.colour_sort_activation_time = None;
                    self.set_colour_sort_state(true);
                } else {
                    self.set_colour_sort_state(false);
                }
            } else {
                self.set_colour_sort_state(true);
            }
        } else if !enemy_detected {
            self.set_colour_sort_state(false);
        }
    }

    #[inline]
    fn alt_colour_sort_active(&self, now: Instant) -> bool {
        if !self.alt_colour_sort_enabled {
            return false;
        }
        if let Some(until) = self.colour_sort_run_until {
            if now >= until {
                return false;
            }
            if let Some(activation) = self.colour_sort_activation_time {
                return now >= activation;
            }
            return true;
        }
        false
    }
    pub fn toggle_hood(&mut self) {
        let _ = self.hood.toggle();
    }
    pub fn handle_intake_subsystem(
        &mut self,
        intake_toggle_pressed: bool,
        reverse_toggle_pressed: bool,
        reverse_held: bool,
        outtake_toggle_pressed: bool,
        outtake_middle_toggle_pressed: bool,
    ) {
        let any_timers = self.outtake_initial_reverse_until.is_some()
            || self.outtake_jam_reverse_until.is_some()
            || self.outtake_middle_initial_reverse_until.is_some()
            || self.outtake_middle_jam_reverse_until.is_some();
        let any_running = self.intake_mode != IntakeMode::Idle || any_timers;

        if (intake_toggle_pressed
            || outtake_toggle_pressed
            || outtake_middle_toggle_pressed
            || reverse_toggle_pressed)
            && any_running
        {
            self.intake_mode = IntakeMode::Idle;
            self.outtake_initial_reverse_until = None;
            self.outtake_jam_reverse_until = None;
            self.outtake_middle_initial_reverse_until = None;
            self.outtake_middle_jam_reverse_until = None;
            self.indexer_run_until = None;
            self.indexer_pending_activation_until = None;
            for m in self.intake_motors.iter_mut() {
                let _ = m.set_voltage(0.0);
            }
            return;
        }

        if !any_running {
            if intake_toggle_pressed {
                self.intake_mode = match self.intake_mode {
                    IntakeMode::Intake => IntakeMode::Idle,
                    _ => IntakeMode::Intake,
                };
            } else if outtake_toggle_pressed {
                self.intake_mode = match self.intake_mode {
                    IntakeMode::Outtake => IntakeMode::Idle,
                    _ => IntakeMode::Outtake,
                };
                if self.intake_mode == IntakeMode::Outtake {
                    self.outtake_initial_reverse_until =
                        Some(Instant::now() + Duration::from_millis(100));
                }
            } else if outtake_middle_toggle_pressed {
                self.intake_mode = match self.intake_mode {
                    IntakeMode::OuttakeMiddle => IntakeMode::Idle,
                    _ => IntakeMode::OuttakeMiddle,
                };
                if self.intake_mode == IntakeMode::OuttakeMiddle {
                    self.outtake_middle_initial_reverse_until =
                        Some(Instant::now() + Duration::from_millis(100));
                }
            } else if reverse_toggle_pressed {
                // L2 acts as a toggle ONLY when not running on top of other commands
                self.intake_mode = match self.intake_mode {
                    IntakeMode::Reverse => IntakeMode::Idle,
                    _ => IntakeMode::Reverse,
                };
            }
        }

        if reverse_held {
            for m in self.intake_motors.iter_mut() {
                let _ = m.set_voltage(-12.0);
            }
            return;
        }

        let stalled = self.intake_stalled();
        Self::poll_stall_and_reverse_for(
            matches!(self.intake_mode, IntakeMode::Outtake),
            stalled,
            &mut self.outtake_initial_reverse_until,
            &mut self.outtake_jam_reverse_until,
        );
        Self::poll_stall_and_reverse_for(
            matches!(self.intake_mode, IntakeMode::OuttakeMiddle),
            stalled,
            &mut self.outtake_middle_initial_reverse_until,
            &mut self.outtake_middle_jam_reverse_until,
        );

        let reversing_outtake = self.outtake_initial_reverse_until.is_some()
            || self.outtake_jam_reverse_until.is_some();
        let reversing_outtake_middle = self.outtake_middle_initial_reverse_until.is_some()
            || self.outtake_middle_jam_reverse_until.is_some();

        let outtake_middle_active =
            matches!(self.intake_mode, IntakeMode::OuttakeMiddle) || reversing_outtake_middle;
        if outtake_middle_active && self.apply_outtake_middle(reversing_outtake_middle) {
            return;
        }

        let outtake_active = matches!(self.intake_mode, IntakeMode::Outtake) || reversing_outtake;
        if outtake_active && self.apply_outtake(reversing_outtake) {
            return;
        }

        if matches!(self.intake_mode, IntakeMode::Reverse) && self.apply_reverse() {
            return;
        }

        if matches!(self.intake_mode, IntakeMode::Intake) && self.apply_intake() {
            return;
        }

        self.stop_all();
    }

    pub fn service_autonomous_intake(&mut self) {
        self.handle_intake_subsystem(false, false, false, false, false);
    }

    fn stop_all(&mut self) {
        for m in self.intake_motors.iter_mut() {
            let _ = m.set_voltage(0.0);
        }
    }

    fn apply_outtake_middle(&mut self, reversing_outtake_middle: bool) -> bool {
        let dir = if reversing_outtake_middle {
            -1.0
        } else if matches!(self.intake_mode, IntakeMode::OuttakeMiddle) {
            1.0
        } else {
            0.0
        };
        if dir != 0.0 {
            self.block_counter.block_count = 0;
            for (i, m) in self.intake_motors.iter_mut().enumerate() {
                let applied = if i == 2 { -8.0 } else { 8.0 };
                let _ = m.set_voltage(applied);
            }
            if dir > 0.0 {
                let _ = self.hood.set_high();
            }
            return true;
        }
        false
    }

    fn apply_outtake(&mut self, reversing_outtake: bool) -> bool {
        let dir = if reversing_outtake { -1.0 } else { 1.0 };
        self.block_counter.block_count = 0;
        for m in self.intake_motors.iter_mut() {
            let _ = m.set_voltage(12.0 * dir);
        }
        if dir > 0.0 {
            let _ = self.hood.set_high();
        }
        true
    }

    fn apply_reverse(&mut self) -> bool {
        for m in self.intake_motors.iter_mut() {
            let _ = m.set_voltage(-10.0);
        }
        true
    }

    fn apply_intake(&mut self) -> bool {
        let _ = self.optical_sensor.set_led_brightness(1.0);
        let prox = self.optical_sensor.proximity().unwrap_or_default();
        let hue = self.optical_sensor.hue().unwrap_or_default();
        let now = Instant::now();

        let detected_colour = if Self::hue_is_red(hue) {
            DetectedColour::Red
        } else if Self::hue_is_blue(hue) {
            DetectedColour::Blue
        } else {
            DetectedColour::Unknown
        };
        if self.alt_colour_sort_enabled {
            let enemy_detected = match detected_colour {
                DetectedColour::Red => !self.team_is_red,
                DetectedColour::Blue => self.team_is_red,
                DetectedColour::Unknown => false,
            };
            const ALT_REVERSE_MS: u64 = 100;
            let pulse = Duration::from_millis(ALT_REVERSE_MS);
            if enemy_detected {
                if !self.alt_colour_last_enemy {
                    match self.alt_colour_sort_run_until {
                        Some(curr) if now < curr => {
                            self.alt_colour_sort_run_until = Some(curr + pulse);
                        }
                        _ => {
                            self.alt_colour_sort_run_until = Some(now + pulse);
                        }
                    }
                    self.alt_colour_last_enemy = true;
                }
            } else {
                self.alt_colour_last_enemy = false;
            }
            if let Some(until) = self.alt_colour_sort_run_until {
                if now < until {
                    if let Some(front) = self.intake_motors.first_mut() {
                        let _ = front.set_voltage(-9.0);
                    }
                    let _ = self.colour_sort.set_low();
                    return true;
                } else {
                    self.alt_colour_sort_run_until = None;
                }
            }
        } else {
            self.update_colour_sort(detected_colour, now);
        }

        if self.block_counter.update(prox) && self.block_counter.block_count <= 7 {
            if let Some(until) = self.indexer_run_until {
                if until < now + Duration::from_millis(200) {
                    self.indexer_run_until = Some(now + Duration::from_millis(200));
                }
            } else {
                let delay = self.block_counter.activation_delay;
                let target = now + delay;
                self.indexer_pending_activation_until =
                    Some(match self.indexer_pending_activation_until {
                        Some(existing) if existing > target => existing,
                        _ => target,
                    });
            }
        }

        if let Some(pending_until) = self.indexer_pending_activation_until
            && now >= pending_until
        {
            self.indexer_run_until = Some(now + Duration::from_millis(200));
            self.indexer_pending_activation_until = None;
        }
        let run_upper = match self.indexer_run_until {
            Some(until) if now < until => true,
            _ => {
                self.indexer_run_until = None;
                false
            }
        };
        if run_upper {
            for m in self.intake_motors.iter_mut() {
                let _ = m.set_voltage(12.0);
            }
        } else {
            for (i, m) in self.intake_motors.iter_mut().enumerate() {
                if i == 0 {
                    let _ = m.set_voltage(12.0);
                } else {
                    let _ = m.set_voltage(0.0);
                }
            }
        }
        let _ = self.hood.set_low();
        true
    }

    fn poll_stall_and_reverse_for(
        is_active: bool,
        stalled: bool,
        initial_reverse_until: &mut Option<Instant>,
        jam_reverse_until: &mut Option<Instant>,
    ) {
        let now = Instant::now();

        if let Some(until) = *initial_reverse_until
            && now >= until
        {
            *initial_reverse_until = None;
        }
        if let Some(until) = *jam_reverse_until
            && now >= until
        {
            *jam_reverse_until = None;
        }

        if initial_reverse_until.is_some() || jam_reverse_until.is_some() {
            return;
        }

        if is_active && stalled {
            *jam_reverse_until = Some(now + Duration::from_millis(150));
        }
    }

    pub fn intake_stalled(&self) -> bool {
        let current_thresh = self.config.stall_current_threshold;
        let vel_thresh = self.config.stall_velocity_threshold;

        let mut any_stalled = false;
        for m in self.intake_motors.iter() {
            let current = m.current().unwrap_or_default().abs();
            let velocity = m.velocity().unwrap_or_default().abs();
            if current >= current_thresh && velocity <= vel_thresh {
                any_stalled = true;
            }
        }
        any_stalled
    }
}
