#[derive(Copy, Clone, Debug)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral_limit: f64,
    pub integral_decay: f64,
    prev_error: Option<f64>,
    integral: f64,
    disabled: bool,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            integral_decay: 0.995,
            prev_error: None,
            integral: 0.0,
            disabled: false,
        }
    }

    pub fn next(&mut self, error: f64, dt: f64) -> f64 {
        if self.disabled {
            self.reset();
            return 0.0;
        }

        self.integral *= self.integral_decay;
        self.integral += error * dt;
        if self.integral_limit > 0.0 {
            self.integral = self
                .integral
                .clamp(-self.integral_limit, self.integral_limit);
        }

        let derivative = match self.prev_error {
            Some(prev) => (error - prev) / dt,
            None => 0.0,
        };

        self.prev_error = Some(error);
        (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    }

    pub fn reset(&mut self) {
        self.prev_error = None;
        self.integral = 0.0;
    }

    pub fn set_disabled(&mut self, disabled: bool) {
        self.disabled = disabled;
        if disabled {
            self.reset();
        }
    }
}
