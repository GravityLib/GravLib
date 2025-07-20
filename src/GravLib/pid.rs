use vexide::time::Instant;
use core::time::Duration;

#[derive(Debug, Clone, Copy)]
pub struct Gains {
    pub k_p: f64,
    pub k_i: f64,
    pub k_d: f64,
}

impl Gains {
    pub fn new(k_p: f64, k_i: f64, k_d: f64) -> Self {
        Self { k_p, k_i, k_d }
    }
}

pub struct PID {
    m_gains: Gains,
    m_sign_flip_reset: bool,
    m_windup_range: f64,
    m_previous_error: f64,
    m_integral: f64,
    m_prev_time: Option<Instant>
}

impl PID {
    pub fn new(k_p: f64, k_i: f64, k_d: f64, windup_range: f64, sign_flip_reset: bool) -> Self {
        Self {
            m_gains: Gains::new(k_p, k_i, k_d),
            m_windup_range: windup_range,
            m_sign_flip_reset: sign_flip_reset,
            m_previous_error: 0.0,
            m_integral: 0.0,
            m_prev_time: None,
        }
    }

    /// Get the current PID gains (Copy trait allows returning by value)
    pub fn gains(&self) -> Gains {
        self.m_gains
    }

    /// Set new PID gains
    pub fn set_gains(&mut self, gains: Gains) {
        self.m_gains = gains;
    }

    /// Get individual gain values
    pub fn k_p(&self) -> f64 {
        self.m_gains.k_p
    }

    pub fn k_i(&self) -> f64 {
        self.m_gains.k_i
    }

    pub fn k_d(&self) -> f64 {
        self.m_gains.k_d
    }

    /// Set individual gain values
    pub fn set_k_p(&mut self, k_p: f64) {
        self.m_gains.k_p = k_p;
    }

    pub fn set_k_i(&mut self, k_i: f64) {
        self.m_gains.k_i = k_i;
    }

    pub fn set_k_d(&mut self, k_d: f64) {
        self.m_gains.k_d = k_d;
    }

    /// Get sign flip reset setting
    pub fn sign_flip_reset(&self) -> bool {
        self.m_sign_flip_reset
    }

    /// Set sign flip reset behavior
    pub fn set_sign_flip_reset(&mut self, sign_flip_reset: bool) {
        self.m_sign_flip_reset = sign_flip_reset;
    }

    /// Get windup range setting
    pub fn windup_range(&self) -> f64 {
        self.m_windup_range
    }

    /// Set windup range (anti-windup threshold)
    pub fn set_windup_range(&mut self, windup_range: f64) {
        self.m_windup_range = windup_range;
    }

    /// Get current integral accumulator value (for debugging)
    pub fn integral(&self) -> f64 {
        self.m_integral
    }

    /// Get previous error value (for debugging)
    pub fn previous_error(&self) -> f64 {
        self.m_previous_error
    }

    pub fn update(&mut self, error: f64) -> f64 {
        // Finding time delta
        let now = Instant::now();

        // TODO - Check algorithm for mapping of prev time
        let dt = if let Some(prev) = self.m_prev_time {
            now.duration_since(prev)  // compute elapsed time
        } else {
            Duration::ZERO           // default to zero on the first call
        };

        self.m_prev_time = Some(now);

        let dt_secs = dt.as_secs_f64();

        let derivative = if dt_secs > 0.0 {
            (error - self.m_previous_error) / dt_secs
        } else {
            0.0
        };
        
        self.m_previous_error = error;

        // accumulate integral (error * time)
        self.m_integral += error * dt_secs;

        // reset integral on sign flip (if enabled)
        if self.m_sign_flip_reset && (error.signum() != self.m_previous_error.signum()) {
            self.m_integral = 0.0;
        }
    
        // anti windup range. Unless error is small enough, set the integral to 0
        if  error.abs() > self.m_windup_range && self.m_windup_range != 0.0 {
            self.m_integral = 0.0;
        }

        // calculate output
        error * self.m_gains.k_p
            + self.m_integral * self.m_gains.k_i
            + derivative * self.m_gains.k_d
    }   

    pub fn reset(&mut self) {
        self.m_previous_error = 0.0;
        self.m_integral = 0.0;
    }
}