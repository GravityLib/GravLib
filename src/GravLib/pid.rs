use vexide::time::Instant;
use core::time::Duration;

#[derive(Debug, Clone, Copy)]
#[repr(C)] // Optimize memory layout
pub struct Gains {
    pub k_p: f64,
    pub k_i: f64,
    pub k_d: f64,
}

impl Gains {
    #[inline(always)]
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
    m_prev_time: Option<Instant>,
    // Cache frequently used values to avoid repeated calculations
    m_has_windup_range: bool, // Pre-calculated: windup_range != 0.0
    m_inv_windup_range: f64,  // Pre-calculated: 1.0 / windup_range (if applicable)
    // Slew rate limiting
    m_previous_output: Option<f64>,
    m_slew_rate: f64,
}

impl PID {
    pub fn new(k_p: f64, k_i: f64, k_d: f64, windup_range: f64, sign_flip_reset: bool) -> Self {
        let has_windup_range = windup_range != 0.0;
        let inv_windup_range = if has_windup_range { 1.0 / windup_range } else { 0.0 };
        
        Self {
            m_gains: Gains::new(k_p, k_i, k_d),
            m_windup_range: windup_range,
            m_sign_flip_reset: sign_flip_reset,
            m_previous_error: 0.0,
            m_integral: 0.0,
            m_prev_time: None,
            m_has_windup_range: has_windup_range,
            m_inv_windup_range: inv_windup_range,
            m_previous_output: None,
            m_slew_rate: 0.0, // No slew rate limiting by default
        }
    }

    /// Create a new PID controller with slew rate limiting
    pub fn new_with_slew(k_p: f64, k_i: f64, k_d: f64, windup_range: f64, sign_flip_reset: bool, slew_rate: f64) -> Self {
        let has_windup_range = windup_range != 0.0;
        let inv_windup_range = if has_windup_range { 1.0 / windup_range } else { 0.0 };
        
        Self {
            m_gains: Gains::new(k_p, k_i, k_d),
            m_windup_range: windup_range,
            m_sign_flip_reset: sign_flip_reset,
            m_previous_error: 0.0,
            m_integral: 0.0,
            m_prev_time: None,
            m_has_windup_range: has_windup_range,
            m_inv_windup_range: inv_windup_range,
            m_previous_output: None,
            m_slew_rate: slew_rate,
        }
    }

    /// Get the current PID gains (Copy trait allows returning by value)
    #[inline(always)]
    pub fn gains(&self) -> Gains {
        self.m_gains
    }

    /// Set new PID gains
    #[inline(always)]
    pub fn set_gains(&mut self, gains: Gains) {
        self.m_gains = gains;
    }

    /// Get individual gain values
    #[inline(always)]
    pub fn k_p(&self) -> f64 {
        self.m_gains.k_p
    }

    #[inline(always)]
    pub fn k_i(&self) -> f64 {
        self.m_gains.k_i
    }

    #[inline(always)]
    pub fn k_d(&self) -> f64 {
        self.m_gains.k_d
    }

    /// Set individual gain values
    #[inline(always)]
    pub fn set_k_p(&mut self, k_p: f64) {
        self.m_gains.k_p = k_p;
    }

    #[inline(always)]
    pub fn set_k_i(&mut self, k_i: f64) {
        self.m_gains.k_i = k_i;
    }

    #[inline(always)]
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
        self.m_has_windup_range = windup_range != 0.0;
        self.m_inv_windup_range = if self.m_has_windup_range { 1.0 / windup_range } else { 0.0 };
    }

    /// Get current integral accumulator value (for debugging)
    pub fn integral(&self) -> f64 {
        self.m_integral
    }

    /// Get previous error value (for debugging)
    pub fn previous_error(&self) -> f64 {
        self.m_previous_error
    }

    /// Get current slew rate setting
    pub fn slew_rate(&self) -> f64 {
        self.m_slew_rate
    }

    /// Set slew rate limiting
    pub fn set_slew_rate(&mut self, slew_rate: f64) {
        self.m_slew_rate = slew_rate;
    }

    /// Get previous output value (for debugging)
    pub fn previous_output(&self) -> Option<f64> {
        self.m_previous_output
    }

    pub fn update(&mut self, error: f64) -> f64 {
        // Finding time delta
        let now = Instant::now();

        // Optimized time delta calculation
        let dt_secs = if let Some(prev) = self.m_prev_time {
            now.duration_since(prev).as_secs_f64()
        } else {
            0.0  // Zero on first call
        };

        self.m_prev_time = Some(now);

        // Calculate derivative with optimized division check
        let derivative = if dt_secs > f64::EPSILON {
            (error - self.m_previous_error) / dt_secs
        } else {
            0.0
        };
        
        // Store current error for next iteration
        let prev_error = self.m_previous_error;
        self.m_previous_error = error;

        // Accumulate integral (error * time)
        self.m_integral += error * dt_secs;

        // Reset integral on sign flip (if enabled) - optimized comparison
        if self.m_sign_flip_reset && ((error > 0.0) != (prev_error > 0.0)) && (error * prev_error < 0.0) {
            self.m_integral = 0.0;
        }
    
        // Anti-windup using pre-calculated flag to avoid repeated comparisons
        if self.m_has_windup_range && error.abs() > self.m_windup_range {
            self.m_integral = 0.0;
        }

        // Calculate output using fused multiply-add pattern for better performance
        let proportional = error * self.m_gains.k_p;
        let integral = self.m_integral * self.m_gains.k_i;
        let derivative_term = derivative * self.m_gains.k_d;
        
        let mut output = proportional + integral + derivative_term;
        
        // Apply slew rate limiting if enabled
        if self.m_slew_rate > 0.0 {
            if let Some(prev_output) = self.m_previous_output {
                let max_change = self.m_slew_rate * dt_secs;
                let change = output - prev_output;
                
                if change.abs() > max_change {
                    output = prev_output + change.signum() * max_change;
                }
            }
        }
        
        self.m_previous_output = Some(output);
        output
    }   

    pub fn reset(&mut self) {
        self.m_previous_error = 0.0;
        self.m_integral = 0.0;
        self.m_previous_output = None;
        self.m_prev_time = None;
    }
}