use vexide::time::Instant;
use core::time::Duration;

pub struct Gains {
    kP: f64,
    kI: f64,
    kD: f64,
}

pub struct PID {
    m_gains: Gains,

    m_signFlipReset: bool,
    m_windupRange: f64,

    m_previousError: f64,
    m_integral: f64,

    m_prevTime: Option<Instant>
}

impl PID {
    pub fn new(kP: f64, kI: f64, kD: f64, windupRange: f64, signFlipReset: bool) -> Self {
        Self {
            m_gains: Gains {
                kP,
                kI,
                kD,
            },
            m_windupRange: windupRange,
            m_signFlipReset: signFlipReset,
            m_previousError: 0.0,
            m_integral: 0.0,
            m_prevTime: None,
        }
    }

    // fn getGains(&self) -> Gains {
    //     self.m_gains
    // }

    // fn setGains(&mut self, gains: Gains) {
    //     self.m_gains = gains;
    // }

    // fn getSignFlipReset(&self) -> bool {
    //     self.signFlipReset
    // }

    // fn setSignFlipReset(&mut self, signFlipReset: bool) {
    //     self.signFlipReset = signFlipReset;
    // }

    // fn setWindupRange(&mut self, windupRange: f64) {
    //     self.m_windupRange = windupRange;
    // }

    // fn getWindupRange(&self) -> f64 {
    //     self.m_windupRange
    // }

    pub fn update(&mut self, error: f64) -> f64 {
        // Finding time delta
        let now = Instant::now();

        // TODO - Check algorithm for mapping of prev time
        let dt = if let Some(prev) = self.m_prevTime {
            now.duration_since(prev)  // compute elapsed time
        } else {
            Duration::ZERO           // default to zero on the first call
        };

        self.m_prevTime = Some(now);

        let dt_secs = dt.as_secs_f64();

        let derivative = if dt_secs > 0.0 {
            (error - self.m_previousError) / dt_secs
        } else {
            0.0
        };
        
        self.m_previousError = error;

        // accumulate integral (error * time)
        self.m_integral += error * dt_secs;

        // reset integral on sign flip (if enabled)
        if self.m_signFlipReset && (error.signum() != self.m_previousError.signum()) {
            self.m_integral = 0.0;
        }
    
        // anti windup range. Unless error is small enough, set the integral to 0
        if  error.abs() > self.m_windupRange && self.m_windupRange != 0.0 {
            self.m_integral = 0.0;
        }

        // calculate output
        error * self.m_gains.kP
            + self.m_integral * self.m_gains.kI
            + derivative * self.m_gains.kD
    }   

    pub fn reset(&mut self) {
        self.m_previousError = 0.0;
        self.m_integral = 0.0;
    }
}