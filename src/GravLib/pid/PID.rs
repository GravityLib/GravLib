use vexide::time::Instant;
use core::time::Duration;

struct Gains {
    kP: f32,
    kI: f32,
    kD: f32,
}

pub struct PID {
    m_gains: Gains,

    m_signFlipReset: bool,
    m_windupRange: f32,

    m_previousError: f32,
    m_integral: f32,

    m_prevTime: Option<Instant>
}

impl PID {
    pub fn new(kP: f32, kI: f32, kD: f32, windupRange: f32, signFlipReset: bool) -> Self {
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

    // fn setWindupRange(&mut self, windupRange: f32) {
    //     self.m_windupRange = windupRange;
    // }

    // fn getWindupRange(&self) -> f32 {
    //     self.m_windupRange
    // }

    pub fn update(&mut self, error: f32) -> f32 {
        // Finding time delta
        let now = Instant::now();

        // TODO - Check algorithm for mapping of prev time
        let dt = if let Some(prev) = self.m_prevTime {
            now.duration_since(prev)  // compute elapsed time
        } else {
            Duration::ZERO           // default to zero on the first call
        };

        self.m_prevTime = Some(now);

        let dt_secs = dt.as_secs_f32();

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