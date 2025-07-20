use vexide::time::Instant;
use core::time::Duration;

/// Represents the current state of an exit condition
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExitConditionState {
    /// The condition is not yet met
    NotMet,
    /// The condition has been satisfied
    Met,
    /// The condition timed out without being met
    Timeout,
}

/// A single exit condition with threshold and timeout parameters
#[derive(Debug, Clone)]
pub struct ExitCondition {
    threshold: f64,
    timeout: Duration,
    start_time: Option<Instant>,
    condition_met_start: Option<Instant>,
}

impl ExitCondition {
    /// Create a new exit condition with the specified threshold and timeout
    pub fn new(threshold: f64, timeout_ms: u64) -> Self {
        Self {
            threshold,
            timeout: Duration::from_millis(timeout_ms),
            start_time: None,
            condition_met_start: None,
        }
    }

    /// Update the exit condition with a new value and return its state
    pub fn update(&mut self, current_value: f64) -> ExitConditionState {
        let now = Instant::now();
        
        // Initialize start time if first update
        if self.start_time.is_none() {
            self.start_time = Some(now);
        }

        let within_threshold = current_value.abs() <= self.threshold;
        
        if within_threshold {
            // Start timing if we just entered the threshold
            if self.condition_met_start.is_none() {
                self.condition_met_start = Some(now);
            } else if now.duration_since(self.condition_met_start.unwrap()) >= self.timeout {
                // Condition has been met for long enough
                return ExitConditionState::Met;
            }
        } else {
            // Reset the condition timer if we're outside threshold
            self.condition_met_start = None;
        }

        ExitConditionState::NotMet
    }

    /// Reset the exit condition to its initial state
    pub fn reset(&mut self) {
        self.start_time = None;
        self.condition_met_start = None;
    }

    /// Get the current threshold value
    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    /// Get the timeout duration
    pub fn timeout(&self) -> Duration {
        self.timeout
    }
}

/// A collection of exit conditions for motion control
/// Typically includes small error, large error, and optionally velocity conditions
#[derive(Debug, Clone)]
pub struct ExitConditions {
    pub small_error: ExitCondition,
    pub large_error: ExitCondition,
    pub velocity: Option<ExitCondition>,
}

impl ExitConditions {
    /// Create new exit conditions with small and large error thresholds
    pub fn new(
        small_threshold: f64,
        small_timeout_ms: u64,
        large_threshold: f64,
        large_timeout_ms: u64,
    ) -> Self {
        Self {
            small_error: ExitCondition::new(small_threshold, small_timeout_ms),
            large_error: ExitCondition::new(large_threshold, large_timeout_ms),
            velocity: None,
        }
    }

    /// Add a velocity-based exit condition
    pub fn with_velocity(mut self, threshold: f64, timeout_ms: u64) -> Self {
        self.velocity = Some(ExitCondition::new(threshold, timeout_ms));
        self
    }

    /// Check if any exit condition is met
    /// Returns true if the motion should exit
    pub fn should_exit(&mut self, error: f64, velocity: Option<f64>) -> bool {
        // Check small error condition
        let small_met = self.small_error.update(error) == ExitConditionState::Met;
        
        // Check large error condition  
        let large_met = self.large_error.update(error) == ExitConditionState::Met;
        
        // Check velocity condition if provided
        let velocity_met = if let (Some(vel_condition), Some(vel)) = (&mut self.velocity, velocity) {
            vel_condition.update(vel) == ExitConditionState::Met
        } else {
            false
        };

        small_met || large_met || velocity_met
    }

    /// Reset all exit conditions
    pub fn reset(&mut self) {
        self.small_error.reset();
        self.large_error.reset();
        if let Some(ref mut vel) = self.velocity {
            vel.reset();
        }
    }
}

impl Default for ExitConditions {
    fn default() -> Self {
        Self::new(
            1.0,  // 1 inch small error threshold
            100,  // 100ms small error timeout
            3.0,  // 3 inch large error threshold 
            500,  // 500ms large error timeout
        )
    }
}