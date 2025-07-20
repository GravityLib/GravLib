use crate::GravLib::drivebase::{Pose, exit_conditions::ExitConditions};
use vexide::time::Instant;
use core::time::Duration;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Error types that can occur during motion execution
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionError {
    /// Motion exceeded the specified timeout
    Timeout,
    /// Motion was cancelled by user request
    Cancelled,
    /// Odometry sensors are not ready or configured
    SensorsNotReady,
    /// Invalid motion parameters were provided
    InvalidParameters,
    /// Hardware error occurred during motion
    HardwareError,
}

impl core::fmt::Display for MotionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MotionError::Timeout => write!(f, "Motion timed out"),
            MotionError::Cancelled => write!(f, "Motion was cancelled"),
            MotionError::SensorsNotReady => write!(f, "Odometry sensors not ready"),
            MotionError::InvalidParameters => write!(f, "Invalid motion parameters"),
            MotionError::HardwareError => write!(f, "Hardware error occurred"),
        }
    }
}

/// Result of a completed motion
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionResult {
    /// Motion completed successfully within tolerances
    Completed,
    /// Motion was cancelled before completion
    Cancelled,
    /// Motion timed out before reaching target
    Timeout,
    /// Motion completed with an error
    Error(MotionError),
}

/// Common parameters for motion control
#[derive(Debug, Clone)]
pub struct MotionParams {
    /// Maximum timeout for the motion in milliseconds
    pub timeout_ms: u32,
    /// Maximum speed (0-127 or voltage)
    pub max_speed: f64,
    /// Minimum speed for smoother motion chaining
    pub min_speed: f64,
    /// Slew rate limiting for acceleration control
    pub slew_rate: f64,
    /// Whether to execute the motion asynchronously
    pub async_execution: bool,
    /// Exit conditions for determining motion completion
    pub exit_conditions: ExitConditions,
}

impl Default for MotionParams {
    fn default() -> Self {
        Self {
            timeout_ms: 5000,
            max_speed: 127.0,
            min_speed: 0.0,
            slew_rate: 10.0,
            async_execution: true,
            exit_conditions: ExitConditions::default(),
        }
    }
}

/// Core trait for all motion controllers
/// Each motion algorithm (moveToPoint, moveToPose, etc.) implements this trait
pub trait MotionController {
    /// Parameter type specific to this motion controller
    type Params: Clone;
    
    /// Create a new motion controller with the specified parameters
    fn new(params: Self::Params) -> Self;
    
    /// Update the motion controller with the current robot pose
    /// Returns (lateral_output, angular_output) for differential drive
    fn update(&mut self, current_pose: Pose) -> (f64, f64);
    
    /// Check if the motion has finished (either completed or failed)
    fn is_finished(&self) -> bool;
    
    /// Reset the motion controller to its initial state
    fn reset(&mut self);
    
    /// Get the current error value for monitoring/debugging
    fn get_error(&self) -> f64;
    
    /// Get the distance remaining to target (if applicable)
    fn distance_remaining(&self) -> Option<f64> {
        None
    }
    
    /// Check if motion controller is ready to start
    fn is_ready(&self) -> bool {
        true
    }
}

/// Utility functions for motion control calculations

/// Apply slew rate limiting to prevent sudden acceleration changes
#[inline(always)]
pub fn slew_rate_limit(target: f64, current: f64, max_change: f64, dt: f64) -> f64 {
    if max_change <= 0.0 {
        return target; // No limiting
    }
    
    let max_delta = max_change * dt;
    let delta = target - current;
    
    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

/// Calculate the shortest angle error between current and target angles
/// Returns error in the range [-π, π]
#[inline(always)]
pub fn angle_error(current: f64, target: f64) -> f64 {
    let mut error = target - current;
    
    // Normalize to [-π, π] range
    while error > core::f64::consts::PI {
        error -= 2.0 * core::f64::consts::PI;
    }
    while error < -core::f64::consts::PI {
        error += 2.0 * core::f64::consts::PI;
    }
    
    error
}

/// Apply speed constraints with proper scaling
#[inline(always)]
pub fn apply_speed_constraints(
    lateral: f64,
    angular: f64,
    max_speed: f64,
    min_speed: f64,
) -> (f64, f64) {
    // Calculate the maximum component
    let max_component = lateral.abs().max(angular.abs());
    
    // Scale down if exceeding max speed
    let (mut final_lateral, mut final_angular) = if max_component > max_speed {
        let scale = max_speed / max_component;
        (lateral * scale, angular * scale)
    } else {
        (lateral, angular)
    };
    
    // Apply minimum speed if not zero
    if min_speed > 0.0 {
        if final_lateral.abs() > 0.0 && final_lateral.abs() < min_speed {
            final_lateral = final_lateral.signum() * min_speed;
        }
        if final_angular.abs() > 0.0 && final_angular.abs() < min_speed {
            final_angular = final_angular.signum() * min_speed;
        }
    }
    
    (final_lateral, final_angular)
}

/// Convert lateral and angular velocities to left/right wheel velocities
/// for differential drive systems
#[inline(always)]
pub fn differential_drive_kinematics(
    lateral_velocity: f64,
    angular_velocity: f64,
    track_width: f64,
) -> (f64, f64) {
    let left_velocity = lateral_velocity + (angular_velocity * track_width / 2.0);
    let right_velocity = lateral_velocity - (angular_velocity * track_width / 2.0);
    (left_velocity, right_velocity)
}

/// Convert left/right wheel velocities back to lateral/angular velocities
#[inline(always)]
pub fn inverse_differential_drive_kinematics(
    left_velocity: f64,
    right_velocity: f64,
    track_width: f64,
) -> (f64, f64) {
    let lateral_velocity = (left_velocity + right_velocity) / 2.0;
    let angular_velocity = (left_velocity - right_velocity) / track_width;
    (lateral_velocity, angular_velocity)
}