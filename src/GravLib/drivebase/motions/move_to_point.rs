use super::motion_controller::{MotionController, angle_error, apply_speed_constraints};
use crate::GravLib::{pid::PID, drivebase::{Pose, exit_conditions::ExitConditions}};
use libm::{cos, sin, atan2, hypot};

/// Parameters for move-to-point motion
#[derive(Debug, Clone)]
pub struct MoveToPointParams {
    /// Target X coordinate in inches
    pub target_x: f64,
    /// Target Y coordinate in inches  
    pub target_y: f64,
    /// Whether to move forwards (true) or backwards (false)
    pub forwards: bool,
    /// Maximum speed (0-127 or voltage)
    pub max_speed: f64,
    /// Minimum speed for motion chaining
    pub min_speed: f64,
    /// Exit conditions for motion completion
    pub exit_conditions: ExitConditions,
    /// Slew rate for smooth acceleration
    pub slew_rate: f64,
    /// Early exit range for smoother motion chaining
    pub early_exit_range: f64,
    /// PID gains for lateral control
    pub lateral_gains: (f64, f64, f64), // (kP, kI, kD)
    /// PID gains for angular control  
    pub angular_gains: (f64, f64, f64), // (kP, kI, kD)
}

impl Default for MoveToPointParams {
    fn default() -> Self {
        Self {
            target_x: 0.0,
            target_y: 0.0,
            forwards: true,
            max_speed: 127.0,
            min_speed: 0.0,
            exit_conditions: ExitConditions::default(),
            slew_rate: 10.0,
            early_exit_range: 0.0,
            lateral_gains: (15.0, 0.0, 0.1), // Tuned for responsive but stable movement
            angular_gains: (2.0, 0.0, 0.1),  // Tuned for quick turning without oscillation
        }
    }
}

impl MoveToPointParams {
    /// Create new move-to-point parameters with target coordinates
    pub fn new(target_x: f64, target_y: f64) -> Self {
        Self {
            target_x,
            target_y,
            ..Default::default()
        }
    }
    
    /// Set movement direction (builder pattern)
    pub fn forwards(mut self, forwards: bool) -> Self {
        self.forwards = forwards;
        self
    }
    
    /// Set maximum speed (builder pattern)
    pub fn max_speed(mut self, max_speed: f64) -> Self {
        self.max_speed = max_speed;
        self
    }
    
    /// Set minimum speed for chaining (builder pattern)
    pub fn min_speed(mut self, min_speed: f64) -> Self {
        self.min_speed = min_speed;
        self
    }
    
    /// Set PID gains for lateral control (builder pattern)
    pub fn lateral_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.lateral_gains = (kp, ki, kd);
        self
    }
    
    /// Set PID gains for angular control (builder pattern)
    pub fn angular_gains(mut self, kp: f64, ki: f64, kd: f64) -> Self {
        self.angular_gains = (kp, ki, kd);
        self
    }
    
    /// Set exit conditions (builder pattern)
    pub fn exit_conditions(mut self, exit_conditions: ExitConditions) -> Self {
        self.exit_conditions = exit_conditions;
        self
    }
}

/// Move-to-point motion controller
/// Implements point-to-point movement while facing the target throughout motion
pub struct MoveToPointController {
    params: MoveToPointParams,
    lateral_pid: PID,
    angular_pid: PID,
    finished: bool,
    close_to_target: bool,
    distance_traveled: f64,
    last_pose: Option<Pose>,
}

impl MotionController for MoveToPointController {
    type Params = MoveToPointParams;
    
    fn new(params: Self::Params) -> Self {
        let lateral_pid = PID::new_with_slew(
            params.lateral_gains.0,
            params.lateral_gains.1,
            params.lateral_gains.2,
            3.0, // windup range
            true, // sign flip reset
            params.slew_rate,
        );
        
        let angular_pid = PID::new_with_slew(
            params.angular_gains.0,
            params.angular_gains.1,
            params.angular_gains.2,
            3.0, // windup range
            true, // sign flip reset
            params.slew_rate,
        );
        
        Self {
            params,
            lateral_pid,
            angular_pid,
            finished: false,
            close_to_target: false,
            distance_traveled: 0.0,
            last_pose: None,
        }
    }
    
    fn update(&mut self, current_pose: Pose) -> (f64, f64) {
        // Update distance traveled for monitoring
        if let Some(last) = self.last_pose {
            self.distance_traveled += current_pose.distance_to(&last);
        }
        self.last_pose = Some(current_pose);
        
        // Calculate distance and angle to target
        let dx = self.params.target_x - current_pose.x();
        let dy = self.params.target_y - current_pose.y();
        let distance_to_target = hypot(dx, dy);
        
        // Calculate angle to target
        let angle_to_target = atan2(dy, dx);
        
        // Adjust robot heading based on direction preference
        let robot_heading = if self.params.forwards {
            current_pose.theta()
        } else {
            current_pose.theta() + core::f64::consts::PI
        };
        
        // Calculate errors
        let angular_error = angle_error(robot_heading, angle_to_target);
        let lateral_error = distance_to_target * cos(angular_error);
        
        // Check if we're close to the target for settling behavior
        if distance_to_target < 7.5 && !self.close_to_target {
            self.close_to_target = true;
        }
        
        // Calculate PID outputs
        let mut lateral_output = self.lateral_pid.update(lateral_error);
        let mut angular_output = if self.close_to_target {
            // Reduce angular correction when close to target for stability
            0.0
        } else {
            self.angular_pid.update(angular_error.to_degrees())
        };
        
        // Prevent moving in wrong direction
        if self.params.forwards && !self.close_to_target {
            lateral_output = lateral_output.max(0.0);
        } else if !self.params.forwards && !self.close_to_target {
            lateral_output = lateral_output.min(0.0);
        }
        
        // Apply speed constraints
        let (final_lateral, final_angular) = apply_speed_constraints(
            lateral_output,
            angular_output,
            self.params.max_speed,
            self.params.min_speed,
        );
        
        // Check exit conditions
        self.finished = self.params.exit_conditions.should_exit(distance_to_target, None);
        
        // Early exit for motion chaining
        if !self.finished && self.params.early_exit_range > 0.0 && distance_to_target <= self.params.early_exit_range {
            self.finished = true;
        }
        
        (final_lateral, final_angular)
    }
    
    fn is_finished(&self) -> bool {
        self.finished
    }
    
    fn reset(&mut self) {
        self.lateral_pid.reset();
        self.angular_pid.reset();
        self.finished = false;
        self.close_to_target = false;
        self.distance_traveled = 0.0;
        self.last_pose = None;
        self.params.exit_conditions.reset();
    }
    
    fn get_error(&self) -> f64 {
        // Return the lateral error as primary error metric
        self.lateral_pid.previous_error()
    }
    
    fn distance_remaining(&self) -> Option<f64> {
        if let Some(pose) = self.last_pose {
            Some(hypot(
                self.params.target_x - pose.x(),
                self.params.target_y - pose.y(),
            ))
        } else {
            None
        }
    }
    
    fn is_ready(&self) -> bool {
        // Check if parameters are valid
        self.params.max_speed > 0.0 && self.params.target_x.is_finite() && self.params.target_y.is_finite()
    }
}