use super::motion_controller::{MotionController, angle_error, apply_speed_constraints};
use crate::GravLib::{pid::PID, drivebase::{Pose, exit_conditions::ExitConditions}};
use libm::{cos, sin, atan2, hypot};

/// Parameters for move-to-pose motion using boomerang controller
#[derive(Debug, Clone)]
pub struct MoveToPoseParams {
    /// Target pose (x, y, theta)
    pub target_pose: Pose,
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
    /// Horizontal drift parameter (corner cutting speed)
    pub horizontal_drift: f64,
    /// Lead distance for boomerang controller
    pub lead: f64,
}

impl Default for MoveToPoseParams {
    fn default() -> Self {
        Self {
            target_pose: Pose::new(0.0, 0.0, 0.0),
            forwards: true,
            max_speed: 127.0,
            min_speed: 0.0,
            exit_conditions: ExitConditions::new(
                1.0, 100, // Small error: 1 inch, 100ms
                3.0, 500, // Large error: 3 inches, 500ms
            ).with_velocity(5.0, 100), // Also check velocity
            slew_rate: 10.0,
            early_exit_range: 0.0,
            lateral_gains: (10.0, 0.0, 0.3), // Slightly less aggressive than moveToPoint
            angular_gains: (3.0, 0.0, 0.2),  // More responsive angular control
            horizontal_drift: 8.0, // Default corner cutting speed
            lead: 0.6, // 60% lead distance
        }
    }
}

impl MoveToPoseParams {
    /// Create new move-to-pose parameters with target pose
    pub fn new(target_pose: Pose) -> Self {
        Self {
            target_pose,
            ..Default::default()
        }
    }
    
    /// Create parameters with target coordinates and heading
    pub fn with_target(x: f64, y: f64, theta: f64) -> Self {
        Self {
            target_pose: Pose::new(x, y, theta),
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
    
    /// Set boomerang controller parameters (builder pattern)
    pub fn boomerang_params(mut self, horizontal_drift: f64, lead: f64) -> Self {
        self.horizontal_drift = horizontal_drift;
        self.lead = lead;
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

/// Move-to-pose motion controller using boomerang algorithm
/// Combines translation and rotation for smooth pose control
pub struct MoveToPoseController {
    params: MoveToPoseParams,
    lateral_pid: PID,
    angular_pid: PID,
    finished: bool,
    close_to_target: bool,
    final_turn_phase: bool,
    distance_traveled: f64,
    last_pose: Option<Pose>,
    carrot_point: Option<Pose>,
}

impl MotionController for MoveToPoseController {
    type Params = MoveToPoseParams;
    
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
            final_turn_phase: false,
            distance_traveled: 0.0,
            last_pose: None,
            carrot_point: None,
        }
    }
    
    fn update(&mut self, current_pose: Pose) -> (f64, f64) {
        // Update distance traveled for monitoring
        if let Some(last) = self.last_pose {
            self.distance_traveled += current_pose.distance_to(&last);
        }
        self.last_pose = Some(current_pose);
        
        // Calculate distance to target position
        let dx = self.params.target_pose.x() - current_pose.x();
        let dy = self.params.target_pose.y() - current_pose.y();
        let distance_to_target = hypot(dx, dy);
        
        // Check if we're close enough for final turn phase
        if distance_to_target < 6.0 && !self.close_to_target {
            self.close_to_target = true;
        }
        
        // Check if we should enter final turn phase (very close to position)
        if distance_to_target < 2.0 && !self.final_turn_phase {
            self.final_turn_phase = true;
        }
        
        let (lateral_output, angular_output) = if self.final_turn_phase {
            // Final turn phase: focus on heading accuracy
            let heading_error = angle_error(current_pose.theta(), self.params.target_pose.theta());
            let angular_out = self.angular_pid.update(heading_error.to_degrees());
            
            // Minimal lateral movement to maintain position
            let lateral_out = self.lateral_pid.update(distance_to_target * 0.1);
            
            (lateral_out, angular_out)
        } else {
            // Boomerang controller phase
            self.calculate_boomerang_output(current_pose, distance_to_target)
        };
        
        // Apply speed constraints
        let (final_lateral, final_angular) = apply_speed_constraints(
            lateral_output,
            angular_output,
            self.params.max_speed,
            self.params.min_speed,
        );
        
        // Check exit conditions - consider both position and heading errors
        let heading_error = angle_error(current_pose.theta(), self.params.target_pose.theta()).abs();
        let combined_error = distance_to_target + (heading_error.to_degrees() * 0.1); // Weight heading less
        
        self.finished = self.params.exit_conditions.should_exit(combined_error, None);
        
        // Early exit for motion chaining
        if !self.finished && self.params.early_exit_range > 0.0 && combined_error <= self.params.early_exit_range {
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
        self.final_turn_phase = false;
        self.distance_traveled = 0.0;
        self.last_pose = None;
        self.carrot_point = None;
        self.params.exit_conditions.reset();
    }
    
    fn get_error(&self) -> f64 {
        // Return combined position and heading error
        if let Some(pose) = self.last_pose {
            let position_error = pose.distance_to(&self.params.target_pose);
            let heading_error = angle_error(pose.theta(), self.params.target_pose.theta()).abs();
            position_error + (heading_error.to_degrees() * 0.1)
        } else {
            0.0
        }
    }
    
    fn distance_remaining(&self) -> Option<f64> {
        if let Some(pose) = self.last_pose {
            Some(pose.distance_to(&self.params.target_pose))
        } else {
            None
        }
    }
    
    fn is_ready(&self) -> bool {
        // Check if parameters are valid
        self.params.max_speed > 0.0 && 
        self.params.target_pose.x().is_finite() && 
        self.params.target_pose.y().is_finite() &&
        self.params.target_pose.theta().is_finite()
    }
}

impl MoveToPoseController {
    /// Calculate boomerang controller output
    /// This implements the sophisticated curve-following algorithm used by LemLib
    fn calculate_boomerang_output(&mut self, current_pose: Pose, distance_to_target: f64) -> (f64, f64) {
        // Calculate carrot point (intermediate target ahead of robot)
        let target_x = self.params.target_pose.x();
        let target_y = self.params.target_pose.y();
        
        // Calculate the direction from robot to target
        let dx = target_x - current_pose.x();
        let dy = target_y - current_pose.y();
        let angle_to_target = atan2(dy, dx);
        
        // Calculate lead distance based on current distance and lead parameter
        let lead_distance = distance_to_target * self.params.lead;
        
        // Calculate carrot point position
        let carrot_x = current_pose.x() + lead_distance * cos(angle_to_target);
        let carrot_y = current_pose.y() + lead_distance * sin(angle_to_target);
        
        // Store carrot point for debugging
        self.carrot_point = Some(Pose::new(carrot_x, carrot_y, 0.0));
        
        // Calculate robot's heading relative to carrot point
        let robot_heading = if self.params.forwards {
            current_pose.theta()
        } else {
            current_pose.theta() + core::f64::consts::PI
        };
        
        // Calculate errors relative to carrot point
        let carrot_dx = carrot_x - current_pose.x();
        let carrot_dy = carrot_y - current_pose.y();
        let carrot_distance = hypot(carrot_dx, carrot_dy);
        let angle_to_carrot = atan2(carrot_dy, carrot_dx);
        
        let angular_error = angle_error(robot_heading, angle_to_carrot);
        let lateral_error = carrot_distance * cos(angular_error);
        
        // Apply horizontal drift for cornering
        let drift_factor = if self.close_to_target {
            0.0 // No drift when close
        } else {
            self.params.horizontal_drift * (angular_error.abs() / core::f64::consts::PI)
        };
        
        // Calculate PID outputs
        let mut lateral_output = self.lateral_pid.update(lateral_error);
        let mut angular_output = self.angular_pid.update(angular_error.to_degrees());
        
        // Apply drift compensation
        angular_output += drift_factor * angular_error.signum();
        
        // Prevent moving in wrong direction
        if self.params.forwards && !self.close_to_target {
            lateral_output = lateral_output.max(0.0);
        } else if !self.params.forwards && !self.close_to_target {
            lateral_output = lateral_output.min(0.0);
        }
        
        (lateral_output, angular_output)
    }
    
    /// Get the current carrot point for debugging/visualization
    pub fn get_carrot_point(&self) -> Option<Pose> {
        self.carrot_point
    }
    
    /// Check if controller is in final turn phase
    pub fn is_final_turn_phase(&self) -> bool {
        self.final_turn_phase
    }
}