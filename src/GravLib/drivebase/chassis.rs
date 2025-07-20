extern crate alloc;

use crate::GravLib::actuator::MotorGroup;
use vexide::devices::smart::rotation::RotationSensor;
use spin::Mutex;
use alloc::boxed::Box;
use core::f64::consts::PI;
use vexide::time;

use super::Drivetrain;
use super::task_system::{TaskConfig, configure_sensors};
use super::odometry::OdomSensors;
use super::motions::{
    MotionController, MotionError, MotionResult,
    MoveToPointController, MoveToPointParams,
    MoveToPoseController, MoveToPoseParams
};
use super::{Pose, exit_conditions::ExitConditions};

/// top‐level Chassis type.
/// Holds a `Drivetrain` and exposes all behaviors from
/// the `drive` and `motions` sub‐modules.
pub struct Chassis {
    pub drivetrain: Drivetrain,
}

impl Chassis {
    pub fn new(
        left:  MotorGroup,
        right: MotorGroup,
        track_width:   f64,
        wheel_diameter: f64,
    ) -> Self {
        let drivetrain = Drivetrain::new(left, right, track_width, wheel_diameter);
        Chassis { drivetrain }
    }

    /// Calibrate the chassis and initialize the background odometry task system
    /// This is equivalent to LemLib's calibrate() method but uses async/await
    pub async fn calibrate(&self, sensors: OdomSensors, calibrate_imu: bool) -> Result<(), &'static str> {
        self.calibrate_with_config(sensors, TaskConfig::default(), calibrate_imu).await
    }

    /// Calibrate the chassis with custom configuration
    /// Provides full control over task parameters
    pub async fn calibrate_with_config(&self, mut sensors: OdomSensors, config: TaskConfig, calibrate_imu: bool) -> Result<(), &'static str> {
        // Calibrate IMU if requested and available
        if calibrate_imu {
            if let Some(ref mut imu) = sensors.imu {
                // Calibrate the IMU - this may take several seconds
                let mut attempts = 0;
                const MAX_ATTEMPTS: u8 = 5;
                
                while attempts < MAX_ATTEMPTS {
                    match imu.calibrate().await {
                        Ok(_) => break,
                        Err(_) => {
                            attempts += 1;
                            if attempts >= MAX_ATTEMPTS {
                                return Err("IMU calibration failed");
                            }
                            // Wait before retrying
                            vexide::time::sleep(core::time::Duration::from_millis(100)).await;
                        }
                    }
                }
            }
        }

        // Configure sensors for odometry
        configure_sensors(sensors)?;

        // Start the odometry task if auto-start is enabled
        if config.auto_start {
            super::task_system::start_task()?;
        }

        Ok(())
    }

    /// Move to a specific point asynchronously
    /// This is equivalent to LemLib's moveToPoint() function
    pub async fn move_to_point(
        &self,
        x: f64,
        y: f64,
        timeout_ms: u32,
        params: Option<MoveToPointParams>,
    ) -> Result<MotionResult, MotionError> {
        let motion_params = params.unwrap_or_else(|| {
            MoveToPointParams::new(x, y)
                .max_speed(127.0)
                .exit_conditions(ExitConditions::new(1.0, 100, 3.0, 500))
        });

        let mut controller = MoveToPointController::new(motion_params);
        self.execute_motion(controller, timeout_ms).await
    }

    /// Move to a specific pose (position + heading) asynchronously
    /// This is equivalent to LemLib's moveToPose() function
    pub async fn move_to_pose(
        &self,
        target_pose: Pose,
        timeout_ms: u32,
        params: Option<MoveToPoseParams>,
    ) -> Result<MotionResult, MotionError> {
        let motion_params = params.unwrap_or_else(|| {
            MoveToPoseParams::new(target_pose)
                .max_speed(100.0)
                .boomerang_params(8.0, 0.6)
                .exit_conditions(ExitConditions::new(1.0, 150, 3.0, 750))
        });

        let mut controller = MoveToPoseController::new(motion_params);
        self.execute_motion(controller, timeout_ms).await
    }

    /// Move to specific coordinates with heading
    pub async fn move_to_pose_coords(
        &self,
        x: f64,
        y: f64,
        theta: f64,
        timeout_ms: u32,
        params: Option<MoveToPoseParams>,
    ) -> Result<MotionResult, MotionError> {
        let target_pose = Pose::new(x, y, theta);
        self.move_to_pose(target_pose, timeout_ms, params).await
    }

    /// Generic motion execution function for all motion controllers
    async fn execute_motion<T: MotionController>(
        &self,
        mut controller: T,
        timeout_ms: u32,
    ) -> Result<MotionResult, MotionError> {
        use vexide::time::Instant;
        use core::time::Duration;
        
        let start_time = Instant::now();
        let timeout = Duration::from_millis(timeout_ms as u64);
        
        // Validate controller is ready
        if !controller.is_ready() {
            return Err(MotionError::InvalidParameters);
        }
        
        controller.reset();
        
        loop {
            // Check timeout
            if start_time.elapsed() > timeout {
                // Stop the drivetrain
                let _ = self.drivetrain.left_motors.move_voltage(0.0);
                let _ = self.drivetrain.right_motors.move_voltage(0.0);
                return Ok(MotionResult::Timeout);
            }

            // Get current pose from odometry
            let current_pose = super::odometry::get_pose(true); // Get pose in radians
            
            // Update controller
            let (lateral_output, angular_output) = controller.update(current_pose);
            
            // Check if finished
            if controller.is_finished() {
                let _ = self.drivetrain.left_motors.move_voltage(0.0);
                let _ = self.drivetrain.right_motors.move_voltage(0.0);
                return Ok(MotionResult::Completed);
            }

            // Calculate differential drive outputs
            let left_power = lateral_output + angular_output;
            let right_power = lateral_output - angular_output;
            
            // Apply power scaling to respect voltage limits (12V = 12000mV)
            let max_power = left_power.abs().max(right_power.abs());
            let voltage_scale = if max_power > 12000.0 {
                12000.0 / max_power
            } else if max_power > 0.0 {
                // Scale from 0-127 to 0-12000mV if using percentage values
                if max_power <= 127.0 {
                    12000.0 / 127.0
                } else {
                    1.0
                }
            } else {
                1.0
            };

            // Set motor voltages
            let left_voltage = left_power * voltage_scale;
            let right_voltage = right_power * voltage_scale;
            
            let _ = self.drivetrain.left_motors.move_voltage(left_voltage);
            let _ = self.drivetrain.right_motors.move_voltage(right_voltage);

            // Wait for next iteration (10ms for 100Hz control loop)
            time::sleep(core::time::Duration::from_millis(10)).await;
        }
    }

    /// Stop all motion immediately
    pub fn stop_motion(&self) {
        let _ = self.drivetrain.left_motors.move_voltage(0.0);
        let _ = self.drivetrain.right_motors.move_voltage(0.0);
    }

    /// Get the current robot pose from odometry
    pub fn get_pose(&self) -> Pose {
        super::odometry::get_pose(true) // Return pose in radians
    }

    /// Get the current robot pose in degrees (for convenience)
    pub fn get_pose_degrees(&self) -> Pose {
        super::odometry::get_pose(false) // Return pose in degrees
    }
}



#[derive(Debug, Clone, Copy)]
pub struct ControllerSettings {
    /// Proportional gain
    pub k_p: f64,
    /// Integral gain
    pub k_i: f64,
    /// Derivative gain
    pub k_d: f64,
    /// Anti‐windup range: if error magnitude ≤ this, integral term is reset
    pub windup_range: f64,
    /// “Small” error threshold: if error magnitude ≤ this, controller may exit
    pub small_error: f64,
    /// Time (ms) the error must remain within `small_error` before exit
    pub small_error_timeout: f64,
    /// “Large” error threshold: similar exit logic but for a larger bound
    pub large_error: f64,
    /// Time (ms) the error must remain within `large_error` before exit
    pub large_error_timeout: f64,
    /// Maximum change per control step
    pub slew: f64,
}

impl ControllerSettings {
    /// Construct a new set of controller constants.
    ///
    /// Any field set to `0.0` will be effectively ignored.
    ///
    /// # Parameters
    /// - `k_p`: proportional gain  
    /// - `k_i`: integral gain  
    /// - `k_d`: derivative gain  
    /// - `windup_range`: integral anti‐windup range  
    /// - `small_error`: small‐error exit threshold  
    /// - `small_error_timeout`: small‐error timeout (ms)  
    /// - `large_error`: large‐error exit threshold  
    /// - `large_error_timeout`: large‐error timeout (ms)  
    /// - `slew`: maximum acceleration (slew rate)  
    pub fn new(
        k_p:              f64,
        k_i:              f64,
        k_d:              f64,
        windup_range:     f64,
        small_error:      f64,
        small_error_timeout: f64,
        large_error:      f64,
        large_error_timeout: f64,
        slew:             f64,
    ) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            windup_range,
            small_error,
            small_error_timeout,
            large_error,
            large_error_timeout,
            slew,
        }
    }
}

pub struct TrackingWheel {
    inner: &'static Mutex<TrackingWheelInner>,
}

impl TrackingWheel {
    pub fn new(encoder: RotationSensor, wheel_diameter: f64, offset: f64, gear_ratio: f64) -> Self {
        let inner = Mutex::new(TrackingWheelInner::new(encoder, wheel_diameter, offset, gear_ratio));
        TrackingWheel { inner: Box::leak(Box::new(inner)) }
    }

    pub fn reset(&self) {
        self.inner.lock().reset();
    }

    pub fn get_distance_travelled(&self) -> f64 {
        self.inner.lock().get_distance_travelled()
    }

    pub fn get_offset(&self) -> f64 {
        self.inner.lock().get_offset()
    }
}

struct TrackingWheelInner {
    pub encoder: RotationSensor,
    pub wheel_diameter: f64,
    pub offset: f64,
    pub gear_ratio: f64
}

impl TrackingWheelInner {
    fn new(encoder: RotationSensor, wheel_diameter: f64, offset: f64, gear_ratio: f64) -> Self {
        Self {
            encoder,
            wheel_diameter,
            offset,
            gear_ratio
        }
    }

    fn reset(&mut self) {
        let _ = self.encoder.reset_position();
    }

    fn get_distance_travelled(&self) -> f64 {
        self.encoder
            .position()
            .map_or(0.0, |pos| {
                // convert raw ticks → revolutions
                let revs = pos.as_revolutions();
                // cast ints → f64
                let wheel_d = self.wheel_diameter;
                let gear = self.gear_ratio;
                // distance = revs * (π * diameter) / gear_ratio
                revs * (PI * wheel_d) / gear
            })
    }

    fn get_offset(&self) -> f64 {
        self.offset
    }

}
