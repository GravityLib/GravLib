// src/GravLib/drivebase/chassis.rs

use crate::GravLib::actuator::motor_group::MotorGroup;
use vexide::devices::controller::Controller;

/// A simple drivetrain which owns two MotorGroups.
pub struct Drivetrain {
    pub left_motors: MotorGroup,
    pub right_motors: MotorGroup,
    pub track_width: f64,
    pub wheel_diameter: f64,
}

impl Drivetrain {
    /// Create a new drivetrain from two motor groups and your geometry.
    pub fn new(
        left_motors: MotorGroup,
        right_motors: MotorGroup,
        track_width: f64,
        wheel_diameter: f64,
    ) -> Self {
        Self {
            left_motors,
            right_motors,
            track_width,
            wheel_diameter,
        }
    }
}

/// A chassis wrapper around the drivetrain (you can add curves, sensors, etc. here).
pub struct Chassis {
    pub drivetrain: Drivetrain,
}

impl Chassis {
    /// Construct a new Chassis by moving in your two MotorGroups.
    pub fn new(
        left_motors: MotorGroup,
        right_motors: MotorGroup,
        track_width: f64,
        wheel_diameter: f64,
    ) -> Self {
        let drivetrain = Drivetrain::new(left_motors, right_motors, track_width, wheel_diameter);
        Chassis { drivetrain }
    }

    /// A simple “tank” drive: map left stick Y to left motors, right stick Y to right motors.
    pub fn tank(&mut self, controller: &Controller) {
        // Read the latest controller state (unwrap_or_default avoids panicking on first call)
        let state = controller.state().unwrap_or_default();

        // Raw stick values (–max..+max) which convert into your Voltage type
        let left_signal  = state.left_stick.y_raw();
        let right_signal = state.right_stick.y_raw();

        // Send those voltages to the motors
        self.drivetrain
            .left_motors
            .move_voltage(left_signal.into());
        self.drivetrain
            .right_motors
            .move_voltage(right_signal.into());
    }
}
