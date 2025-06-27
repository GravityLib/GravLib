use crate::GravLib::actuator::motor_group::MotorGroup;
use vexide::devices::controller::Controller;
use vexide::io::println;


/**
 * TODO: 
 * [] - Add
 */

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

    pub fn tank_drive(&mut self, controller: &Controller) {
        // Read the latest controller state (unwrap_or_default avoids panicking on first call)
        let state = controller.state().unwrap_or_default();

        // Raw stick values (–max..+max) which convert into your Voltage type
        let mut left_signal  = state.left_stick.y_raw() as f64;
        let mut right_signal = state.right_stick.y_raw() as f64;
        
        // conversion from -127, 127 input to -12, 12 voltage input
        left_signal = left_signal as f64 * (12.0/127.0);
        right_signal = right_signal as f64 * (12.0/127.0);

        // Send those voltages to the motors
        self.drivetrain
            .left_motors
            .move_voltage(left_signal.into());
        self.drivetrain
            .right_motors
            .move_voltage(right_signal.into());
    }

    pub fn split_arcade_drive(&mut self, controller: &Controller) {
        // Read the latest controller state (unwrap_or_default avoids panicking on first call)
        let state = controller.state().unwrap_or_default();

        // Raw stick values (–max..+max) which convert into your Voltage type
        let mut left_signal  = state.left_stick.y_raw() as f64;
        let mut right_signal = state.right_stick.x_raw() as f64;
        
        // conversion from -127, 127 input to -12, 12 voltage input
        left_signal = left_signal as f64 * (12.0/127.0);
        right_signal = right_signal as f64 * (12.0/127.0);

        self.drivetrain
            .left_motors
            .move_voltage(left_signal.into());
        self.drivetrain
            .right_motors
            .move_voltage(right_signal.into());
    }

    // pub fn curvature_drive(&mut self, controller: &Controller) {
    //     let state = controller.state().unwrap_or_default();
    //     let forward = state.left_stick.y_raw() as f64 / 127.0;
    //     let curvature = state.right_stick.x_raw() as f64 / 127.0;

    //     let scaled_turn = forward.abs() * curvature;
    //     let left_output = (forward + scaled_turn) * 127.0;
    //     let right_output = (forward - scaled_turn) * 127.0;

    //     self.drivetrain.left_motors.move_voltage(left_output.clamp(-127.0, 127.0));
    //     self.drivetrain.right_motors.move_voltage(right_output.clamp(-127.0, 127.0));
    // }

    pub fn single_arcade_drive(&mut self, controller: &Controller) {
        // Read the latest controller state (unwrap_or_default avoids panicking on first call)
        let state = controller.state().unwrap_or_default();

        // Raw stick values (–max..+max) which convert into your Voltage type
        let mut left_signal  = state.left_stick.y_raw() as f64;
        let mut right_signal = state.left_stick.x_raw() as f64;
        
        // conversion from -127, 127 input to -12, 12 voltage input
        left_signal = left_signal as f64 * (12.0/127.0);
        right_signal = right_signal as f64 * (12.0/127.0);

        self.drivetrain.left_motors.move_voltage(left_signal.into());
        self.drivetrain.right_motors.move_voltage(right_signal.into());
    }
}
