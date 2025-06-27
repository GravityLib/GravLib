use crate::GravLib::actuator::MotorGroup;
use super::Drivetrain;

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
}
