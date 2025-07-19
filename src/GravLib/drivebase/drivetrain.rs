use crate::GravLib::actuator::MotorGroup;

/// The raw motor‐&‐geometry bundle.
pub struct Drivetrain {
    pub left_motors:  MotorGroup,
    pub right_motors: MotorGroup,
    pub track_width:   f64,
    pub wheel_diameter: f64,
}

impl Drivetrain {
    pub fn new(
        left:  MotorGroup,
        right: MotorGroup,
        track_width:   f64,
        wheel_diameter: f64,
    ) -> Self {
        Self { left_motors: left, right_motors: right, track_width, wheel_diameter }
    }
}
