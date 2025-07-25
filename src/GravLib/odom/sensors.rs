use vexide::{
    devices::smart::RotationSensor,
};

use core::f64::consts::PI;
pub struct TrackingWheel {
    rotation: RotationSensor,
    diameter: f64,
    offset: f64,
    ratio: f64,
}

impl TrackingWheel {
    pub fn new(rotation: RotationSensor, diameter: f64, offset: f64, ratio: f64) -> Self {
        Self {
            rotation,
            diameter,
            offset,
            ratio: if ratio == 0.0 { 1.0 } else { ratio },
        }
    }

    /// Returns the distance travelled in the same units as `diameter`.
    pub fn get_distance_travelled(&self) -> f64 {
        // Assume get_angle() returns the angle in degrees.
        let angle_degrees = self.rotation.position().unwrap().as_degrees();
        // Convert degrees to number of rotations.
        let rotations = angle_degrees / 360.0;
        // Calculate circumference.
        let circumference = PI * self.diameter;
        rotations * circumference * self.ratio
    }

    /// Returns the offset of the tracking wheel.
    pub fn get_offset(&self) -> f64 {
        self.offset
    }

    /// Resets the sensor and tracking wheel.
    /// We assume that rotation.reset() resets the internal counter and returns an i64 status code.
    pub fn reset(&mut self) {
        self.rotation.reset_position();
    }
}