use vexide::{
    devices::smart::RotationSensor,
    devices::smart::InertialSensor,
};

use core::f64::consts::PI;
use alloc::{sync::Arc, vec::Vec};
use spin::Mutex;

pub struct Sensors {
    pub horizontal_wheels: Vec<Arc<Mutex<TrackingWheel>>>,
    pub vertical_wheels: Vec<Arc<Mutex<TrackingWheel>>>,
    pub imu: Arc<Mutex<InertialSensor>>,
}

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

        angle_degrees * self.diameter * PI / 360.0
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