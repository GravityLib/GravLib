extern crate alloc;

use crate::GravLib::actuator::MotorGroup;
use vexide::devices::smart::rotation::RotationSensor;
use vexide::devices::smart::imu::InertialSensor;

use spin::Mutex;
use alloc::boxed::Box;
use core::f64::consts::PI;

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

pub struct OdomSensors {
    pub vertical1: TrackingWheel,
    pub vertical2:  TrackingWheel,
    pub horizontal1: TrackingWheel,
    pub horizontal2: TrackingWheel,
    pub imu: InertialSensor,
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
        self.encoder.reset_position();
    }

    fn get_distance_travelled(&self) -> f64 {
        self.encoder
            .position()
            .map_or(0.0, |pos| {
                // convert raw ticks → revolutions
                let revs = pos.as_revolutions();
                // cast ints → f64
                let wheel_d = self.wheel_diameter;
                let gear    = self.gear_ratio;
                // distance = revs * (π * diameter) / gear_ratio
                revs * (PI * wheel_d) / gear
            })
    }

    fn get_offset(&self) -> f64 {
        self.offset
    }

}
