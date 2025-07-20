extern crate alloc;

use crate::GravLib::actuator::MotorGroup;
use vexide::devices::smart::rotation::RotationSensor;
use vexide::devices::smart::imu::InertialSensor;

use spin::Mutex;
use alloc::boxed::Box;
use core::f64::consts::PI;

use super::{Drivetrain, Pose};

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



#[derive(Debug, Clone, Copy)]
pub struct ControllerSettings {
    /// Proportional gain
    pub k_p: f32,
    /// Integral gain
    pub k_i: f32,
    /// Derivative gain
    pub k_d: f32,
    /// Anti‐windup range: if error magnitude ≤ this, integral term is reset
    pub windup_range: f32,
    /// “Small” error threshold: if error magnitude ≤ this, controller may exit
    pub small_error: f32,
    /// Time (ms) the error must remain within `small_error` before exit
    pub small_error_timeout: f32,
    /// “Large” error threshold: similar exit logic but for a larger bound
    pub large_error: f32,
    /// Time (ms) the error must remain within `large_error` before exit
    pub large_error_timeout: f32,
    /// Maximum change per control step
    pub slew: f32,
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
        k_p:              f32,
        k_i:              f32,
        k_d:              f32,
        windup_range:     f32,
        small_error:      f32,
        small_error_timeout: f32,
        large_error:      f32,
        large_error_timeout: f32,
        slew:             f32,
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
