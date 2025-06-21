use alloc::vec::Vec;
use alloc::vec;
use vexide::devices::adi::encoder::{self, AdiEncoder};
use vexide::devices::smart::{Motor, RotationSensor};
use vexide::prelude::*;

#[derive(Debug)]
pub enum TrackingSource {
    Encoder(AdiEncoder),
    Rotation(RotationSensor),
    MotorGroup(Vec<Motor>, f64), // motors + rpm
}

pub struct TrackingWheel {
    pub source: TrackingSource,
    pub diameter: f64,
    pub offset: f64,
    pub gear_ratio: f64,
}

impl TrackingWheel {
    pub fn new_encoder(encoder: AdiEncoder, diameter: f64, offset: f64, gear_ratio: f64) -> Self {
        Self {
            source: TrackingSource::Encoder(encoder),
            diameter,
            offset,
            gear_ratio,
        }
    }

    pub fn new_rotation(rotation: RotationSensor, diameter: f64, offset: f64, gear_ratio: f64) -> Self {
        Self {
            source: TrackingSource::Rotation(rotation),
            diameter,
            offset,
            gear_ratio,
        }
    }

    pub fn new_motor_group(motors: Vec<Motor>, diameter: f64, offset: f64, rpm: f64) -> Self {
        Self {
            source: TrackingSource::MotorGroup(motors, rpm),
            diameter,
            offset,
            gear_ratio: 1.0,
        }
    }

    pub fn reset(&mut self) {
        match &mut self.source {
            TrackingSource::Encoder(enc) => {
                enc.reset_position();
            },
            TrackingSource::Rotation(rot) => {
                rot.reset_position();
            },
            TrackingSource::MotorGroup(motors, _) => {
                for motor in motors {
                    motor.reset_position();
                }
            },
        }
    }

    pub fn get_distance(&self) -> f64 {
        match &self.source {
            TrackingSource::Encoder(enc) => {
                let ticks = enc.position().unwrap_or_default().as_ticks(360);
                (ticks as f64 * self.diameter * core::f64::consts::PI / 360.0) / self.gear_ratio
            },
            TrackingSource::Rotation(rot) => {
                let position = rot.position().unwrap_or(Position::from_ticks(0, 36000));
                let ticks = position.as_ticks(36000) as f64;
                (ticks * self.diameter * core::f64::consts::PI / 36000.0) / self.gear_ratio
            },
            TrackingSource::MotorGroup(motors, rpm) => {
                let mut distances = vec![];
                for motor in motors {
                    let position = motor.position().unwrap_or(Position::from_ticks(0, 360));
                    // Default to Green gearset RPM since Motor does not expose gearing info
                    let in_rpm = 200.0;
                    let ticks = position.as_ticks(360) as f64;
                    distances.push(ticks * (self.diameter * core::f64::consts::PI) * (rpm / in_rpm) / 360.0);
                }
                distances.iter().sum::<f64>() / distances.len().max(1) as f64
            },
        }
    }

    pub fn get_offset(&self) -> f64 {
        self.offset
    }

    pub fn get_type(&self) -> u8 {
        match &self.source {
            TrackingSource::MotorGroup(_, _) => 1,
            _ => 0,
        }
    }
}
