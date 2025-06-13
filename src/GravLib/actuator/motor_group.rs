extern crate alloc;

use alloc::vec::Vec;
use vexide::prelude::{BrakeMode, Gearset, Motor};

use uom::si::{angular_velocity::*, f64::AngularVelocity, velocity};
use libm::{roundf, round};  

pub struct motorGroup {
    motors: Vec<Motor>
}


impl motorGroup {
    pub fn new(motors: Vec<Motor>) -> Self {
        Self { motors }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_voltage(voltage);
        }
    }
    
    pub fn voltage(&self) -> f64 {
        let mut total = 0.0;

        for motor in &self.motors {
            if let Ok(voltage) = motor.voltage() {
                total += voltage;
            }
        }
        total / self.motors.len() as f64
    }


    pub fn position(&self) -> f64 {
        let mut total = 0.0;
        for motor in &self.motors {
            if let Ok(angle) = motor.position() {
                total += angle.as_radians();
            }
        }
        total
    }

        // @dev_note: set_velocity method is built in PID by VEXIDE devs.
    pub fn set_velocity(&mut self, velocity_percentage: f64) {
        // Calculate velocity as percentage of max velocity

        for motor in self.motors.iter_mut() {
            let gearset = motor.gearset().unwrap();

            let max_rpm = match gearset {
                Gearset::Red   => 100,
                Gearset::Green => 200,
                Gearset::Blue  => 600,
            };

            // Convert percentages to rpm
            let velocity_raw = 
                (velocity_percentage as f32 / 100.0)
                * (max_rpm as f32);

            let velocity = roundf(velocity_raw) as i32;
            let _ = motor.set_velocity(velocity);
        }
    }

    pub fn brake(&mut self, mode: BrakeMode) {
        for motor in self.motors.iter_mut() {
            let _ = motor.brake(mode);
        }
    }
}