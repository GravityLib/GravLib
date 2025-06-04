extern crate alloc;

use alloc::vec::Vec;
use vexide::prelude::{BrakeMode, Gearset, Motor};
use crate::mission_control::commander::Commander;

use uom::si::{angular_velocity::*, f64::AngularVelocity, velocity};
use libm::{roundf, round};  

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum motorState {
    Moving,
    Brake,
    Idle
}

pub struct motorGroup {
    motors: Vec<Motor>,
    motor_state: motorState,
    state_velocity: f64
}


impl motorGroup {
    pub fn new(houston: Commander, motors: Vec<Motor>) -> Self {

        let group = motorGroup {
            motors, 
            motor_state: motorState::Idle,
            state_velocity: 0.0,
        };

        houston.add_motor_group_item(&group);

        Self { motors, motor_state: motorState::Idle, state_velocity: 0.0,}
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_voltage(voltage);
        }
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

    pub fn change_state(&mut self, velocity_percentage: f64, brakemode: BrakeMode) {
        // TODO - Send signal to mission_control
    }

    pub fn get_state(&self) -> motorState {
        self.motor_state
    }

    pub fn get_state_velocity(&self) -> f64 {
        self.state_velocity
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

}