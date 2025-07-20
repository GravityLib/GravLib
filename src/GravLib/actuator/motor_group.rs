// motorgroup.rs
extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use vexide::devices::adi::motor;
use core::time::Duration;
use spin::Mutex;
use vexide::prelude::{BrakeMode, Gearset, Motor};
use vexide::task;
use libm;

pub struct MotorGroup {
    inner: &'static Mutex<Inner>,
}

struct Inner {
    motors: Vec<Motor>,
}

impl Inner{
    fn new(motors: Vec<Motor>) -> Self {
        Self { motors }
    }

    fn move_voltage(&mut self, voltage: f64) {
        for motor in &mut self.motors {
            let _ = motor.set_voltage(voltage); 
        }
    }

    // @dev_note: set_velocity method is built in PID by VEXIDE devs.
    fn move_velocity(&mut self, velocity_percentage: f64) {
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
                (velocity_percentage / 100.0)
                * (max_rpm as f64);

            let velocity = libm::round(velocity_raw) as i32;
            let _ = motor.set_velocity(velocity);
        }
    }

    fn voltage(&self) -> f64 {
        let mut total = 0.0;

        for motor in &self.motors {
            if let Ok(voltage) = motor.voltage() {
                total += voltage;
            }
        }
        total / self.motors.len() as f64
    }
    
    fn position(&self) -> f64 {
        let mut total = 0.0;
        for motor in &self.motors {
            if let Ok(angle) = motor.position() {
                total += angle.as_radians();
            }
        }
        total
    }
    

    fn brake(&mut self, mode: BrakeMode) {
        for motor in self.motors.iter_mut() {
            let _ = motor.brake(mode);
        }
    }
}


impl MotorGroup {
    pub fn new(motors: Vec<Motor>) -> Self {
        let boxed = Box::new(Mutex::new(Inner::new(motors)));
        let static_mutex = Box::leak(boxed); // leak the Box to get a static reference

        let handle: MotorGroup = MotorGroup {inner: static_mutex}; 
        // task::spawn(Self::tracking_task(handle.inner)).detach(); // TODO - optional, useable in chasiss, spawn background tracking task

        handle
    }

    pub fn move_voltage(&self, voltage: f64) {
        let mut guard = self.inner.lock();
        guard.move_voltage(voltage);
    }

    pub fn move_velocity(&self, velocity_percentage: f64) {
        let mut guard = self.inner.lock();
        guard.move_velocity(velocity_percentage);
    }

    pub fn voltage(&self) -> f64 {
        let guard = self.inner.lock();
        guard.voltage()
    }

    pub fn position(&self) -> f64 {
        let guard = self.inner.lock();
        guard.position()
    }

    pub fn brake(&self, mode: BrakeMode) {
        let mut guard = self.inner.lock();
        guard.brake(mode);
    }
}












// _________________________________________ //





// pub struct motorGroup {
//     motors: Vec<Motor>
// }


// impl motorGroup {
//     pub fn new(motors: Vec<Motor>) -> Self {
//         Self { motors }
//     }

//     pub fn set_voltage(&mut self, voltage: f64) {
//         for motor in self.motors.iter_mut() {
//             let _ = motor.set_voltage(voltage);
//         }
//     }
    
//     pub fn voltage(&self) -> f64 {
//         let mut total = 0.0;

//         for motor in &self.motors {
//             if let Ok(voltage) = motor.voltage() {
//                 total += voltage;
//             }
//         }
//         total / self.motors.len() as f64
//     }


//     pub fn position(&self) -> f64 {
//         let mut total = 0.0;
//         for motor in &self.motors {
//             if let Ok(angle) = motor.position() {
//                 total += angle.as_radians();
//             }
//         }
//         total
//     }

//         // @dev_note: set_velocity method is built in PID by VEXIDE devs.
//     pub fn set_velocity(&mut self, velocity_percentage: f64) {
//         // Calculate velocity as percentage of max velocity

//         for motor in self.motors.iter_mut() {
//             let gearset = motor.gearset().unwrap();

//             let max_rpm = match gearset {
//                 Gearset::Red   => 100,
//                 Gearset::Green => 200,
//                 Gearset::Blue  => 600,
//             };

//             // Convert percentages to rpm
//             let velocity_raw = 
//                 (velocity_percentage / 100.0)
//                 * (max_rpm as f64);

//             let velocity = libm::round(velocity_raw) as i32;
//             let _ = motor.set_velocity(velocity);
//         }
//     }

//     pub fn brake(&mut self, mode: BrakeMode) {
//         for motor in self.motors.iter_mut() {
//             let _ = motor.brake(mode);
//         }
//     }
// }