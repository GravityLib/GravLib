// motorgroup.rs
extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use vexide::devices::adi::motor;
use core::time::Duration;
use spin::RwLock;
use vexide::prelude::{BrakeMode, Gearset, Motor};
use vexide::task;
use libm;

pub struct MotorGroup {
    inner: &'static RwLock<Inner>,
}

struct Inner {
    motors: Vec<Motor>,
}

impl Inner{
    fn new(motors: Vec<Motor>) -> Self {
        Self { motors }
    }

    fn move_voltage(&mut self, voltage: f64) {
        // Batch voltage setting to reduce individual I/O calls
        for motor in &mut self.motors {
            let _ = motor.set_voltage(voltage); 
        }
    }

    // Optimized method that takes a pre-calculated voltage array
    fn move_voltage_batch(&mut self, voltages: &[f64]) {
        let len = core::cmp::min(voltages.len(), self.motors.len());
        for i in 0..len {
            let _ = self.motors[i].set_voltage(voltages[i]);
        }
    }

    // @dev_note: set_velocity method is built in PID by VEXIDE devs.
    fn move_velocity(&mut self, velocity_percentage: f64) {
        // Pre-calculate velocity to avoid repeated calculations
        let velocity_factor = velocity_percentage * 0.01; // More efficient than division by 100.0

        for motor in self.motors.iter_mut() {
            let gearset = motor.gearset().unwrap();

            // Use const values for better performance
            let max_rpm = match gearset {
                Gearset::Red   => 100.0,
                Gearset::Green => 200.0,
                Gearset::Blue  => 600.0,
            };

            // Optimized calculation - avoid libm::round for better performance
            let velocity = (velocity_factor * max_rpm + 0.5) as i32;
            let _ = motor.set_velocity(velocity);
        }
    }

    // Optimized batch velocity setting
    fn move_velocity_batch(&mut self, velocity_percentages: &[f64]) {
        let len = core::cmp::min(velocity_percentages.len(), self.motors.len());
        
        for i in 0..len {
            let motor = &mut self.motors[i];
            let gearset = motor.gearset().unwrap();
            let velocity_factor = velocity_percentages[i] * 0.01;

            let max_rpm = match gearset {
                Gearset::Red   => 100.0,
                Gearset::Green => 200.0,
                Gearset::Blue  => 600.0,
            };

            let velocity = (velocity_factor * max_rpm + 0.5) as i32;
            let _ = motor.set_velocity(velocity);
        }
    }

    fn voltage(&self) -> f64 {
        let mut total = 0.0;
        let mut count = 0;

        // Only divide once and handle potential division by zero
        for motor in &self.motors {
            if let Ok(voltage) = motor.voltage() {
                total += voltage;
                count += 1;
            }
        }
        
        if count > 0 {
            total / count as f64
        } else {
            0.0
        }
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
        let boxed = Box::new(RwLock::new(Inner::new(motors)));
        let static_rwlock = Box::leak(boxed); // leak the Box to get a static reference

        let handle: MotorGroup = MotorGroup {inner: static_rwlock}; 
        // task::spawn(Self::tracking_task(handle.inner)).detach(); // TODO - optional, useable in chasiss, spawn background tracking task

        handle
    }

    pub fn move_voltage(&self, voltage: f64) {
        let mut guard = self.inner.write();
        guard.move_voltage(voltage);
    }

    pub fn move_velocity(&self, velocity_percentage: f64) {
        let mut guard = self.inner.write();
        guard.move_velocity(velocity_percentage);
    }

    // New batch methods for better performance
    pub fn move_voltage_batch(&self, voltages: &[f64]) {
        let mut guard = self.inner.write();
        guard.move_voltage_batch(voltages);
    }

    pub fn move_velocity_batch(&self, velocity_percentages: &[f64]) {
        let mut guard = self.inner.write();
        guard.move_velocity_batch(velocity_percentages);
    }

    pub fn voltage(&self) -> f64 {
        let guard = self.inner.read();
        guard.voltage()
    }

    pub fn position(&self) -> f64 {
        let guard = self.inner.read();
        guard.position()
    }

    pub fn brake(&self, mode: BrakeMode) {
        let mut guard = self.inner.write();
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