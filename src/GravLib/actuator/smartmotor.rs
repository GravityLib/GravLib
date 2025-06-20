use crate::GravLib::pid::PID::PID;
use crate::GravLib::actuator::motor_group::MotorGroup;

use vexide::devices::smart::rotation::RotationSensor;
use vexide::{devices::{adi::motor, smart}, prelude::*};
use spin::Mutex;
use crate::Box;

/**
 * TODO List:
 *  [] - Test movePID system
 *  [x] - Implement mutex for SmartMotor readings. 
 */

pub struct SmartMotor {
    inner: &'static Mutex<Inner>
}

struct Inner {
    actuator: MotorGroup,
    sensor: RotationSensor,
    controller: PID,
}

impl Inner {
    fn new (actuator: MotorGroup, sensor: RotationSensor, controller: PID) -> Self {
        Self {
            controller,
            sensor,
            actuator
        }
    }

    fn movePID(&mut self, target: f64, timeout: f64, acceptable_range: f64, asyncro: bool, debug: bool) -> i32{
        if asyncro {
            // Start thread
            // recall method with same parameters 
            vexide::task::spawn(async move {
                self.movePID(target, timeout, acceptable_range, false, debug);
            });
            return 2; // indicate async operation
        }

        let start_time = vexide::time::Instant::now();

        while (true) {
            let current_pos = self.sensor.position().unwrap().as_degrees();
            let mut error = target - current_pos;

            if error.abs() < acceptable_range {
                break; // exit condition -> target reached
            }

            if vexide::time::Instant::now().duration_since(start_time).as_secs_f64() > timeout {
                break;
            }


            let mut control_signal = self.controller.update(error as f32);

            self.actuator.move_voltage(control_signal.into());
        
            if debug {
                // println!("Current Position: {}, Target: {}, Error: {}, Control Signal: {}", 
                //          current_pos, target, error, control_signal);
            }

            vexide::task::sleep(vexide::time::Duration::from_millis(10)); // Sleep to prevent busy-waiting
        }
        self.actuator.brake(BrakeMode::Coast);
        self.controller.reset(); // @dev_note: CRITICAL!! Resets PID Integral

        return if (target - self.get_rotation()).abs() < 0.01 { 1 } else { 0 };
    }

    fn reset(&mut self) {
        self.sensor.reset_position();
    }

    fn get_rotation(&self) -> f64 {
        // `position()` returns `Result<Position, PortError>`
        // unwrap panics if there's an error
        let pos = self.sensor.position().unwrap();  
        
        // assuming `Position` is a tuple struct around f64:
        pos.as_degrees()
    }
}

impl SmartMotor {
    pub fn new(actuator: MotorGroup, sensor: RotationSensor, controller: PID) -> Self {
        let boxed = Box::new(Mutex::new(Inner::new(actuator, sensor, controller)));
         
        let static_mutex = Box::leak(boxed); // leak the Box to get a static reference

        let handle: SmartMotor = SmartMotor { inner: static_mutex };

        handle
    }

    pub fn movePID(&self, target: f64, timeout: f64, acceptable_range: f64, asyncro: bool, debug: bool) -> i32 {
        let mut guard = self.inner.lock();
        guard.movePID(target, timeout, acceptable_range, asyncro, debug)
    }

    pub fn reset(&self) {
        let mut guard = self.inner.lock();
        guard.reset();
    }

    pub fn get_rotation(&self) -> f64 {
        let guard = self.inner.lock();
        guard.get_rotation()
    }
}