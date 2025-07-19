use core::time::Duration;

use crate::GravLib::PID;
use crate::GravLib::actuator::MotorGroup;

use vexide::devices::smart::rotation::RotationSensor;
use vexide::time::{Instant};
use vexide::io::println;
use spin::Mutex;

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
    controller: PID
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
            return 2; // async not supported in Inner, handle at SmartMotor level
        }

        let start_time = Instant::now();

        loop {
            let current_pos = self.sensor.position();
            let error = target - current_pos;

            if error.abs() < acceptable_range {
                break; // exit condition -> target reached
            }

            if Instant::now() - start_time > Duration::from_secs(timeout as u64) {
                break; // exit condition -> timeout reached
            }

            let control_signal = self.controller.update(error);

            self.actuator.move_voltage(control_signal as f64);
        
            if debug {
                println!("Current Position: {}, Target: {}, Error: {}, Control Signal: {}", 
                         current_pos, target, error, control_signal);
            }

            vexide::task::sleep(Duration::from_millis(10)); // Sleep to prevent busy-waiting
        }
        self.actuator.brake();
        self.controller.reset(); // @dev_note: CRITICAL!! Resets PID Integral

        return if (target - self.get_rotation()).abs() < 0.01 { 1 } else { 0 };
    }

    fn move_logic() {}

    fn reset(&mut self) {
        self.sensor.reset_position();
    }

    fn get_rotation(&self) -> f64 {
        self.sensor.position()
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