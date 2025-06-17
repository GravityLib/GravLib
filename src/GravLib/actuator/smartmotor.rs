use crate::GravLib::PID;
use crate::GravLib::actuator::motor_group::MotorGroup;

use vexide::devices::smart::rotation::RotationSensor;

struct SmartMotor {
    actuator: MotorGroup,
    sensor: RotationSensor,
    controller: PID
}

impl SmartMotor {
    pub fn new (actuator: MotorGroup, sensor: RotationSensor, controller: PID) -> Self {
        let controller = PID::new(0.0, 0.0, 0.0, 0.0);
        Self {
            controller,
            sensor,
            actuator
        }
    }

    pub fn movePID(&mut self, target: f64, timeout: f64, acceptable_range: f64, asyncro: bool, debug: bool) -> i32{
        if asyncro {
            // Start thread
            // recall method with same parameters 
            vexide::task::spawn(async move {
                self.movePID(target, timeout, acceptable_range, false, debug);
            });
            return 2; // indicate async operation
        }

        let start_time = vexide::time::now();

        while (true) {
            let mut current_pos = self.sensor.position();
            let mut error = target - current_pos;

            if error.abs() < acceptable_range {
                break; // exit condition -> target reached
            }

            if vexide::time::now() - start_time > timeout {
                break; // exit condition -> timeout reached
            }

            let mut control_signal = self.controller.update(error);

            self.actuator.move_voltage(control_signal);
        
            if debug {
                println!("Current Position: {}, Target: {}, Error: {}, Control Signal: {}", 
                         current_pos, target, error, control_signal);
            }

            vexide::task::sleep(vexide::time::Duration::from_millis(10)); // Sleep to prevent busy-waiting
        }
        self.actuator.brake();
        self.controller.reset(); // @dev_note: CRITICAL!! Resets PID Integral

        return if (target - self.get_rotation()).abs() < 0.01 { 1 } else { 0 };
    }

    pub fn reset(&mut self) {}

    pub fn get_rotation(&self) -> f64 {}
}