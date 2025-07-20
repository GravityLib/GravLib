use crate::GravLib::drivebase::Chassis;
use vexide::devices::controller::Controller;

// Pre-calculated constant for better performance
const VOLTAGE_SCALE: f64 = 12.0 / 127.0;

impl Chassis {
    #[inline(always)]
    pub fn tank_drive(&mut self, controller: &Controller) {
        let s = controller.state().unwrap_or_default();
        
        // Optimized voltage calculation using pre-calculated constant
        let left_voltage = (s.left_stick.y_raw() as f64) * VOLTAGE_SCALE;
        let right_voltage = (s.right_stick.y_raw() as f64) * VOLTAGE_SCALE;
        
        // Batch voltage commands for potentially better performance
        self.drivetrain.left_motors.move_voltage(left_voltage);
        self.drivetrain.right_motors.move_voltage(right_voltage);
    }
}
