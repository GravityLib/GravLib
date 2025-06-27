
use crate::GravLib::drivebase::Chassis;
use vexide::devices::controller::Controller;

impl Chassis {
    pub fn single_arcade_drive(&mut self, controller: &Controller) {
        let s = controller.state().unwrap_or_default();
        let mut speed = s.left_stick.y_raw() as f64 * (12.0 / 127.0);
        let turn = s.right_stick.x_raw() as f64 * (12.0 / 127.0);

        // Adjust speed based on the turn input
        speed += turn;

        self.drivetrain.left_motors.move_voltage(speed.into());
        self.drivetrain.right_motors.move_voltage(speed.into());
    }
}
