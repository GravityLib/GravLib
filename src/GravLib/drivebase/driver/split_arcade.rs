use crate::GravLib::drivebase::Chassis;
use vexide::devices::controller::Controller;

impl Chassis {
    pub fn split_arcade_drive(&mut self, controller: &Controller) {
        let s = controller.state().unwrap_or_default();
        let mut l = s.left_stick.y_raw() as f64 * (12.0 / 127.0);
        let mut r = s.right_stick.y_raw() as f64 * (12.0 / 127.0);
        let turn = s.right_stick.x_raw() as f64 * (12.0 / 127.0);

        // Adjust left and right speeds based on the turn input
        l += turn;
        r -= turn;

        self.drivetrain.left_motors.move_voltage(l.into());
        self.drivetrain.right_motors.move_voltage(r.into());
    }
}
