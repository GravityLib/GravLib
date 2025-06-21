use crate::GravLib::drivebase::chassis::Chassis;
// use crate::utils::geometry::{Point, angle_between_points};
// use crate::utils::math::clamp;
// TODO: Add odometry stuff for chassis and then work on this

// pub struct TurnToPointConfig {
//     pub max_angular_speed: f64,
//     pub tolerance_rad: f64,
//     pub kP: f64,
// }

// impl Default for TurnToPointConfig {
//     fn default() -> Self {
//         Self {
//             max_angular_speed: 3.0, // radians/sec
//             tolerance_rad: 0.02,    // ~1 degree
//             kP: 2.0,
//         }
//     }
// }

// pub struct TurnToPoint {
//     pub target: Point,
//     pub config: TurnToPointConfig,
//     finished: bool,
// }

// impl TurnToPoint {
//     pub fn new(target: Point, config: TurnToPointConfig) -> Self {
//         Self {
//             target,
//             config,
//             finished: false,
//         }
//     }

//     pub fn update(&mut self, chassis: &mut Chassis) {
//         let pose = chassis.pose();
//         let target_angle = angle_between_points(pose.position, self.target);
//         let error = angle_diff(target_angle, pose.heading);

//         if error.abs() < self.config.tolerance_rad {
//             chassis.set_velocity(0.0, 0.0);
//             self.finished = true;
//             return;
//         }

//         let angular_speed = clamp(self.config.kP * error, -self.config.max_angular_speed, self.config.max_angular_speed);
//         chassis.set_velocity(0.0, angular_speed);
//     }

//     pub fn is_finished(&self) -> bool {
//         self.finished
//     }
// }

// // Helper: shortest signed angle difference [-PI, PI]
// fn angle_diff(target: f64, current: f64) -> f64 {
//     let mut diff = target - current;
//     while diff > std::f64::consts::PI { diff -= 2.0 * std::f64::consts::PI; }
//     while diff < -std::f64::consts::PI { diff += 2.0 * std::f64::consts::PI; }
//     diff
// }