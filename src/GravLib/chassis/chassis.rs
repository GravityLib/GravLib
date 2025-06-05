extern crate alloc;

use alloc::vec::Vec;
use vexide::prelude::Motor;
use gravlib::pid::PID;
use gravlib::actuator::MotorGroup;



pub struct OdomSensors {
    pub vertical1: Motor,
    pub vertical2: Motor,
    pub horizontal1: Motor,
    pub horizontal2: Motor,
    pub imu: vexide::devices::imu::Imu,
}

impl OdomSensors {
    pub fn ControllerSettings(
        vertical1: Motor,
        vertical2: Motor,
        horizontal1: Motor,
        horizontal2: Motor,
        imu: vexide::devices::imu::Imu,
    ) -> Self {
        OdomSensors {
            vertical1,
            vertical2,
            horizontal1,
            horizontal2,
            imu,
        }
    }
}

// --------------------------------------------------------------------------------------------------

// pub struct ControllerSettings {
    
// }

// impl ControllerSettings {
//     pub fn new(
//         kP: f64,
//         kI: f64,
//         kD: f64,
//         windupRange: f64,
//         smallError: f64,
//         smallErrorTimeout: f64,
//         largeError: f64,
//         largeErrorTimeout: f64,
//         slew: f64,
//     ) -> Self {
//         ControllerSettings {
//             kP,
//             kI,
//             kD,
//             windupRange,
//             smallError,
//             smallErrorTimeout,
//             largeError,
//             largeErrorTimeout,
//             slew,
//         }
//     }
// }

// --------------------------------------------------------------------------------------------------

pub struct Drivetrain {
    pub leftMotors: leftMotors,
    pub rightMotors: rightMotors,
    pub trackWidth: f64,
    pub wheelDiameter: f64,
    pub rpm: f64,
    pub horizontalDrift: f64,
}

impl Drivetrain {
    pub fn Drivetrain(

    ) -> Self {
        Drivetrain {}
    }
}

// --------------------------------------------------------------------------------------------------


pub enum AngularDirection {
    Auto,
    CW_Clockwise,
    CCW_CounterClockwise,
}

impl Default for AngularDirection {
    fn default() -> Self {
        AngularDirection::Auto
    }
}

pub struct TurnToPointParams {
    pub forwards: bool,
    pub direction: AngularDirection,
    pub max_speed: i32,
    pub min_speed: i32,
    pub early_exit_range: f64,
}

impl Default for TurnToPointParams {
    fn TurnToPoint_default() -> Self {
        TurnToPointParams {
            forwards: true,
            direction: AngularDirection::default(),
            max_speed: 127,
            min_speed: 0,
            early_exit_range: 0.0,
        }
    }
}

// --------------------------------------------------------------------------------------------------

pub struct TurnToHeadingParams {
    pub direction: AngularDirection,
    pub max_speed: i32,
    pub min_speed: i32,
    pub early_exit_range: f64,
}

impl Default for TurnToHeadingParams {
    fn TurnToHeading_default() -> Self {
        TurnToHeadingParams {
            direction: AngularDirection::default(),
            max_speed: 127,
            min_speed: 0,
            early_exit_range: 0.0,
        }
    }
}

// --------------------------------------------------------------------------------------------------

pub enum DriveSide {
    LEFT,
    RIGHT,
}

pub struct SwingToPoseParams {
    pub forwards: bool,
    pub direction: AngularDirection,
    pub max_speed: i32,
    pub min_speed: i32,
    pub early_exit_range: f64,
}

impl Default for SwingToPoseParams {
    fn SwingToPose_default() -> Self {
        SwingToPose {
            forwards: true,
            direction: AngularDirection::default(),
            max_speed: 127,
            min_speed: 0,
            early_exit_range: 0.0,
        }
    }
}

// --------------------------------------------------------------------------------------------------

pub struct SwingToHeadingParams {
    pub direction: AngularDirection,
    pub max_speed: i32,
    pub min_speed: i32,
    pub early_exit_range: f64,
}

impl Default for SwingToHeadingParams {
    fn SwingToHeading_default() -> Self {
        SwingToPose {
            direction: AngularDirection::default(),
            max_speed: 127,
            min_speed: 0,
            early_exit_range: 0.0,
        }
    }
}

// --------------------------------------------------------------------------------------------------

pub struct MoveToPoseParams {
    pub direction: AngularDirection,
    pub max_speed: i32,
    pub min_speed: i32,
    pub early_exit_range: f64,
}

impl Default for MoveToPoseParams {
    fn MoveToPose_default() -> Self {
        MoveToPoseParams {
            direction: AngularDirection::default(),
            max_speed: 127,
            min_speed: 0,
            early_exit_range: 0.0,
        }
    }
}

// --------------------------------------------------------------------------------------------------

pub struct MoveToPointParams {
    pub forwards: bool,
    pub max_speed: i32,
    pub min_speed: i32,
    pub early_exit_range: f64,
}

impl Default for MoveToPointParams {
    fn MoveToPoint_default() -> Self {
        MoveToPointParams {
            forwards: true,
            max_speed: 127,
            min_speed: 0,
            early_exit_range: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------------------------------

// !! TRANSLATED FROM LEMLIB C++ --> RUST !!

impl chassis::Drivetrain {
    #[allow(clippy::too_many_arguments)]
    pub fn moveToPose(
        &mut self,
        x: f64,
        y: f64,
        theta: f64,
        timeout_ms: u64,
        mut params: MoveToPoseParams,
        async_mode: bool,
        // Add references to PIDs, pose, etc. as needed
    ) {
        // If async, spawn a new task/thread (tokio or std::thread)
        if async_mode {
            // You may want to use async runtimes or threads here
            // Example with std::thread:
            let mut drivetrain = self.clone(); // Ensure Drivetrain is Clone
            std::thread::spawn(move || {
                drivetrain.move_to_pose(x, y, theta, timeout_ms, params, false);
            });
            // Optionally: sleep to let the thread start
            std::thread::sleep(std::time::Duration::from_millis(10));
            return;
        }

        // Reset PIDs and exit conditions
        // lateral_pid.reset();
        // lateral_large_exit.reset();
        // lateral_small_exit.reset();
        // angular_pid.reset();
        // angular_large_exit.reset();
        // angular_small_exit.reset();

        // Calculate target pose
        let mut target_theta = std::f64::consts::FRAC_PI_2 - theta.to_radians();
        if !params.forwards {
            target_theta = (target_theta + std::f64::consts::PI) % (2.0 * std::f64::consts::PI);
        }
        // Use global horizontal_drift if params.horizontal_drift == 0
        if params.horizontal_drift == 0.0 {
            params.horizontal_drift = self.horizontalDrift;
        }

        // Initialize variables
        // let mut last_pose = self.get_pose();
        // let mut dist_traveled = 0.0;
        let start_time = std::time::Instant::now();
        let mut close = false;
        let mut lateral_settled = false;
        let mut prev_same_side = false;
        let mut prev_lateral_out = 0.0;
        let mut prev_angular_out = 0.0;

        // Main loop
        loop {
            let elapsed = start_time.elapsed().as_millis() as u64;
            if elapsed > timeout_ms {
                break;
            }

            // let pose = self.get_pose();
            // dist_traveled += pose.distance(&last_pose);
            // last_pose = pose;

            // let dist_target = pose.distance(&target);

            // if dist_target < 0.1 && !close {
            //     close = true;
            //     params.max_speed = prev_lateral_out.abs().max(60.0) as i32;
            // }

            // if lateral_large_exit.get_exit() && lateral_small_exit.get_exit() {
            //     lateral_settled = true;
            // }

            // Calculate carrot point, robotSide, carrotSide, sameSide, etc.
            // let carrot = ...;
            // let robot_side = ...;
            // let carrot_side = ...;
            // let same_side = robot_side == carrot_side;
            // if !same_side && prev_same_side && close && params.min_speed != 0 {
            //     break;
            // }
            // prev_same_side = same_side;

            // let adjusted_robot_theta = if params.forwards { pose.theta } else { pose.theta + std::f64::consts::PI };
            // let angular_error = if close {
            //     angle_error(adjusted_robot_theta, target_theta)
            // } else {
            //     angle_error(adjusted_robot_theta, pose.angle(&carrot))
            // };
            // let mut lateral_error = pose.distance(&carrot);
            // if close {
            //     lateral_error *= angle_error(pose.theta, pose.angle(&carrot)).cos();
            // } else {
            //     lateral_error *= angle_error(pose.theta, pose.angle(&carrot)).cos().signum();
            // }

            // Update exit conditions, get PID outputs
            // lateral_small_exit.update(lateral_error);
            // lateral_large_exit.update(lateral_error);
            // angular_small_exit.update(rad_to_deg(angular_error));
            // angular_large_exit.update(rad_to_deg(angular_error));

            // let mut lateral_out = lateral_pid.update(lateral_error);
            // let mut angular_out = angular_pid.update(rad_to_deg(angular_error));

            // Clamp outputs
            // angular_out = angular_out.clamp(-(params.max_speed as f64), params.max_speed as f64);
            // lateral_out = lateral_out.clamp(-(params.max_speed as f64), params.max_speed as f64);

            // Slew rate limiting
            // if !close {
            //     lateral_out = slew(lateral_out, prev_lateral_out, lateral_settings.slew);
            // }

            // Max slip speed
            // let radius = 1.0 / get_curvature(&pose, &carrot).abs();
            // let max_slip_speed = (params.horizontal_drift * radius * 9.8).sqrt();
            // lateral_out = lateral_out.clamp(-max_slip_speed, max_slip_speed);

            // Overturn prioritization
            // let overturn = angular_out.abs() + lateral_out.abs() - params.max_speed as f64;
            // if overturn > 0.0 {
            //     lateral_out -= if lateral_out > 0.0 { overturn } else { -overturn };
            // }

            // Prevent wrong direction
            // if params.forwards && !close {
            //     lateral_out = lateral_out.max(0.0);
            // } else if !params.forwards && !close {
            //     lateral_out = lateral_out.min(0.0);
            // }

            // Min speed constraint
            // if params.forwards && lateral_out < params.min_speed.abs() as f64 && lateral_out > 0.0 {
            //     lateral_out = params.min_speed.abs() as f64;
            // }
            // if !params.forwards && -lateral_out < params.min_speed.abs() as f64 && lateral_out < 0.0 {
            //     lateral_out = -(params.min_speed.abs() as f64);
            // }

            // prev_angular_out = angular_out;
            // prev_lateral_out = lateral_out;

            // println!("lateralOut: {}, angularOut: {}", lateral_out, angular_out);

            // Ratio speeds
            // let mut left_power = lateral_out + angular_out;
            // let mut right_power = lateral_out - angular_out;
            // let ratio = left_power.abs().max(right_power.abs()) / params.max_speed as f64;
            // if ratio > 1.0 {
            //     left_power /= ratio;
            //     right_power /= ratio;
            // }

            // Move drivetrain
            // self.leftMotors.move(left_power);
            // self.rightMotors.move(right_power);

            // Sleep to save resources
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        // Stop drivetrain
        // self.leftMotors.move(0.0);
        // self.rightMotors.move(0.0);
        // dist_traveled = -1.0;
        // self.end_motion();
    }
}

// ----------------------------------------------------------------------------------------------------

// !! TRANSLATED FROM LEMLIB C++ --> RUST !!

impl OdomSensors {
    pub fn new(
        vertical1: Motor,
        vertical2: Motor,
        horizontal1: Motor,
        horizontal2: Motor,
        imu: vexide::devices::imu::Imu,
    ) -> Self {
        OdomSensors {
            vertical1,
            vertical2,
            horizontal1,
            horizontal2,
            imu,
        }
    }
}

impl Drivetrain {
    pub fn new(
        left_motors: MotorGroup,
        right_motors: MotorGroup,
        track_width: f64,
        wheel_diameter: f64,
        rpm: f64,
        horizontal_drift: f64,
    ) -> Self {
        Drivetrain {
            leftMotors: left_motors,
            rightMotors: right_motors,
            trackWidth: track_width,
            wheelDiameter: wheel_diameter,
            rpm,
            horizontalDrift: horizontal_drift,
        }
    }
}

pub struct Chassis {
    pub drivetrain: Drivetrain,
    pub lateral_settings: ControllerSettings,
    pub angular_settings: ControllerSettings,
    pub sensors: OdomSensors,
    // throttle_curve: Option<DriveCurve>,
    // steer_curve: Option<DriveCurve>,
    // lateral_pid: PID,
    // angular_pid: PID,
    // lateral_large_exit: ExitCondition,
    // lateral_small_exit: ExitCondition,
    // angular_large_exit: ExitCondition,
    // angular_small_exit: ExitCondition,
}

impl Chassis {
    pub fn new(
        drivetrain: Drivetrain,
        lateral_settings: ControllerSettings,
        angular_settings: ControllerSettings,
        sensors: OdomSensors,
        // throttle_curve: Option<DriveCurve>,
        // steer_curve: Option<DriveCurve>,
    ) -> Self {
        Chassis {
            drivetrain,
            lateral_settings,
            angular_settings,
            sensors,
            // throttle_curve,
            // steer_curve,
            // lateral_pid: PID::new(...),
            // angular_pid: PID::new(...),
            // lateral_large_exit: ExitCondition::new(...),
            // lateral_small_exit: ExitCondition::new(...),
            // angular_large_exit: ExitCondition::new(...),
            // angular_small_exit: ExitCondition::new(...),
        }
    }

    pub fn calibrate(&mut self, calibrate_imu: bool) {
        if calibrate_imu {
            self.calibrate_imu();
        }
        // TODO: Initialize odometry wheels if needed
        // self.sensors.vertical1.reset();
        // self.sensors.vertical2.reset();
        // if let Some(h1) = &mut self.sensors.horizontal1 { h1.reset(); }
        // if let Some(h2) = &mut self.sensors.horizontal2 { h2.reset(); }
        // self.set_sensors();
        // self.init();
        // Indicate success (controller feedback)
    }

    fn calibrate_imu(&mut self) {
        let mut attempt = 1;
        let mut calibrated = false;
        while attempt <= 5 {
            self.sensors.imu.reset();
            // Wait until IMU is calibrated
            while self.sensors.imu.is_calibrating() {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            // Check if IMU is calibrated
            let heading = self.sensors.imu.get_heading();
            if !heading.is_nan() && !heading.is_infinite() {
                calibrated = true;
                break;
            }
            // Indicate error (controller rumble, log warning)
            // TODO: Add controller rumble and logging
            attempt += 1;
        }
        if !calibrated {
            // self.sensors.imu = None; // If using Option
            // TODO: Log error, fallback to tracking wheels
        }
    }

    pub fn set_pose(&mut self, x: f64, y: f64, theta: f64, radians: bool) {
        // TODO: Implement pose setting logic
    }

    pub fn get_pose(&self, radians: bool, standard_pos: bool) -> Pose {
        // TODO: Implement pose getting logic
        // let mut pose = self.get_pose_internal();
        // if standard_pos { pose.theta = std::f64::consts::FRAC_PI_2 - pose.theta; }
        // if !radians { pose.theta = pose.theta.to_degrees(); }
        // pose
        unimplemented!()
    }

    pub fn wait_until(&self, dist: f64) {
        // TODO: Implement wait until distance traveled
    }

    pub fn wait_until_done(&self) {
        // TODO: Implement wait until done
    }

    pub fn request_motion_start(&mut self) {
        // TODO: Implement motion queue logic
    }

    pub fn end_motion(&mut self) {
        // TODO: Implement end motion logic
    }

    pub fn cancel_motion(&mut self) {
        // TODO: Implement cancel motion logic
    }

    pub fn cancel_all_motions(&mut self) {
        // TODO: Implement cancel all motions logic
    }

    pub fn is_in_motion(&self) -> bool {
        // TODO: Implement check for motion running
        false
    }

    pub fn reset_local_position(&mut self) {
        // TODO: Implement reset local position
    }

    pub fn set_brake_mode(&mut self, mode: vexide::devices::motor::BrakeMode) {
        // self.drivetrain.leftMotors.set_brake_mode_all(mode);
        // self.drivetrain.rightMotors.set_brake_mode_all(mode);
    }
}


// ----------------------------------------------------------------------------------------------------

impl vexide::devices::imu::Imu for OdomSensors {
    fn get_heading(&self) -> f64 {
        self.imu.get_heading()
    }

    fn get_roll(&self) -> f64 {
        self.imu.get_roll()
    }

    fn get_pitch(&self) -> f64 {
        self.imu.get_pitch()
    }
}


