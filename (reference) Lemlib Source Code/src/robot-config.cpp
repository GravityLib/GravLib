#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/smartMotor.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

// ---------------------------------------
// Drivetrain & Chassis
// ---------------------------------------


// CLAWBOT CONFIGS

pros::MotorGroup leftMotors({-19, -10, -1}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({7, 8, 9}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// PID settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.9, // 11.6 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3"25' omnis
                              450, // drivetrain rpm is 200 (green direct)
                              5 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor roller(-20, pros::MotorGearset::green);
pros::Motor hook(-5, pros::MotorGearset::blue);

pros::MotorGroup intake({-20, -5}, pros::MotorGearset::green); // front 1, back 6
pros::adi::DigitalOut mogo_mech (1);
// pros::adi::DigitalOut doinker (7);
// pros::adi::DigitalOut endgame (1);

// ---------------------------------------
// Sensors (Miscellaneous)
// ---------------------------------------

// pros::Imu imu(19);
// pros::Optical colorSort (18);

// ---------------------------------------
// Macro (Wall Stake Mech)
// ---------------------------------------

pros::MotorGroup lady({-6}); // TODO - Placeholder
lemlib::PID ladypid(10, 0.4, 50, 0, false);
pros::Rotation lady_rotation (12);
lemlib::SmartMotor ladySmart(&lady, &lady_rotation, ladypid);

// ---------------------------------------
// Odometry
// ---------------------------------------

// pros::Rotation horizontal_encoder(13); // Change to the "A" tagged encoder

// pros::Rotation vertical_encoder(17);

// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, +1.95);

// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -1.5);

// chassis: 12.8 across, 13.5 height
// traking cneter: 6.4, 6.75 

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // &horizontal_tracking_wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr     // inertial sensor &imu
);

// ---------------------------------------
// PID Controller
// ---------------------------------------

lemlib::ControllerSettings linearController(  6, // proportional gain (kP)
                                              -0.0045, // integral gain (kI) 0.11
                                              2, // derivative gain (kD) 1.5
                                              0, // anti windup
                                              1, // small error range, in inches
                                              200, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              75 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(0.75, // proportional gain (kP)
                                             0.00010, // integral gain (kI)
                                             0.5, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             200, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
); 

// ---------------------------------------
// ExpoDrive Constraints
// ---------------------------------------

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain 1.019 (LOG 0.9991)
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.010 // expo curve gain org 1.019
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);