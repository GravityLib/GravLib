#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

// ---------------------------------------
// Drivetrain & Chassis
// ---------------------------------------
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Drivetrain drivetrain;

// ---------------------------------------
// Subsystem Configuration
// ---------------------------------------
extern pros::Controller master;
extern pros::Motor roller;
extern pros::Motor hook;
extern pros::MotorGroup intake;
extern pros::adi::DigitalOut mogo_mech;
extern pros::adi::DigitalOut doinker;
extern pros::adi::DigitalOut endgame;

// ---------------------------------------
// Sensors (Miscellaneous)
// ---------------------------------------
extern pros::Imu imu;
extern pros::Optical colorSort;

// ---------------------------------------
// Macro (Wall Stake Mech)
// ---------------------------------------

extern pros::MotorGroup lady;
extern lemlib::PID ladypid;
extern pros::Rotation lady_rotation;
extern lemlib::SmartMotor ladySmart;

// ---------------------------------------
// Odometry
// ---------------------------------------
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;

// ---------------------------------------
// PID Controller
// ---------------------------------------
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;

// ---------------------------------------
// ExpoDrive Constraints
// ---------------------------------------
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;

#endif // ROBOT_CONFIG_HPP
