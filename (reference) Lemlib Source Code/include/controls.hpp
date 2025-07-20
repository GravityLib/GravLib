#ifndef CONTROLS_HPP
#define CONTROLS_HPP

/**
 * @file controls.hpp
 * @brief Header file for robot control mechanisms.
 *
 * This file declares the global variables and functions used to manage various subsystems
 * of the robot, such as alliance selection, drivetrain, and intake control.
 *
 * ## Global Variables
 * - `char alliance`: Tracks the selected alliance ('R' for Red, 'B' for Blue, 'N' for None).
 * - `bool allianceConfirmed`: Indicates if the alliance selection has been finalized.
 * - `double lady_zero`: Initial position of the "lady" subsystem.
 * - `double lady_err`: Error value for the "lady" subsystem.
 * - `double lady_pos`: Current position of the "lady" subsystem.
 */

#include "main.h"

// Variables to keep track of alliance and selection status
extern char alliance;
extern bool allianceConfirmed;
extern double lady_zero;
extern double lady_err;
extern double lady_pos;

/**
 * @brief Toggles between Red and Blue alliance and updates the LLEMU LCD display.
 */
void setColorSort();

/**
 * @brief Inspects rings to determine if they match the alliance color.
 * 
 * @return `true` if the ring passes the inspection, `false` otherwise.
 */
bool ringInspect();

/**
 * @brief Main operation control function for teleoperation mode.
 */
void opcontrol();

/**
 * @brief Continuously logs the robot's odometry position (x, y) while in motion.
 */
void graphOdom();

/**
 * @brief Moves the intake system with sorting functionality.
 * 
 * @param voltage Voltage to apply to the intake motor.
 * @param msDelay Duration of the movement in milliseconds.
 * @param penaltyFactor Multiplier for voltage if an invalid ring is detected.
 * @param async If `true`, runs the movement asynchronously.
 */
void Intake_SortedMove(int voltage, float msDelay, int penaltyFactor, bool async);

/**
 * @brief Controls the intake mechanism based on controller input.
 */
void intake_control();

/**
 * @brief Toggles the mobile goal lift system.
 */
void mogo_control();

/**
 * @brief Toggles the "doinker" subsystem for special functions.
 */
void doinker_control();

/**
 * @brief Provides joystick-based drivetrain control with toggleable braking modes.
 */
void drivetrain_control();

/**
 * @brief Controls the "lady" subsystem for position adjustments and movements.
 */
void ladyctl();

/**
 * @brief Toggles the endgame mechanism.
 */
void endgame_control();

#endif // CONTROLS_HPP