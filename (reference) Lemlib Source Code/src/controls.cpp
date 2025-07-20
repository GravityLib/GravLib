#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

#include "robot-config.hpp"
#include "colorsort.hpp"

char alliance = 'N';
bool allianceConfirmed = false;

double lady_zero;
double lady_err;
double lady_pos;

void graphOdom() {
    do {
        lemlib::Pose position = chassis.getPose();
        float x = position.x;
        float y = position.y;

        printf("X: %f| Y: %f \n", x, y);
        pros::delay(10);
    }
    while (chassis.isInMotion());
}


void ladyMoveToSTDPose(float target, float acceptableRange) {
    float error = target - lady_rotation.get_angle();
    while (!(error < error + acceptableRange && error > error - acceptableRange)) {
        if (error > 0) { // when error is positive
            lady.move(50);
        }
        else { // when error is negative
            lady.move(-50);
        }
    }
    lady.brake();
}

void ladyctl() {
    bool spinning = false;
    lady.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    int counter = 0;

    while (true) {
        // lady_pos = lady.get_position() - lady_zero;

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lady.move(-100);
            spinning = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lady.move(100);
            spinning = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            lady.move(100);
            pros::delay(400);
            lady.move(0);
            lady_rotation.set_position(0);
            ladySmart.movePID(-900, 2000, 0.5, false);
        }
        else if (spinning == true) {
            lady.brake();
            spinning = false;
        }
        pros::delay(10);
    }
}

void intake_control() {
    const double stuck_lim_low = 0.0;
    const double stuck_lim_high = 0.0;
    const int reverse_time = 200;
    const int forward_delay = 100;

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    bool intake_spinning = true;
    
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127); // Spin out
            intake_spinning = false;
            printf("Intake Activitated\n");
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // bool passed = ringInspect();
            bool passed = true; // TODO - TEMPORARY
            if (passed) {
                double velocity = intake.get_actual_velocity();
                if (false) {// velocity < stuck_lim_low && velocity > -stuck_lim_high
                    // Reverse the motor briefly
                    intake.move(-127);
                    pros::delay(reverse_time);
                    // Resume forward spin
                    intake.move(127);
                    pros::delay(forward_delay);
                } else {
                    intake.move(127); // Spin in normally
                }
                printf("Intake (INSPECT) Activitated\n");
                printf("    PASSED: True \n");
            }
            else {
                printf("Intake (INSPECT) COMPROMISED [!]\n");
                printf("    PASSED: False \n");
                intake.move(127);
                pros::delay(220); // TODO - Tune delay accordingly to intake
                intake.brake();
                pros::delay(200);
            }
            intake_spinning = false;
        }
        else if (intake_spinning == false) {
            intake.move(0);
            intake_spinning = true;
        }
        pros::delay(10);
    }
}

void mogo_control() {
    // Start with the opposite default state
    bool toggle = true;
    bool latch = false;

    // Set mogo_mech to the default state immediately
    mogo_mech.set_value(toggle);

    while (true) {
        bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);

        if (b_button && !latch) {
            toggle = !toggle;             // Flip the toggle state
            mogo_mech.set_value(toggle);  // Update mogo_mech based on the new toggle state
            latch = true;                 // Engage latch to prevent repeated toggles
        } else if (!b_button) {
            latch = false;                // Reset latch when button is released
        }
        pros::delay(10);
    }
}

// void doinker_control() {
//     bool toggle = true;
//     bool latch = false;

//     while (true) {
//         bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

//         if (b_button && !latch) {
//             toggle = !toggle;             // Flip the toggle state
//             doinker.set_value(toggle);  // Update mogo_mech based on the new toggle state
//             latch = true;                 // Engage latch to prevent repeated toggles
//         } else if (!b_button) {
//             latch = false;                // Reset latch when button is released
//         }
//         pros::delay(10);
//     }
// }

// void endgame_control() {
//     bool toggle = true;
//     bool latch = false;

//     while (true) {
//         bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

//         if (b_button && !latch) {
//             toggle = !toggle;             // Flip the toggle state
//             endgame.set_value(toggle);  // Update mogo_mech based on the new toggle state
//             latch = true;                 // Engage latch to prevent repeated toggles
//         } else if (!b_button) {
//             latch = false;                // Reset latch when button is released
//         }
//         pros::delay(10);
//     }
// }

void drivetrain_control() {
    while (true) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        
          // ------------------------------- //
         //  EXPERIMENTAL - Variable Ctl   //
        // ----------------------------- //

        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        
        chassis.variableCurvature(leftY, rightX, 1, false);                     // Regular tank

        pros::delay(10);
    }
}
