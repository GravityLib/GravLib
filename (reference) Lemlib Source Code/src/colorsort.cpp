#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <sys/wait.h>

#include "controls.hpp"
#include "robot-config.hpp"


void autonColorSort();
void driverColorSort();
void ColorSortAlg();



void setColorSort() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::clear_line(3);
        pros::lcd::set_text(3, "[CLS] Alliance: RED");
        alliance = 'R';
    } else {
        pros::lcd::clear_line(3);
        pros::lcd::set_text(3, "[CLS] Alliance: BLUE");
        alliance = 'B';
    }
}

bool ringInspect() {
    const float blue_lim_low = 150, blue_lim_high = 250;
    const float red_lim_low = 0, red_lim_high = 50;
    double hue = colorSort.get_hue();

    if (alliance == 'B') {
        return !(red_lim_low < hue && hue < red_lim_high);
    } else if (alliance == 'R') {
        return !(blue_lim_low < hue && hue < blue_lim_high);
    }
    return true; // Default: passes for any other color
}

void Intake_SortedMove(int voltage, float msDelay, int penaltyFactor, bool async) {
    // Get the start time
    if (async) {
        pros::Task task([&]() { Intake_SortedMove(voltage, msDelay, penaltyFactor, false); });
        pros::delay(10); // delay to give the task time to start
        return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::milliseconds(static_cast<int>(msDelay));

    // While the elapsed time is less than msDelay
    while (std::chrono::high_resolution_clock::now() < endTime) {
        if (voltage > 0) {
            // Intaking
            bool passed = ringInspect();
            if (passed) {
                intake.move(voltage);
            } else {
                intake.move(voltage * penaltyFactor);
            }
        } else {
            // Outtaking or stopping
            intake.move(voltage);
        }
        pros::delay(10);
    }

    // Stop intake after delay
    intake.move(0);
}

// void autonColorSort() {}
//     double blue = color.get_rgb().blue; //color is not defined (you have to configure your optical sensor basically and name it color)
//     double red = color.get_rgb().red;
//     double green = color.get_rgb().green;
//     int RED_MIN = 300;
//     int BLUE_MIN = 150;
//     int GREEN_MAX = 100;


//     if(blue>BLUE_MIN) {

//         pros::c::delay(50);
//         intake.move(0);
//         pros::c::delay(500);
//     }

//     else if (red > RED_MIN) {

//         master.rumble("."); //Check if red is working
//     }
void driverColorSort() {}
    
void ColorSortAlg() {}
    
