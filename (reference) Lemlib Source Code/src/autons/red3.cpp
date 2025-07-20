#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Red 3  Autonomous Routine //
// ------------------------- //

// e.g. goal rush

int red3() {
    
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 0, 0, 10000, {}, true);
    pros::delay(5000);
    chassis.turnToHeading(-359, 100000);

    return 0;
}


