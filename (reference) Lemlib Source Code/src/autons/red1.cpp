#include "main.h"

#include "robot-config.hpp"
#include "controls.hpp"
#include "lemlib/selector.hpp"
#include "lemlib/smartMotor.hpp"
#include "lemlib/chassis/chassis.hpp"



// ------------------------- //
// Red 1  Autonomous Routine //
// ------------------------- //

// e.g. red right

int red1() {
    
    chassis.setPose(0, 0, 0);

    return 0;
}


