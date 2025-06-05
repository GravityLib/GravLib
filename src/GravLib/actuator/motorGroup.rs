extern crate alloc;

use alloc::vec::Vec;

use vexide::prelude::Motor;

pub struct MotorGroup {
    ports: Vec<i32>,
    cartridge: vexide::devices::smart::Motor,
}

/* 
    TODO: 
    1. Motorgroup finish
    2. Motorgroup move
    3. Motorgroup move_absolute
    4. Motorgroup move_relative
    5. Motorgroup move_velocity
    6. Motorgroup move_voltage
    7. Motorgroup brake

*/