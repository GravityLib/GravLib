#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use alloc::vec::Vec;

use vexide::prelude::*;
use crate::GravLib::*;

struct Robot {}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        println!("Driver!");
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {};
    let commander = GravLib::mission_control::commander::Commander::new();
    let motors = Vec::from([
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
    ]);

    let mut funny = actuator::motor_group::motorGroup::new(commander, motors);
    {
        let _ = funny.set_velocity(100.0);
    }
    let _ = funny.set_velocity(50.0);

    // robot.compete().await;
}