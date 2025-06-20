#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use core::time::Duration;
use spin::Mutex;
use alloc::vec::Vec;
use alloc::boxed::Box;

use vexide::{devices::adi::motor, prelude::*};
use crate::GravLib::actuator::motor_group::MotorGroup;

use crate::GravLib::drivebase::chassis::{Drivetrain, Chassis};

struct Robot {
    chassis: Chassis,
    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        // your autonomous code here
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        // driver-control loop: reads sticks & sets voltages
        loop {
            self.chassis.tank(&self.controller);
            // yield to the runtime so the controller-update task can run
            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // Build two groups of three motors each
    let left = MotorGroup::new(Vec::from([
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ]));

    let right = MotorGroup::new(Vec::from([
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
    ]));

    // Wrap them in your chassis
    let chassis = Chassis::new(left, right, 320.0, 100.0);
    let controller = peripherals.primary_controller;

    let mut robot = Robot { chassis, controller };
    // This hands off control to vexide’s scheduler (autonomous → driver)
    robot.compete().await;
}
