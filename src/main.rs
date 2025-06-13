#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use core::time::Duration;

use alloc::vec::Vec;

use vexide::{devices::adi::motor, prelude::*};
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

async fn location_track(group_ptr: *mut actuator::motor_group::motorGroup) {
    loop {
        unsafe {
            let group: &actuator::motor_group::motorGroup = &*group_ptr;
            println!(
                "Voltage: {} | Position {}",
                group.voltage(),
                group.position()
            );
        }
        sleep(Duration::from_millis(100)).await;
    }
}
#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot {};

    let motors = Vec::from([
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
    ]);

    let mut funny = actuator::motor_group::motorGroup::new(motors);

    let funny_ptr: *mut actuator::motor_group::motorGroup = &mut funny as *mut _;
    vexide::task::spawn(location_track(funny_ptr)).detach();

    funny.set_voltage(-6.0);

    sleep(Duration::from_secs(1)).await;

    funny.set_voltage(6.0);

    sleep(Duration::from_secs(1)).await;

    funny.brake(BrakeMode::Coast);

    sleep(Duration::from_secs(1000)).await;

    // robot.compete().await;
}