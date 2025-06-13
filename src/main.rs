#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use core::time::Duration;
use spin::Mutex;
use alloc::vec::Vec;
use alloc::boxed::Box;

use vexide::{devices::adi::motor, prelude::*};
use crate::GravLib::actuator::motor_group::motorGroup;

//------------------------------------------------------
// 1) Change your worker to take a &'static Mutex<motorGroup>
//------------------------------------------------------
async fn location_track(g: &'static Mutex<motorGroup>) {
    loop {
        {
            let mut guard = g.lock();  // lock the mutex, protecting it from other task access
            println!(
                "Voltage: {} | Position: {}",
                guard.voltage(),
                guard.position()
            );
            // guard dropped here, stop protection
        }
        sleep(Duration::from_millis(100)).await;
    }
}

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


    //------------------------------------------------------
    // SECTION 1: ROBOT CONFIGURATION
    //------------------------------------------------------


    //------------------------------------------------------
    // 2) Allocate your Mutex<motorGroup> on the heap and leak it,
    //    so you get a &'static reference you can both spawn
    //    and continue to use in `main`.
    //------------------------------------------------------
    let boxed_group = Box::new(Mutex::new(
        motorGroup::new(
            Vec::from([
                Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
                Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
            ])
        )
    )); 
    let static_group: &'static Mutex<motorGroup> = Box::leak(boxed_group); // leak the Box to get a static reference
           // Why static reference? to extend **LIFETIME** to ENTIRE PROGRAM

    // 3) Spawn the tracking task
    vexide::task::spawn(location_track(static_group)).detach();

    //------------------------------------------------------

    {
        let mut guard = static_group.lock(); // lock the mutex to access the motor group
        guard.set_voltage(-6.0);
    }
    sleep(Duration::from_secs(1)).await;

    {
        let mut guard = static_group.lock();
        guard.set_voltage(6.0);
    }
    sleep(Duration::from_secs(1)).await;

    {
        let mut guard = static_group.lock();
        guard.brake(BrakeMode::Coast);
    }
    sleep(Duration::from_secs(1_000)).await;

    // 5) Finally run your Robot
    robot.compete().await;
}
