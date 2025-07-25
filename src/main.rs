#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use core::time::Duration;

use alloc::{sync::Arc, vec::Vec};
use spin::Mutex;

use vexide::{devices::adi::motor, prelude::*};

use crate::GravLib::{
    actuator::MotorGroup,
    odom::sensors::TrackingWheel
};


struct Robot {
    controller: Controller,
    tracking_wheel: Arc<Mutex<TrackingWheel>>,
}

impl Robot {
    pub fn new(peripherals: Peripherals) -> Self {
        let tracking_wheel = Arc::new(Mutex::new(TrackingWheel::new(
            RotationSensor::new(peripherals.port_9, Direction::Forward),
            4.0, // diameter in inches
            0.0, // offset
            1.0, // ratio
        )));

        Self {
            controller: peripherals.primary_controller,
            tracking_wheel,
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        // your autonomous code here
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        // driver-control loop: reads sticks & sets voltages
        loop {
            let mut track = self.tracking_wheel.lock().get_distance_travelled();
            println!("Tracking Wheel Distance: {:.4} inches", track);
            //delay 10ms
            vexide::time::sleep(Duration::from_millis(10)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals);
    // This hands off control to vexide’s scheduler (autonomous → driver)
    robot.compete().await;
}
