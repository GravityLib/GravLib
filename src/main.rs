#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use core::time::Duration;

use alloc::{sync::Arc, vec::Vec};
use alloc::vec;
use spin::Mutex;

use vexide::prelude::*;

use crate::GravLib::odom::{
    sensors::{TrackingWheel, Sensors},
    localisation::Localisation
};


struct Robot {
    controller: Controller,
    localisation: Arc<Mutex<Localisation>>,
}

impl Robot {
    pub fn new(peripherals: Peripherals) -> Self {
        // Sensor configuration placeholders
        let vertical_wheel = Arc::new(Mutex::new(TrackingWheel::new(
            RotationSensor::new(peripherals.port_10, Direction::Forward), // PLACEHOLDER: Configure vertical tracking wheel port
            4.0, // PLACEHOLDER: Set wheel diameter in inches
            6.0, // PLACEHOLDER: Set vertical offset from center in inches
            1.0, // PLACEHOLDER: Set gear ratio
        )));
        
        let horizontal_wheel = Arc::new(Mutex::new(TrackingWheel::new(
            RotationSensor::new(peripherals.port_9, Direction::Forward), // PLACEHOLDER: Configure horizontal tracking wheel port
            4.0, // PLACEHOLDER: Set wheel diameter in inches 
            -6.0, // PLACEHOLDER: Set horizontal offset from center in inches
            1.0, // PLACEHOLDER: Set gear ratio
        )));

        let sensors = Arc::new(Mutex::new(Sensors {
            horizontal_wheels: vec![horizontal_wheel],
            vertical_wheels: vec![vertical_wheel],
            imu: Arc::new(Mutex::new(InertialSensor::new(peripherals.port_3))), // PLACEHOLDER: Configure IMU port
        }));

        let localisation = Arc::new(Mutex::new(Localisation::new(sensors)));

        Self {
            controller: peripherals.primary_controller,
            localisation,
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
            // Get current robot position from localization
            let pose = self.localisation.lock().get_pose();
            let (x, y, theta) = pose.lock().get_position();
            
            println!("Robot Position - X: {:.4}, Y: {:.4}, Theta: {:.4}°", x, y, theta);
            
            //delay 10ms
            vexide::time::sleep(Duration::from_millis(10)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals);
    
    // Clone localisation for the update task
    let localisation_clone = robot.localisation.clone();
    
    // Spawn the localization update task
    vexide::task::spawn(async move {
        localisation_clone.lock().update();
    });
    
    // This hands off control to vexide's scheduler (autonomous → driver)
    robot.compete().await;
}
