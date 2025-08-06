#![no_main]
#![no_std]
extern crate alloc;
mod GravLib;

use core::time::Duration;

use alloc::{sync::Arc, vec::Vec};
use alloc::vec;
use spin::Mutex;

use vexide::devices::math::Point2;
use vexide::prelude::*;
use vexide::devices::{display::*};

use crate::GravLib::misc::gravlib_logo;
use crate::GravLib::odom::{
    sensors::{TrackingWheel, Sensors},
    localisation::Localisation
};

struct Robot {
    controller: Controller,
    display: Arc<Mutex<Display>>,
    localisation: Arc<Mutex<Localisation>>,
}

impl Robot {
    pub fn new(peripherals: Peripherals) -> Self {
        // Sensor configuration placeholders
        let vertical_wheel = Arc::new(Mutex::new(TrackingWheel::new(
            RotationSensor::new(peripherals.port_10, Direction::Forward), // PLACEHOLDER: Configure vertical tracking wheel port
            2.75, // PLACEHOLDER: Set wheel diameter in inches
            0.0, // PLACEHOLDER: Set vertical offset from center in inches
            1.0, // PLACEHOLDER: Set gear ratio
        )));
        
        let horizontal_wheel = Arc::new(Mutex::new(TrackingWheel::new(
            RotationSensor::new(peripherals.port_9, Direction::Forward), // PLACEHOLDER: Configure horizontal tracking wheel port
            2.75, // PLACEHOLDER: Set wheel diameter in inches 
            -4.5, // PLACEHOLDER: Set horizontal offset from center in inches
            1.0, // PLACEHOLDER: Set gear ratio
        )));

        let sensors = Arc::new(Mutex::new(Sensors {
            horizontal_wheels: vec![horizontal_wheel],
            vertical_wheels: vec![vertical_wheel],
            imu: Arc::new(Mutex::new(InertialSensor::new(peripherals.port_7))), // PLACEHOLDER: Configure IMU port
        }));

        let localisation = Arc::new(Mutex::new(Localisation::new(sensors)));
        let display = Arc::new(Mutex::new(peripherals.display));

        Self {
            controller: peripherals.primary_controller,
            display,
            localisation,
        }
    }

    #[allow(dead_code)]
    pub async fn initialise(&mut self) {
        // 1. Calibrate IMU
        println!("Robot calibration Started.");
        
        // 2. Draw the gravlib logo on the display
        {
            let mut d = self.display.lock();
            gravlib_logo(&mut *d);
        }
        self.localisation.lock().calibrate(true).await;

        println!("Robot calibration complete.");
        

        // 3. Spawn a background task for continual localisation updates & telemetry
        let loc = Arc::clone(&self.localisation);
        let disp = Arc::clone(&self.display);

        vexide::task::spawn(async move {
            loop {
                {
                    let mut local = loc.lock();
                    let mut d = disp.lock();
                    local.update(&mut *d);
                }
                // sleep between updates
                let _ = vexide::time::sleep(Duration::from_millis(10)).await;
            }
        }).detach();
    }
}


impl Compete for Robot {
    async fn autonomous(&mut self) {
        // your autonomous code here
        println!("Autonomous!");
    }

    async fn driver(&mut self) {
        // driver-control loop: reads sticks & sets voltages
        // loop {
        //     // Get current robot position from localization
        //     let pose = self.localisation.lock();
        //     let (x, y, theta) = pose.lock().get_position();
            
        //     println!("Robot Position - X: {:.4}, Y: {:.4}, Theta: {:.4}°", x, y, theta);
            
        //     //delay 10ms
        //     vexide::time::sleep(Duration::from_millis(10)).await;
        // }
        loop {
            vexide::time::sleep(Duration::from_millis(10)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut robot = Robot::new(peripherals);
    robot.initialise().await;
    
    // This hands off control to vexide's scheduler (autonomous → driver)
    robot.compete().await;
}
