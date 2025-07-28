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
            imu: Arc::new(Mutex::new(InertialSensor::new(peripherals.port_7))), // PLACEHOLDER: Configure IMU port
        }));

        let localisation = Arc::new(Mutex::new(Localisation::new(sensors)));

        let mut display = peripherals.display;

        vexide::task::spawn(async move {
            
            display.draw_text(&Text::new(r" $$$$$$\                               $$\       $$\ $$\       "
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 10])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));
            display.draw_text(&Text::new(r"$$  __$$\                              $$ |      \__|$$ |      "
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 20])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));
            display.draw_text(&Text::new(r"$$ /  \__| $$$$$$\  $$$$$$\ $$\    $$\ $$ |      $$\ $$$$$$$\  "
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 30])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));
            display.draw_text(&Text::new(r"$$ |$$$$\ $$  __$$\ \____$$\\$$\  $$  |$$ |      $$ |$$  __$$\ "
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 40])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));
            display.draw_text(&Text::new(r"$$ |\_$$ |$$ |  \__|$$$$$$$ |\$$\$$  / $$ |      $$ |$$ |  $$ |"
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 50])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));

            display.draw_text(&Text::new(r"\$$$$$$  |$$ |     \$$$$$$$ |  \$  /   $$$$$$$$\ $$ |$$$$$$$  |"
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 60])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));
            display.draw_text(&Text::new(r" \______/ \__|      \_______|   \_/    \________|\__|\_______/ "
            , Font::new(FontSize::EXTRA_SMALL, FontFamily::Monospace), Point2::<i16>::from([10, 70])),
                                Rgb::new(255, 0, 0),
                                Some(Rgb::new(0, 0, 0)));
            vexide::time::sleep(Duration::from_millis(10)).await;
        }).detach();

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
    let robot = Robot::new(peripherals);
    
    // Clone localisation for the update task
    let localisation_clone: Arc<spin::mutex::Mutex<Localisation>> = robot.localisation.clone();

    println!("Starting localization calibration...");
    localisation_clone.lock().sensors.lock().imu.lock().calibrate().await;
    println!("Localization calibration complete.");

    // Spawn the localization update task
    vexide::task::spawn(async move {
        localisation_clone.lock().update();
    }).detach();
    println!("Localization update task started.");
    
    // This hands off control to vexide's scheduler (autonomous → driver)
    robot.compete().await;
}
