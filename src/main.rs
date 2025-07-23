#![no_main]
#![no_std]
extern crate alloc;

use alloc::format;
use vexide::fs::Display;
mod GravLib;

use core::time::Duration;
use spin::Mutex;
use alloc::vec::Vec;
use alloc::boxed::Box;

use vexide::{devices::adi::motor, prelude::*};

use crate::GravLib::actuator::MotorGroup;
use crate::GravLib::drivebase::{configure_sensors, start_task, Chassis};

use crate::GravLib::PID;
use vexide::devices::controller;

use crate::GravLib::drivebase::odometry;
use crate::GravLib::drivebase::Pose;
use crate::GravLib::drivebase::task_loop;
use crate::GravLib::drivebase::OdomSensors;
use crate::GravLib::drivebase::TrackingWheel;

struct Robot {
    chassis: Chassis,
    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        // your autonomous code here
        println!("Autonomous!");
        // Move forward 0.5 meters (50 cm)

        self.chassis.get_pose();

        self.chassis.move_to_point(0.1, 0.0, 1000, None).await;

        // Move to (0.5, 0.5) meters
        self.chassis.move_to_point(0.1, 0.1, 1000, None).await;

        // Move to pose (0.3, 0.3, 45 degrees)
        self.chassis.move_to_pose(Pose::new(0.3, 0.3, 45.0), 1000, None).await;

        self.chassis.stop_motion();
    }

    async fn driver(&mut self) {
        // driver-control loop: reads sticks & sets voltages
        loop {
            self.chassis.tank_drive(&self.controller);

                let pose = odometry::get_pose(true);
                println!(
                    "Odometry - X: {:.2}, Y: {:.2}, Th: {:.2}",
                    pose.x(),
                    pose.y(),
                    pose.theta()
                );
                sleep(Duration::from_millis(50)).await;
            

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

    let sensors = OdomSensors {
        vertical1: Some(TrackingWheel::new(
            RotationSensor::new(peripherals.port_9, Direction::Forward),
            2.75, // wheel diameter in inches or cm
            0.0,  // offset (change as needed)
            1.0,  // gear ratio (change as needed)
        )),
        vertical2: Some(TrackingWheel::new(
            RotationSensor::new(peripherals.port_10, Direction::Forward),
            2.75, // wheel diameter in inches or cm
            0.0,  // offset (change as needed)
            1.0,  // gear ratio (change as needed)
        )),
        horizontal1: None,
        horizontal2: None,
        imu: None,
    };
    configure_sensors(sensors).unwrap();

    // Start the odometry background task
    start_task().unwrap();
    spawn(task_loop()).detach();

    let mut robot = Robot { chassis, controller };
    // This hands off control to vexide’s scheduler (autonomous → driver)
    robot.compete().await;

    loop{
        let pose = odometry::get_pose(true);
        controller.screen.set_text(&format!("X: {:.2}", pose.x()), 0, 0);
        controller.screen.set_text(&format!("Y: {:.2}", pose.y()), 1, 0);
        controller.screen.set_text(&format!("Th: {:.2}", pose.theta()), 2, 0);
        sleep(Duration::from_millis(100)).await;
    }

}
