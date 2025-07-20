#![no_main]
#![no_std]
extern crate alloc;

use core::time::Duration;
use alloc::vec::Vec;

use vexide::prelude::*;
use gravity::GravLib::actuator::MotorGroup;
use gravity::GravLib::drivebase::{Chassis, TrackingWheel, OdomSensors, Pose};
use gravity::GravLib::drivebase::task_manager::{
    odometry_task_loop, get_odometry_stats, get_odometry_status, 
    OdometryTaskConfig
};
use gravity::GravLib::drivebase::odometry;

struct Robot {
    chassis: Chassis,
    controller: Controller,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous started!");
        
        // Example: Get the robot's current position
        let pose = odometry::get_pose(false); // false = degrees
        println!("Starting position: ({:.2}, {:.2}, {:.2}°)", 
                 pose.x(), pose.y(), pose.theta());
        
        // Wait a bit and check the position again
        vexide::core::time::sleep(Duration::from_millis(1000)).await;
        
        let new_pose = odometry::get_pose(false);
        println!("After 1 second: ({:.2}, {:.2}, {:.2}°)", 
                 new_pose.x(), new_pose.y(), new_pose.theta());
        
        // Get task statistics
        if let Some(stats) = get_odometry_stats().await {
            println!("Odometry stats - Updates: {}, Avg time: {}μs", 
                     stats.updates_completed, stats.avg_update_time_us);
        }
    }

    async fn driver(&mut self) {
        println!("Driver control started!");
        
        // Example of pausing and resuming odometry during driver control
        // (you might want to do this during calibration or reconfiguration)
        
        loop {
            // Normal tank drive
            self.chassis.tank_drive(&self.controller);
            
            // Example: Print position periodically
            static mut COUNTER: u32 = 0;
            unsafe {
                COUNTER += 1;
                if COUNTER % 100 == 0 { // Every ~1 second at 100Hz
                    let pose = odometry::get_pose(false);
                    let speed = odometry::get_speed(false);
                    println!("Pos: ({:.1}, {:.1}, {:.1}°) Speed: ({:.1}, {:.1}, {:.1}°/s)", 
                             pose.x(), pose.y(), pose.theta(),
                             speed.x(), speed.y(), speed.theta());
                }
            }
            
            // Yield to allow other tasks to run
            vexide::core::time::sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    println!("BigGravLib Async Odometry Example");

    // Create motor groups
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

    // Create chassis
    let chassis = Chassis::new(left, right, 320.0, 100.0);
    
    // Set up odometry sensors
    // Example with tracking wheels and IMU
    let vertical_left = TrackingWheel::new(
        RotationSensor::new(peripherals.port_7, Direction::Forward).unwrap(),
        65.0,   // wheel diameter in mm
        -160.0, // offset from center in mm  
        1.0     // gear ratio
    );
    
    let vertical_right = TrackingWheel::new(
        RotationSensor::new(peripherals.port_8, Direction::Forward).unwrap(),
        65.0,   // wheel diameter in mm
        160.0,  // offset from center in mm
        1.0     // gear ratio
    );
    
    let horizontal = TrackingWheel::new(
        RotationSensor::new(peripherals.port_9, Direction::Forward).unwrap(),
        65.0,  // wheel diameter in mm
        100.0, // offset from center in mm
        1.0    // gear ratio
    );

    let imu = InertialSensor::new(peripherals.port_10);
    
    let sensors = OdomSensors::new(
        Some(vertical_left),
        Some(vertical_right),
        Some(horizontal),
        None, // no second horizontal wheel
        Some(imu)
    );

    // Custom task configuration for high-precision odometry
    let task_config = OdometryTaskConfig {
        frequency_hz: 200,        // 200Hz = 5ms intervals (higher precision)
        priority: 250,            // Very high priority
        auto_start: true,         // Start automatically after calibration
        max_jitter_us: 1000,      // 1ms jitter tolerance
    };

    // Calibrate chassis with custom configuration
    println!("Calibrating chassis and IMU...");
    match chassis.calibrate_with_config(sensors, task_config, true).await {
        Ok(_) => println!("Calibration successful!"),
        Err(e) => {
            println!("Calibration failed: {:?}", e);
            return;
        }
    }

    // Spawn the background odometry task
    vexide::async_runtime::spawn(odometry_task_loop()).detach();

    // Wait a moment for the task to start
    vexide::core::time::sleep(Duration::from_millis(100)).await;
    
    // Check that odometry is running
    let status = get_odometry_status();
    println!("Odometry status: {:?}", status);

    // Set initial pose (example: starting at field center facing forward)
    odometry::set_pose(Pose::new(0.0, 0.0, 0.0), false);
    
    let controller = peripherals.primary_controller;
    let mut robot = Robot { chassis, controller };
    
    // Start the competition
    robot.compete().await;
}

// Helper function demonstrating advanced odometry features
async fn demonstrate_advanced_features() {
    // Example of temporarily pausing odometry
    use gravity::GravLib::drivebase::task_manager::{pause_odometry, resume_odometry};
    
    println!("Pausing odometry for recalibration...");
    let _ = pause_odometry().await;
    
    // Do some recalibration work here...
    vexide::core::time::sleep(Duration::from_millis(500)).await;
    
    println!("Resuming odometry...");
    let _ = resume_odometry().await;
    
    // Get detailed statistics
    if let Some(stats) = get_odometry_stats().await {
        println!("Detailed odometry statistics:");
        println!("  Total updates: {}", stats.updates_completed);
        println!("  Max update time: {}μs", stats.max_update_time_us);
        println!("  Average update time: {}μs", stats.avg_update_time_us);
        println!("  Jitter violations: {}", stats.jitter_violations);
        if let Some(last_update) = stats.last_update_time {
            println!("  Last update: {:?}", last_update);
        }
    }
    
    // Example of pose estimation
    let current_pose = odometry::get_pose(true); // true = radians
    let estimated_pose = odometry::estimate_pose(0.5, true); // Where will we be in 0.5 seconds?
    
    println!("Current pose: ({:.2}, {:.2}, {:.3})", 
             current_pose.x(), current_pose.y(), current_pose.theta());
    println!("Estimated pose in 0.5s: ({:.2}, {:.2}, {:.3})", 
             estimated_pose.x(), estimated_pose.y(), estimated_pose.theta());
}