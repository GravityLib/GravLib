//! Motion Algorithm Demonstration
//! 
//! This example shows how to use the new moveToPoint and moveToPose
//! algorithms that have been ported from LemLib to BigGravLib.

#![no_std]
#![no_main]

extern crate alloc;
use alloc::vec;

use vexide::prelude::*;
use gravity::GravLib::drivebase::*;
use gravity::GravLib::actuator::MotorGroup;
use gravity::GravLib::drivebase::motions::*;
use gravity::GravLib::drivebase::exit_conditions::ExitConditions;

#[vexide::main]
async fn main(peripherals: Peripherals) -> Result<(), Box<dyn core::error::Error>> {
    println!("BigGravLib Motion Demo Starting...");
    
    // Initialize drivetrain motors
    let left_motors = vec![
        Motor::new(peripherals.port_1, Gearset::SixToOne, Direction::Forward)?,
        Motor::new(peripherals.port_2, Gearset::SixToOne, Direction::Forward)?,
    ];
    let right_motors = vec![
        Motor::new(peripherals.port_3, Gearset::SixToOne, Direction::Reverse)?,
        Motor::new(peripherals.port_4, Gearset::SixToOne, Direction::Reverse)?,
    ];
    
    let left_group = MotorGroup::new(left_motors);
    let right_group = MotorGroup::new(right_motors);
    
    // Create chassis with 10.5 inch track width and 4 inch wheels
    let chassis = Chassis::new(left_group, right_group, 10.5, 4.0);
    
    // Initialize sensors (IMU only for this demo)
    let imu = InertialSensor::new(peripherals.port_10)?;
    let sensors = OdomSensors::new(None, None, None, None, Some(imu));
    
    // Calibrate chassis and start odometry
    println!("Calibrating chassis...");
    chassis.calibrate(sensors, true).await?;
    
    println!("Starting autonomous motion demo!");
    
    // Demo 1: Basic moveToPoint
    println!("Demo 1: Move to Point (24, 24)");
    match chassis.move_to_point(24.0, 24.0, 4000, None).await {
        Ok(result) => println!("Move to point result: {:?}", result),
        Err(error) => println!("Move to point error: {:?}", error),
    }
    
    // Wait a moment
    vexide::time::sleep(Duration::from_millis(1000)).await;
    
    // Demo 2: Advanced moveToPoint with custom parameters
    println!("Demo 2: Move to Point with custom parameters");
    let move_params = MoveToPointParams::new(0.0, 24.0)
        .max_speed(80.0)
        .forwards(false) // Move backwards
        .lateral_gains(12.0, 0.0, 0.2)
        .exit_conditions(ExitConditions::new(0.5, 150, 2.0, 600));
    
    match chassis.move_to_point(0.0, 24.0, 5000, Some(move_params)).await {
        Ok(result) => println!("Custom move to point result: {:?}", result),
        Err(error) => println!("Custom move to point error: {:?}", error),
    }
    
    vexide::time::sleep(Duration::from_millis(1000)).await;
    
    // Demo 3: MoveToPose with heading control
    println!("Demo 3: Move to Pose (24, 0) facing 90 degrees");
    let target_pose = Pose::new(24.0, 0.0, 90.0_f64.to_radians());
    
    match chassis.move_to_pose(target_pose, 6000, None).await {
        Ok(result) => println!("Move to pose result: {:?}", result),
        Err(error) => println!("Move to pose error: {:?}", error),
    }
    
    vexide::time::sleep(Duration::from_millis(1000)).await;
    
    // Demo 4: MoveToPose with custom boomerang parameters
    println!("Demo 4: Move to Pose with custom boomerang controller");
    let pose_params = MoveToPoseParams::with_target(0.0, 0.0, 0.0)
        .max_speed(100.0)
        .boomerang_params(12.0, 0.8) // Higher drift, longer lead
        .lateral_gains(8.0, 0.0, 0.3)
        .angular_gains(4.0, 0.0, 0.25);
    
    match chassis.move_to_pose_coords(0.0, 0.0, 0.0, 7000, Some(pose_params)).await {
        Ok(result) => println!("Custom move to pose result: {:?}", result),
        Err(error) => println!("Custom move to pose error: {:?}", error),
    }
    
    // Demo complete
    println!("Motion demo completed!");
    
    // Show final pose
    let final_pose = chassis.get_pose_degrees();
    println!("Final pose: x={:.2}, y={:.2}, theta={:.2}°", 
             final_pose.x(), final_pose.y(), final_pose.theta());
    
    Ok(())
}