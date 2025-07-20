use vexide::prelude::*;
use biggrlib::GravLib::drivebase::{
    Chassis, OdomSensors, TaskConfig, 
    task_loop, configure_sensors, start_task
};
use biggrlib::GravLib::actuator::MotorGroup;

/// Simple example showing BigGravLib's background task system
#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals);

    // === BASIC USAGE (LemLib-compatible) ===

    // Create motor groups
    let left_motors = MotorGroup::new(vec![
        Motor::new(robot.peripherals.port_1, Gearset::EighteenToOne, Direction::Forward).unwrap(),
        Motor::new(robot.peripherals.port_2, Gearset::EighteenToOne, Direction::Forward).unwrap(),
    ]);
    
    let right_motors = MotorGroup::new(vec![
        Motor::new(robot.peripherals.port_3, Gearset::EighteenToOne, Direction::Reverse).unwrap(),
        Motor::new(robot.peripherals.port_4, Gearset::EighteenToOne, Direction::Reverse).unwrap(),
    ]);

    // Create chassis (track width: 13.5", wheel diameter: 4.0")
    let chassis = Chassis::new(left_motors, right_motors, 342.9, 101.6);

    // Configure sensors - simplified setup with just IMU
    let sensors = OdomSensors::new(
        None,  // Vertical left
        None,  // Vertical right  
        None,  // Horizontal
        None,  // No second horizontal
        Some(InertialSensor::new(robot.peripherals.port_8).unwrap()),  // IMU
    );

    // Basic setup (equivalent to LemLib)
    match chassis.calibrate(sensors, true).await {
        Ok(_) => println!("Odometry calibrated successfully"),
        Err(e) => {
            println!("Calibration failed: {}", e);
            return;
        }
    }
    
    // Spawn background task (equivalent to LemLib::init())
    vexide::async_runtime::spawn(task_loop()).detach();

    println!("BigGravLib task system started successfully!");

    // Demonstrate runtime monitoring
    for i in 0..10 {
        vexide::time::sleep(vexide::time::Duration::from_secs(1)).await;
        
        // Get current pose
        let pose = biggrlib::GravLib::drivebase::odometry::get_pose(false);
        let stats = biggrlib::GravLib::drivebase::task_system::get_stats();
        
        println!("Iteration {}: Updates: {}, Pose: ({:.2}, {:.2}, {:.2}°)", 
                 i + 1, stats.updates_completed, pose.x(), pose.y(), pose.theta());
    }

    // Start competition
    robot.compete().await;
}

/// Helper function to create tracking wheels (for future use)
fn create_tracking_wheel(port: SmartPort, diameter: f64, offset: f64, gear_ratio: f64) -> biggrlib::GravLib::drivebase::TrackingWheel {
    let encoder = RotationSensor::new(port).expect("Failed to create rotation sensor");
    biggrlib::GravLib::drivebase::TrackingWheel::new(encoder, diameter, offset, gear_ratio)
}