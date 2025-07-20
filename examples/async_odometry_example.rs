use vexide::prelude::*;
use biggrlib::GravLib::drivebase::{
    Chassis, OdomSensors, OdometryTaskConfig, 
    odometry_task_loop, configure_odometry_sensors, start_odometry,
    get_odometry_stats
};
use biggrlib::GravLib::actuator::MotorGroup;

/// Example showing BigGravLib's LemLib-compatible background task system
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

    // Configure sensors
    let sensors = OdomSensors::new(
        Some(create_tracking_wheel(robot.peripherals.port_5, 63.5, -152.4, 1.0)),  // Vertical left
        Some(create_tracking_wheel(robot.peripherals.port_6, 63.5, 152.4, 1.0)),   // Vertical right  
        Some(create_tracking_wheel(robot.peripherals.port_7, 63.5, 0.0, 1.0)),     // Horizontal
        None,                                                                        // No second horizontal
        Some(InertialSensor::new(robot.peripherals.port_8).unwrap()),              // IMU
    );

    // Basic setup (equivalent to LemLib)
    chassis.calibrate(sensors, true).await.expect("Calibration failed");
    
    // Spawn background task (equivalent to LemLib::init())
    vexide::async_runtime::spawn(odometry_task_loop()).detach();

    // === ADVANCED USAGE ===

    // Custom configuration for high-precision applications
    let high_precision_config = OdometryTaskConfig {
        frequency_hz: 200,        // 5ms updates (2x faster than LemLib)
        priority: 250,            // Very high priority
        auto_start: true,
        max_jitter_us: 500,       // Strict timing tolerance
    };

    // Advanced calibration with custom config
    chassis.calibrate_with_config(sensors, high_precision_config, true).await
        .expect("Advanced calibration failed");

    // === RUNTIME CONTROL ===

    // Wait for initial setup
    vexide::time::sleep(Duration::from_secs(2)).await;

    // Demonstration of runtime control
    println!("Starting odometry performance demonstration...");

    // Run for 10 seconds and monitor performance
    for i in 0..100 {
        vexide::time::sleep(Duration::from_millis(100)).await;
        
        // Get current pose
        let pose = biggrlib::GravLib::drivebase::odometry::get_pose(false);
        
        // Get performance stats every second
        if i % 10 == 0 {
            if let Some(stats) = get_odometry_stats().await {
                println!("Performance Stats:");
                println!("  Updates: {}", stats.updates_completed);
                println!("  Avg update time: {}μs", 
                    if stats.updates_completed > 0 {
                        stats.total_update_time_us / stats.updates_completed as u64
                    } else { 0 }
                );
                println!("  Max update time: {}μs", stats.max_update_time_us);
                println!("  Jitter violations: {}", stats.jitter_violations);
                println!("  Pose: ({:.2}, {:.2}, {:.2}°)", pose.x(), pose.y(), pose.theta());
                println!();
            }
        }
    }

    // Start competition
    robot.compete().await;
}

/// Helper function to create tracking wheels
fn create_tracking_wheel(port: SmartPort, diameter: f64, offset: f64, gear_ratio: f64) -> biggrlib::GravLib::drivebase::TrackingWheel {
    let encoder = RotationSensor::new(port).expect("Failed to create rotation sensor");
    biggrlib::GravLib::drivebase::TrackingWheel::new(encoder, diameter, offset, gear_ratio)
}

/// Example autonomous routine using BigGravLib odometry
async fn autonomous_routine(chassis: &Chassis) {
    use biggrlib::GravLib::drivebase::odometry::{get_pose, set_pose};
    
    println!("Starting autonomous routine with BigGravLib odometry");
    
    // Set starting position
    set_pose(biggrlib::GravLib::drivebase::Pose::new(0.0, 0.0, 0.0), false);
    
    // Example movements with pose feedback
    for i in 0..5 {
        // Move forward
        chassis.drivetrain.arcade_drive(50.0, 0.0);
        vexide::time::sleep(Duration::from_millis(1000)).await;
        chassis.drivetrain.stop();
        
        let current_pose = get_pose(false);
        println!("Step {}: Robot at ({:.1}, {:.1}, {:.1}°)", 
                 i + 1, current_pose.x(), current_pose.y(), current_pose.theta());
        
        vexide::time::sleep(Duration::from_millis(500)).await;
    }
}

/// Example driver control with odometry monitoring
async fn driver_control(robot: Robot, chassis: Chassis) {
    use biggrlib::GravLib::drivebase::odometry::{get_pose, get_speed};
    
    println!("Starting driver control with odometry feedback");
    
    loop {
        // Get controller input
        let controller = robot.peripherals.primary_controller;
        let arcade_speed = controller.left_stick_y().unwrap_or(0) as f64;
        let arcade_turn = controller.right_stick_x().unwrap_or(0) as f64;
        
        // Apply control
        chassis.drivetrain.arcade_drive(arcade_speed, arcade_turn);
        
        // Display odometry on controller every 100ms
        if let Some(stats) = get_odometry_stats().await {
            if stats.updates_completed % 10 == 0 {
                let pose = get_pose(false);
                let speed = get_speed(false);
                
                let _ = controller.screen().draw_text_centered(
                    format!("Pos: ({:.1}, {:.1}, {:.0}°)", pose.x(), pose.y(), pose.theta()),
                    TextFormat::Small,
                    0
                ).await;
                
                let _ = controller.screen().draw_text_centered(
                    format!("Speed: {:.1} in/s", (speed.x().powi(2) + speed.y().powi(2)).sqrt()),
                    TextFormat::Small,
                    1
                ).await;
            }
        }
        
        vexide::time::sleep(Duration::from_millis(10)).await;
    }
}

/// Migration example showing LemLib to BigGravLib conversion
mod lemlib_migration_example {
    use super::*;
    
    // LemLib equivalent:
    // chassis.calibrate(); 
    // lemlib::init();
    
    // BigGravLib equivalent:
    async fn biggrlib_setup(chassis: Chassis, sensors: OdomSensors) {
        chassis.calibrate(sensors, true).await.expect("Calibration failed");
        vexide::async_runtime::spawn(odometry_task_loop()).detach();
    }
    
    // The odometry API remains identical:
    // - odometry::get_pose(false) 
    // - odometry::set_pose(pose, false)
    // - odometry::get_speed(false)
    // - odometry::get_local_speed(false)
    // - odometry::estimate_pose(time, false)
}