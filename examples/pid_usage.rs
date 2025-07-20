// Example of how to use the PID controller with proper ownership patterns

use crate::GravLib::pid::{PID, Gains};

fn pid_usage_example() {
    // Method 1: Create PID with constructor
    let mut pid_controller = PID::new(
        1.0,   // k_p
        0.1,   // k_i  
        0.05,  // k_d
        10.0,  // windup_range
        true   // sign_flip_reset
    );

    // Method 2: Create PID using Gains struct (more organized for complex tuning)
    let gains = Gains::new(1.2, 0.15, 0.08);
    let mut pid_controller2 = PID::new(
        gains.k_p, 
        gains.k_i, 
        gains.k_d, 
        15.0, 
        false
    );

    // Using getter methods (no ownership issues!)
    println!("PID Gains: P={}, I={}, D={}", 
             pid_controller.k_p(), 
             pid_controller.k_i(), 
             pid_controller.k_d());

    // Get all gains as a struct (Copy trait allows this)
    let current_gains = pid_controller.gains();
    println!("Gains struct: {:?}", current_gains);

    // Using setter methods
    pid_controller.set_k_p(1.5);
    pid_controller.set_windup_range(20.0);

    // Set all gains at once
    let new_gains = Gains::new(2.0, 0.2, 0.1);
    pid_controller2.set_gains(new_gains);

    // Runtime control loop usage
    loop {
        let target = 100.0;
        let current_position = get_sensor_reading(); // Your sensor reading
        let error = target - current_position;

        let control_output = pid_controller.update(error);
        
        // Apply control output to motors
        apply_control_output(control_output);

        // Debug information
        println!("Error: {}, Output: {}, Integral: {}", 
                 error, 
                 control_output, 
                 pid_controller.integral());

        // Exit condition
        if error.abs() < 0.1 { break; }
    }

    // Reset PID for next use
    pid_controller.reset();
}

// Mock functions for example
fn get_sensor_reading() -> f64 { 50.0 }
fn apply_control_output(_output: f64) {}