use std::time::Duration;
use std::thread;

mod test_framework;

use test_framework::*;
use test_framework::assertions::*;
use test_framework::hardware_sim::*;

struct DrivebaseMotionTest {
    name: String,
}

impl TestCase for DrivebaseMotionTest {
    fn name(&self) -> &str {
        &self.name
    }

    fn run(&mut self, context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let mut hardware_sim = context.hardware_sim.lock().unwrap();
        
        hardware_sim.set_motor_velocity(1, 100.0); // Left motor
        hardware_sim.set_motor_velocity(2, 100.0); // Right motor
        
        let initial_left_encoder = hardware_sim.get_sensor_value("left_encoder").unwrap_or(0.0);
        let initial_right_encoder = hardware_sim.get_sensor_value("right_encoder").unwrap_or(0.0);
        
        hardware_sim.advance_time(Duration::from_secs(1));
        
        let final_left_encoder = hardware_sim.get_sensor_value("left_encoder").unwrap_or(0.0);
        let final_right_encoder = hardware_sim.get_sensor_value("right_encoder").unwrap_or(0.0);
        
        let left_distance = final_left_encoder - initial_left_encoder;
        let right_distance = final_right_encoder - initial_right_encoder;
        
        assertions.push(Assert::greater_than(0.0, left_distance, "Left wheel should move forward"));
        assertions.push(Assert::greater_than(0.0, right_distance, "Right wheel should move forward"));
        
        assertions.push(Assert::approximately(left_distance, right_distance, 0.1, 
            "Left and right distances should be approximately equal for straight motion"));

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            message = format!("Motion test failed. Left: {:.3}, Right: {:.3}", 
                left_distance, right_distance);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_secs(1),
            assertions,
        }
    }
}

impl DrivebaseMotionTest {
    fn new() -> Self {
        Self {
            name: "Drivebase Forward Motion".to_string(),
        }
    }
}

struct OdometryTrackingTest {
    name: String,
}

impl TestCase for OdometryTrackingTest {
    fn name(&self) -> &str {
        &self.name
    }

    fn run(&mut self, context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let mut hardware_sim = context.hardware_sim.lock().unwrap();
        
        let initial_x = hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
        let initial_y = hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
        let initial_heading = hardware_sim.get_sensor_value("imu_heading").unwrap_or(0.0);
        
        hardware_sim.set_motor_velocity(1, 100.0);
        hardware_sim.set_motor_velocity(2, 100.0);
        hardware_sim.advance_time(Duration::from_secs(2));
        
        let final_x = hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
        let final_y = hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
        let final_heading = hardware_sim.get_sensor_value("imu_heading").unwrap_or(0.0);
        
        let distance_traveled = ((final_x - initial_x).powi(2) + (final_y - initial_y).powi(2)).sqrt();
        let heading_change = (final_heading - initial_heading).abs();
        
        assertions.push(Assert::greater_than(0.1, distance_traveled, "Robot should move forward"));
        assertions.push(Assert::less_than(5.0, heading_change, "Heading should remain relatively stable"));
        assertions.push(Assert::greater_than(initial_x, final_x, "X position should increase (forward motion)"));

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            message = format!("Odometry test failed. Distance: {:.3}, Heading change: {:.3}°", 
                distance_traveled, heading_change);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_secs(2),
            assertions,
        }
    }
}

impl OdometryTrackingTest {
    fn new() -> Self {
        Self {
            name: "Odometry Position Tracking".to_string(),
        }
    }
}

struct TurnInPlaceTest {
    name: String,
}

impl TestCase for TurnInPlaceTest {
    fn name(&self) -> &str {
        &self.name
    }

    fn run(&mut self, context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let mut hardware_sim = context.hardware_sim.lock().unwrap();
        
        let initial_x = hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
        let initial_y = hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
        let initial_heading = hardware_sim.get_sensor_value("imu_heading").unwrap_or(0.0);
        
        hardware_sim.set_motor_velocity(1, -100.0); // Left motor backwards
        hardware_sim.set_motor_velocity(2, 100.0);  // Right motor forwards
        hardware_sim.advance_time(Duration::from_millis(1570)); // Time to turn ~90 degrees
        
        let final_x = hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
        let final_y = hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
        let final_heading = hardware_sim.get_sensor_value("imu_heading").unwrap_or(0.0);
        
        let position_drift = ((final_x - initial_x).powi(2) + (final_y - initial_y).powi(2)).sqrt();
        let heading_change = (final_heading - initial_heading).abs();
        
        assertions.push(Assert::less_than(0.1, position_drift, "Position should not drift much during turn"));
        assertions.push(Assert::within_range(80.0, 100.0, heading_change, "Should turn approximately 90 degrees"));

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            message = format!("Turn test failed. Drift: {:.3}m, Heading change: {:.1}°", 
                position_drift, heading_change);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_millis(1570),
            assertions,
        }
    }
}

impl TurnInPlaceTest {
    fn new() -> Self {
        Self {
            name: "Turn in Place (90 degrees)".to_string(),
        }
    }
}

struct FieldObstacleTest {
    name: String,
}

impl TestCase for FieldObstacleTest {
    fn name(&self) -> &str {
        &self.name
    }

    fn run(&mut self, context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let field_sim = create_field_sim();
        let mut hardware_sim = field_sim.lock().unwrap();
        
        hardware_sim.set_motor_velocity(1, 200.0); // High speed towards obstacle
        hardware_sim.set_motor_velocity(2, 200.0);
        
        let initial_velocity = hardware_sim.get_motor_velocity(1);
        
        hardware_sim.advance_time(Duration::from_secs(3)); // Long enough to hit obstacle
        
        let final_left_velocity = hardware_sim.get_motor_velocity(1);
        let final_right_velocity = hardware_sim.get_motor_velocity(2);
        
        assertions.push(Assert::less_than(final_left_velocity, initial_velocity * 0.5, 
            "Velocity should decrease after collision"));
        assertions.push(Assert::less_than(final_right_velocity, initial_velocity * 0.5, 
            "Velocity should decrease after collision"));

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            message = format!("Obstacle collision test failed. Initial: {:.1}, Final L: {:.1}, Final R: {:.1}", 
                initial_velocity, final_left_velocity, final_right_velocity);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_secs(3),
            assertions,
        }
    }
}

impl FieldObstacleTest {
    fn new() -> Self {
        Self {
            name: "Field Obstacle Collision".to_string(),
        }
    }
}

struct AutonomousPathTest {
    name: String,
}

impl TestCase for AutonomousPathTest {
    fn name(&self) -> &str {
        &self.name
    }

    fn run(&mut self, context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let mut hardware_sim = context.hardware_sim.lock().unwrap();
        
        let waypoints = vec![
            (0.0, 0.0),   // Start
            (1.0, 0.0),   // Forward 1m
            (1.0, 1.0),   // Turn and move 1m right
            (0.0, 1.0),   // Back 1m
            (0.0, 0.0),   // Return to start
        ];
        
        for (i, &(target_x, target_y)) in waypoints.iter().enumerate().skip(1) {
            let current_x = hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
            let current_y = hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
            
            let distance_to_target = ((target_x - current_x).powi(2) + (target_y - current_y).powi(2)).sqrt();
            
            let mut steps = 0;
            while distance_to_target > 0.1 && steps < 100 {
                let error_x = target_x - hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
                let error_y = target_y - hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
                
                let forward_speed = (error_x * 100.0).clamp(-200.0, 200.0);
                let turn_speed = (error_y * 50.0).clamp(-100.0, 100.0);
                
                hardware_sim.set_motor_velocity(1, forward_speed - turn_speed);
                hardware_sim.set_motor_velocity(2, forward_speed + turn_speed);
                hardware_sim.advance_time(Duration::from_millis(100));
                
                steps += 1;
            }
            
            let final_x = hardware_sim.get_sensor_value("x_position").unwrap_or(0.0);
            let final_y = hardware_sim.get_sensor_value("y_position").unwrap_or(0.0);
            let final_distance = ((target_x - final_x).powi(2) + (target_y - final_y).powi(2)).sqrt();
            
            assertions.push(Assert::less_than(0.2, final_distance, 
                &format!("Should reach waypoint {} within tolerance", i)));
        }

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            message = "Autonomous path following failed".to_string();
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_secs(10),
            assertions,
        }
    }
}

impl AutonomousPathTest {
    fn new() -> Self {
        Self {
            name: "Autonomous Path Following".to_string(),
        }
    }
}

pub fn create_integration_test_suite() -> TestSuite {
    let mut suite = TestSuite::new("Integration Tests");
    
    suite.add_test(DrivebaseMotionTest::new());
    suite.add_test(OdometryTrackingTest::new());
    suite.add_test(TurnInPlaceTest::new());
    suite.add_test(FieldObstacleTest::new());
    suite.add_test(AutonomousPathTest::new());
    
    suite
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_drivebase_motion() {
        let mut test = DrivebaseMotionTest::new();
        let hardware_sim = create_physics_sim();
        let mut context = TestContext::new(hardware_sim);
        let result = test.run(&mut context);
        assert!(result.passed, "Drivebase motion test failed: {}", result.message);
    }

    #[test]
    fn test_odometry_tracking() {
        let mut test = OdometryTrackingTest::new();
        let hardware_sim = create_physics_sim();
        let mut context = TestContext::new(hardware_sim);
        let result = test.run(&mut context);
        assert!(result.passed, "Odometry tracking test failed: {}", result.message);
    }
}