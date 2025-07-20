use crate::test_framework::{HardwareSimulator};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use core::time::Duration;

pub struct MockVEXBrain {
    motors: HashMap<u8, MockMotor>,
    sensors: HashMap<String, f64>,
    current_time: Duration,
    time_scale: f64,
}

#[derive(Debug, Clone)]
pub struct MockMotor {
    pub velocity: f64,
    pub position: f64,
    pub temperature: f64,
    pub current: f64,
    pub voltage: f64,
    pub power: f64,
}

impl MockMotor {
    pub fn new() -> Self {
        Self {
            velocity: 0.0,
            position: 0.0,
            temperature: 25.0,
            current: 0.0,
            voltage: 12.0,
            power: 0.0,
        }
    }

    pub fn update(&mut self, dt: f64) {
        self.position += self.velocity * dt;
        self.current = (self.velocity.abs() * 0.1).min(2.5);
        self.power = self.voltage * self.current;
        self.temperature = 25.0 + (self.power * 0.5).min(50.0);
    }
}

impl MockVEXBrain {
    pub fn new() -> Self {
        let mut brain = Self {
            motors: HashMap::new(),
            sensors: HashMap::new(),
            current_time: Duration::from_secs(0),
            time_scale: 1.0,
        };

        for port in 1..=21 {
            brain.motors.insert(port, MockMotor::new());
        }

        brain.sensors.insert("left_encoder".to_string(), 0.0);
        brain.sensors.insert("right_encoder".to_string(), 0.0);
        brain.sensors.insert("back_encoder".to_string(), 0.0);
        brain.sensors.insert("imu_heading".to_string(), 0.0);
        brain.sensors.insert("imu_pitch".to_string(), 0.0);
        brain.sensors.insert("imu_roll".to_string(), 0.0);
        brain.sensors.insert("battery_voltage".to_string(), 12.0);

        brain
    }

    pub fn get_motor(&self, port: u8) -> Option<&MockMotor> {
        self.motors.get(&port)
    }

    pub fn get_motor_mut(&mut self, port: u8) -> Option<&mut MockMotor> {
        self.motors.get_mut(&port)
    }

    pub fn set_time_scale(&mut self, scale: f64) {
        self.time_scale = scale;
    }

    pub fn simulate_drivetrain_physics(&mut self, dt: f64) {
        let left_velocity = self.get_motor_velocity(1);
        let right_velocity = self.get_motor_velocity(2);
        
        let linear_velocity = (left_velocity + right_velocity) / 2.0;
        let angular_velocity = (right_velocity - left_velocity) / 0.3; // 30cm wheelbase
        
        let current_heading = self.get_sensor_value("imu_heading").unwrap_or(0.0);
        let new_heading = current_heading + angular_velocity * dt;
        self.set_sensor_value("imu_heading", new_heading);
        
        let left_distance = left_velocity * dt;
        let right_distance = right_velocity * dt;
        
        let current_left = self.get_sensor_value("left_encoder").unwrap_or(0.0);
        let current_right = self.get_sensor_value("right_encoder").unwrap_or(0.0);
        
        self.set_sensor_value("left_encoder", current_left + left_distance);
        self.set_sensor_value("right_encoder", current_right + right_distance);
    }
}

impl HardwareSimulator for MockVEXBrain {
    fn reset(&mut self) {
        for motor in self.motors.values_mut() {
            *motor = MockMotor::new();
        }
        
        self.sensors.insert("left_encoder".to_string(), 0.0);
        self.sensors.insert("right_encoder".to_string(), 0.0);
        self.sensors.insert("back_encoder".to_string(), 0.0);
        self.sensors.insert("imu_heading".to_string(), 0.0);
        self.sensors.insert("imu_pitch".to_string(), 0.0);
        self.sensors.insert("imu_roll".to_string(), 0.0);
        
        self.current_time = Duration::from_secs(0);
    }

    fn set_motor_velocity(&mut self, port: u8, velocity: f64) {
        if let Some(motor) = self.motors.get_mut(&port) {
            motor.velocity = velocity.clamp(-600.0, 600.0); // VEX motor limits
        }
    }

    fn get_motor_velocity(&self, port: u8) -> f64 {
        self.motors.get(&port).map_or(0.0, |m| m.velocity)
    }

    fn set_sensor_value(&mut self, sensor: &str, value: f64) {
        self.sensors.insert(sensor.to_string(), value);
    }

    fn get_sensor_value(&self, sensor: &str) -> Option<f64> {
        self.sensors.get(sensor).copied()
    }

    fn advance_time(&mut self, duration: Duration) {
        let dt_secs = duration.as_secs_f64() * self.time_scale;
        self.current_time += duration;
        
        for motor in self.motors.values_mut() {
            motor.update(dt_secs);
        }
        
        self.simulate_drivetrain_physics(dt_secs);
    }

    fn current_time(&self) -> Duration {
        self.current_time
    }
}

pub fn create_mock_brain() -> Arc<Mutex<dyn HardwareSimulator>> {
    Arc::new(Mutex::new(MockVEXBrain::new()))
}