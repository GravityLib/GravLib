use crate::test_framework::{HardwareSimulator, TestError};
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use core::time::Duration;

pub struct PhysicsSimulator {
    pub drivetrain: DrivetrainSim,
    pub odometry: OdometrySim,
    pub environment: Environment,
}

#[derive(Debug, Clone)]
pub struct DrivetrainSim {
    pub left_velocity: f64,
    pub right_velocity: f64,
    pub wheelbase: f64,
    pub wheel_diameter: f64,
    pub max_velocity: f64,
    pub acceleration: f64,
    pub friction: f64,
}

impl Default for DrivetrainSim {
    fn default() -> Self {
        Self {
            left_velocity: 0.0,
            right_velocity: 0.0,
            wheelbase: 0.35, // 35cm wheelbase
            wheel_diameter: 0.1, // 10cm wheel diameter
            max_velocity: 200.0, // RPM
            acceleration: 100.0, // RPM/s
            friction: 0.1,
        }
    }
}

#[derive(Debug, Clone)]
pub struct OdometrySim {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub left_distance: f64,
    pub right_distance: f64,
    pub back_distance: f64,
    pub tracking_wheel_offset: f64,
}

impl Default for OdometrySim {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            left_distance: 0.0,
            right_distance: 0.0,
            back_distance: 0.0,
            tracking_wheel_offset: 0.15, // 15cm offset for back tracking wheel
        }
    }
}

#[derive(Debug, Clone)]
pub struct Environment {
    pub gravity: f64,
    pub field_friction: f64,
    pub air_resistance: f64,
    pub temperature: f64,
    pub battery_voltage: f64,
    pub obstacles: Vec<Obstacle>,
}

impl Default for Environment {
    fn default() -> Self {
        Self {
            gravity: 9.81,
            field_friction: 0.8,
            air_resistance: 0.01,
            temperature: 25.0,
            battery_voltage: 12.0,
            obstacles: Vec::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
    pub height: f64,
}

impl PhysicsSimulator {
    pub fn new() -> Self {
        Self {
            drivetrain: DrivetrainSim::default(),
            odometry: OdometrySim::default(),
            environment: Environment::default(),
        }
    }

    pub fn set_motor_speeds(&mut self, left_rpm: f64, right_rpm: f64) {
        self.drivetrain.left_velocity = left_rpm.clamp(-self.drivetrain.max_velocity, self.drivetrain.max_velocity);
        self.drivetrain.right_velocity = right_rpm.clamp(-self.drivetrain.max_velocity, self.drivetrain.max_velocity);
    }

    pub fn update(&mut self, dt: f64) {
        self.update_physics(dt);
        self.update_odometry(dt);
        self.check_collisions();
    }

    fn update_physics(&mut self, dt: f64) {
        let left_velocity = self.drivetrain.left_velocity;
        let right_velocity = self.drivetrain.right_velocity;
        
        let linear_velocity = (left_velocity + right_velocity) / 2.0;
        let angular_velocity = (right_velocity - left_velocity) / self.drivetrain.wheelbase;
        
        let cos_heading = self.odometry.heading.cos();
        let sin_heading = self.odometry.heading.sin();
        
        let linear_velocity_ms = linear_velocity * self.drivetrain.wheel_diameter * std::f64::consts::PI / 60.0;
        
        let dx = linear_velocity_ms * cos_heading * dt;
        let dy = linear_velocity_ms * sin_heading * dt;
        let dtheta = angular_velocity * dt;
        
        self.odometry.x += dx;
        self.odometry.y += dy;
        self.odometry.heading += dtheta;
        
        while self.odometry.heading > std::f64::consts::PI {
            self.odometry.heading -= 2.0 * std::f64::consts::PI;
        }
        while self.odometry.heading < -std::f64::consts::PI {
            self.odometry.heading += 2.0 * std::f64::consts::PI;
        }
    }

    fn update_odometry(&mut self, dt: f64) {
        let left_distance_delta = self.drivetrain.left_velocity * self.drivetrain.wheel_diameter * std::f64::consts::PI / 60.0 * dt;
        let right_distance_delta = self.drivetrain.right_velocity * self.drivetrain.wheel_diameter * std::f64::consts::PI / 60.0 * dt;
        
        self.odometry.left_distance += left_distance_delta;
        self.odometry.right_distance += right_distance_delta;
        
        let back_distance_delta = ((self.drivetrain.left_velocity + self.drivetrain.right_velocity) / 2.0) * 
                                  self.drivetrain.wheel_diameter * std::f64::consts::PI / 60.0 * dt;
        self.odometry.back_distance += back_distance_delta;
    }

    fn check_collisions(&mut self) {
        for obstacle in &self.environment.obstacles {
            let distance = ((self.odometry.x - obstacle.x).powi(2) + (self.odometry.y - obstacle.y).powi(2)).sqrt();
            if distance < obstacle.radius + 0.2 { // 20cm robot radius
                self.drivetrain.left_velocity *= 0.1; // Collision dampening
                self.drivetrain.right_velocity *= 0.1;
            }
        }
    }

    pub fn add_obstacle(&mut self, x: f64, y: f64, radius: f64, height: f64) {
        self.environment.obstacles.push(Obstacle { x, y, radius, height });
    }

    pub fn reset_position(&mut self, x: f64, y: f64, heading: f64) {
        self.odometry.x = x;
        self.odometry.y = y;
        self.odometry.heading = heading;
        self.odometry.left_distance = 0.0;
        self.odometry.right_distance = 0.0;
        self.odometry.back_distance = 0.0;
    }

    pub fn get_pose(&self) -> (f64, f64, f64) {
        (self.odometry.x, self.odometry.y, self.odometry.heading)
    }

    pub fn get_encoder_values(&self) -> (f64, f64, f64) {
        (self.odometry.left_distance, self.odometry.right_distance, self.odometry.back_distance)
    }
}

pub struct SimulatedTestBed {
    physics: PhysicsSimulator,
    sensors: HashMap<String, f64>,
    current_time: Duration,
}

impl SimulatedTestBed {
    pub fn new() -> Self {
        Self {
            physics: PhysicsSimulator::new(),
            sensors: HashMap::new(),
            current_time: Duration::from_secs(0),
        }
    }

    pub fn with_field_setup() -> Self {
        let mut testbed = Self::new();
        
        testbed.physics.add_obstacle(0.5, 0.5, 0.1, 0.3); // Goal post
        testbed.physics.add_obstacle(-0.5, -0.5, 0.1, 0.3); // Goal post
        testbed.physics.add_obstacle(1.0, 0.0, 0.15, 0.2); // Mobile goal
        
        testbed.physics.environment.field_friction = 0.9;
        testbed.physics.environment.battery_voltage = 12.6;
        
        testbed
    }

    pub fn get_physics_mut(&mut self) -> &mut PhysicsSimulator {
        &mut self.physics
    }

    pub fn get_physics(&self) -> &PhysicsSimulator {
        &self.physics
    }
}

impl HardwareSimulator for SimulatedTestBed {
    fn reset(&mut self) {
        self.physics = PhysicsSimulator::new();
        self.sensors.clear();
        self.current_time = Duration::from_secs(0);
    }

    fn set_motor_velocity(&mut self, port: u8, velocity: f64) {
        match port {
            1 => {
                let (_, right_vel) = (self.physics.drivetrain.left_velocity, self.physics.drivetrain.right_velocity);
                self.physics.set_motor_speeds(velocity, right_vel);
            }
            2 => {
                let (left_vel, _) = (self.physics.drivetrain.left_velocity, self.physics.drivetrain.right_velocity);
                self.physics.set_motor_speeds(left_vel, velocity);
            }
            _ => {} // Other motors not simulated in drivetrain
        }
    }

    fn get_motor_velocity(&self, port: u8) -> f64 {
        match port {
            1 => self.physics.drivetrain.left_velocity,
            2 => self.physics.drivetrain.right_velocity,
            _ => 0.0,
        }
    }

    fn set_sensor_value(&mut self, sensor: &str, value: f64) {
        self.sensors.insert(sensor.to_string(), value);
    }

    fn get_sensor_value(&self, sensor: &str) -> Option<f64> {
        match sensor {
            "left_encoder" => Some(self.physics.odometry.left_distance),
            "right_encoder" => Some(self.physics.odometry.right_distance),
            "back_encoder" => Some(self.physics.odometry.back_distance),
            "imu_heading" => Some(self.physics.odometry.heading.to_degrees()),
            "x_position" => Some(self.physics.odometry.x),
            "y_position" => Some(self.physics.odometry.y),
            "battery_voltage" => Some(self.physics.environment.battery_voltage),
            _ => self.sensors.get(sensor).copied(),
        }
    }

    fn advance_time(&mut self, duration: Duration) {
        let dt_secs = duration.as_secs_f64();
        self.physics.update(dt_secs);
        self.current_time += duration;
    }

    fn current_time(&self) -> Duration {
        self.current_time
    }
}

pub fn create_physics_sim() -> Arc<Mutex<dyn HardwareSimulator>> {
    Arc::new(Mutex::new(SimulatedTestBed::new()))
}

pub fn create_field_sim() -> Arc<Mutex<dyn HardwareSimulator>> {
    Arc::new(Mutex::new(SimulatedTestBed::with_field_setup()))
}