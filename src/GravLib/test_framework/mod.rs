pub mod mocks;
pub mod test_runner;
pub mod assertions;
pub mod hardware_sim;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use core::time::Duration;

pub trait TestCase {
    fn name(&self) -> &str;
    fn run(&mut self, context: &mut TestContext) -> TestResult;
    fn setup(&mut self) -> Result<(), TestError> { Ok(()) }
    fn teardown(&mut self) -> Result<(), TestError> { Ok(()) }
}

#[derive(Debug, Clone)]
pub struct TestResult {
    pub passed: bool,
    pub message: String,
    pub duration: Duration,
    pub assertions: Vec<AssertionResult>,
}

#[derive(Debug, Clone)]
pub struct AssertionResult {
    pub passed: bool,
    pub expected: String,
    pub actual: String,
    pub message: String,
}

#[derive(Debug)]
pub enum TestError {
    SetupFailed(String),
    TeardownFailed(String),
    HardwareMissing(String),
    TimeoutError(String),
    AssertionFailed(String),
}

pub struct TestContext {
    pub hardware_sim: Arc<Mutex<dyn HardwareSimulator>>,
    pub data: HashMap<String, Box<dyn std::any::Any>>,
    pub start_time: std::time::Instant,
}

impl TestContext {
    pub fn new(hardware_sim: Arc<Mutex<dyn HardwareSimulator>>) -> Self {
        Self {
            hardware_sim,
            data: HashMap::new(),
            start_time: std::time::Instant::now(),
        }
    }

    pub fn set_data<T: 'static>(&mut self, key: &str, value: T) {
        self.data.insert(key.to_string(), Box::new(value));
    }

    pub fn get_data<T: 'static>(&self, key: &str) -> Option<&T> {
        self.data.get(key)?.downcast_ref::<T>()
    }

    pub fn elapsed(&self) -> Duration {
        self.start_time.elapsed()
    }
}

pub trait HardwareSimulator: Send + Sync {
    fn reset(&mut self);
    fn set_motor_velocity(&mut self, port: u8, velocity: f64);
    fn get_motor_velocity(&self, port: u8) -> f64;
    fn set_sensor_value(&mut self, sensor: &str, value: f64);
    fn get_sensor_value(&self, sensor: &str) -> Option<f64>;
    fn advance_time(&mut self, duration: Duration);
    fn current_time(&self) -> Duration;
}

pub struct TestSuite {
    pub name: String,
    pub tests: Vec<Box<dyn TestCase>>,
    pub setup: Option<fn() -> Result<(), TestError>>,
    pub teardown: Option<fn() -> Result<(), TestError>>,
}

impl TestSuite {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            tests: Vec::new(),
            setup: None,
            teardown: None,
        }
    }

    pub fn add_test<T: TestCase + 'static>(&mut self, test: T) {
        self.tests.push(Box::new(test));
    }

    pub fn with_setup(mut self, setup: fn() -> Result<(), TestError>) -> Self {
        self.setup = Some(setup);
        self
    }

    pub fn with_teardown(mut self, teardown: fn() -> Result<(), TestError>) -> Self {
        self.teardown = Some(teardown);
        self
    }
}