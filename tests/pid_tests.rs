use gravity::GravLib::pid::{PID, Gains};
use std::time::Duration;

#[path = "../src/test_framework/mod.rs"]
mod test_framework;

use test_framework::*;
use test_framework::assertions::*;

struct PIDBasicTest {
    pid: PID,
}

impl TestCase for PIDBasicTest {
    fn name(&self) -> &str {
        "PID Basic Functionality"
    }

    fn setup(&mut self) -> Result<(), TestError> {
        self.pid.reset();
        Ok(())
    }

    fn run(&mut self, _context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let result = self.pid.update(10.0);
        assertions.push(Assert::equals(50.0, result, "Proportional term should be 10.0 * 5.0 = 50.0"));
        
        if !assertions.last().unwrap().passed {
            passed = false;
            message = "Proportional calculation failed".to_string();
        }

        let gains = self.pid.gains();
        assertions.push(Assert::equals(5.0, gains.k_p, "Kp should be 5.0"));
        assertions.push(Assert::equals(0.1, gains.k_i, "Ki should be 0.1"));
        assertions.push(Assert::equals(0.05, gains.k_d, "Kd should be 0.05"));

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            if message.is_empty() {
                message = "Gains verification failed".to_string();
            }
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_millis(0),
            assertions,
        }
    }
}

impl PIDBasicTest {
    fn new() -> Self {
        Self {
            pid: PID::new(5.0, 0.1, 0.05, 50.0, true),
        }
    }
}

struct PIDIntegralWindupTest {
    pid: PID,
}

impl TestCase for PIDIntegralWindupTest {
    fn name(&self) -> &str {
        "PID Integral Windup Prevention"
    }

    fn setup(&mut self) -> Result<(), TestError> {
        self.pid.reset();
        Ok(())
    }

    fn run(&mut self, _context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        for _ in 0..10 {
            self.pid.update(100.0);
            std::thread::sleep(Duration::from_millis(10));
        }

        let integral_before = self.pid.integral();
        
        self.pid.update(100.0);
        std::thread::sleep(Duration::from_millis(10));
        let integral_after = self.pid.integral();

        assertions.push(Assert::equals(0.0, integral_after, "Integral should be reset due to windup range"));
        
        if !assertions.last().unwrap().passed {
            passed = false;
            message = format!("Windup prevention failed. Integral before: {}, after: {}", 
                integral_before, integral_after);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_millis(0),
            assertions,
        }
    }
}

impl PIDIntegralWindupTest {
    fn new() -> Self {
        Self {
            pid: PID::new(1.0, 0.5, 0.0, 50.0, false), // Windup at error > 50
        }
    }
}

struct PIDSlewRateTest {
    pid: PID,
}

impl TestCase for PIDSlewRateTest {
    fn name(&self) -> &str {
        "PID Slew Rate Limiting"
    }

    fn setup(&mut self) -> Result<(), TestError> {
        self.pid.reset();
        Ok(())
    }

    fn run(&mut self, _context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        let first_output = self.pid.update(100.0);
        std::thread::sleep(Duration::from_millis(100)); // 100ms delay
        
        let second_output = self.pid.update(100.0);
        let output_change = (second_output - first_output).abs();
        
        let max_expected_change = 50.0 * 0.1; // 50 units/sec * 0.1 sec
        
        assertions.push(Assert::less_than(max_expected_change + 1.0, output_change, 
            "Output change should be limited by slew rate"));
        
        if !assertions.last().unwrap().passed {
            passed = false;
            message = format!("Slew rate limiting failed. Change: {}, Expected max: {}", 
                output_change, max_expected_change);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_millis(0),
            assertions,
        }
    }
}

impl PIDSlewRateTest {
    fn new() -> Self {
        Self {
            pid: PID::new_with_slew(2.0, 0.0, 0.0, 0.0, false, 50.0), // 50 units/sec slew rate
        }
    }
}

struct PIDSignFlipResetTest {
    pid: PID,
}

impl TestCase for PIDSignFlipResetTest {
    fn name(&self) -> &str {
        "PID Sign Flip Reset"
    }

    fn setup(&mut self) -> Result<(), TestError> {
        self.pid.reset();
        Ok(())
    }

    fn run(&mut self, _context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        let mut passed = true;
        let mut message = String::new();

        for _ in 0..5 {
            self.pid.update(10.0);
            std::thread::sleep(Duration::from_millis(20));
        }

        let integral_before = self.pid.integral();
        assertions.push(Assert::greater_than(0.0, integral_before, "Integral should have accumulated"));

        self.pid.update(-10.0);
        let integral_after = self.pid.integral();
        
        assertions.push(Assert::equals(0.0, integral_after, "Integral should reset on sign flip"));

        if !assertions.iter().all(|a| a.passed) {
            passed = false;
            message = format!("Sign flip reset failed. Integral before: {}, after: {}", 
                integral_before, integral_after);
        }

        TestResult {
            passed,
            message,
            duration: Duration::from_millis(0),
            assertions,
        }
    }
}

impl PIDSignFlipResetTest {
    fn new() -> Self {
        Self {
            pid: PID::new(1.0, 1.0, 0.0, 0.0, true), // Sign flip reset enabled
        }
    }
}

pub fn create_pid_test_suite() -> TestSuite {
    let mut suite = TestSuite::new("PID Controller Tests");
    
    suite.add_test(PIDBasicTest::new());
    suite.add_test(PIDIntegralWindupTest::new());
    suite.add_test(PIDSlewRateTest::new());
    suite.add_test(PIDSignFlipResetTest::new());
    
    suite
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_basic() {
        let mut test = PIDBasicTest::new();
        let hardware_sim = test_framework::mocks::create_mock_brain();
        let mut context = TestContext::new(hardware_sim);
        let result = test.run(&mut context);
        assert!(result.passed, "PID basic test failed: {}", result.message);
    }

    #[test]
    fn test_pid_windup() {
        let mut test = PIDIntegralWindupTest::new();
        let hardware_sim = test_framework::mocks::create_mock_brain();
        let mut context = TestContext::new(hardware_sim);
        let result = test.run(&mut context);
        assert!(result.passed, "PID windup test failed: {}", result.message);
    }

    #[test]
    fn test_gains_modification() {
        let mut pid = PID::new(1.0, 2.0, 3.0, 0.0, false);
        
        let gains = Gains::new(5.0, 6.0, 7.0);
        pid.set_gains(gains);
        
        assert_eq!(pid.k_p(), 5.0);
        assert_eq!(pid.k_i(), 6.0);
        assert_eq!(pid.k_d(), 7.0);
    }

    #[test]
    fn test_individual_gain_setters() {
        let mut pid = PID::new(1.0, 2.0, 3.0, 0.0, false);
        
        pid.set_k_p(10.0);
        pid.set_k_i(20.0);
        pid.set_k_d(30.0);
        
        assert_eq!(pid.k_p(), 10.0);
        assert_eq!(pid.k_i(), 20.0);
        assert_eq!(pid.k_d(), 30.0);
    }
}