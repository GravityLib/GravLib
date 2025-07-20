use crate::test_framework::*;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use std::collections::HashMap;

pub struct TestRunner {
    suites: Vec<TestSuite>,
    config: TestConfig,
    results: Vec<SuiteResult>,
}

#[derive(Debug, Clone)]
pub struct TestConfig {
    pub timeout: core::time::Duration,
    pub parallel: bool,
    pub verbose: bool,
    pub stop_on_failure: bool,
    pub hardware_simulation: bool,
}

impl Default for TestConfig {
    fn default() -> Self {
        Self {
            timeout: core::time::Duration::from_secs(30),
            parallel: false,
            verbose: true,
            stop_on_failure: false,
            hardware_simulation: true,
        }
    }
}

#[derive(Debug)]
pub struct SuiteResult {
    pub name: String,
    pub tests: Vec<TestResult>,
    pub duration: core::time::Duration,
    pub passed: usize,
    pub failed: usize,
    pub skipped: usize,
}

impl SuiteResult {
    fn new(name: String) -> Self {
        Self {
            name,
            tests: Vec::new(),
            duration: core::time::Duration::from_secs(0),
            passed: 0,
            failed: 0,
            skipped: 0,
        }
    }

    fn add_result(&mut self, result: TestResult) {
        if result.passed {
            self.passed += 1;
        } else {
            self.failed += 1;
        }
        self.tests.push(result);
    }
}

impl TestRunner {
    pub fn new() -> Self {
        Self {
            suites: Vec::new(),
            config: TestConfig::default(),
            results: Vec::new(),
        }
    }

    pub fn with_config(mut self, config: TestConfig) -> Self {
        self.config = config;
        self
    }

    pub fn add_suite(&mut self, suite: TestSuite) {
        self.suites.push(suite);
    }

    pub fn run(&mut self) -> RunnerResult {
        let start_time = Instant::now();
        self.results.clear();

        if self.config.verbose {
            println!("🚀 Starting test run with {} suite(s)", self.suites.len());
        }

        let mut total_passed = 0;
        let mut total_failed = 0;
        let mut total_skipped = 0;

        for suite in &mut self.suites {
            let suite_result = self.run_suite(suite);
            
            total_passed += suite_result.passed;
            total_failed += suite_result.failed;
            total_skipped += suite_result.skipped;
            
            if self.config.verbose {
                self.print_suite_result(&suite_result);
            }
            
            self.results.push(suite_result);
            
            if self.config.stop_on_failure && total_failed > 0 {
                break;
            }
        }

        let total_duration = start_time.elapsed();
        
        if self.config.verbose {
            self.print_summary(total_passed, total_failed, total_skipped, total_duration);
        }

        RunnerResult {
            total_tests: total_passed + total_failed + total_skipped,
            passed: total_passed,
            failed: total_failed,
            skipped: total_skipped,
            duration: total_duration,
            suite_results: self.results.clone(),
        }
    }

    fn run_suite(&mut self, suite: &mut TestSuite) -> SuiteResult {
        let start_time = Instant::now();
        let mut suite_result = SuiteResult::new(suite.name.clone());

        if self.config.verbose {
            println!("\n📁 Running suite: {}", suite.name);
        }

        if let Some(setup) = suite.setup {
            if let Err(e) = setup() {
                if self.config.verbose {
                    println!("❌ Suite setup failed: {:?}", e);
                }
                suite_result.failed = suite.tests.len();
                return suite_result;
            }
        }

        let hardware_sim = if self.config.hardware_simulation {
            crate::test_framework::mocks::create_mock_brain()
        } else {
            crate::test_framework::mocks::create_mock_brain() // Fallback for now
        };

        for test in &mut suite.tests {
            let test_result = self.run_test(test.as_mut(), &hardware_sim);
            suite_result.add_result(test_result);
            
            if self.config.stop_on_failure && suite_result.failed > 0 {
                break;
            }
        }

        if let Some(teardown) = suite.teardown {
            if let Err(e) = teardown() {
                if self.config.verbose {
                    println!("⚠️ Suite teardown failed: {:?}", e);
                }
            }
        }

        suite_result.duration = start_time.elapsed();
        suite_result
    }

    fn run_test(&self, test: &mut dyn TestCase, hardware_sim: &Arc<Mutex<dyn HardwareSimulator>>) -> TestResult {
        let test_name = test.name().to_string();
        let start_time = Instant::now();
        
        if self.config.verbose {
            print!("  🧪 {}: ", test_name);
        }

        hardware_sim.lock().unwrap().reset();
        
        if let Err(e) = test.setup() {
            let result = TestResult {
                passed: false,
                message: format!("Setup failed: {:?}", e),
                duration: start_time.elapsed(),
                assertions: vec![],
            };
            
            if self.config.verbose {
                println!("❌ SETUP FAILED");
            }
            return result;
        }

        let mut context = TestContext::new(hardware_sim.clone());
        let test_result = test.run(&mut context);

        if let Err(e) = test.teardown() {
            if self.config.verbose {
                println!("⚠️ Teardown warning: {:?}", e);
            }
        }

        let duration = start_time.elapsed();
        let mut final_result = test_result;
        final_result.duration = duration;

        if self.config.verbose {
            let status = if final_result.passed { "✅ PASS" } else { "❌ FAIL" };
            println!("{} ({:.2}ms)", status, duration.as_millis());
            
            if !final_result.passed && !final_result.message.is_empty() {
                println!("    💬 {}", final_result.message);
            }
        }

        final_result
    }

    fn print_suite_result(&self, result: &SuiteResult) {
        println!("  📊 {} - {} passed, {} failed, {} skipped ({:.2}ms)",
            result.name, result.passed, result.failed, result.skipped, 
            result.duration.as_millis());
    }

    fn print_summary(&self, passed: usize, failed: usize, skipped: usize, duration: std::time::Duration) {
        println!("\n" + &"=".repeat(60));
        println!("🎯 TEST SUMMARY");
        println!("=".repeat(60));
        println!("✅ Passed:  {}", passed);
        println!("❌ Failed:  {}", failed);
        println!("⏭️  Skipped: {}", skipped);
        println!("⏱️  Duration: {:.2}s", duration.as_secs_f32());
        
        let total = passed + failed + skipped;
        if total > 0 {
            let pass_rate = (passed as f32 / total as f32) * 100.0;
            println!("📈 Pass rate: {:.1}%", pass_rate);
        }
        
        println!("=".repeat(60));
        
        if failed == 0 {
            println!("🎉 All tests passed!");
        } else {
            println!("💥 {} test(s) failed", failed);
        }
    }
}

#[derive(Debug)]
pub struct RunnerResult {
    pub total_tests: usize,
    pub passed: usize,
    pub failed: usize,
    pub skipped: usize,
    pub duration: std::time::Duration,
    pub suite_results: Vec<SuiteResult>,
}

impl RunnerResult {
    pub fn success(&self) -> bool {
        self.failed == 0
    }
    
    pub fn pass_rate(&self) -> f32 {
        if self.total_tests == 0 {
            0.0
        } else {
            (self.passed as f32 / self.total_tests as f32) * 100.0
        }
    }
}