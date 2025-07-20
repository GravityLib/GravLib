use std::time::Duration;

#[path = "../src/test_framework/mod.rs"]
mod test_framework;
mod pid_tests;
mod integration_tests;

use test_framework::*;
use test_framework::test_runner::*;
use pid_tests::create_pid_test_suite;
use integration_tests::create_integration_test_suite;

fn main() {
    println!("🤖 BigGravLib Test Automation Framework");
    println!("========================================");

    let config = TestConfig {
        timeout: Duration::from_secs(60),
        parallel: false,
        verbose: true,
        stop_on_failure: false,
        hardware_simulation: true,
    };

    let mut runner = TestRunner::new().with_config(config);

    runner.add_suite(create_pid_test_suite());
    runner.add_suite(create_integration_test_suite());

    let result = runner.run();

    if result.success() {
        println!("\n🎉 All tests passed! 🎉");
        std::process::exit(0);
    } else {
        println!("\n💥 Some tests failed!");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod runner_tests {
    use super::*;

    #[test]
    fn test_runner_basic() {
        let mut runner = TestRunner::new();
        runner.add_suite(create_pid_test_suite());
        
        let result = runner.run();
        
        assert!(result.total_tests > 0, "Should have run some tests");
        println!("Test result: {} passed, {} failed", result.passed, result.failed);
    }

    #[test]
    fn test_config_validation() {
        let config = TestConfig::default();
        assert_eq!(config.timeout, Duration::from_secs(30));
        assert!(config.verbose);
        assert!(config.hardware_simulation);
    }
}