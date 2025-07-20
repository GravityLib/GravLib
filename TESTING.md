# BigGravLib Test Automation Framework

A comprehensive test automation system for VEX robotics code that works without requiring physical hardware.

## 🚀 Quick Start

### Prerequisites
```bash
# Install just command runner
cargo install just

# Install coverage tools (optional)
cargo install cargo-tarpaulin
```

### Running Tests
```bash
# Run all tests
just test-all

# Run specific test categories  
just test-unit          # Unit tests only
just test-integration   # Integration tests only
just test-pid          # PID controller tests

# Run with hardware simulation
just test-sim

# Generate coverage report
just coverage
```

## 🏗️ Framework Architecture

The test framework consists of several key components:

### 1. **Test Framework Core** (`src/GravLib/test_framework/`)
- `mod.rs` - Core traits and structures
- `test_runner.rs` - Test execution engine
- `assertions.rs` - Assertion helpers and macros
- `mocks.rs` - Mock VEX brain implementation
- `hardware_sim.rs` - Physics simulation system

### 2. **Mock Hardware System**
Simulates VEX V5 brain and sensors without requiring physical hardware:

- **MockVEXBrain**: Simulates motors, encoders, IMU, and battery
- **PhysicsSimulator**: Realistic drivetrain physics and odometry
- **Environment Simulation**: Field obstacles and collision detection

### 3. **Test Categories**

#### Unit Tests (`tests/pid_tests.rs`)
- PID controller mathematical accuracy
- Integral windup prevention
- Slew rate limiting
- Sign flip reset functionality

#### Integration Tests (`tests/integration_tests.rs`)  
- Drivebase motion control
- Odometry position tracking
- Turn-in-place operations
- Obstacle collision handling
- Autonomous path following

## 🧪 Writing Tests

### Creating a New Test Case
```rust
use crate::test_framework::*;

struct MyCustomTest {
    name: String,
}

impl TestCase for MyCustomTest {
    fn name(&self) -> &str {
        &self.name
    }

    fn setup(&mut self) -> Result<(), TestError> {
        // Test initialization
        Ok(())
    }

    fn run(&mut self, context: &mut TestContext) -> TestResult {
        let mut assertions = Vec::new();
        
        // Access hardware simulator
        let mut hardware_sim = context.hardware_sim.lock().unwrap();
        
        // Set motor speeds
        hardware_sim.set_motor_velocity(1, 100.0);
        
        // Advance simulation time
        hardware_sim.advance_time(Duration::from_secs(1));
        
        // Check results
        let distance = hardware_sim.get_sensor_value("left_encoder").unwrap_or(0.0);
        assertions.push(Assert::greater_than(0.0, distance, "Motor should move forward"));
        
        TestResult {
            passed: assertions.iter().all(|a| a.passed),
            message: "Test completed".to_string(),
            duration: context.elapsed(),
            assertions,
        }
    }

    fn teardown(&mut self) -> Result<(), TestError> {
        // Test cleanup
        Ok(())
    }
}
```

### Using Assertions
```rust
use crate::test_framework::assertions::*;

// Exact equality
let result = Assert::equals(expected, actual, "Values should match");

// Floating point approximation
let result = Assert::approximately(10.0, 10.001, 0.01, "Should be close");

// Range checking
let result = Assert::within_range(5.0, 15.0, actual_value, "Should be in range");

// Boolean conditions
let result = Assert::is_true(condition, "Condition should be true");

// Using macros for quick assertions
assert_approx!(expected, actual, tolerance)?;
assert_range!(min, max, actual)?;
```

### Hardware Simulation Features
```rust
// Motor control
hardware_sim.set_motor_velocity(1, 150.0);  // Port 1, 150 RPM
let current_velocity = hardware_sim.get_motor_velocity(1);

// Sensor readings
let encoder_value = hardware_sim.get_sensor_value("left_encoder")?;
let heading = hardware_sim.get_sensor_value("imu_heading")?;
let x_pos = hardware_sim.get_sensor_value("x_position")?;

// Time advancement
hardware_sim.advance_time(Duration::from_millis(100));

// Environment setup
let field_sim = create_field_sim(); // Includes obstacles
```

## 🎯 Test Suites

### Creating Test Suites
```rust
pub fn create_my_test_suite() -> TestSuite {
    let mut suite = TestSuite::new("My Custom Tests");
    
    suite.add_test(MyCustomTest::new());
    suite.add_test(AnotherTest::new());
    
    suite.with_setup(|| {
        // Suite-wide setup
        Ok(())
    }).with_teardown(|| {
        // Suite-wide cleanup  
        Ok(())
    })
}
```

### Running Custom Suites
```rust
fn main() {
    let config = TestConfig {
        timeout: Duration::from_secs(30),
        verbose: true,
        hardware_simulation: true,
        ..Default::default()
    };
    
    let mut runner = TestRunner::new().with_config(config);
    runner.add_suite(create_my_test_suite());
    
    let result = runner.run();
    
    if !result.success() {
        std::process::exit(1);
    }
}
```

## 🔧 Advanced Features

### Physics Simulation
The framework includes a realistic physics simulator:

- **Drivetrain Dynamics**: Wheel slip, acceleration limits, friction
- **Odometry Calculation**: Three-wheel odometry with realistic sensor noise
- **Field Simulation**: Obstacles, walls, and collision detection
- **Battery Simulation**: Voltage drop under load

### Performance Testing
```rust
// Measure execution time
let start = context.start_time;
// ... run code ...
let execution_time = context.elapsed();

// Check performance constraints
assertions.push(Assert::less_than(
    Duration::from_millis(100), 
    execution_time, 
    "Should complete within 100ms"
));
```

### Custom Hardware Mocks
```rust
impl HardwareSimulator for MyCustomHardware {
    fn reset(&mut self) { /* Reset state */ }
    
    fn set_motor_velocity(&mut self, port: u8, velocity: f64) {
        // Custom motor simulation
    }
    
    fn get_sensor_value(&self, sensor: &str) -> Option<f64> {
        // Custom sensor simulation
    }
    
    // ... other methods
}
```

## 📊 CI/CD Integration

The framework includes GitHub Actions workflow (`.github/workflows/test.yml`):

- **Automated Testing**: Runs on push/PR
- **Cross-compilation**: Checks VEX V5 target compilation
- **Code Coverage**: Generates coverage reports
- **Performance Monitoring**: Benchmark tracking

### Local CI Simulation
```bash
# Run complete CI pipeline locally
just ci

# Individual steps
just fmt      # Format check
just lint     # Clippy analysis
just check    # Compilation check  
just test-all # Full test suite
```

## 🐛 Debugging

### Test Output
```bash
# Verbose output with timing
just test-all

# Debug specific test
cargo test test_name -- --nocapture

# Run single test with details
cargo test --test pid_tests test_pid_basic -- --exact --nocapture
```

### Common Issues

1. **Test Timeouts**: Increase timeout in `TestConfig`
2. **Hardware Simulation**: Ensure mock hardware is reset between tests
3. **Floating Point**: Use `Assert::approximately()` for float comparisons
4. **Memory Usage**: Monitor for leaks in long-running tests

## 📈 Metrics and Reporting

### Coverage Reports
```bash
just coverage
# Open target/coverage/tarpaulin-report.html
```

### Test Metrics
- **Pass Rate**: Percentage of tests passing
- **Execution Time**: Performance tracking
- **Coverage**: Code coverage percentage
- **Regression Detection**: Automated failure detection

## 🔄 Continuous Development

### Watch Mode
```bash
# Auto-run tests on file changes
just watch
```

### Quick Development Cycle
```bash
# Fast iteration testing
just test-quick

# Debug mode testing
just test-debug
```

This test automation framework enables comprehensive testing of robotics code without physical hardware, ensuring reliable and maintainable VEX V5 robot programs.