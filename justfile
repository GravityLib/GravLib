# BigGravLib Test Automation Commands
# Requires just: cargo install just

# Default recipe - run all tests
default: test-all

# Run all tests with verbose output
test-all:
    @echo "🚀 Running all BigGravLib tests..."
    cargo test --workspace --verbose
    @echo "✅ All tests completed"

# Run unit tests only
test-unit:
    @echo "🧪 Running unit tests..."
    cargo test --lib --verbose

# Run integration tests only  
test-integration:
    @echo "🔧 Running integration tests..."
    cargo test --test integration_tests --verbose

# Run PID controller tests specifically
test-pid:
    @echo "🎛️ Running PID controller tests..."
    cargo test --test pid_tests --verbose

# Run the test automation framework
test-framework:
    @echo "🤖 Running test automation framework..."
    cargo run --bin test_runner_example

# Run tests with hardware simulation
test-sim:
    @echo "🎮 Running tests with hardware simulation..."
    RUST_TEST_THREADS=1 cargo test --verbose -- --test-threads=1

# Run performance benchmarks (if available)
bench:
    @echo "⚡ Running performance benchmarks..."
    cargo bench

# Clean test artifacts
clean-test:
    @echo "🧹 Cleaning test artifacts..."
    cargo clean
    rm -rf target/debug/deps/*test*

# Generate test coverage report
coverage:
    @echo "📊 Generating test coverage report..."
    cargo tarpaulin --out html --output-dir target/coverage
    @echo "📈 Coverage report generated at target/coverage/tarpaulin-report.html"

# Continuous testing - watch for changes
watch:
    @echo "👀 Watching for changes and running tests..."
    cargo watch -x "test --workspace"

# Run static analysis with clippy
lint:
    @echo "🔍 Running Clippy analysis..."
    cargo clippy --workspace --all-targets --all-features -- -D warnings

# Format code
fmt:
    @echo "✨ Formatting code..."
    cargo fmt --all

# Check code without building
check:
    @echo "🔎 Checking code..."
    cargo check --workspace --all-targets --all-features

# Full CI pipeline
ci: fmt lint check test-all
    @echo "🎯 CI pipeline completed successfully"

# Setup development environment
setup:
    @echo "⚙️ Setting up development environment..."
    cargo install cargo-watch cargo-tarpaulin
    @echo "🎉 Development environment ready!"

# Quick test for debugging - runs fastest tests first
test-quick:
    @echo "⚡ Running quick tests for debugging..."
    cargo test --lib pid_tests::tests::test_gains_modification --verbose
    cargo test --lib pid_tests::tests::test_individual_gain_setters --verbose

# Test with different optimization levels
test-debug:
    @echo "🐛 Running tests in debug mode..."
    cargo test --workspace

test-release:
    @echo "🚀 Running tests in release mode..."
    cargo test --workspace --release

# Memory leak detection (if valgrind available)
test-memory:
    @echo "🧠 Running memory leak tests..."
    cargo test --workspace --target x86_64-unknown-linux-gnu
    # Note: Requires valgrind setup for embedded targets

# Cross-compilation test for VEX V5
test-cross:
    @echo "🎯 Cross-compiling for VEX V5..."
    cargo check --target armv7a-vex-v5

# Help - show all available commands
help:
    @echo "BigGravLib Test Automation Commands:"
    @echo "===================================="
    @just --list