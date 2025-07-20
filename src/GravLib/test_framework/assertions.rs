use crate::test_framework::{AssertionResult, TestError};

pub struct Assert;

impl Assert {
    pub fn equals<T: PartialEq + std::fmt::Debug>(expected: T, actual: T, message: &str) -> AssertionResult {
        let passed = expected == actual;
        AssertionResult {
            passed,
            expected: format!("{:?}", expected),
            actual: format!("{:?}", actual),
            message: message.to_string(),
        }
    }

    pub fn not_equals<T: PartialEq + std::fmt::Debug>(not_expected: T, actual: T, message: &str) -> AssertionResult {
        let passed = not_expected != actual;
        AssertionResult {
            passed,
            expected: format!("not {:?}", not_expected),
            actual: format!("{:?}", actual),
            message: message.to_string(),
        }
    }

    pub fn approximately(expected: f64, actual: f64, tolerance: f64, message: &str) -> AssertionResult {
        let passed = (expected - actual).abs() <= tolerance;
        AssertionResult {
            passed,
            expected: format!("{:.6} ± {:.6}", expected, tolerance),
            actual: format!("{:.6}", actual),
            message: message.to_string(),
        }
    }

    pub fn greater_than<T: PartialOrd + std::fmt::Debug>(threshold: T, actual: T, message: &str) -> AssertionResult {
        let passed = actual > threshold;
        AssertionResult {
            passed,
            expected: format!("> {:?}", threshold),
            actual: format!("{:?}", actual),
            message: message.to_string(),
        }
    }

    pub fn less_than<T: PartialOrd + std::fmt::Debug>(threshold: T, actual: T, message: &str) -> AssertionResult {
        let passed = actual < threshold;
        AssertionResult {
            passed,
            expected: format!("< {:?}", threshold),
            actual: format!("{:?}", actual),
            message: message.to_string(),
        }
    }

    pub fn within_range<T: PartialOrd + std::fmt::Debug>(min: T, max: T, actual: T, message: &str) -> AssertionResult {
        let passed = actual >= min && actual <= max;
        AssertionResult {
            passed,
            expected: format!("[{:?}, {:?}]", min, max),
            actual: format!("{:?}", actual),
            message: message.to_string(),
        }
    }

    pub fn is_true(condition: bool, message: &str) -> AssertionResult {
        AssertionResult {
            passed: condition,
            expected: "true".to_string(),
            actual: condition.to_string(),
            message: message.to_string(),
        }
    }

    pub fn is_false(condition: bool, message: &str) -> AssertionResult {
        AssertionResult {
            passed: !condition,
            expected: "false".to_string(),
            actual: condition.to_string(),
            message: message.to_string(),
        }
    }

    pub fn contains<T: AsRef<str>>(haystack: T, needle: &str, message: &str) -> AssertionResult {
        let haystack_str = haystack.as_ref();
        let passed = haystack_str.contains(needle);
        AssertionResult {
            passed,
            expected: format!("contains '{}'", needle),
            actual: haystack_str.to_string(),
            message: message.to_string(),
        }
    }

    pub fn is_some<T>(option: &Option<T>, message: &str) -> AssertionResult {
        let passed = option.is_some();
        AssertionResult {
            passed,
            expected: "Some(_)".to_string(),
            actual: if passed { "Some(_)" } else { "None" }.to_string(),
            message: message.to_string(),
        }
    }

    pub fn is_none<T>(option: &Option<T>, message: &str) -> AssertionResult {
        let passed = option.is_none();
        AssertionResult {
            passed,
            expected: "None".to_string(),
            actual: if passed { "None" } else { "Some(_)" }.to_string(),
            message: message.to_string(),
        }
    }

    pub fn is_ok<T, E>(result: &Result<T, E>, message: &str) -> AssertionResult {
        let passed = result.is_ok();
        AssertionResult {
            passed,
            expected: "Ok(_)".to_string(),
            actual: if passed { "Ok(_)" } else { "Err(_)" }.to_string(),
            message: message.to_string(),
        }
    }

    pub fn is_err<T, E>(result: &Result<T, E>, message: &str) -> AssertionResult {
        let passed = result.is_err();
        AssertionResult {
            passed,
            expected: "Err(_)".to_string(),
            actual: if passed { "Err(_)" } else { "Ok(_)" }.to_string(),
            message: message.to_string(),
        }
    }
}

pub fn assert_equals<T: PartialEq + std::fmt::Debug>(expected: T, actual: T, message: &str) -> Result<(), TestError> {
    let result = Assert::equals(expected, actual, message);
    if result.passed {
        Ok(())
    } else {
        Err(TestError::AssertionFailed(format!("{}: expected {}, got {}", 
            message, result.expected, result.actual)))
    }
}

pub fn assert_approximately(expected: f64, actual: f64, tolerance: f64, message: &str) -> Result<(), TestError> {
    let result = Assert::approximately(expected, actual, tolerance, message);
    if result.passed {
        Ok(())
    } else {
        Err(TestError::AssertionFailed(format!("{}: expected {}, got {}", 
            message, result.expected, result.actual)))
    }
}

pub fn assert_within_range<T: PartialOrd + std::fmt::Debug + Copy>(min: T, max: T, actual: T, message: &str) -> Result<(), TestError> {
    let result = Assert::within_range(min, max, actual, message);
    if result.passed {
        Ok(())
    } else {
        Err(TestError::AssertionFailed(format!("{}: expected {}, got {}", 
            message, result.expected, result.actual)))
    }
}

pub fn assert_true(condition: bool, message: &str) -> Result<(), TestError> {
    let result = Assert::is_true(condition, message);
    if result.passed {
        Ok(())
    } else {
        Err(TestError::AssertionFailed(message.to_string()))
    }
}

#[macro_export]
macro_rules! assert_approx {
    ($expected:expr, $actual:expr, $tolerance:expr) => {
        assert_approximately($expected, $actual, $tolerance, 
            &format!("Approximation failed at {}:{}", file!(), line!()))
    };
    ($expected:expr, $actual:expr, $tolerance:expr, $message:expr) => {
        assert_approximately($expected, $actual, $tolerance, $message)
    };
}

#[macro_export]
macro_rules! assert_range {
    ($min:expr, $max:expr, $actual:expr) => {
        assert_within_range($min, $max, $actual, 
            &format!("Range assertion failed at {}:{}", file!(), line!()))
    };
    ($min:expr, $max:expr, $actual:expr, $message:expr) => {
        assert_within_range($min, $max, $actual, $message)
    };
}