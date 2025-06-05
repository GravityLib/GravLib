use std::ops::{Add, Sub, Mul, Div};

/// A pose in 2D space: position (x, y) and heading (theta).
/// All operations ignore heading unless otherwise noted.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

impl Pose {
    /// Create a new pose. Theta defaults to 0.0.
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self { x, y, theta }
    }

    /// Linearly interpolate between two poses (ignores heading).
    pub fn lerp(&self, other: Pose, t: f32) -> Pose {
        Pose {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            theta: self.theta, // heading is not interpolated
        }
    }

    /// Get the distance between two poses (ignores heading).
    pub fn distance(&self, other: Pose) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    /// Get the angle between two poses (ignores heading), in radians.
    pub fn angle(&self, other: Pose) -> f32 {
        (other.y - self.y).atan2(other.x - self.x)
    }

    /// Rotate this pose by an angle (in radians, ignores heading).
    pub fn rotate(&self, angle: f32) -> Pose {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        Pose {
            x: self.x * cos_a - self.y * sin_a,
            y: self.x * sin_a + self.y * cos_a,
            theta: self.theta,
        }
    }
}

// Operator overloading

impl Add for Pose {
    type Output = Pose;
    /// Add two poses (heading is not modified, taken from self).
    fn add(self, other: Pose) -> Pose {
        Pose {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta,
        }
    }
}

impl Sub for Pose {
    type Output = Pose;
    /// Subtract two poses (heading is not modified, taken from self).
    fn sub(self, other: Pose) -> Pose {
        Pose {
            x: self.x - other.x,
            y: self.y - other.y,
            theta: self.theta,
        }
    }
}

/// Dot product of two poses (ignores heading).
impl Mul for Pose {
    type Output = f32;
    fn mul(self, other: Pose) -> f32 {
        self.x * other.x + self.y * other.y
    }
}

/// Multiply pose by a scalar (ignores heading).
impl Mul<f32> for Pose {
    type Output = Pose;
    fn mul(self, rhs: f32) -> Pose {
        Pose {
            x: self.x * rhs,
            y: self.y * rhs,
            theta: self.theta,
        }
    }
}

/// Divide pose by a scalar (ignores heading).
impl Div<f32> for Pose {
    type Output = Pose;
    fn div(self, rhs: f32) -> Pose {
        Pose {
            x: self.x / rhs,
            y: self.y / rhs,
            theta: self.theta,
        }
    }
}

/// Format a pose as a string.
pub fn format_as(pose: &Pose) -> String {
    format!("Pose {{ x: {:.3}, y: {:.3}, theta: {:.3} }}", pose.x, pose.y, pose.theta)
}
