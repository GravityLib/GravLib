use core::ops::{Add, Sub, Mul, Div};
use libm::hypot;


#[derive(Debug, Copy, Clone)]
pub struct Pose {
    x: f64,
    y: f64,
    theta: f64, // orientation in radians
}


/// Implementing arithmetic operations for Pose Vector
impl Add for Pose {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Pose {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta + other.theta,
        }
    }
}

impl Sub for Pose {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Pose {
            x: self.x - other.x,
            y: self.y - other.y,
            theta: self.theta - other.theta,
        }
    }
}

impl Mul<f64> for Pose {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self::Output {
        Pose {
            x: self.x * scalar,
            y: self.y * scalar,
            theta: self.theta * scalar,
        }
    }
}

impl Div<f64> for Pose {
    type Output = Self;

    fn div(self, scalar: f64) -> Self::Output {
        Pose {
            x: self.x / scalar,
            y: self.y / scalar,
            theta: self.theta / scalar,
        }
    }
}

impl Pose {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Pose { x, y, theta }
    }

    pub fn linear_interpolate(&self, other: Pose, value: f64) -> Pose {
        Pose {
            x: self.x + (other.x - self.x) * value,
            y: self.y + (other.y - self.y) * value,
            theta: self.theta,
        }
    }

    pub fn distance_to(&self, other: &Pose) -> f64 {
        hypot(self.x - other.x, self.y - other.y)
    }

    pub fn angle_to(&self, other: &Pose) -> f64 {
        let dy = other.y - self.y;
        let dx = other.x - self.x;
        // libm’s `atan2f` works just like C’s atan2f(…)
        libm::atan2(dy, dx)
    }

    pub fn rotate_by(&self, angle: f64) -> Pose {
        let c = libm::cos(angle);
        let s = libm::sin(angle);
        Pose {
            x: self.x * c - self.y * s,
            y: self.x * s + self.y * c,
            theta: self.theta,
        }
    }
}

