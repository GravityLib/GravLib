use core::ops::{Add, Sub, Mul, Div};
use libm::hypot;


#[derive(Debug, Copy, Clone)]
#[repr(C, align(32))] // Align for potential SIMD operations
pub struct Pose {
    x: f64,
    y: f64,
    theta: f64, // orientation in radians
    _padding: f64, // Padding for 32-byte alignment (4 x f64)
}


/// Implementing arithmetic operations for Pose Vector
impl Add for Pose {
    type Output = Self;

    #[inline(always)]
    fn add(self, other: Self) -> Self::Output {
        Pose {
            x: self.x + other.x,
            y: self.y + other.y,
            theta: self.theta + other.theta,
            _padding: 0.0,
        }
    }
}

impl Sub for Pose {
    type Output = Self;

    #[inline(always)]
    fn sub(self, other: Self) -> Self::Output {
        Pose {
            x: self.x - other.x,
            y: self.y - other.y,
            theta: self.theta - other.theta,
            _padding: 0.0,
        }
    }
}

impl Mul<f64> for Pose {
    type Output = Self;

    #[inline(always)]
    fn mul(self, scalar: f64) -> Self::Output {
        Pose {
            x: self.x * scalar,
            y: self.y * scalar,
            theta: self.theta * scalar,
            _padding: 0.0,
        }
    }
}

impl Div<f64> for Pose {
    type Output = Self;

    #[inline(always)]
    fn div(self, scalar: f64) -> Self::Output {
        // Use multiplication by reciprocal for better performance
        let inv_scalar = 1.0 / scalar;
        Pose {
            x: self.x * inv_scalar,
            y: self.y * inv_scalar,
            theta: self.theta * inv_scalar,
            _padding: 0.0,
        }
    }
}

impl Pose {
    #[inline(always)]
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Pose { x, y, theta, _padding: 0.0 }
    }

    #[inline(always)]
    pub fn x(&self) -> f64 {
        self.x
    }

    #[inline(always)]
    pub fn y(&self) -> f64 {
        self.y
    }

    #[inline(always)]
    pub fn theta(&self) -> f64 {
        self.theta
    }

    #[inline(always)]
    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    #[inline(always)]
    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    #[inline(always)]
    pub fn set_theta(&mut self, theta: f64) {
        self.theta = theta;
    }

    #[inline(always)]
    pub fn linear_interpolate(&self, other: Pose, value: f64) -> Pose {
        Pose {
            x: self.x + (other.x - self.x) * value,
            y: self.y + (other.y - self.y) * value,
            theta: self.theta,
            _padding: 0.0,
        }
    }

    #[inline(always)]
    pub fn distance_to(&self, other: &Pose) -> f64 {
        hypot(self.x - other.x, self.y - other.y)
    }

    #[inline(always)]
    pub fn angle_to(&self, other: &Pose) -> f64 {
        let dy = other.y - self.y;
        let dx = other.x - self.x;
        // libm's `atan2` works just like C's atan2
        libm::atan2(dy, dx)
    }

    #[inline(always)]
    pub fn rotate_by(&self, angle: f64) -> Pose {
        let c = libm::cos(angle);
        let s = libm::sin(angle);
        Pose {
            x: self.x * c - self.y * s,
            y: self.x * s + self.y * c,
            theta: self.theta,
            _padding: 0.0,
        }
    }
}

