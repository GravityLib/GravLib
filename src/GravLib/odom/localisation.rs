use core::time::Duration;

use vexide::devices::adi::digital;
use vexide::{
    devices::smart::InertialSensor,
};

use alloc::{sync::Arc, vec::Vec};
use vexide::io::println;
use spin::Mutex;
use alloc::vec;

use uom::{si::f64::Angle, ConstZero};

use libm;

use vexide::devices::math::Point2;
use vexide::prelude::*;
use vexide::devices::display::{self, *};

use crate::GravLib::odom::sensors::{TrackingWheel, Sensors};

use alloc::format;

#[derive(Debug, Clone, Copy)]
pub struct Pose {
    x: f64,
    y: f64,
    theta: f64, // in degrees
}

impl Pose {
    pub const fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }

    #[inline]
    pub const fn get_position(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.theta)
    }

    pub fn set_pose(&mut self, x: f64, y: f64, theta: f64) {
        self.x = x;
        self.y = y;
        self.theta = theta;
    }
}

pub struct Localisation {
    pub sensors: Arc<Mutex<Sensors>>,
    pub m_pose: Arc<Mutex<Pose>>,
    prev_vertical_total: Vec<f64>,
    prev_horizontal_total: Vec<f64>,
    prev_imu_heading: f64,
    // Add circular buffers for smoothing (no_std friendly)
    vertical_history: [f64; 5],
    horizontal_history: [f64; 5],
    history_index: usize,
}

// Arc tracking constants
const MIN_ROTATION_THRESHOLD: f64 = 0.001; // radians
const OUTLIER_THRESHOLD: f64 = 2.0; // standard deviations
const HISTORY_SIZE: usize = 5;
const MAX_DELTA_THRESHOLD: f64 = 100.0; // mm, reject obviously wrong readings

/// Fast approximate sine using Taylor series (no_std optimization) (ChatGPT says to use Taylor series tf is that T_T)
#[inline]
fn fast_sin(x: f64) -> f64 {
    libm::sin(x)
}

/// Fast approximate cosine using Taylor series (no_std optimization)
#[inline]
fn fast_cos(x: f64) -> f64 {
    // Use libm for accuracy in robotics applications  
    libm::cos(x)
}

/// Compute arc parameters from wheel deltas
#[inline]
fn compute_arc_parameters(
    delta_theta: f64,
    vertical_delta: f64,
    horizontal_delta: f64,
    vertical_offset: f64,
    horizontal_offset: f64,
) -> (f64, f64, f64, f64) {
    if delta_theta.abs() < MIN_ROTATION_THRESHOLD {
        // Straight line case - direct return for performance
        return (vertical_delta, horizontal_delta, 0.0, 0.0);
    }
    
    // Arc case - compute radius for each tracking direction
    let vertical_radius = vertical_delta / delta_theta + vertical_offset;
    let horizontal_radius = horizontal_delta / delta_theta + horizontal_offset;
    
    // Chord length for each axis
    let half_theta = delta_theta * 0.5;
    let chord_factor = 2.0 * fast_sin(half_theta);
    let vertical_chord = chord_factor * vertical_radius;
    let horizontal_chord = chord_factor * horizontal_radius;
    
    (vertical_chord, horizontal_chord, vertical_radius, horizontal_radius)
}

/// No-std friendly outlier rejection using simple statistics
fn reject_outliers_and_average(deltas: &[f64], threshold: f64) -> f64 {
    let len = deltas.len();
    
    if len == 0 {
        return 0.0;
    }
    
    if len == 1 {
        return deltas[0];
    }
    
    // Simple mean and standard deviation calculation
    let mean: f64 = deltas.iter().sum::<f64>() / len as f64;
    
    if len == 2 {
        return mean; // Not enough data for outlier rejection
    }
    
    // Calculate standard deviation without heap allocation
    let variance: f64 = deltas.iter()
        .map(|x| {
            let diff = x - mean;
            diff * diff
        })
        .sum::<f64>() / (len - 1) as f64;
    
    let std_dev = libm::sqrt(variance);
    
    // Filter and average (avoiding heap allocation)
    let mut sum = 0.0;
    let mut count = 0;
    
    for &delta in deltas {
        if (delta - mean).abs() <= threshold * std_dev {
            sum += delta;
            count += 1;
        }
    }
    
    if count == 0 {
        mean // Fallback to mean if all rejected
    } else {
        sum / count as f64
    }
}

/// Normalize angle to [-180, 180] degrees
#[inline]
fn normalize_angle_deg(angle: f64) -> f64 {
    let mut normalized = angle % 360.0;
    if normalized > 180.0 {
        normalized -= 360.0;
    } else if normalized < -180.0 {
        normalized += 360.0;
    }
    normalized
}

impl Localisation {
    pub fn new(sensors: Arc<Mutex<Sensors>>) -> Self {
        let num_v = sensors.lock().vertical_wheels.len();
        let num_h = sensors.lock().horizontal_wheels.len();
        let initial_heading = sensors.lock().imu.lock().heading().unwrap_or(0.0);
        
        Self {
            sensors,
            m_pose: Arc::new(Mutex::new(Pose::new())),
            prev_vertical_total: vec![0.0; num_v],
            prev_horizontal_total: vec![0.0; num_h],
            prev_imu_heading: initial_heading,
            vertical_history: [0.0; HISTORY_SIZE],
            horizontal_history: [0.0; HISTORY_SIZE],
            history_index: 0,
        }
    }

    /// Get current pose (thread-safe)
    pub fn get_pose(&self) -> Pose {
        *self.m_pose.lock()
    }

    /// Reset pose to origin
    pub fn reset_pose(&mut self) {
        let mut pose = self.m_pose.lock();
        pose.set_pose(0.0, 0.0, 0.0);
    }

    /// Set pose to specific position
    pub fn set_pose(&mut self, x: f64, y: f64, theta: f64) {
        let mut pose = self.m_pose.lock();
        pose.set_pose(x, y, theta);
    }

    pub fn update(&mut self, display: &mut Display) {
        loop {
            // 1. Read deltas from each wheel
            let mut vertical_deltas = Vec::new();
            let mut horizontal_deltas = Vec::new();

            {
                let s = self.sensors.lock();

                // Vertical wheels
                for (i, w) in s.vertical_wheels.iter().enumerate() {
                    let total = w.lock().get_distance_travelled();
                    let delta = total - self.prev_vertical_total[i];
                    
                    // Sanity check for sensor errors
                    if delta.abs() < MAX_DELTA_THRESHOLD {
                        vertical_deltas.push(delta);
                    }
                    
                    self.prev_vertical_total[i] = total;
                }

                // Horizontal wheels
                for (i, w) in s.horizontal_wheels.iter().enumerate() {
                    let total = w.lock().get_distance_travelled();
                    let delta = total - self.prev_horizontal_total[i];
                    
                    // Sanity check for sensor errors
                    if delta.abs() < MAX_DELTA_THRESHOLD {
                        horizontal_deltas.push(delta);
                    }
                    
                    self.prev_horizontal_total[i] = total;
                }
            }

            // Enhanced outlier rejection
            let vertical_delta = reject_outliers_and_average(&vertical_deltas, OUTLIER_THRESHOLD);
            let horizontal_delta = reject_outliers_and_average(&horizontal_deltas, OUTLIER_THRESHOLD);

            // Add to circular buffer for smoothing
            self.vertical_history[self.history_index] = vertical_delta;
            self.horizontal_history[self.history_index] = horizontal_delta;
            self.history_index = (self.history_index + 1) % HISTORY_SIZE;

            // 2. Get heading change with error handling
            let current_heading = match self.sensors.lock().imu.lock().heading() {
                Ok(heading) => heading,
                Err(_) => {
                    println!("IMU error - using previous heading");
                    self.prev_imu_heading
                }
            };

            // 3. Calculate heading change with wrap-around handling
            let delta_theta_deg = normalize_angle_deg(current_heading - self.prev_imu_heading);
            let delta_theta_rad = delta_theta_deg.to_radians();
            self.prev_imu_heading = current_heading;

            // 4. Get wheel offsets
            let (vertical_offset, horizontal_offset) = {
                let s = self.sensors.lock();
                let v_off = if !s.vertical_wheels.is_empty() {
                    s.vertical_wheels[0].lock().get_offset()
                } else {
                    0.0
                };
                let h_off = if !s.horizontal_wheels.is_empty() {
                    s.horizontal_wheels[0].lock().get_offset()
                } else {
                    0.0
                };
                (v_off, h_off)
            };

            // 5. Compute arc parameters
            let (local_x, local_y, _, _) = compute_arc_parameters(
                delta_theta_rad,
                vertical_delta,
                horizontal_delta,
                vertical_offset,
                horizontal_offset,
            );

            // 6. Transform to global coordinates
            let old_theta = self.m_pose.lock().theta;
            let avg_heading_rad = (old_theta + current_heading * 0.5).to_radians();
            
            let cos_h = fast_cos(avg_heading_rad);
            let sin_h = fast_sin(avg_heading_rad);
            
            // Coordinate transformation: X is forward/back, Y is left/right
            let global_dx = local_x * cos_h - local_y * sin_h;
            let global_dy = local_x * sin_h + local_y * cos_h;

            // 7. Update pose
            {
                let mut pose = self.m_pose.lock();
                
                pose.x += global_dx;
                pose.y += global_dy;
                pose.theta = current_heading;
                
                // Efficient debug output
                if delta_theta_rad.abs() > MIN_ROTATION_THRESHOLD {
                    println!(
                        "Arc: Δθ={:.2}° | Pose: x={:.3}, y={:.3}, θ={:.1}°",
                        delta_theta_deg, pose.x, pose.y, pose.theta
                    );
                } else {
                    println!(
                        "Line: Δ({:.2},{:.2}) | Pose: x={:.3}, y={:.3}, θ={:.1}°",
                        global_dx, global_dy, pose.x, pose.y, pose.theta
                    );
                }
            }

            // 8. Update display efficiently
            self.update_display(display, delta_theta_rad);

            // Sleep to maintain update rate
            vexide::time::sleep(Duration::from_millis(10));
        }
    }

    /// Separate display update for better code organization
    fn update_display(&self, display: &mut Display, delta_theta_rad: f64) {
        let pose = self.m_pose.lock();
        let (x, y, theta) = pose.get_position();
        drop(pose);

        display.set_render_mode(RenderMode::DoubleBuffered);
        display.erase(Rgb::new(0, 0, 0));
        
        let motion_type = if delta_theta_rad.abs() > MIN_ROTATION_THRESHOLD {
            "ARC"
        } else {
            "LINE"
        };
        
        display.draw_text(
            &Text::new(
                &format!(
                    "X: {:+.3}\nY: {:+.3}\nθ: {:.1}°\nMode: {}",
                    x, y, theta, motion_type
                ),
                Font::new(FontSize::MEDIUM, FontFamily::Monospace),
                Point2::<i16>::from([10, 10]),
            ),
            Rgb::new(255, 255, 255),
            Some(Rgb::new(0, 0, 0)),
        );
    }
}

// Additional utility functions optimized for no_std

/// Estimate instantaneous curvature from differential drive
#[inline]
pub fn estimate_curvature(left_vel: f64, right_vel: f64, wheelbase: f64) -> f64 {
    let vel_diff = right_vel - left_vel;
    let avg_vel = (left_vel + right_vel) * 0.5;
    
    if vel_diff.abs() < 0.001 || avg_vel.abs() < 0.001 {
        0.0 // Straight line or stationary
    } else {
        vel_diff / (wheelbase * avg_vel)
    }
}

/// Predict future position along an arc (no heap allocation)
pub fn predict_arc_position(
    current_pose: &Pose,
    arc_length: f64,
    curvature: f64,
) -> (f64, f64, f64) {
    if curvature.abs() < 0.001 {
        // Straight line prediction
        let theta_rad = current_pose.theta.to_radians();
        let dx = arc_length * fast_cos(theta_rad);
        let dy = arc_length * fast_sin(theta_rad);
        (current_pose.x + dx, current_pose.y + dy, current_pose.theta)
    } else {
        // Arc prediction
        let radius = 1.0 / curvature;
        let delta_theta = arc_length / radius;
        
        let chord_length = 2.0 * radius * fast_sin(delta_theta * 0.5);
        let chord_angle = current_pose.theta.to_radians() + delta_theta * 0.5;
        
        let dx = chord_length * fast_cos(chord_angle);
        let dy = chord_length * fast_sin(chord_angle);
        
        (
            current_pose.x + dx,
            current_pose.y + dy,
            normalize_angle_deg(current_pose.theta + delta_theta.to_degrees()),
        )
    }
}

/// Calculate distance between two poses
#[inline]
pub fn pose_distance(pose1: &Pose, pose2: &Pose) -> f64 {
    let dx = pose2.x - pose1.x;
    let dy = pose2.y - pose1.y;
    libm::sqrt(dx * dx + dy * dy)
}

/// Calculate angular difference between two poses (in degrees)
#[inline]
pub fn angular_difference(pose1: &Pose, pose2: &Pose) -> f64 {
    normalize_angle_deg(pose2.theta - pose1.theta)
}