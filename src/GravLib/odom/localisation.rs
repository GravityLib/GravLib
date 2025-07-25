use core::time::Duration;

use vexide::{
    devices::smart::InertialSensor,
};

use alloc::{sync::Arc, vec::Vec};
use vexide::io::println;
use spin::Mutex;


use uom::{si::f64::Angle, ConstZero};

use libm;

use crate::GravLib::odom::sensors::{TrackingWheel, Sensors};

pub struct Pose {
    x: f64,
    y: f64,
    theta: f64, // in degrees
}

impl Pose {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }

    pub fn get_position(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.theta)
    }
}

pub struct Localisation {
    pub sensors: Arc<Mutex<Sensors>>,
    pub m_pose: Arc<Mutex<Pose>>,
}

fn calculate_wheel_heading(wheels: &Vec<Arc<Mutex<TrackingWheel>>>) -> f64 {
    // if not enough wheels
    if wheels.len() < 2 {
        return 0.0; // or some default value
    }

    // get data
    let distance1 = wheels[0].lock().get_distance_travelled();
    let distance2 = wheels[1].lock().get_distance_travelled();

    let offset1 = wheels[0].lock().get_offset();
    let offset2 = wheels[1].lock().get_offset();

    // TODO - Add error logic for:
        // [] - equal offsets
        // [] - error wheel readings

    // calculate heading
    ((distance1 - distance2) / (offset1 - offset2)) + 90.0
}

fn find_lateral_delta(sensors: Vec<Arc<Mutex<TrackingWheel>>>) -> f64 {
    for i in 0..sensors.len() {
        let sensor = sensors[i].lock();
        let data = sensor.get_distance_travelled();
        // TODO - Handle fault sensor.
        return data;
    }
    // Return 0.0 if no data was found
    0.0
}

/// Compute the robot’s local (Δx, Δy) given wheel deltas & offsets and a rotation Δθ.
fn compute_local_position(
    delta_theta: f64,
    vertical_delta: f64,
    horizontal_delta: f64,
    vertical_offset: f64,
    horizontal_offset: f64,
) -> (f64, f64) {
    let lateral_deltas  = (vertical_delta,   horizontal_delta);
    let lateral_offsets = (vertical_offset,  horizontal_offset);

    if delta_theta == 0.0 {
        // straight‐line case
        lateral_deltas
    } else {
        // chord‐length formula on each axis
        let factor    = 2.0 * libm::sin(delta_theta * 0.5);
        let inv_theta = 1.0 / delta_theta;
        let radius = (
            lateral_deltas.0 * inv_theta + lateral_offsets.0,
            lateral_deltas.1 * inv_theta + lateral_offsets.1,
        );
        (factor * radius.0, factor * radius.1)
    }
}

impl Localisation {
    pub fn new(sensors: Arc<Mutex<Sensors>>) -> Self {
        // Pre‑allocate space to store the last total for each wheel
        let num_v = sensors.lock().vertical_wheels.len();
        let num_h = sensors.lock().horizontal_wheels.len();
        Self {
            sensors,
            m_pose: Arc::new(Mutex::new(Pose::new())),
            prev_vertical_total: vec![0.0; num_v],
            prev_horizontal_total: vec![0.0; num_h],
        }
    }

    pub fn update(&mut self) {
        loop {
            // 1. Read *deltas* from each wheel
            let mut vertical_delta  = 0.0;
            let mut horizontal_delta = 0.0;

            {
                let s = self.sensors.lock();
                for (i, w) in s.vertical_wheels.iter().enumerate() {
                    let total = w.lock().get_distance_travelled();
                    let delta = total - self.prev_vertical_total[i];
                    self.prev_vertical_total[i] = total;
                    vertical_delta = delta;
                    break;  
                }
                for (i, w) in s.horizontal_wheels.iter().enumerate() {
                    let total = w.lock().get_distance_travelled();
                    let delta = total - self.prev_horizontal_total[i];
                    self.prev_horizontal_total[i] = total;
                    horizontal_delta = delta;
                    break;
                }
            }

            // 2. Correctly pick up theta and *assign* it
            let mut theta = 0.0;
            if let Ok(imu_heading) = self.sensors.lock().imu.lock().heading() {
                theta = imu_heading; 
            } else {
                println!("IMU WENT WRONG!!");
            }

            // 3. Convert heading difference into *radians* for the chord formula
            let old_theta = self.m_pose.lock().theta;
            let delta_theta_deg = theta - old_theta;
            let delta_theta_rad = delta_theta_deg.to_radians();

            // 4. Compute local Δx/Δy using the chord formula
            let vertical_offset   = self.sensors.lock().vertical_wheels[0].lock().get_offset();
            let horizontal_offset = self.sensors.lock().horizontal_wheels[0].lock().get_offset();
            let (delta_x, delta_y) = if delta_theta_rad.abs() < 1e-6 {
                // straight line
                (vertical_delta, horizontal_delta)
            } else {
                let factor = 2.0 * libm::sin(delta_theta_rad * 0.5);
                let inv_theta = 1.0 / delta_theta_rad;
                let r_x = vertical_delta   * inv_theta + vertical_offset;
                let r_y = horizontal_delta * inv_theta + horizontal_offset;
                (factor * r_x, factor * r_y)
            };

            // 5. Rotate into the global frame
            let mid_heading = (old_theta + delta_theta_deg * 0.5).to_radians();
            let cos_h = libm::cos(mid_heading);
            let sin_h = libm::sin(mid_heading);
            let global_dx = delta_x * cos_h - delta_y * sin_h;
            let global_dy = delta_x * sin_h + delta_y * cos_h;

            // 6. Update your pose
            {
                let mut pose = self.m_pose.lock();
                pose.x     += global_dx;
                pose.y     += global_dy;
                pose.theta  = theta;
                println!(
                  "Pose → x: {:+.4}, y: {:+.4}, θ: {:.2}°",
                  pose.x, pose.y, pose.theta
                );
            }

            // 7. Sleep until next tick
            vexide::time::sleep(Duration::from_millis(10));
        }
    }
}
