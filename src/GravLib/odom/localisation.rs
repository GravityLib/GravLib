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

pub struct Localisation {
    sensors: Arc<Mutex<Sensors>>,
    m_pose: Arc<Mutex<Pose>>,
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
    // TODO - initialise m_pose to 0

    pub fn update(&mut self) {

        let prev_time = vexide::time::Instant::now();
        
        loop {
            let time_now = vexide::time::Instant::now();
            let delta_time = time_now.duration_since(prev_time);

            // 1. Get tracking wheel deltas
            let horizontal_delta = find_lateral_delta(self.sensors.lock().horizontal_wheels);
            let vertical_delta = find_lateral_delta(self.sensors.lock().vertical_wheels);
            
            // 2. calculate headings.
                // Option 1: IMU
                // Option 2: Horizontal Wheel
                // Option 3: Vertical Wheel
                // Option 4: Drivetrain
            let theta_opt: Option<f64> = self.sensors.lock().imu.lock().heading().unwrap()
                .or_else(|| calculate_wheel_heading(&self.sensors.lock().horizontal_wheels))
                .or_else(|| calculate_wheel_heading(&self.sensors.lock().vertical_wheels));

            let theta = theta_opt.unwrap_or(0.0);

            // TODO - Add drivetrain Odom
            
            // 3. Calculate change in local coordinates
            let delta_theta = theta - self.m_pose.lock().theta;

            let vertical_offset = self.sensors.lock().vertical_wheels.lock().get_offset();
            let horizontal_offset = self.sensors.lock().horizontal_wheels.lock().get_offset();

            let (delta_x, delta_y) = compute_local_position(
                delta_theta,
                vertical_delta,
                horizontal_delta,
                vertical_offset,
                horizontal_offset,
            );  

            // TODO - CHECK THIS FOLLOWING CODE

            // 4. update gobal postion
            let mid_heading_rad = (self.m_pose.lock().theta + delta_theta * 0.5).to_radians();

            // 5. rotate local Δx/Δy into global frame:
            let cos_h = libm::cos(mid_heading_rad);
            let sin_h = libm::sin(mid_heading_rad);
            let global_dx = delta_x * cos_h - delta_y * sin_h;
            let global_dy = delta_x * sin_h + delta_y * cos_h;

            // 6. update your global pose
            self.m_pose.lock().x += global_dx;
            self.m_pose.lock().y += global_dy;
            self.m_pose.lock().theta = theta;   // store the new absolute heading (in degrees)

            vexide::time::sleep(Duration::from_millis(10));
        }

    }
}