use crate::chassis::{Drivetrain, OdomSensors};
use crate::pose::Pose;
use std::sync::Mutex;
use std::f32::consts::PI;
use crate::chassis::{Drivetrain, OdomSensors, TrackingWheel};
use std::sync::OnceLock;
use std::thread;
use std::time::Duration;

// odom.rs


/// Set the sensors to be used for odometry
///
/// # Arguments
/// * `sensors` - The sensors to be used
/// * `drivetrain` - Drivetrain to be used
pub fn set_sensors(sensors: OdomSensors, drivetrain: Drivetrain) {
    // TODO: implement
}

/// Get the pose of the robot
///
/// # Arguments
/// * `radians` - true for theta in radians, false for degrees (default: false)
pub fn get_pose(radians: bool) -> Pose {
    // TODO: implement
    unimplemented!()
}

/// Set the pose of the robot
///
/// # Arguments
/// * `pose` - The new pose
/// * `radians` - true if theta is in radians, false if in degrees (default: false)
pub fn set_pose(pose: Pose, radians: bool) {
    // TODO: implement
}

/// Get the speed of the robot
///
/// # Arguments
/// * `radians` - true for theta in radians, false for degrees (default: false)
pub fn get_speed(radians: bool) -> Pose {
    // TODO: implement
    unimplemented!()
}

/// Get the local speed of the robot
///
/// # Arguments
/// * `radians` - true for theta in radians, false for degrees (default: false)
pub fn get_local_speed(radians: bool) -> Pose {
    // TODO: implement
    unimplemented!()
}

/// Estimate the pose of the robot after a certain amount of time
///
/// # Arguments
/// * `time` - Time in seconds
/// * `radians` - false for degrees, true for radians (default: false)
pub fn estimate_pose(time: f32, radians: bool) -> Pose {
    // TODO: implement
    unimplemented!()
}

/// Update the pose of the robot
pub fn update() {
    // TODO: implement
}

/// Initialize the odometry system
pub fn init() {
    // TODO: implement
}

// ------------------------------------------------------------------------------------- //


// Global state (use Mutex for mutability in threads)
static ODOM_SENSORS: OnceLock<Mutex<OdomSensors>> = OnceLock::new();
static DRIVETRAIN: OnceLock<Mutex<Drivetrain>> = OnceLock::new();
static ODOM_POSE: OnceLock<Mutex<Pose>> = OnceLock::new();
static ODOM_SPEED: OnceLock<Mutex<Pose>> = OnceLock::new();
static ODOM_LOCAL_SPEED: OnceLock<Mutex<Pose>> = OnceLock::new();

static PREV_VERTICAL: OnceLock<Mutex<f32>> = OnceLock::new();
static PREV_VERTICAL1: OnceLock<Mutex<f32>> = OnceLock::new();
static PREV_VERTICAL2: OnceLock<Mutex<f32>> = OnceLock::new();
static PREV_HORIZONTAL: OnceLock<Mutex<f32>> = OnceLock::new();
static PREV_HORIZONTAL1: OnceLock<Mutex<f32>> = OnceLock::new();
static PREV_HORIZONTAL2: OnceLock<Mutex<f32>> = OnceLock::new();
static PREV_IMU: OnceLock<Mutex<f32>> = OnceLock::new();

fn deg_to_rad(deg: f32) -> f32 {
    deg * PI / 180.0
}
fn rad_to_deg(rad: f32) -> f32 {
    rad * 180.0 / PI
}
fn ema(new: f32, prev: f32, alpha: f32) -> f32 {
    alpha * prev + (1.0 - alpha) * new
}

pub fn set_sensors(sensors: OdomSensors, drivetrain: Drivetrain) {
    ODOM_SENSORS.get_or_init(|| Mutex::new(sensors)).lock().unwrap().clone_from(&sensors);
    DRIVETRAIN.get_or_init(|| Mutex::new(drivetrain)).lock().unwrap().clone_from(&drivetrain);
}

pub fn get_pose(radians: bool) -> Pose {
    let pose = ODOM_POSE.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap().clone();
    if radians {
        pose
    } else {
        Pose { x: pose.x, y: pose.y, theta: rad_to_deg(pose.theta) }
    }
}

pub fn set_pose(pose: Pose, radians: bool) {
    let mut odom_pose = ODOM_POSE.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap();
    if radians {
        *odom_pose = pose;
    } else {
        *odom_pose = Pose { x: pose.x, y: pose.y, theta: deg_to_rad(pose.theta) };
    }
}

pub fn get_speed(radians: bool) -> Pose {
    let speed = ODOM_SPEED.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap().clone();
    if radians {
        speed
    } else {
        Pose { x: speed.x, y: speed.y, theta: rad_to_deg(speed.theta) }
    }
}

pub fn get_local_speed(radians: bool) -> Pose {
    let local_speed = ODOM_LOCAL_SPEED.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap().clone();
    if radians {
        local_speed
    } else {
        Pose { x: local_speed.x, y: local_speed.y, theta: rad_to_deg(local_speed.theta) }
    }
}

pub fn estimate_pose(time: f32, radians: bool) -> Pose {
    let cur_pose = get_pose(true);
    let local_speed = get_local_speed(true);
    let delta_local_pose = local_speed * time;
    let mut future_pose = cur_pose;
    let avg_heading = cur_pose.theta + delta_local_pose.theta / 2.0;
    future_pose.x += delta_local_pose.y * avg_heading.sin();
    future_pose.y += delta_local_pose.y * avg_heading.cos();
    future_pose.x += delta_local_pose.x * -avg_heading.cos();
    future_pose.y += delta_local_pose.x * avg_heading.sin();
    if !radians {
        future_pose.theta = rad_to_deg(future_pose.theta);
    }
    future_pose
}

pub fn update() {
    // TODO: add particle filter
    let mut odom_pose = ODOM_POSE.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap();
    let mut odom_speed = ODOM_SPEED.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap();
    let mut odom_local_speed = ODOM_LOCAL_SPEED.get_or_init(|| Mutex::new(Pose::default())).lock().unwrap();
    let mut prev_vertical = PREV_VERTICAL.get_or_init(|| Mutex::new(0.0)).lock().unwrap();
    let mut prev_vertical1 = PREV_VERTICAL1.get_or_init(|| Mutex::new(0.0)).lock().unwrap();
    let mut prev_vertical2 = PREV_VERTICAL2.get_or_init(|| Mutex::new(0.0)).lock().unwrap();
    let mut prev_horizontal = PREV_HORIZONTAL.get_or_init(|| Mutex::new(0.0)).lock().unwrap();
    let mut prev_horizontal1 = PREV_HORIZONTAL1.get_or_init(|| Mutex::new(0.0)).lock().unwrap();
    let mut prev_horizontal2 = PREV_HORIZONTAL2.get_or_init(|| Mutex::new(0.0)).lock().unwrap();
    let mut prev_imu = PREV_IMU.get_or_init(|| Mutex::new(0.0)).lock().unwrap();

    let sensors = ODOM_SENSORS.get_or_init(|| Mutex::new(OdomSensors::default())).lock().unwrap();

    // get the current sensor values
    let mut vertical1_raw = 0.0;
    let mut vertical2_raw = 0.0;
    let mut horizontal1_raw = 0.0;
    let mut horizontal2_raw = 0.0;
    let mut imu_raw = 0.0;
    if let Some(ref v1) = sensors.vertical1 { vertical1_raw = v1.get_distance_traveled(); }
    if let Some(ref v2) = sensors.vertical2 { vertical2_raw = v2.get_distance_traveled(); }
    if let Some(ref h1) = sensors.horizontal1 { horizontal1_raw = h1.get_distance_traveled(); }
    if let Some(ref h2) = sensors.horizontal2 { horizontal2_raw = h2.get_distance_traveled(); }
    if let Some(ref imu) = sensors.imu { imu_raw = deg_to_rad(imu.get_rotation()); }

    // calculate the change in sensor values
    let delta_vertical1 = vertical1_raw - *prev_vertical1;
    let delta_vertical2 = vertical2_raw - *prev_vertical2;
    let delta_horizontal1 = horizontal1_raw - *prev_horizontal1;
    let delta_horizontal2 = horizontal2_raw - *prev_horizontal2;
    let delta_imu = imu_raw - *prev_imu;

    // update the previous sensor values
    *prev_vertical1 = vertical1_raw;
    *prev_vertical2 = vertical2_raw;
    *prev_horizontal1 = horizontal1_raw;
    *prev_horizontal2 = horizontal2_raw;
    *prev_imu = imu_raw;

    // calculate the heading of the robot
    let mut heading = odom_pose.theta;
    // calculate the heading using the horizontal tracking wheels
    if sensors.horizontal1.is_some() && sensors.horizontal2.is_some() {
        let h1 = sensors.horizontal1.as_ref().unwrap();
        let h2 = sensors.horizontal2.as_ref().unwrap();
        heading -= (delta_horizontal1 - delta_horizontal2) / (h1.get_offset() - h2.get_offset());
    } else if sensors.vertical1.as_ref().map_or(false, |v| !v.get_type())
        && sensors.vertical2.as_ref().map_or(false, |v| !v.get_type())
    {
        let v1 = sensors.vertical1.as_ref().unwrap();
        let v2 = sensors.vertical2.as_ref().unwrap();
        heading -= (delta_vertical1 - delta_vertical2) / (v1.get_offset() - v2.get_offset());
    } else if sensors.imu.is_some() {
        heading += delta_imu;
    } else if sensors.vertical1.is_some() && sensors.vertical2.is_some() {
        let v1 = sensors.vertical1.as_ref().unwrap();
        let v2 = sensors.vertical2.as_ref().unwrap();
        heading -= (delta_vertical1 - delta_vertical2) / (v1.get_offset() - v2.get_offset());
    }
    let delta_heading = heading - odom_pose.theta;
    let avg_heading = odom_pose.theta + delta_heading / 2.0;

    // choose tracking wheels to use
    let mut vertical_wheel: Option<&TrackingWheel> = None;
    let mut horizontal_wheel: Option<&TrackingWheel> = None;
    if sensors.vertical1.as_ref().map_or(false, |v| !v.get_type()) {
        vertical_wheel = sensors.vertical1.as_ref();
    } else if sensors.vertical2.as_ref().map_or(false, |v| !v.get_type()) {
        vertical_wheel = sensors.vertical2.as_ref();
    } else if sensors.vertical1.is_some() {
        vertical_wheel = sensors.vertical1.as_ref();
    }
    if sensors.horizontal1.is_some() {
        horizontal_wheel = sensors.horizontal1.as_ref();
    } else if sensors.horizontal2.is_some() {
        horizontal_wheel = sensors.horizontal2.as_ref();
    }
    let mut raw_vertical = 0.0;
    let mut raw_horizontal = 0.0;
    if let Some(vw) = vertical_wheel { raw_vertical = vw.get_distance_traveled(); }
    if let Some(hw) = horizontal_wheel { raw_horizontal = hw.get_distance_traveled(); }
    let mut vertical_offset = 0.0;
    let mut horizontal_offset = 0.0;
    if let Some(vw) = vertical_wheel { vertical_offset = vw.get_offset(); }
    if let Some(hw) = horizontal_wheel { horizontal_offset = hw.get_offset(); }

    // calculate change in x and y
    let mut delta_x = 0.0;
    let mut delta_y = 0.0;
    if vertical_wheel.is_some() { delta_y = raw_vertical - *prev_vertical; }
    if horizontal_wheel.is_some() { delta_x = raw_horizontal - *prev_horizontal; }
    *prev_vertical = raw_vertical;
    *prev_horizontal = raw_horizontal;

    // calculate local x and y
    let (local_x, local_y) = if delta_heading == 0.0 {
        (delta_x, delta_y)
    } else {
        (
            2.0 * (delta_heading / 2.0).sin() * (delta_x / delta_heading + horizontal_offset),
            2.0 * (delta_heading / 2.0).sin() * (delta_y / delta_heading + vertical_offset),
        )
    };

    // save previous pose
    let prev_pose = *odom_pose;

    // calculate global x and y
    odom_pose.x += local_y * avg_heading.sin();
    odom_pose.y += local_y * avg_heading.cos();
    odom_pose.x += local_x * -avg_heading.cos();
    odom_pose.y += local_x * avg_heading.sin();
    odom_pose.theta = heading;

    // calculate speed
    odom_speed.x = ema((odom_pose.x - prev_pose.x) / 0.01, odom_speed.x, 0.95);
    odom_speed.y = ema((odom_pose.y - prev_pose.y) / 0.01, odom_speed.y, 0.95);
    odom_speed.theta = ema((odom_pose.theta - prev_pose.theta) / 0.01, odom_speed.theta, 0.95);

    // calculate local speed
    odom_local_speed.x = ema(local_x / 0.01, odom_local_speed.x, 0.95);
    odom_local_speed.y = ema(local_y / 0.01, odom_local_speed.y, 0.95);
    odom_local_speed.theta = ema(delta_heading / 0.01, odom_local_speed.theta, 0.95);
}

pub fn init() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        thread::spawn(|| {
            loop {
                update();
                thread::sleep(Duration::from_millis(10));
            }
        });
    });
}
