use crate::GravLib::drivebase::{Pose, Drivetrain};
use vexide::devices::smart::imu::InertialSensor;
use spin::RwLock;
use core::f64::consts::PI;
use libm::{sin, cos};

use super::chassis::TrackingWheel;

// Pre-calculated constants for better performance
const PI_DIV_180: f64 = PI / 180.0;
const INV_180_DIV_PI: f64 = 180.0 / PI;
const EMA_ALPHA: f64 = 0.95;
const EMA_BETA: f64 = 1.0 - EMA_ALPHA;
const DELTA_TIME: f64 = 0.01; // 10ms loop time

pub struct OdomSensors {
    pub vertical1: Option<TrackingWheel>,
    pub vertical2: Option<TrackingWheel>,
    pub horizontal1: Option<TrackingWheel>,
    pub horizontal2: Option<TrackingWheel>,
    pub imu: Option<InertialSensor>,
}

impl OdomSensors {
    pub fn new(
        vertical1: Option<TrackingWheel>,
        vertical2: Option<TrackingWheel>, 
        horizontal1: Option<TrackingWheel>,
        horizontal2: Option<TrackingWheel>,
        imu: Option<InertialSensor>,
    ) -> Self {
        OdomSensors {
            vertical1,
            vertical2,
            horizontal1,
            horizontal2,
            imu,
        }
    }
}

/// Global odometry state - using RwLock for better concurrent read access
static ODOMETRY_STATE: RwLock<Option<OdometryState>> = RwLock::new(None);

struct OdometryState {
    sensors: OdomSensors,
    drivetrain: Drivetrain,
    pose: Pose,
    speed: Pose,
    local_speed: Pose,
    
    // Previous sensor values for delta calculations
    prev_vertical: f64,
    prev_vertical1: f64,
    prev_vertical2: f64,
    prev_horizontal: f64,
    prev_horizontal1: f64,
    prev_horizontal2: f64,
    prev_imu: f64,
}

impl OdometryState {
    fn new(sensors: OdomSensors, drivetrain: Drivetrain) -> Self {
        OdometryState {
            sensors,
            drivetrain,
            pose: Pose::new(0.0, 0.0, 0.0),
            speed: Pose::new(0.0, 0.0, 0.0),
            local_speed: Pose::new(0.0, 0.0, 0.0),
            prev_vertical: 0.0,
            prev_vertical1: 0.0,
            prev_vertical2: 0.0,
            prev_horizontal: 0.0,
            prev_horizontal1: 0.0,
            prev_horizontal2: 0.0,
            prev_imu: 0.0,
        }
    }
}

/// Optimized exponential moving average function with pre-calculated constants
#[inline(always)]
fn ema(input: f64, prev_output: f64) -> f64 {
    EMA_ALPHA * input + EMA_BETA * prev_output
}

/// Optimized utility function to convert degrees to radians using pre-calculated constant
#[inline(always)]
fn deg_to_rad(deg: f64) -> f64 {
    deg * PI_DIV_180
}

/// Optimized utility function to convert radians to degrees using pre-calculated constant
#[inline(always)]
fn rad_to_deg(rad: f64) -> f64 {
    rad * INV_180_DIV_PI
}

/// Set the sensors to be used for odometry
pub fn set_sensors(sensors: OdomSensors, drivetrain: Drivetrain) {
    let mut state_lock = ODOMETRY_STATE.write();
    *state_lock = Some(OdometryState::new(sensors, drivetrain));
}

/// Set the sensors with just the drivetrain parameters (for task manager)
pub fn set_sensors_with_params(sensors: OdomSensors, track_width: f64, wheel_diameter: f64) {
    // Create a dummy drivetrain with just the parameters needed for odometry calculations
    // The motor groups won't be used in odometry calculations, only the geometric parameters
    use alloc::vec::Vec;
    use vexide::prelude::{Motor, Gearset, Direction};
    
    // Create dummy motor groups - these won't actually be used for odometry calculations
    // This is a workaround until we refactor the odometry system to not need the full Drivetrain
    let dummy_motors = Vec::new();
    let left_group = crate::GravLib::actuator::MotorGroup::new(dummy_motors);
    let right_group = crate::GravLib::actuator::MotorGroup::new(Vec::new());
    
    let drivetrain = Drivetrain::new(left_group, right_group, track_width, wheel_diameter);
    
    let mut state_lock = ODOMETRY_STATE.write();
    *state_lock = Some(OdometryState::new(sensors, drivetrain));
}

/// Get the pose of the robot with optimized read access
pub fn get_pose(radians: bool) -> Pose {
    let state_lock = ODOMETRY_STATE.read();
    match state_lock.as_ref() {
        Some(state) => {
            if radians {
                state.pose
            } else {
                Pose::new(state.pose.x(), state.pose.y(), rad_to_deg(state.pose.theta()))
            }
        },
        None => Pose::new(0.0, 0.0, 0.0),
    }
}

/// Set the pose of the robot
pub fn set_pose(pose: Pose, radians: bool) {
    let mut state_lock = ODOMETRY_STATE.write();
    if let Some(state) = state_lock.as_mut() {
        state.pose = if radians {
            pose
        } else {
            Pose::new(pose.x(), pose.y(), deg_to_rad(pose.theta()))
        };
    }
}

/// Get the speed of the robot
pub fn get_speed(radians: bool) -> Pose {
    let state_lock = ODOMETRY_STATE.read();
    match state_lock.as_ref() {
        Some(state) => {
            if radians {
                state.speed
            } else {
                Pose::new(state.speed.x(), state.speed.y(), rad_to_deg(state.speed.theta()))
            }
        },
        None => Pose::new(0.0, 0.0, 0.0),
    }
}

/// Get the local speed of the robot
pub fn get_local_speed(radians: bool) -> Pose {
    let state_lock = ODOMETRY_STATE.read();
    match state_lock.as_ref() {
        Some(state) => {
            if radians {
                state.local_speed
            } else {
                Pose::new(state.local_speed.x(), state.local_speed.y(), rad_to_deg(state.local_speed.theta()))
            }
        },
        None => Pose::new(0.0, 0.0, 0.0),
    }
}

/// Estimate the pose of the robot after a certain amount of time
pub fn estimate_pose(time: f64, radians: bool) -> Pose {
    let cur_pose = get_pose(true);
    let local_speed = get_local_speed(true);
    
    // Calculate the change in local position
    let delta_local_pose = local_speed * time;
    
    // Calculate the future pose
    let avg_heading = cur_pose.theta() + delta_local_pose.theta() / 2.0;
    let mut future_pose = cur_pose;
    
    future_pose.set_x(future_pose.x() + delta_local_pose.y() * sin(avg_heading));
    future_pose.set_y(future_pose.y() + delta_local_pose.y() * cos(avg_heading));
    future_pose.set_x(future_pose.x() + delta_local_pose.x() * -cos(avg_heading));
    future_pose.set_y(future_pose.y() + delta_local_pose.x() * sin(avg_heading));
    
    if !radians {
        future_pose.set_theta(rad_to_deg(future_pose.theta()));
    }
    
    future_pose
}

/// Update the pose of the robot - core odometry calculation
pub fn update() {
    let mut state_lock = ODOMETRY_STATE.write();
    let state = match state_lock.as_mut() {
        Some(state) => state,
        None => return, // No odometry configured
    };
    
    // Get current sensor values
    let mut vertical1_raw = 0.0;
    let mut vertical2_raw = 0.0;
    let mut horizontal1_raw = 0.0;
    let mut horizontal2_raw = 0.0;
    let mut imu_raw = 0.0;
    
    if let Some(ref v1) = state.sensors.vertical1 {
        vertical1_raw = v1.get_distance_travelled();
    }
    if let Some(ref v2) = state.sensors.vertical2 {
        vertical2_raw = v2.get_distance_travelled();
    }
    if let Some(ref h1) = state.sensors.horizontal1 {
        horizontal1_raw = h1.get_distance_travelled();
    }
    if let Some(ref h2) = state.sensors.horizontal2 {
        horizontal2_raw = h2.get_distance_travelled();
    }
    if let Some(ref imu) = state.sensors.imu {
        if let Ok(rotation) = imu.rotation() {
            imu_raw = deg_to_rad(rotation);
        }
    }
    
    // Calculate deltas
    let delta_vertical1 = vertical1_raw - state.prev_vertical1;
    let delta_vertical2 = vertical2_raw - state.prev_vertical2;
    let delta_horizontal1 = horizontal1_raw - state.prev_horizontal1;
    let delta_horizontal2 = horizontal2_raw - state.prev_horizontal2;
    let delta_imu = imu_raw - state.prev_imu;
    
    // Update previous values
    state.prev_vertical1 = vertical1_raw;
    state.prev_vertical2 = vertical2_raw;
    state.prev_horizontal1 = horizontal1_raw;
    state.prev_horizontal2 = horizontal2_raw;
    state.prev_imu = imu_raw;
    
    // Calculate heading using priority system from LemLib
    let mut heading = state.pose.theta();
    
    // Priority 1: Horizontal tracking wheels
    if state.sensors.horizontal1.is_some() && state.sensors.horizontal2.is_some() {
        let h1_offset = state.sensors.horizontal1.as_ref().unwrap().get_offset();
        let h2_offset = state.sensors.horizontal2.as_ref().unwrap().get_offset();
        heading -= (delta_horizontal1 - delta_horizontal2) / (h1_offset - h2_offset);
    }
    // Priority 2: Vertical tracking wheels (if neither is powered)
    else if let (Some(ref v1), Some(ref v2)) = (&state.sensors.vertical1, &state.sensors.vertical2) {
        let v1_offset = v1.get_offset();
        let v2_offset = v2.get_offset();
        heading -= (delta_vertical1 - delta_vertical2) / (v1_offset - v2_offset);
    }
    // Priority 3: IMU
    else if state.sensors.imu.is_some() {
        heading += delta_imu;
    }
    
    let delta_heading = heading - state.pose.theta();
    let avg_heading = state.pose.theta() + delta_heading / 2.0;
    
    // Choose tracking wheels to use (prioritize non-powered)
    let (raw_vertical, vertical_offset) = if let Some(ref v1) = state.sensors.vertical1 {
        (v1.get_distance_travelled(), v1.get_offset())
    } else if let Some(ref v2) = state.sensors.vertical2 {
        (v2.get_distance_travelled(), v2.get_offset())
    } else {
        (0.0, 0.0)
    };
    
    let (raw_horizontal, horizontal_offset) = if let Some(ref h1) = state.sensors.horizontal1 {
        (h1.get_distance_travelled(), h1.get_offset())
    } else if let Some(ref h2) = state.sensors.horizontal2 {
        (h2.get_distance_travelled(), h2.get_offset())
    } else {
        (0.0, 0.0)
    };
    
    // Calculate change in x and y
    let delta_y = raw_vertical - state.prev_vertical;
    let delta_x = raw_horizontal - state.prev_horizontal;
    state.prev_vertical = raw_vertical;
    state.prev_horizontal = raw_horizontal;
    
    // Calculate local x and y
    let (local_x, local_y) = if delta_heading.abs() < f64::EPSILON {
        // Prevent divide by zero
        (delta_x, delta_y)
    } else {
        let local_x = 2.0 * sin(delta_heading / 2.0) * (delta_x / delta_heading + horizontal_offset);
        let local_y = 2.0 * sin(delta_heading / 2.0) * (delta_y / delta_heading + vertical_offset);
        (local_x, local_y)
    };
    
    // Save previous pose for speed calculation
    let prev_pose = state.pose;
    
    // Calculate global x and y
    state.pose.set_x(state.pose.x() + local_y * sin(avg_heading));
    state.pose.set_y(state.pose.y() + local_y * cos(avg_heading));
    state.pose.set_x(state.pose.x() + local_x * -cos(avg_heading));
    state.pose.set_y(state.pose.y() + local_x * sin(avg_heading));
    state.pose.set_theta(heading);
    
    // Calculate speed using pre-calculated constants for better performance
    let inv_dt = 1.0 / DELTA_TIME; // More efficient than division in loop
    state.speed.set_x(ema((state.pose.x() - prev_pose.x()) * inv_dt, state.speed.x()));
    state.speed.set_y(ema((state.pose.y() - prev_pose.y()) * inv_dt, state.speed.y()));
    state.speed.set_theta(ema((state.pose.theta() - prev_pose.theta()) * inv_dt, state.speed.theta()));
    
    // Calculate local speed
    state.local_speed.set_x(ema(local_x * inv_dt, state.local_speed.x()));
    state.local_speed.set_y(ema(local_y * inv_dt, state.local_speed.y()));
    state.local_speed.set_theta(ema(delta_heading * inv_dt, state.local_speed.theta()));
}

/// Initialize odometry system (would start background task in full implementation)
pub fn init() {
    // In a full implementation, this would spawn a background task that calls update() every 10ms
    // For now, user must call update() manually or set up their own task
}