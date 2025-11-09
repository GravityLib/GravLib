use alloc::{sync::Arc, vec::Vec};
use core::{f64::consts::TAU, ops::Add, time::Duration};

use vexide::{
    core::{sync::Mutex, time::Instant},
};

use crate::{
    actuator::MotorGroup,
};

#[allow(dead_code)]
pub struct DriveTrain {
    left: Arc<Mutex<MotorGroup>>,
    right: Arc<Mutex<MotorGroup>>,
    _odom_task: Task<()>,
}

impl DriveTrain {
    pub async fn new(
        left_motors: Arc<Mutex<Vec<MotorGroup>>>,
        right_motors: Arc<Mutex<Vec<MotorGroup>>>,
        imu: InertialSensor
    )
}