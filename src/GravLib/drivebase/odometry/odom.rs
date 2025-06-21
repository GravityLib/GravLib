use alloc::sync::Arc;
use spin::Mutex;
use vexide::devices::adi::encoder::AdiEncoder;
use vexide::devices::smart::InertialSensor as Inertial;
use libm::{sin, cos};

#[derive(Debug)]
pub struct OdometryState {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub last_left: f64,
    pub last_right: f64,
    pub last_horizontal: f64,
}

pub struct Odometry {
    left_encoder: AdiEncoder,
    right_encoder: AdiEncoder,
    horizontal_encoder: AdiEncoder,
    inertial: Inertial,
    state: Arc<Mutex<OdometryState>>,
    track_width: f64,
}

impl Odometry {
    pub fn new(
        left_encoder: AdiEncoder,
        right_encoder: AdiEncoder,
        horizontal_encoder: AdiEncoder,
        inertial: Inertial,
        track_width: f64,
    ) -> Self {
        let state = OdometryState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            last_left: 0.0,
            last_right: 0.0,
            last_horizontal: 0.0,
        };

        Self {
            left_encoder,
            right_encoder,
            horizontal_encoder,
            inertial,
            state: Arc::new(Mutex::new(state)),
            track_width,
        }
    }

    pub fn update(&self) {
        let left_pos = self.left_encoder.position().unwrap_or_default().as_ticks(360) as f64;
        let right_pos = self.right_encoder.position().unwrap_or_default().as_ticks(360) as f64;
        let horiz_pos = self.horizontal_encoder.position().unwrap_or_default().as_ticks(360) as f64;

        let mut state = self.state.lock();

        let d_left = left_pos - state.last_left;
        let d_right = right_pos - state.last_right;
        let d_horizontal = horiz_pos - state.last_horizontal;

        state.last_left = left_pos;
        state.last_right = right_pos;
        state.last_horizontal = horiz_pos;

        let d_center = (d_left + d_right) / 2.0;
        let delta_theta = (d_right - d_left) / self.track_width;

        let sin_theta = sin(state.theta);
        let cos_theta = cos(state.theta);

        state.x += d_center * cos_theta + d_horizontal * sin_theta;
        state.y += d_center * sin_theta - d_horizontal * cos_theta;
        state.y += d_center * sin_theta - d_horizontal * cos_theta;
    }

    pub fn get_position(&self) -> (f64, f64, f64) {
        let state = self.state.lock();
        (state.x, state.y, state.theta)
    }

    pub fn state(&self) -> Arc<Mutex<OdometryState>> {
        Arc::clone(&self.state)
    }
}
