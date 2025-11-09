use crate::Gravlib::PID;

enum locked_side{
    LEFT,
    RIGHT,
}

enum angular_direction{
    CLOCKWISE,
    COUNTERCLOCKWISE,
}

struct turn_to_params{
    config: Option<locked_side>,
    direction: Option<angular_direction>,
    maxSpeed: Option<f64>, // TODO - Default = 1
    minSpeedL: Option<f64>, // TODO - Default = 0
    slew: Option<f64>, // TODO - Default = 0
    earlyExitRange: Option<f64>, // TODO - 0 degrees
}

struct turn_to_settings{
    pid: PID,
}

fn turnTo(target: f64, timeout: f64, params: turn_to_params, turn_to_settings: turn_to_settings) {
    
}