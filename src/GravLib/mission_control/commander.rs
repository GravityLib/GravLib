
use crate::actuator::motor_group::motorGroup;
use crate::actuator::motor_group::motorState;
use alloc::vec::Vec;

pub struct Commander { 
    motor_group_que: Vec<motorGroup>
}

impl Commander {
    pub fn new() -> Self {
        Self {
            motor_group_que: Vec::new()
        }
    }

    pub fn add_motor_group_item(&mut self, item: &motorGroup) {
        self.motor_group_que.push(&item);
    }

    async fn scan(&mut self) {
        loop {
            for group in self.motor_group_que.iter_mut() {
                match group.get_state() {
                    motorState::Moving => {
                        // Handle moving state
                        let velocity = group.get_state_velocity();
                        group.set_velocity(velocity);
                    }
                    motorState::Brake => {

                    }
                    motorState::Idle => {

                    }
                }
            }
        }
    }
}