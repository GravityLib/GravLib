use vexide::time::{sleep_until, Instant};
use vexide::competition;
use core::time::Duration;
use vexide::task;

pub struct MotionCancelHelper {
    m_firstIteration: bool,
    m_prevTime: Instant,
    m_originalCompStatus: competition::CompetitionMode,
    m_period: Duration
}

/**
 *  const int64_t now = int64_t(pros::millis());
    if (now - int64_t(m_prevTime) > int64_t(processedTimeout)) m_prevTime = now - processedTimeout;
    // only delay if this is not the first iteration
    if (!m_firstIteration) pros::Task::delay_until(&m_prevTime, processedTimeout);
    else m_firstIteration = false;
 */

impl MotionCancelHelper {
    pub fn new(period: Duration) -> Self {
        let status = competition::status().mode();
        MotionCancelHelper {
            m_firstIteration: true,
            m_prevTime: Instant::now() - period, 
            m_originalCompStatus: status,
            m_period: period
        }
    }

    pub fn wait(&mut self) -> bool {
        let mut m_prevPrevTime: Instant = self.m_prevTime;
        let processedTimeout: Duration = self.m_period; 
        
        let status = competition::status().mode();

        let now =  Instant::now(); 
        if now - self.m_prevTime > processedTimeout {
            self.m_prevTime = now - processedTimeout;
        }

        // delay if not first iteration
        if !self.m_firstIteration { 
            sleep_until(self.m_prevTime + processedTimeout);
        } else {
            self.m_firstIteration = false;
        }

        // different competition status as started, exit motion
        if status != self.m_originalCompStatus {
            false // end motion
        }

        
    }
}