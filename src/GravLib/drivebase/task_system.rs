// Simplified background task system for BigGravLib odometry
// Based on LemLib's design but adapted for VEXide's Embassy async runtime

use core::sync::atomic::{AtomicBool, Ordering};
use spin::Mutex;
use super::odometry::{OdomSensors, update};

/// Configuration for the odometry background task
#[derive(Debug, Clone, Copy)]
pub struct TaskConfig {
    /// Update frequency in Hz (50-200)
    pub frequency_hz: u32,
    /// Whether to auto-start the task after calibration
    pub auto_start: bool,
}

impl Default for TaskConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 100,  // 10ms updates (same as LemLib)
            auto_start: true,
        }
    }
}

/// Performance statistics for the odometry task
#[derive(Debug, Clone, Copy, Default)]
pub struct TaskStats {
    pub updates_completed: u32,
    pub is_running: bool,
}

/// Global task state
static TASK_RUNNING: AtomicBool = AtomicBool::new(false);
static SENSORS_CONFIGURED: AtomicBool = AtomicBool::new(false);
static TASK_STATS: Mutex<TaskStats> = Mutex::new(TaskStats {
    updates_completed: 0,
    is_running: false,
});

/// Configure sensors for odometry (equivalent to LemLib's setSensors)
pub fn configure_sensors(sensors: OdomSensors) -> Result<(), &'static str> {
    super::odometry::set_sensors_with_params(sensors, 320.0, 100.0);
    SENSORS_CONFIGURED.store(true, Ordering::Release);
    Ok(())
}

/// Start the odometry task (equivalent to LemLib's init())
pub fn start_task() -> Result<(), &'static str> {
    if !SENSORS_CONFIGURED.load(Ordering::Acquire) {
        return Err("Sensors must be configured before starting task");
    }
    
    TASK_RUNNING.store(true, Ordering::Release);
    
    // Update stats
    {
        let mut stats = TASK_STATS.lock();
        stats.is_running = true;
    }
    
    Ok(())
}

/// Stop the odometry task
pub fn stop_task() {
    TASK_RUNNING.store(false, Ordering::Release);
    
    // Update stats
    {
        let mut stats = TASK_STATS.lock();
        stats.is_running = false;
    }
}

/// Get current task statistics
pub fn get_stats() -> TaskStats {
    *TASK_STATS.lock()
}

/// Check if the task should be running
pub fn should_run() -> bool {
    TASK_RUNNING.load(Ordering::Acquire)
}

/// Record an update completion
fn record_update() {
    let mut stats = TASK_STATS.lock();
    stats.updates_completed += 1;
}

/// Main odometry task loop (equivalent to LemLib's background task)
/// This should be spawned as a background task: `vexide::async_runtime::spawn(task_loop()).detach();`
pub async fn task_loop() {
    loop {
        if should_run() {
            // Perform odometry update
            update();
            
            // Record statistics
            record_update();
            
            // Sleep for update interval (10ms for 100Hz)
            vexide::time::sleep(core::time::Duration::from_millis(10)).await;
        } else {
            // Task not running, sleep longer
            vexide::time::sleep(core::time::Duration::from_millis(50)).await;
        }
    }
}