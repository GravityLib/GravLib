use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex},
    channel::{Channel, Receiver, Sender},
};
use spin::Mutex;
use super::odometry::OdomSensors;

/// Configuration for the odometry background task
#[derive(Debug, Clone, Copy)]
pub struct OdometryTaskConfig {
    /// Update frequency in Hz (50-1000)
    pub frequency_hz: u32,
    /// Task priority (0-255, higher = more important)
    pub priority: u8,
    /// Whether to auto-start the task after calibration
    pub auto_start: bool,
    /// Maximum allowed jitter in microseconds
    pub max_jitter_us: u32,
}

impl Default for OdometryTaskConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 100,  // 10ms updates (same as LemLib)
            priority: 200,      // High priority
            auto_start: true,
            max_jitter_us: 500, // 0.5ms tolerance
        }
    }
}

/// Task commands for runtime control
#[derive(Debug, Clone, Copy)]
pub enum OdometryTaskCommand {
    Start,
    Stop,
    Pause,
    Resume,
    UpdateConfig(OdometryTaskConfig),
}

/// Performance statistics for the odometry task
#[derive(Debug, Clone, Copy, Default)]
pub struct OdometryTaskStats {
    pub updates_completed: u32,
    pub total_update_time_us: u64,
    pub max_update_time_us: u32,
    pub min_update_time_us: u32,
    pub jitter_violations: u32,
    pub is_running: bool,
    pub is_paused: bool,
}

/// Task manager for the odometry background system
pub struct OdometryTaskManager {
    config: Mutex<OdometryTaskConfig>,
    is_running: AtomicBool,
    is_paused: AtomicBool,
    sensors_configured: AtomicBool,
    stats: Mutex<OdometryTaskStats>,
    command_sender: Sender<'static, CriticalSectionRawMutex, OdometryTaskCommand, 8>,
    command_receiver: Receiver<'static, CriticalSectionRawMutex, OdometryTaskCommand, 8>,
}

/// Global task manager instance
static TASK_MANAGER: once_cell::sync::Lazy<OdometryTaskManager> = 
    once_cell::sync::Lazy::new(|| OdometryTaskManager::new());

/// Channel for task commands
static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, OdometryTaskCommand, 8> = 
    Channel::new();

impl OdometryTaskManager {
    fn new() -> Self {
        let (sender, receiver) = COMMAND_CHANNEL.split();
        
        Self {
            config: Mutex::new(OdometryTaskConfig::default()),
            is_running: AtomicBool::new(false),
            is_paused: AtomicBool::new(false),
            sensors_configured: AtomicBool::new(false),
            stats: Mutex::new(OdometryTaskStats::default()),
            command_sender: sender,
            command_receiver: receiver,
        }
    }

    /// Configure sensors for odometry (equivalent to LemLib's setSensors)
    pub fn configure_sensors(&self, sensors: OdomSensors) -> Result<(), &'static str> {
        // For now, use a placeholder drivetrain - the task manager doesn't need the full drivetrain
        // This will be refactored when we implement the full sensor abstraction
        super::odometry::set_sensors_with_params(sensors, 320.0, 100.0);
        self.sensors_configured.store(true, Ordering::Release);
        Ok(())
    }

    /// Start the odometry task
    pub fn start(&self) -> Result<(), &'static str> {
        if !self.sensors_configured.load(Ordering::Acquire) {
            return Err("Sensors must be configured before starting task");
        }
        
        if !self.is_running.load(Ordering::Acquire) {
            self.is_running.store(true, Ordering::Release);
            self.is_paused.store(false, Ordering::Release);
            
            // Update stats
            {
                let mut stats = self.stats.lock();
                stats.is_running = true;
                stats.is_paused = false;
            }
        }
        
        Ok(())
    }

    /// Stop the odometry task
    pub fn stop(&self) {
        self.is_running.store(false, Ordering::Release);
        self.is_paused.store(false, Ordering::Release);
        
        // Update stats
        {
            let mut stats = self.stats.lock();
            stats.is_running = false;
            stats.is_paused = false;
        }
    }

    /// Pause the odometry task
    pub fn pause(&self) {
        if self.is_running.load(Ordering::Acquire) {
            self.is_paused.store(true, Ordering::Release);
            
            // Update stats
            {
                let mut stats = self.stats.lock();
                stats.is_paused = true;
            }
        }
    }

    /// Resume the odometry task
    pub fn resume(&self) {
        if self.is_running.load(Ordering::Acquire) {
            self.is_paused.store(false, Ordering::Release);
            
            // Update stats
            {
                let mut stats = self.stats.lock();
                stats.is_paused = false;
            }
        }
    }

    /// Update configuration
    pub fn update_config(&self, config: OdometryTaskConfig) {
        *self.config.lock() = config;
    }

    /// Get current statistics
    pub fn get_stats(&self) -> OdometryTaskStats {
        *self.stats.lock()
    }

    /// Check if the task should be running
    pub fn should_run(&self) -> bool {
        self.is_running.load(Ordering::Acquire) && !self.is_paused.load(Ordering::Acquire)
    }

    /// Record an update completion
    pub fn record_update(&self, update_time_us: u32, was_late: bool) {
        let mut stats = self.stats.lock();
        
        stats.updates_completed += 1;
        stats.total_update_time_us += update_time_us as u64;
        
        if stats.max_update_time_us == 0 || update_time_us > stats.max_update_time_us {
            stats.max_update_time_us = update_time_us;
        }
        
        if stats.min_update_time_us == 0 || update_time_us < stats.min_update_time_us {
            stats.min_update_time_us = update_time_us;
        }
        
        if was_late {
            stats.jitter_violations += 1;
        }
    }

    /// Get current configuration
    pub fn get_config(&self) -> OdometryTaskConfig {
        *self.config.lock()
    }
}

/// Global task management functions (LemLib-compatible API)

/// Start the odometry task (equivalent to LemLib's init())
pub async fn start_odometry() -> Result<(), &'static str> {
    TASK_MANAGER.command_sender.send(OdometryTaskCommand::Start).await;
    Ok(())
}

/// Stop the odometry task
pub async fn stop_odometry() -> Result<(), &'static str> {
    TASK_MANAGER.command_sender.send(OdometryTaskCommand::Stop).await;
    Ok(())
}

/// Pause the odometry task
pub async fn pause_odometry() -> Result<(), &'static str> {
    TASK_MANAGER.command_sender.send(OdometryTaskCommand::Pause).await;
    Ok(())
}

/// Resume the odometry task
pub async fn resume_odometry() -> Result<(), &'static str> {
    TASK_MANAGER.command_sender.send(OdometryTaskCommand::Resume).await;
    Ok(())
}

/// Update task configuration
pub async fn update_odometry_config(config: OdometryTaskConfig) -> Result<(), &'static str> {
    TASK_MANAGER.command_sender.send(OdometryTaskCommand::UpdateConfig(config)).await;
    Ok(())
}

/// Get current task statistics
pub async fn get_odometry_stats() -> Option<OdometryTaskStats> {
    Some(TASK_MANAGER.get_stats())
}

/// Configure sensors for odometry (equivalent to LemLib's setSensors)
pub fn configure_odometry_sensors(sensors: OdomSensors) -> Result<(), &'static str> {
    TASK_MANAGER.configure_sensors(sensors)
}

/// Main odometry task loop (equivalent to LemLib's background task)
/// This should be spawned as a background task: `vexide::async_runtime::spawn(odometry_task_loop()).detach();`
pub async fn odometry_task_loop() {
    let mut next_update = vexide::time::Instant::now();
    
    loop {
        // Check for commands
        if let Ok(command) = TASK_MANAGER.command_receiver.try_receive() {
            match command {
                OdometryTaskCommand::Start => {
                    let _ = TASK_MANAGER.start();
                }
                OdometryTaskCommand::Stop => {
                    TASK_MANAGER.stop();
                }
                OdometryTaskCommand::Pause => {
                    TASK_MANAGER.pause();
                }
                OdometryTaskCommand::Resume => {
                    TASK_MANAGER.resume();
                }
                OdometryTaskCommand::UpdateConfig(config) => {
                    TASK_MANAGER.update_config(config);
                }
            }
        }
        
        // Update odometry if running
        if TASK_MANAGER.should_run() {
            let start_time = vexide::time::Instant::now();
            
            // Perform odometry update
            super::odometry::update();
            
            let update_duration = start_time.elapsed();
            let update_time_us = update_duration.as_micros() as u32;
            
            // Check for timing violations
            let config = TASK_MANAGER.get_config();
            let expected_interval_ms = 1000 / config.frequency_hz as u64;
            let expected_interval = vexide::time::Duration::from_millis(expected_interval_ms);
            let now = vexide::time::Instant::now();
            let jitter_tolerance = vexide::time::Duration::from_micros(config.max_jitter_us as u64);
            let was_late = now > next_update + jitter_tolerance;
            
            // Record statistics
            TASK_MANAGER.record_update(update_time_us, was_late);
            
            // Calculate next update time
            next_update += expected_interval;
            
            // Sleep until next update
            if now < next_update {
                vexide::time::sleep_until(next_update).await;
            } else {
                // We're running late, just yield and reset timing
                vexide::task::yield_now().await;
                next_update = now + expected_interval;
            }
        } else {
            // Task not running, sleep longer
            vexide::time::sleep(vexide::time::Duration::from_millis(10)).await;
        }
    }
}

/// Error types for task management
#[derive(Debug, Clone, Copy)]
pub enum TaskError {
    SensorsNotConfigured,
    InvalidConfiguration,
    TaskAlreadyRunning,
    TaskNotRunning,
}

impl core::fmt::Display for TaskError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TaskError::SensorsNotConfigured => write!(f, "Sensors must be configured before starting task"),
            TaskError::InvalidConfiguration => write!(f, "Invalid task configuration"),
            TaskError::TaskAlreadyRunning => write!(f, "Task is already running"),
            TaskError::TaskNotRunning => write!(f, "Task is not running"),
        }
    }
}