# Odometry Formula Sheet

This document provides a comprehensive reference of all mathematical formulas used in the BigGravLib Rust odometry tracking system.

## Table of Contents
1. [Angle Conversion](#angle-conversion)
2. [Digital Signal Processing](#digital-signal-processing)
3. [Pose Estimation](#pose-estimation)
4. [Heading Calculation](#heading-calculation)
5. [Arc Motion Modeling](#arc-motion-modeling)
6. [Coordinate Transformations](#coordinate-transformations)
7. [Velocity Calculations](#velocity-calculations)
8. [Pose Vector Operations](#pose-vector-operations)
9. [Control Theory](#control-theory)
10. [Sensor Conversions](#sensor-conversions)
11. [Motor Control](#motor-control)

---

## Angle Conversion

### Degrees to Radians
**Purpose:** Convert angle measurements from degrees to radians for trigonometric calculations  
**Location:** `src/GravLib/drivebase/odometry.rs:80-82`  
**Implementation:** `deg_to_rad(deg: f64) -> f64`

$$\text{radians} = \text{degrees} \times \frac{\pi}{180}$$

### Radians to Degrees
**Purpose:** Convert angle measurements from radians to degrees for user-friendly output  
**Location:** `src/GravLib/drivebase/odometry.rs:85-87`  
**Implementation:** `rad_to_deg(rad: f64) -> f64`

$$\text{degrees} = \text{radians} \times \frac{180}{\pi}$$

---

## Digital Signal Processing

### Exponential Moving Average (EMA) Filter
**Purpose:** Smooth velocity calculations and reduce sensor noise  
**Location:** `src/GravLib/drivebase/odometry.rs:75-77`  
**Implementation:** `ema(input: f64, prev_output: f64, alpha: f64) -> f64`  
**Parameters:** $\alpha = 0.95$ (smoothing factor)

$$y_n = \alpha \cdot x_n + (1 - \alpha) \cdot y_{n-1}$$

Where:
- $y_n$ = current filtered output
- $x_n$ = current input sample
- $y_{n-1}$ = previous filtered output
- $\alpha$ = smoothing factor (0 < α < 1)

---

## Pose Estimation

### Average Heading Calculation
**Purpose:** Calculate intermediate heading for accurate coordinate transformation during motion  
**Location:** `src/GravLib/drivebase/odometry.rs:161`

$$\theta_{avg} = \theta_{current} + \frac{\Delta\theta_{local}}{2}$$

### Future Pose Prediction
**Purpose:** Transform local velocity into global coordinate changes for pose prediction  
**Location:** `src/GravLib/drivebase/odometry.rs:164-167`

$$\begin{align}
x_{future} &= x_{current} + \Delta y_{local} \sin(\theta_{avg}) + \Delta x_{local} \cdot (-\cos(\theta_{avg})) \\
y_{future} &= y_{current} + \Delta y_{local} \cos(\theta_{avg}) + \Delta x_{local} \sin(\theta_{avg})
\end{align}$$

Where:
- $\Delta x_{local}$, $\Delta y_{local}$ = local frame displacement
- $\theta_{avg}$ = average heading during motion

---

## Heading Calculation

The system uses a prioritized sensor fusion approach for heading determination.

### Priority 1: Horizontal Tracking Wheels
**Purpose:** Most accurate heading calculation using dedicated horizontal tracking wheels  
**Location:** `src/GravLib/drivebase/odometry.rs:230`

$$\theta = \theta_{prev} - \frac{\Delta s_{h1} - \Delta s_{h2}}{d_{h1} - d_{h2}}$$

Where:
- $\Delta s_{h1}$, $\Delta s_{h2}$ = distance changes of horizontal wheels 1 and 2
- $d_{h1}$, $d_{h2}$ = distances of wheels from robot center

### Priority 2: Vertical Tracking Wheels
**Purpose:** Backup heading calculation when horizontal wheels unavailable  
**Location:** `src/GravLib/drivebase/odometry.rs:236`

$$\theta = \theta_{prev} - \frac{\Delta s_{v1} - \Delta s_{v2}}{d_{v1} - d_{v2}}$$

Where:
- $\Delta s_{v1}$, $\Delta s_{v2}$ = distance changes of vertical wheels 1 and 2
- $d_{v1}$, $d_{v2}$ = distances of wheels from robot center

### Priority 3: IMU Integration
**Purpose:** Direct heading measurement from inertial sensor  
**Location:** `src/GravLib/drivebase/odometry.rs:240`

$$\theta = \theta_{prev} + \Delta\theta_{IMU}$$

Where $\Delta\theta_{IMU}$ = change in IMU reading (radians)

---

## Arc Motion Modeling

### Local Coordinate Calculation for Curved Motion
**Purpose:** Account for arc length vs chord length difference in curved robot motion  
**Location:** `src/GravLib/drivebase/odometry.rs:274-276`

$$\begin{align}
x_{local} &= 2 \sin\left(\frac{\Delta\theta}{2}\right) \left(\frac{\Delta x}{\Delta\theta} + d_{horizontal}\right) \\
y_{local} &= 2 \sin\left(\frac{\Delta\theta}{2}\right) \left(\frac{\Delta y}{\Delta\theta} + d_{vertical}\right)
\end{align}$$

Where:
- $\Delta\theta$ = change in robot orientation
- $\Delta x$, $\Delta y$ = raw wheel movements
- $d_{horizontal}$, $d_{vertical}$ = wheel positions relative to robot center

**Note:** This formula converts chord measurements to arc lengths for accurate motion tracking.

---

## Coordinate Transformations

### Global Position Update
**Purpose:** Transform local robot movement into global field coordinates  
**Location:** `src/GravLib/drivebase/odometry.rs:283-286`

$$\begin{align}
x_{global} &= x_{prev} + y_{local} \sin(\theta_{avg}) + x_{local} \cdot (-\cos(\theta_{avg})) \\
y_{global} &= y_{prev} + y_{local} \cos(\theta_{avg}) + x_{local} \sin(\theta_{avg})
\end{align}$$

Where:
- $(x_{local}, y_{local})$ = movement in robot's local coordinate frame
- $\theta_{avg}$ = average robot orientation during movement
- Forward component: $y_{local}$ terms
- Strafe component: $x_{local}$ terms

### Coordinate Rotation Matrix
**Purpose:** Rotate point coordinates by specified angle  
**Location:** `src/GravLib/drivebase/pose.rs:110-118`

$$\begin{bmatrix} x' \\ y' \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix}$$

---

## Velocity Calculations

### Global Velocity with EMA Filtering
**Purpose:** Calculate smoothed global velocity components  
**Location:** `src/GravLib/drivebase/odometry.rs:291-293`  
**Parameters:** $\Delta t = 0.01s$ (10ms loop time), $\alpha = 0.95$

$$\begin{align}
\dot{x} &= \text{EMA}\left(\frac{x - x_{prev}}{\Delta t}, \dot{x}_{prev}, 0.95\right) \\
\dot{y} &= \text{EMA}\left(\frac{y - y_{prev}}{\Delta t}, \dot{y}_{prev}, 0.95\right) \\
\dot{\theta} &= \text{EMA}\left(\frac{\theta - \theta_{prev}}{\Delta t}, \dot{\theta}_{prev}, 0.95\right)
\end{align}$$

### Local Velocity with EMA Filtering
**Purpose:** Calculate smoothed robot-relative velocity components  
**Location:** `src/GravLib/drivebase/odometry.rs:296-298`

$$\begin{align}
\dot{x}_{local} &= \text{EMA}\left(\frac{x_{local}}{\Delta t}, \dot{x}_{local,prev}, 0.95\right) \\
\dot{y}_{local} &= \text{EMA}\left(\frac{y_{local}}{\Delta t}, \dot{y}_{local,prev}, 0.95\right) \\
\dot{\theta}_{local} &= \text{EMA}\left(\frac{\Delta\theta}{\Delta t}, \dot{\theta}_{local,prev}, 0.95\right)
\end{align}$$

---

## Pose Vector Operations

### Vector Addition
**Location:** `src/GravLib/drivebase/pose.rs:17-23`
$$\vec{P}_3 = \vec{P}_1 + \vec{P}_2 = (x_1 + x_2, y_1 + y_2, \theta_1 + \theta_2)$$

### Vector Subtraction
**Location:** `src/GravLib/drivebase/pose.rs:29-35`
$$\vec{P}_3 = \vec{P}_1 - \vec{P}_2 = (x_1 - x_2, y_1 - y_2, \theta_1 - \theta_2)$$

### Scalar Multiplication
**Location:** `src/GravLib/drivebase/pose.rs:41-47`
$$k\vec{P} = (kx, ky, k\theta)$$

### Scalar Division
**Location:** `src/GravLib/drivebase/pose.rs:53-59`
$$\frac{\vec{P}}{k} = \left(\frac{x}{k}, \frac{y}{k}, \frac{\theta}{k}\right)$$

### Linear Interpolation (LERP)
**Purpose:** Smooth interpolation between two poses  
**Location:** `src/GravLib/drivebase/pose.rs:91-97`
$$\vec{P}_{interp} = \vec{P}_1 + t(\vec{P}_2 - \vec{P}_1) = (1-t)\vec{P}_1 + t\vec{P}_2$$

Where $t \in [0,1]$ is the interpolation parameter.

### Euclidean Distance
**Location:** `src/GravLib/drivebase/pose.rs:99-101`
$$d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$$

### Angle to Target
**Purpose:** Calculate bearing angle to target pose  
**Location:** `src/GravLib/drivebase/pose.rs:103-108`
$$\theta_{target} = \text{atan2}(\Delta y, \Delta x)$$

Where $\Delta x = x_2 - x_1$ and $\Delta y = y_2 - y_1$.

---

## Control Theory

### PID Controller

#### Derivative Term
**Purpose:** Rate of change of error for derivative control  
**Location:** `src/GravLib/pid.rs:119-123`
$$\frac{de}{dt} = \frac{e_n - e_{n-1}}{\Delta t}$$

#### Integral Term
**Purpose:** Accumulated error over time for integral control  
**Location:** `src/GravLib/pid.rs:128`
$$\int e \, dt \approx \sum_{i=0}^{n} e_i \Delta t$$

#### PID Output Formula
**Purpose:** Combined PID control signal calculation  
**Location:** `src/GravLib/pid.rs:141-143`
$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

Where:
- $K_p$, $K_i$, $K_d$ = proportional, integral, derivative gains
- $e(t)$ = error signal
- $u(t)$ = control output

---

## Sensor Conversions

### Encoder to Distance Conversion
**Purpose:** Convert encoder readings to linear distance traveled  
**Location:** `src/GravLib/drivebase/chassis.rs:149-150`

$$d = \frac{n_{revs} \times \pi \times D_{wheel}}{G_{ratio}}$$

Where:
- $n_{revs}$ = encoder revolutions
- $D_{wheel}$ = physical wheel diameter
- $G_{ratio}$ = mechanical gear reduction ratio
- $d$ = linear distance traveled

---

## Motor Control

### Velocity Percentage to RPM
**Purpose:** Convert percentage command to motor RPM  
**Location:** `src/GravLib/actuator/motor_group.rs:46-48`

$$\text{RPM} = \frac{\text{velocity percentage}}{100} \times \text{RPM}_{max}$$

Where $\text{RPM}_{max}$ varies by gearset:
- Red: 100 RPM
- Green: 200 RPM  
- Blue: 600 RPM

### Controller Input Scaling
**Purpose:** Scale controller input to motor voltage  
**Location:** Drive control files

$$V_{motor} = \frac{\text{controller input} \times 12V}{127}$$

Maps controller input range $[-127, +127]$ to motor voltage range $[-12V, +12V]$.

---

## Mathematical Constants

- $\pi = 3.14159265359...$
- Loop time: $\Delta t = 0.01s$ (10ms)
- EMA smoothing factor: $\alpha = 0.95$
- Controller input range: $[-127, +127]$
- Motor voltage range: $[-12V, +12V]$

---

## Implementation Notes

1. **Numerical Stability:** Arc calculations include checks for $\Delta\theta \approx 0$ to avoid division by zero.

2. **Sensor Fusion Priority:** Heading calculation follows strict priority: Horizontal wheels → Vertical wheels → IMU.

3. **Coordinate System:** Uses standard robotics convention (x-forward, y-left, θ-counterclockwise positive).

4. **Filtering:** EMA filters with α=0.95 provide good noise reduction while maintaining responsiveness.

5. **Units:** All angles internally stored in radians, distances in implementation-defined units (typically inches or meters).

---

## References

This formula sheet covers **28 distinct mathematical formulas** implementing:
- Trigonometric transformations and coordinate rotations
- Sensor fusion algorithms with prioritized heading calculation
- Arc motion modeling for curved robot paths  
- Digital filtering with EMA for noise reduction
- Classical PID control theory
- Kinematic modeling for wheel encoder conversion
- Vector mathematics for pose arithmetic and interpolation

The mathematics follows established robotics principles with proper handling of coordinate transformations, sensor fusion priority systems, and numerical stability considerations.