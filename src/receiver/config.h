/**
 * @file config.h
 * @brief Hardened Receiver (Robotic Arm) Configuration
 *
 * Gesture-Controlled Robotic Arm
 * Receiver Side — Arduino Uno + PCA9685 + nRF24L01
 */

#ifndef CONFIG_H
#define CONFIG_H

// ─── nRF24L01 Radio ──────────────────────────────────────────────────────────
#define RF_CHANNEL 100              // Must match transmitter
#define RF_READ_PIPE 0xE8E8F0F0E1LL // Must match transmitter write pipe

// ─── PCA9685 PWM Driver (I2C) ────────────────────────────────────────────────
#define PCA9685_ADDR 0x40
#define PWM_FREQUENCY 50 // 50Hz standard servo frequency

// Servo channel assignments on PCA9685
#define SERVO_BASE 0     // Base rotation
#define SERVO_SHOULDER 1 // Shoulder joint
#define SERVO_ELBOW 2    // Elbow joint
#define SERVO_GRIPPER 3  // Gripper open/close

// ─── Servo PWM Pulse Widths (microseconds) ───────────────────────────────────
// TowerPro SG90 / MG996R standard calibration: 500µs = 0°, 2400µs = 180°
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400

// ─── Joint Angle Limits (degrees) ────────────────────────────────────────────
#define BASE_MIN 0
#define BASE_MAX 180
#define SHOULDER_MIN 30
#define SHOULDER_MAX 150
#define ELBOW_MIN 0
#define ELBOW_MAX 160
#define GRIPPER_OPEN 30
#define GRIPPER_CLOSED 120

// ─── Radio Failsafe & Watchdog ───────────────────────────────────────────────
#define RADIO_TIMEOUT_MS 1000     // Lost link trigger threshold (milliseconds)
#define FAILSAFE_PARK_STEP 0.8f   // Safe, gentle parking velocity (deg/tick @ 50Hz = 40 deg/s)

// Safe Home / Park Joint Coordinates (degrees)
#define BASE_HOME 90.0f
#define SHOULDER_HOME 90.0f
#define ELBOW_HOME 90.0f
#define GRIPPER_HOME GRIPPER_OPEN

// ─── Motion Profiling & Velocity Limits ──────────────────────────────────────
// Maximum angular displacement per 20ms control tick (degrees per tick):
// 1.0 deg/tick = 50 deg/sec | 2.0 deg/tick = 100 deg/sec | 3.0 deg/tick = 150 deg/sec
#define MAX_VEL_BASE 2.0f      // Base turntable: 100 deg/s max (high rotational inertia)
#define MAX_VEL_SHOULDER 1.8f  // Shoulder: 90 deg/s max (high gravitational load)
#define MAX_VEL_ELBOW 2.2f     // Elbow joint: 110 deg/s max
#define MAX_VEL_GRIPPER 4.0f   // Gripper actuator: 200 deg/s max

// Exponential approach factor (0.05 to 0.40):
// Controls deceleration curvature approaching target; prevents abrupt deceleration shock
#define VELOCITY_SMOOTH_ALPHA 0.22f
#define MIN_DEADBAND_DEG 0.25f // Minimum angle delta to prevent micro-oscillation servo hunting

// ─── Arm Geometry (mm) ───────────────────────────────────────────────────────
#define LINK1_LEN 120.0f // Upper arm length
#define LINK2_LEN 100.0f // Forearm length

// ─── Deterministic Scheduling ────────────────────────────────────────────────
#define CONTROL_INTERVAL_MS 20      // Deterministic 50 Hz motion loop
#define TELEMETRY_INTERVAL_MS 1000  // Diagnostic reporting period (1 Hz)

#endif // CONFIG_H
