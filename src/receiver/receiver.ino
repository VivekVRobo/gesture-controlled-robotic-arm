/**
 * @file receiver.ino
 * @brief Hardened Robotic Arm Receiver — Arduino Uno + PCA9685 + nRF24L01
 *
 * Gesture-Controlled Robotic Arm Project
 * GitHub: https://github.com/VivekVRobo/gesture-controlled-robotic-arm
 *
 * Hardware:
 *   - Arduino Uno
 *   - PCA9685 16-channel PWM driver (I2C: SDA=A4, SCL=A5)
 *   - nRF24L01 2.4GHz transceiver (SPI: CE=D9, CSN=D10)
 *   - Servos on PCA9685 channels 0–3 (Base, Shoulder, Elbow, Gripper)
 *
 * Hardened Features:
 *   1. Decoupled Architecture: Non-blocking 50Hz control loop independent of RF packet arrivals.
 *   2. Radio Watchdog & Failsafe: Automatically detects packet dropouts (>1000ms) and smoothly parks
 *      the manipulator in safe neutral home coordinates without gear shock.
 *   3. Dynamic Velocity Profile Ramping: Exponential asymptotic deceleration with strict per-joint
 *      angular speed limits (clamped degrees/tick) eliminating mechanical jerk and current surges.
 *   4. Diagnostic Telemetry: 1Hz structured serial telemetry reporting link state, packet throughput (Hz),
 *      and active joint angles.
 */

#include "config.h"
#include <Adafruit_PWMServoDriver.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

// ─── Radio & Actuator Drivers ────────────────────────────────────────────────
RF24 radio(9, 10); // CE=D9, CSN=D10
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ─── Data Packet (Binary contract matching transmitter) ───────────────────────
struct ControlPacket {
  uint8_t baseAngle;     // 0–180°
  uint8_t shoulderAngle; // 0–180°
  uint8_t elbowAngle;    // 0–180°
  uint8_t gripperAngle;  // 0–180°
};

// ─── Link State Machine ──────────────────────────────────────────────────────
enum LinkState {
  LINK_STATE_INIT,   // Booted, waiting for initial radio handshake
  LINK_STATE_ONLINE, // Telemetry stream active and healthy
  LINK_STATE_LOST    // Packets timed out; failsafe parking engaged
};

LinkState linkState = LINK_STATE_INIT;

// ─── State & Coordinate Storage ──────────────────────────────────────────────
// Current actual joint positions (degrees)
float currentBase = BASE_HOME;
float currentShoulder = SHOULDER_HOME;
float currentElbow = ELBOW_HOME;
float currentGripper = GRIPPER_HOME;

// Target setpoints commanded by glove (or failsafe)
float targetBase = BASE_HOME;
float targetShoulder = SHOULDER_HOME;
float targetElbow = ELBOW_HOME;
float targetGripper = GRIPPER_HOME;

// ─── Non-Blocking Timers & Metrics ───────────────────────────────────────────
unsigned long lastPacketMillis = 0;
unsigned long lastControlTick = 0;
unsigned long lastTelemetryMillis = 0;

uint16_t packetsReceivedWindow = 0;
uint16_t packetsLostEvents = 0;

// ─── Actuation Helpers ───────────────────────────────────────────────────────

/**
 * Convert joint angle (degrees) to PCA9685 12-bit PWM tick count (0–4095).
 * Tick = (pulse_us / 1e6) * PWM_FREQUENCY * 4096
 */
uint16_t angleToPWM(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  float pulseUs = map(angle, 0.0f, 180.0f, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  return (uint16_t)(pulseUs / 1e6f * PWM_FREQUENCY * 4096.0f);
}

/**
 * Write calibrated PWM pulse to a specific PCA9685 servo channel.
 */
void writeServo(uint8_t channel, float angle) {
  pwm.setPWM(channel, 0, angleToPWM(angle));
}

/**
 * Smoothly ramp current joint angle toward target using velocity-clamped
 * asymptotic deceleration. Eliminates instantaneous acceleration jerk.
 *
 * @param current Current joint angle in degrees
 * @param target Desired target angle in degrees
 * @param maxStep Maximum allowed angular displacement per control tick
 * @return New interpolated joint angle
 */
float updateJointTrajectory(float current, float target, float maxStep) {
  float error = target - current;
  if (fabs(error) <= MIN_DEADBAND_DEG) {
    return current;
  }

  // Calculate exponential step based on proportional error
  float desiredStep = error * VELOCITY_SMOOTH_ALPHA;

  // Enforce velocity ceiling (degrees per 20ms tick)
  if (desiredStep > maxStep) {
    desiredStep = maxStep;
  } else if (desiredStep < -maxStep) {
    desiredStep = -maxStep;
  }

  // Ensure monotonic progression if outside deadband
  if (fabs(desiredStep) < MIN_DEADBAND_DEG && fabs(error) > MIN_DEADBAND_DEG) {
    desiredStep = (error > 0.0f) ? MIN_DEADBAND_DEG : -MIN_DEADBAND_DEG;
  }

  return current + desiredStep;
}

/**
 * Optional Planar 2-Link Inverse Kinematics Helper (Law of Cosines).
 * Maps Cartesian end-effector coordinates (x, y) in mm to (θ1, θ2) joint angles.
 */
bool inverseKinematics(float x, float y, float &theta1, float &theta2) {
  float L1 = LINK1_LEN;
  float L2 = LINK2_LEN;

  float distSq = x * x + y * y;
  float cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  if (cosTheta2 < -1.0f || cosTheta2 > 1.0f) {
    return false; // Target point is outside kinematic reach
  }

  theta2 = acos(cosTheta2) * 180.0f / PI; // Elbow angle in degrees
  float k1 = L1 + L2 * cos(theta2 * PI / 180.0f);
  float k2 = L2 * sin(theta2 * PI / 180.0f);
  theta1 = (atan2(y, x) - atan2(k2, k1)) * 180.0f / PI; // Shoulder angle in degrees

  return true;
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  delay(200);
  Serial.println(F("===================================================="));
  Serial.println(F("  GESTURE-CONTROLLED ROBOTIC ARM — RECEIVER FIRMWARE"));
  Serial.println(F("  Hardened Watchdog & Velocity Ramping Active"));
  Serial.println(F("===================================================="));

  // --- PCA9685 PWM Driver Init ---
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQUENCY);
  delay(10);

  // Initialize servos to safe home posture
  writeServo(SERVO_BASE, currentBase);
  writeServo(SERVO_SHOULDER, currentShoulder);
  writeServo(SERVO_ELBOW, currentElbow);
  writeServo(SERVO_GRIPPER, currentGripper);
  Serial.println(F("[INIT] Servos initialized to safe neutral coordinates."));

  // --- nRF24L01 Radio Init ---
  if (!radio.begin()) {
    Serial.println(F("[FATAL] nRF24L01 hardware failure. Check CE/CSN and 3.3V rail."));
    while (true) {
      delay(1000);
    }
  }

  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS); // 250kbps for maximum RF link budget and penetration
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openReadingPipe(1, RF_READ_PIPE);
  radio.startListening();

  lastPacketMillis = millis();
  lastControlTick = millis();
  lastTelemetryMillis = millis();

  Serial.println(F("[RADIO] nRF24L01 listening on Channel 100 @ 250kbps."));
  Serial.println(F("[READY] Awaiting telemetry stream from gesture glove..."));
}

// ─── Main Execution Loop ─────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── 1. Telemetry Ingestion ─────────────────────────────────────────────────
  while (radio.available()) {
    ControlPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    // Clamp input payload to physical joint safety limits
    targetBase = constrain((float)pkt.baseAngle, (float)BASE_MIN, (float)BASE_MAX);
    targetShoulder = constrain((float)pkt.shoulderAngle, (float)SHOULDER_MIN, (float)SHOULDER_MAX);
    targetElbow = constrain((float)pkt.elbowAngle, (float)ELBOW_MIN, (float)ELBOW_MAX);
    targetGripper = constrain((float)pkt.gripperAngle, (float)GRIPPER_OPEN, (float)GRIPPER_CLOSED);

    packetsReceivedWindow++;
    lastPacketMillis = now;

    if (linkState != LINK_STATE_ONLINE) {
      linkState = LINK_STATE_ONLINE;
      Serial.println(F("[LINK] Telemetry link acquired. Normal actuation active."));
    }
  }

  // ── 2. Link Watchdog & Failsafe Evaluation ──────────────────────────────────
  unsigned long timeSinceLastPacket = now - lastPacketMillis;

  if (linkState == LINK_STATE_ONLINE && timeSinceLastPacket > RADIO_TIMEOUT_MS) {
    linkState = LINK_STATE_LOST;
    packetsLostEvents++;
    Serial.println(F("[WATCHDOG] Radio link timeout (>1000ms). Engaging failsafe park."));
  }

  // When link is lost or initialising, override targets to safe neutral coordinates
  if (linkState == LINK_STATE_LOST || linkState == LINK_STATE_INIT) {
    targetBase = BASE_HOME;
    targetShoulder = SHOULDER_HOME;
    targetElbow = ELBOW_HOME;
    targetGripper = GRIPPER_HOME;
  }

  // ── 3. Deterministic 50Hz Motion Control Loop ──────────────────────────────
  if (now - lastControlTick >= CONTROL_INTERVAL_MS) {
    lastControlTick = now;

    // Use gentle parking step limits during failsafe, or full operational limits when online
    float stepBase = (linkState == LINK_STATE_LOST) ? FAILSAFE_PARK_STEP : MAX_VEL_BASE;
    float stepShoulder = (linkState == LINK_STATE_LOST) ? FAILSAFE_PARK_STEP : MAX_VEL_SHOULDER;
    float stepElbow = (linkState == LINK_STATE_LOST) ? FAILSAFE_PARK_STEP : MAX_VEL_ELBOW;
    float stepGripper = (linkState == LINK_STATE_LOST) ? FAILSAFE_PARK_STEP : MAX_VEL_GRIPPER;

    // Compute velocity-profiled trajectories
    currentBase = updateJointTrajectory(currentBase, targetBase, stepBase);
    currentShoulder = updateJointTrajectory(currentShoulder, targetShoulder, stepShoulder);
    currentElbow = updateJointTrajectory(currentElbow, targetElbow, stepElbow);
    currentGripper = updateJointTrajectory(currentGripper, targetGripper, stepGripper);

    // Command PWM output to servos
    writeServo(SERVO_BASE, currentBase);
    writeServo(SERVO_SHOULDER, currentShoulder);
    writeServo(SERVO_ELBOW, currentElbow);
    writeServo(SERVO_GRIPPER, currentGripper);
  }

  // ── 4. Non-Blocking 1Hz Diagnostics & Telemetry ────────────────────────────
  if (now - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS) {
    unsigned long dtTelem = now - lastTelemetryMillis;
    lastTelemetryMillis = now;

    float pps = (packetsReceivedWindow * 1000.0f) / (float)dtTelem;
    packetsReceivedWindow = 0;

    Serial.print(F("[TELEM] Link: "));
    if (linkState == LINK_STATE_ONLINE) {
      Serial.print(F("ONLINE ("));
      Serial.print(pps, 1);
      Serial.print(F(" Hz)"));
    } else if (linkState == LINK_STATE_LOST) {
      Serial.print(F("LOST (idle "));
      Serial.print(timeSinceLastPacket);
      Serial.print(F(" ms, "));
      Serial.print(packetsLostEvents);
      Serial.print(F(" drops) [SAFE_PARK]"));
    } else {
      Serial.print(F("WAITING_FOR_GLOVE"));
    }

    Serial.print(F(" | Base:"));
    Serial.print(currentBase, 1);
    Serial.print(F(" Shld:"));
    Serial.print(currentShoulder, 1);
    Serial.print(F(" Elb:"));
    Serial.print(currentElbow, 1);
    Serial.print(F(" Grip:"));
    Serial.println(currentGripper, 1);
  }
}
