/**
 * @file receiver.ino
 * @brief Robotic Arm Receiver — Arduino Uno
 *
 * Gesture-Controlled Robotic Arm Project
 * GitHub: https://github.com/vasu4990/gesture-controlled-robotic-arm
 *
 * Hardware:
 *   - Arduino Uno
 *   - PCA9685 16-channel PWM driver (I2C: SDA=A4, SCL=A5)
 *   - nRF24L01 2.4GHz transceiver (SPI: CE=D9, CSN=D10)
 *   - hobby servos on PCA9685 channels 0–3
 *
 * Libraries required (install via Arduino Library Manager):
 *   - RF24 by TMRh20
 *   - Adafruit PWM Servo Driver Library by Adafruit
 *
 * Active signal flow:
 *   nRF24L01 RX → joint limits → interpolation → PCA9685 → servos
 *
 * Note: a two-link inverse-kinematics helper is included below for future
 * Cartesian-command experiments, but it is not used by the active packet path.
 */

#include "config.h"
#include <Adafruit_PWMServoDriver.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>


// ─── Radio
// ────────────────────────────────────────────────────────────────────
RF24 radio(9, 10); // CE=D9, CSN=D10

// ─── PCA9685 ─────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ─── Data Packet (must match transmitter) ────────────────────────────────────
struct ControlPacket {
  uint8_t baseAngle;
  uint8_t shoulderAngle;
  uint8_t elbowAngle;
  uint8_t gripperAngle;
};

// ─── Current servo positions (for smooth interpolation) ──────────────────────
float currentBase = 90.0f;
float currentShoulder = 90.0f;
float currentElbow = 90.0f;
float currentGripper = GRIPPER_OPEN;

// ─── Helpers
// ──────────────────────────────────────────────────────────────────

/**
 * Convert angle in degrees to PCA9685 PWM tick count.
 * Tick = (pulse_us / 1e6) * PWM_FREQUENCY * 4096
 */
uint16_t angleToPWM(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  float pulseUs =
      map(angle, 0.0f, 180.0f, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  return (uint16_t)(pulseUs / 1e6f * PWM_FREQUENCY * 4096);
}

/**
 * Write angle to a specific PCA9685 servo channel.
 */
void writeServo(uint8_t channel, float angle) {
  pwm.setPWM(channel, 0, angleToPWM(angle));
}

/**
 * Smoothly step currentAngle toward targetAngle by INTERP_STEP.
 */
float interpolate(float current, float target) {
  float diff = target - current;
  if (fabs(diff) <= INTERP_STEP)
    return target;
  return current + (diff > 0 ? INTERP_STEP : -INTERP_STEP);
}

/**
 * Inverse Kinematics — law of cosines.
 * Maps (x, y) end-effector Cartesian position → (θ1, θ2) joint angles.
 * Returns false if target is out of reach.
 *
 * This helper is not part of the active gesture packet path.
 */
bool inverseKinematics(float x, float y, float &theta1, float &theta2) {
  float L1 = LINK1_LEN;
  float L2 = LINK2_LEN;

  float distSq = x * x + y * y;
  float cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

  if (cosTheta2 < -1.0f || cosTheta2 > 1.0f)
    return false; // Unreachable

  theta2 = acos(cosTheta2) * 180.0f / PI; // Elbow angle (degrees)
  float k1 = L1 + L2 * cos(theta2 * PI / 180.0f);
  float k2 = L2 * sin(theta2 * PI / 180.0f);
  theta1 = (atan2(y, x) - atan2(k2, k1)) * 180.0f / PI; // Shoulder angle

  return true;
}

// ─── Setup
// ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // --- PCA9685 Init ---
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(PWM_FREQUENCY);
  delay(10);

  // Move all servos to home/safe position
  writeServo(SERVO_BASE, 90);
  writeServo(SERVO_SHOULDER, 90);
  writeServo(SERVO_ELBOW, 90);
  writeServo(SERVO_GRIPPER, GRIPPER_OPEN);
  Serial.println(F("Servos homed."));

  // --- nRF24L01 Init ---
  if (!radio.begin()) {
    Serial.println(F("ERROR: nRF24L01 not found. Check wiring!"));
    while (true)
      ;
  }
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openReadingPipe(1, RF_READ_PIPE);
  radio.startListening();

  Serial.println(F("Receiver ready. Waiting for gesture data..."));
}

// ─── Main Loop
// ────────────────────────────────────────────────────────────────
void loop() {
  ControlPacket pkt;

  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));

    // Clamp incoming angles to defined joint limits
    float targetBase = constrain((float)pkt.baseAngle, BASE_MIN, BASE_MAX);
    float targetShoulder =
        constrain((float)pkt.shoulderAngle, SHOULDER_MIN, SHOULDER_MAX);
    float targetElbow = constrain((float)pkt.elbowAngle, ELBOW_MIN, ELBOW_MAX);
    float targetGripper =
        constrain((float)pkt.gripperAngle, GRIPPER_OPEN, GRIPPER_CLOSED);

    // Optional future Cartesian command path. This is intentionally disabled;
    // the current transmitter sends joint-angle targets directly.
    /*
    float x = pkt.baseAngle;
    float y = pkt.shoulderAngle;
    float ikShoulder, ikElbow;
    if (inverseKinematics(x, y, ikShoulder, ikElbow)) {
        targetShoulder = ikShoulder;
        targetElbow = ikElbow;
    }
    */

    // Smoothly interpolate to target
    currentBase = interpolate(currentBase, targetBase);
    currentShoulder = interpolate(currentShoulder, targetShoulder);
    currentElbow = interpolate(currentElbow, targetElbow);
    currentGripper = interpolate(currentGripper, targetGripper);

    // Write to servos
    writeServo(SERVO_BASE, currentBase);
    writeServo(SERVO_SHOULDER, currentShoulder);
    writeServo(SERVO_ELBOW, currentElbow);
    writeServo(SERVO_GRIPPER, currentGripper);

    // Debug
    Serial.print(F("Base:"));
    Serial.print(currentBase, 0);
    Serial.print(F(" Shld:"));
    Serial.print(currentShoulder, 0);
    Serial.print(F(" Elbow:"));
    Serial.print(currentElbow, 0);
    Serial.print(F(" Grip:"));
    Serial.println(currentGripper, 0);
  }

  delay(LOOP_INTERVAL_MS);
}
