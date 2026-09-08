/**
 * @file transmitter.ino
 * @brief Gesture Glove Transmitter — Arduino Nano
 *
 * Gesture-Controlled Robotic Arm Project
 * GitHub: https://github.com/vasu4990/gesture-controlled-robotic-arm
 *
 * Hardware:
 *   - Arduino Nano
 *   - MPU6050 6-axis IMU (I2C: SDA=A4, SCL=A5)
 *   - nRF24L01 2.4GHz transceiver (SPI: CE=D9, CSN=D10)
 *
 * Libraries required (install via Arduino Library Manager):
 *   - RF24 by TMRh20
 *   - MPU6050 by Electronic Cats (or Wire.h for raw I2C)
 *
 * Signal flow:
 *   MPU6050 → complementary filter → dead-zone → map → nRF24L01 TX
 */

#include "config.h"
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>


// ─── Radio
// ────────────────────────────────────────────────────────────────────
RF24 radio(9, 10); // CE=D9, CSN=D10

// ─── Data Packet ─────────────────────────────────────────────────────────────
struct ControlPacket {
  uint8_t baseAngle;     // 0–180°
  uint8_t shoulderAngle; // 0–180°
  uint8_t elbowAngle;    // 0–180°
  uint8_t gripperAngle;  // 0–180°
};

// ─── IMU State
// ────────────────────────────────────────────────────────────────
float pitch = 0.0f;
float roll = 0.0f;
unsigned long lastTime = 0;

// ─── Setup
// ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // --- MPU6050 Init ---
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up (clear sleep bit)
  Wire.endTransmission(true);

  // Set gyro range to ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set accel range to ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // --- nRF24L01 Init ---
  if (!radio.begin()) {
    Serial.println(F("ERROR: nRF24L01 not found. Check wiring!"));
    while (true)
      ;
  }
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS); // Slower = better range & reliability
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openWritingPipe(RF_WRITE_PIPE);
  radio.stopListening();

  Serial.println(F("Transmitter ready. Sending gesture data..."));
  lastTime = micros();
}

// ─── Read IMU Raw Data
// ────────────────────────────────────────────────────────
struct RawIMU {
  float ax, ay, az; // Accelerometer (g)
  float gx, gy, gz; // Gyroscope (°/s)
};

RawIMU readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t ax_raw = (Wire.read() << 8) | Wire.read();
  int16_t ay_raw = (Wire.read() << 8) | Wire.read();
  int16_t az_raw = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read(); // Skip temperature
  int16_t gx_raw = (Wire.read() << 8) | Wire.read();
  int16_t gy_raw = (Wire.read() << 8) | Wire.read();
  int16_t gz_raw = (Wire.read() << 8) | Wire.read();

  RawIMU imu;
  imu.ax = ax_raw / ACCEL_SCALE;
  imu.ay = ay_raw / ACCEL_SCALE;
  imu.az = az_raw / ACCEL_SCALE;
  imu.gx = gx_raw / GYRO_SCALE;
  imu.gy = gy_raw / GYRO_SCALE;
  imu.gz = gz_raw / GYRO_SCALE;
  return imu;
}

// ─── Complementary Filter
// ─────────────────────────────────────────────────────
void updateAngles(const RawIMU &imu, float dt) {
  // Accelerometer-based angle estimate
  float accelPitch = atan2(imu.ay, imu.az) * 180.0f / PI;
  float accelRoll =
      atan2(-imu.ax, sqrt(imu.ay * imu.ay + imu.az * imu.az)) * 180.0f / PI;

  // Gyroscope integration
  pitch = COMP_FILTER_ALPHA * (pitch + imu.gx * dt) +
          (1.0f - COMP_FILTER_ALPHA) * accelPitch;
  roll = COMP_FILTER_ALPHA * (roll + imu.gy * dt) +
         (1.0f - COMP_FILTER_ALPHA) * accelRoll;
}

// ─── Apply Dead Zone ─────────────────────────────────────────────────────────
float applyDeadZone(float angle, float deadZone) {
  if (fabs(angle) < deadZone)
    return 0.0f;
  return angle;
}

// ─── Map Float to Servo Angle
// ─────────────────────────────────────────────────
uint8_t mapAngle(float value, float inMin, float inMax, uint8_t outMin,
                 uint8_t outMax) {
  float mapped = (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
  return (uint8_t)constrain(mapped, outMin, outMax);
}

// ─── Main Loop
// ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6f; // seconds
  lastTime = now;

  RawIMU imu = readIMU();
  updateAngles(imu, dt);

  // Apply dead-zone to avoid jitter at rest
  float filteredPitch = applyDeadZone(pitch, DEAD_ZONE_PITCH);
  float filteredRoll = applyDeadZone(roll, DEAD_ZONE_ROLL);

  // Build control packet
  ControlPacket pkt;
  // Base: controlled by roll (left/right tilt)
  pkt.baseAngle =
      mapAngle(filteredRoll, ROLL_MIN_ANGLE, ROLL_MAX_ANGLE, 0, 180);
  // Shoulder: controlled by pitch (forward/back tilt)
  pkt.shoulderAngle =
      mapAngle(filteredPitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE, 30, 150);
  // Elbow: fixed mid-range for now; extend with second IMU or flex sensor
  pkt.elbowAngle = 90;
  // Gripper: open (flat hand) or closed (fist) — extend with flex sensor on
  // finger
  pkt.gripperAngle = 30; // Default open

  // Transmit
  bool ok = radio.write(&pkt, sizeof(pkt));

  // Debug output
  Serial.print(F("P:"));
  Serial.print(pitch, 1);
  Serial.print(F(" R:"));
  Serial.print(roll, 1);
  Serial.print(F(" | Base:"));
  Serial.print(pkt.baseAngle);
  Serial.print(F(" Shoulder:"));
  Serial.print(pkt.shoulderAngle);
  Serial.print(F(" TX:"));
  Serial.println(ok ? F("OK") : F("FAIL"));

  delay(LOOP_INTERVAL_MS);
}
