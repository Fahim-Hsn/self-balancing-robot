/*
 * =====================================================================
 * FILE: src/balance.cpp
 * PURPOSE: MPU-6050 raw I2C driver, complementary filter pitch
 *          estimation, and PID controller for RoboBot self-balance.
 * =====================================================================
 * Dependencies:
 *   - Wire.h  (Arduino I2C library - ESP32 built-in)
 *   - math.h  (atan2, fabs)
 *   - config.h (all defined constants)
 * NO external MPU library - raw register access only.
 * =====================================================================
 */

#include "balance.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

// -----------------------------------------------------------------------
// MPU-6050 REGISTER MAP (raw I2C)
// -----------------------------------------------------------------------
// PWR_MGMT_1 (0x6B): Power management. Write 0x00 to wake the chip.
#define MPU_REG_PWR_MGMT_1   0x6B

// CONFIG (0x1A): Digital Low Pass Filter (DLPF) setting.
// 0x03 = ~43 Hz bandwidth, reduces mechanical vibration noise.
#define MPU_REG_CONFIG       0x1A

// GYRO_CONFIG (0x1B): Gyroscope full-scale range.
// 0x00 = +/-250 deg/s, scale factor 131 LSB/(deg/s)
#define MPU_REG_GYRO_CONFIG  0x1B

// ACCEL_CONFIG (0x1C): Accelerometer full-scale range.
// 0x00 = +/-2g, scale factor 16384 LSB/g
#define MPU_REG_ACCEL_CONFIG 0x1C

// ACCEL_XOUT_H (0x3B): First of 14 consecutive output registers.
// 14 bytes in order: AcX, AcY, AcZ, Temp, GyX, GyY, GyZ (big-endian).
#define MPU_REG_DATA_START   0x3B
#define MPU_DATA_BYTES       14

// -----------------------------------------------------------------------
// MODULE STATE (private / static)
// -----------------------------------------------------------------------

// Calibration offsets subtracted from raw readings to remove bias
static float s_gyroYOffset   = 0.0f; // Gyro Y bias (LSB)
static float s_pitchOffset   = 0.0f; // Pitch offset at vertical balance point (degrees)

// Angle and gyro state
static float s_angle    = 0.0f; // Current pitch estimate (degrees)
static float s_gyroRate = 0.0f; // Gyro Y rate after scaling (deg/s)

// Timing
static unsigned long s_lastMicros = 0; // Timestamp of last balance_update()
static float         s_lastDt     = 0.01f; // Measured dt of last update (seconds)

// PID state
static float s_pidKp         = PID_KP;
static float s_pidKi         = PID_KI;
static float s_pidKd         = PID_KD;
static float s_targetTrim    = 0.0f;     // Real-time target angle trim offset (degrees)
static float s_motorMinSpeed = MOTOR_MIN_SPEED; // Starting torque deadband offset
static float s_pidIntegral   = 0.0f;     // Accumulated integral term
static float s_pidPrevError  = 0.0f;     // Previous error for derivative
static float s_gyroDamped    = 0.0f;     // Low-pass filtered gyro rate for vibration elimination
static int   s_lastOutput    = 0;        // Previous motor output command
static bool  s_calibrated    = false;    // Calibration status flag

// -----------------------------------------------------------------------
// LOW-LEVEL I2C HELPERS
// -----------------------------------------------------------------------

/**
 * mpu_write_reg - Write one byte to an MPU-6050 register.
 * @param reg   Register address
 * @param value Byte to write
 */
static void mpu_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true); // Send STOP condition
}

/**
 * mpu_read_raw - Read the 14-byte sensor data burst starting at 0x3B.
 * Auto-increments from AcX through GyZ in a single I2C burst transaction.
 */
static void mpu_read_raw(int16_t &acX, int16_t &acY, int16_t &acZ,
                          int16_t &gyX, int16_t &gyY, int16_t &gyZ) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU_REG_DATA_START);
    Wire.endTransmission(false); // Repeated START, keep bus active

    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)MPU_DATA_BYTES, (uint8_t)true);

    acX = (int16_t)((Wire.read() << 8) | Wire.read());
    acY = (int16_t)((Wire.read() << 8) | Wire.read());
    acZ = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // Skip temperature registers
    gyX = (int16_t)((Wire.read() << 8) | Wire.read());
    gyY = (int16_t)((Wire.read() << 8) | Wire.read());
    gyZ = (int16_t)((Wire.read() << 8) | Wire.read());
}

/**
 * mpu_compute_raw_pitch - Compute pitch angle for vertical-mounted MPU-6050.
 * On vertical chassis:
 *   X points UP along chassis (+16384 LSB when vertical)
 *   Z points FORWARD (out of PCB plane)
 *   Pitch angle = atan2(-acZ, acX) * (180.0 / PI)
 *   When upright: acZ = 0, acX = +16384 -> atan2(0, 16384) = 0.0 deg
 *   When tilted forward: acZ is negative, -acZ is positive -> positive pitch angle
 */
static float mpu_compute_raw_pitch(int16_t acX, int16_t acZ) {
    return atan2f(-(float)acZ, (float)acX) * (180.0f / (float)M_PI);
}

// -----------------------------------------------------------------------
// balance_init()
// Configures MPU-6050 registers (wake-up, DLPF, ranges)
// -----------------------------------------------------------------------
void balance_init() {
    // Step 1: Wake MPU-6050 from power-on sleep mode
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);
    delay(100);

    // Step 2: Set DLPF to ~43 Hz bandwidth to filter vibrations
    mpu_write_reg(MPU_REG_CONFIG, 0x03);

    // Step 3: Gyroscope range = +/-250 deg/s (131 LSB per deg/s)
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x00);

    // Step 4: Accelerometer range = +/-2g (16384 LSB per g)
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);

    Serial.println("[balance_init] MPU-6050 configured OK");
}

// -----------------------------------------------------------------------
// balance_calibrate()
// Collects 500 samples to establish gyro bias and upright balance angle.
// User must hold the robot upright at its physical balance point!
// -----------------------------------------------------------------------
bool balance_calibrate() {
    Serial.println("[balance_calibrate] Starting calibration...");
    Serial.println("  Hold the robot VERTICAL and STILL at its balance point!");

    // Discard initial readings during power stabilisation
    int16_t acX, acY, acZ, gyX, gyY, gyZ;
    for (int i = 0; i < 50; i++) {
        mpu_read_raw(acX, acY, acZ, gyX, gyY, gyZ);
        delay(5);
    }

    const int SAMPLES = 500;
    long sumGyY = 0;
    float sumPitch = 0.0f;

    for (int i = 0; i < SAMPLES; i++) {
        mpu_read_raw(acX, acY, acZ, gyX, gyY, gyZ);
        sumGyY += gyY;
        sumPitch += mpu_compute_raw_pitch(acX, acZ);
        delay(4);

        if ((i + 1) % 100 == 0) {
            int remaining = SAMPLES - (i + 1);
            Serial.print("  Calibrating... samples remaining: ");
            Serial.println(remaining);
        }
    }

    // Gyro Y offset: bias calibration
    s_gyroYOffset = (float)sumGyY / (float)SAMPLES;

    // Upright pitch offset: neutral center-of-mass balance point
    s_pitchOffset = sumPitch / (float)SAMPLES;

    // Initialise angle estimate to 0 (perfectly calibrated upright)
    s_angle      = 0.0f;
    s_lastMicros = micros();
    s_calibrated = true;

    Serial.println("[balance_calibrate] Done!");
    Serial.print("  gyroY bias offset (LSB): "); Serial.println(s_gyroYOffset, 2);
    Serial.print("  upright balance pitch offset (deg): "); Serial.println(s_pitchOffset, 2);
    return true;
}

// -----------------------------------------------------------------------
// balance_update()
// Called every loop iteration. Reads MPU-6050 and updates pitch angle at 200 Hz.
// Returns true when a new sample is computed, false otherwise.
// -----------------------------------------------------------------------
bool balance_update() {
    if (!s_calibrated) return false;

    // Enforce 5 ms interval (200 Hz fixed control frequency)
    unsigned long now = micros();
    unsigned long elapsed = now - s_lastMicros;

    if (elapsed < 5000UL) return false;

    // Guard against huge step if loop was stalled by WiFi/blocking calls (max 50ms)
    if (elapsed > 50000UL) elapsed = 50000UL;
    float dt = (float)elapsed * 1e-6f;
    s_lastMicros = now;
    s_lastDt     = dt;

    int16_t acX, acY, acZ, gyX, gyY, gyZ;
    mpu_read_raw(acX, acY, acZ, gyX, gyY, gyZ);

    // Calculate raw pitch and subtract calibrated upright balance offset
    float rawPitch = mpu_compute_raw_pitch(acX, acZ);
    float accelAngle = rawPitch - s_pitchOffset;

    // Calibrated gyro pitch rate in degrees per second
    // Inverted so positive rate corresponds with forward tilt
    s_gyroRate = -((float)gyY - s_gyroYOffset) / MPU_GYRO_SCALE;

    // Complementary filter: blend gyro integration with accelerometer anchor
    // angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accelAngle
    s_angle = COMP_FILTER_ALPHA * (s_angle + s_gyroRate * dt)
            + (1.0f - COMP_FILTER_ALPHA) * accelAngle;
    return true;
}

// -----------------------------------------------------------------------
// balance_compute(float forwardBias)
// PID controller for balancing. Clean P+I+D with friction compensation.
// Velocity damping removed: low-RPM N20 motors cannot afford the opposing
// correction - it caused the robot to fall before recovery was possible.
// -----------------------------------------------------------------------
int balance_compute(float forwardBias) {
    // Cut off motors and reset PID if robot has fallen over
    if (balance_is_fallen()) {
        balance_reset_pid();
        return 0;
    }

    // Clamp forwardBias within valid range
    if (forwardBias >  SEGWAY_MAX_TILT) forwardBias =  SEGWAY_MAX_TILT;
    if (forwardBias < -SEGWAY_MAX_TILT) forwardBias = -SEGWAY_MAX_TILT;

    // Setpoint: target balance angle + user trim + joystick forward lean
    float setpoint = PID_TARGET_ANGLE + s_targetTrim + forwardBias;

    // Error: how far from upright (positive = tilted forward)
    float error = s_angle - setpoint;

    // dt clamped to safe range
    float dt = s_lastDt;
    if (dt <= 0.001f || dt > 0.05f) dt = 0.005f;

    // Integral term: only accumulates when close to upright (zone integral)
    // Prevents windup when fallen far; leaks slowly when tilted far
    if (fabsf(error) < 2.5f) {
        s_pidIntegral += error * dt;
    } else {
        // Fast leak when far from balance point - prevents runaway
        s_pidIntegral *= 0.80f;
    }

    if (s_pidIntegral >  PID_INTEGRAL_MAX) s_pidIntegral =  PID_INTEGRAL_MAX;
    if (s_pidIntegral < -PID_INTEGRAL_MAX) s_pidIntegral = -PID_INTEGRAL_MAX;

    // Derivative: use lightly filtered gyro rate to suppress motor gear vibration
    // Alpha 0.60 (was 0.70) - slightly more responsive to actual tilt changes
    s_gyroDamped = 0.60f * s_gyroDamped + 0.40f * s_gyroRate;
    float derivative = s_gyroDamped;

    // PID output
    float output = (s_pidKp * error)
                 + (s_pidKd * derivative)
                 + (s_pidKi * s_pidIntegral);

    // Friction / deadband compensation:
    // Quiet zone: when nearly upright AND almost stationary, stop motors.
    // This prevents N20 gear chatter and micro-oscillation when balanced.
    // Outside quiet zone: immediately add starting torque so N20 reacts before
    // the robot tilts past the recovery limit (~2 degrees for low-RPM motors).
    if (fabsf(error) < 0.15f && fabsf(derivative) < 1.2f) {
        output = 0.0f;
    } else {
        if (output > 0.0f) {
            output += s_motorMinSpeed;
        } else if (output < 0.0f) {
            output -= s_motorMinSpeed;
        }
    }

    // Clamp output within allowable motor limits
    if (output >  PID_OUTPUT_MAX) output =  PID_OUTPUT_MAX;
    if (output <  PID_OUTPUT_MIN) output =  PID_OUTPUT_MIN;

    s_lastOutput = (int)output;

    // Detailed serial debug every 100ms (driven by caller frequency)
    static unsigned long s_dbgMicros = 0;
    unsigned long nowUs = micros();
    if (nowUs - s_dbgMicros >= 100000UL) {
        s_dbgMicros = nowUs;
        Serial.printf("[PID] Angle:%.2f Set:%.2f Err:%.2f Gyro:%.2f | P:%.1f I:%.1f D:%.1f | Out:%d\n",
            s_angle, setpoint, error, s_gyroRate,
            s_pidKp * error,
            s_pidKi * s_pidIntegral,
            s_pidKd * derivative,
            s_lastOutput);
    }

    return (int)output;
}

float balance_get_angle() {
    return s_angle;
}

float balance_get_gyro_rate() {
    return s_gyroRate;
}

void balance_set_pid(float kp, float ki, float kd, float targetTrim, float minSpeed) {
    s_pidKp         = kp;
    s_pidKi         = ki;
    s_pidKd         = kd;
    s_targetTrim    = targetTrim;
    s_motorMinSpeed = minSpeed;
}

void balance_get_pid(float &kp, float &ki, float &kd, float &targetTrim, float &minSpeed) {
    kp         = s_pidKp;
    ki         = s_pidKi;
    kd         = s_pidKd;
    targetTrim = s_targetTrim;
    minSpeed   = s_motorMinSpeed;
}

bool balance_is_fallen() {
    return fabsf(s_angle) > BALANCE_FALLEN_ANGLE;
}

bool balance_is_tilting() {
    return fabsf(s_angle) > BALANCE_WARN_ANGLE;
}

void balance_reset_pid() {
    s_pidIntegral  = 0.0f;
    s_pidPrevError = 0.0f;
    s_gyroDamped   = 0.0f;
    s_lastOutput   = 0;
}
