/*
 * =====================================================
 * File: obstacle.cpp
 * Purpose: HC-SR04 ultrasonic sensor - non-blocking
 *          interrupt-driven echo measurement
 * =====================================================
 * Trigger: millis()-based, every OBSTACLE_TRIGGER_MS (50 ms)
 * Echo:    RISING/FALLING interrupt on PIN_ECHO (18)
 *          measures pulse width with micros()
 * Distance = pulse_us / 58.0 (in cm, speed-of-sound formula)
 * =====================================================
 */

#include "obstacle.h"

// ------------------------------------------------
// INTERNAL CONSTANTS
// ------------------------------------------------
// Speed-of-sound formula: 1 cm is approximately 58 us round-trip
#define US_TO_CM_DIVISOR   58.0f

// Maximum valid distance for HC-SR04 (~400 cm)
#define MAX_DISTANCE_CM    400.0f

// TRIG pulse width required by HC-SR04 (>=10 us)
#define TRIG_PULSE_US      10

// ------------------------------------------------
// MODULE STATE
// ------------------------------------------------
// Accessed from ISR - must be volatile
static volatile unsigned long s_echoStart  = 0;    // Rising edge timestamp
static volatile unsigned long s_pulseDur   = 0;    // Measured pulse width (us)
static volatile bool          s_newReading = false; // New measurement flag

// Accessed from main task
static float          s_distCm   = -1.0f;   // Last valid distance (cm)
static unsigned long  s_lastTrig = 0;       // Last trigger timestamp (ms)
static bool           s_enabled  = true;    // Module enable flag

// ------------------------------------------------
// ECHO ISR - Interrupt Service Routine
// ------------------------------------------------
// IRAM_ATTR: runs from IRAM for deterministic latency
// ------------------------------------------------
void IRAM_ATTR echoISR()
{
    if (digitalRead(PIN_ECHO) == HIGH) {
        // Rising edge: pulse start
        s_echoStart = micros();
    } else {
        // Falling edge: pulse ended, compute duration
        unsigned long now = micros();
        if (now > s_echoStart) {
            s_pulseDur   = now - s_echoStart;
            s_newReading = true;
        }
    }
}

// =====================================================
// PUBLIC API IMPLEMENTATIONS
// =====================================================

// ------------------------------------------------
// obstacle_init - Pin setup and ISR attachment
// ------------------------------------------------
void obstacle_init()
{
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);

    // Attach ISR on both edges of ECHO
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echoISR, CHANGE);

    s_distCm    = -1.0f;
    s_lastTrig  = 0;
    s_newReading = false;
    s_enabled   = true;
}

// ------------------------------------------------
// obstacle_update - Main loop trigger and reading
// ------------------------------------------------
void obstacle_update()
{
    unsigned long now = millis();

    // Process any new ISR reading
    if (s_newReading) {
        // Copy ISR shared data, then clear flag atomically
        noInterrupts();
        unsigned long dur = s_pulseDur;
        s_newReading = false;
        interrupts();

        // Calculate distance
        float dist = (float)dur / US_TO_CM_DIVISOR;

        // Range check: 2 - 400 cm is valid for HC-SR04
        if (dist >= 2.0f && dist <= MAX_DISTANCE_CM) {
            s_distCm = dist;
        } else {
            s_distCm = -1.0f;
        }
    }

    // Trigger a new measurement every 50 ms
    if (!s_enabled) return;

    if ((now - s_lastTrig) >= OBSTACLE_TRIGGER_MS) {
        s_lastTrig = now;

        // HC-SR04 protocol: pulse TRIG HIGH for >=10 us
        digitalWrite(PIN_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(TRIG_PULSE_US);
        digitalWrite(PIN_TRIG, LOW);
    }
}

// ------------------------------------------------
// obstacle_get_distance - Return last valid reading
// ------------------------------------------------
float obstacle_get_distance()
{
    return s_distCm;
}

// ------------------------------------------------
// obstacle_get_level - Classify distance into level
// ------------------------------------------------
ObstacleLevel obstacle_get_level()
{
    if (!s_enabled || s_distCm < 0.0f) {
        return OBS_CLEAR;
    }

    if (s_distCm < (float)OBSTACLE_STOP_CM) {
        return OBS_STOP;   // < 15 cm: Stop
    } else if (s_distCm < (float)OBSTACLE_SLOW_CM) {
        return OBS_SLOW;   // 15 - 20 cm: Slow
    } else if (s_distCm < (float)OBSTACLE_WARN_CM) {
        return OBS_WARN;   // 20 - 30 cm: Warn
    } else {
        return OBS_CLEAR;  // >= 30 cm: Clear
    }
}

// ------------------------------------------------
// obstacle_is_enabled / obstacle_set_enabled
// ------------------------------------------------
bool obstacle_is_enabled()
{
    return s_enabled;
}

void obstacle_set_enabled(bool en)
{
    s_enabled = en;
    if (!en) {
        digitalWrite(PIN_TRIG, LOW);
        s_distCm = -1.0f;
    }
}
