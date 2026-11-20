/*
 * =====================================================
 * RoboBot - src/main.cpp
 * Top-level Orchestration File
 * =====================================================
 * Coordinates all subsystems:
 *   motors, buzzer, obstacle, balance, display, webserver
 * setup() initialises all peripherals.
 * loop() updates each subsystem without blocking delays.
 * =====================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "motors.h"
#include "buzzer.h"
#include "obstacle.h"
#include "balance.h"
#include "display.h"
#include "robot_webserver.h"

// ============================================================
// GLOBAL STATE
// ============================================================

// Current operating mode (starts in self-balance Segway mode)
RobotMode     g_mode            = MODE_BALANCE_SEGWAY;

// Obstacle detection enable flag
bool          g_obstacleEnabled = true;

// Latest command received from web dashboard
// Defaults: speed 70%, buzzer enabled, joystick centered
WebCommand    g_webCmd          = { 0, 0, 70, true, false };

// Idle timer tracking joystick activity
unsigned long g_lastIdleMs      = 0;

// Mode info overlay display timer
unsigned long g_modeInfoTimer   = 0;
bool          g_showModeInfo    = false;

// ============================================================
// OBSTACLE AVOIDANCE STATE MACHINE (MODE_REGULAR_DRIVE)
// Active in regular drive mode when an obstacle is within 15 cm
// States: AVOID_IDLE -> AVOID_BACK -> AVOID_TURN -> AVOID_IDLE
// ============================================================
enum AvoidState { AVOID_IDLE, AVOID_BACK, AVOID_TURN };
static AvoidState    s_avoidState = AVOID_IDLE;
static unsigned long s_avoidTimer = 0;

// ============================================================
// BUTTON DEBOUNCE & STATE TRACKING
// Edge-triggered: triggers only on HIGH -> LOW transition.
// ============================================================
static unsigned long s_lastBtnModeMs = 0;
static unsigned long s_lastBtnObsMs  = 0;
static int           s_prevBtnMode   = HIGH;
static int           s_prevBtnObs    = HIGH;

// ============================================================
// HELPER: expressionName()
// EyeExpression enum -> const char* (for telemetry JSON)
// ============================================================
static const char* expressionName(EyeExpression e) {
    switch (e) {
        case EYE_NORMAL:     return "EYE_NORMAL";
        case EYE_HAPPY:      return "EYE_HAPPY";
        case EYE_SURPRISED:  return "EYE_SURPRISED";
        case EYE_ANGRY:      return "EYE_ANGRY";
        case EYE_SLEEPY:     return "EYE_SLEEPY";
        case EYE_LOOK_LEFT:  return "EYE_LOOK_LEFT";
        case EYE_LOOK_RIGHT: return "EYE_LOOK_RIGHT";
        case EYE_DIZZY:      return "EYE_DIZZY";
        case EYE_COOL:       return "EYE_COOL";
        case EYE_SMIRK:      return "EYE_SMIRK";
        case EYE_ZZZ:        return "EYE_ZZZ";
        case EYE_HEART:      return "EYE_HEART";
        case EYE_SWEAT:      return "EYE_SWEAT";
        case EYE_SPINNING:   return "EYE_SPINNING";
        default:             return "EYE_NORMAL";
    }
}

// ============================================================
// SETUP
// Runs once at power-on. Initialises every subsystem.
// ============================================================
void setup() {
    // 1. Serial monitor initialization
    Serial.begin(115200);
    Serial.println("\n=== RoboBot Starting ===");

    // 2. Start I2C bus shared by MPU-6050 and OLED
    Wire.begin(PIN_SDA, PIN_SCL);
    Serial.println("[I2C] Bus started");

    // 3. Initialise OLED display module
    display_init();
    Serial.println("[Display] Initialised");

    // 4. Play startup boot animation on OLED
    display_show_startup_animation();

    // 5. Initialise L298N motor driver PWM channels
    motors_init();
    Serial.println("[Motors] Initialised");

    // 6. Initialise buzzer driver (FreeRTOS task on Core 0)
    buzzer_init();
    Serial.println("[Buzzer] Initialised");

    // 7. Initialise ultrasonic sensor with echo interrupt
    obstacle_init();
    Serial.println("[Obstacle] Initialised");

    // 8. Initialise MPU-6050 and calibrate sensors
    balance_init();
    Serial.println("[Balance] MPU-6050 Initialised");

    display_show_text("Calibrating...", "Hold Upright!");
    balance_calibrate();
    Serial.println("[Balance] Calibration done");

    // 9. Initialise web server command linkage
    webserver_init(&g_webCmd);
    Serial.println("[WebServer] Initialised");

    // 10. Start WiFi AP + DNS captive portal + HTTP server
    display_set_expression(EYE_SPINNING);
    display_show_text("WiFi Starting", "RoboBot AP");
    webserver_start();
    Serial.println("[WebServer] Started - SSID: " WIFI_SSID);

    delay(500);

    // 11. WiFi ready indicator
    buzzer_wifi_connected();
    display_set_expression(EYE_SMIRK);
    display_show_text("Portal Active", "192.168.4.1");
    Serial.println("[WiFi] Captive portal active at 192.168.4.1");

    // 12. Play startup melody
    buzzer_startup_melody();

    // Configure push button inputs with internal pull-ups
    pinMode(PIN_BTN_MODE,     INPUT_PULLUP);
    pinMode(PIN_BTN_OBSTACLE, INPUT_PULLUP);

    g_lastIdleMs = millis();
    Serial.println("[Setup] Complete - entering main loop");
}

// ============================================================
// LOOP
// Runs repeatedly. Updates all subsystems without blocking.
// ============================================================
void loop() {
    unsigned long now = millis();

    // ---------------------------------------------------------
    // 1. Read Buttons (strict edge-triggered debouncing)
    // ---------------------------------------------------------
    {
        // Mode toggle button (Button 1)
        int curMode = digitalRead(PIN_BTN_MODE);
        if (curMode != s_prevBtnMode && (now - s_lastBtnModeMs) >= BUTTON_DEBOUNCE_MS) {
            s_lastBtnModeMs = now;
            s_prevBtnMode = curMode;

            // Only trigger on falling edge (HIGH -> LOW = pressed)
            if (curMode == LOW) {
                if (g_mode == MODE_BALANCE_SEGWAY) {
                    g_mode = MODE_REGULAR_DRIVE;
                } else {
                    g_mode = MODE_BALANCE_SEGWAY;
                    balance_reset_pid();
                }

                buzzer_mode_changed(g_mode);
                display_show_mode_info(g_mode, g_obstacleEnabled, buzzer_is_enabled());
                g_showModeInfo = true;
                g_modeInfoTimer = now;

                Serial.print("[Mode] Switched to ");
                Serial.println(g_mode == MODE_BALANCE_SEGWAY ? "BALANCE" : "DRIVE");
            }
        }

        // Obstacle detection toggle button (Button 2)
        int curObs = digitalRead(PIN_BTN_OBSTACLE);
        if (curObs != s_prevBtnObs && (now - s_lastBtnObsMs) >= BUTTON_DEBOUNCE_MS) {
            s_lastBtnObsMs = now;
            s_prevBtnObs = curObs;

            // Only trigger on falling edge (HIGH -> LOW = pressed)
            if (curObs == LOW) {
                g_obstacleEnabled = !g_obstacleEnabled;
                obstacle_set_enabled(g_obstacleEnabled);
                display_show_mode_info(g_mode, g_obstacleEnabled, buzzer_is_enabled());
                g_showModeInfo = true;
                g_modeInfoTimer = now;

                Serial.print("[Obstacle] ");
                Serial.println(g_obstacleEnabled ? "ENABLED" : "DISABLED");
            }
        }
    }

    if (g_showModeInfo && (now - g_modeInfoTimer) >= DISPLAY_MODE_SHOW_MS) {
        g_showModeInfo = false;
    }

    // ---------------------------------------------------------
    // 2. Ultrasonic sensor update (non-blocking 50 ms trigger)
    // ---------------------------------------------------------
    obstacle_update();

    // ---------------------------------------------------------
    // 3. Balance IMU reading and complementary filter update
    // ---------------------------------------------------------
    balance_update();

    float angle = balance_get_angle();
    float dist  = obstacle_get_distance();

    // ---------------------------------------------------------
    // 4. Obstacle alert and threat classification
    // ---------------------------------------------------------
    ObstacleLevel obsLevel = OBS_CLEAR;
    if (g_obstacleEnabled) {
        obsLevel = obstacle_get_level();
        buzzer_obstacle_set_distance(dist);
    } else {
        buzzer_obstacle_set_distance(-1.0f);
    }

    // ---------------------------------------------------------
    // 5. Motor control by active mode
    // ---------------------------------------------------------
    int motorL = 0, motorR = 0;

    if (g_mode == MODE_BALANCE_SEGWAY) {
        // Segway balance mode
        bool fallen = (fabsf(angle) > BALANCE_FALLEN_ANGLE);

        if (fallen) {
            motors_stop();
            motorL = 0;
            motorR = 0;
        } else {
            float forwardBias = (float)g_webCmd.joyY * SEGWAY_MAX_TILT / 100.0f;
            int baseSpeed = balance_compute(forwardBias);
            int turnOffset = g_webCmd.joyX * 40 / 100;
            motorL = baseSpeed + turnOffset;
            motorR = baseSpeed - turnOffset;
            motors_set(motorL, motorR);
        }

    } else {
        // Regular drive mode
        float speedScale = (float)g_webCmd.speed / 100.0f;

        if (g_obstacleEnabled && obsLevel == OBS_STOP) {
            // Evasion sequence: Stop -> Reverse 300 ms -> Turn 500 ms
            switch (s_avoidState) {
                case AVOID_IDLE:
                    motors_stop();
                    motorL = 0;
                    motorR = 0;
                    s_avoidState = AVOID_BACK;
                    s_avoidTimer = now;
                    break;

                case AVOID_BACK:
                    if ((now - s_avoidTimer) < 300) {
                        int rev = -(int)(MOTOR_MAX_SPEED * 0.5f * speedScale);
                        motorL = rev;
                        motorR = rev;
                        motors_set(motorL, motorR);
                    } else {
                        s_avoidState = AVOID_TURN;
                        s_avoidTimer = now;
                    }
                    break;

                case AVOID_TURN:
                    if ((now - s_avoidTimer) < 500) {
                        int tq = (int)(MOTOR_MAX_SPEED * 0.5f * speedScale);
                        motorL = tq;
                        motorR = -tq;
                        motors_set(motorL, motorR);
                    } else {
                        s_avoidState = AVOID_IDLE;
                        buzzer_obstacle_avoided();
                        display_set_expression(EYE_HEART);
                    }
                    break;
            }

        } else {
            s_avoidState = AVOID_IDLE;

            int fwd  = (int)((float)g_webCmd.joyY * MOTOR_MAX_SPEED * speedScale / 100.0f);
            int turn = (int)((float)g_webCmd.joyX * 60.0f            * speedScale / 100.0f);

            if (g_obstacleEnabled && obsLevel == OBS_SLOW) {
                fwd  /= 2;
                turn /= 2;
            }

            motorL = fwd + turn;
            motorR = fwd - turn;
            motors_set(motorL, motorR);
        }
    }

    // ---------------------------------------------------------
    // Debug telemetry printout to Serial (every 200 ms)
    // ---------------------------------------------------------
    static unsigned long s_lastDebugMs = 0;
    if (now - s_lastDebugMs >= 200) {
        s_lastDebugMs = now;
        Serial.print(g_mode == MODE_BALANCE_SEGWAY ? "[Segway] " : "[Drive] ");
        Serial.print("Angle: ");
        Serial.print(angle, 1);
        Serial.print(" deg | GyroRate: ");
        Serial.print(balance_get_gyro_rate(), 1);
        Serial.print(" deg/s | Motor: ");
        Serial.println(motorL);
    }

    // ---------------------------------------------------------
    // Idle timer check
    // ---------------------------------------------------------
    if (g_webCmd.joyX != 0 || g_webCmd.joyY != 0) {
        g_lastIdleMs = now;
    }
    unsigned long idleTime = now - g_lastIdleMs;

    // ---------------------------------------------------------
    // 6. Eye expression resolution (by priority)
    // ---------------------------------------------------------
    EyeExpression expr = EYE_NORMAL;

    if (!g_showModeInfo) {
        bool fallen = (fabsf(angle) > BALANCE_FALLEN_ANGLE);
        bool moving = (g_webCmd.joyX != 0 || g_webCmd.joyY != 0);

        if (g_mode == MODE_BALANCE_SEGWAY && fallen) {
            expr = EYE_DIZZY;
        } else if (g_mode == MODE_BALANCE_SEGWAY && fabsf(angle) > BALANCE_WARN_ANGLE) {
            expr = EYE_ANGRY;
        } else if (g_obstacleEnabled && obsLevel == OBS_STOP) {
            expr = EYE_SURPRISED;
        } else if (g_obstacleEnabled && obsLevel == OBS_WARN) {
            expr = EYE_SURPRISED;
        } else if (g_webCmd.joyX < -30) {
            expr = EYE_LOOK_LEFT;
        } else if (g_webCmd.joyX > 30) {
            expr = EYE_LOOK_RIGHT;
        } else if (idleTime > IDLE_ZZZ_MS) {
            expr = EYE_ZZZ;
        } else if (idleTime > IDLE_SLEEPY_MS) {
            expr = EYE_SLEEPY;
        } else if (g_mode == MODE_BALANCE_SEGWAY && fabsf(angle) < 5.0f && moving) {
            expr = EYE_HAPPY;
        } else if (g_mode == MODE_REGULAR_DRIVE && !moving) {
            expr = EYE_SMIRK;
        } else {
            expr = EYE_NORMAL;
        }

        display_set_expression(expr);
    }

    // ---------------------------------------------------------
    // 7. Update telemetry for GET /status endpoint
    // ---------------------------------------------------------
    webserver_set_telemetry(
        angle,
        dist,
        expressionName(expr),
        g_mode,
        g_obstacleEnabled,
        g_webCmd.buzzerEnabled,
        motorL,
        motorR
    );

    // ---------------------------------------------------------
    // 8. Process web server and DNS requests
    // ---------------------------------------------------------
    webserver_update();

    // ---------------------------------------------------------
    // 9. Update buzzer timing and sync web toggle state
    // ---------------------------------------------------------
    buzzer_update();

    if (g_webCmd.buzzerEnabled != buzzer_is_enabled()) {
        buzzer_set_enabled(g_webCmd.buzzerEnabled);
    }

    // ---------------------------------------------------------
    // 10. Update OLED display
    // ---------------------------------------------------------
    display_set_angle(balance_get_angle());
    display_set_distance(obstacle_get_distance());
    display_update();

    // Clear updated flag once consumed
    g_webCmd.updated = false;
}
