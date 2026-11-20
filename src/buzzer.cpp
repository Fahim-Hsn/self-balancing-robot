/*
 * =====================================================
 * File: buzzer.cpp
 * Purpose: Non-blocking buzzer driver for RoboBot
 *          Melodies run on Core 0 FreeRTOS task.
 *          Obstacle beep is millis()-based in buzzer_update().
 * =====================================================
 * Compatible with both ESP32 Arduino Core 2.x and 3.x.
 * PIN_BUZZER (19) -> PWM Audio Output
 * =====================================================
 */

#include "buzzer.h"
#include <esp_arduino_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define USE_LEDC_V3 1
#else
#define USE_LEDC_V3 0
#endif

// ------------------------------------------------
// INTERNAL CONSTANTS
// ------------------------------------------------
#define BUZZER_TASK_STACK   2048    // Stack size for sound task
#define BUZZER_QUEUE_LEN    8       // Max queued commands
#define BUZZER_LEDC_RES     8       // 8-bit PWM resolution (0-255)
#define BUZZER_DUTY_ON      128     // 50% duty = square wave tone
#define BUZZER_NOTE_GAP_MS  30      // Gap between notes in a melody

// ------------------------------------------------
// MODULE STATE
// ------------------------------------------------
static QueueHandle_t  s_queue      = nullptr; // FreeRTOS command queue
static bool           s_enabled    = true;    // Global mute flag
static float          s_obsDist    = -1.0f;   // Last obstacle distance (cm)
static unsigned long  s_nextBeepMs = 0;       // Next obstacle beep timestamp
static bool           s_beepState  = false;   // Current beep on/off state

// ------------------------------------------------
// HELPER: Play a single note (blocking inside task)
// ------------------------------------------------
// freq_hz  -> 0 = silence
// dur_ms   -> duration in milliseconds
// ------------------------------------------------
static void playNote(uint32_t freq_hz, uint32_t dur_ms)
{
    if (!s_enabled) {
        vTaskDelay(pdMS_TO_TICKS(dur_ms));
        return;
    }
    if (freq_hz == 0) {
#if USE_LEDC_V3
        ledcWriteTone(PIN_BUZZER, 0);
        ledcWrite(PIN_BUZZER, 0);
#else
        ledcWrite(PWM_CH_BUZZER, 0);
#endif
    } else {
#if USE_LEDC_V3
        ledcWriteTone(PIN_BUZZER, freq_hz);
#else
        ledcSetup(PWM_CH_BUZZER, freq_hz, BUZZER_LEDC_RES);
        ledcAttachPin(PIN_BUZZER, PWM_CH_BUZZER);
        ledcWrite(PWM_CH_BUZZER, BUZZER_DUTY_ON);
#endif
    }
    vTaskDelay(pdMS_TO_TICKS(dur_ms));

#if USE_LEDC_V3
    ledcWriteTone(PIN_BUZZER, 0);
    ledcWrite(PIN_BUZZER, 0);
#else
    ledcWrite(PWM_CH_BUZZER, 0);
#endif
}

// ------------------------------------------------
// MELODIES
// Called from within the FreeRTOS sound task
// ------------------------------------------------

// Startup melody: 4-note ascending arpeggio
static void playStartup()
{
    const uint32_t notes[] = {523, 659, 784, 1047};
    const uint32_t durs[]  = {100, 100, 100, 200};
    for (int i = 0; i < 4; i++) {
        playNote(notes[i], durs[i]);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_NOTE_GAP_MS));
    }
}

// Mode 0 (balance): ascending 2-note
static void playMode0()
{
    playNote(440, 80);
    vTaskDelay(pdMS_TO_TICKS(BUZZER_NOTE_GAP_MS));
    playNote(880, 80);
}

// Mode 1 (drive): descending 2-note
static void playMode1()
{
    playNote(880, 80);
    vTaskDelay(pdMS_TO_TICKS(BUZZER_NOTE_GAP_MS));
    playNote(440, 80);
}

// Balance lost alert: 3x low buzz
static void playBalanceLost()
{
    for (int i = 0; i < 3; i++) {
        playNote(220, 150);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// WiFi connected chime: 3-note ascending
static void playWifiOk()
{
    const uint32_t notes[] = {784, 988, 1175};
    for (int i = 0; i < 3; i++) {
        playNote(notes[i], 100);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_NOTE_GAP_MS));
    }
}

// Obstacle avoided victory: short-long ascending
static void playObstacleAvoided()
{
    playNote(1047, 50);
    vTaskDelay(pdMS_TO_TICKS(BUZZER_NOTE_GAP_MS));
    playNote(1319, 100);
}

// ------------------------------------------------
// FREERTOS SOUND TASK
// Runs on Core 0, never blocks main loop
// ------------------------------------------------
static void soundTask(void* /*pvParams*/)
{
    BuzzerCmd cmd;
    for (;;) {
        if (xQueueReceive(s_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd) {
                case BUZZ_STARTUP:          playStartup();         break;
                case BUZZ_MODE_0:           playMode0();           break;
                case BUZZ_MODE_1:           playMode1();           break;
                case BUZZ_BALANCE_LOST:     playBalanceLost();     break;
                case BUZZ_WIFI_OK:          playWifiOk();          break;
                case BUZZ_OBSTACLE_AVOIDED: playObstacleAvoided(); break;
                default: break;
            }
#if USE_LEDC_V3
            ledcWriteTone(PIN_BUZZER, 0);
            ledcWrite(PIN_BUZZER, 0);
#else
            ledcWrite(PWM_CH_BUZZER, 0);
#endif
        }
    }
}

// =====================================================
// PUBLIC API IMPLEMENTATIONS
// =====================================================

// ------------------------------------------------
// buzzer_init - Hardware and task initialisation
// ------------------------------------------------
void buzzer_init()
{
#if USE_LEDC_V3
    ledcAttach(PIN_BUZZER, 2000, BUZZER_LEDC_RES);
    ledcWriteTone(PIN_BUZZER, 0);
#else
    ledcSetup(PWM_CH_BUZZER, 2000, BUZZER_LEDC_RES);
    ledcAttachPin(PIN_BUZZER, PWM_CH_BUZZER);
    ledcWrite(PWM_CH_BUZZER, 0);
#endif

    s_queue = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(BuzzerCmd));

    xTaskCreatePinnedToCore(
        soundTask,
        "buzzer_task",
        BUZZER_TASK_STACK,
        nullptr,
        1,
        nullptr,
        0
    );

    s_enabled    = true;
    s_obsDist    = -1.0f;
    s_nextBeepMs = 0;
    s_beepState  = false;
}

static void enqueue(BuzzerCmd cmd)
{
    if (!s_enabled || s_queue == nullptr) return;
    xQueueSend(s_queue, &cmd, 0);
}

void buzzer_startup_melody()   { enqueue(BUZZ_STARTUP);          }
void buzzer_balance_lost()     { enqueue(BUZZ_BALANCE_LOST);     }
void buzzer_wifi_connected()   { enqueue(BUZZ_WIFI_OK);          }
void buzzer_obstacle_avoided() { enqueue(BUZZ_OBSTACLE_AVOIDED); }

void buzzer_mode_changed(RobotMode newMode)
{
    if (newMode == MODE_BALANCE_SEGWAY) {
        enqueue(BUZZ_MODE_0);
    } else {
        enqueue(BUZZ_MODE_1);
    }
}

void buzzer_set_enabled(bool en)
{
    s_enabled = en;
    if (!en) {
#if USE_LEDC_V3
        ledcWriteTone(PIN_BUZZER, 0);
        ledcWrite(PIN_BUZZER, 0);
#else
        ledcWrite(PWM_CH_BUZZER, 0);
#endif
    }
}

bool buzzer_is_enabled() { return s_enabled; }

void buzzer_obstacle_set_distance(float distCm)
{
    s_obsDist = distCm;
}

void buzzer_update()
{
    if (!s_enabled) return;

    uint32_t interval = 0;

    if (s_obsDist > 0.0f) {
        if (s_obsDist < (float)OBSTACLE_STOP_CM) {
            interval = 100;
        } else if (s_obsDist < (float)OBSTACLE_SLOW_CM) {
            interval = 300;
        } else if (s_obsDist < (float)OBSTACLE_WARN_CM) {
            interval = 600;
        }
    }

    if (interval == 0) {
        if (s_beepState) {
#if USE_LEDC_V3
            ledcWriteTone(PIN_BUZZER, 0);
            ledcWrite(PIN_BUZZER, 0);
#else
            ledcWrite(PWM_CH_BUZZER, 0);
#endif
            s_beepState = false;
        }
        s_nextBeepMs = 0;
        return;
    }

    unsigned long now = millis();
    if (now >= s_nextBeepMs) {
        s_beepState = !s_beepState;
        if (s_beepState) {
#if USE_LEDC_V3
            ledcWriteTone(PIN_BUZZER, 1000);
#else
            ledcSetup(PWM_CH_BUZZER, 1000, BUZZER_LEDC_RES);
            ledcAttachPin(PIN_BUZZER, PWM_CH_BUZZER);
            ledcWrite(PWM_CH_BUZZER, BUZZER_DUTY_ON);
#endif
        } else {
#if USE_LEDC_V3
            ledcWriteTone(PIN_BUZZER, 0);
            ledcWrite(PIN_BUZZER, 0);
#else
            ledcWrite(PWM_CH_BUZZER, 0);
#endif
        }
        s_nextBeepMs = now + interval;
    }
}
