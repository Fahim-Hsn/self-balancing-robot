/*
 * =====================================================
 * File   : src/display.cpp
 * Project: RoboBot - ESP32 Self-Balance Car
 * Purpose: OLED Display Implementation
 * =====================================================
 * Hardware:
 *   - 1.3" SH1106 OLED (or SSD1306), 128x64, I2C
 *   - U8g2 library - full buffer mode (smooth animation)
 *
 * Eye Canvas:
 *   Left  eye box : x=10, y=12, w=42, h=40  (center 31,32)
 *   Right eye box : x=76, y=12, w=42, h=40  (center 97,32)
 *   Inner border  : 2 px inset
 *   Normal pupil  : filled disc, r=10
 * =====================================================
 */

#include "display.h"
#include <Wire.h>

// =====================================================
// U8g2 INSTANCE
// =====================================================
// Most 1.3" OLEDs use SH1106. Switch comment if yours is SSD1306.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// =====================================================
// EYE GEOMETRY CONSTANTS
// =====================================================
// Left eye box
static const int L_BOX_X = 10, L_BOX_Y = 12, L_BOX_W = 42, L_BOX_H = 40;
static const int L_CX    = 31, L_CY    = 32; // pupil center

// Right eye box
static const int R_BOX_X = 76, R_BOX_Y = 12, R_BOX_W = 42, R_BOX_H = 40;
static const int R_CX    = 97, R_CY    = 32; // pupil center

static const int PUPIL_R  = 10; // normal pupil radius
static const int BORDER   =  2; // inner highlight inset

// =====================================================
// MODULE STATE
// =====================================================
static EyeExpression _currentExpression = EYE_NORMAL;
static float         _distanceCm        = -1.0f;
static float         _angleDeg          =  0.0f;

// Mode-info overlay
static bool          _showingModeInfo   = false;
static unsigned long _modeInfoStartMs   = 0;
static RobotMode     _modeInfoMode      = MODE_BALANCE_SEGWAY;
static bool          _modeInfoObstacle  = true;
static bool          _modeInfoBuzzer    = true;

// Frame-rate throttle
static unsigned long _lastDrawMs        = 0;
static const uint16_t FRAME_MS         = 50; // 20 fps

// Blink timer (EYE_NORMAL)
static unsigned long _blinkNextMs       = 0;
static bool          _blinkActive       = false;
static unsigned long _blinkEndMs        = 0;

// ZZZ animation
static uint8_t       _zzzFrame          = 0;
static unsigned long _zzzNextMs         = 0;

// DIZZY animation
static uint8_t       _dizzyFrame        = 0;
static unsigned long _dizzyNextMs       = 0;

// SPINNING animation
static uint8_t       _spinFrame         = 0;
static unsigned long _spinNextMs        = 0;

// SWEAT drop animation
static uint8_t       _sweatFrame        = 0;
static unsigned long _sweatNextMs       = 0;

// =====================================================
// INTERNAL HELPERS
// =====================================================

/**
 * Draw a standard eye box with inner border highlight.
 */
static void drawEyeBox(int bx, int by, int bw, int bh) {
    u8g2.drawFrame(bx, by, bw, bh);
    u8g2.drawFrame(bx + BORDER, by + BORDER, bw - 2*BORDER, bh - 2*BORDER);
}

/**
 * Draw both standard eye boxes.
 */
static void drawBothBoxes() {
    drawEyeBox(L_BOX_X, L_BOX_Y, L_BOX_W, L_BOX_H);
    drawEyeBox(R_BOX_X, R_BOX_Y, R_BOX_W, R_BOX_H);
}

/**
 * Draw normal circular pupils at given offsets.
 * @param dx Horizontal pupil offset
 * @param dy Vertical pupil offset
 * @param r  Pupil radius
 */
static void drawPupils(int dx, int dy, int r) {
    u8g2.drawDisc(L_CX + dx, L_CY + dy, r);
    u8g2.drawDisc(R_CX + dx, R_CY + dy, r);
}

// =====================================================
// EXPRESSION DRAW FUNCTIONS
// =====================================================

// ------ 1. EYE_NORMAL --------------------------------
// Standard box eye with centered pupils and blinking
static void drawNormal() {
    unsigned long now = millis();

    if (!_blinkActive && now >= _blinkNextMs) {
        _blinkActive = true;
        _blinkEndMs  = now + 80;                        // Squish for 80 ms
        _blinkNextMs = now + 3000 + random(1000);       // Next blink in 3-4 s
    }
    if (_blinkActive && now >= _blinkEndMs) {
        _blinkActive = false;
    }

    drawBothBoxes();

    if (_blinkActive) {
        // Squished pupil (height = 2 px)
        u8g2.drawBox(L_CX - PUPIL_R, L_CY - 1, PUPIL_R * 2, 2);
        u8g2.drawBox(R_CX - PUPIL_R, R_CY - 1, PUPIL_R * 2, 2);
    } else {
        drawPupils(0, 0, PUPIL_R);
    }
}

// ------ 2. EYE_HAPPY ---------------------------------
// Happy expression: upward curved smile lines
static void drawHappy() {
    drawBothBoxes();

    // Fill bottom 1/3 of each box with black
    int coverH = L_BOX_H / 3;
    int coverY_L = L_BOX_Y + L_BOX_H - coverH;
    int coverY_R = R_BOX_Y + R_BOX_H - coverH;
    u8g2.setDrawColor(0);
    u8g2.drawBox(L_BOX_X + 1, coverY_L, L_BOX_W - 2, coverH - 1);
    u8g2.drawBox(R_BOX_X + 1, coverY_R, R_BOX_W - 2, coverH - 1);
    u8g2.setDrawColor(1);

    // Left eye smile arc
    u8g2.drawLine(L_CX - 8, L_CY + 2, L_CX,     L_CY - 4);
    u8g2.drawLine(L_CX,     L_CY - 4, L_CX + 8, L_CY + 2);

    // Right eye smile arc
    u8g2.drawLine(R_CX - 8, R_CY + 2, R_CX,     R_CY - 4);
    u8g2.drawLine(R_CX,     R_CY - 4, R_CX + 8, R_CY + 2);
}

// ------ 3. EYE_SURPRISED ----------------------------
// Surprised: larger box, large pupil
static void drawSurprised() {
    int lbx = max(0,  L_BOX_X - 4);
    int lby = max(0,  L_BOX_Y - 4);
    int lbw = min(DISPLAY_WIDTH  - lbx, L_BOX_W + 8);
    int lbh = min(DISPLAY_HEIGHT - lby, L_BOX_H + 8);

    int rbx = max(0,  R_BOX_X - 4);
    int rby = max(0,  R_BOX_Y - 4);
    int rbw = min(DISPLAY_WIDTH  - rbx, R_BOX_W + 8);
    int rbh = min(DISPLAY_HEIGHT - rby, R_BOX_H + 8);

    u8g2.drawFrame(lbx, lby, lbw, lbh);
    u8g2.drawFrame(rbx, rby, rbw, rbh);

    u8g2.drawDisc(L_CX, L_CY, 14);
    u8g2.drawDisc(R_CX, R_CY, 14);
}

// ------ 4. EYE_ANGRY ---------------------------------
// Angry: slanted eyebrows, pupils lowered
static void drawAngry() {
    drawBothBoxes();
    drawPupils(0, 4, PUPIL_R);

    // Left brow: outer-high to inner-low
    u8g2.drawLine(L_BOX_X,              L_BOX_Y - 5,
                  L_BOX_X + L_BOX_W - 1, L_BOX_Y - 2);
    // Right brow: inner-low to outer-high
    u8g2.drawLine(R_BOX_X,              R_BOX_Y - 2,
                  R_BOX_X + R_BOX_W - 1, R_BOX_Y - 5);
}

// ------ 5. EYE_SLEEPY --------------------------------
// Sleepy: eyelids lowered over top half
static void drawSleepy() {
    drawBothBoxes();

    int lidH = (L_BOX_H * 55) / 100;
    u8g2.setDrawColor(0);
    u8g2.drawBox(L_BOX_X + 1, L_BOX_Y + 1, L_BOX_W - 2, lidH);
    u8g2.drawBox(R_BOX_X + 1, R_BOX_Y + 1, R_BOX_W - 2, lidH);
    u8g2.setDrawColor(1);

    int pupilY = L_BOX_Y + lidH + PUPIL_R / 2 + 2;
    u8g2.drawDisc(L_CX, pupilY, PUPIL_R / 2);
    u8g2.drawDisc(R_CX, pupilY, PUPIL_R / 2);
}

// ------ 6. EYE_LOOK_LEFT ----------------------------
static void drawLookLeft() {
    drawBothBoxes();
    drawPupils(-10, 0, PUPIL_R);
}

// ------ 7. EYE_LOOK_RIGHT ---------------------------
static void drawLookRight() {
    drawBothBoxes();
    drawPupils(10, 0, PUPIL_R);
}

// ------ 8. EYE_DIZZY ---------------------------------
// Dizzy: animated cross (X) eyes
static void drawDizzy() {
    unsigned long now = millis();
    if (now >= _dizzyNextMs) {
        _dizzyFrame  = (_dizzyFrame + 1) % 2;
        _dizzyNextMs = now + 150;
    }

    drawBothBoxes();

    int hw = 7;
    for (int cx : {L_CX, R_CX}) {
        if (_dizzyFrame == 0) {
            u8g2.drawLine(cx - hw, L_CY - hw, cx + hw, L_CY + hw);
            u8g2.drawLine(cx + hw, L_CY - hw, cx - hw, L_CY + hw);
        } else {
            u8g2.drawLine(cx - hw + 2, L_CY - hw, cx + hw + 2, L_CY + hw);
            u8g2.drawLine(cx + hw - 2, L_CY - hw, cx - hw - 2, L_CY + hw);
        }
    }
}

// ------ 9. EYE_COOL ----------------------------------
// Cool: sunglasses bar with center bridge
static void drawCool() {
    u8g2.drawRBox(8, 20, 112, 16, 3);

    u8g2.setDrawColor(0);
    u8g2.drawBox(60, 22, 8, 12);
    u8g2.setDrawColor(1);
    u8g2.drawLine(60, 28, 68, 28);

    u8g2.drawRFrame(8, 20, 112, 16, 3);
}

// ------ 10. EYE_SMIRK --------------------------------
static void drawSmirk() {
    drawEyeBox(L_BOX_X, L_BOX_Y, L_BOX_W, L_BOX_H);
    u8g2.drawDisc(L_CX, L_CY, PUPIL_R);

    drawEyeBox(R_BOX_X, R_BOX_Y, R_BOX_W, R_BOX_H);

    int rx = R_BOX_X, ry = R_BOX_Y, rw = R_BOX_W, rh = R_BOX_H;
    u8g2.drawLine(rx + 4,       ry + rh/2,       R_CX,       ry + rh - 6);
    u8g2.drawLine(R_CX,         ry + rh - 6,     rx + rw - 4, ry + rh/2 + 4);
    u8g2.drawDisc(R_CX, R_CY, PUPIL_R);
}

// ------ 11. EYE_ZZZ ----------------------------------
static void drawZZZ() {
    unsigned long now = millis();
    if (now >= _zzzNextMs) {
        _zzzFrame  = (_zzzFrame + 1) % 3;
        _zzzNextMs = now + 400;
    }

    drawBothBoxes();
    int lidH = (L_BOX_H * 40) / 100;
    u8g2.setDrawColor(0);
    u8g2.drawBox(L_BOX_X + 1, L_BOX_Y + 1, L_BOX_W - 2, lidH);
    u8g2.drawBox(R_BOX_X + 1, R_BOX_Y + 1, R_BOX_W - 2, lidH);
    u8g2.setDrawColor(1);
    int pY = L_BOX_Y + lidH + 4;
    u8g2.drawDisc(L_CX, pY, 6);
    u8g2.drawDisc(R_CX, pY, 6);

    u8g2.setFont(u8g2_font_6x10_tf);
    if (_zzzFrame >= 0) u8g2.drawStr(110, 20, "z");
    if (_zzzFrame >= 1) u8g2.drawStr(115, 14, "Z");
    if (_zzzFrame >= 2) u8g2.drawStr(120,  8, "Z");
}

// ------ 12. EYE_HEART --------------------------------
static void drawHeart() {
    drawBothBoxes();

    for (int cx : {L_CX, R_CX}) {
        int cy = L_CY;
        u8g2.drawDisc(cx - 4, cy - 3, 4);
        u8g2.drawDisc(cx + 4, cy - 3, 4);
        u8g2.drawTriangle(cx - 8, cy - 1,
                          cx + 8, cy - 1,
                          cx,     cy + 8);
    }
}

// ------ 13. EYE_SWEAT --------------------------------
static void drawSweat() {
    unsigned long now = millis();
    if (now >= _sweatNextMs) {
        _sweatFrame  = (_sweatFrame + 1) % 6;
        _sweatNextMs = now + 250;
    }

    drawBothBoxes();
    drawPupils(0, 0, PUPIL_R);

    int dropX = R_BOX_X + R_BOX_W + 4;
    int dropY = R_BOX_Y + 4 + _sweatFrame * 5;

    u8g2.drawDisc(dropX, dropY, 3);
    u8g2.drawTriangle(dropX - 3, dropY + 1,
                      dropX + 3, dropY + 1,
                      dropX,     dropY + 7);
}

// ------ 14. EYE_SPINNING -----------------------------
static void drawSpinning() {
    unsigned long now = millis();
    if (now >= _spinNextMs) {
        _spinFrame  = (_spinFrame + 1) % 4;
        _spinNextMs = now + 150;
    }

    drawBothBoxes();

    static const int8_t arms[4][4][2] = {
        {{ 0,-10},{ 0, 10},{-10, 0},{ 10, 0}},
        {{-7,-7},{ 7, 7},{ 7,-7},{-7, 7}},
        {{ 0,-10},{ 0, 10},{-10, 0},{ 10, 0}},
        {{-7,-7},{ 7, 7},{ 7,-7},{-7, 7}}
    };

    for (int cx : {L_CX, R_CX}) {
        for (int i = 0; i < 2; i++) {
            int f = (_spinFrame % 2 == 0) ? 0 : 1;
            u8g2.drawLine(cx + arms[f][i*2][0],   L_CY + arms[f][i*2][1],
                          cx + arms[f][i*2+1][0], L_CY + arms[f][i*2+1][1]);
        }
    }
}

// =====================================================
// MODE INFO OVERLAY
// =====================================================
static void drawModeInfo() {
    u8g2.setDrawColor(0);
    u8g2.drawBox(10, 8, 108, 48);
    u8g2.setDrawColor(1);
    u8g2.drawRFrame(10, 8, 108, 48, 5);

    u8g2.setFont(u8g2_font_6x10_tf);

    u8g2.setCursor(16, 22);
    if (_modeInfoMode == MODE_BALANCE_SEGWAY) {
        u8g2.print("MODE: BALANCE");
    } else {
        u8g2.print("MODE: DRIVE");
    }

    u8g2.setCursor(16, 36);
    u8g2.print("OBSTACLE: ");
    u8g2.print(_modeInfoObstacle ? "ON" : "OFF");

    u8g2.setCursor(16, 50);
    u8g2.print("BUZZER: ");
    u8g2.print(_modeInfoBuzzer ? "ON" : "OFF");
}

// =====================================================
// STARTUP ANIMATION
// =====================================================
void display_show_startup_animation() {
    // Frame 1: RoboBot text centered
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(24, 38, "RoboBot");
    u8g2.sendBuffer();
    delay(500);

    // Frame 2: Robot face icon
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(24, 20, "RoboBot");
    u8g2.drawCircle(44, 48, 12);
    u8g2.drawDisc(40, 45, 2);
    u8g2.drawDisc(48, 45, 2);
    u8g2.drawLine(39, 53, 49, 53);
    u8g2.sendBuffer();
    delay(300);

    // Frame 3: Eyes open from center outward
    for (int grow = 1; grow <= 20; grow += 2) {
        u8g2.clearBuffer();
        int lhalf = grow;
        u8g2.drawFrame(L_CX - lhalf, L_CY - lhalf / 2, lhalf * 2, lhalf);
        u8g2.drawFrame(R_CX - lhalf, R_CY - lhalf / 2, lhalf * 2, lhalf);
        u8g2.sendBuffer();
        delay(50);
    }

    // Frame 4: Cool sunglasses eye
    u8g2.clearBuffer();
    drawCool();
    u8g2.sendBuffer();
    delay(1000);

    _currentExpression = EYE_NORMAL;
    _blinkNextMs = millis() + 3000;
}

// =====================================================
// PUBLIC API IMPLEMENTATIONS
// =====================================================

void display_init() {
    u8g2.begin();
    u8g2.setContrast(200);
    u8g2.clearBuffer();
    u8g2.sendBuffer();

    _blinkNextMs = millis() + 3000;
}

void display_set_expression(EyeExpression expr) {
    _currentExpression = expr;
}

EyeExpression display_get_expression() {
    return _currentExpression;
}

void display_show_mode_info(RobotMode mode, bool obstacleEnabled, bool buzzerEnabled) {
    _showingModeInfo  = true;
    _modeInfoStartMs  = millis();
    _modeInfoMode     = mode;
    _modeInfoObstacle = obstacleEnabled;
    _modeInfoBuzzer   = buzzerEnabled;
}

void display_set_distance(float distCm) {
    _distanceCm = distCm;
}

void display_set_angle(float angleDeg) {
    _angleDeg = angleDeg;
}

void display_show_text(const char* line1, const char* line2) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (line1) {
        int w1 = u8g2.getStrWidth(line1);
        u8g2.drawStr((DISPLAY_WIDTH - w1) / 2, 28, line1);
    }
    if (line2) {
        int w2 = u8g2.getStrWidth(line2);
        u8g2.drawStr((DISPLAY_WIDTH - w2) / 2, 48, line2);
    }
    u8g2.sendBuffer();
}

// =====================================================
// MAIN UPDATE
// =====================================================
void display_update() {
    unsigned long now = millis();

    // Frame-rate throttle: max 20 fps
    if (now - _lastDrawMs < FRAME_MS) return;
    _lastDrawMs = now;

    if (_showingModeInfo && (now - _modeInfoStartMs >= DISPLAY_MODE_SHOW_MS)) {
        _showingModeInfo = false;
    }

    u8g2.clearBuffer();
    u8g2.setDrawColor(1);

    if (_showingModeInfo) {
        drawModeInfo();
    } else {
        switch (_currentExpression) {
            case EYE_NORMAL:     drawNormal();    break;
            case EYE_HAPPY:      drawHappy();     break;
            case EYE_SURPRISED:  drawSurprised(); break;
            case EYE_ANGRY:      drawAngry();     break;
            case EYE_SLEEPY:     drawSleepy();    break;
            case EYE_LOOK_LEFT:  drawLookLeft();  break;
            case EYE_LOOK_RIGHT: drawLookRight(); break;
            case EYE_DIZZY:      drawDizzy();     break;
            case EYE_COOL:       drawCool();      break;
            case EYE_SMIRK:      drawSmirk();     break;
            case EYE_ZZZ:        drawZZZ();       break;
            case EYE_HEART:      drawHeart();     break;
            case EYE_SWEAT:      drawSweat();     break;
            case EYE_SPINNING:   drawSpinning();  break;
            default:             drawNormal();    break;
        }
    }

    u8g2.sendBuffer();
}
