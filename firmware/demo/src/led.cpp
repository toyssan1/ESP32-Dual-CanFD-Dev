#include "led.h"
#include <Adafruit_NeoPixel.h>

static const char* TAG = "led";

// Single status LED strip instance.
static Adafruit_NeoPixel statusStrip(NUM_STATUS_LEDS, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

static LedState currentState = LED_OFF;
static LedPattern currentPattern = PATTERN_RAINBOW_CYCLE;
static unsigned long lastUpdate = 0;
static unsigned long patternChangeTime = 0;
static const unsigned long RANDOM_PATTERN_DURATION = 3000;

static float speedMultiplier = 1.0f;
static const float MIN_SPEED = 0.2f;
static const float MAX_SPEED = 5.0f;
static const float SPEED_STEP = 0.2f;

static uint16_t rainbowHue = 0;
static int breathingBrightness = 0;
static bool breathingDirection = true;
static bool blinkState = false;
static bool manualMode = false;

static uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static void set_led(uint32_t color) {
    statusStrip.setPixelColor(0, color);
    statusStrip.show();
}

void led_init() {
    Serial.printf("[%s] Initializing %d LED on pin %d\n", TAG, NUM_STATUS_LEDS, STATUS_LED_PIN);

    statusStrip.begin();
    statusStrip.setBrightness(128);
    statusStrip.clear();
    statusStrip.show();

    randomSeed(analogRead(0));

    // Boot flash to confirm LED is wired, then enter demo mode by default.
    set_led(rgb(255, 0, 0));
    delay(120);
    set_led(rgb(0, 0, 0));

    currentState = LED_RANDOM;
    patternChangeTime = millis();
    led_random_pattern();

    Serial.printf("[%s] Demo mode enabled at startup\n", TAG);
}

void led_set_state(LedState state) {
    currentState = state;
    manualMode = false;

    if (state == LED_OFF) {
        set_led(rgb(0, 0, 0));
        Serial.printf("[%s] LED OFF\n", TAG);
        return;
    }

    patternChangeTime = millis();
    if (state == LED_RANDOM) {
        led_random_pattern();
        Serial.printf("[%s] LED RANDOM demo mode\n", TAG);
    } else {
        Serial.printf("[%s] LED ON, pattern=%d\n", TAG, (int)currentPattern);
    }
}

void led_set_pattern(LedPattern pattern) {
    if (pattern >= PATTERN_COUNT) {
        return;
    }

    currentPattern = pattern;
    rainbowHue = 0;
    breathingBrightness = 0;
    breathingDirection = true;
    blinkState = false;
    lastUpdate = 0;
}

void led_random_pattern() {
    led_set_pattern((LedPattern)random(0, PATTERN_COUNT));
}

LedState led_get_state() {
    return currentState;
}

LedPattern led_get_current_pattern() {
    return currentPattern;
}

void led_speed_faster() {
    speedMultiplier += SPEED_STEP;
    if (speedMultiplier > MAX_SPEED) {
        speedMultiplier = MAX_SPEED;
    }
}

void led_speed_slower() {
    speedMultiplier -= SPEED_STEP;
    if (speedMultiplier < MIN_SPEED) {
        speedMultiplier = MIN_SPEED;
    }
}

float led_get_speed_multiplier() {
    return speedMultiplier;
}

void led_set_manual_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    manualMode = true;
    currentState = LED_ON;
    statusStrip.setBrightness(brightness);
    set_led(rgb(r, g, b));
}

static unsigned long pattern_interval_ms(LedPattern pattern) {
    switch (pattern) {
        case PATTERN_RED_CHASE:     return 120;
        case PATTERN_RAINBOW_CYCLE: return 25;
        case PATTERN_COLOR_WIPE:    return 220;
        case PATTERN_THEATER_CHASE: return 120;
        case PATTERN_FIRE:          return 60;
        case PATTERN_TWINKLE:       return 90;
        case PATTERN_BREATHING:     return 20;
        case PATTERN_SCANNER:       return 120;
        case PATTERN_COMET:         return 80;
        case PATTERN_WAVE:          return 30;
        default:                    return 120;
    }
}

static void render_pattern_once(LedPattern pattern) {
    switch (pattern) {
        case PATTERN_RED_CHASE:
            blinkState = !blinkState;
            set_led(blinkState ? rgb(255, 0, 0) : rgb(24, 0, 0));
            break;

        case PATTERN_RAINBOW_CYCLE:
            set_led(statusStrip.ColorHSV(rainbowHue, 255, 255));
            rainbowHue += 768;
            break;

        case PATTERN_COLOR_WIPE:
            blinkState = !blinkState;
            set_led(blinkState ? rgb(0, 255, 0) : rgb(0, 0, 255));
            break;

        case PATTERN_THEATER_CHASE:
            blinkState = !blinkState;
            set_led(blinkState ? rgb(0, 0, 255) : rgb(0, 0, 0));
            break;

        case PATTERN_FIRE:
            set_led(rgb((uint8_t)random(160, 256), (uint8_t)random(0, 120), 0));
            break;

        case PATTERN_TWINKLE:
            set_led((random(0, 100) < 40) ? rgb(255, 255, 255) : rgb(0, 0, 0));
            break;

        case PATTERN_BREATHING:
            if (breathingDirection) {
                breathingBrightness += 5;
                if (breathingBrightness >= 255) {
                    breathingBrightness = 255;
                    breathingDirection = false;
                }
            } else {
                breathingBrightness -= 5;
                if (breathingBrightness <= 0) {
                    breathingBrightness = 0;
                    breathingDirection = true;
                }
            }
            set_led(rgb(0, 0, (uint8_t)breathingBrightness));
            break;

        case PATTERN_SCANNER:
            blinkState = !blinkState;
            set_led(blinkState ? rgb(255, 0, 0) : rgb(0, 0, 0));
            break;

        case PATTERN_COMET:
            blinkState = !blinkState;
            set_led(blinkState ? rgb(255, 255, 255) : rgb(24, 24, 24));
            break;

        case PATTERN_WAVE:
            set_led(statusStrip.ColorHSV(rainbowHue, 255, 160));
            rainbowHue += 384;
            break;

        default:
            set_led(rgb(255, 0, 0));
            break;
    }
}

void led_update() {
    if (currentState == LED_OFF) {
        return;
    }

    if (manualMode) {
        return;
    }

    const unsigned long now = millis();

    if (currentState == LED_RANDOM && (now - patternChangeTime) >= RANDOM_PATTERN_DURATION) {
        patternChangeTime = now;
        led_random_pattern();
    }

    const unsigned long baseInterval = pattern_interval_ms(currentPattern);
    const unsigned long interval = (unsigned long)(baseInterval / speedMultiplier);
    if ((now - lastUpdate) < interval) {
        return;
    }

    lastUpdate = now;
    render_pattern_once(currentPattern);
}
