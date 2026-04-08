#ifndef LED_H
#define LED_H

#include <Arduino.h>


// Status indicator LEDs (1 LEDs on GPIO21)
#define STATUS_LED_PIN 21
#define NUM_STATUS_LEDS 1

// LED states
enum LedState {
    LED_OFF,
    LED_ON,
    LED_RANDOM
};

// LED patterns
enum LedPattern {
    PATTERN_RED_CHASE,
    PATTERN_RAINBOW_CYCLE,
    PATTERN_COLOR_WIPE,
    PATTERN_THEATER_CHASE,
    PATTERN_FIRE,
    PATTERN_TWINKLE,
    PATTERN_BREATHING,
    PATTERN_SCANNER,
    PATTERN_COMET,
    PATTERN_WAVE,
    PATTERN_COUNT  // Keep this last
};

// Function declarations
void led_init();
void led_update();
void led_set_state(LedState state);
void led_set_pattern(LedPattern pattern);
void led_random_pattern();
void led_speed_faster();
void led_speed_slower();
LedState led_get_state();
LedPattern led_get_current_pattern();
float led_get_speed_multiplier();
void led_set_manual_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

#endif // LED_H
