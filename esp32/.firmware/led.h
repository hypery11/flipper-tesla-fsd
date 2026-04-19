#pragma once

// LED colour codes for the single SK6812/WS2812B NeoPixel on the ATOM Lite.
typedef enum {
    LED_OFF    = 0,
    LED_BLUE,     // Listen-Only mode (passive, no TX)
    LED_GREEN,    // Active mode (FSD transmitting)
    LED_YELLOW,   // OTA detected (TX suspended)
    LED_RED,      // Error (no CAN traffic / wiring fault)
    LED_PURPLE,   // Car asleep (no CAN traffic, TX suspended)
} LedColor;

void led_init();
void led_set(LedColor color);
