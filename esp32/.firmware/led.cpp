#include "led.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

// LED_COUNT defaults to 1 (ATOM Lite single SK6812). Override to 25 for the
// ATOM Matrix 5x5 grid; both share PIN_LED = GPIO27 and the GRB SK6812 driver.
#ifndef LED_COUNT
#define LED_COUNT 1
#endif

// 25 LEDs at brightness=25 white draws ~150 mA; cap matrix brightness lower so
// USB power and the M5Stack regulator stay happy.
#if LED_COUNT > 1
#define LED_BRIGHT_NORMAL 8
#define LED_BRIGHT_DIM    2
#define LED_BRIGHT_FULL   40
#else
#define LED_BRIGHT_NORMAL 25
#define LED_BRIGHT_DIM    5
#define LED_BRIGHT_FULL   255
#endif

static Adafruit_NeoPixel g_strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

static inline void fill(uint32_t c) {
    for (uint16_t i = 0; i < LED_COUNT; i++) g_strip.setPixelColor(i, c);
}

void led_init() {
    g_strip.begin();
    g_strip.setBrightness(LED_BRIGHT_NORMAL);
    g_strip.clear();
    g_strip.show();
}

void led_set(LedColor color) {
    uint32_t c;
    if (color == LED_SLEEP) {
        g_strip.setBrightness(LED_BRIGHT_DIM);
        fill(g_strip.Color(255, 255, 255));
        g_strip.show();
        g_strip.setBrightness(LED_BRIGHT_NORMAL);
        return;
    }
    if (color == LED_WHITE) {
        g_strip.setBrightness(LED_BRIGHT_FULL);
        fill(g_strip.Color(255, 255, 255));
        g_strip.show();
        g_strip.setBrightness(LED_BRIGHT_NORMAL);
        return;
    }
    switch (color) {
        case LED_BLUE:   c = g_strip.Color(  0,   0, 255); break;
        case LED_GREEN:  c = g_strip.Color(  0, 255,   0); break;
        case LED_YELLOW: c = g_strip.Color(255, 200,   0); break;
        case LED_RED:    c = g_strip.Color(255,   0,   0); break;
        default:         c = 0;                             break;
    }
    fill(c);
    g_strip.show();
}

void led_factory_blink() {
    for (int i = 0; i < 3; i++) {
        led_set(LED_WHITE);
        delay(300);
        led_set(LED_OFF);
        delay(300);
    }
    led_set(LED_WHITE);  // stay lit to signal armed
}
