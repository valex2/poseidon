#include <Adafruit_NeoPixel.h>

// Settings
#define PIN            6          // Pin where NeoPixel strip is connected
#define NUMPIXELS      150        // Total number of pixels
#define DELAY_MS       10        // Delay between brightness steps (controls speed)

// Create NeoPixel strip object
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Breathe in (brightness up)
  for (int brightness = 40; brightness <= 200; brightness++) {
    setBlueBreathing(brightness);
    delay(DELAY_MS);
  }

  // Breathe out (brightness down)
  for (int brightness = 200; brightness >= 40; brightness--) {
    setBlueBreathing(brightness);
    delay(DELAY_MS);
  }
}

void setBlueBreathing(uint8_t brightness) {
  // Generate dynamic shades of blue
  uint8_t blue = brightness;
  uint8_t green = brightness / 4;   // Add a hint of green for cyan tones
  uint8_t red = brightness / 8;    // Very slight purple tint at higher brightness

  uint32_t color = strip.Color(red, green, blue);

  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}
