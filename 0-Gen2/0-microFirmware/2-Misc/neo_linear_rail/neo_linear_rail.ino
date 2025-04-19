#include <Adafruit_NeoPixel.h>

// Settings
#define PIN            6          // Pin where NeoPixel strip is connected
#define NUMPIXELS      100        // Total number of pixels (max 100)
#define DELAY_MS       80        // Delay between color changes (in ms)

// Create NeoPixel strip object
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Define colors to cycle through
uint32_t colors[] = {
  strip.Color(255, 0, 0),   // Red
  strip.Color(0, 255, 0),   // Green
  strip.Color(0, 0, 255),   // Blue
  strip.Color(255, 255, 255), // White
  strip.Color(0, 0, 0)      // Off
};
const uint8_t numColors = sizeof(colors) / sizeof(colors[0]);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  for (int i = 0; i < NUMPIXELS; i++) {
    for (int c = 0; c < numColors; c++) {
      strip.clear();                 // Turn off all pixels
      strip.setPixelColor(i, colors[c]); // Set current pixel to color
      strip.show();                 // Update strip
      delay(DELAY_MS);
    }
  }
}
