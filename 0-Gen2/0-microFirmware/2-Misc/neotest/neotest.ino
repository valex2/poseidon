#include <Adafruit_NeoPixel.h>
#include <math.h>

// === Configuration ===
#define NEOPIXEL_PIN 32
const int NUM_PIXELS = 128;     // Number of logical pixels
const int STRIP_LENGTH = 200;   // Number of physical pixels
const int MOTORS = 8;

Adafruit_NeoPixel strip(STRIP_LENGTH, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct MotorLightState {
  int pwm = 1500;
  uint8_t baseBrightness = 0;
  uint32_t lastUpdate = 0;
};

MotorLightState motorStates[MOTORS];

// === Initialize motors ===
void initializeMotors() {
  unsigned long t = millis();
  for (int i = 0; i < MOTORS; i++) {
    motorStates[i].pwm = 1500;
    motorStates[i].lastUpdate = t;
    motorStates[i].baseBrightness = 0;
  }
}

// === Gradient logic: Red→White→Green ===
uint32_t computeColorFromPWM(int pwm) {
  pwm = constrain(pwm, 1100, 1900);

  const uint8_t blueR = 0;
  const uint8_t blueG = 0;
  const uint8_t blueB = 20;  // Deep, dark ocean blue

  if (pwm < 1500) {
    float t = (pwm - 1100) / 400.0;
    // Red → Deep Blue
    uint8_t r = 180 * (1.0 - t);       // fade red
    uint8_t g = 12 * t;                // fade in muted green
    uint8_t b = 80 * t;                // fade in deep blue
    return strip.Color(r, g, b);
  } else if (pwm > 1500) {
    float t = (pwm - 1500) / 400.0;
    // Deep Blue → Green
    uint8_t r = 0;
    uint8_t g = 12 + (168 * t);        // fade in green from 12 → 180
    uint8_t b = 80 * (1.0 - t);        // fade out blue
    return strip.Color(r, g, b);
  } else {
    // Exactly 1500
    return strip.Color(blueR, blueG, blueB);
  }
}

// === Update motor PWM info ===
void updateMotorLights(int motorIndex, int pwm) {
  motorStates[motorIndex].pwm = pwm;
  motorStates[motorIndex].lastUpdate = millis();

  int speed = abs(pwm - 1500);
  uint8_t brightness = map(speed, 0, 400, 0, 192);  // cap brightness a bit
  motorStates[motorIndex].baseBrightness = brightness;
}

// === Update NeoPixel strip ===
void updateStrips() {
  unsigned long t = millis();
  int segSize = NUM_PIXELS / MOTORS;

  // Full clear
  for (int i = 0; i < STRIP_LENGTH; i++) {
    strip.setPixelColor(i, 0);
  }

  for (int i = 0; i < MOTORS; i++) {
    unsigned long dt = (t >= motorStates[i].lastUpdate) ? t - motorStates[i].lastUpdate : 0;
    float fade = exp(-min(dt, 5000UL) / 1000.0);
    float pulse = 0.5 * (1 + sin(t / 200.0 + i));

    uint8_t dynamicBrightness = motorStates[i].baseBrightness * pulse * fade;
    dynamicBrightness = max(dynamicBrightness, 24); // never fully off

    uint32_t color;

    // Soft idle breathing if near neutral and idle for >5s
    if (abs(motorStates[i].pwm - 1500) < 10 && dt > 5000) {
      float breath = 0.5 * (1 + sin(t / 1000.0 + i));
      uint8_t breathLevel = 32 + 32 * breath;  // 32–64 range
      color = strip.Color(0, 0, breathLevel);  // breathing blue
    } else {
      color = computeColorFromPWM(motorStates[i].pwm);
    }

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    int segStart = i * segSize;
    int segEnd = segStart + segSize;

    for (int j = segStart; j < segEnd && j < NUM_PIXELS; j++) {
      uint8_t scaledR = (r * dynamicBrightness) / 255;
      uint8_t scaledG = (g * dynamicBrightness) / 255;
      uint8_t scaledB = (b * dynamicBrightness) / 255;

      if (scaledR + scaledG + scaledB == 0) {
        // Ensure at least something lights up, apply floor to overall intensity
        scaledB = 16;  // dim blue by default
      }

      strip.setPixelColor(j, strip.Color(scaledR, scaledG, scaledB));
    }

    // Optional serial debug
    Serial.printf("Motor %d PWM: %d Brightness: %d Fade: %.2f\n", i, motorStates[i].pwm, dynamicBrightness, fade);
  }

  strip.show();
}

// === Simulate random PWM changes ===
void randomMotorUpdates() {
  for (int i = 0; i < MOTORS; i++) {
    int pwm = random(1400, 1600);
    // int pwm = 1500;
    updateMotorLights(i, pwm);
  }
  updateStrips();
}

// === Setup ===
void setup() {
  Serial.begin(9600);
  delay(200);
  strip.begin();
  strip.show();
  initializeMotors();

  // Full clear at boot
  for (int i = 0; i < STRIP_LENGTH; i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
  Serial.println("Setup complete");
}

// === Main loop ===
void loop() {
  randomMotorUpdates();
  delay(200);  // ~20 FPS
}
