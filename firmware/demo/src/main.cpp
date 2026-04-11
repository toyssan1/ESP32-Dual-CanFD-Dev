#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <ACAN2517FD.h>
#include <ESP32-TWAI-CAN.hpp>
#include <Arduino_BMI270_BMM150.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include "led.h"

// -------------------- CAN (TWAI + SPI MCP2517FD) --------------------

static const uint8_t PIN_TWAI_TX = 18;
static const uint8_t PIN_TWAI_RX = 17;

// -------------------- NEO-6M GPS (UART1) --------------------

static const uint8_t  PIN_GPS_TX  = 43;   // TXD0 (ESP TX → GPS RX)
static const uint8_t  PIN_GPS_RX  = 44;   // RXD0 (GPS TX → ESP RX)
static const uint32_t GPS_BAUD    = 9600;
static const uint32_t GPS_PRINT_INTERVAL_MS = 1000;

static const uint8_t PIN_SPI_MISO = 10;
static const uint8_t PIN_SPI_MOSI = 11;
static const uint8_t PIN_SPI_SCK = 14;
static const uint8_t PIN_CAN_INT = 12;
static const uint8_t PIN_CAN_CS = 13;
static const int8_t PIN_SD_CS = 47;


static const uint8_t LED_PULSE_MS = 50;
static const uint8_t PIN_LED_TWAI_TX = 38;
static const uint8_t PIN_LED_SPI_TX = 39;

// GPIO controls for demo TX enable.
// Using INPUT_PULLUP: LOW = enabled, HIGH = disabled.
static const uint8_t PIN_DEMO_TWAI_ENABLE = 4;
static const uint8_t PIN_DEMO_SPI_ENABLE = 5;
static const uint8_t PIN_BMI_ENABLE = 6;
static const uint8_t PIN_MMC_ENABLE = 15;
static const uint8_t PIN_CAL_MODE = 16;
static const uint8_t PIN_SCREEN_CYCLE = 40;

static const uint32_t CAN_SPI_BITRATE = 500000;
static const uint32_t CAN_TWAI_BITRATE = 500;
static const uint8_t SPI_CAN_BOOT_INIT_RETRIES = 5;
static const uint32_t SPI_CAN_RETRY_DELAY_MS = 500;
static const uint32_t SPI_CAN_RETRY_INTERVAL_MS = 3000;
static const uint32_t SPI_CAN_RX_STALL_MS = 750;

static const uint16_t TWAI_DEMO_ID = 0x321;
static const uint16_t SPI_DEMO_ID = 0x322;
static const uint32_t DEMO_TX_INTERVAL_MS = 500;
static const uint32_t LED_STARTUP_DEMO_MS = 5000;

static uint32_t twaiTxLedOffAt = 0;
static uint32_t twaiRxLedOffAt = 0;
static uint32_t spiTxLedOffAt = 0;
static uint32_t spiRxLedOffAt = 0;

static bool spiCanOnline = false;
static bool spiCanStarted = false;
static uint32_t nextSpiCanRetryMs = 0;
static uint32_t lastSpiRxMs = 0;
static uint32_t spiIntLowSinceMs = 0;

static uint32_t nextTwaiDemoTxMs = 0;
static uint32_t nextSpiDemoTxMs = 0;
static uint8_t twaiDemoCounter = 0;
static uint8_t spiDemoCounter = 0;
static uint32_t spiDemoAttempts = 0;
static uint32_t spiDemoQueued = 0;
static uint32_t spiDemoFailed = 0;
static uint32_t spiDemoSkippedOffline = 0;

static const uint32_t BMI270_POLL_INTERVAL_MS = 50;
static const uint32_t MMC5983MA_POLL_INTERVAL_MS = 100;
static const uint32_t MMC5983MA_LOG_INTERVAL_MS  = 500;  // 2 Hz serial/SD logging
static const uint8_t  MMC5983MA_I2C_ADDR = 0x30;
static const uint8_t  MMC5983MA_PRODUCT_ID = 0x30;
static const uint8_t  MMC5983MA_REG_PRODUCT_ID = 0x2F;
static const uint8_t  OLED_I2C_ADDR  = 0x3C;
static const uint8_t  OLED_WIDTH     = 128;
static const uint8_t  OLED_HEIGHT    = 64;
static const float BMI270_ACCEL_CHANGE_THRESHOLD_G = 0.03f;
static const float BMI270_TILT_THRESHOLD_G = 0.25f;
static const float BMI270_ACCEL_DELTA_THRESHOLD_G = 0.015f;
static const float MOTION_FILTER_ALPHA = 0.15f;       // lower = smoother input, less jitter
static const float LED_COLOR_FILTER_ALPHA = 0.40f;    // higher = snappier color transitions
static const float LED_BRIGHTNESS_FILTER_ALPHA = 0.30f; // higher = snappier brightness transitions

static bool bmiOnline = false;
static bool bmiHasBaseline = false;
static bool bmiGateWasEnabled = false;
static uint32_t nextBmiReinitMs = 0;
static bool ledStartupDemoActive = true;
static bool ledCurrentlyOff = false;
static uint32_t nextBmiPollMs = 0;
static uint32_t ledStartupDemoEndMs = 0;
static uint32_t directionEventUntilMs = 0;
static float lastAx = 0.0f;
static float lastAy = 0.0f;
static float lastAz = 0.0f;
static float gravAx = 0.0f;   // gravity baseline captured at first reading
static float gravAy = 0.0f;
static float gravAz = 0.0f;
static float lastAccelMagnitude = 1.0f;
static float lastDirectionAxis = 0.0f;
static bool motionFilterReady = false;
static float filtAx = 0.0f;
static float filtAy = 0.0f;
static float filtAz = 0.0f;
static float ledBrightnessSmoothed = 64.0f;
static float ledRSmoothed = 0.0f;
static float ledGSmoothed = 0.0f;
static float ledBSmoothed = 0.0f;

static SFE_MMC5983MA magMMC;
static bool mmcOnline = false;
static bool mmcGateWasEnabled = false;
static uint32_t nextMmcPollMs = 0;
static uint32_t nextMmcLogMs  = 0;
static uint32_t nextMmcReinitMs = 0;
static const uint32_t MMC5983MA_REINIT_INTERVAL_MS = 2000;
static const uint32_t BMI270_REINIT_INTERVAL_MS    = 2000;
// MotionCal calibration stream: 100 Hz, mag in µT * 10, accel in m/s² * 100
// MMC5983MA: 18-bit, ±8 Gauss = ±800 µT, centred at 131072
static const float   MMC_UT_PER_COUNT = 800.0f / 131072.0f;
static const uint32_t CAL_TX_INTERVAL_MS = 10;
static bool     calModeActive = false;
static uint32_t nextCalTxMs  = 0;
static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool oledOnline = false;

enum OledScreen : uint8_t { SCREEN_COMPASS = 0, SCREEN_GPS = 1, SCREEN_ACCEL = 2, SCREEN_COUNT = 3 };
static OledScreen currentScreen = SCREEN_COMPASS;
static float    lastOledHeading = 0.0f;
static bool     lastOledHeadingValid = false;
static bool     screenBtnLastState = true;
static uint32_t screenBtnDebounceMs = 0;
static uint32_t nextOledRefreshMs = 0;
static const uint32_t SCREEN_BTN_DEBOUNCE_MS  = 250;
static const uint32_t OLED_REFRESH_INTERVAL_MS = 200;

static bool sdOnline = false;
static File sdLogFile;
static uint32_t nextSdFlushMs = 0;
static const uint32_t SD_FLUSH_INTERVAL_MS = 1000;

static TinyGPSPlus gps;
static uint32_t nextGpsPrintMs = 0;
static bool     gpsRawDiagActive = false;
static uint32_t gpsRawDiagEndMs = 0;
static char     gpsNmeaLineBuf[128];
static uint8_t  gpsNmeaLineBufPos = 0;

SPIClass spiBus(FSPI);
ACAN2517FD canSPI(PIN_CAN_CS, spiBus, PIN_CAN_INT);
static void printSPIPinMap() {
  Serial.printf("SPI pin map: SCK=GPIO%u MISO=GPIO%u MOSI=GPIO%u CS=GPIO%u INT=GPIO%u\n",
                PIN_SPI_SCK,
                PIN_SPI_MISO,
                PIN_SPI_MOSI,
                PIN_CAN_CS,
                PIN_CAN_INT);
}

static void pulseLed(uint8_t pin, uint32_t& offAt) {
  digitalWrite(pin, HIGH);
  offAt = millis() + LED_PULSE_MS;
}

static void resetI2CBus() {
  Wire.end();
  delay(10);
  Wire.begin();
  delay(5);
  oledOnline = false; // OLED needs reinit after bus reset
}

static void setupOLED(); // forward declaration — defined below BMI270 setup

static void setupBMI270() {
  resetI2CBus();
  if (!IMU.begin()) {
    bmiOnline = false;
    Serial.println("BMI270 init FAILED");
    // OLED was killed by resetI2CBus — bring it back even on failure
    if (!oledOnline) setupOLED();
    return;
  }

  bmiOnline = true;
  bmiHasBaseline = false;
  nextBmiPollMs = millis() + BMI270_POLL_INTERVAL_MS;
  Serial.println("BMI270 init OK (default I2C address)");
  // OLED was killed by resetI2CBus — bring it back
  if (!oledOnline) setupOLED();
}

static void setupOLED() {
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("SSD1306 OLED init FAILED (check wiring: I2C 0x3C)");
    oledOnline = false;
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("COMPASS");
  oled.setCursor(0, 20);
  oled.print("Initialising...");
  oled.display();
  oledOnline = true;
  Serial.println("SSD1306 OLED init OK (I2C 0x3C, 128x64)");
}

static bool bmiEnabled() {
  return digitalRead(PIN_BMI_ENABLE) == LOW;
}

static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static float lerpFloat(float from, float to, float alpha) {
  return from + ((to - from) * alpha);
}

static void setLedOffOnce() {
  if (!ledCurrentlyOff) {
    led_set_state(LED_OFF);
    ledCurrentlyOff = true;
  }
}

static void applyMotionLed(float ax, float ay, float az) {
  if (ledStartupDemoActive) {
    return;
  }

  if (!motionFilterReady) {
    filtAx = ax;
    filtAy = ay;
    filtAz = az;
    motionFilterReady = true;
  } else {
    filtAx = lerpFloat(filtAx, ax, MOTION_FILTER_ALPHA);
    filtAy = lerpFloat(filtAy, ay, MOTION_FILTER_ALPHA);
    filtAz = lerpFloat(filtAz, az, MOTION_FILTER_ALPHA);
  }

  const uint32_t now = millis();
  const float accelMagnitude = sqrtf((filtAx * filtAx) + (filtAy * filtAy) + (filtAz * filtAz));
  const float accelDelta = accelMagnitude - lastAccelMagnitude;
  const float tiltLevel = fmaxf(fabsf(filtAx), fabsf(filtAy));
  const float directionAxis = (fabsf(filtAx) >= fabsf(filtAy)) ? filtAx : filtAy;

  if (fabsf(lastDirectionAxis) > 0.15f && fabsf(directionAxis) > 0.15f) {
    const bool prevPositive = lastDirectionAxis > 0.0f;
    const bool currPositive = directionAxis > 0.0f;
    if (prevPositive != currPositive) {
      directionEventUntilMs = now + 350;
    }
  }

  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  if ((int32_t)(now - directionEventUntilMs) < 0) {
    if (fabsf(directionAxis) < 0.20f) {
      r = 0; g = 255; b = 0;      // Green near center while changing direction
    } else if (directionAxis > 0.0f) {
      r = 255; g = 140; b = 0;    // Orange for one direction
    } else {
      r = 160; g = 0; b = 255;    // Purple for opposite direction
    }
  } else if (tiltLevel >= BMI270_TILT_THRESHOLD_G) {
    if ((filtAx + filtAy) >= 0.0f) {
      r = 0; g = 0; b = 255;      // Blue for tilt one way
    } else {
      r = 255; g = 255; b = 0;    // Yellow for tilt the other way
    }
  } else {
    if (accelDelta >= BMI270_ACCEL_DELTA_THRESHOLD_G) {
      r = 0; g = 255; b = 0;      // Green when speeding up
    } else if (accelDelta <= -BMI270_ACCEL_DELTA_THRESHOLD_G) {
      r = 255; g = 0; b = 0;      // Red when slowing down
    } else {
      r = 16; g = 16; b = 16;     // Neutral dim white
    }
  }

  const float tiltIntensity = clamp01(tiltLevel / 1.0f);
  const float accelIntensity = clamp01(fabsf(accelDelta) * 6.0f);
  const float intensity = fmaxf(tiltIntensity, accelIntensity);
  const float targetBrightness = 32.0f + (223.0f * intensity);

  ledRSmoothed = lerpFloat(ledRSmoothed, (float)r, LED_COLOR_FILTER_ALPHA);
  ledGSmoothed = lerpFloat(ledGSmoothed, (float)g, LED_COLOR_FILTER_ALPHA);
  ledBSmoothed = lerpFloat(ledBSmoothed, (float)b, LED_COLOR_FILTER_ALPHA);
  ledBrightnessSmoothed = lerpFloat(ledBrightnessSmoothed, targetBrightness, LED_BRIGHTNESS_FILTER_ALPHA);

  const uint8_t outR = (uint8_t)ledRSmoothed;
  const uint8_t outG = (uint8_t)ledGSmoothed;
  const uint8_t outB = (uint8_t)ledBSmoothed;
  const uint8_t brightness = (uint8_t)ledBrightnessSmoothed;

  led_set_manual_rgb(outR, outG, outB, brightness);
  ledCurrentlyOff = false;

  lastAccelMagnitude = accelMagnitude;
  lastDirectionAxis = directionAxis;
}

static void serviceLedStartupDemo() {
  if (!ledStartupDemoActive) {
    return;
  }

  const uint32_t now = millis();
  if ((int32_t)(now - ledStartupDemoEndMs) >= 0) {
    ledStartupDemoActive = false;
    Serial.println("LED startup demo complete");
    setLedOffOnce();
  }
}

static void serviceBMI270() {
  if (!bmiOnline) return;

  const uint32_t now = millis();
  if ((int32_t)(now - nextBmiPollMs) < 0) return;
  nextBmiPollMs = now + BMI270_POLL_INTERVAL_MS;

  if (!IMU.accelerationAvailable()) {
    // accelerationAvailable() can fail if the I2C bus locked up mid-session.
    // Mark offline so serviceBMIGated() triggers a full reinit with bus reset.
    bmiOnline = false;
    bmiHasBaseline = false;
    Serial.println("BMI270 I2C fault detected — reinit on next gate cycle");
    return;
  }

  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  IMU.readAcceleration(ax, ay, az);

  if (sdOnline) {
    sdLogFile.printf("%lu,ACCEL,%.4f,%.4f,%.4f,%.4f\n",
                     (unsigned long)millis(), ax, ay, az,
                     sqrtf((ax * ax) + (ay * ay) + (az * az)));
  }

  if (!bmiHasBaseline) {
    lastAx = ax;
    lastAy = ay;
    lastAz = az;
    gravAx = ax;   // capture gravity vector once
    gravAy = ay;
    gravAz = az;
    lastAccelMagnitude = sqrtf((ax * ax) + (ay * ay) + (az * az));
    lastDirectionAxis = (fabsf(ax) >= fabsf(ay)) ? ax : ay;
    bmiHasBaseline = true;
    Serial.printf("BMI270 baseline: ax=%.3f ay=%.3f az=%.3f g\n", ax, ay, az);

    if (!ledStartupDemoActive) {
      applyMotionLed(ax, ay, az);
    }
    return;
  }

  const bool changed =
      (fabsf(ax - lastAx) >= BMI270_ACCEL_CHANGE_THRESHOLD_G) ||
      (fabsf(ay - lastAy) >= BMI270_ACCEL_CHANGE_THRESHOLD_G) ||
      (fabsf(az - lastAz) >= BMI270_ACCEL_CHANGE_THRESHOLD_G);

  if (changed) {
    Serial.printf("BMI270 change: ax=%.3f ay=%.3f az=%.3f g\n", ax, ay, az);
    lastAx = ax;
    lastAy = ay;
    lastAz = az;
    applyMotionLed(ax, ay, az);
  }
}

static void serviceBMIGated() {
  const bool enabled = bmiEnabled();

  if (enabled) {
    if (!bmiGateWasEnabled) {
      Serial.println("BMI270 trigger: GPIO6 LOW (enabled)");
      bmiGateWasEnabled = true;
    }

    if (!bmiOnline) {
      const uint32_t now = millis();
      if ((int32_t)(now - nextBmiReinitMs) >= 0) {
        setupBMI270();
        nextBmiReinitMs = now + BMI270_REINIT_INTERVAL_MS;
      }
    }

    serviceBMI270();
    return;
  }

  if (bmiGateWasEnabled) {
    Serial.println("BMI270 trigger: GPIO6 HIGH (disabled)");
    bmiGateWasEnabled = false;
  }

  if (bmiOnline) {
    // Keep sensor powered, but disable polling/reporting until GPIO6 is grounded again.
    bmiOnline = false;
    bmiHasBaseline = false;
    gravAx = 0.0f;
    gravAy = 0.0f;
    gravAz = 0.0f;
    motionFilterReady = false;
    ledRSmoothed = 0.0f;
    ledGSmoothed = 0.0f;
    ledBSmoothed = 0.0f;
    ledBrightnessSmoothed = 64.0f;
    Serial.println("BMI270 reporting paused");
  }

  if (!ledStartupDemoActive) {
    setLedOffOnce();
  }
}

// ------------------------------ MMC5983MA (magnetometer) ------------------------------

static bool mmcEnabled() {
  return digitalRead(PIN_MMC_ENABLE) == LOW;
}

static void setupMMC5983() {
  // Step 1: verify raw I2C communication before handing off to library
  Wire.beginTransmission(MMC5983MA_I2C_ADDR);
  Wire.write(MMC5983MA_REG_PRODUCT_ID);
  if (Wire.endTransmission(false) != 0) {
    Serial.printf("MMC5983MA init FAILED: no I2C ACK at address 0x%02X\n", MMC5983MA_I2C_ADDR);
    mmcOnline = false;
    return;
  }
  Wire.requestFrom((uint8_t)MMC5983MA_I2C_ADDR, (uint8_t)1);
  if (!Wire.available()) {
    Serial.println("MMC5983MA init FAILED: no data returned from product ID register");
    mmcOnline = false;
    return;
  }
  const uint8_t productId = Wire.read();
  if (productId != MMC5983MA_PRODUCT_ID) {
    Serial.printf("MMC5983MA init FAILED: bad product ID 0x%02X (expected 0x%02X)\n",
                  productId, MMC5983MA_PRODUCT_ID);
    mmcOnline = false;
    return;
  }
  Serial.printf("MMC5983MA I2C OK: product ID=0x%02X\n", productId);

  // Step 2: library init
  resetI2CBus();
  if (!magMMC.begin(Wire)) {
    Serial.println("MMC5983MA library init FAILED");
    mmcOnline = false;
    return;
  }

  mmcOnline = true;
  nextMmcPollMs = millis() + MMC5983MA_POLL_INTERVAL_MS;
  Serial.println("MMC5983MA init OK (I2C address 0x30)");
}

static const char* headingToCardinal(float deg) {
  static const char* const dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  return dirs[(uint8_t)((deg + 22.5f) / 45.0f) % 8];
}

// ------------------------------ OLED multi-screen display ------------------------------

static void renderCompassScreen() {
  if (!oledOnline) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("COMPASS [1/3]");
  if (!lastOledHeadingValid) {
    oled.setCursor(0, 28);
    oled.print("-- OFFLINE --");
    oled.display();
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", lastOledHeading);
  oled.setTextSize(2);
  oled.setCursor(0, 12);
  oled.print(buf);
  oled.setTextSize(1);
  oled.setCursor(80, 14);
  oled.print("deg");
  oled.setTextSize(2);
  oled.setCursor(0, 44);
  oled.print(headingToCardinal(lastOledHeading));
  oled.display();
}

static void renderGpsScreen() {
  if (!oledOnline) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  char buf[22];
  if (gps.location.isValid()) {
    oled.setCursor(0, 0);
    oled.print("GPS  FIX [2/3]");
    snprintf(buf, sizeof(buf), "%.5f", gps.location.lat());
    oled.setCursor(0, 12);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "%.5f", gps.location.lng());
    oled.setCursor(0, 22);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "Alt:%.0fm", gps.altitude.meters());
    oled.setCursor(0, 34);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "Sat:%u", gps.satellites.value());
    oled.setCursor(70, 34);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "%.1fkph", gps.speed.kmph());
    oled.setCursor(0, 46);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "Crs:%.0f", gps.course.deg());
    oled.setCursor(70, 46);
    oled.print(buf);
  } else {
    oled.setCursor(0, 0);
    oled.print("GPS NO FIX [2/3]");
    snprintf(buf, sizeof(buf), "Sats: %u",
             gps.satellites.isValid() ? gps.satellites.value() : 0);
    oled.setCursor(0, 14);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "Chars:%lu", (unsigned long)gps.charsProcessed());
    oled.setCursor(0, 28);
    oled.print(buf);
    snprintf(buf, sizeof(buf), "Fixes:%lu", (unsigned long)gps.sentencesWithFix());
    oled.setCursor(0, 42);
    oled.print(buf);
  }
  oled.display();
}

static void renderAccelScreen() {
  if (!oledOnline) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("ACCEL [3/3]");
  if (!bmiOnline) {
    oled.setCursor(0, 22);
    oled.print("-- OFFLINE --");
    oled.setCursor(0, 34);
    oled.print("GPIO6=enable");
    oled.display();
    return;
  }
  char buf[16];
  const float dx = lastAx - gravAx;
  const float dy = lastAy - gravAy;
  const float dz = lastAz - gravAz;
  snprintf(buf, sizeof(buf), "X:% .3fg", dx);
  oled.setCursor(0, 14);
  oled.print(buf);
  snprintf(buf, sizeof(buf), "Y:% .3fg", dy);
  oled.setCursor(0, 26);
  oled.print(buf);
  snprintf(buf, sizeof(buf), "Z:% .3fg", dz);
  oled.setCursor(0, 38);
  oled.print(buf);
  const float mag = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
  snprintf(buf, sizeof(buf), "M:% .3fg", mag);
  oled.setCursor(0, 50);
  oled.print(buf);
  oled.display();
}

static void renderOledCurrentScreen() {
  if (!oledOnline || calModeActive) return;
  switch (currentScreen) {
    case SCREEN_COMPASS: renderCompassScreen(); break;
    case SCREEN_GPS:     renderGpsScreen();     break;
    case SCREEN_ACCEL:   renderAccelScreen();   break;
    default: break;
  }
}

static void serviceScreenButton() {
  const bool currHigh = (digitalRead(PIN_SCREEN_CYCLE) == HIGH);
  const uint32_t now = millis();
  // Detect falling edge (was HIGH, now LOW) with debounce
  if (!currHigh && screenBtnLastState && (int32_t)(now - screenBtnDebounceMs) >= 0) {
    currentScreen = (OledScreen)((uint8_t)(currentScreen + 1) % SCREEN_COUNT);
    screenBtnDebounceMs = now + SCREEN_BTN_DEBOUNCE_MS;
    static const char* const names[] = {"COMPASS", "GPS", "ACCEL"};
    Serial.printf("OLED screen -> %s\n", names[currentScreen]);
    renderOledCurrentScreen();
  }
  screenBtnLastState = currHigh;
}

static void serviceOledDisplay() {
  const uint32_t now = millis();
  if ((int32_t)(now - nextOledRefreshMs) < 0) return;
  nextOledRefreshMs = now + OLED_REFRESH_INTERVAL_MS;
  renderOledCurrentScreen();
}

static void serviceMMC5983() {
  if (!mmcOnline) return;

  const uint32_t now = millis();
  if ((int32_t)(now - nextMmcPollMs) < 0) return;
  nextMmcPollMs = now + MMC5983MA_POLL_INTERVAL_MS;

  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  magMMC.getMeasurementXYZ(&rawX, &rawY, &rawZ);

  // 18-bit values, null field at center (131072)
  const float fx = (float)rawX - 131072.0f;
  const float fy = (float)rawY - 131072.0f;

  float heading = atan2f(fy, fx) * 180.0f / (float)M_PI;
  if (heading < 0.0f) heading += 360.0f;

  lastOledHeading = heading;
  lastOledHeadingValid = true;

  if ((int32_t)(now - nextMmcLogMs) >= 0) {
    nextMmcLogMs = now + MMC5983MA_LOG_INTERVAL_MS;
    if (sdOnline) {
      sdLogFile.printf("%lu,MAG,%.2f,%lu,%lu,%lu\n",
                       (unsigned long)millis(), heading,
                       (unsigned long)rawX, (unsigned long)rawY, (unsigned long)rawZ);
    }
    Serial.printf("MMC5983MA heading: %.1f deg  X=%lu Y=%lu Z=%lu\n",
                  heading, (unsigned long)rawX, (unsigned long)rawY, (unsigned long)rawZ);
  }
}

static void serviceCalMode() {
  const bool active = (digitalRead(PIN_CAL_MODE) == LOW);

  if (!active) {
    if (calModeActive) {
      calModeActive = false;
      Serial.println("CAL MODE OFF — resuming normal output");
      renderOledCurrentScreen();
    }
    return;
  }

  if (!calModeActive) {
    calModeActive = true;
    nextCalTxMs = millis();
    Serial.println("\n=== CAL MODE ON ===");
    Serial.println("Open MotionCal (pjrc.com/store/prop_shield.html) and connect to this port.");
    Serial.println("Rotate sensor slowly in all directions until gaps < 1%.");
    Serial.println("GPIO6 LOW recommended (enables BMI270 accel data for better results).");
    Serial.println("GPIO16 HIGH to exit.");
    if (oledOnline) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(0, 0);
      oled.print("CALIBRATING");
      oled.setCursor(0, 14);
      oled.print("Rotate sensor");
      oled.setCursor(0, 24);
      oled.print("in all dirs");
      oled.setCursor(0, 36);
      oled.print("Open MotionCal");
      oled.setCursor(0, 48);
      oled.print("on PC (PJRC)");
      oled.display();
    }
  }

  const uint32_t now = millis();
  if ((int32_t)(now - nextCalTxMs) < 0) return;
  nextCalTxMs = now + CAL_TX_INTERVAL_MS;

  // Read fresh mag
  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  if (mmcOnline) {
    magMMC.getMeasurementXYZ(&rawX, &rawY, &rawZ);
  }

  // MotionCal mag format: µT * 10 as integer
  const int mx = (int)(((float)rawX - 131072.0f) * MMC_UT_PER_COUNT * 10.0f);
  const int my = (int)(((float)rawY - 131072.0f) * MMC_UT_PER_COUNT * 10.0f);
  const int mz = (int)(((float)rawZ - 131072.0f) * MMC_UT_PER_COUNT * 10.0f);

  // MotionCal accel format: m/s² * 100 as integer (use last BMI270 reading, 0 if offline)
  const int iax = (int)(lastAx * 9.80665f * 100.0f);
  const int iay = (int)(lastAy * 9.80665f * 100.0f);
  const int iaz = (int)(lastAz * 9.80665f * 100.0f);

  // MotionCal gyro: 0 (no dedicated gyro available)
  Serial.printf("Raw:%d,%d,%d,0,0,0,%d,%d,%d\r\n", iax, iay, iaz, mx, my, mz);
}

static void serviceMMCAlways() {  if (!mmcOnline) {
    const uint32_t now = millis();
    if ((int32_t)(now - nextMmcReinitMs) >= 0) {
      setupMMC5983();
      if (mmcOnline && !oledOnline) {
        setupOLED();
      } else if (!mmcOnline) {
        nextMmcReinitMs = now + MMC5983MA_REINIT_INTERVAL_MS;
        lastOledHeadingValid = false;
      }
    }
    return;
  }
  serviceMMC5983();
}

// -----------------------------------------------------------------------

// ------------------------------ SD card logging ------------------------------

static void setupSD() {
  pinMode((uint8_t)PIN_SD_CS, OUTPUT);
  digitalWrite((uint8_t)PIN_SD_CS, HIGH);

  if (!SD.begin((uint8_t)PIN_SD_CS, spiBus)) {
    Serial.println("SD card init FAILED (check wiring: CS=GPIO47)");
    sdOnline = false;
    return;
  }

  // find the next available log file
  char filename[16];
  for (uint16_t i = 1; i <= 999; i++) {
    snprintf(filename, sizeof(filename), "/LOG%03u.CSV", i);
    if (!SD.exists(filename)) break;
  }

  sdLogFile = SD.open(filename, FILE_WRITE);
  if (!sdLogFile) {
    Serial.printf("SD log file open FAILED: %s\n", filename);
    sdOnline = false;
    return;
  }

  sdLogFile.println("time_ms,type,f1,f2,f3,f4");
  sdLogFile.flush();
  sdOnline = true;
  nextSdFlushMs = millis() + SD_FLUSH_INTERVAL_MS;
  Serial.printf("SD card OK  logging to %s\n", filename);
}

static void serviceSD() {
  if (!sdOnline) return;
  const uint32_t now = millis();
  if ((int32_t)(now - nextSdFlushMs) >= 0) {
    sdLogFile.flush();
    nextSdFlushMs = now + SD_FLUSH_INTERVAL_MS;
  }
}

// ------------------------------ NEO-6M GPS (UART1) ------------------------------

static void setupGPS() {
  Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  gpsRawDiagActive  = true;
  gpsRawDiagEndMs   = millis() + 10000;
  gpsNmeaLineBufPos = 0;
  Serial.printf("NEO-6M GPS init OK (UART1 RX=GPIO%u TX=GPIO%u @ %lu baud)\n",
                PIN_GPS_RX, PIN_GPS_TX, (unsigned long)GPS_BAUD);
  Serial.println("GPS: raw NMEA will print for 10s — silence means a wiring problem");
}

static void serviceGPS() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    gps.encode(c);
    if (gpsRawDiagActive) {
      if (c == '\n') {
        if (gpsNmeaLineBufPos > 0) {
          gpsNmeaLineBuf[gpsNmeaLineBufPos] = '\0';
          Serial.printf("[GPS RAW] %s\n", gpsNmeaLineBuf);
          gpsNmeaLineBufPos = 0;
        }
      } else if (c != '\r' && gpsNmeaLineBufPos < (uint8_t)(sizeof(gpsNmeaLineBuf) - 1)) {
        gpsNmeaLineBuf[gpsNmeaLineBufPos++] = c;
      }
    }
  }

  if (gpsRawDiagActive && (int32_t)(millis() - gpsRawDiagEndMs) >= 0) {
    gpsRawDiagActive = false;
    if (gps.charsProcessed() == 0) {
      Serial.println("GPS WARNING: 0 chars in 10s — check wiring (GPS TX->GPIO2, GPS RX->GPIO1, 3.3V, GND)");
    } else {
      Serial.printf("GPS diag done: %lu chars, %lu sentences with fix, %lu bad checksum\n",
                    (unsigned long)gps.charsProcessed(),
                    (unsigned long)gps.sentencesWithFix(),
                    (unsigned long)gps.failedChecksum());
    }
  }

  const uint32_t now = millis();
  if ((int32_t)(now - nextGpsPrintMs) < 0) return;
  nextGpsPrintMs = now + GPS_PRINT_INTERVAL_MS;

  if (gps.location.isValid()) {
    Serial.printf("GPS FIX: lat=%.6f lon=%.6f alt=%.1fm speed=%.1fkph course=%.1fdeg sats=%u hdop=%.2f\n",
                  gps.location.lat(),
                  gps.location.lng(),
                  gps.altitude.meters(),
                  gps.speed.kmph(),
                  gps.course.deg(),
                  gps.satellites.value(),
                  gps.hdop.hdop());
    if (sdOnline) {
      sdLogFile.printf("%lu,GPS,%.6f,%.6f,%.1f,%.1f\n",
                       (unsigned long)millis(),
                       gps.location.lat(),
                       gps.location.lng(),
                       gps.altitude.meters(),
                       gps.speed.kmph());
    }
  } else {
    Serial.printf("GPS NO FIX: chars=%lu sentences=%lu failed=%lu sats=%u\n",
                  (unsigned long)gps.charsProcessed(),
                  (unsigned long)gps.sentencesWithFix(),
                  (unsigned long)gps.failedChecksum(),
                  gps.satellites.isValid() ? gps.satellites.value() : 0);
  }
}

// -----------------------------------------------------------------------

static void serviceLedIndicators() {
  const uint32_t now = millis();

  if (twaiTxLedOffAt != 0 && (int32_t)(now - twaiTxLedOffAt) >= 0) {
    digitalWrite(PIN_LED_TWAI_TX, LOW);
    twaiTxLedOffAt = 0;
  }
  if (twaiRxLedOffAt != 0 && (int32_t)(now - twaiRxLedOffAt) >= 0) {
    digitalWrite(PIN_LED_TWAI_TX, LOW);
    twaiRxLedOffAt = 0;
  }
  if (spiTxLedOffAt != 0 && (int32_t)(now - spiTxLedOffAt) >= 0) {
    digitalWrite(PIN_LED_SPI_TX, LOW);
    spiTxLedOffAt = 0;
  }
  if (spiRxLedOffAt != 0 && (int32_t)(now - spiRxLedOffAt) >= 0) {
    digitalWrite(PIN_LED_SPI_TX, LOW);
    spiRxLedOffAt = 0;
  }
}

static void setupTWAI() {
  ESP32Can.setPins(PIN_TWAI_TX, PIN_TWAI_RX);
  if (ESP32Can.begin(ESP32Can.convertSpeed(CAN_TWAI_BITRATE))) {
    Serial.println("TWAI started @ 500k (TX=GPIO8 RX=GPIO9)");
  } else {
    Serial.println("TWAI FAILED");
    while (true) delay(1000);
  }
}

static bool setupSPICANOnce() {
  if (spiCanStarted) {
    canSPI.end();
    delay(2);
  }

  if (PIN_SD_CS >= 0) {
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
  }

  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  pinMode(PIN_CAN_INT, INPUT_PULLUP);

  spiBus.end();
  delay(5);
  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CAN_CS);
  delay(10);

  ACAN2517FDSettings settings(
    ACAN2517FDSettings::OSC_40MHz,
    CAN_SPI_BITRATE,
    ACAN2517FDSettings::DATA_BITRATE_x1,
    256
  );
  settings.mDriverReceiveFIFOSize = 200;
  settings.mRequestedMode = ACAN2517FDSettings::Normal20B;

  const uint32_t errorCode = canSPI.begin(settings, [] { canSPI.isr(); });
  if (errorCode == 0) {
    spiCanStarted = true;
    spiCanOnline = true;
    lastSpiRxMs = millis();
    spiIntLowSinceMs = 0;
    Serial.printf("SPI MCP2517FD started @ %lu\n", (unsigned long)CAN_SPI_BITRATE);
    Serial.printf("SPI MCP2517FD init OK: CS=GPIO%u INT=GPIO%u\n", PIN_CAN_CS, PIN_CAN_INT);
    return true;
  }

  Serial.print("SPI MCP2517FD FAILED, error=0x");
  Serial.println(errorCode, HEX);
  Serial.printf("SPI MCP2517FD init FAILED on CS=GPIO%u INT=GPIO%u\n", PIN_CAN_CS, PIN_CAN_INT);
  return false;
}

static bool setupSPICANWithRetry() {
  for (uint8_t attempt = 1; attempt <= SPI_CAN_BOOT_INIT_RETRIES; attempt++) {
    Serial.printf("SPI MCP2517FD init attempt %u/%u...\n", attempt, SPI_CAN_BOOT_INIT_RETRIES);
    if (setupSPICANOnce()) return true;
    if (attempt < SPI_CAN_BOOT_INIT_RETRIES) delay(SPI_CAN_RETRY_DELAY_MS);
  }

  spiCanOnline = false;
  return false;
}

static void serviceSPICANRetry() {
  if (spiCanOnline) return;

  const uint32_t now = millis();
  if ((int32_t)(now - nextSpiCanRetryMs) < 0) return;

  Serial.println("SPI MCP2517FD offline, retrying init...");
  if (setupSPICANOnce()) {
    spiCanOnline = true;
    Serial.println("SPI MCP2517FD recovered.");
  } else {
    nextSpiCanRetryMs = now + SPI_CAN_RETRY_INTERVAL_MS;
  }
}

static void serviceSPICANHealth() {
  if (!spiCanOnline) return;

  canSPI.poll();

  const uint32_t now = millis();
  const bool intIsLow = (digitalRead(PIN_CAN_INT) == LOW);
  if (intIsLow) {
    if (spiIntLowSinceMs == 0) {
      spiIntLowSinceMs = now;
    } else if ((int32_t)(now - spiIntLowSinceMs) >= (int32_t)SPI_CAN_RX_STALL_MS &&
               (int32_t)(now - lastSpiRxMs) >= (int32_t)SPI_CAN_RX_STALL_MS) {
      Serial.println("SPI CAN RX stall detected; forcing reinit");
      spiCanOnline = false;
      nextSpiCanRetryMs = now;
      spiIntLowSinceMs = 0;
    }
  } else {
    spiIntLowSinceMs = 0;
  }
}

static void printTwaiFrame(const CanFrame& frame) {
  Serial.printf("TWAI RX ID=0x%03lX LEN=%u DATA=", (unsigned long)frame.identifier, frame.data_length_code);
  for (uint8_t i = 0; i < frame.data_length_code; i++) {
    Serial.printf("%02X", frame.data[i]);
    if (i + 1 < frame.data_length_code) Serial.print(" ");
  }
  Serial.println();
}

static void printSpiFrame(const CANFDMessage& frame, uint8_t len) {
  Serial.printf("SPI  RX ID=0x%03lX LEN=%u DATA=", (unsigned long)frame.id, len);
  for (uint8_t i = 0; i < len; i++) {
    Serial.printf("%02X", frame.data[i]);
    if (i + 1 < len) Serial.print(" ");
  }
  Serial.println();
}

static void receiveCanTWAI() {
  CanFrame frame = {};
  uint8_t count = 0;
  while ((count < 20) && ESP32Can.readFrame(frame, 0)) {
    pulseLed(PIN_LED_TWAI_TX, twaiRxLedOffAt);
    printTwaiFrame(frame);
    count++;
  }
}

static void receiveCanSPI() {
  if (!spiCanOnline) return;

  CANFDMessage frame;
  uint8_t count = 0;
  while ((count < 64) && canSPI.receive(frame)) {
    lastSpiRxMs = millis();
    spiIntLowSinceMs = 0;
    pulseLed(PIN_LED_SPI_TX, spiRxLedOffAt);

    const uint8_t len = (frame.len > 8) ? 8 : frame.len;
    printSpiFrame(frame, len);
    count++;
  }
}

static bool demoTwaiEnabled() {
  return digitalRead(PIN_DEMO_TWAI_ENABLE) == LOW;
}

static bool demoSpiEnabled() {
  return digitalRead(PIN_DEMO_SPI_ENABLE) == LOW;
}

static void sendDemoTwai() {
  CanFrame frame = {};
  frame.identifier = TWAI_DEMO_ID;
  frame.extd = 0;
  frame.rtr = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0x54; // 'T'
  frame.data[1] = 0x57; // 'W'
  frame.data[2] = 0x41; // 'A'
  frame.data[3] = 0x49; // 'I'
  frame.data[4] = twaiDemoCounter++;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  if (ESP32Can.writeFrame(frame, 20)) {
    pulseLed(PIN_LED_TWAI_TX, twaiTxLedOffAt);
    Serial.printf("TWAI TX ID=0x%03X CNT=%u\n", TWAI_DEMO_ID, frame.data[4]);
  }
}

static void sendDemoSpi() {
  if (!spiCanOnline) {
    spiDemoSkippedOffline++;
    Serial.printf("SPI TX skipped: controller offline (skipped=%lu)\n", (unsigned long)spiDemoSkippedOffline);
    return;
  }

  CANFDMessage frame;
  frame.id = SPI_DEMO_ID;
  frame.ext = false;
  frame.type = CANFDMessage::CAN_DATA;
  frame.len = 8;
  frame.data[0] = 0x53; // 'S'
  frame.data[1] = 0x50; // 'P'
  frame.data[2] = 0x49; // 'I'
  frame.data[3] = 0x43; // 'C'
  frame.data[4] = spiDemoCounter++;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  spiDemoAttempts++;
  if (canSPI.tryToSend(frame)) {
    spiDemoQueued++;
    pulseLed(PIN_LED_SPI_TX, spiTxLedOffAt);
    Serial.printf("SPI  TX ID=0x%03X CNT=%u queued ok (attempts=%lu queued=%lu failed=%lu skipped=%lu)\n",
                  SPI_DEMO_ID,
                  frame.data[4],
                  (unsigned long)spiDemoAttempts,
                  (unsigned long)spiDemoQueued,
                  (unsigned long)spiDemoFailed,
                  (unsigned long)spiDemoSkippedOffline);
  } else {
    spiDemoFailed++;
    Serial.printf("SPI  TX ID=0x%03X CNT=%u queue FAILED (attempts=%lu queued=%lu failed=%lu skipped=%lu)\n",
                  SPI_DEMO_ID,
                  frame.data[4],
                  (unsigned long)spiDemoAttempts,
                  (unsigned long)spiDemoQueued,
                  (unsigned long)spiDemoFailed,
                  (unsigned long)spiDemoSkippedOffline);
  }
}

static void serviceDemoTx() {
  const uint32_t now = millis();

  if (demoTwaiEnabled() && (int32_t)(now - nextTwaiDemoTxMs) >= 0) {
    sendDemoTwai();
    nextTwaiDemoTxMs = now + DEMO_TX_INTERVAL_MS;
  }

  if (demoSpiEnabled() && (int32_t)(now - nextSpiDemoTxMs) >= 0) {
    sendDemoSpi();
    nextSpiDemoTxMs = now + DEMO_TX_INTERVAL_MS;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED_TWAI_TX, OUTPUT);
  pinMode(PIN_LED_SPI_TX, OUTPUT);
  digitalWrite(PIN_LED_TWAI_TX, LOW);;
  digitalWrite(PIN_LED_SPI_TX, LOW);

  pinMode(PIN_DEMO_TWAI_ENABLE, INPUT_PULLUP);
  pinMode(PIN_DEMO_SPI_ENABLE, INPUT_PULLUP);
  pinMode(PIN_BMI_ENABLE, INPUT_PULLUP);
  pinMode(PIN_CAL_MODE, INPUT_PULLUP);
  pinMode(PIN_SCREEN_CYCLE, INPUT_PULLUP);

  Wire.begin();

  Serial.println("ESP32-S3 CAN demo bridge");
  printSPIPinMap();
  Serial.println("GPIO4 LOW = enable TWAI demo TX");
  Serial.println("GPIO5 LOW = enable SPI demo TX");
  Serial.println("GPIO6 LOW = enable BMI270 reporting");
  Serial.println("GPIO16 LOW = enter MotionCal calibration mode");
  Serial.println("GPIO40 LOW = cycle OLED screen (COMPASS -> GPS -> ACCEL)");
  Serial.println("MMC5983MA + OLED always enabled");
  Serial.printf("NEO-6M GPS on UART1 RX=GPIO%u TX=GPIO%u\n", PIN_GPS_RX, PIN_GPS_TX);
  Serial.printf("Boot input state: GPIO4=%s GPIO5=%s\n",
                demoTwaiEnabled() ? "LOW/enabled" : "HIGH/disabled",
                demoSpiEnabled() ? "LOW/enabled" : "HIGH/disabled");
  Serial.printf("Boot input state: GPIO6=%s\n",
                bmiEnabled() ? "LOW/enabled" : "HIGH/disabled");

  setupTWAI();
  if (!setupSPICANWithRetry()) {
    Serial.println("SPI MCP2517FD failed to start after retries; loop retries enabled.");
    nextSpiCanRetryMs = millis() + SPI_CAN_RETRY_INTERVAL_MS;
  }

  setupSD();
  setupGPS();
  setupMMC5983();
  setupOLED();

  nextTwaiDemoTxMs = millis() + DEMO_TX_INTERVAL_MS;
  nextSpiDemoTxMs = millis() + DEMO_TX_INTERVAL_MS;
  ledStartupDemoEndMs = millis() + LED_STARTUP_DEMO_MS;

  led_init();
  led_set_state(LED_RANDOM);
  Serial.printf("LED startup demo for %lu ms\n", (unsigned long)LED_STARTUP_DEMO_MS);

  Serial.println("Ready.");
}

void loop() {
  serviceLedStartupDemo();
  serviceLedIndicators();
  led_update();

  serviceSPICANRetry();
  serviceSPICANHealth();

  receiveCanTWAI();
  receiveCanSPI();
  serviceDemoTx();
  serviceBMIGated();
  serviceCalMode();
  serviceMMCAlways();
  serviceGPS();
  serviceScreenButton();
  serviceOledDisplay();
  serviceSD();

  delay(1);
}
