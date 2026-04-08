# ESP32-S3 Dual CAN-FD Development Board

An ESP32-S3-based development board featuring dual CAN bus interfaces (TWAI + SPI MCP2517FD), a 6-axis IMU, magnetometer compass, OLED display, SD card logging, and an addressable RGB status LED.

## Features

- **Dual CAN Bus** — Built-in ESP32 TWAI (CAN 2.0) + external MCP2517FD (CAN-FD) via SPI
- **IMU** — BMI270 6-axis accelerometer/gyroscope (I2C) with motion-reactive LED feedback
- **Magnetometer** — MMC5983MA digital compass (I2C) with MotionCal calibration support
- **OLED Display** — SSD1306 128×64 (I2C) showing live compass heading and cardinal direction
- **SD Card Logging** — Logs accelerometer and magnetometer data to CSV on a microSD card (SPI)
- **RGB Status LED** — WS2812 NeoPixel on GPIO 21 with 10 animated patterns and motion-driven color modes
- **Demo TX Mode** — GPIO-gated CAN transmit for testing both buses independently

## Hardware

- **MCU:** ESP32-S3-DevKitM-1
- **CAN-FD Controller:** MCP2517FD (SPI)
- **IMU:** BMI270 + BMM150 (I2C)
- **Magnetometer:** MMC5983MA (I2C, address 0x30)
- **Display:** SSD1306 OLED 128×64 (I2C, address 0x3C)
- **LED:** WS2812 NeoPixel (1 pixel)

## Pin Assignments

### I2C Bus (Wire — default ESP32-S3 pins)

| Function | Pin |
|---|---|
| SDA | Default ESP32-S3 I2C SDA |
| SCL | Default ESP32-S3 I2C SCL |

Devices on I2C: BMI270 IMU, MMC5983MA magnetometer (0x30), SSD1306 OLED (0x3C)

### SPI Bus (FSPI)

| Function | GPIO |
|---|---|
| SCK | 14 |
| MISO | 10 |
| MOSI | 11 |
| CAN CS | 13 |
| CAN INT | 12 |
| SD CS | 47 |

### CAN Bus — TWAI (built-in)

| Function | GPIO |
|---|---|
| TWAI TX | 18 |
| TWAI RX | 17 |

Default bitrate: 500 kbps

### CAN Bus — SPI MCP2517FD

| Function | GPIO |
|---|---|
| CS | 13 |
| INT | 12 |
| SCK | 14 |
| MISO | 10 |
| MOSI | 11 |

Default bitrate: 500 kbps (CAN 2.0B mode)

### Status / Indicator LEDs

| Function | GPIO |
|---|---|
| NeoPixel RGB LED | 21 |
| TWAI TX/RX Activity LED | 38 |
| SPI CAN TX/RX Activity LED | 39 |

### Control Inputs (active LOW, INPUT_PULLUP)

| Function | GPIO | Description |
|---|---|---|
| TWAI Demo TX Enable | 4 | Ground to enable periodic TWAI demo transmissions (ID 0x321) |
| SPI Demo TX Enable | 5 | Ground to enable periodic SPI CAN demo transmissions (ID 0x322) |
| BMI270 Enable | 6 | Ground to enable IMU polling and motion-reactive LED |
| MMC5983MA Enable | 15 | Ground to enable magnetometer polling |
| Calibration Mode | 16 | Ground to enter MotionCal calibration stream mode |

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
cd firmware/demo
pio run                              # build
pio run --target upload              # flash via USB
pio device monitor                   # serial monitor @ 115200 baud
```

## Serial Commands / Runtime Behavior

On boot, the firmware prints the GPIO state and starts all peripherals. Control behavior by grounding the input pins:

- **GPIO 4 LOW** — Transmit TWAI demo frames (ID 0x321) every 500 ms
- **GPIO 5 LOW** — Transmit SPI CAN demo frames (ID 0x322) every 500 ms
- **GPIO 6 LOW** — Enable BMI270 accelerometer; LED color reflects tilt and motion
- **GPIO 16 LOW** — Enter MotionCal calibration mode (streams `Raw:` data at 100 Hz for PJRC MotionCal)

The MMC5983MA magnetometer and OLED compass display are always active (auto-reinit on failure).

## SD Card Logging

When a microSD card is present (CS on GPIO 47, sharing the FSPI bus), sensor data is logged to `/LOGnnn.CSV` with columns:

```
time_ms,type,f1,f2,f3,f4
```

- `ACCEL` rows: ax, ay, az, magnitude (g)
- `MAG` rows: heading (deg), rawX, rawY, rawZ

## Dependencies

Managed by PlatformIO (`lib_deps` in `platformio.ini`):

| Library | Version |
|---|---|
| ArduinoJson | ^6.21.5 |
| acan2517FD | ^2.1.16 |
| ESP32-TWAI-CAN | ^1.0.1 |
| Adafruit NeoPixel | ^1.12.5 |
| Arduino_BMI270_BMM150 | ^1.2.3 |
| SparkFun MMC5983MA | ^1.0.5 |
| Adafruit SSD1306 | ^2.5.9 |

## License

See repository root for license details.
