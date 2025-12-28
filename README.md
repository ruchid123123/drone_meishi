# ESP32 Mini Drone FC

Please see the blog post for full details.
https://fumimaker.net/drone_business_card_rev32

![Figure 5](docs/fig/image2.png)

## Overview

Firmware for an ESP32-based micro drone FC with IMU fusion, PID control, and a Web UI controller.

[![YouTube](https://img.youtube.com/vi/yCas-xbfeBE/0.jpg)](https://www.youtube.com/watch?v=yCas-xbfeBE)

## Runtime Behavior

- Control loop: 500 Hz (IMU read, attitude estimate, PID, motor output)
- Motor PWM: 20 kHz
- WebSocket control input: 50 Hz (Web UI)
- Telemetry: 20 Hz default
- Web server/WebSocket: event-driven via ESPAsyncWebServer + AsyncTCP

## Dependencies

- ESPAsyncWebServer
- AsyncTCP

## Flashing

1. Install Arduino IDE + ESP32 core.
2. Install the dependencies above.
3. Select **ESP32 Devkit** and upload.

## Usage

1. Power on: ESP32 starts a Wi-Fi AP.
   - SSID: `ESP32-DRONE`
   - Password: `12345678`
2. Connect and open `http://192.168.4.1/`.
3. Calibrate in **SETTINGS**: GYRO, then ACC.
4. Adjust PID gains if needed, then **SAVE**.
5. Switch to **CONTROL**, enable **ARM** (arms after ~800 ms), and fly.
6. **DISARM** stops immediately; tilt > ~80° auto-disarms.

![Figure 1](docs/fig/IMG_6699.jpeg)
![Figure 3](docs/fig/IMG_6702.jpeg)
![Figure 2](docs/fig/IMG_6701.jpeg)
![Figure 4](docs/fig/IMG_6724.jpeg)

