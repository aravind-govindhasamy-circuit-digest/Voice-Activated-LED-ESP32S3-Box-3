# ESP32-S3-BOX-3 Smart LED Control

An enhanced ESP-IDF project for ESP32-S3-BOX-3 that demonstrates remote LED control via WiFi/MQTT, touchscreen, and speech recognition.

## Features

- **Multi-Control LED**: Toggle the LED state using:
    - **Voice Commands**: "Turn on light" and "Turn off light"
    - **Touchscreen**: Interactive image button on the main screen.
    - **MQTT**: Remote control via your favorite MQTT broker/dashboard.
- **Bi-directional Sync**: Any change to the LED state (via touch, voice, or MQTT) is immediately reflected on the device screen and published back to MQTT.
- **Connectivity Status Screen**: Access a dedicated status page (via the gear icon) to see real-time WiFi (SSID, IP) and MQTT status.
- **Robust Connection**: Automated WiFi auto-reconnect and persistent MQTT connection management.
- **Simplified Configuration**: Use `idf.py menuconfig` to set your credentials without touching code.

## Hardware Requirements

- **ESP32-S3-BOX-3** development board
- ✅ **Touch Control**: Full-screen button with ON/OFF images (320x240)
- ✅ **Speech Recognition**: English voice commands
  - "turn on light" → LED ON
  - "turn off light" → LED OFF
- ✅ **SR Visual Feedback**: Listening screen with animation and text overlay
- ✅ **Audio Feedback**: Tones for wake word, command recognition, and timeout
- ✅ **State Persistence**: LED state saved to NVS and restored on boot

## Project Structure

```
factory_demo/
├── main/
│   ├── app/
│   │   ├── app_led.c/h          # Single LED GPIO control (GPIO40)
│   │   ├── app_sr.c/h            # Speech recognition engine
│   │   └── app_sr_handler.c/h    # SR command handler + audio feedback
│   ├── gui/
│   │   ├── ui_boot_animate.c/h   # Boot animation
│   │   ├── ui_sr.c/h             # SR listening screen overlay
│   │   └── image/
│   │       ├── switchOn.h        # ON state image (BMP converted)
│   │       └── switchOff.h       # OFF state image (BMP converted)
│   ├── light_ctrl.c/h            # LED control abstraction layer
│   ├── light_state.c/h           # NVS state persistence
│   ├── light_ui.c/h              # LVGL touchscreen UI
│   └── main.c                    # Application entry point
├── spiffs/                       # SPIFFS filesystem (audio files)
│   └── echo_en_*.wav             # English audio feedback tones
└── CMakeLists.txt
```

## Prerequisites

1. **ESP-IDF v5.5+** installed and configured
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

2. **Python 3.12+** (required by ESP-IDF)

## Build and Flash

1. **Navigate to project directory:**
   ```bash
   cd /Users/semiconmediapvtltd/Desktop/box3/factory_demo
   ```

2. **Set up ESP-IDF environment:**
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

3. **Configure (optional):**
   ```bash
   idf.py menuconfig
   ```

4. **Build:**
   ```bash
   idf.py build
   ```

5. **Flash and monitor:**
   ```bash
   idf.py flash monitor
   ```

   Or flash and monitor separately:
   ```bash
   idf.py flash
   idf.py monitor
   ```

## Usage

### Touch Control
- Tap anywhere on the screen to toggle the LED state
- The screen displays either the "ON" or "OFF" image based on current LED state

### Voice Control
1. Say the wake word (configured in ESP-SR)
2. Wait for the listening animation
3. Say one of the commands:
   - **"turn on light"** → Turns LED ON
   - **"turn off light"** → Turns LED OFF
4. Audio feedback confirms command recognition

### LED Connection
Connect your LED to **GPIO40**:
- **Anode** → GPIO40 (via current-limiting resistor, e.g., 220Ω)
- **Cathode** → GND

The LED is driven as a simple digital output (HIGH = ON, LOW = OFF).

## Configuration

### LED GPIO Pin
To change the LED GPIO pin, edit `main/app/app_led.c`:
```c
#define SINGLE_LED_GPIO GPIO_NUM_40  // Change to your desired GPIO
```

### Speech Commands
Speech commands are defined in `main/app/app_sr.c`:
```c
static const sr_cmd_t g_default_cmd_info[] = {
    {SR_CMD_LIGHT_ON, SR_LANG_EN, 0, "turn on light", "TkN nN LiT", {NULL}},
    {SR_CMD_LIGHT_OFF, SR_LANG_EN, 0, "turn off light", "TkN eF LiT", {NULL}},
};
```

### Audio Feedback
Audio feedback tones are stored in `spiffs/`:
- `echo_en_wake.wav` - Wake word detected
- `echo_en_ok.wav` - Command recognized
- `echo_en_end.wav` - Timeout/end of listening

## Troubleshooting

### LED Always On/Off
- Check GPIO40 connection and wiring
- Verify LED polarity (common cathode)
- Check if GPIO40 conflicts with other peripherals

### Speech Recognition Not Working
- Ensure microphone array is connected
- Check audio codec initialization in logs
- Verify SR model files are flashed correctly

### White Screen After Boot
- Check image conversion (BMP to RGB565)
- Verify LVGL display initialization
- Check display backlight settings

### Audio Not Playing
- Verify SPIFFS partition is mounted
- Check audio files exist in `spiffs/`
- Verify I2S codec initialization

## License

SPDX-License-Identifier: Unlicense OR CC0-1.0

Copyright (c) 2015-2024 Espressif Systems (Shanghai) CO LTD
