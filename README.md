# ESP32-C6 FreeRTOS RGB blink (WS2812 one-wire, red)


This project contains a minimal FreeRTOS example that blinks the on-board addressable RGB LED (WS2812/NeoPixel, one-wire) on an ESP32-C6 in red.

By default it uses `GPIO8` and expects a single LED. Change these at the top of `main/main.c`:

- `#define LED_STRIP_GPIO 8`
- `#define LED_STRIP_LED_NUM 1`



## How to build, flash, and monitor

Prerequisites: ESP-IDF installed and the environment exported (PowerShell).

1) Configure your board target (only needed once per workspace):

```powershell
idf.py set-target esp32c6
```

2) Build, flash, and monitor (adjust COM port if needed):

```powershell
idf.py -p COMx build flash monitor
```

Exit the monitor with `Ctrl+]`.

## Example folder contents

The project contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
### Notes

- Uses the ESP-IDF `led_strip` driver (RMT) to drive WS2812 LEDs.
- ESP32-C6-DevKit boards route the on-board addressable RGB LED data line to `GPIO8`.
