# ESP32-C6 FreeRTOS RGB blink + Stepper Motor Control

This project contains a minimal FreeRTOS example for ESP32-C6 with:
- **RGB LED**: Blinks the on-board addressable RGB LED (WS2812/NeoPixel) in red
- **Stepper Motor**: Controls a stepper motor via boot button (GPIO9)

## Hardware Configuration

### RGB LED (WS2812)
By default uses `GPIO8`. Change at the top of `main/main.c`:
- `#define LED_STRIP_GPIO 8`
- `#define LED_STRIP_LED_NUM 1`

### Stepper Motor
Default GPIO assignments (configurable in `main/main.c`):
- **Direction**: `GPIO0` - Controls rotation direction
- **Enable**: `GPIO1` - Enables/disables motor (active low for most drivers)
- **Step**: `GPIO2` - Step pulse signal
- **Button**: `GPIO9` - Boot button (active low, internal pull-up)

**Motor behavior:**
- Press and hold the boot button to enable and rotate the motor
- Release the button to stop and disable the motor
- Default speed: ~1000 steps/second (adjustable via `STEP_DELAY_US`)

### Wiring Example
For common stepper drivers (A4988, DRV8825, TMC2208, etc.):
```
ESP32-C6      Stepper Driver
--------      --------------
GPIO0    -->  DIR
GPIO1    -->  EN (or !EN)
GPIO2    -->  STEP
GND      -->  GND
3.3V     -->  VDD (logic power)
           
Driver needs separate motor power supply connected to VMOT/VIN
```



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
