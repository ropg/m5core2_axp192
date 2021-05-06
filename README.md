# M5Core2 AXP192 Power Management component

This will let you initialise the M5Core2 device. If you do

```c
#include "m5core2_axp192.h"

m5core2_init();
```

your M5Core2 will do the same things that happen in the Arduino AXP192 init routine:

```
I (386) m5core2_axp192: Initialising
I (396) i2c_manager: Starting I2C master at port 0.
I (396) i2c_manager: Initialised port 0 (SDA: 21, SCL: 22, speed: 1000000 Hz.)
I (406) m5core2_axp192: 	Vbus limit off
I (406) m5core2_axp192: 	Speaker amplifier off
I (416) m5core2_axp192: 	RTC battery charging enabled (3v, 200uA)
I (426) m5core2_axp192: 	ESP32 power voltage set to 3.35v
I (426) m5core2_axp192: 	LCD backlight voltage set to 2.80v
I (436) m5core2_axp192: 	LCD logic and sdcard voltage preset to 3.3v
I (436) m5core2_axp192: 	Vibrator voltage preset to 2v
I (446) m5core2_axp192: 	LED on
I (446) m5core2_axp192: 	Charge current set to 100 mA
I (456) m5core2_axp192: 	Battery voltage now: 4.18 volts
I (466) m5core2_axp192: 	Power key set, 4 seconds for hard shutdown
I (466) m5core2_axp192: 	Enabled all ADC channels
I (476) m5core2_axp192: 	USB / battery powered
I (576) m5core2_axp192: 	LCD and touch reset
```

This component presently implements the following functions:

```c
esp_err_t m5core2_init();
esp_err_t m5core2_int_5v(bool on);
esp_err_t m5core2_led(bool on);
esp_err_t m5core2_vibration(bool on);
esp_err_t m5core2_speaker(bool on);
```

This component depends on [`axp192`](https://github.com/ropg/axp192) (my modified version)  and [`i2c_manager`](https://github.com/ropg/i2c_manager). I2C port 0 needs to be set up to the internal I2C port of the M5Core2. To see this component in action, simply follow the instructions at this build-ready [demo application](https://github.com/ropg/m5core2_esp-idf_demo).
