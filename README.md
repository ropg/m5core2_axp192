# M5Core2 AXP192 Power Management component

This will let you initialise the M5Core2 device. If you do

```c
#include "m5core2_axp192.h"

m5core2_init();
```

your M5Core2 will do the same things that happen in `m5.begin()` on Arduino:

```
I (387) m5core2_axp192: Initialising
I (397) i2c_manager: Starting I2C master at port 0.
I (397) i2c_manager: Initialised port 0 (SDA: 21, SCL: 22, speed: 1000000 Hz.)
I (407) m5core2_axp192: 	Vbus limit off
I (407) m5core2_axp192: 	Speaker amplifier off
I (417) m5core2_axp192: 	RTC battery charging enabled (3v, 200uA)
I (427) m5core2_axp192: 	ESP32 power voltage set to 3.35v
I (427) m5core2_axp192: 	LCD backlight voltage set to 2.80v
I (437) m5core2_axp192: 	LCD logic and sdcard voltage set to 3.3v
I (437) m5core2_axp192: 	Vibrator voltage preset to 2v
I (447) m5core2_axp192: 	LED on
I (447) m5core2_axp192: 	Charge current set to 100 mA
I (457) m5core2_axp192: 	Battery voltage now: 4.18 volts
I (467) m5core2_axp192: 	Power key set, 4 seconds for hard shutdown
I (467) m5core2_axp192: 	Enabled all ADC channels
I (477) m5core2_axp192: 	USB / battery powered, 5V bus on
I (577) m5core2_axp192: 	LCD and touch reset
```

Presently the following functions are implemented:

```c
esp_err_t m5core2_init();
esp_err_t m5core2_int_5v(bool on);
esp_err_t m5core2_led(bool on);
esp_err_t m5core2_vibration(bool on);
esp_err_t m5core2_speaker(bool on);
esp_err_t m5core2_axp_read_reg(uint8_t reg, uint8_t *buffer);
esp_err_t m5core2_axp_write_reg(uint8_t reg, uint8_t value);
esp_err_t m5core2_axp_read(uint8_t reg, float *buffer);
esp_err_t m5core2_axp_twiddle(uint8_t reg, uint8_t affect, uint8_t value);
esp_err_t m5core2_get_rail_state(axp192_rail_t rail, bool *enabled);
esp_err_t m5core2_set_rail_state(axp192_rail_t rail, bool enabled);
esp_err_t m5core2_get_rail_mv(axp192_rail_t rail, uint16_t *millivolts);
esp_err_t m5core2_set_rail_mv(axp192_rail_t rail, uint16_t millivolts);
```

This component depends on Mika Tuupola's [`axp192`](https://github.com/tuupola/axp192) and [`i2c_manager`](https://github.com/ropg/i2c_manager). The only menuconfig setting is to say which ESP32 I2C port is used for the internal I2C bus of the M5Core2. To see this component in action, simply follow the instructions at this build-ready [demo application](https://github.com/ropg/m5core2_esp-idf_demo).
