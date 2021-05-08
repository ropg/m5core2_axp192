#ifndef _M5CORE2_H
#define _M5CORE2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "axp192.h"

#include <esp_err.h>

// Some functions modified from / inpspired by @usedbytes - Brian Starkey's code at
// https://github.com/usedbytes/axp192

typedef enum {
    AXP192_RAIL_DCDC1 = 0,
    AXP192_RAIL_DCDC2,
    AXP192_RAIL_DCDC3,
    AXP192_RAIL_LDO1,
    AXP192_RAIL_LDO2,
    AXP192_RAIL_LDO3,
    AXP192_RAIL_EXTEN,

    AXP192_RAIL_COUNT,
} axp192_rail_t;

#define ESP_POWER       AXP192_RAIL_DCDC1
#define LCD_BACKLIGHT   AXP192_RAIL_DCDC3
#define LOGIC_AND_SD    AXP192_RAIL_LDO2
#define VIBRATOR        AXP192_RAIL_LDO3


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


#ifdef __cplusplus
}
#endif
#endif
