#ifndef _M5CORE2_H
#define _M5CORE2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "axp192.h"

#include <esp_err.h>

esp_err_t m5core2_init();
esp_err_t m5core2_int_5v(bool on);
esp_err_t m5core2_led(bool on);
esp_err_t m5core2_vibration(bool on);
esp_err_t m5core2_speaker(bool on);


#ifdef __cplusplus
}
#endif
#endif
