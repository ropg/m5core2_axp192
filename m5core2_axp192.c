#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "axp192.h"
#include "i2c_manager.h"

#include "m5core2_axp192.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>


#define AXP_I2C	i2c_hal(0)

static const char* TAG = "m5core2_axp192";



esp_err_t m5core2_init() {

    ESP_LOGI(TAG, "Initialising");

    // turn off everything except bit 2 and then turn bit 1 on
    if (axp192_bits(AXP_I2C, AXP192_VBUS_IPSOUT_CHANNEL, ~0x04, 0x02) == ESP_OK) {
        ESP_LOGI(TAG, "\tVbus limit off");
    }

    if (axp192_bits(AXP_I2C, AXP192_GPIO2_CONTROL, 0x07, 0x00) == ESP_OK &&
        m5core2_speaker(false) == ESP_OK) {
        ESP_LOGI(TAG, "\tSpeaker amplifier off");
    }

    if (axp192_bits(AXP_I2C, AXP192_BATTERY_CHARGE_CONTROL, ~0x1c, 0xa2) == ESP_OK) {
        ESP_LOGI(TAG, "\tRTC battery charging enabled (3v, 200uA)");
    }

    if (axp192_set_rail_millivolts(AXP_I2C, AXP192_RAIL_DCDC1, 3350) == ESP_OK &&
        axp192_set_rail_state(AXP_I2C, AXP192_RAIL_DCDC1, true) == ESP_OK) {
        ESP_LOGI(TAG, "\tESP32 power voltage set to 3.35v");
    }

    if (axp192_set_rail_millivolts(AXP_I2C, AXP192_RAIL_DCDC3, 2800) == ESP_OK &&
        axp192_set_rail_state(AXP_I2C, AXP192_RAIL_DCDC3, true) == ESP_OK) {
        ESP_LOGI(TAG, "\tLCD backlight voltage set to 2.80v");
    }

    if (axp192_set_rail_millivolts(AXP_I2C, AXP192_RAIL_LDO2, 3300) == ESP_OK &&
        axp192_set_rail_state(AXP_I2C, AXP192_RAIL_LDO2, true) == ESP_OK) {
        ESP_LOGI(TAG, "\tLCD logic and sdcard voltage preset to 3.3v");
    }

    if (axp192_set_rail_millivolts(AXP_I2C, AXP192_RAIL_LDO3, 2000) == ESP_OK) {
        ESP_LOGI(TAG, "\tVibrator voltage preset to 2v");
    }

    if (axp192_bits(AXP_I2C, AXP192_GPIO1_CONTROL, 0x07, 0x00) == ESP_OK &&
        m5core2_led(true) == ESP_OK) {
        ESP_LOGI(TAG, "\tLED on");
    }

    if (axp192_bits(AXP_I2C, AXP192_CHARGE_CONTROL_1, 0x0f, 0x00) == ESP_OK) {
        ESP_LOGI(TAG, "\tCharge current set to 100 mA");
    }

	float volts;
	if (axp192_read(AXP_I2C, AXP192_BATTERY_VOLTAGE, &volts) == ESP_OK) {
		ESP_LOGI(TAG, "\tBattery voltage now: %.2f volts", volts);
    }

    if (axp192_bits(AXP_I2C, AXP192_PEK, 0xff, 0x4c) == ESP_OK) {
    	ESP_LOGI(TAG, "\tPower key set, 4 seconds for hard shutdown");
    }

    if (axp192_bits(AXP_I2C, AXP192_ADC_ENABLE_1, 0x00, 0xff) == ESP_OK) {
    	ESP_LOGI(TAG, "\tEnabled all ADC channels");
    }

    if (m5core2_int_5v(true) == ESP_OK) {
    	ESP_LOGI(TAG, "\tUSB / battery powered");
    }
	
	axp192_bits(AXP_I2C, AXP192_GPIO40_FUNCTION_CONTROL, ~0x72, 0x84);
    axp192_bits(AXP_I2C, AXP192_GPIO40_SIGNAL_STATUS, 0x02, 0x00);
    vTaskDelay(100 / portTICK_RATE_MS);
    if (axp192_bits(AXP_I2C, AXP192_GPIO40_SIGNAL_STATUS, 0x02, 0x02) == ESP_OK) {
    	ESP_LOGI(TAG, "\tLCD and touch reset");
    }
    vTaskDelay(100 / portTICK_RATE_MS);
    
    return ESP_OK;
}

esp_err_t m5core2_int_5v(bool on) {

	// To enable the on-board 5V supply, first N_VBUSEN needs to be pulled
	// high using GPIO0, then we can enable the EXTEN output, to enable
	// the SMPS.
	// To disable it (so either no 5V, or externally supplied 5V), we
	// do the opposite: First disable EXTEN, then leave GPIO0 floating.
	// N_VBUSEN will be pulled down by the on-board resistor.
	// Side note: The pull down is 10k according to the schematic, so that's
	// a 0.5 mA drain from the GPIO0 LDO as long as the bus supply is active.
	
	esp_err_t ret = ESP_OK;

	if (on) {
		ret |= axp192_write_reg(AXP_I2C, AXP192_GPIO0_LDOIO0_VOLTAGE, 0xf0);
		ret |= axp192_write_reg(AXP_I2C, AXP192_GPIO0_CONTROL, (0x02));
		ret |= axp192_set_rail_state(AXP_I2C, AXP192_RAIL_EXTEN, true);
	} else {
		ret |= axp192_set_rail_state(AXP_I2C, AXP192_RAIL_EXTEN, false);
		ret |= axp192_write_reg(AXP_I2C, AXP192_GPIO0_CONTROL, (0x07));
	}
	return ret;
}

esp_err_t m5core2_led(bool on) {
    return axp192_bits(AXP_I2C, AXP192_GPIO20_SIGNAL_STATUS, 0x02, on ? 0x00 : 0x02);
}

esp_err_t m5core2_vibration(bool on) {
	return axp192_set_rail_state(AXP_I2C, AXP192_RAIL_LDO3, on);
}

esp_err_t m5core2_speaker(bool on) {
    return axp192_bits(AXP_I2C, AXP192_GPIO20_SIGNAL_STATUS, 0x04, on ? 0x04 : 0x00);
}
