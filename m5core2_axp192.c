#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "axp192.h"
#include "i2c_manager.h"

#include "m5core2_axp192.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>


#define AXP_I2C	i2c_hal(CONFIG_M5CORE2_I2C_INTERNAL)

static const char* TAG = "m5core2_axp192";



esp_err_t m5core2_init() {

    ESP_LOGI(TAG, "Initialising");

    // turn off everything except bit 2 and then turn bit 1 on
    if (m5core2_axp_twiddle(AXP192_VBUS_IPSOUT_CHANNEL, 0b11111011, 0x02) == ESP_OK) {
        ESP_LOGI(TAG, "\tVbus limit off");
    }

    if (m5core2_axp_twiddle(AXP192_GPIO2_CONTROL, 0b00000111, 0x00) == ESP_OK &&
        m5core2_speaker(false) == ESP_OK) {
        ESP_LOGI(TAG, "\tSpeaker amplifier off");
    }

    if (m5core2_axp_twiddle(AXP192_BATTERY_CHARGE_CONTROL, 0b11100011, 0b10100010) == ESP_OK) {
        ESP_LOGI(TAG, "\tRTC battery charging enabled (3v, 200uA)");
    }

    if (m5core2_set_rail_mv(ESP_POWER, 3350) == ESP_OK &&
        m5core2_set_rail_state(ESP_POWER, true) == ESP_OK) {
        ESP_LOGI(TAG, "\tESP32 power voltage set to 3.35v");
    }

    if (m5core2_set_rail_mv(LCD_BACKLIGHT, 2800) == ESP_OK &&
        m5core2_set_rail_state(LCD_BACKLIGHT, true) == ESP_OK) {
        ESP_LOGI(TAG, "\tLCD backlight voltage set to 2.80v");
    }

    if (m5core2_set_rail_mv(LOGIC_AND_SD, 3300) == ESP_OK &&
        m5core2_set_rail_state(LOGIC_AND_SD, true) == ESP_OK) {
        ESP_LOGI(TAG, "\tLCD logic and sdcard voltage set to 3.3v");
    }

    if (m5core2_set_rail_mv(VIBRATOR, 2000) == ESP_OK) {
        ESP_LOGI(TAG, "\tVibrator voltage preset to 2v");
    }

    if (m5core2_axp_twiddle(AXP192_GPIO1_CONTROL, 0x07, 0x00) == ESP_OK &&
        m5core2_led(true) == ESP_OK) {
        ESP_LOGI(TAG, "\tLED on");
    }

    if (m5core2_axp_twiddle(AXP192_CHARGE_CONTROL_1, 0x0f, 0x00) == ESP_OK) {
        ESP_LOGI(TAG, "\tCharge current set to 100 mA");
    }

	float volts;
	if (m5core2_axp_read(AXP192_BATTERY_VOLTAGE, &volts) == ESP_OK) {
		ESP_LOGI(TAG, "\tBattery voltage now: %.2f volts", volts);
    }

    if (m5core2_axp_twiddle(AXP192_PEK, 0xff, 0x4c) == ESP_OK) {
    	ESP_LOGI(TAG, "\tPower key set, 4 seconds for hard shutdown");
    }

    if (m5core2_axp_twiddle(AXP192_ADC_ENABLE_1, 0x00, 0xff) == ESP_OK) {
    	ESP_LOGI(TAG, "\tEnabled all ADC channels");
    }

    if (m5core2_int_5v(true) == ESP_OK) {
    	ESP_LOGI(TAG, "\tUSB / battery powered, 5V bus on");
    }
	
	// GPIO4 is reset for LCD and touch
	m5core2_axp_twiddle(AXP192_GPIO40_FUNCTION_CONTROL, ~0x72, 0x84);
    m5core2_axp_twiddle(AXP192_GPIO40_SIGNAL_STATUS, 0x02, 0x00);
    vTaskDelay(100 / portTICK_RATE_MS);
    if (m5core2_axp_twiddle(AXP192_GPIO40_SIGNAL_STATUS, 0x02, 0x02) == ESP_OK) {
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
		ret |= m5core2_axp_twiddle(AXP192_GPIO0_LDOIO0_VOLTAGE, 0xf0, 0xf0);
		ret |= m5core2_axp_twiddle(AXP192_GPIO0_CONTROL, 0x07, 0x02);
		ret |= m5core2_set_rail_state(AXP192_RAIL_EXTEN, true);
	} else {
		ret |= m5core2_set_rail_state(AXP192_RAIL_EXTEN, false);
		ret |= m5core2_axp_twiddle(AXP192_GPIO0_CONTROL, 0x07, 0x01);
	}
	return ret;
}

esp_err_t m5core2_led(bool on) {
    return m5core2_axp_twiddle(AXP192_GPIO20_SIGNAL_STATUS, 0x02, on ? 0x00 : 0x02);
}

esp_err_t m5core2_vibration(bool on) {
	return m5core2_set_rail_state(AXP192_RAIL_LDO3, on);
}

esp_err_t m5core2_speaker(bool on) {
    return m5core2_axp_twiddle(AXP192_GPIO20_SIGNAL_STATUS, 0x04, on ? 0x04 : 0x00);
}


esp_err_t m5core2_axp_read_reg(uint8_t reg, uint8_t *buffer) {
	axp192_t* ptr = (axp192_t*)AXP_I2C;
	return ptr->read(ptr->handle, AXP192_ADDRESS, reg, buffer, 1);
}

esp_err_t m5core2_axp_write_reg(uint8_t reg, uint8_t value) {
	axp192_t* ptr = (axp192_t*)AXP_I2C;
	uint8_t buffer = value;
	return ptr->write(ptr->handle, AXP192_ADDRESS, reg, &buffer, 1);
}

esp_err_t m5core2_axp_read(uint8_t reg, float *buffer) {
	axp192_t* ptr = (axp192_t*)AXP_I2C;
	return axp192_read(ptr, reg, buffer);
}

esp_err_t m5core2_axp_twiddle(uint8_t reg, uint8_t affect, uint8_t value) {
	esp_err_t ret;
	uint8_t buffer;
	ret = m5core2_axp_read_reg(reg, &buffer);
	if (ret == ESP_OK) {
		buffer &= ~affect;
		buffer |= (value & affect);
		ret = m5core2_axp_write_reg(reg, buffer);
	}
	return ret;
}

typedef struct {
    uint16_t min_millivolts;
    uint16_t max_millivolts;
    uint16_t step_millivolts;
    uint8_t voltage_reg;
    uint8_t voltage_lsb;
    uint8_t voltage_mask;
} axp192_rail_cfg_t;

const axp192_rail_cfg_t axp192_rail_configs[] = {
    [AXP192_RAIL_DCDC1] = {
        .min_millivolts = 700,
        .max_millivolts = 3500,
        .step_millivolts = 25,
        .voltage_reg = AXP192_DCDC1_VOLTAGE,
        .voltage_lsb = 0,
        .voltage_mask = (1 << 7) - 1,
    },
    [AXP192_RAIL_DCDC2] = {
        .min_millivolts = 700,
        .max_millivolts = 2275,
        .step_millivolts = 25,
        .voltage_reg = AXP192_DCDC2_VOLTAGE,
        .voltage_lsb = 0,
        .voltage_mask = (1 << 6) - 1,
    },
    [AXP192_RAIL_DCDC3] = {
        .min_millivolts = 700,
        .max_millivolts = 3500,
        .step_millivolts = 25,
        .voltage_reg = AXP192_DCDC3_VOLTAGE,
        .voltage_lsb = 0,
        .voltage_mask = (1 << 7) - 1,
    },
    [AXP192_RAIL_LDO2] = {
        .min_millivolts = 1800,
        .max_millivolts = 3300,
        .step_millivolts = 100,
        .voltage_reg = AXP192_LDO23_VOLTAGE,
        .voltage_lsb = 4,
        .voltage_mask = 0xf0,
    },
    [AXP192_RAIL_LDO3] = {
        .min_millivolts = 1800,
        .max_millivolts = 3300,
        .step_millivolts = 100,
        .voltage_reg = AXP192_LDO23_VOLTAGE,
        .voltage_lsb = 0,
        .voltage_mask = 0x0f,
    },
};

esp_err_t m5core2_get_rail_state(axp192_rail_t rail, bool *enabled)
{
    esp_err_t ret;
    uint8_t val;

    ret = m5core2_axp_read_reg(AXP192_DCDC13_LDO23_CONTROL, &val);
    if (ret != ESP_OK) {
        return ret;
    }

    switch (rail) {
        case AXP192_RAIL_DCDC1:
            *enabled = !!(val & (1 << 0));
            break;
        case AXP192_RAIL_DCDC2:
            *enabled = !!(val & (1 << 4));
            break;
        case AXP192_RAIL_DCDC3:
            *enabled = !!(val & (1 << 1));
            break;
        case AXP192_RAIL_LDO2:
            *enabled = !!(val & (1 << 2));
            break;
        case AXP192_RAIL_LDO3:
            *enabled = !!(val & (1 << 3));
            break;
        case AXP192_RAIL_EXTEN:
            *enabled = !!(val & (1 << 6));
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t m5core2_set_rail_state(axp192_rail_t rail, bool enabled)
{
    esp_err_t ret;
    uint8_t val;
    uint8_t mask;

    ret = m5core2_axp_read_reg(AXP192_DCDC13_LDO23_CONTROL, &val);
    if (ret != ESP_OK) {
        return ret;
    }

    switch (rail) {
        case AXP192_RAIL_DCDC1:
            mask = (1 << 0);
            break;
        case AXP192_RAIL_DCDC2:
            mask = (1 << 4);
            break;
        case AXP192_RAIL_DCDC3:
            mask = (1 << 1);
            break;
        case AXP192_RAIL_LDO2:
            mask = (1 << 2);
            break;
        case AXP192_RAIL_LDO3:
            mask = (1 << 3);
            break;
        case AXP192_RAIL_EXTEN:
            mask = (1 << 6);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (enabled) {
        val |= mask;
    } else {
        val = val & ~mask;
    }

    ret = m5core2_axp_write_reg(AXP192_DCDC13_LDO23_CONTROL, val);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t m5core2_get_rail_mv(axp192_rail_t rail, uint16_t *millivolts)
{
    esp_err_t ret;
    uint8_t val;

    if ((rail < AXP192_RAIL_DCDC1) || (rail >= AXP192_RAIL_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    const axp192_rail_cfg_t *cfg = &axp192_rail_configs[rail];
    if (cfg->step_millivolts == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = m5core2_axp_read_reg(cfg->voltage_reg, &val);
    if (ret != ESP_OK) {
        return ret;
    }

    val = (val & cfg->voltage_mask) >> cfg->voltage_lsb;

    *millivolts = cfg->min_millivolts + cfg->step_millivolts * val;

    return ESP_OK;
}

esp_err_t m5core2_set_rail_mv(axp192_rail_t rail, uint16_t millivolts)
{

    esp_err_t ret;
    uint8_t val, steps;

    if ((rail < AXP192_RAIL_DCDC1) || (rail >= AXP192_RAIL_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }

    const axp192_rail_cfg_t *cfg = &axp192_rail_configs[rail];
    if (cfg->step_millivolts == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if ((millivolts < cfg->min_millivolts) || (millivolts > cfg->max_millivolts)) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = m5core2_axp_read_reg(cfg->voltage_reg, &val);
    if (ret != ESP_OK) {
        return ret;
    }

    steps = (millivolts - cfg->min_millivolts) / cfg->step_millivolts;
    val = (val & ~(cfg->voltage_mask)) | (steps << cfg->voltage_lsb);

    ret = m5core2_axp_write_reg(cfg->voltage_reg, val);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}
