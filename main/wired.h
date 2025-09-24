#pragma once
#include <driver/gpio.h>
#include <esp_matter.h>
#include <esp_err.h>
#include <ctime>

extern const gpio_num_t MOSFET_PIN;

namespace wired
{
    void pwm_init();
    void set_mosfet(uint8_t level, bool onoff);
    uint8_t get_last_level();

    // ATH10 I2C
    constexpr auto ATH10_READ_INTERVAL_MS = 30000;
    void start_aht10(esp_matter::endpoint_t *temp_ep, esp_matter::endpoint_t *hum_ep);

    esp_err_t handle_attribute_update(uint16_t endpoint_id,
                                      uint32_t cluster_id,
                                      uint32_t attribute_id,
                                      esp_matter_attr_val_t *val);
}
