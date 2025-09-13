#pragma once
#include <driver/gpio.h>
#include <esp_matter.h>
#include <ctime>
#include <string>
#include <map>

extern const gpio_num_t PIN_CLK;
extern const gpio_num_t PIN_DT;
extern const gpio_num_t PIN_SW;
extern const gpio_num_t MOSFET_PIN;

namespace wired
{
    void pwm_init();
    void set_mosfet(uint8_t level, bool onoff);
    uint8_t get_last_level();
    void encoder_init_pins();
    void start_encoder(esp_matter::endpoint_t *encoder_ep);

    constexpr auto DHT_READ_INTERVAL_MS = 30000;
    enum class dht_type_t
    {
        DHT11,
        DHT22
    };
    struct dht_config_t
    {
        gpio_num_t pin;
        dht_type_t type;
        uint16_t temp_ep_id;
        uint16_t hum_ep_id;
        uint32_t read_interval_ms = DHT_READ_INTERVAL_MS;
    };
    void start_dht(const dht_config_t *cfg);

    esp_err_t handle_attribute_update(uint16_t endpoint_id,
                                 uint32_t cluster_id,
                                 uint32_t attribute_id,
                                 esp_matter_attr_val_t *val);
}
