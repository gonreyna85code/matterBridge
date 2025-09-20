#include <wired.h>
#include <dht.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <algorithm>
#include <esp_matter.h>
#include <driver/ledc.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "WIRED";

// Pines globales
const gpio_num_t MOSFET_PIN = GPIO_NUM_7;

// Variables internas
static bool g_onoff = false;
static uint8_t g_level = 128;
static uint8_t last_nonzero_level = 128;

// ------------------------------------------------------------------
// PWM / MOSFET
// ------------------------------------------------------------------
namespace wired
{
    void pwm_init()
    {
        ledc_timer_config_t timer = {};
        timer.duty_resolution = LEDC_TIMER_10_BIT;
        timer.freq_hz = 1000;
        timer.speed_mode = LEDC_LOW_SPEED_MODE;
        timer.timer_num = LEDC_TIMER_0;
        ledc_timer_config(&timer);

        ledc_channel_config_t channel = {};
        channel.channel = LEDC_CHANNEL_0;
        channel.duty = 0;
        channel.gpio_num = MOSFET_PIN;
        channel.speed_mode = LEDC_LOW_SPEED_MODE;
        channel.timer_sel = LEDC_TIMER_0;
        ledc_channel_config(&channel);
    }

    void set_mosfet(uint8_t level, bool onoff)
    {
        static int last_duty = -1;
        int duty = onoff ? (level * 1023) / 254 : 0;

        if (duty != last_duty)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ESP_LOGI(TAG, "set_mosfet level=%d onoff=%d duty=%d", level, onoff, duty);
            last_duty = duty;
        }
    }

    uint8_t get_last_level()
    {
        return g_level;
    }

    struct dht_instance_t
    {
        dht_config_t cfg;
        esp_timer_handle_t timer;
    };

    static void dht_timer_cb(void *arg)
    {
        dht_instance_t *inst = (dht_instance_t *)arg;
        float t, h;

        if (dht_read_float_data(DHT_TYPE_DHT11, inst->cfg.pin, &h, &t) == ESP_OK)
        {
            ESP_LOGI(TAG, "Hum: %.1f%% Temp: %.1f°C", h, t);

            // Temperature
            if (auto attr = attribute::get(inst->cfg.temp_ep_id,
                                           chip::app::Clusters::TemperatureMeasurement::Id,
                                           chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id))
            {
                esp_matter_attr_val_t val = esp_matter_int16(static_cast<int16_t>(t * 100));
                attribute::set_val(attr, &val);
                attribute::report(inst->cfg.temp_ep_id,
                                  chip::app::Clusters::TemperatureMeasurement::Id,
                                  chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,
                                  &val);
            }

            // Humidity
            if (auto attr = attribute::get(inst->cfg.hum_ep_id,
                                           chip::app::Clusters::RelativeHumidityMeasurement::Id,
                                           chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id))
            {
                esp_matter_attr_val_t val = esp_matter_int16(static_cast<int16_t>(h * 100));
                attribute::set_val(attr, &val);
                attribute::report(inst->cfg.hum_ep_id,
                                  chip::app::Clusters::RelativeHumidityMeasurement::Id,
                                  chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                                  &val);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Failed to read DHT");
        }
    }

    void start_dht(const dht_config_t *cfg)
    {
        dht_instance_t *inst = new dht_instance_t;
        inst->cfg = *cfg;

        gpio_set_pull_mode(cfg->pin, GPIO_PULLUP_ONLY);

        esp_timer_create_args_t t_args = {};
        t_args.callback = dht_timer_cb;
        t_args.arg = inst;
        t_args.name = "dht_timer";

        esp_timer_create(&t_args, &inst->timer);
        esp_timer_start_periodic(inst->timer, cfg->read_interval_ms * 1000); // µs
    }
}

// --------------------- Handler de updates de atributos ---------------------
esp_err_t wired::handle_attribute_update(uint16_t endpoint_id,
                                         uint32_t cluster_id,
                                         uint32_t attribute_id,
                                         esp_matter_attr_val_t *val)
{
    if (!val) return ESP_ERR_INVALID_ARG;

    bool update_needed = false;

    // --- On/Off cluster ---
    if (cluster_id == chip::app::Clusters::OnOff::Id &&
        attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id)
    {
        g_onoff = val->val.b;

        // Si encendemos y el nivel actual es 0, restauramos el último nivel válido
        if (g_onoff && g_level == 0)
            g_level = last_nonzero_level;

        update_needed = true;
    }

    // --- LevelControl cluster ---
    else if (cluster_id == chip::app::Clusters::LevelControl::Id &&
             attribute_id == chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id)
    {
        uint8_t new_level = val->val.u8;

        // Ignoramos cambios “fantasmas” de nivel 1 cuando ya estamos On
        if (g_onoff && new_level == 1 && g_level > 1) {
            // descartamos este nivel intermedio
        } else {
            g_level = new_level;
            if (g_level > 0) last_nonzero_level = g_level;
            update_needed = true;
        }
    }

    // --- Aplicar update al MOSFET ---
    if (update_needed) {
        wired::set_mosfet(g_level, g_onoff);
        ESP_LOGI(TAG, "Update -> OnOff=%d, Level=%d", g_onoff, g_level);
    }

    return ESP_OK;
}