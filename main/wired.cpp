#include "wired.h"
#include <esp_log.h>
#include <freertos/task.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <esp_matter.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "WIRED";

// Pines y variables globales
const gpio_num_t MOSFET_PIN = GPIO_NUM_7;
static bool g_onoff = false;
static uint8_t g_level = 128;
static uint8_t last_nonzero_level = 128;
static esp_matter::endpoint_t *temp_ep_global = nullptr;
static esp_matter::endpoint_t *hum_ep_global = nullptr;
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

    uint8_t get_last_level() { return g_level; }

// ------------------------------------------------------------------
// I2C / ATH10
// ------------------------------------------------------------------
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_1
#define I2C_MASTER_SCL_IO GPIO_NUM_2
#define I2C_MASTER_FREQ_HZ 100000
#define ATH10_ADDR 0x38

    static void i2c_init()
    {
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = I2C_MASTER_SDA_IO;
        conf.scl_io_num = I2C_MASTER_SCL_IO;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    }

    static esp_err_t read_aht10(float *temp, float *hum)
    {
        uint8_t cmd1 = 0x00; // comando de medición según datasheet ATH10

        // Enviar comando de medición
        esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, ATH10_ADDR, &cmd1, 1, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "ATH10 write command failed");
            return ret;
        }

        // Esperar conversión (datasheet indica ~80ms mínimo)
        vTaskDelay(pdMS_TO_TICKS(100));

        // Leer 4 bytes: 2 humedad, 2 temperatura
        uint8_t buf[6];
        uint8_t cmd[3] = {0xAC, 0x33, 0x00};
        i2c_master_write_to_device(I2C_MASTER_NUM, ATH10_ADDR, cmd, 3, 1000 / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(80)); // esperar medición
        i2c_master_read_from_device(I2C_MASTER_NUM, ATH10_ADDR, buf, 6, 1000 / portTICK_PERIOD_MS);

        uint32_t raw_hum = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((buf[3] & 0xF0) >> 4);
        uint32_t raw_temp = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

        *hum = raw_hum / 1048576.0f * 100.0f;
        *temp = raw_temp / 1048576.0f * 200.0f - 50.0f;
        return ESP_OK;
    }

    struct aht10_instance_t
    {
        esp_timer_handle_t timer;
    };

    static void aht10_timer_cb(void *arg)
    {
        float t, h;
        if (read_aht10(&t, &h) == ESP_OK)
        {
            ESP_LOGI(TAG, "ATH10 -> Temp: %.1f°C Hum: %.1f%%", t, h);

            // Temperature

            if (auto attr = attribute::get(endpoint::get_id(temp_ep_global), TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id))
            {
                esp_matter_attr_val_t val = esp_matter_int16(static_cast<int16_t>(t * 100));
                attribute::set_val(attr, &val);
                attribute::report(endpoint::get_id(temp_ep_global), TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);
            }

            // Humidity
            if (auto attr = attribute::get(endpoint::get_id(hum_ep_global), RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id))
            {
                esp_matter_attr_val_t val = esp_matter_int16(static_cast<int16_t>(h * 100));
                attribute::set_val(attr, &val);
                attribute::report(endpoint::get_id(hum_ep_global), RelativeHumidityMeasurement::Id, RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, &val);
            }
        }
        else
        {
            ESP_LOGW(TAG, "ATH10 read failed");
        }
    }

    void start_aht10(esp_matter::endpoint_t *temp_ep, esp_matter::endpoint_t *hum_ep)
    {
        temp_ep_global = temp_ep;
        hum_ep_global = hum_ep;
        i2c_init();
        aht10_instance_t *inst = new aht10_instance_t;

        esp_timer_create_args_t t_args = {};
        t_args.callback = aht10_timer_cb;
        t_args.arg = inst;
        t_args.name = "aht10_timer";

        esp_timer_create(&t_args, &inst->timer);
        esp_timer_start_periodic(inst->timer, ATH10_READ_INTERVAL_MS * 1000); // µs
    }
}

// ------------------------------------------------------------------
// Handler de updates de atributos
// ------------------------------------------------------------------
esp_err_t wired::handle_attribute_update(uint16_t endpoint_id,
                                         uint32_t cluster_id,
                                         uint32_t attribute_id,
                                         esp_matter_attr_val_t *val)
{
    if (!val)
        return ESP_ERR_INVALID_ARG;

    bool update_needed = false;

    // --- On/Off cluster ---
    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id)
    {
        g_onoff = val->val.b;
        if (g_onoff && g_level == 0)
            g_level = last_nonzero_level;
        update_needed = true;
    }
    // --- LevelControl cluster ---
    else if (cluster_id == LevelControl::Id && attribute_id == LevelControl::Attributes::CurrentLevel::Id)
    {
        uint8_t new_level = val->val.u8;
        if (!(g_onoff && new_level == 1 && g_level > 1))
        {
            g_level = new_level;
            if (g_level > 0)
                last_nonzero_level = g_level;
            update_needed = true;
        }
    }

    if (update_needed)
    {
        wired::set_mosfet(g_level, g_onoff);
        ESP_LOGI(TAG, "Update -> OnOff=%d, Level=%d", g_onoff, g_level);
    }

    return ESP_OK;
}
