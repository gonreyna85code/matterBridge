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
using namespace esp_matter::cluster;

static const char *TAG = "WIRED";

// Pines globales
const gpio_num_t PIN_CLK = GPIO_NUM_2;
const gpio_num_t PIN_DT = GPIO_NUM_3;
const gpio_num_t PIN_SW = GPIO_NUM_5;
const gpio_num_t MOSFET_PIN = GPIO_NUM_7;

// Variables internas
static QueueHandle_t s_enc_queue = nullptr;
static uint8_t s_last_level = 128;
static endpoint_t *s_encoder_ep = nullptr;

static volatile int8_t clk_last = 0;
static volatile int8_t dt_last = 0;
static volatile TickType_t sw_last_tick = 0;
static const TickType_t DEBOUNCE_MS = pdMS_TO_TICKS(50);

typedef enum
{
    ENC_EVT_ROT_CW,
    ENC_EVT_ROT_CCW,
    ENC_EVT_SW_PRESS
} enc_event_type_t;

typedef struct
{
    enc_event_type_t type;
} enc_event_t;

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
        int duty = onoff ? (level * 1023) / 254 : 0;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ESP_LOGI(TAG, "set_mosfet level=%d onoff=%d duty=%d", level, onoff, duty);
    }

    uint8_t get_last_level()
    {
        return s_last_level;
    }

    static const char *TAG = "wired_dht";

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
            if (auto attr = esp_matter::attribute::get(inst->cfg.temp_ep_id,
                                                       chip::app::Clusters::TemperatureMeasurement::Id,
                                                       chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id))
            {
                esp_matter_attr_val_t val = esp_matter_int16(static_cast<int16_t>(t * 100));
                esp_matter::attribute::set_val(attr, &val);
                esp_matter::attribute::report(inst->cfg.temp_ep_id,
                                              chip::app::Clusters::TemperatureMeasurement::Id,
                                              chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,
                                              &val);
            }

            // Humidity
            if (auto attr = esp_matter::attribute::get(inst->cfg.hum_ep_id,
                                                       chip::app::Clusters::RelativeHumidityMeasurement::Id,
                                                       chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id))
            {
                esp_matter_attr_val_t val = esp_matter_int16(static_cast<int16_t>(h * 100));
                esp_matter::attribute::set_val(attr, &val);
                esp_matter::attribute::report(inst->cfg.hum_ep_id,
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

// ------------------------------------------------------------------
// Encoder ISR / Task
// ------------------------------------------------------------------
static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    uintptr_t gpio_num = (uintptr_t)arg;
    if (!s_enc_queue)
        return;

    if (gpio_num == PIN_CLK || gpio_num == PIN_DT)
    {
        int clk = gpio_get_level(PIN_CLK);
        int dt = gpio_get_level(PIN_DT);

        if (clk != clk_last)
        {
            enc_event_t ev;
            ev.type = (clk_last == dt) ? ENC_EVT_ROT_CW : ENC_EVT_ROT_CCW;
            BaseType_t hpw = pdFALSE;
            xQueueSendFromISR(s_enc_queue, &ev, &hpw);
            if (hpw)
                portYIELD_FROM_ISR();
        }
        clk_last = clk;
        dt_last = dt;
        return;
    }

    if (gpio_num == PIN_SW)
    {
        TickType_t now = xTaskGetTickCountFromISR();
        if ((now - sw_last_tick) > DEBOUNCE_MS)
        {
            enc_event_t ev = {ENC_EVT_SW_PRESS};
            BaseType_t hpw = pdFALSE;
            xQueueSendFromISR(s_enc_queue, &ev, &hpw);
            if (hpw)
                portYIELD_FROM_ISR();
            sw_last_tick = now;
        }
    }
}

static void encoder_task(void *arg)
{
    if (!s_encoder_ep)
    {
        ESP_LOGE(TAG, "encoder_task: endpoint null");
        vTaskDelete(NULL);
        return;
    }

    enc_event_t ev;
    while (true)
    {
        if (xQueueReceive(s_enc_queue, &ev, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            bool level_changed = false;

            attribute_t *onoff_attr = attribute::get(
                endpoint::get_id(s_encoder_ep),
                chip::app::Clusters::OnOff::Id,
                chip::app::Clusters::OnOff::Attributes::OnOff::Id);

            bool current_onoff = true;
            if (onoff_attr)
            {
                esp_matter_attr_val_t v;
                attribute::get_val(onoff_attr, &v);
                current_onoff = v.val.b;
            }

            switch (ev.type)
            {
            case ENC_EVT_ROT_CW:
                if (s_last_level < 254)
                {
                    s_last_level = std::min<uint8_t>(254, s_last_level + 8);
                    level_changed = true;
                }
                break;
            case ENC_EVT_ROT_CCW:
                if (s_last_level > 0)
                {
                    s_last_level = (s_last_level >= 8) ? s_last_level - 8 : 0;
                    level_changed = true;
                }
                break;
            case ENC_EVT_SW_PRESS:
                if (onoff_attr)
                {
                    esp_matter_attr_val_t val = esp_matter_bool(!current_onoff);
                    attribute::set_val(onoff_attr, &val);
                    attribute::report(endpoint::get_id(s_encoder_ep),
                                      chip::app::Clusters::OnOff::Id,
                                      chip::app::Clusters::OnOff::Attributes::OnOff::Id,
                                      &val);
                    current_onoff = val.val.b;
                    ESP_LOGI(TAG, "Botón -> nuevo onoff=%d", current_onoff);
                    wired::set_mosfet(s_last_level, current_onoff);
                }
                break;
            }

            if (level_changed)
            {
                attribute_t *level_attr = attribute::get(
                    endpoint::get_id(s_encoder_ep),
                    chip::app::Clusters::LevelControl::Id,
                    chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id);
                if (level_attr)
                {
                    esp_matter_attr_val_t val = esp_matter_uint8(s_last_level);
                    attribute::set_val(level_attr, &val);
                    attribute::report(endpoint::get_id(s_encoder_ep),
                                      chip::app::Clusters::LevelControl::Id,
                                      chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id,
                                      &val);
                    ESP_LOGI(TAG, "Encoder nivel -> %d", s_last_level);
                }

                if (onoff_attr && s_last_level == 0 && current_onoff)
                {
                    esp_matter_attr_val_t val = esp_matter_bool(false);
                    attribute::set_val(onoff_attr, &val);
                    attribute::report(endpoint::get_id(s_encoder_ep),
                                      chip::app::Clusters::OnOff::Id,
                                      chip::app::Clusters::OnOff::Attributes::OnOff::Id,
                                      &val);
                    ESP_LOGI(TAG, "Auto-off por nivel 0");
                    wired::set_mosfet(s_last_level, false);
                }
                else
                {
                    wired::set_mosfet(s_last_level, current_onoff);
                }
            }
        }
    }
}

// --------------------- Handler de updates de atributos ---------------------
esp_err_t wired::handle_attribute_update(uint16_t endpoint_id,
                                 uint32_t cluster_id,
                                 uint32_t attribute_id,
                                 esp_matter_attr_val_t *val)
{
    esp_err_t ret = ESP_OK; 
    if (!s_encoder_ep || endpoint_id != endpoint::get_id(s_encoder_ep))
        return ESP_ERR_NOT_FOUND;
    if (cluster_id == chip::app::Clusters::OnOff::Id &&
        attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id &&
        val->type == ESP_MATTER_VAL_TYPE_BOOLEAN)
    {
        bool onoff = val->val.b;
        ESP_LOGI(TAG, "OnOff update -> %d", onoff);
        wired::set_mosfet(s_last_level, onoff);
    }
    else if (cluster_id == chip::app::Clusters::LevelControl::Id &&
             attribute_id == chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id &&
             val->type == ESP_MATTER_VAL_TYPE_UINT8)
    {
        uint8_t level = std::min<uint8_t>(254, val->val.u8);
        if (level != s_last_level)
        {
            s_last_level = level;
            ESP_LOGI(TAG, "CurrentLevel update -> %d", level);
            attribute_t *onoff_attr = attribute::get(
                endpoint::get_id(s_encoder_ep),
                chip::app::Clusters::OnOff::Id,
                chip::app::Clusters::OnOff::Attributes::OnOff::Id);
            bool current_onoff = true;
            if (onoff_attr)
            {
                esp_matter_attr_val_t v;
                attribute::get_val(onoff_attr, &v);
                current_onoff = v.val.b;
            }
            if (onoff_attr && level == 0 && current_onoff)
            {
                esp_matter_attr_val_t v = esp_matter_bool(false);
                attribute::set_val(onoff_attr, &v);
                attribute::report(endpoint::get_id(s_encoder_ep),
                                  chip::app::Clusters::OnOff::Id,
                                  chip::app::Clusters::OnOff::Attributes::OnOff::Id,
                                  &v);
                ESP_LOGI(TAG, "Auto-off por nivel 0");
                wired::set_mosfet(s_last_level, false);
            }
            else
            {
                wired::set_mosfet(s_last_level, current_onoff);
            }
        }
    }
    else
    {
        ret = ESP_ERR_NOT_SUPPORTED;
    }
    return ret;
}

// ------------------------------------------------------------------
// Inicialización y API pública
// ------------------------------------------------------------------
void wired::encoder_init_pins()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << PIN_CLK) | (1ULL << PIN_DT) | (1ULL << PIN_SW);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

void wired::start_encoder(esp_matter::endpoint_t *encoder_ep)
{
    if (!encoder_ep)
    {
        ESP_LOGE(TAG, "start_encoder: endpoint null");
        return;
    }

    s_encoder_ep = encoder_ep;

    if (!s_enc_queue)
    {
        s_enc_queue = xQueueCreate(16, sizeof(enc_event_t));
        if (!s_enc_queue)
        {
            ESP_LOGE(TAG, "No se pudo crear cola enc");
            return;
        }
    }

    encoder_init_pins();
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_CLK, encoder_isr_handler, (void *)PIN_CLK);
    gpio_isr_handler_add(PIN_DT, encoder_isr_handler, (void *)PIN_DT);
    gpio_isr_handler_add(PIN_SW, encoder_isr_handler, (void *)PIN_SW);

    xTaskCreate(encoder_task, "encoder_task", 4096, nullptr, 5, NULL);
    ESP_LOGI(TAG, "encoder iniciado");
}
