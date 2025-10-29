#include <esp_err.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <map>
#include <string>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_console_bridge.h>
#include <common_macros.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <inttypes.h>
#include <wired.h>
#include <bridge.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <webserver.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace bridge;
using namespace wired;

static const char *TAG = "app_main";
static endpoint_t *led_ep_global = nullptr;
static endpoint_t *activator = nullptr;
static bool activator_state = false;
static bool initialized = false;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type)
    {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address Changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;
    default:
        break;
    }
}

static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == PRE_UPDATE)
    {
        ESP_LOGI(TAG, "Pre update → Endpoint=0x%X, Cluster=0x%lX, Attribute=0x%lX",
                 endpoint_id, cluster_id, attribute_id);
    }
    uint16_t encoder_id = endpoint::get_id(led_ep_global);
    if (endpoint_id == encoder_id)
    {
        ESP_LOGI(TAG, "→ Dispatch to wired handler");
        wired::handle_attribute_update(endpoint_id, cluster_id, attribute_id, val);
        return ESP_OK;
    }
    uint16_t act_id = endpoint::get_id(activator);
    if (endpoint_id == act_id && type == PRE_UPDATE)
    {
        if (!initialized)
        {
            ESP_LOGI(TAG, "Activator update ignored: not initialized yet");
            return ESP_OK;
        }

        bool new_state = val->val.b;
        if (new_state == activator_state)
        {
            ESP_LOGI(TAG, "No state change (%.1f → %.1f), ignoring", (float)activator_state, (float)new_state);
            return ESP_OK;
        }

        activator_state = new_state;
        ESP_LOGI(TAG, "→ Toggling bridge activation to %s", activator_state ? "ON" : "OFF");

        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("bridge_nvs", NVS_READWRITE, &nvs_handle);
        if (err == ESP_OK)
        {
            nvs_set_u8(nvs_handle, "bridge_active", activator_state ? 1 : 0);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            ESP_LOGI(TAG, "Restarting to apply changes...");
            esp_rom_delay_us(500000);
            esp_restart();
        }
        else
        {
            ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        }
        return ESP_OK;
    }    
    else
    {
        ESP_LOGI(TAG, "→ Dispatch to bridge handler");
        bridge::handle_attribute_update(endpoint_id, cluster_id, attribute_id, val);
        return ESP_OK;
    }
}

extern "C" void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t nvs_handle;
    err = nvs_open("bridge_nvs", NVS_READWRITE, &nvs_handle);
    uint8_t bridge_active = 1;
    if (err == ESP_OK)
    {
        esp_err_t res = nvs_get_u8(nvs_handle, "bridge_active", &bridge_active);
        if (res == ESP_ERR_NVS_NOT_FOUND)
        {
            nvs_set_u8(nvs_handle, "bridge_active", bridge_active);
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    activator_state = (bridge_active != 0);
    ESP_LOGI(TAG, "Bridge active (from NVS): %s", activator_state ? "ON" : "OFF");
    // --- Init de tus módulos ---
    wired::pwm_init();
    devices::init_types();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Aggregator
    aggregator::config_t aggregator_config;
    endpoint_t *aggregator = endpoint::aggregator::create(node, &aggregator_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(aggregator != nullptr, ESP_LOGE(TAG, "Failed to create aggregator endpoint"));

    temperature_sensor::config_t temp_sensor_config;
    endpoint_t *temp_ep = temperature_sensor::create(node, &temp_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    // Cluster BridgedDeviceBasicInformation
    cluster::bridged_device_basic_information::config_t basic_info_cfg{};
    cluster_t *tem_cl = cluster::bridged_device_basic_information::create(temp_ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);
    cluster::bridged_device_basic_information::attribute::create_product_name(
        tem_cl,
        (char *)"Temperature",
        strlen("Temperature"));
    uint16_t ep_id = endpoint::get_id(temp_ep);
    attribute_t *node_label_attr = attribute::get(
        ep_id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    if (node_label_attr)
    {
        esp_matter_attr_val_t val0 = esp_matter_char_str("temperature", strlen("temperature"));
        attribute::set_val(node_label_attr, &val0);
        attribute::report(ep_id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                          &val0);
    }
    ABORT_APP_ON_FAILURE(temp_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));

    // Humidity sensor
    humidity_sensor::config_t humidity_sensor_config;
    endpoint_t *hum_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    // Cluster BridgedDeviceBasicInformation
    cluster::bridged_device_basic_information::config_t basic_humidity_cfg{};
    cluster_t *hum_cl = cluster::bridged_device_basic_information::create(hum_ep, &basic_humidity_cfg, CLUSTER_FLAG_SERVER);
    cluster::bridged_device_basic_information::attribute::create_product_name(
        hum_cl,
        (char *)"Humidity",
        strlen("Humidity"));
    uint16_t ep_id2 = endpoint::get_id(hum_ep);
    attribute_t *node_label_attr2 = attribute::get(
        ep_id2,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    if (node_label_attr2)
    {
        esp_matter_attr_val_t val1 = esp_matter_char_str("humidity", strlen("humidity"));
        attribute::set_val(node_label_attr2, &val1);
        attribute::report(ep_id2,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                          &val1);
    }

    ABORT_APP_ON_FAILURE(hum_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));

    wired::start_aht10(temp_ep, hum_ep);

    // Led endpoint (remote dimmer switch)
    dimmable_light::config_t bridge_config{};
    endpoint_t *epl = dimmable_light::create(node, &bridge_config, ENDPOINT_FLAG_NONE, nullptr);
    cluster::bridged_device_basic_information::config_t basic_info3_cfg{};
    cluster_t *led_cl = cluster::bridged_device_basic_information::create(epl, &basic_info3_cfg, CLUSTER_FLAG_SERVER);
    cluster::bridged_device_basic_information::attribute::create_product_name(
        led_cl,
        (char *)"Dimmable Light",
        strlen("Dimmable Light"));
    led_ep_global = epl;
    uint16_t ep_id3 = endpoint::get_id(epl);
    attribute_t *node_label_attr3 = attribute::get(
        ep_id3,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    if (node_label_attr3)
    {
        esp_matter_attr_val_t val2 = esp_matter_char_str("dimmable_light", strlen("dimmable_light"));
        attribute::set_val(node_label_attr3, &val2);
        attribute::report(ep_id3,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                          &val2);
    }
    ABORT_APP_ON_FAILURE(epl != nullptr, ESP_LOGE(TAG, "Failed to create led endpoint"));

    // switch for activate the bridge
    on_off_plugin_unit::config_t switch_config;
    endpoint_t *SW1_ep = on_off_plugin_unit::create(node, &switch_config, ENDPOINT_FLAG_NONE, NULL);
    cluster::bridged_device_basic_information::config_t basic_info_cfg1{};
    cluster_t *SW1 = cluster::bridged_device_basic_information::create(SW1_ep, &basic_info_cfg1, CLUSTER_FLAG_SERVER);
    cluster::bridged_device_basic_information::attribute::create_product_name(
        SW1,
        (char *)"AntiMatter",
        strlen("AntiMatter"));
    uint16_t ep_id1 = endpoint::get_id(SW1_ep);
    activator = SW1_ep;
    attribute_t *node_label_attr1 = attribute::get(
        ep_id1,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    if (node_label_attr1)
    {
        esp_matter_attr_val_t val = esp_matter_char_str("bridge_switch", strlen("bridge_switch"));
        attribute::set_val(node_label_attr1, &val);
        attribute::report(ep_id1,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                          &val);
    }
    ABORT_APP_ON_FAILURE(SW1_ep != nullptr, ESP_LOGE(TAG, "Failed to create switch endpoint"));

    /* Load devices from NVS */
    devices::load_devices_from_nvs(node);

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    esp_matter_attr_val_t val = esp_matter_bool(activator_state);
    attribute::set_val(attribute::get(
                           endpoint::get_id(activator),
                           chip::app::Clusters::OnOff::Id,
                           chip::app::Clusters::OnOff::Attributes::OnOff::Id),
                       &val);
    attribute::report(
        endpoint::get_id(activator),
        chip::app::Clusters::OnOff::Id,
        chip::app::Clusters::OnOff::Attributes::OnOff::Id,
        &val);

    initialized = true;

    // === Iniciar bridge si está activo ===
    if (activator_state)
    {
        bridge::init(node);
    }
}