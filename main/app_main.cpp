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
#include <webserver.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace bridge;
using namespace wired;

static const char *TAG = "app_main";
static endpoint_t *led_ep_global = nullptr;

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
    if (type == PRE_UPDATE) {
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
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
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
    cluster::bridged_device_basic_information::create(temp_ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);
    uint16_t ep_id = endpoint::get_id(temp_ep);
    attribute_t *node_label_attr = attribute::get(
        ep_id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    if (node_label_attr)
    {
        esp_matter_attr_val_t val = esp_matter_char_str("temperature", strlen("temperature"));
        attribute::set_val(node_label_attr, &val);
        attribute::report(ep_id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                          &val);
    }
    ABORT_APP_ON_FAILURE(temp_ep != nullptr, ESP_LOGE(TAG, "Failed to create temperature_sensor endpoint"));

    // Humidity sensor
    humidity_sensor::config_t humidity_sensor_config;
    endpoint_t *hum_ep = humidity_sensor::create(node, &humidity_sensor_config, ENDPOINT_FLAG_NONE, NULL);
    // Cluster BridgedDeviceBasicInformation
    cluster::bridged_device_basic_information::config_t basic_info2_cfg{};
    cluster::bridged_device_basic_information::create(hum_ep, &basic_info2_cfg, CLUSTER_FLAG_SERVER);
    uint16_t ep_id2 = endpoint::get_id(hum_ep);
    
    attribute_t *node_label_attr2 = attribute::get(
        ep_id2,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
    if (node_label_attr2)
    {
        esp_matter_attr_val_t val2 = esp_matter_char_str("humidity", strlen("humidity"));
        attribute::set_val(node_label_attr2, &val2);
        attribute::report(ep_id2,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                          &val2);
    }
    ABORT_APP_ON_FAILURE(hum_ep != nullptr, ESP_LOGE(TAG, "Failed to create humidity_sensor endpoint"));
   
    wired::start_aht10(temp_ep, hum_ep);
    
    // Led endpoint (remote dimmer switch)
    dimmable_light::config_t bridge_config{};
    endpoint_t *ep = dimmable_light::create(node, &bridge_config, ENDPOINT_FLAG_NONE, nullptr);
    if (ep)
    {
        led_ep_global = ep;
        uint16_t ep_id = endpoint::get_id(ep);

        // Cluster BridgedDeviceBasicInformation
        cluster::bridged_device_basic_information::config_t basic_info_cfg{};
        cluster::bridged_device_basic_information::create(ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);

        // Cluster OnOff
        cluster::on_off::config_t onoff_cfg{};
        cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);

        // Cluster LevelControl
        cluster::level_control::config_t lvl_cfg{};
        cluster::level_control::create(ep, &lvl_cfg, CLUSTER_FLAG_SERVER);

        // NodeLabel
        attribute_t *node_label_attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (node_label_attr)
        {
            esp_matter_attr_val_t val = esp_matter_char_str("dimmable_light", strlen("dimmable_light"));
            attribute::set_val(node_label_attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }
    }

    /* Restore previously created endpoints */
    devices::load_devices_from_nvs(node);

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));    
    
    bridge::init(node);  

    // --- Start WebGUI ---
    static webgui::config_t cfg;
    cfg.bridge_name = "Matter Bridge v2.0";
    cfg.udp_port = 12345;
    cfg.offline_timeout_ms = 60000;

    webgui::start(&bridge::get_device_map(), &cfg);

}   