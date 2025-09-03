/*
   Este código mantiene todas tus funcionalidades existentes
   y agrega la creación dinámica del endpoint On/Off al detectar un ESP01 en la red.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <map>
#include <string>

#define BROADCAST_PORT 12345
#define BUF_SIZE 128

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_console_bridge.h>

#include <common_macros.h>

static const char *TAG = "app_main";
uint16_t on_off_plugin_unit_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;

esp_err_t set_reachable(uint16_t endpoint_id, bool reachable)
{
    attribute_t *attr = attribute::get(endpoint_id,
                                       chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                       chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id);

    if (!attr)
    {
        ESP_LOGW("APP", "No se encontró atributo Reachable en endpoint %d", endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }

    esp_matter_attr_val_t val = esp_matter_bool(reachable);
    attribute::set_val(attr, &val);
    attribute::report(
        endpoint_id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Id,
        chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id,
        &val);
    ESP_LOGI(TAG, "Endpoint %d -> seted reachable=%d and reported", endpoint_id, reachable);
    return ESP_OK;
}

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

static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    if (type == PRE_UPDATE)
    {
        ESP_LOGI(TAG, "Pre update of Endpoint 0x%x Cluster 0x%lx attribute_id 0x%lx", endpoint_id, cluster_id, attribute_id);
    }
    return err;
}

// Task FreeRTOS para escuchar broadcast del ESP01

struct esp01_info_t
{
    endpoint_t *ep;
    time_t last_seen;
    bool reachable;
};

static std::map<std::string, esp01_info_t> esp01_map;

std::string map_str_from_map(const std::map<std::string, esp01_info_t> &map)
{
    std::string s;
    for (auto &entry : map)
    {
        if (entry.second.ep)
            s += entry.first + ":" + std::to_string(endpoint::get_id(entry.second.ep)) + "," + (entry.second.reachable ? "1" : "0") + ",";
    }
    if (!s.empty())
        s.pop_back();
    return s;
}

void udp_task(void *pvParameters)
{
    node_t *node = (node_t *)pvParameters;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "No se pudo crear el socket");
        vTaskDelete(NULL);
        return;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(BROADCAST_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ESP_LOGE(TAG, "No se pudo bindear el socket");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    char buf[BUF_SIZE];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    static std::map<std::string, esp01_info_t> esp01_map;

    // Inicializar NVS
    nvs_flash_init();
    nvs_handle_t nvs_handle;
    nvs_open("esp01", NVS_READWRITE, &nvs_handle);

    while (true)
    {
        time_t now = esp_timer_get_time() / 1000; // ms
        int len = recvfrom(sock, buf, BUF_SIZE - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len > 0)
        {
            buf[len] = 0;
            std::string device_id(buf);
            ESP_LOGI(TAG, "ESP01 detectado: %s", buf);

            auto &info = esp01_map[device_id];
            info.last_seen = now;

            endpoint_t *ep = info.ep;

            if (!ep)
            {
                // Crear nuevo endpoint
                on_off_plugin_unit::config_t on_off_plugin_unit_config;
                ep = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                if (ep)
                {
                    uint16_t ep_id = endpoint::get_id(ep);

                    // Crear Clusters
                    cluster::on_off::config_t onoff_cfg{};
                    cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);

                    cluster::bridged_device_basic_information::config_t basic_info_cfg{};
                    cluster::bridged_device_basic_information::create(ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);

                    endpoint::enable(ep);
                    info.ep = ep;
                    info.reachable = true;

                    // Guardar en NVS
                    std::string map_str;
                    for (auto &entry : esp01_map)
                    {
                        if (entry.second.ep)
                        {
                            map_str += entry.first + ":" + std::to_string(endpoint::get_id(entry.second.ep)) + "," + (entry.second.reachable ? "1" : "0") + ",";
                        }
                    }
                    if (!map_str.empty())
                        map_str.pop_back();
                    nvs_set_str(nvs_handle, "device_map", map_str.c_str());
                    nvs_commit(nvs_handle);

                    ESP_LOGI(TAG, "Nuevo endpoint creado: UID=%s, EP=%d", device_id.c_str(), ep_id);
                }
            }
            else if (!info.reachable)
            {
                info.reachable = true;
                set_reachable(endpoint::get_id(ep), true);
                ESP_LOGI(TAG, "ESP01 REACTIVADO: %s", device_id.c_str());

                // Guardar flag en NVS
                nvs_set_str(nvs_handle, "device_map", map_str_from_map(esp01_map).c_str());
                nvs_commit(nvs_handle);
            }
        }

        // Revisar dispositivos offline
        for (auto &entry : esp01_map)
        {
            if (entry.second.ep && entry.second.reachable && now - entry.second.last_seen > 10000)
            {
                entry.second.reachable = false;
                set_reachable(endpoint::get_id(entry.second.ep), false);
                ESP_LOGI(TAG, "ESP01 OFFLINE: %s, endpoint desactivado", entry.first.c_str());

                // Guardar flag en NVS
                nvs_set_str(nvs_handle, "device_map", map_str_from_map(esp01_map).c_str());
                nvs_commit(nvs_handle);
            }
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    nvs_close(nvs_handle);
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    aggregator::config_t aggregator_config;
    endpoint_t *aggregator = endpoint::aggregator::create(node, &aggregator_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(aggregator != nullptr, ESP_LOGE(TAG, "Failed to create aggregator endpoint"));

    /* ejemplo de endpoint On/Off (enchufe especificamente) */
    // on_off_plugin_unit::config_t on_off_plugin_unit_config;
    // endpoint_t *endpoint = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_NONE, NULL);
    // ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create switch endpoint"));

    // on_off_plugin_unit_endpoint_id = endpoint::get_id(endpoint);
    // ESP_LOGI(TAG, "Light creado con endpoint_id %d", on_off_plugin_unit_endpoint_id);

    // cluster::on_off::config_t onoff_cfg{};
    // onoff_cfg.on_off = false;  // estado inicial
    // cluster_t *onoff_cluster = cluster::on_off::create(endpoint, &onoff_cfg, CLUSTER_FLAG_SERVER);

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::bridge_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::init();
#endif

    // Crear task para detectar ESP01 y agregar endpoint dinámicamente
    xTaskCreate(udp_task, "udp_task", 4096, node, 5, NULL);
}
