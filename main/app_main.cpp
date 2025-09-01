/*
   Este código mantiene todas tus funcionalidades existentes
   y agrega la creación dinámica del endpoint On/Off al detectar un ESP01 en la red.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <set>
#include <string>

#define BROADCAST_PORT 12345
#define BUF_SIZE 128
static const char *TAG = "UDP_DETECT";

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_console_bridge.h>

#include <common_macros.h>

static const char *MTAG = "app_main";

static std::set<std::string> known_uids;
uint16_t on_off_plugin_unit_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;

static bool endpoint_created = false; // flag para crear solo una vez

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(MTAG, "Interface IP Address Changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(MTAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(MTAG, "Commissioning failed, fail safe timer expired");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(MTAG, "Commissioning session started");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(MTAG, "Commissioning session stopped");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(MTAG, "Commissioning window opened");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(MTAG, "Commissioning window closed");
        break;
    default:
        break;
    }
}

static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    if (type == PRE_UPDATE) {
        ESP_LOGI(MTAG, "Pre update of Endpoint 0x%x Cluster 0x%lx attribute_id 0x%lx", endpoint_id, cluster_id, attribute_id);
    }
    return err;
}

// Task FreeRTOS para escuchar broadcast del ESP01
void udp_task(void* pvParameters) {
    node_t* node = (node_t*)pvParameters;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "No se pudo crear el socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(BROADCAST_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "No se pudo bindear el socket");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    char buf[BUF_SIZE];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (true) {
        int len = recvfrom(sock, buf, BUF_SIZE - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len > 0) {
            buf[len] = 0; // null terminate
            std::string uid(buf);

            if (known_uids.find(uid) == known_uids.end()) {
                // nuevo UID detectado
                known_uids.insert(uid);

                // convertir UID hexadecimal a número
                uint16_t ep_id = (uint16_t)strtol(uid.c_str(), NULL, 16);
                if (ep_id == 0) ep_id = 1; // evitar endpoint 0 que es root

                // crear endpoint On/Off
                on_off_plugin_unit::config_t on_off_plugin_unit_config;
                endpoint_t* endpoint = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_NONE, ep_id);
                if (endpoint != nullptr) {
                    uint16_t created_id = endpoint::get_id(endpoint);
                    cluster::on_off::config_t onoff_cfg{};
                    onoff_cfg.on_off = false;
                    cluster::on_off::create(endpoint, &onoff_cfg, CLUSTER_FLAG_SERVER);
                    ESP_LOGI(TAG, "Endpoint On/Off creado dinámicamente: UID=%s, endpoint_id=%d", uid.c_str(), created_id);
                } else {
                    ESP_LOGE(TAG, "Error creando endpoint para UID=%s", uid.c_str());
                }
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(MTAG, "Failed to create Matter node"));

    aggregator::config_t aggregator_config;
    endpoint_t *aggregator = endpoint::aggregator::create(node, &aggregator_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(aggregator != nullptr, ESP_LOGE(MTAG, "Failed to create aggregator endpoint"));

    on_off_plugin_unit::config_t on_off_plugin_unit_config;    
    endpoint_t *endpoint = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(MTAG, "Failed to create switch endpoint"));

    on_off_plugin_unit_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(MTAG, "Light creado con endpoint_id %d", on_off_plugin_unit_endpoint_id);

    cluster::on_off::config_t onoff_cfg{};
    onoff_cfg.on_off = false;  // estado inicial
    cluster_t *onoff_cluster = cluster::on_off::create(endpoint, &onoff_cfg, CLUSTER_FLAG_SERVER);

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(MTAG, "Failed to start Matter, err:%d", err));

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
