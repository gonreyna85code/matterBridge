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

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;

#define BROADCAST_PORT 12345
#define COMMANDS_PORT 12346
#define BUF_SIZE 256

static SemaphoreHandle_t esp01_mutex = nullptr;
static const char *TAG = "app_main";
uint16_t on_off_plugin_unit_endpoint_id = 0;

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

struct esp01_cmd_t
{
    std::string ip;
    std::string payload;
};

void esp01_cmd_task(void *pvParameters)
{
    esp01_cmd_t *cmd = (esp01_cmd_t *)pvParameters;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "No se pudo crear socket para enviar comando");
        delete cmd;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(COMMANDS_PORT);
    inet_pton(AF_INET, cmd->ip.c_str(), &dest_addr.sin_addr);

    int sent = sendto(sock, cmd->payload.c_str(), cmd->payload.size(), 0,
                      (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (sent < 0)
    {
        ESP_LOGE(TAG, "Error enviando comando a %s", cmd->ip.c_str());
    }
    else
    {
        ESP_LOGI(TAG, "Comando enviado a %s: %s", cmd->ip.c_str(), cmd->payload.c_str());
    }

    close(sock);
    delete cmd; // liberar memoria
    vTaskDelete(NULL);
}

struct esp01_info_t
{
    endpoint_t *ep;
    time_t last_seen;
    bool reachable;
    std::string ip;
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

void send_command_to_esp01(const std::string &device_id, const std::string &payload)
{
    if (xSemaphoreTake(esp01_mutex, portMAX_DELAY) == pdTRUE)
    {
        auto it = esp01_map.find(device_id);
        if (it != esp01_map.end() && it->second.reachable)
        {
            esp01_cmd_t *cmd = new esp01_cmd_t{it->second.ip, payload};
            xTaskCreate(esp01_cmd_task, "esp01_cmd_task", 4096, cmd, 5, NULL);
        }
        else
        {
            ESP_LOGW(TAG, "No se encontró dispositivo %s o no está reachable", device_id.c_str());
        }
        xSemaphoreGive(esp01_mutex);
    }
}

static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE)
    {
        ESP_LOGI(TAG, "Pre update Endpoint=0x%x Cluster=0x%lx Attr=0x%lx",
                 endpoint_id, cluster_id, attribute_id);

        if (cluster_id == chip::app::Clusters::OnOff::Id &&
            attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id)
        {

            bool new_state = val->val.b;
            ESP_LOGI(TAG, "OnOff -> %s", new_state ? "ON" : "OFF");

            for (auto &entry : esp01_map)
            {
                if (entry.second.ep &&
                    endpoint::get_id(entry.second.ep) == endpoint_id &&
                    entry.second.reachable)
                {

                    ESP_LOGI(TAG, "Match endpoint %d -> mando a %s",
                             endpoint_id, entry.first.c_str());

                    send_command_to_esp01(entry.first, new_state ? "ON" : "OFF");
                    break;
                }
            }
        }
    }

    return err;
}

void udp_task(void *pvParameters)
{
    node_t *node = (node_t *)pvParameters;

    if (!esp01_mutex)
        esp01_mutex = xSemaphoreCreateMutex();

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "No se pudo crear el socket");
        vTaskDelete(NULL);
        return;
    }

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

    const int OFFLINE_TIMEOUT_MS = 120000; // 2 minutos

    while (true)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);

        struct timeval tv;
        tv.tv_sec = 60;
        tv.tv_usec = 0;

        int ret = select(sock + 1, &rfds, NULL, NULL, &tv);

        time_t now = esp_timer_get_time() / 1000; // ms

        // --- Procesar paquete si llegó ---
        if (ret > 0 && FD_ISSET(sock, &rfds))
        {
            int len = recvfrom(sock, buf, BUF_SIZE - 1, 0,
                               (struct sockaddr *)&source_addr, &socklen);
            if (len > 0)
            {
                buf[len] = 0;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &source_addr.sin_addr, ip_str, sizeof(ip_str));
                std::string ip(ip_str);
                std::string device_id(buf);

                ESP_LOGI(TAG, "ESP01 detectado: %s", buf);

                if (xSemaphoreTake(esp01_mutex, portMAX_DELAY) == pdTRUE)
                {
                    auto it = esp01_map.find(device_id);
                    if (it != esp01_map.end() && it->second.ep != nullptr)
                    {
                        it->second.last_seen = now;
                        it->second.ip = ip;
                        if (!it->second.reachable)
                        {
                            it->second.reachable = true;
                            set_reachable(endpoint::get_id(it->second.ep), true);

                            attribute_t *onoff_attr = attribute::get(
                                endpoint::get_id(it->second.ep),
                                chip::app::Clusters::OnOff::Id,
                                chip::app::Clusters::OnOff::Attributes::OnOff::Id);

                            if (onoff_attr)
                            {
                                esp_matter_attr_val_t val = esp_matter_bool(false);
                                attribute::set_val(onoff_attr, &val);
                                attribute::report(endpoint::get_id(it->second.ep),
                                                  chip::app::Clusters::OnOff::Id,
                                                  chip::app::Clusters::OnOff::Attributes::OnOff::Id,
                                                  &val);
                            }
                            ESP_LOGI(TAG, "ESP01 REACTIVADO: %s", device_id.c_str());
                        }
                        xSemaphoreGive(esp01_mutex);
                        continue;
                    }

                    // --- Nuevo dispositivo ---
                    nvs_handle_t nvs_handle;
                    if (nvs_open("esp01", NVS_READWRITE, &nvs_handle) == ESP_OK)
                    {
                        esp01_info_t &info = esp01_map[device_id];
                        info.last_seen = now;
                        info.ip = ip;

                        on_off_plugin_unit::config_t on_off_plugin_unit_config;
                        endpoint_t *ep = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                        if (ep)
                        {
                            uint16_t ep_id = endpoint::get_id(ep);

                            cluster::on_off::config_t onoff_cfg{};
                            cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);

                            cluster::bridged_device_basic_information::config_t basic_info_cfg{};
                            cluster::bridged_device_basic_information::create(ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);

                            attribute_t *node_label_attr = attribute::get(
                                ep_id,
                                chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);

                            if (node_label_attr)
                            {
                                esp_matter_attr_val_t val = esp_matter_char_str((char *)device_id.c_str(), device_id.size());
                                attribute::set_val(node_label_attr, &val);
                                attribute::report(ep_id,
                                                  chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                                  chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                                                  &val);
                            }

                            endpoint::enable(ep);
                            info.ep = ep;
                            info.reachable = true;

                            nvs_set_str(nvs_handle, "device_map", map_str_from_map(esp01_map).c_str());
                            nvs_commit(nvs_handle);

                            ESP_LOGI(TAG, "Nuevo endpoint creado: UID=%s, EP=%d", device_id.c_str(), ep_id);
                        }
                        nvs_close(nvs_handle);
                    }
                    xSemaphoreGive(esp01_mutex);
                }
            }
        }

        // --- Revisar dispositivos offline siempre ---
        if (xSemaphoreTake(esp01_mutex, portMAX_DELAY) == pdTRUE)
        {
            for (auto &entry : esp01_map)
            {
                if (entry.second.ep && entry.second.reachable &&
                    now - entry.second.last_seen > OFFLINE_TIMEOUT_MS)
                {
                    entry.second.reachable = false;
                    set_reachable(endpoint::get_id(entry.second.ep), false);
                    ESP_LOGI(TAG, "ESP01 OFFLINE: %s", entry.first.c_str());
                }
            }
            xSemaphoreGive(esp01_mutex);
        }
    }
}

void restore_endpoints(node_t *node)
{
    nvs_handle_t nvs_handle;
    if (nvs_open("esp01", NVS_READWRITE, &nvs_handle) != ESP_OK) return;

    size_t required_size = 0;
    if (nvs_get_str(nvs_handle, "device_map", nullptr, &required_size) != ESP_OK || required_size == 0) {
        nvs_close(nvs_handle);
        return;
    }

    std::vector<char> buf(required_size);
    if (nvs_get_str(nvs_handle, "device_map", buf.data(), &required_size) != ESP_OK) {
        nvs_close(nvs_handle);
        return;
    }

    const char *str = buf.data();
    while (*str) {
        const char *colon = strchr(str, ':');
        if (!colon) break;

        std::string uid(str, colon - str);

        const char *comma = strchr(colon + 1, ',');
        if (!comma) break;

        std::string ep_str(colon + 1, comma - (colon + 1));
        char *endptr = nullptr;
        long ep_id_l = strtol(ep_str.c_str(), &endptr, 10);
        if (endptr == ep_str.c_str() || ep_id_l <= 0 || ep_id_l > 0xFFFF) {
            str = comma + 1;
            continue; // invalid endpoint id, skip
        }
        uint16_t ep_id = static_cast<uint16_t>(ep_id_l);

        const char *next_comma = strchr(comma + 1, ',');
        bool reachable = false;
        if (next_comma) reachable = *(comma + 1) == '1';
        else reachable = *(comma + 1) == '1'; // last field

        // Crear endpoint si no existe
        if (esp01_map.find(uid) == esp01_map.end()) {
            on_off_plugin_unit::config_t cfg{};
            endpoint_t *ep = on_off_plugin_unit::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);
            if (!ep) {
                str = next_comma ? next_comma + 1 : comma + 1;
                continue;
            }

            uint16_t new_ep_id = endpoint::get_id(ep);
            cluster::on_off::config_t onoff_cfg{};
            cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);

            cluster::bridged_device_basic_information::config_t bdbi_cfg{};
            cluster::bridged_device_basic_information::create(ep, &bdbi_cfg, CLUSTER_FLAG_SERVER);

            attribute_t *node_label_attr = attribute::get(new_ep_id,
                                                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
            if (node_label_attr) {
                esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
                attribute::set_val(node_label_attr, &val);
                attribute::report(new_ep_id,
                                  chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                  chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                                  &val);
            }

            endpoint::enable(ep);
            esp01_map[uid] = {ep, esp_timer_get_time() / 1000, reachable};
            if (!reachable) set_reachable(new_ep_id, false);
        } else {
            // ya existe
            esp01_map[uid].reachable = reachable;
            esp01_map[uid].last_seen = esp_timer_get_time() / 1000;
            if (!reachable) set_reachable(ep_id, false);
        }

        str = next_comma ? next_comma + 1 : comma + 1;
        vTaskDelay(1 / portTICK_PERIOD_MS); // ceder CPU
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

    /* Restore previously created endpoints */
    restore_endpoints(node);

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    // Crear task para detectar ESP01 y agregar endpoint dinámicament
    xTaskCreate(udp_task, "udp_task", 4096, node, 5, NULL);
}
