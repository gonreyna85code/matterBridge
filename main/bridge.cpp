#include "bridge.h"
#include <esp_log.h>
#include <freertos/semphr.h>
#include <lwip/sockets.h>
#include <sstream>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_matter.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace bridge;

#define BROADCAST_PORT 12345
#define BUF_SIZE 128

namespace bridge
{

    static const char *TAG = "BRIDGE";
    static SemaphoreHandle_t esp01_mutex = nullptr;
    static std::map<std::string, esp01_info_t> esp01_map;
    static esp_matter::node_t *s_node = nullptr;
    uint16_t on_off_plugin_unit_endpoint_id = 0;

    std::string map_str_from_map(const std::map<std::string, esp01_info_t> &map)
    {
        std::string s;
        for (auto &entry : map)
        {
            if (entry.second.ep)
            {
                s += entry.second.uid + ":" + std::to_string(endpoint::get_id(entry.second.ep)) + ";";
            }
        }
        return s;
    }

    void init(esp_matter::node_t *node)
    {
        if (!esp01_mutex)
            esp01_mutex = xSemaphoreCreateMutex();

        s_node = node;

        // Crear la task directamente aquí, idéntico a wired::start_encoder
        xTaskCreate(udp_task, "bridge_udp_task", 8192, s_node, 5, nullptr);
    }

    // ----------- Enviar comando a ESP01 -----------
    void send_command_to_esp01(const std::string &device_id, const std::string &payload)
    {
        if (xSemaphoreTake(esp01_mutex, portMAX_DELAY) != pdTRUE)
            return;

        auto it = esp01_map.find(device_id);
        if (it != esp01_map.end() && it->second.reachable)
        {
            // Crear tarea que envía el comando
            struct esp01_cmd_t
            {
                std::string ip;
                std::string payload;
            };
            esp01_cmd_t *cmd = new esp01_cmd_t{it->second.ip, payload};

            xTaskCreate([](void *pv)
                        {
                esp01_cmd_t *c = (esp01_cmd_t *)pv;
                int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                if (sock >= 0) {
                    struct sockaddr_in addr{};
                    addr.sin_family = AF_INET;
                    addr.sin_port = htons(12346);
                    inet_pton(AF_INET, c->ip.c_str(), &addr.sin_addr);
                    sendto(sock, c->payload.c_str(), c->payload.size(), 0,
                        (struct sockaddr *)&addr, sizeof(addr));
                    close(sock);
                }
                delete c;
                vTaskDelete(NULL); }, "esp01_cmd_task", 4096, cmd, 5, NULL);
        }
        else
        {
            ESP_LOGW(TAG, "No se encontró dispositivo %s o no está reachable", device_id.c_str());
        }

        xSemaphoreGive(esp01_mutex);
    }

    // ----------- UDP listener -----------
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

        nvs_handle_t nvs_handle;
        nvs_open("esp01", NVS_READWRITE, &nvs_handle);

        while (true)
        {
            time_t now = esp_timer_get_time() / 1000; // ms
            int len = recvfrom(sock, buf, BUF_SIZE - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &source_addr.sin_addr, ip_str, sizeof(ip_str));
            std::string ip(ip_str);

            if (len > 0)
            {
                buf[len] = 0;
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
                            ESP_LOGI(TAG, "ESP01 REACTIVADO: %s", device_id.c_str());
                        }
                        xSemaphoreGive(esp01_mutex);
                        continue;
                    }

                    // Nuevo dispositivo
                    esp01_info_t &info = esp01_map[device_id];
                    info.last_seen = now;
                    info.ip = ip;
                    info.uid = device_id;

                    // Crear endpoint dinámico
                    on_off_plugin_unit::config_t on_off_plugin_unit_config;
                    endpoint_t *ep = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                    if (!ep)
                    {
                        xSemaphoreGive(esp01_mutex);
                        continue;
                    }

                    uint16_t ep_id = endpoint::get_id(ep);

                    cluster::on_off::config_t onoff_cfg{};
                    cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
                    cluster::bridged_device_basic_information::config_t basic_info_cfg{};
                    cluster::bridged_device_basic_information::create(ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);

                    // Configurar NodeLabel con UID
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
                    info.ip = ip;
                    info.reachable = true;

                    nvs_set_str(nvs_handle, "device_map", map_str_from_map(esp01_map).c_str());
                    nvs_commit(nvs_handle);

                    ESP_LOGI(TAG, "Nuevo endpoint creado: UID=%s, EP=%d", device_id.c_str(), ep_id);

                    xSemaphoreGive(esp01_mutex);
                }
            }

            // Revisar dispositivos offline
            if (xSemaphoreTake(esp01_mutex, portMAX_DELAY) == pdTRUE)
            {
                for (auto it = esp01_map.begin(); it != esp01_map.end();)
                {
                    if (it->second.ep && it->second.reachable && now - it->second.last_seen > 60000)
                    {
                        it->second.reachable = false;
                        set_reachable(endpoint::get_id(it->second.ep), false);
                        ESP_LOGI(TAG, "ESP01 OFFLINE: %s, endpoint desactivado", it->first.c_str());
                    }

                    if (now - it->second.last_seen > 24UL * 60 * 60 * 1000) // 24h en ms
                    {
                        ESP_LOGI(TAG, "ESP01 ELIMINADO: %s", it->first.c_str());
                        endpoint::destroy(node, it->second.ep); // destruye correctamente el endpoint
                        it = esp01_map.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
                xSemaphoreGive(esp01_mutex);
            }

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        nvs_close(nvs_handle);
    }

    void start_udp_task(void *pvParameters)
    {
        udp_task((esp_matter::node_t *)pvParameters);
    }

    // ----------- Restore endpoints -----------
    void restore_endpoints(node_t *node)
    {
        nvs_handle_t nvs_handle;
        if (nvs_open("esp01", NVS_READWRITE, &nvs_handle) != ESP_OK)
            return;

        size_t required_size = 0;
        if (nvs_get_str(nvs_handle, "device_map", nullptr, &required_size) != ESP_OK || required_size == 0)
        {
            nvs_close(nvs_handle);
            return;
        }

        std::vector<char> buf(required_size);
        if (nvs_get_str(nvs_handle, "device_map", buf.data(), &required_size) != ESP_OK)
        {
            nvs_close(nvs_handle);
            return;
        }

        std::stringstream ss(buf.data());
        std::string token;
        while (std::getline(ss, token, ';'))
        {
            if (token.empty())
                continue;
            auto pos = token.find(':');
            if (pos == std::string::npos)
                continue;

            std::string uid = token.substr(0, pos);
            uint16_t ep_id = std::stoi(token.substr(pos + 1));

            // --- recrear endpoint dinámico ---
            on_off_plugin_unit::config_t on_off_plugin_unit_config;
            endpoint_t *ep = on_off_plugin_unit::create(node, &on_off_plugin_unit_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
            if (!ep)
                continue;

            cluster::on_off::config_t onoff_cfg{};
            cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
            cluster::bridged_device_basic_information::config_t basic_info_cfg{};
            cluster::bridged_device_basic_information::create(ep, &basic_info_cfg, CLUSTER_FLAG_SERVER);

            // set NodeLabel con UID
            attribute_t *node_label_attr = attribute::get(
                endpoint::get_id(ep),
                chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);

            if (node_label_attr)
            {
                esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
                attribute::set_val(node_label_attr, &val);
                attribute::report(endpoint::get_id(ep),
                                  chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                  chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                                  &val);
            }

            endpoint::enable(ep);

            esp01_info_t &info = esp01_map[uid];
            info.uid = uid;
            info.ep = ep;
            info.reachable = false; // arranca siempre offline
            info.last_seen = 0;

            set_reachable(endpoint::get_id(ep), false);

            ESP_LOGI(TAG, "Endpoint restaurado: UID=%s, EP=%d (antes=%d)", uid.c_str(), endpoint::get_id(ep), ep_id);
        }

        nvs_close(nvs_handle);
    }

    // ----------- Reachable / Unreachable -----------
    esp_err_t set_reachable(uint16_t endpoint_id, bool reachable)
    {
        if (auto attr = esp_matter::attribute::get(endpoint_id,
                                                   chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                                   chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id))
        {
            esp_matter_attr_val_t val = esp_matter_bool(reachable);
            esp_matter::attribute::set_val(attr, &val);
            esp_matter::attribute::report(endpoint_id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                                          chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id,
                                          &val);
            return ESP_OK;
        }
        return ESP_ERR_NOT_FOUND;
    }

    // ----------- Attribute update handler -----------
    esp_err_t handle_attribute_update(uint16_t endpoint_id,
                                      uint32_t cluster_id,
                                      uint32_t attribute_id,
                                      esp_matter_attr_val_t *val)
    {
        // Aquí va la lógica que antes tenías en PRE_UPDATE
        if (cluster_id == chip::app::Clusters::OnOff::Id &&
            attribute_id == chip::app::Clusters::OnOff::Attributes::OnOff::Id)
        {
            // Buscar el device_id en el mapa
            std::string device_id;
            if (xSemaphoreTake(esp01_mutex, portMAX_DELAY) == pdTRUE)
            {
                for (const auto &entry : esp01_map)
                {
                    if (entry.second.ep && endpoint::get_id(entry.second.ep) == endpoint_id)
                    {
                        device_id = entry.first;
                        break;
                    }
                }
                xSemaphoreGive(esp01_mutex);
            }

            if (!device_id.empty())
            {
                std::string payload = (val->val.b ? "ON" : "OFF");
                ESP_LOGI(TAG, "Enviando comando a %s: %s", device_id.c_str(), payload.c_str());
                send_command_to_esp01(device_id, payload);
                return ESP_OK;
            }
            else
            {
                ESP_LOGW(TAG, "No se encontró device_id para endpoint %d", endpoint_id);
                return ESP_ERR_NOT_FOUND;
            }
        }
        return ESP_OK;
    }

} // namespace bridge
