#include "bridge.h"
#include "devices.h" // o devices.h si luego decides exponerlo
#include <esp_log.h>
#include <lwip/sockets.h>

namespace bridge
{

    static const char *TAG = "BRIDGE";
    static SemaphoreHandle_t mutex = nullptr;

    void init(esp_matter::node_t *node)
    {
        if (!mutex)
            mutex = xSemaphoreCreateMutex();
        xTaskCreate(udp_task, "bridge_udp_task", 8192, node, 5, nullptr);
    }

    const std::map<std::string, devices::device_t> &get_device_map()
    {
        // Asegurate de incluir "devices.h"
        return devices::registry;
    }

    void udp_task(void *pvParameters)
    {
        esp_matter::node_t *node = (esp_matter::node_t *)pvParameters;
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Socket error");
            vTaskDelete(NULL);
            return;
        }

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(12345);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            ESP_LOGE(TAG, "Bind failed");
            close(sock);
            vTaskDelete(NULL);
            return;
        }

        constexpr size_t BUF_SIZE = 1024;
        char buf[BUF_SIZE];
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);

        ESP_LOGI(TAG, "UDP listener ready");

        constexpr uint64_t OFFLINE_TIMEOUT_MS = 10000; // 10s

        while (true)
        {
            int len = recvfrom(sock, buf, BUF_SIZE - 1, 0, (struct sockaddr *)&src, &slen);
            if (len > 0)
            {
                buf[len] = '\0';
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &src.sin_addr, ip_str, sizeof(ip_str));

                std::string ip(ip_str);
                std::string json_str(buf);

                if (json_str.front() != '{' || json_str.back() != '}')
                {
                    ESP_LOGW(TAG, "Invalid UDP payload from %s: %s", ip.c_str(), buf);
                }
                else
                {
                    devices::create_or_update(ip, node, json_str);
                }
            }

            // 🔄 Revisar estado de todos los dispositivos
            uint64_t now = esp_timer_get_time() / 1000;
            if (mutex)
                xSemaphoreTake(mutex, portMAX_DELAY);

            for (auto &[uid, dev] : devices::registry)
            {
                int64_t diff = (int64_t)now - (int64_t)dev.last_seen;
                if (diff < 0)
                    diff = 0;
                if (dev.reachable && diff > OFFLINE_TIMEOUT_MS)
                {
                    dev.reachable = false;
                    ESP_LOGW(TAG, "Device %s is now OFFLINE", uid.c_str());
                    for (auto &[type, ep] : dev.endpoints)
                        devices::report_reachable(ep, false);
                }
            }

            if (mutex)
                xSemaphoreGive(mutex);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        close(sock);
        vTaskDelete(NULL);
    }

    esp_err_t handle_attribute_update(uint16_t ep, uint32_t cl, uint32_t at, esp_matter_attr_val_t *val)
    {
        return devices::handle_attribute_update(ep, cl, at, val);
    }

} // namespace bridge
