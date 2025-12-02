#include "bridge.h"
#include "devices.h"
#include <esp_log.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#include <sys/select.h>
#include "webserver.h"

namespace bridge
{
    static const char *TAG = "BRIDGE";
    static SemaphoreHandle_t mutex = nullptr;
    static TaskHandle_t udp_task_handle = nullptr;
    static uint32_t startup_delay_ms = 60000;
    static int udp_sock = -1;
    static esp_matter::node_t *node_global = nullptr;

    
    void init(esp_matter::node_t *node)
    {
        if (!mutex)
            mutex = xSemaphoreCreateMutex();
        node_global = node;
        devices::init_types();
        devices::load_devices_from_nvs(node);
        webgui::load_config(webgui::cfg);
        startup_delay_ms = 120000;        
        xTaskCreate(udp_task, "bridge_udp_task", 8192, node, 5, &udp_task_handle);        
        webgui::start(&bridge::get_device_map());
    }

    const std::map<std::string, devices::device_t> &get_device_map()
    {
        return devices::registry;
    }

    void udp_task(void *pvParameters)
    {
        esp_matter::node_t *node = (esp_matter::node_t *)pvParameters;

        vTaskDelay(pdMS_TO_TICKS(startup_delay_ms));

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Socket error");
            udp_task_handle = nullptr;
            vTaskDelete(NULL);
            return;
        }
        int32_t port = webgui::cfg.broadcast_port;
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            ESP_LOGE(TAG, "Bind failed");
            close(sock);
            udp_task_handle = nullptr;
            vTaskDelete(NULL);
            return;
        }

        ESP_LOGI(TAG, "UDP listener ready");

        udp_sock = sock;
        int32_t OFFLINE_TIMEOUT_MS = webgui::cfg.offline_timeout_ms;
        constexpr size_t BUF_SIZE = 1024;
        char buf[BUF_SIZE];

        while (true)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sock, &readfds);

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;

            int sel = select(sock + 1, &readfds, NULL, NULL, &tv);

            if (sel > 0 && FD_ISSET(sock, &readfds))
            {
                struct sockaddr_in src;
                socklen_t slen = sizeof(src);
                int len = recvfrom(sock, buf, BUF_SIZE - 1, 0, (struct sockaddr *)&src, &slen);

                if (len > 0)
                {
                    buf[len] = '\0';
                    char ip_str[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &src.sin_addr, ip_str, sizeof(ip_str));

                    std::string ip(ip_str);
                    std::string json_str(buf);

                    if (json_str.front() == '{' && json_str.back() == '}')
                    {
                        if (mutex)
                            xSemaphoreTake(mutex, portMAX_DELAY);
                        devices::create_or_update(ip, node, json_str);
                        if (mutex)
                            xSemaphoreGive(mutex);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Invalid UDP payload from %s", ip.c_str());
                    }
                }
            }

            uint64_t now = esp_timer_get_time() / 1000;
            if (mutex)
                xSemaphoreTake(mutex, portMAX_DELAY);
            for (auto &[uid, dev] : devices::registry)
            {
                int64_t diff = (int64_t)now - (int64_t)dev.last_seen;
                if (dev.reachable && diff > OFFLINE_TIMEOUT_MS)
                {
                    dev.reachable = false;
                    for (auto &[type, ep] : dev.endpoints)
                        devices::report_reachable(ep, false);
                }
            }
            if (mutex)
                xSemaphoreGive(mutex);
        }
        close(sock);
        udp_sock = -1;
        udp_task_handle = nullptr;
        vTaskDelete(NULL);
    }

    esp_err_t handle_attribute_update(uint16_t ep, uint32_t cl, uint32_t at, esp_matter_attr_val_t *val)
    {
        return devices::handle_attribute_update(ep, cl, at, val);
    }

} // namespace bridge
