#include "webserver.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <sstream>
#include <string>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_event.h>
#include <esp_netif.h>
#include "bridge.h"
#include "devices.h"

static const char *NS = "cfg";
static httpd_handle_t server = nullptr;
static const std::map<std::string, devices::device_t> *g_device_map = nullptr;

namespace webgui
{

    // ===================== HTML builder =====================
    static std::string build_html()
    {
        std::stringstream html;
        html << "<html><head>"
                "<meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             << "<style>"
             << "body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:0}"
             << "h2{background:#222;margin:0;padding:12px}"
             << "table{width:95%;margin:auto;border-collapse:collapse}"
             << "td,th{padding:8px;border-bottom:1px solid #333;text-align:center}"
             << "a{color:#4af;text-decoration:none}"
             << ".ok{color:#0f0}.bad{color:#f44}"
             << "</style></head><body>";

        html << "<h2>" << webgui::cfg.bridge_name << "</h2>";

        html << "<table><tr>"
             << "<th>UID</th><th>Endpoint</th><th>IP</th><th>Status</th><th>Last seen (s)</th><th>Settings</th><th>Remove</th>"
             << "</tr>";

        if (g_device_map && !g_device_map->empty())
        {
            time_t now = esp_timer_get_time() / 1000000;
            for (const auto &it : *g_device_map)
            {
                const auto &d = it.second;
                int age = now - (d.last_seen / 1000);

                for (const auto &ep : d.endpoints)
                {
                    const std::string &t = ep.first;

                    html << "<tr>";
                    html << "<td>" << d.uid << "</td>";
                    html << "<td>" << t << "</td>";
                    html << "<td>" << d.ip << "</td>";
                    html << "<td class='" << (d.reachable ? "ok" : "bad") << "'>"
                         << (d.reachable ? "Online" : "Offline") << "</td>";
                    html << "<td>" << age << "</td>";

                    if (!d.ip.empty())
                        html << "<td><a href='http://" << d.ip << "' target='_blank'>⚙️</a></td>";
                    else
                        html << "<td>-</td>";

                    html << "<td><a href='/remove?uid=" << d.uid
                         << "&type=" << t
                         << "' style='color:#f44;font-weight:bold'>✖</a></td>";

                    html << "</tr>";
                }
            }
        }
        else
        {
            html << "<tr><td colspan='7'>No devices detected</td></tr>";
        }

        html << "</table><br><a href='/config'>⚙️ Config</a>";
        html << "</body></html>";

        return html.str();
    }

    static esp_err_t config_get_handler(httpd_req_t *req)
    {
        std::stringstream html;
        html << "<html><head>"
                "<meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             << "<style>"
             << "body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:0}"
             << "h2{background:#222;margin:0;padding:12px}"
             << "form{margin-top:13px}"
             << "input{margin:5px}"
             << "table{width:95%;margin:auto;border-collapse:collapse}"
             << "td,th{padding:8px;border-bottom:1px solid #333;text-align:center}"
             << "a{color:#4af;text-decoration:none}"
             << ".ok{color:#0f0}.bad{color:#f44}"
             << "</style></head><body>";

        html << "<h2>Settings</h2><form method='POST' action='/config'>";
        html << "Broadcast Port:<br><input name='broadcast_port' value='" << webgui::cfg.broadcast_port << "'><br><br>";
        html << "Command Port:<br><input name='command_port' value='" << webgui::cfg.command_port << "'><br><br>";
        html << "Offline Timeout (ms):<br><input name='offline_timeout' value='" << webgui::cfg.offline_timeout_ms << "'><br><br>";
        html << "<button type='submit'>Save</button></form>";
        html << "<br><a href='/antimatter'>🏠 Back</a></body></html>";

        httpd_resp_set_type(req, "text/html");
        return httpd_resp_send(req, html.str().c_str(), html.str().size());
    }

    // ===================== HTTP Handlers =====================
    static esp_err_t root_get_handler(httpd_req_t *req)
    {
        std::string html = build_html();
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, html.c_str(), html.size());
        return ESP_OK;
    }

    static esp_err_t handle_remove_endpoint(httpd_req_t *req)
    {
        char uid[64], type[16];
        size_t uid_len = httpd_req_get_url_query_len(req) + 1;
        if (uid_len <= 1)
            return ESP_FAIL;

        char *buf = (char *)malloc(uid_len);
        httpd_req_get_url_query_str(req, buf, uid_len);

        if (httpd_query_key_value(buf, "uid", uid, sizeof(uid)) != ESP_OK ||
            httpd_query_key_value(buf, "type", type, sizeof(type)) != ESP_OK)
        {
            free(buf);
            return ESP_FAIL;
        }
        free(buf);

        devices::remove_endpoint(uid, type);

        httpd_resp_set_status(req, "302 Temporary Redirect");
        httpd_resp_set_hdr(req, "Location", "/antimatter");
        return httpd_resp_send(req, NULL, 0);
    }

    config_t cfg;

    static esp_err_t config_post_handler(httpd_req_t *req)
    {
        char buf[256];
        int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
        if (len <= 0)
            return ESP_FAIL;
        buf[len] = 0;

        // formato esperado: bridge_name=XXX&udp_port=12345&timeout=60000
        config_t new_cfg = webgui::cfg;

        char val[128];
        if (httpd_query_key_value(buf, "broadcast_port", val, sizeof(val)) == ESP_OK)
            new_cfg.broadcast_port = atoi(val);

        if (httpd_query_key_value(buf, "command_port", val, sizeof(val)) == ESP_OK)
            new_cfg.command_port = atoi(val);

        if (httpd_query_key_value(buf, "offline_timeout", val, sizeof(val)) == ESP_OK)
            new_cfg.offline_timeout_ms = atoi(val);

        save_config(new_cfg);
        webgui::cfg = new_cfg;

        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/config");
        return httpd_resp_send(req, NULL, 0);
    }

    // ===================== Webserver control =====================
    static void start_server()
    {
        if (server)
            return;

        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 80;

        if (httpd_start(&server, &config) == ESP_OK)
        {
            httpd_uri_t root_uri = {"/antimatter", HTTP_GET, root_get_handler, nullptr};
            httpd_uri_t cfg_uri = {"/config", HTTP_GET, config_get_handler, nullptr};
            httpd_uri_t remove_uri = {"/remove", HTTP_GET, handle_remove_endpoint, nullptr};
            httpd_uri_t config_post_uri = {"/config", HTTP_POST, config_post_handler, nullptr};

            httpd_register_uri_handler(server, &remove_uri);
            httpd_register_uri_handler(server, &root_uri);
            httpd_register_uri_handler(server, &cfg_uri);
            httpd_register_uri_handler(server, &config_post_uri);
        }
    }

    static void wait_for_ip_and_start_server(void *arg)
    {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif)
        {
            esp_netif_ip_info_t ip;
            if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0)
            {
                start_server();
                vTaskDelete(NULL);
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_for_ip_and_start_server(arg);
    }

    void start(const std::map<std::string, devices::device_t> *device_map)
    {
        g_device_map = device_map;        
        xTaskCreate(wait_for_ip_and_start_server, "wait_ip_web", 4096, NULL, 5, NULL);
    }

    void stop()
    {
        if (server)
        {
            httpd_stop(server);
            server = nullptr;
        }
    }

    void update_device_map(const std::map<std::string, devices::device_t> *map)
    {
        g_device_map = map;
    }

    void load_config(config_t &cfg)
    {
        nvs_handle_t h;
        if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK)
            return;
        nvs_get_i32(h, "broadcast_port", &cfg.broadcast_port);
        nvs_get_i32(h, "command_port", &cfg.command_port);
        nvs_get_i32(h, "offline_timeout", &cfg.offline_timeout_ms);
        nvs_close(h);
    }

    void save_config(const config_t &cfg)
    {
        nvs_handle_t h;
        if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK)
            return;
        nvs_set_i32(h, "broadcast_port", cfg.broadcast_port);
        nvs_set_i32(h, "command_port", cfg.command_port);
        nvs_set_i32(h, "offline_timeout", cfg.offline_timeout_ms);
        nvs_commit(h);
        nvs_close(h);
    }

} // namespace webgui