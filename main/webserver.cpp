#include "webserver.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <sstream>
#include <string>
#include "bridge.h"

static const char *TAG = "WEBGUI";
static httpd_handle_t server = nullptr;
static const std::map<std::string, devices::device_t> *g_device_map = nullptr;
static const webgui::config_t *g_cfg = nullptr;

namespace webgui {

// ===================== HTML builder =====================
static std::string build_html()
{
    std::stringstream html;
    html << "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
         << "<style>"
         << "body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:0}"
         << "h2{background:#222;margin:0;padding:12px}"
         << "table{width:95%;margin:auto;border-collapse:collapse}"
         << "td,th{padding:8px;border-bottom:1px solid #333}"
         << "a{color:#4af;text-decoration:none}"
         << ".ok{color:#0f0}.bad{color:#f44}"
         << "</style></head><body>";
    html << "<h2>" << g_cfg->bridge_name << "</h2>";
    html << "<p>UDP Port: " << g_cfg->udp_port << "</p>";

    html << "<table><tr><th>UID</th><th>IP</th><th>Status</th><th>Last seen (s)</th><th>Link</th></tr>";

    if (g_device_map && !g_device_map->empty())
    {
        time_t now = esp_timer_get_time() / 1000000;
        for (const auto &it : *g_device_map)
        {
            const auto &d = it.second;
            int age = now - (d.last_seen / 1000);
            html << "<tr>";
            html << "<td>" << d.uid << "</td>";
            html << "<td>" << d.ip << "</td>";
            html << "<td class='" << (d.reachable ? "ok" : "bad") << "'>"
                 << (d.reachable ? "Online" : "Offline") << "</td>";
            html << "<td>" << age << "</td>";
            if (!d.ip.empty())
                html << "<td><a href='http://" << d.ip << "' target='_blank'>Open</a></td>";
            else
                html << "<td>-</td>";
            html << "</tr>";
        }
    }
    else
    {
        html << "<tr><td colspan='5'>No devices detected</td></tr>";
    }

    html << "</table><br><a href='/api/devices'>📡 JSON API</a> | <a href='/api/config'>⚙️ Config</a>";
    html << "</body></html>";
    return html.str();
}

// ===================== HTTP Handlers =====================
static esp_err_t root_get_handler(httpd_req_t *req)
{
    std::string html = build_html();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), html.size());
    return ESP_OK;
}

static esp_err_t devices_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    std::stringstream json;
    json << "[";
    bool first = true;
    if (g_device_map)
    {
        for (const auto &it : *g_device_map)
        {
            if (!first)
                json << ",";
            const auto &d = it.second;
            json << "{";
            json << "\\\"uid\\\":\\\"" << d.uid << "\\\",";
            json << "\\\"ip\\\":\\\"" << d.ip << "\\\",";
            json << "\\\"reachable\\\":" << (d.reachable ? "true" : "false") << ",";
            json << "\\\"last_seen\\\":" << d.last_seen;
            json << "}";
            first = false;
        }
    }
    json << "]";
    httpd_resp_send(req, json.str().c_str(), json.str().size());
    return ESP_OK;
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    std::stringstream json;
    json << "{";
    json << "\\\"bridge_name\\\":\\\"" << g_cfg->bridge_name << "\\\",";
    json << "\\\"udp_port\\\":" << g_cfg->udp_port << ",";
    json << "\\\"offline_timeout_ms\\\":" << g_cfg->offline_timeout_ms;
    json << "}";
    httpd_resp_send(req, json.str().c_str(), json.str().size());
    return ESP_OK;
}

// ===================== Webserver control =====================
void start(const std::map<std::string, devices::device_t> *device_map,
                   const config_t *cfg)
{
    g_device_map = device_map;
    g_cfg = cfg;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
        httpd_uri_t root_uri = {"/", HTTP_GET, root_get_handler, nullptr};
        httpd_uri_t dev_uri = {"/api/devices", HTTP_GET, devices_get_handler, nullptr};
        httpd_uri_t cfg_uri = {"/api/config", HTTP_GET, config_get_handler, nullptr};

        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &dev_uri);
        httpd_register_uri_handler(server, &cfg_uri);
    }
}

void stop()
{
    if (server)
    {
        httpd_stop(server);
        server = nullptr;
        ESP_LOGI(TAG, "Web server stopped");
    }
}

void update_device_map(const std::map<std::string, devices::device_t> *map)
{
    g_device_map = map;
}

} // namespace webgui