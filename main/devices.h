#pragma once
#include <map>
#include <string>
#include <functional>
#include <ctime>
#include <cJSON.h>
#include <esp_matter.h>

namespace devices {

// Alias cortos desde esp_matter
using node_t = esp_matter::node_t;
using endpoint_t = esp_matter::endpoint_t;

struct device_t {
    std::string uid;
    std::string ip;
    node_t *node = nullptr;
    time_t last_seen = 0;
    bool reachable = false;
    std::map<std::string, endpoint_t *> endpoints;
    cJSON *data = nullptr;
};

using creator_t = std::function<endpoint_t *(node_t *node, const std::string &uid)>;

extern std::map<std::string, device_t> registry;

void register_device_type(const std::string &type, creator_t fn);
esp_err_t load_devices_from_nvs(node_t *node);
void report_reachable(endpoint_t *ep, bool reachable);
void create_or_update(const std::string &ip, node_t *node, const std::string &json_str);
void init_types();
esp_err_t handle_attribute_update(uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val);

} // namespace devices
