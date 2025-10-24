#pragma once
#include "devices.h"
#include <esp_matter.h>
#include <string>
#include <map>
#include <ctime>   // time_t

namespace bridge {

struct esp01_info_t {
    esp_matter::endpoint_t *ep;
    time_t last_seen;
    std::string uid;
    std::string ip;
    bool reachable;
};

const std::map<std::string, devices::device_t> &get_device_map();

// -------------------- Inicialización --------------------
void init(esp_matter::node_t *node);

// -------------------- UDP broadcast listener --------------------
void udp_task(void *pvParameters);

// -------------------- Handler de updates de atributos --------------------
esp_err_t handle_attribute_update(uint16_t endpoint_id,
                                  uint32_t cluster_id,
                                  uint32_t attribute_id,
                                  esp_matter_attr_val_t *val);

} // namespace bridge
