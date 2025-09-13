#pragma once

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

// -------------------- Inicialización --------------------
void init(esp_matter::node_t *node);

// -------------------- Envío de comandos --------------------
void send_command_to_esp01(const std::string &device_id, const std::string &payload);

// -------------------- UDP broadcast listener --------------------
void udp_task(void *pvParameters);
void start_udp_task(void *pvParameters);

// -------------------- Restore endpoints --------------------
void restore_endpoints(esp_matter::node_t *node);

// -------------------- Reachable / Unreachable --------------------
esp_err_t set_reachable(uint16_t endpoint_id, bool reachable);

// -------------------- Handler de updates de atributos --------------------
esp_err_t handle_attribute_update(uint16_t endpoint_id,
                                  uint32_t cluster_id,
                                  uint32_t attribute_id,
                                  esp_matter_attr_val_t *val);

} // namespace bridge
