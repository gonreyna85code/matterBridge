#pragma once
#include <esp_http_server.h>
#include <string>
#include <map>
#include "devices.h"

namespace webgui {

struct config_t {
    uint16_t udp_port = 12345;
    uint32_t offline_timeout_ms = 60000;
    std::string bridge_name = "Matter UDP Bridge";
};

// Inicia el servidor web
void start(const std::map<std::string, devices::device_t> *device_map,
           const config_t *cfg);

// Detiene el servidor
void stop();

// Actualiza el mapa de dispositivos
void update_device_map(const std::map<std::string, devices::device_t> *map);

} // namespace webgui
