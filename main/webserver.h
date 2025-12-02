#pragma once
#include <esp_http_server.h>
#include <string>
#include <map>
#include "devices.h"

namespace webgui
{

    struct config_t
    {
        std::string bridge_name = "AntiMatter Bridge";
        int32_t broadcast_port = 12345;
        int32_t command_port = 12346;
        int32_t offline_timeout_ms = 120000;
    };

    // Inicia el servidor web
    void start(const std::map<std::string, devices::device_t> *device_map);

    // Detiene el servidor
    void stop();

    void load_config(config_t &cfg);
    void save_config(const config_t &cfg);

    extern config_t cfg;

    // Actualiza el mapa de dispositivos
    void update_device_map(const std::map<std::string, devices::device_t> *map);

} // namespace webgui
