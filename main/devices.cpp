#include <map>
#include <string>
#include <functional>
#include <vector>
#include <ctime>
#include <esp_matter.h>
#include <esp_log.h>
#include <cJSON.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <lwip/sockets.h>
#include <math.h>
#include "webserver.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;

namespace devices
{

    static const char *TAG = "DEVICES";
    static const char *NVS_NAMESPACE = "devices";

    struct device_nvs_endpoint_t
    {
        std::string type;
        uint16_t endpoint_id;
    };

    struct device_nvs_data_t
    {
        std::string uid;
        std::string ip;
        std::vector<device_nvs_endpoint_t> endpoints;
    };

    struct cluster_map_t
    {
        uint32_t cluster_id;
        uint32_t attribute_id;
        float multiplier;
    };

    static const std::map<std::string, cluster_map_t> type_map = {
        {"REL0", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL1", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL2", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL3", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL4", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL5", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL6", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"REL7", {chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Attributes::OnOff::Id, 1.0f}},
        {"DIM0", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM1", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM2", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM3", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM4", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM5", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM6", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"DIM7", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"TEMP", {chip::app::Clusters::TemperatureMeasurement::Id, chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id, 100.0f}},
        {"HUMI", {chip::app::Clusters::RelativeHumidityMeasurement::Id, chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, 100.0f}},
        {"LUMI", {chip::app::Clusters::IlluminanceMeasurement::Id, chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Id, 1000.0f / 65535.0f}}};

    using creator_t = std::function<endpoint_t *(node_t *, const std::string &)>;
    std::map<std::string, device_t> registry;
    static std::map<std::string, creator_t> creators;

    void register_device_type(const std::string &type, creator_t fn)
    {
        creators[type] = fn;
        ESP_LOGI(TAG, "Registered device type: %s", type.c_str());
    }

    esp_err_t save_device_to_nvs(const device_nvs_data_t &device_data)
    {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) opening NVS!", esp_err_to_name(err));
            return err;
        }

        std::string key = "device_" + device_data.uid;
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "uid", device_data.uid.c_str());
        cJSON_AddStringToObject(root, "ip", device_data.ip.c_str());

        cJSON *eps = cJSON_CreateArray();
        for (auto &e : device_data.endpoints)
        {
            cJSON *item = cJSON_CreateObject();
            cJSON_AddStringToObject(item, "type", e.type.c_str());
            if (e.endpoint_id != 0)
            {
                cJSON_AddNumberToObject(item, "endpoint_id", e.endpoint_id);
            }
            cJSON_AddItemToArray(eps, item);
        }
        cJSON_AddItemToObject(root, "endpoints", eps);

        char *json = cJSON_PrintUnformatted(root);
        err = nvs_set_str(nvs_handle, key.c_str(), json);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        cJSON_free(json);
        cJSON_Delete(root);
        return err;
    }

    void report_reachable(endpoint_t *ep, bool reachable)
    {
        if (!ep)
            return;
        auto attr = esp_matter::attribute::get(
            esp_matter::endpoint::get_id(ep),
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id);

        if (attr)
        {
            esp_matter_attr_val_t val = esp_matter_bool(reachable);
            attribute::set_val(attr, &val);
            attribute::report(
                endpoint::get_id(ep),
                chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::Reachable::Id,
                &val);
        }
    }    

    esp_err_t load_devices_from_nvs(node_t *node)
    {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGI(TAG, "NVS namespace doesn't exist yet. It will be created on first device add.");
            return ESP_OK;
        }

        size_t key_count = 0;
        nvs_iterator_t it = NULL;
        err = nvs_entry_find("nvs", NVS_NAMESPACE, NVS_TYPE_STR, &it);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error finding NVS entries: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }

        while (it != NULL)
        {
            nvs_entry_info_t info;
            nvs_entry_info(it, &info);
            if (strncmp(info.key, "device_", 7) == 0)
            {
                key_count++;

                char *json_str = NULL;
                size_t required_size;
                err = nvs_get_str(nvs_handle, info.key, NULL, &required_size);
                if (err == ESP_OK)
                {
                    json_str = (char *)malloc(required_size);
                    if (!json_str)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for JSON string");
                        nvs_close(nvs_handle);
                        return ESP_ERR_NO_MEM;
                    }
                    err = nvs_get_str(nvs_handle, info.key, json_str, &required_size);
                }

                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Error (%s) reading device data from NVS!", esp_err_to_name(err));
                    free(json_str);
                    nvs_close(nvs_handle);
                    return err;
                }

                cJSON *root = cJSON_Parse(json_str);
                free(json_str);
                if (!root)
                {
                    ESP_LOGE(TAG, "Error parsing JSON: %s", cJSON_GetErrorPtr());
                    nvs_close(nvs_handle);
                    return ESP_FAIL;
                }

                cJSON *uid_j = cJSON_GetObjectItem(root, "uid");
                cJSON *ip_j = cJSON_GetObjectItem(root, "ip");
                cJSON *endpoints_j = cJSON_GetObjectItem(root, "endpoints");
                cJSON *types_j = cJSON_GetObjectItem(root, "type"); // legacy support

                if (!cJSON_IsString(uid_j) || !cJSON_IsString(ip_j) || !(cJSON_IsArray(endpoints_j) || cJSON_IsArray(types_j)))
                {
                    ESP_LOGE(TAG, "Invalid JSON format in NVS (expect uid/ip/endpoints)");
                    cJSON_Delete(root);
                    nvs_close(nvs_handle);
                    return ESP_FAIL;
                }

                std::string uid = uid_j->valuestring;
                std::string ip = ip_j->valuestring;
                std::vector<device_nvs_endpoint_t> endpoints;

                // New format: endpoints array of objects {type, endpoint_id?}
                if (cJSON_IsArray(endpoints_j))
                {
                    cJSON *ep_elem = NULL;
                    cJSON_ArrayForEach(ep_elem, endpoints_j)
                    {
                        if (cJSON_IsObject(ep_elem))
                        {
                            cJSON *t = cJSON_GetObjectItem(ep_elem, "type");
                            cJSON *eid = cJSON_GetObjectItem(ep_elem, "endpoint_id");
                            if (cJSON_IsString(t))
                            {
                                device_nvs_endpoint_t e;
                                e.type = t->valuestring;
                                e.endpoint_id = (cJSON_IsNumber(eid) ? (uint16_t)eid->valueint : 0);
                                endpoints.push_back(e);
                            }
                        }
                    }
                }
                else
                {
                    // Legacy format: "type": ["REL0","REL1"...]
                    cJSON *type_elem = NULL;
                    cJSON_ArrayForEach(type_elem, types_j)
                    {
                        if (cJSON_IsString(type_elem))
                        {
                            device_nvs_endpoint_t e;
                            e.type = type_elem->valuestring;
                            e.endpoint_id = 0;
                            endpoints.push_back(e);
                        }
                    }
                }

                cJSON_Delete(root);

                // Build device_t
                device_t dev;
                dev.uid = uid;
                dev.ip = ip;
                dev.node = node;
                dev.last_seen = 0;
                dev.reachable = false;
                dev.type_order.clear();
                dev.data = nullptr;

                // Re-attach endpoints:
                for (const auto &e : endpoints)
                {
                    auto itc = creators.find(e.type);
                    if (itc != creators.end())
                    {
                        endpoint_t *created = itc->second(node, uid + "_" + e.type);
                        if (created)
                        {
                            uint16_t created_id = endpoint::get_id(created);
                            dev.endpoints[e.type] = created;
                            dev.type_order.push_back(e.type);
                            report_reachable(created, false);
                            ESP_LOGI(TAG, "Created endpoint (no saved id) %u type '%s' for %s from NVS", created_id, e.type.c_str(), uid.c_str());
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to create endpoint type '%s' from NVS", e.type.c_str());
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "No creator registered for type: %s", e.type.c_str());
                    }
                }

                registry[uid] = dev;
                ESP_LOGI(TAG, "Loaded device %s from NVS", uid.c_str());
            }
            nvs_entry_next(&it);
        }

        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Loaded %d devices from NVS", key_count);
        return ESP_OK;
    }

    esp_err_t remove_endpoint(const std::string &uid, const std::string &type)
    {
        auto dev_it = registry.find(uid);
        if (dev_it == registry.end())
            return ESP_ERR_NOT_FOUND;

        auto &dev = dev_it->second;

        auto ep_it = dev.endpoints.find(type);
        if (ep_it == dev.endpoints.end())
            return ESP_ERR_NOT_FOUND;

        endpoint_t *ep = ep_it->second;
        if (ep)
        {
            endpoint::destroy(dev.node, ep);
        }

        dev.endpoints.erase(type);

        auto new_end = std::vector<std::string>{};
        for (auto &t : dev.type_order)
            if (t != type)
                new_end.push_back(t);
        dev.type_order.swap(new_end);

        // rebuild NVS payload including endpoint ids that remain
        device_nvs_data_t data;
        data.uid = dev.uid;
        data.ip = dev.ip;
        for (auto &t : dev.type_order)
        {
            uint16_t id = 0;
            auto it_ep = dev.endpoints.find(t);
            if (it_ep != dev.endpoints.end() && it_ep->second)
                id = endpoint::get_id(it_ep->second);
            data.endpoints.push_back({t, id});
        }

        save_device_to_nvs(data);

        ESP_LOGI(TAG, "Removed endpoint '%s' from device '%s'", type.c_str(), uid.c_str());
        return ESP_OK;
    }

    void create_or_update(const std::string &ip, node_t *node, const std::string &json_str)
    {
        cJSON *root = cJSON_Parse(json_str.c_str());
        if (!root)
        {
            ESP_LOGE(TAG, "Invalid JSON");
            return;
        }

        cJSON *uid_j = cJSON_GetObjectItem(root, "uid");
        cJSON *types_j = cJSON_GetObjectItem(root, "type");
        cJSON *cmd_j = cJSON_GetObjectItem(root, "command_support");

        if (!cJSON_IsString(uid_j) || !cJSON_IsArray(types_j))
        {
            ESP_LOGE(TAG, "Missing uid or types[]");
            cJSON_Delete(root);
            return;
        }

        std::string uid(uid_j->valuestring);
        auto &dev = registry[uid];
        dev.uid = uid;
        dev.ip = ip;
        dev.node = node;
        dev.last_seen = esp_timer_get_time() / 1000;
        dev.command_support = (!cmd_j || cJSON_IsTrue(cmd_j));

        if (!dev.reachable)
        {
            dev.reachable = true;
            for (auto &[t, ep] : dev.endpoints)
            {
                report_reachable(ep, dev.reachable);
            }
        }

        cJSON *type_elem = nullptr;
        bool saved_any = false;
        cJSON_ArrayForEach(type_elem, types_j)
        {
            if (!cJSON_IsString(type_elem))
                continue;
            std::string type(type_elem->valuestring);

            if (dev.endpoints.find(type) == dev.endpoints.end())
            {
                auto it = creators.find(type);
                if (it == creators.end())
                {
                    ESP_LOGW(TAG, "No creator for type %s", type.c_str());
                    continue;
                }

                endpoint_t *ep = nullptr;

                // Create endpoint using creator (first-time create). The created endpoint will get a stable id from esp-matter.
                ep = it->second(node, uid + "_" + type);
                if (!ep)
                {
                    ESP_LOGE(TAG, "Creator returned NULL for uid=%s type=%s", uid.c_str(), type.c_str());
                    continue;
                }

                dev.endpoints[type] = ep;
                if (std::find(dev.type_order.begin(), dev.type_order.end(), type) == dev.type_order.end())
                    dev.type_order.push_back(type);
                report_reachable(ep, true);

                // Persist endpoint id
                device_nvs_data_t data;
                data.uid = uid;
                data.ip = ip;
                for (auto &t : dev.type_order)
                {
                    uint16_t id = 0;
                    auto it_ep = dev.endpoints.find(t);
                    if (it_ep != dev.endpoints.end() && it_ep->second)
                        id = endpoint::get_id(it_ep->second);
                    data.endpoints.push_back({t, id});
                }
                save_device_to_nvs(data);
                saved_any = true;
            }
        }

        // optional: log when we persisted
        if (saved_any)
        {
            ESP_LOGI(TAG, "Persisted device %s endpoints to NVS", uid.c_str());
        }

        // existing data handling unchanged...
        cJSON *data = cJSON_GetObjectItem(root, "data");
        if (data)
        {
            cJSON *type_item = nullptr;
            cJSON_ArrayForEach(type_item, data)
            {
                std::string type = type_item->string;
                cJSON *field = cJSON_GetObjectItem(type_item, "0");
                if (!field)
                    continue;

                auto it_ep = dev.endpoints.find(type);
                auto it_map = type_map.find(type);
                if (it_ep == dev.endpoints.end() || it_map == type_map.end())
                    continue;

                endpoint_t *ep = it_ep->second;
                auto &map = it_map->second;
                uint16_t ep_id = endpoint::get_id(ep);

                auto attr = attribute::get(ep_id, map.cluster_id, map.attribute_id);
                if (!attr)
                    continue;

                esp_matter_attr_val_t val{};

                if (cJSON_IsBool(field))
                {
                    val = esp_matter_bool(cJSON_IsTrue(field));
                }
                else if (cJSON_IsNumber(field))
                {
                    float raw = static_cast<float>(field->valuedouble);
                    float scaled = raw * map.multiplier;

                    if (type == "LUMI")
                    {
                        float lux = (field->valuedouble / 65535.0f) * 1000.0f;
                        float measured = 10000.0f * log10f(lux + 1.0f) + 1.0f;
                        int16_t int_val = (int16_t)roundf(measured);
                        val = esp_matter_int16(int_val);
                    }
                    else
                    {
                        if (scaled > 32767.0f)
                            scaled = 32767.0f;
                        if (scaled < 0.0f)
                            scaled = 0.0f;

                        int16_t int_val = static_cast<int16_t>(roundf(scaled));
                        val = esp_matter_int16(int_val);
                    }
                }

                attribute::set_val(attr, &val);
                attribute::report(ep_id, map.cluster_id, map.attribute_id, &val);
            }
        }

        cJSON_Delete(root);
    }

    void send_udp_json(const std::string &ip, const std::string &payload)
    {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0)
            return;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        int32_t command_port = webgui::cfg.command_port;
        addr.sin_port = htons(command_port);
        inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
        sendto(sock, payload.c_str(), payload.size(), 0, (sockaddr *)&addr, sizeof(addr));
        close(sock);
    }

    esp_err_t handle_attribute_update(uint16_t ep, uint32_t cluster, uint32_t attr, esp_matter_attr_val_t *val)
    {
        if (!val)
            return ESP_ERR_INVALID_ARG;

        for (auto &[uid, dev] : registry)
        {
            if (!dev.command_support)
                continue;

            for (auto &[type, endpoint] : dev.endpoints)
            {
                if (!endpoint)
                {
                    ESP_LOGW(TAG, "Endpoint NULL for type %s, skipping", type.c_str());
                    continue;
                }

                if (endpoint::get_id(endpoint) != ep)
                    continue;

                auto it_map = type_map.find(type);
                if (it_map == type_map.end())
                {
                    ESP_LOGW(TAG, "Type %s not in type_map", type.c_str());
                    continue; // seguimos buscando otros endpoints
                }

                cJSON *root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "uid", uid.c_str());
                cJSON_AddBoolToObject(root, "command_support", dev.command_support);

                cJSON *type_arr = cJSON_CreateArray();
                for (auto &t : dev.type_order)
                    cJSON_AddItemToArray(type_arr, cJSON_CreateString(t.c_str()));
                cJSON_AddItemToObject(root, "type", type_arr);

                cJSON *data = cJSON_CreateObject();
                cJSON *inner = cJSON_CreateObject();

                switch (val->type)
                {

                case ESP_MATTER_VAL_TYPE_BOOLEAN:
                    // Booleano: directo
                    cJSON_AddBoolToObject(inner, "0", val->val.b);
                    break;

                case ESP_MATTER_VAL_TYPE_INT16:
                    // Enteros con signo: enviar tal cual
                    cJSON_AddNumberToObject(inner, "0", val->val.i16);
                    break;

                case ESP_MATTER_VAL_TYPE_UINT16:
                    // Enteros sin signo: enviar tal cual
                    cJSON_AddNumberToObject(inner, "0", (int)val->val.u16);
                    break;

                case ESP_MATTER_VAL_TYPE_FLOAT:
                {
                    float rounded = roundf(val->val.f * 100.0f) / 100.0f; // redondea a 2 decimales
                    cJSON_AddNumberToObject(inner, "0", rounded);
                    break;
                }

                default:
                    ESP_LOGW(TAG, "Unhandled value type %d", val->type);
                    break;
                }

                cJSON_AddItemToObject(data, type.c_str(), inner);
                cJSON_AddItemToObject(root, "data", data);

                char *json = cJSON_PrintUnformatted(root);
                if (json)
                {
                    send_udp_json(dev.ip, std::string(json));
                    ESP_LOGI(TAG, "Sent JSON to %s: %s", dev.ip.c_str(), json);
                    cJSON_free(json);
                }

                cJSON_Delete(root);
            }
        }

        return ESP_OK;
    }

    void init_LUMI_type()
    {
        register_device_type("LUMI", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                light_sensor::config_t light_sensor_config;
                                auto epL = endpoint::light_sensor::create(n, &light_sensor_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);                   
                                cluster::bridged_device_basic_information::config_t basic_info_cfgL{};
                                cluster_t *basic_clL = cluster::bridged_device_basic_information::create(epL, &basic_info_cfgL, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_clL, "Luminosity", strlen("Luminosity"));
                                endpoint::enable(epL);
                                return epL; });
    }

    void init_TEMP_type()
    {
        register_device_type("TEMP", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                temperature_sensor::config_t temperature_sensor_config;
                                auto epT = endpoint::temperature_sensor::create(n, &temperature_sensor_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfgT{};
                                cluster_t *basic_clT = cluster::bridged_device_basic_information::create(epT, &basic_info_cfgT, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_clT, "Temperature", strlen("Temperature"));                                   
                                endpoint::enable(epT);
                                return epT; });
    }

    void init_HUMI_type()
    {
        register_device_type("HUMI", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                humidity_sensor::config_t humidity_sensor_config;
                                auto epH = endpoint::humidity_sensor::create(n, &humidity_sensor_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfgH{};
                                cluster_t *basic_clH = cluster::bridged_device_basic_information::create(epH, &basic_info_cfgH, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_clH, "Humidity", strlen("Humidity"));
                                endpoint::enable(epH);
                                return epH; });
    }

    void init_REL0_type()
    {
        register_device_type("REL0", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t cfg;
                                auto ep0 = endpoint::on_off_plugin_unit::create(n, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t bicfg{};
                                cluster_t *basic = cluster::bridged_device_basic_information::create(ep0, &bicfg, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL0";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep0);
                                return ep0; });
    }

    void init_REL1_type()
    {
        register_device_type("REL1", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config1;    
                                auto ep1 = endpoint::on_off_plugin_unit::create(n, &on_off_config1, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg1{};
                                cluster_t *basic_cl1 = cluster::bridged_device_basic_information::create(ep1, &basic_info_cfg1, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl1, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL1";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl1,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep1);
                                return ep1; });
    }

    void init_REL2_type()
    {
        register_device_type("REL2", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config2;    
                                auto ep0 = endpoint::on_off_plugin_unit::create(n, &on_off_config2, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg2{};
                                cluster_t *basic_cl2 = cluster::bridged_device_basic_information::create(ep0, &basic_info_cfg2, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl2, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL1";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl2,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep0);
                                return ep0; });
    }

    void init_REL3_type()
    {
        register_device_type("REL3", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config;    
                                auto ep3 = endpoint::on_off_plugin_unit::create(n, &on_off_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL3";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl3,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_REL4_type()
    {
        register_device_type("REL4", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config;    
                                auto ep4 = endpoint::on_off_plugin_unit::create(n, &on_off_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg4{};
                                cluster_t *basic_cl4 = cluster::bridged_device_basic_information::create(ep4, &basic_info_cfg4, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl4, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL4";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl4,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep4);
                                return ep4; });
    }

    void init_REL5_type()
    {
        register_device_type("REL5", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config;    
                                auto ep5 = endpoint::on_off_plugin_unit::create(n, &on_off_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg5{};
                                cluster_t *basic_cl5 = cluster::bridged_device_basic_information::create(ep5, &basic_info_cfg5, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl5, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL5";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl5,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep5);
                                return ep5; });
    }

    void init_REL6_type()
    {
        register_device_type("REL6", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config;    
                                auto ep6 = endpoint::on_off_plugin_unit::create(n, &on_off_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg6{};
                                cluster_t *basic_cl6 = cluster::bridged_device_basic_information::create(ep6, &basic_info_cfg6, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl6, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL6";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl6,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep6);
                                return ep6; });
    }

    void init_REL7_type()
    {
        register_device_type("REL7", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config;    
                                auto ep7 = endpoint::on_off_plugin_unit::create(n, &on_off_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg7{};
                                cluster_t *basic_cl7 = cluster::bridged_device_basic_information::create(ep7, &basic_info_cfg7, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl7, "Outlet", strlen("Outlet"));
                                std::string unique_id = uid + "_REL7";
                                cluster::bridged_device_basic_information::attribute::create_unique_id(
                                    basic_cl7,
                                    const_cast<char *>(unique_id.c_str()),
                                    unique_id.length()
                                );
                                endpoint::enable(ep7);
                                return ep7; });
    }

    void init_DIM0_type()
    {
        register_device_type("DIM0", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM1_type()
    {
        register_device_type("DIM1", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM2_type()
    {
        register_device_type("DIM2", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM3_type()
    {
        register_device_type("DIM3", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM4_type()
    {
        register_device_type("DIM4", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM5_type()
    {
        register_device_type("DIM5", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM6_type()
    {
        register_device_type("DIM6", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_DIM7_type()
    {
        register_device_type("DIM7", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                dimmable_plugin_unit::config_t dimmable_plugin_config;
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, &dimmable_plugin_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_types()
    {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            nvs_flash_erase();
            nvs_flash_init();
        }
        init_TEMP_type();
        init_HUMI_type();
        init_LUMI_type();
        init_REL0_type();
        init_REL1_type();
        init_REL2_type();
        init_REL3_type();
        init_REL4_type();
        init_REL5_type();
        init_REL6_type();
        init_REL7_type();
        init_DIM0_type();
        init_DIM1_type();
        init_DIM2_type();
        init_DIM3_type();
        init_DIM4_type();
        init_DIM5_type();
        init_DIM6_type();
        init_DIM7_type();
    }

} // namespace devices
