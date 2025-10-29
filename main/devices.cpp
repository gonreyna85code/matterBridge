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

using namespace esp_matter;
using namespace esp_matter::endpoint;

namespace devices
{

    static const char *TAG = "DEVICES";
    static const char *NVS_NAMESPACE = "devices";

    struct device_t
    {
        std::string uid;
        std::string ip;
        node_t *node;
        time_t last_seen;
        bool reachable;
        bool command_support = true;
        std::map<std::string, endpoint_t *> endpoints;
        std::vector<std::string> type_order; // <-- NEW: preserve creation order
        cJSON *data = nullptr;
    };

    struct device_nvs_data_t
    {
        std::string uid;
        std::string ip;
        std::vector<std::string> type;
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
        {"TEMP", {chip::app::Clusters::TemperatureMeasurement::Id, chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id, 100.0f}},
        {"HUMI", {chip::app::Clusters::RelativeHumidityMeasurement::Id, chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, 100.0f}},
        {"DIMM", {chip::app::Clusters::LevelControl::Id, chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id, 1.0f}},
        {"LUMI", {chip::app::Clusters::IlluminanceMeasurement::Id, chip::app::Clusters::IlluminanceMeasurement::Attributes::MeasuredValue::Id, 1.0f}},
    };

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

        cJSON *types = cJSON_CreateArray();
        for (auto &t : device_data.type)
            cJSON_AddItemToArray(types, cJSON_CreateString(t.c_str()));
        cJSON_AddItemToObject(root, "type", types);

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
            return ESP_OK; // This is not an error, just means that there are no devices saved yet
        }

        // Iterate over all keys in the namespace
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

                // Read the JSON string from NVS
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

                // Parse the JSON string
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
                cJSON *types_j = cJSON_GetObjectItem(root, "type");

                if (!cJSON_IsString(uid_j) || !cJSON_IsString(ip_j) || !cJSON_IsArray(types_j))
                {
                    ESP_LOGE(TAG, "Invalid JSON format in NVS");
                    cJSON_Delete(root);
                    nvs_close(nvs_handle);
                    return ESP_FAIL;
                }

                std::string uid = uid_j->valuestring;
                std::string ip = ip_j->valuestring;
                std::vector<std::string> type;

                cJSON *type_elem = NULL;
                cJSON_ArrayForEach(type_elem, types_j)
                {
                    if (cJSON_IsString(type_elem))
                    {
                        type.push_back(type_elem->valuestring);
                    }
                }

                cJSON_Delete(root);

                // Create the device and endpoints
                device_t dev;
                dev.uid = uid;
                dev.ip = ip;
                dev.node = node;
                dev.last_seen = 0;
                dev.reachable = false;
                dev.type_order = type;
                dev.data = nullptr;

                // Create endpoints
                for (const auto &type : type)
                {
                    auto it = creators.find(type);
                    if (it != creators.end())
                    {
                        endpoint_t *ep = it->second(node, uid + "_" + type); // Node is not available yet
                        if (ep)
                        {
                            dev.endpoints[type] = ep;
                            report_reachable(ep, false);
                            ESP_LOGI(TAG, "Created endpoint type '%s' for %s from NVS", type.c_str(), uid.c_str());
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to create endpoint type '%s' from NVS", type.c_str());
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "No creator registered for type: %s", type.c_str());
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

    // =============================================================
    // 🧩 CREAR / ACTUALIZAR DISPOSITIVO
    // =============================================================
    void create_or_update(const std::string &ip, node_t *node, const std::string &json_str)
    {
        ESP_LOGW(TAG, "Received JSON: %s", json_str.c_str());
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
                report_reachable(ep, true);
        }

        cJSON *type_elem = nullptr;
        cJSON_ArrayForEach(type_elem, types_j)
        {
            if (!cJSON_IsString(type_elem))
                continue;
            std::string type(type_elem->valuestring);

            if (dev.endpoints.find(type) == dev.endpoints.end())
            {
                auto it = creators.find(type);
                if (it == creators.end())
                    continue;

                endpoint_t *ep = it->second(node, uid + "_" + type);
                if (!ep)
                    continue;

                dev.endpoints[type] = ep;
                if (std::find(dev.type_order.begin(), dev.type_order.end(), type) == dev.type_order.end())
                    dev.type_order.push_back(type); // <-- preserve insertion order
                report_reachable(ep, true);

                device_nvs_data_t data;
                data.uid = uid;
                data.ip = ip;
                for (auto &t : dev.type_order) // <-- use type_order for NVS
                    data.type.push_back(t);
                save_device_to_nvs(data);
            }
        }

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
                    val = esp_matter_bool(cJSON_IsTrue(field));
                else if (cJSON_IsNumber(field))
                    val = esp_matter_int16((int16_t)(field->valuedouble * map.multiplier));

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
        addr.sin_port = htons(12346);
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

                const auto &map = it_map->second;
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
                    cJSON_AddBoolToObject(inner, "0", val->val.b);
                    break;
                case ESP_MATTER_VAL_TYPE_INT16:
                    cJSON_AddNumberToObject(inner, "0", (float)val->val.i16 / map.multiplier);
                    break;
                case ESP_MATTER_VAL_TYPE_UINT16:
                    cJSON_AddNumberToObject(inner, "0", (float)val->val.u16 / map.multiplier);
                    break;
                default:
                    ESP_LOGW(TAG, "Unhandled value type %d", val->type);
                    break;
                }

                cJSON_AddItemToObject(data, type.c_str(), inner);
                cJSON_AddItemToObject(root, "data", data);

                char *json = cJSON_PrintUnformatted(root);
                send_udp_json(dev.ip, std::string(json));
                ESP_LOGI(TAG, "Sent JSON to %s: %s", dev.ip.c_str(), json);

                cJSON_free(json);
                cJSON_Delete(root);

                // NO hacemos return aquí: seguimos iterando para endpoints anidados
            }
        }

        return ESP_OK;
    }

    // =============================================================
    // 🔧 REGISTRO DE TIPOS
    // =============================================================
    void init_REL0_type()
    {
        register_device_type("REL0", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                on_off_plugin_unit::config_t on_off_config;    
                                auto ep0 = endpoint::on_off_plugin_unit::create(n, &on_off_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg0{};
                                cluster_t *basic_cl0 = cluster::bridged_device_basic_information::create(ep0, &basic_info_cfg0, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl0, "Outlet", strlen("Outlet"));
                                endpoint::enable(ep0);
                                return ep0; });
    }

    void init_TEMP_type()
    {
        register_device_type("TEMP", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                temperature_sensor::config_t temperature_sensor_config;
                                auto ep1 = endpoint::temperature_sensor::create(n, &temperature_sensor_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg1{};
                                cluster_t *basic_cl1 = cluster::bridged_device_basic_information::create(ep1, &basic_info_cfg1, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl1, "Temperature", strlen("Temperature"));                                   
                                endpoint::enable(ep1);
                                return ep1; });
    }

    void init_HUMI_type()
    {
        register_device_type("HUMI", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                humidity_sensor::config_t humidity_sensor_config;
                                auto ep2 = endpoint::humidity_sensor::create(n, &humidity_sensor_config, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg2{};
                                cluster_t *basic_cl2 = cluster::bridged_device_basic_information::create(ep2, &basic_info_cfg2, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl2, "Humidity", strlen("Humidity"));
                                endpoint::enable(ep2);
                                return ep2; });
    }

    void init_DIMM_type()
    {
        register_device_type("DIMM", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                auto ep3 = endpoint::dimmable_plugin_unit::create(n, nullptr, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg3{};
                                cluster_t *basic_cl3 = cluster::bridged_device_basic_information::create(ep3, &basic_info_cfg3, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl3, "Dimmer", strlen("Dimmer"));
                                endpoint::enable(ep3);
                                return ep3; });
    }

    void init_LUMI_type()
    {
        register_device_type("LUMI", [](node_t *n, const std::string &uid) -> endpoint_t *
                             {
                                auto ep4 = endpoint::light_sensor::create(n, nullptr, ENDPOINT_FLAG_DESTROYABLE, nullptr);
                                cluster::bridged_device_basic_information::config_t basic_info_cfg4{};
                                cluster_t *basic_cl4 = cluster::bridged_device_basic_information::create(ep4, &basic_info_cfg4, CLUSTER_FLAG_SERVER);
                                cluster::bridged_device_basic_information::attribute::create_product_name(basic_cl4, "Luminosity", strlen("Luminosity"));
                                endpoint::enable(ep4);
                                return ep4; });
    }

    void init_types()
    {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            nvs_flash_erase();
            nvs_flash_init();
        }
        init_REL0_type();
        init_TEMP_type();
        init_HUMI_type();
        init_DIMM_type();
        init_LUMI_type();
    }

} // namespace devices
