#include <map>
#include <string>
#include <functional>
#include <esp_matter.h>
#include <esp_log.h>
#include <cJSON.h>
#include <ctime>
#include <nvs_flash.h>
#include <nvs.h>

using namespace esp_matter;

namespace devices
{

    static const char *TAG = "DEVICES";
    static const char *NVS_NAMESPACE = "devices";

    // =============================================================
    // 🔧 ESTRUCTURAS Y TIPOS
    // =============================================================
    struct device_t
    {
        std::string uid;
        std::string ip;
        node_t *node;
        time_t last_seen;
        bool reachable;
        std::map<std::string, endpoint_t *> endpoints; // type → endpoint
        cJSON *data = nullptr;
    };

    // Structure to store device data in NVS
    struct device_nvs_data_t
    {
        std::string uid;
        std::string ip;
        std::vector<std::string> type; // List of endpoint types
    };

    // Handler para cada tipo de dispositivo
    using creator_t = std::function<endpoint_t *(node_t *node, const std::string &uid)>;

    // =============================================================
    // 🔧 REGISTROS INTERNOS
    // =============================================================
    std::map<std::string, device_t> registry;
    static std::map<std::string, creator_t> creators;

    // =============================================================
    // 🧠 REGISTROS PÚBLICOS
    // =============================================================
    void register_device_type(const std::string &type, creator_t fn)
    {
        creators[type] = fn;
        ESP_LOGI(TAG, "Registered device type: %s", type.c_str());
    }

    // =============================================================
    // 💾 NVS FUNCTIONS
    // =============================================================
    esp_err_t save_device_to_nvs(const device_nvs_data_t &device_data)
    {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) opening NVS namespace!", esp_err_to_name(err));
            return err;
        }

        // Create a key for this device (e.g., "device_uid")
        std::string key = "device_" + device_data.uid;

        // Serialize the device data to JSON
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "uid", device_data.uid.c_str());
        cJSON_AddStringToObject(root, "ip", device_data.ip.c_str());

        cJSON *types_array = cJSON_CreateArray();
        for (const auto &type : device_data.type)
        {
            cJSON_AddItemToArray(types_array, cJSON_CreateString(type.c_str()));
        }
        cJSON_AddItemToObject(root, "type", types_array);

        char *json_str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        // Store the JSON string in NVS
        err = nvs_set_str(nvs_handle, key.c_str(), json_str);
        free(json_str);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) writing device data to NVS!", esp_err_to_name(err));
        }

        // Commit changes
        esp_err_t commit_err = nvs_commit(nvs_handle);
        if (commit_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) committing NVS changes!", esp_err_to_name(commit_err));
        }

        // Close
        nvs_close(nvs_handle);
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
            esp_matter::attribute::set_val(attr, &val);
            esp_matter::attribute::report(
                esp_matter::endpoint::get_id(ep),
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
            ESP_LOGE(TAG, "Invalid JSON: %s", json_str.c_str());
            return;
        }

        const cJSON *uid_j = cJSON_GetObjectItem(root, "uid");
        const cJSON *types_j = cJSON_GetObjectItem(root, "type"); // <-- This is now correct
        // const cJSON *data_j = cJSON_GetObjectItem(root, "data");

        if (!cJSON_IsString(uid_j) || !cJSON_IsArray(types_j))
        {
            ESP_LOGE(TAG, "Missing uid or types[] in JSON");
            cJSON_Delete(root);
            return;
        }

        std::string uid(uid_j->valuestring);
        auto &dev = registry[uid];
        dev.uid = uid;
        dev.ip = ip;
        dev.node = node;
        dev.last_seen = esp_timer_get_time() / 1000; // <--- siempre actualizar
        if (!dev.reachable)
        {
            dev.reachable = true;
            ESP_LOGW(TAG, "Device %s is now ONLINE", uid.c_str());
            for (auto &[type, ep] : dev.endpoints)
                report_reachable(ep, true);
        }

        // --- Procesar cada tipo ---
        cJSON *type_elem = nullptr;
        cJSON_ArrayForEach(type_elem, types_j)
        {
            if (!cJSON_IsString(type_elem))
                continue;
            std::string type(type_elem->valuestring);

            if (dev.endpoints.find(type) == dev.endpoints.end())
            {
                ESP_LOGW(TAG, "CHECKING ENDPOINT");
                auto it = creators.find(type);
                if (it == creators.end())
                {
                    ESP_LOGW(TAG, "No creator registered for type: %s", type.c_str());
                    continue;
                }

                endpoint_t *ep = it->second(node, uid + "_" + type);
                if (!ep)
                {
                    ESP_LOGE(TAG, "Creator returned null or failed for type '%s'", type.c_str());
                    continue;
                }

                ESP_LOGW(TAG, "CREATING ENDPOINT");
                if (ep)
                {
                    dev.endpoints[type] = ep;
                    ESP_LOGW(TAG, "Created endpoint type '%s' for %s", type.c_str(), uid.c_str());
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to create endpoint type '%s'", type.c_str());
                }
            }
        }

        device_nvs_data_t device_data;
        device_data.uid = uid;
        device_data.ip = ip;
        device_data.type.clear();

        cJSON_ArrayForEach(type_elem, types_j)
        {
            if (cJSON_IsString(type_elem))
                device_data.type.push_back(type_elem->valuestring);
        }
        cJSON_Delete(root);
        save_device_to_nvs(device_data);
    }

    esp_err_t handle_attribute_update(uint16_t ep, uint32_t cluster, uint32_t attr, esp_matter_attr_val_t *val)
    {
        for (auto &[uid, dev] : registry)
        {
            for (auto &[type, endpoint] : dev.endpoints)
            {
                if (endpoint::get_id(endpoint) == ep)
                {
                    ESP_LOGI(TAG, "Attribute update for device %s, type %s", uid.c_str(), type.c_str());
                    // Aquí podrías agregar lógica específica por tipo o atributo
                    return ESP_OK;
                }
            }
        }
        return ESP_ERR_NOT_FOUND;
    }

    // =============================================================
    // 💡 EJEMPLOS DE TIPOS (podés moverlos a archivos separados)
    // =============================================================
    void init_REL0_type()
    {
        register_device_type("REL0", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::on_off_plugin_unit::config_t cfg{};
        endpoint_t *ep = endpoint::on_off_plugin_unit::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        cluster::on_off::config_t onoff_cfg{};
        cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    void init_REL1_type()
    {
        register_device_type("REL1", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::on_off_plugin_unit::config_t cfg{};
        endpoint_t *ep = endpoint::on_off_plugin_unit::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        cluster::on_off::config_t onoff_cfg{};
        cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    void init_REL2_type()
    {
        register_device_type("REL2", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::on_off_plugin_unit::config_t cfg{};
        endpoint_t *ep = endpoint::on_off_plugin_unit::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        cluster::on_off::config_t onoff_cfg{};
        cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    void init_REL3_type()
    {
        register_device_type("REL3", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::on_off_plugin_unit::config_t cfg{};
        endpoint_t *ep = endpoint::on_off_plugin_unit::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        cluster::on_off::config_t onoff_cfg{};
        cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);
        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    void init_TEMP_type()
    {
        register_device_type("TEMP", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::temperature_sensor::config_t cfg{};
        endpoint_t *ep = endpoint::temperature_sensor::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        cluster::temperature_measurement::config_t temp_cfg{};
        cluster::temperature_measurement::create(ep, &temp_cfg, CLUSTER_FLAG_SERVER);

        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    void init_HUMI_type()
    {
        register_device_type("HUMI", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::humidity_sensor::config_t cfg{};
        endpoint_t *ep = endpoint::humidity_sensor::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        cluster::relative_humidity_measurement::config_t hum_cfg{};
        cluster::relative_humidity_measurement::create(ep, &hum_cfg, CLUSTER_FLAG_SERVER);

        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    void init_DIMM_type()
    {
        register_device_type("DIMM", [](node_t *node, const std::string &uid) -> endpoint_t *
                             {
        endpoint::dimmable_light::config_t cfg{};
        endpoint_t *ep = endpoint::dimmable_light::create(node, &cfg, ENDPOINT_FLAG_DESTROYABLE, nullptr);

        // Clusters principales
        cluster::on_off::config_t onoff_cfg{};
        cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_SERVER);

        cluster::level_control::config_t level_cfg{};
        cluster::level_control::create(ep, &level_cfg, CLUSTER_FLAG_SERVER);

        // Información básica
        cluster::bridged_device_basic_information::config_t bi{};
        cluster::bridged_device_basic_information::create(ep, &bi, CLUSTER_FLAG_SERVER);

        // NodeLabel (UID visible)
        uint16_t ep_id = endpoint::get_id(ep);
        auto *attr = attribute::get(
            ep_id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Id,
            chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_char_str((char *)uid.c_str(), uid.size());
            attribute::set_val(attr, &val);
            attribute::report(ep_id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Id,
                              chip::app::Clusters::BridgedDeviceBasicInformation::Attributes::NodeLabel::Id,
                              &val);
        }

        endpoint::enable(ep);
        return ep; });
    }

    // =============================================================
    // 🏁 INIT DE TODOS LOS TIPOS
    // =============================================================
    void init_types()
    {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
        init_REL0_type();
        init_REL1_type();
        init_REL2_type();
        init_REL3_type();
        init_TEMP_type();
        init_HUMI_type();
        init_DIMM_type();
    }
} // namespace devices