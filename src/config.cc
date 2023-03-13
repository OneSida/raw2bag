#include "config.h"
#include "nlohmann/json.hpp"

#include <fstream>
#include <ros/console.h>


int Config::init(const std::string &config_file) {
    std::ifstream fin(config_file);
    if (!fin.good()) {
        ROS_ERROR("Failed to open config file");
        return -1;
    }
    nlohmann::json json_data;
    fin >> json_data;
    fin.close();

    for (auto it = json_data.begin(); it != json_data.end(); ++it) {
        std::string sensor_name = it.key();
        if (sensor_name.empty()
                || config_map_.find(sensor_name) != config_map_.end()) {
            ROS_ERROR("sensor name error");
            return -1;
        }
        nlohmann::json sensor_config = it.value();
        SensorInfo sensor_info;
        if (!sensor_config.contains("type")) {
            ROS_ERROR("sensor type error");
            return -1;
        }
        if ("camera" == sensor_config["type"]) {
            sensor_info.type = CAMERA;
        } else {
            return -1;
        }
        if (!sensor_config.contains("in_topic")
                || sensor_config["in_topic"].empty()) {
            ROS_ERROR("input topic error");
            return -1;
        }
        sensor_info.in_topic = sensor_config["in_topic"];
        if (!sensor_config.contains("out_topic")
                || sensor_config["out_topic"].empty()) {
            ROS_ERROR("ouptut topic error");
            return -1;
        }
        sensor_info.out_topic = sensor_config["out_topic"];
        config_map_[sensor_name] = std::move(sensor_info);
    }

    return 0;
}

void Config::get_topic_map_by_sensor_type(const SensorType sensor_type,
        std::unordered_map<std::string, std::string> &topic_map) const {
    topic_map.clear();
    for (auto it = config_map_.begin(); it != config_map_.end(); ++it) {
        const auto &sensor_info = it->second;
        if (sensor_type == sensor_info.type) {
            topic_map[sensor_info.in_topic] = sensor_info.out_topic;
        }
    }
}
