#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <unordered_map>

enum SensorType {
    NONE = 0,
    CAMERA = 1,
    LIDAR = 2
};

struct SensorInfo {
    SensorType type;
    std::string in_topic;
    std::string out_topic;
};

class Config {
public:
    int32_t init(const std::string &config_file);
    void get_topic_map_by_sensor_type(const SensorType sensor_type,
        std::unordered_map<std::string, std::string> &topic_map) const;

private:
    std::unordered_map<std::string, SensorInfo> config_map_;
};

#endif
