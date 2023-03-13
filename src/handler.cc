#include "handler.h"

#include <ros/console.h>
#include <sensor_msgs/Image.h>


int Handler::work(const Config &config, const std::string &in_file, const std::string &out_file) {
    std::unordered_map<std::string, std::string> camera_topic_map;
    config.get_topic_map_by_sensor_type(CAMERA, camera_topic_map);
    ROS_INFO("camera topic number: %lu", camera_topic_map.size());

    char buf[1024];
    FILE *fp;
    fp = fopen(in_file.c_str(), "rb");
	int len = fread(buf, 1, 1024, fp);
	ROS_INFO("fread len: %d", len);
    
    rosbag::Bag out_bag;
    out_bag.open(out_file, rosbag::bagmode::Write);
    handleCameraData("/test", out_bag);

    // while (!fin.eof()) {
    //     auto it = camera_topic_map.find(topic);
    //     if (it != camera_topic_map.end()) {
    //         handleCameraData(it->second, out_bag);
    //         continue;
    //     }
    // }

    out_bag.close();
    fclose(fp);

    return 0;
}

void Handler::handleCameraData(const std::string &out_topic, rosbag::Bag &out_bag) {
    sensor_msgs::Image image_msg;
    out_bag.write(out_topic, ros::Time::now(), image_msg);
}

