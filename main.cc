#include "config.h"
#include "handler.h"

#include <chrono>
#include <ros/console.h>


int main(int argc, char **argv) {
    ROS_INFO("Start!");
    ros::Time::init();
    auto start_time = std::chrono::steady_clock::now();

    std::string root_path = "/root/ros_workspace/raw2bag_workspace/src/raw2bag";
    std::string config_name = "demo";

    std::string config_file = root_path + "/conf/" + config_name + ".json";
    Config config;
    int ret = config.init(config_file);
    if (0 != ret) {
        return -1;
    }

    std::string in_file = root_path + "/data/" + config_name + ".bin";
    std::string out_file = root_path + "/data/" + config_name + ".bag";
    Handler handler;
    handler.work(config, in_file, out_file);
    if (0 != ret) {
        return -1;
    }

    auto finish_time = std::chrono::steady_clock::now();
    ROS_INFO("time elapsed: %.2lf sec", std::chrono::duration_cast<
        std::chrono::duration<double>>(finish_time - start_time).count());
    
    ROS_INFO("Finish!");
    return 0;
}

