#include "config.h"
#include "handler.h"

#include <chrono>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>


int32_t main(int32_t argc, char **argv) {
    ROS_INFO("Start!");
    ros::Time::init();
    auto start_time = std::chrono::steady_clock::now();

    std::string config_file = argv[1];
    Config config;
    int32_t ret = config.init(config_file);
    if (0 != ret) {
        return -1;
    }

    std::vector<std::string> in_files;
    cv::glob(argv[2], in_files, false);
    std::string out_file = argv[3];
    Handler handler;
    handler.dump(config, in_files, out_file);

    auto finish_time = std::chrono::steady_clock::now();
    ROS_INFO("time elapsed: %.2lf sec", std::chrono::duration_cast<
        std::chrono::duration<double>>(finish_time - start_time).count());
    
    ROS_INFO("Finish!");
    return 0;
}

