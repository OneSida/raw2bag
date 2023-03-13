#ifndef HANDLER_H
#define HANDLER_H

#include "config.h"

#include <rosbag/bag.h>

class Handler {
public:
    int work(const Config &config, const std::string &in_file, const std::string &out_file);
    void handleCameraData(const std::string &out_topic, rosbag::Bag &out_bag);
};

#endif
