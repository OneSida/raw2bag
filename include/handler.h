#ifndef HANDLER_H
#define HANDLER_H

#include "config.h"
#include "message.pb.h"
#include "sensor_image.pb.h"

#include <rosbag/bag.h>

class Handler {
public:
    void dump(const Config &config, const std::string &in_file, const std::string &out_file);
    void handleCameraData(const drivers::CompressedImage &comp_img,
        const std::string &out_topic, rosbag::Bag &out_bag);
};

#endif
