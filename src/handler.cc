#include "handler.h"

#include <fstream>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>


void Handler::dump(const Config &config, const std::string &in_file, const std::string &out_file) {
    ROS_INFO("in file: %s", in_file.c_str());
    ROS_INFO("out file: %s", out_file.c_str());

    std::unordered_map<std::string, std::string> camera_topic_map;
    config.get_topic_map_by_sensor_type(CAMERA, camera_topic_map);
    ROS_INFO("camera topic number: %lu", camera_topic_map.size());

    std::fstream fin(in_file, std::ios::in | std::ios::binary);
    common::Chunk chunk;
    chunk.ParseFromIstream(&fin);
    fin.close();

    rosbag::Bag out_bag;
    out_bag.open(out_file, rosbag::bagmode::Write);
    drivers::CompressedImage comp_img;
    for (int32_t i = 0; i < chunk.messages_size(); ++i) {
        const auto &msg = chunk.messages(i);
        ROS_INFO("msg_id=%d", i);
        if ("camera" == msg.sensor_name()) {
            comp_img.ParseFromString(msg.content());
            auto it = camera_topic_map.find(comp_img.frame_id());
            if (it != camera_topic_map.end() && "jpeg" == comp_img.format()) {
                handleCameraData(comp_img, it->second, out_bag);
            }
        }
    }
    out_bag.close();
}

void Handler::handleCameraData(const drivers::CompressedImage &comp_img,
        const std::string &out_topic, rosbag::Bag &out_bag) {
    sensor_msgs::Image image_msg;
    std_msgs::Header header;
    header.seq = comp_img.header().sequence_num();
    header.stamp = ros::Time(static_cast<double>(comp_img.header().camera_timestamp()) * 1e-9);
    header.frame_id = comp_img.frame_id();
    ROS_INFO("frame_id=%s seq=%u", header.frame_id.c_str(), header.seq);

    cv::Mat jpeg_data(1, comp_img.data().size(), CV_8UC1);
    jpeg_data.data = reinterpret_cast<uchar*>(const_cast<char*>(comp_img.data().c_str()));
    cv::InputArray cv_jpeg_data(jpeg_data);
    cv::Mat decoded_image = cv::imdecode(cv_jpeg_data, cv::IMREAD_COLOR);
    // cv::imshow("debug", decoded_image);
    // cv::waitKey(0);
    auto image_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, decoded_image);
    image_bridge.toImageMsg(image_msg);

    out_bag.write(out_topic, ros::Time(comp_img.header().timestamp_sec() * 1e-3), image_msg);
}

