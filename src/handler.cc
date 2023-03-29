#include "handler.h"

#include <fstream>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl_ros/point_cloud.h>


void Handler::dump(const Config &config, const std::vector<std::string> &in_files,
        const std::string &out_file) {
    ROS_INFO("out file: %s", out_file.c_str());

    std::unordered_map<std::string, std::string> camera_topic_map;
    config.get_topic_map_by_sensor_type(CAMERA, camera_topic_map);
    ROS_INFO("camera topic number: %lu", camera_topic_map.size());

    std::unordered_map<std::string, std::string> lidar_topic_map;
    config.get_topic_map_by_sensor_type(LIDAR, lidar_topic_map);
    ROS_INFO("lidar topic number: %lu", lidar_topic_map.size());

    rosbag::Bag out_bag;
    out_bag.open(out_file, rosbag::bagmode::Write);
    drivers::CompressedImage comp_img;
    drivers::PointCloud pcd;

    for (const auto &in_file : in_files) {
        std::fstream fin(in_file, std::ios::in | std::ios::binary);
        common::Chunk chunk;
        chunk.ParseFromIstream(&fin);
        fin.close();
        for (int32_t i = 0; i < chunk.messages_size(); ++i) {
            const auto &msg = chunk.messages(i);
            // ROS_INFO("msg_id=%d sensor_name=%s", i, msg.sensor_name().c_str());
            if ("camera" == msg.sensor_name()) {
                comp_img.ParseFromString(msg.content());
                auto it = camera_topic_map.find(comp_img.frame_id());
                if (it != camera_topic_map.end() && "jpeg" == comp_img.format()) {
                    ROS_INFO("msg_id=%d sensor_name=%s frame_id=%s",
                        i, msg.sensor_name().c_str(), comp_img.frame_id().c_str());
                    handleCameraData(comp_img, it->second, out_bag);
                }
            } else if ("lidar" == msg.sensor_name()) {
                pcd.ParseFromString(msg.content());
                auto it = lidar_topic_map.find(pcd.frame_id());
                if (it != lidar_topic_map.end() && pcd.has_measurement_time()) {
                    ROS_INFO("msg_id=%d sensor_name=%s frame_id=%s",
                        i, msg.sensor_name().c_str(), pcd.frame_id().c_str());
                    handleLidarData(pcd, it->second, out_bag);
                }
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
    // header.stamp = ros::Time(static_cast<double>(comp_img.header().camera_timestamp()) * 1e-9);
    header.stamp = ros::Time::now();
    header.frame_id = comp_img.frame_id();
    ROS_INFO("seq=%u", header.seq);

    cv::Mat jpeg_data(1, comp_img.data().size(), CV_8UC1);
    jpeg_data.data = reinterpret_cast<uchar*>(const_cast<char*>(comp_img.data().c_str()));
    cv::InputArray cv_jpeg_data(jpeg_data);
    cv::Mat decoded_image = cv::imdecode(cv_jpeg_data, cv::IMREAD_COLOR);
    cv::Mat resized_image;
    cv::resize(decoded_image, resized_image, cv::Size(960, 540));
    auto image_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, resized_image);
    image_bridge.toImageMsg(image_msg);

    // out_bag.write(out_topic, ros::Time(comp_img.header().timestamp_sec() * 1e-3), image_msg);
    out_bag.write(out_topic, image_msg.header.stamp, image_msg);
}

void Handler::handleLidarData(const drivers::PointCloud &pcd,
        const std::string &out_topic, rosbag::Bag &out_bag) {
    sensor_msgs::PointCloud2 lidar_msg;
    std_msgs::Header header;
    header.seq = pcd.header().sequence_num();
    // header.stamp = ros::Time(pcd.measurement_time());
    header.stamp = ros::Time::now();
    header.frame_id = pcd.frame_id();
    ROS_INFO("seq=%u", header.seq);

    pcl::PointCloud<myPointXYZRID> points;
    points.reserve(pcd.point_size());
    for (auto &&pt : pcd.point()) {
        myPointXYZRID pcl_pt;
        pcl_pt.x = pt.x();
        pcl_pt.y = pt.y();
        pcl_pt.z = pt.z();
        pcl_pt.intensity = pt.intensity();
        pcl_pt.ring = pt.ring();
        points.push_back(std::move(pcl_pt));
    }
    pcl::toROSMsg(points, lidar_msg);
    lidar_msg.header = std::move(header);

    // pcl::io::savePCDFileASCII(std::to_string(header.seq) + ".pcd", points);
    // std::cout << header.seq << " " << std::to_string(pcd.measurement_time()) << std::endl;

    out_bag.write(out_topic, lidar_msg.header.stamp, lidar_msg);
}

