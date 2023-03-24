#ifndef HANDLER_H
#define HANDLER_H

#include "config.h"
#include "message.pb.h"
#include "sensor_image.pb.h"
#include "pointcloud.pb.h"

#include <rosbag/bag.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


struct myPointXYZRID {
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t ring; ///< laser ring number
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    myPointXYZRID,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
)


class Handler {
public:
    void dump(const Config &config, const std::string &in_file,
        const std::string &out_file);
    void handleCameraData(const drivers::CompressedImage &comp_img,
        const std::string &out_topic, rosbag::Bag &out_bag);
    void handleLidarData(const drivers::PointCloud &pcd,
        const std::string &out_topic, rosbag::Bag &out_bag);
};

#endif
