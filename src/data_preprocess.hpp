/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "bag_loader.hpp"
#include "common_lib.h"

using namespace std;

class DataPreprocess
{
public:
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params, const rclcpp::Node::SharedPtr& node)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        const string bag_path = params.bag_path;
        const string image_path = params.image_path;
        const string lidar_topic = params.lidar_topic;
        const auto logger = node->get_logger();

        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            RCLCPP_ERROR(logger, "Loading the image %s failed", image_path.c_str());
            return;
        }

        std::fstream file_;
        file_.open(bag_path, ios::in);
        if (!file_)
        {
            RCLCPP_ERROR(logger, "Loading the rosbag %s failed", bag_path.c_str());
            return;
        }
        file_.close();

        RCLCPP_INFO(logger, "Loading the rosbag %s", bag_path.c_str());
        if (!loadCloudFromBag(logger, bag_path, lidar_topic, cloud_input_, lidar_type_))
        {
            RCLCPP_ERROR(logger, "LOADING BAG FAILED: %s", bag_path.c_str());
            return;
        }

        RCLCPP_INFO(logger, "Loaded %zu points from the rosbag.", cloud_input_->size());
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif
