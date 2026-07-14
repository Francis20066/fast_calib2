#ifndef BAG_LOADER_HPP
#define BAG_LOADER_HPP

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <filesystem>
#include <fstream>
#include <unordered_map>

#include "common_lib.h"

inline bool hasPointCloudField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name)
{
    for (const auto& field : msg.fields)
    {
        if (field.name == name) return true;
    }
    return false;
}

inline bool appendLivoxCustomMsg(const livox_ros_driver2::msg::CustomMsg& livox_msg,
                                 pcl::PointCloud<Common::Point>::Ptr cloud,
                                 LiDARType& detected_type)
{
    detected_type = LiDARType::Solid;
    cloud->reserve(cloud->size() + livox_msg.point_num);
    for (uint32_t i = 0; i < livox_msg.point_num; ++i)
    {
        Common::Point p;
        p.x = livox_msg.points[i].x;
        p.y = livox_msg.points[i].y;
        p.z = livox_msg.points[i].z;
        p.intensity = static_cast<float>(livox_msg.points[i].reflectivity);
        p.ring = static_cast<std::uint16_t>(livox_msg.points[i].line);
        cloud->push_back(p);
    }
    return true;
}

inline bool appendPointCloud2(const sensor_msgs::msg::PointCloud2& pcl_msg,
                              pcl::PointCloud<Common::Point>::Ptr cloud,
                              LiDARType& detected_type)
{
    const bool has_ring = hasPointCloudField(pcl_msg, "ring");
    const bool has_intensity = hasPointCloudField(pcl_msg, "intensity");
    const bool has_reflectivity = hasPointCloudField(pcl_msg, "reflectivity");

    if (detected_type == LiDARType::Unknown)
    {
        detected_type = has_ring ? LiDARType::Mech : LiDARType::Solid;
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(pcl_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(pcl_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(pcl_msg, "z");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
    if (has_ring)
    {
        it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(pcl_msg, "ring"));
    }

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity_ptr;
    if (has_intensity)
    {
        it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(pcl_msg, "intensity"));
    }
    else if (has_reflectivity)
    {
        it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(pcl_msg, "reflectivity"));
    }

    const size_t n = static_cast<size_t>(pcl_msg.width) * pcl_msg.height;
    cloud->reserve(cloud->size() + n);
    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
    {
        Common::Point p;
        p.x = *it_x;
        p.y = *it_y;
        p.z = *it_z;
        p.ring = 0xFFFF;
        p.intensity = 0.0f;

        if (it_ring_ptr)
        {
            p.ring = **it_ring_ptr;
            ++(*it_ring_ptr);
        }
        if (it_intensity_ptr)
        {
            p.intensity = **it_intensity_ptr;
            ++(*it_intensity_ptr);
        }

        cloud->push_back(p);
    }
    return true;
}

inline bool loadCloudFromBag(const rclcpp::Logger& logger,
                             const std::string& bag_path,
                             const std::string& lidar_topic,
                             pcl::PointCloud<Common::Point>::Ptr cloud,
                             LiDARType& detected_type)
{
    cloud->clear();
    detected_type = LiDARType::Unknown;

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    if (bag_path.size() >= 5 && bag_path.substr(bag_path.size() - 5) == ".mcap")
    {
        storage_options.storage_id = "mcap";
    }
    else if (std::filesystem::is_directory(bag_path))
    {
        storage_options.storage_id = "sqlite3";
        for (const auto& entry : std::filesystem::directory_iterator(bag_path))
        {
            if (entry.path().extension() == ".mcap")
            {
                storage_options.storage_id = "mcap";
                break;
            }
        }
    }
    else
    {
        storage_options.storage_id = "sqlite3";
    }

    try
    {
        reader.open(storage_options);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger, "Failed to open bag %s: %s", bag_path.c_str(), e.what());
        return false;
    }

    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serializer;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serializer;
    size_t message_count = 0;

    std::unordered_map<std::string, std::string> topic_types;
    for (const auto& info : reader.get_metadata().topics_with_message_count)
    {
        topic_types[info.topic_metadata.name] = info.topic_metadata.type;
    }

    while (reader.has_next())
    {
        const auto bag_message = reader.read_next();
        if (bag_message->topic_name != lidar_topic) continue;

        const auto type_it = topic_types.find(lidar_topic);
        const std::string topic_type =
            type_it != topic_types.end() ? type_it->second : "";

        if (topic_type == "livox_ros_driver2/msg/CustomMsg" ||
            topic_type == "livox_ros_driver/CustomMsg")
        {
            livox_ros_driver2::msg::CustomMsg livox_msg;
            rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
            livox_serializer.deserialize_message(&serialized, &livox_msg);
            appendLivoxCustomMsg(livox_msg, cloud, detected_type);
            ++message_count;
            continue;
        }

        if (topic_type == "sensor_msgs/msg/PointCloud2" ||
            topic_type == "sensor_msgs/PointCloud2")
        {
            sensor_msgs::msg::PointCloud2 pcl_msg;
            rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
            pc2_serializer.deserialize_message(&serialized, &pcl_msg);
            appendPointCloud2(pcl_msg, cloud, detected_type);
            ++message_count;
            continue;
        }

        try
        {
            livox_ros_driver2::msg::CustomMsg livox_msg;
            rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
            livox_serializer.deserialize_message(&serialized, &livox_msg);
            appendLivoxCustomMsg(livox_msg, cloud, detected_type);
            ++message_count;
            continue;
        }
        catch (const std::exception&)
        {
        }

        sensor_msgs::msg::PointCloud2 pcl_msg;
        rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
        pc2_serializer.deserialize_message(&serialized, &pcl_msg);
        appendPointCloud2(pcl_msg, cloud, detected_type);
        ++message_count;
    }

    RCLCPP_INFO(logger, "Loaded %zu messages, %zu points from %s",
                message_count, cloud->size(), bag_path.c_str());
    return message_count > 0 && !cloud->empty();
}

#endif
