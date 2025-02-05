#pragma once

#ifdef USE_ROS2
#include <string>
#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <IO/dataSource/dataSource.h>
#include <unordered_map>
#include <utils/Config.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>
#include <deque>
#include <mutex>


namespace DeltaVins {

class DataSource_ROS2_bag : public DataSource, public rclcpp::Node {
    public:
    DataSource_ROS2_bag(const std::string& bag_file);
    ~DataSource_ROS2_bag();
    
    bool HaveThingsTodo() override;
    void DoWhatYouNeedToDo() override;

    private:
    enum class StereoType { LEFT = 0, RIGHT = 1 };
    std::shared_ptr<rosbag2_cpp::Reader> reader_;
    std::unordered_map<std::string, std::pair<ROS2SensorType,int>> topic_map_;
    std::unordered_map<std::string,StereoType> stereo_map_;

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer_;

    std::pair<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> stereo_sub_;

    std::deque<sensor_msgs::msg::Image::SharedPtr> cv_ptr_left_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> cv_ptr_right_;
    std::mutex mutex_stereo_;

    ImageData::Ptr image_data_cache_;
};

} // namespace DeltaVins
#endif