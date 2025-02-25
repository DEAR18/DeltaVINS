#pragma once

#if USE_ROS2
#include <unordered_map>

#include "cv_bridge/cv_bridge.h"
#include "dataSource.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rclcpp/serialization.hpp"

namespace DeltaVins {
class DataSource_ROS2 : public DataSource, public rclcpp::Node {
   public:
    DataSource_ROS2(bool is_bag);
    ~DataSource_ROS2();

    bool HaveThingsTodo() override;

    void DoWhatYouNeedToDo() override;

   private:
    enum class StereoType { LEFT = 0, RIGHT = 1 };

    // void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg,
                       int sensor_id);
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, int sensor_id);
    void StereoCallback(const sensor_msgs::msg::Image::SharedPtr msg,
                        StereoType type, int sensor_id);
    void NavSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                           int sensor_id);

   private:
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
        image_sub_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr>
        imu_sub_;

    // TODO: only support one stereo camera now
    std::pair<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr,
              rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
        stereo_sub_;

    std::unordered_map<std::string, int> topic_map_;

    std::deque<sensor_msgs::msg::Image::SharedPtr> cv_ptr_left_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> cv_ptr_right_;
    std::mutex mutex_stereo_;

    int image_count_;

    bool is_bag_;
    std::shared_ptr<rosbag2_cpp::Reader> reader_;
    std::unordered_map<std::string, std::pair<ROS2SensorType, int>>
        topic_map_bag_;
    std::unordered_map<std::string, StereoType> stereo_map_bag_;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer_;
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> nav_sat_fix_serializer_;
    ImageData::Ptr image_data_cache_;
};
}  // namespace DeltaVins
#endif