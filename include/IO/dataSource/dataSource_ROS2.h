#pragma once

#if USE_ROS2
#include "dataSource.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cv_bridge/cv_bridge.hpp"


namespace DeltaVins{
    class DataSource_ROS2 : public DataSource, public rclcpp::Node{

        public:
            DataSource_ROS2();
            ~DataSource_ROS2();

            bool HaveThingsTodo() override;

            void DoWhatYouNeedToDo() override;

        private:
            void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
            void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);


        private:
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        int image_count_;
    };
}
#endif