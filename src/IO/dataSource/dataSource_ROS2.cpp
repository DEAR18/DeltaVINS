#if USE_ROS2

#include "IO/dataSource/dataSource_ROS2.h"

#include <cv_bridge/cv_bridge.hpp>

#include "Algorithm/vision/camModel/camModel.h"

namespace DeltaVins {

DataSource_ROS2::DataSource_ROS2() : rclcpp::Node("data_source_ros2") {
    std::string topic_name = "/cam0/image_raw";
    // Create camera subscriber
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10,
        std::bind(&DataSource_ROS2::ImageCallback, this,
                  std::placeholders::_1));
    image_count_ = 0;
    // Create camera info subscriber
    // cameraInfoSubscriber =
    // node->create_subscription<sensor_msgs::msg::CameraInfo>(cameraInfoTopicName,
    // 10, std::bind(&DataSource_ROS2::cameraInfoCallback, this,
    // std::placeholders::_1));

    std::string imu_topic_name = "/imu0";
    // Create imu subscriber
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name,rclcpp::QoS(100),
        std::bind(&DataSource_ROS2::ImuCallback, this, std::placeholders::_1));

    // Create odom subscriber
    // odomSubscriber =
    // node->create_subscription<nav_msgs::msg::Odometry>(odomTopicName, 10,
    // std::bind(&DataSource_ROS2::odomCallback, this, std::placeholders::_1));
}

DataSource_ROS2::~DataSource_ROS2() {}

void DataSource_ROS2::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS2 image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Publish image
    ImageData::Ptr image_data = std::make_shared<ImageData>();
    // image_data->image = cv_ptr->image;
    image_data->timestamp =
        msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

    image_count_++;
    // Debug for Euroc data

    int width_crop = 0;
    if (cv_ptr->image.cols > 640) width_crop = (cv_ptr->image.cols - 640) / 2;
    cv::cvtColor(cv_ptr->image.colRange(width_crop, width_crop + 640), image_data->image, cv::COLOR_BGR2GRAY);
    // image_data->image =
    //     cv_ptr->image.colRange(width_crop, width_crop + 640).clone();
    {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        for (auto& observer : image_observers_) {
            observer->OnImageReceived(image_data);
        }
    }
}

void DataSource_ROS2::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    Matrix3f Rci = CamModel::getCamModel()->getRci();
    // Publish IMU data
    ImuData imu_data;
    imu_data.timestamp =
        msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    Eigen::Vector3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    Eigen::Vector3f gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                         msg->angular_velocity.z);

    // Rotate the acceleration and angular velocity to the camera frame
    imu_data.acc = Rci * acc;
    imu_data.gyro = Rci * gyro;

    {
        std::lock_guard<std::mutex> lck(mtx_imu_observer_);
        for (auto& observer : imu_observers_) {
            observer->OnImuReceived(imu_data);
        }
    }
}

// // void DataSource_ROS2::Join() {
// //     if (modules_thread_->joinable()) modules_thread_->join();
// //     run_ = false;
// }

bool DataSource_ROS2::HaveThingsTodo() {
    if (!rclcpp::ok()) {
        keep_running_ = false;
        return false;
    }
    return true;
}

void DataSource_ROS2::DoWhatYouNeedToDo() {

}

}  // namespace DeltaVins

#endif