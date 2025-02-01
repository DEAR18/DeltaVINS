#if USE_ROS2
#include "IO/dataSource/dataSource_ROS2.h"

#include <cv_bridge/cv_bridge.h>

#include "Algorithm/vision/camModel/camModel.h"
#include "utils/Config.h"

namespace DeltaVins {

DataSource_ROS2::DataSource_ROS2() : rclcpp::Node("data_source_ros2") {
    // Create subscribers
    for (auto& topic : Config::ROS2SensorTopics) {
        if (topic.type == ROS2SensorType::MonoCamera) {
            std::string topic_name = topic.topics[0];
            // Create camera subscriber
            image_sub_.push_back( this->create_subscription<sensor_msgs::msg::Image>(
                topic_name, 10,[this,topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                    ImageCallback(msg,topic.sensor_id);
                }));
            topic_map_[topic_name] = topic.sensor_id;
            LOGI("Create camera subscriber: %s", topic_name.c_str());
        } 
        else if (topic.type == ROS2SensorType::IMU) {
            std::string topic_name = topic.topics[0];
            // Create imu subscriber
            imu_sub_.push_back(this->create_subscription<sensor_msgs::msg::Imu>(
                topic_name, rclcpp::SensorDataQoS(),[this,topic](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    ImuCallback(msg,topic.sensor_id);
                }));
            topic_map_[topic_name] = topic.sensor_id;
            LOGI("Create imu subscriber: %s", topic_name.c_str());
        } else if (topic.type == ROS2SensorType::StereoCamera) {
            std::string topic_name = topic.topics[0];

            // Create stereo subscriber
            stereo_sub_.first =
                this->create_subscription<sensor_msgs::msg::Image>(
                    topic.topics[0], 10,
                    [this,topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                        StereoCallback(msg, StereoType::LEFT, topic.sensor_id);
                    });
            stereo_sub_.second =
                this->create_subscription<sensor_msgs::msg::Image>(
                    topic.topics[1], 10,
                    [this,topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                        StereoCallback(msg, StereoType::RIGHT, topic.sensor_id);
                    });

            topic_map_[topic_name] = topic.sensor_id;
            topic_map_[topic.topics[1]] = topic.sensor_id;
            LOGI("Create stereo subscriber: %s, %s", topic_name.c_str(),
                 topic.topics[1].c_str());
        }
    }

    // std::string topic_name = "/cam0/image_raw";
    // // Create camera subscriber
    // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     topic_name, 10,
    //     std::bind(&DataSource_ROS2::ImageCallback, this,
    //               std::placeholders::_1));
    // image_count_ = 0;
    // // Create camera info subscriber
    // // cameraInfoSubscriber =
    // //
    // // node->create_subscription<sensor_msgs::msg::CameraInfo>(cameraInfoTopicName,
    // // 10, std::bind(&DataSource_ROS2::cameraInfoCallback, this,
    // // std::placeholders::_1));

    // std::string imu_topic_name = "/imu0";
    // // Create imu subscriber
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     imu_topic_name, rclcpp::SensorDataQoS(),
    //     std::bind(&DataSource_ROS2::ImuCallback, this, std::placeholders::_1));

    // // Create odom subscriber
    // // odomSubscriber =
    // // node->create_subscription<nav_msgs::msg::Odometry>(odomTopicName, 10,
    // // std::bind(&DataSource_ROS2::odomCallback, this,
    // std::placeholders::_1));
}

DataSource_ROS2::~DataSource_ROS2() {}

void DataSource_ROS2::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg,int sensor_id) {
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

    image_data->cam_id = sensor_id;

    image_count_++;
    // Debug for Euroc data

    // int width_crop = 0;
    // if (cv_ptr->image.cols > 640) width_crop = (cv_ptr->image.cols - 640) /
    // 2; cv::cvtColor(cv_ptr->image.colRange(width_crop, width_crop + 640),
    // image_data->image, cv::COLOR_BGR2GRAY); image_data->image =
    //     cv_ptr->image.colRange(width_crop, width_crop + 640).clone();
    cv::cvtColor(cv_ptr->image, image_data->image, cv::COLOR_BGR2GRAY);
    {
        std::lock_guard<std::mutex> lck(mtx_image_observer_);
        for (auto& observer : image_observers_) {
            observer->OnImageReceived(image_data);
        }
    }
}

void DataSource_ROS2::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg,int sensor_id) {
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

void DataSource_ROS2::StereoCallback(
    const sensor_msgs::msg::Image::SharedPtr msg, StereoType stereo_type,
    int sensor_id) {
    std::lock_guard<std::mutex> lck(mutex_stereo_);
    if (stereo_type == StereoType::LEFT) {
        cv_ptr_left_.push_back(msg);
    } else {
        cv_ptr_right_.push_back(msg);
    }
    if (cv_ptr_left_.empty() || cv_ptr_right_.empty()) return;

    auto left = cv_ptr_left_.front();
    do {
        auto right = cv_ptr_right_.front();
        // Need to make sure the left and right image have the same timestamp
        if (left->header.stamp == right->header.stamp) {
            cv_ptr_left_.pop_front();
            cv_ptr_right_.pop_front();

            // Convert ROS2 image to cv::Mat
            cv_bridge::CvImageConstPtr cv_ptr_left;
            cv_bridge::CvImageConstPtr cv_ptr_right;
            try {
                cv_ptr_left = cv_bridge::toCvShare(
                    left, sensor_msgs::image_encodings::BGR8);
                cv_ptr_right = cv_bridge::toCvShare(
                    right, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s",
                             e.what());
                return;
            }
            // if(cv_ptr_left->image.channels() != 1){
            //     LOGE("Left image channel is not 1");
            //     return;
            // }
            // Publish image
            ImageData::Ptr image_data = std::make_shared<ImageData>();
            cv::cvtColor(cv_ptr_left->image, image_data->image, cv::COLOR_BGR2GRAY);
            // image_data->image = cv_ptr_left->image;
            image_data->timestamp =
                left->header.stamp.sec * 1e9 + left->header.stamp.nanosec;
            image_data->cam_id = sensor_id;
            cv::cvtColor(cv_ptr_right->image, image_data->right_image, cv::COLOR_BGR2GRAY);
            // image_data->right_image = cv_ptr_right->image;
            {
                std::lock_guard<std::mutex> lck(mtx_image_observer_);
                for (auto& observer : image_observers_) {
                    observer->OnImageReceived(image_data);
                }
            }
        } else {
            auto left_stamp = left->header.stamp.sec * 1000000000 +
                              left->header.stamp.nanosec;
            auto right_stamp = right->header.stamp.sec * 1000000000 +
                               right->header.stamp.nanosec;
            if (left_stamp > right_stamp) {
                cv_ptr_right_.pop_front();
            } else if (left_stamp < right_stamp) {
                cv_ptr_left_.pop_front();
            }
        }
    } while (!cv_ptr_left_.empty() && !cv_ptr_right_.empty());
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

void DataSource_ROS2::DoWhatYouNeedToDo() {}

}  // namespace DeltaVins

#endif